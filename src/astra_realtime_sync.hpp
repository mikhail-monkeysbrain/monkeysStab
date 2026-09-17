#pragma once

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

#include "astra_reference_pipeline.hpp"

// Realtime adapter for the literal offline Astra timeline semantics.
// A camera frame is not released to the reference pipeline until an attitude
// sample exists on BOTH sides of the camera timestamp. This reproduces linear
// interpolation without turning a not-yet-arrived future ATTITUDE sample into
// a rejected visual interval.
namespace astra_realtime_sync {

struct AttitudeSample {
  int64_t t_ns=0;
  double roll=0.0;
  double pitch=0.0;
  double yaw=0.0;
  bool valid=false;
};

struct RangeSample {
  int64_t t_ns=0;
  double range_m=0.0;
  bool valid=false;
};

struct PendingFrame {
  int64_t frame_id=-1;
  int64_t camera_ts_ns=0;
  cv::Mat gray;
};

inline double wrapDelta(double d){
  while(d>M_PI) d-=2.0*M_PI;
  while(d<-M_PI) d+=2.0*M_PI;
  return d;
}

inline double lerpAngle(double a,double b,double f){
  return a+f*wrapDelta(b-a);
}

inline bool bracketAttitude(const std::deque<AttitudeSample>& h,int64_t t,
                            AttitudeSample& a,AttitudeSample& b){
  if(h.size()<2) return false;
  for(size_t i=1;i<h.size();++i){
    if(!h[i-1].valid || !h[i].valid) continue;
    if(h[i-1].t_ns<=t && t<=h[i].t_ns && h[i].t_ns>h[i-1].t_ns){
      a=h[i-1]; b=h[i]; return true;
    }
  }
  return false;
}

inline bool interpolateAttitude(const std::deque<AttitudeSample>& h,int64_t t,
                                astra_metric::M3& body_to_ned,
                                double* bracket_gap_ms=nullptr){
  AttitudeSample a,b;
  if(!bracketAttitude(h,t,a,b)) return false;
  const double f=double(t-a.t_ns)/double(b.t_ns-a.t_ns);
  const double r=a.roll+f*(b.roll-a.roll);
  const double p=a.pitch+f*(b.pitch-a.pitch);
  const double y=lerpAngle(a.yaw,b.yaw,f);
  body_to_ned=astra_metric::rotation(r,p,y);
  if(bracket_gap_ms) *bracket_gap_ms=double(b.t_ns-a.t_ns)*1e-6;
  return true;
}

// Original offline Astra uses np.interp for TF-Luna. For realtime, use the
// latest range sample at or before the frame time when the future sample is not
// yet available; when a bracket exists, linearly interpolate exactly.
inline bool interpolateRange(const std::deque<RangeSample>& h,int64_t t,double& out,
                             double* bracket_gap_ms=nullptr){
  if(h.empty()) return false;
  const RangeSample* prev=nullptr;
  for(const auto& s:h){
    if(!s.valid) continue;
    if(s.t_ns<=t) prev=&s;
    if(s.t_ns>=t){
      if(prev && s.t_ns>prev->t_ns){
        const double f=double(t-prev->t_ns)/double(s.t_ns-prev->t_ns);
        out=prev->range_m+f*(s.range_m-prev->range_m);
        if(bracket_gap_ms) *bracket_gap_ms=double(s.t_ns-prev->t_ns)*1e-6;
        return std::isfinite(out) && out>0.0;
      }
      if(s.t_ns==t){ out=s.range_m; if(bracket_gap_ms)*bracket_gap_ms=0.0; return std::isfinite(out)&&out>0.0; }
      break;
    }
  }
  if(prev){ out=prev->range_m; if(bracket_gap_ms)*bracket_gap_ms=-1.0; return std::isfinite(out)&&out>0.0; }
  return false;
}

class PendingSynchronizer {
 public:
  void clear(){ pending_.clear(); }
  void pushFrame(int64_t id,int64_t ts,const cv::Mat& gray){ pending_.push_back({id,ts,gray.clone()}); }
  size_t pending() const{return pending_.size();}

  bool popReady(const std::deque<AttitudeSample>& ah,
                const std::deque<RangeSample>& rh,
                astra_reference::FrameInput& out,
                double* att_gap_ms=nullptr,
                double* range_gap_ms=nullptr){
    if(pending_.empty()) return false;
    const auto& p=pending_.front();
    astra_metric::M3 R;
    if(!interpolateAttitude(ah,p.camera_ts_ns,R,att_gap_ms)) return false;
    double range=0.0;
    if(!interpolateRange(rh,p.camera_ts_ns,range,range_gap_ms)) return false;
    out.frame_id=p.frame_id;
    out.camera_ts_ns=p.camera_ts_ns;
    out.gray=p.gray;
    out.sensor.body_to_ned=R;
    out.sensor.attitude_valid=true;
    out.sensor.range_m=range;
    out.sensor.range_valid=true;
    pending_.pop_front();
    return true;
  }

 private:
  std::deque<PendingFrame> pending_;
};

} // namespace astra_realtime_sync
