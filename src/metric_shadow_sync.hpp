#pragma once

#include "metric_odometry_shadow.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>

namespace metric_shadow {

struct TimedAttitude {
  double roll=0.0,pitch=0.0,yaw=0.0;
  int64_t sample_ns=0;
  bool valid=false;
};

struct AttitudeLookup {
  Attitude attitude{};
  bool valid=false;
  double bracket_gap_ms=-1.0;
  double nearest_age_ms=-1.0;
};

inline double wrapPi(double x){
  constexpr double pi=3.14159265358979323846;
  while(x>pi) x-=2.0*pi;
  while(x<-pi) x+=2.0*pi;
  return x;
}

inline double interpAngle(double a,double b,double u){
  return wrapPi(a+u*wrapPi(b-a));
}

// Interpolate only when the requested camera timestamp is actually bracketed
// by two attitude samples.  No nearest-sample extrapolation is allowed: a
// missing temporal link is a measurement gap, not zero angular motion.
inline AttitudeLookup interpolateAttitude(const std::deque<TimedAttitude>& h,
                                          int64_t t_ns,
                                          double max_bracket_gap_ms=30.0){
  AttitudeLookup out;
  if(h.size()<2 || t_ns<=0) return out;

  auto hi=std::lower_bound(h.begin(),h.end(),t_ns,
    [](const TimedAttitude& a,int64_t t){ return a.sample_ns<t; });
  if(hi==h.end()) return out;
  if(hi->sample_ns==t_ns && hi->valid){
    out.attitude={hi->roll,hi->pitch,hi->yaw,hi->sample_ns,true};
    out.valid=true;
    out.bracket_gap_ms=0.0;
    out.nearest_age_ms=0.0;
    return out;
  }
  if(hi==h.begin()) return out;
  const auto lo=std::prev(hi);
  if(!lo->valid || !hi->valid || hi->sample_ns<=lo->sample_ns) return out;

  const double gap_ms=(hi->sample_ns-lo->sample_ns)*1e-6;
  const double nearest_ms=std::min(t_ns-lo->sample_ns,hi->sample_ns-t_ns)*1e-6;
  out.bracket_gap_ms=gap_ms;
  out.nearest_age_ms=nearest_ms;
  if(!(gap_ms>=0.0 && gap_ms<=max_bracket_gap_ms)) return out;

  const double u=(double)(t_ns-lo->sample_ns)/(double)(hi->sample_ns-lo->sample_ns);
  if(!(u>=0.0 && u<=1.0)) return out;
  out.attitude.roll=interpAngle(lo->roll,hi->roll,u);
  out.attitude.pitch=interpAngle(lo->pitch,hi->pitch,u);
  out.attitude.yaw=interpAngle(lo->yaw,hi->yaw,u);
  out.attitude.sample_ns=t_ns;
  out.attitude.valid=true;
  out.valid=true;
  return out;
}


struct TimedBodyRate {
  double x=0.0,y=0.0,z=0.0; // body FRD rad/s
  int64_t sample_ns=0;
  bool valid=false;
};

struct BodyRateIntegration {
  cv::Matx33d delta_R=cv::Matx33d::eye(); // body(t0) -> body(t1), right-multiplied into body-to-local R
  bool valid=false;
  double max_bracket_gap_ms=-1.0;
  double integrated_angle_deg=0.0;
  int segments=0;
};

inline cv::Matx33d expSO3(const cv::Vec3d& rv){
  const double th=cv::norm(rv);
  if(!(th>1e-12) || !std::isfinite(th)) return cv::Matx33d::eye();
  const cv::Vec3d a=rv*(1.0/th);
  const double x=a[0],y=a[1],z=a[2];
  const double c=std::cos(th),s=std::sin(th),v=1.0-c;
  return cv::Matx33d(
    c+x*x*v,   x*y*v-z*s, x*z*v+y*s,
    y*x*v+z*s, c+y*y*v,   y*z*v-x*s,
    z*x*v-y*s, z*y*v+x*s, c+z*z*v);
}

inline bool interpolateBodyRate(const std::deque<TimedBodyRate>& h,
                                int64_t t_ns,
                                TimedBodyRate* out,
                                double* bracket_gap_ms,
                                double max_bracket_gap_ms){
  if(!out || h.size()<2 || t_ns<=0) return false;
  auto hi=std::lower_bound(h.begin(),h.end(),t_ns,
    [](const TimedBodyRate& a,int64_t t){ return a.sample_ns<t; });
  if(hi==h.end()) return false;
  if(hi->sample_ns==t_ns && hi->valid){
    *out=*hi;
    if(bracket_gap_ms) *bracket_gap_ms=0.0;
    return true;
  }
  if(hi==h.begin()) return false;
  const auto lo=std::prev(hi);
  if(!lo->valid || !hi->valid || hi->sample_ns<=lo->sample_ns) return false;
  const double gap=(hi->sample_ns-lo->sample_ns)*1e-6;
  if(bracket_gap_ms) *bracket_gap_ms=gap;
  if(!(gap>=0.0 && gap<=max_bracket_gap_ms)) return false;
  const double u=(double)(t_ns-lo->sample_ns)/(double)(hi->sample_ns-lo->sample_ns);
  if(!(u>=0.0 && u<=1.0)) return false;
  out->x=lo->x+u*(hi->x-lo->x);
  out->y=lo->y+u*(hi->y-lo->y);
  out->z=lo->z+u*(hi->z-lo->z);
  out->sample_ns=t_ns;
  out->valid=true;
  return true;
}

// Integrate FC body rates exactly over the camera interval using piecewise
// trapezoidal angular velocity and SO(3) composition. Timestamps are still RPi
// MAVLink receive times in this shadow; the purpose is to test whether rate
// integration is less sensitive to ATTITUDE phase/latency than endpoint Euler
// interpolation, not to hide the remaining clock-mapping limitation.
inline BodyRateIntegration integrateBodyRates(const std::deque<TimedBodyRate>& h,
                                              int64_t t0_ns,int64_t t1_ns,
                                              double max_bracket_gap_ms=30.0){
  BodyRateIntegration out;
  if(h.size()<2 || t0_ns<=0 || t1_ns<=t0_ns) return out;

  TimedBodyRate r0{},r1{};
  double g0=-1.0,g1=-1.0;
  if(!interpolateBodyRate(h,t0_ns,&r0,&g0,max_bracket_gap_ms) ||
     !interpolateBodyRate(h,t1_ns,&r1,&g1,max_bracket_gap_ms))
    return out;

  std::vector<TimedBodyRate> p;
  p.push_back(r0);
  for(const auto& q:h)
    if(q.valid && q.sample_ns>t0_ns && q.sample_ns<t1_ns) p.push_back(q);
  p.push_back(r1);
  std::sort(p.begin(),p.end(),
    [](const TimedBodyRate& a,const TimedBodyRate& b){return a.sample_ns<b.sample_ns;});

  cv::Matx33d dR=cv::Matx33d::eye();
  double angle_sum=0.0;
  int segs=0;
  for(size_t k=1;k<p.size();++k){
    const double dt=(p[k].sample_ns-p[k-1].sample_ns)*1e-9;
    if(!(dt>0.0 && dt<0.1)) return out;
    const cv::Vec3d w(
      0.5*(p[k-1].x+p[k].x),
      0.5*(p[k-1].y+p[k].y),
      0.5*(p[k-1].z+p[k].z));
    const cv::Vec3d rv=w*dt;
    dR=dR*expSO3(rv);
    angle_sum+=cv::norm(rv);
    ++segs;
  }

  out.delta_R=dR;
  out.valid=segs>0;
  out.max_bracket_gap_ms=std::max(g0,g1);
  out.integrated_angle_deg=angle_sum*180.0/3.14159265358979323846;
  out.segments=segs;
  return out;
}


// Causal HIGHRES integration for realtime camera intervals. Unlike
// integrateBodyRates(), this never requires a sample newer than t1. The rate
// known at each instant is held until the next already-available sample. The
// first held sample must be no older than max_hold_ms at t0; the final held
// sample must be no older than max_hold_ms at t1. This keeps the extrapolation
// explicitly bounded instead of silently treating missing IMU coverage as zero.
inline BodyRateIntegration integrateBodyRatesCausalHold(
    const std::deque<TimedBodyRate>& h,
    int64_t t0_ns,int64_t t1_ns,
    double max_hold_ms=15.0){
  BodyRateIntegration out;
  if(h.empty() || t0_ns<=0 || t1_ns<=t0_ns) return out;

  auto first_after_t0=std::upper_bound(h.begin(),h.end(),t0_ns,
    [](int64_t t,const TimedBodyRate& a){ return t<a.sample_ns; });
  if(first_after_t0==h.begin()) return out;
  auto cur=std::prev(first_after_t0);
  if(!cur->valid || cur->sample_ns<=0) return out;

  const double start_hold_ms=(t0_ns-cur->sample_ns)*1e-6;
  if(!(start_hold_ms>=0.0 && start_hold_ms<=max_hold_ms)) return out;

  cv::Matx33d dR=cv::Matx33d::eye();
  double angle_sum=0.0;
  double max_seen_hold_ms=start_hold_ms;
  int segs=0;
  int64_t seg_start=t0_ns;
  TimedBodyRate rate=*cur;

  for(auto it=first_after_t0; it!=h.end() && it->sample_ns<t1_ns; ++it){
    if(!it->valid || it->sample_ns<=seg_start) continue;
    const double dt=(it->sample_ns-seg_start)*1e-9;
    if(!(dt>0.0 && dt<0.1)) return out;
    const cv::Vec3d rv(rate.x*dt,rate.y*dt,rate.z*dt);
    dR=dR*expSO3(rv);
    angle_sum+=cv::norm(rv);
    ++segs;
    rate=*it;
    seg_start=it->sample_ns;
  }

  const double end_hold_ms=(t1_ns-rate.sample_ns)*1e-6;
  max_seen_hold_ms=std::max(max_seen_hold_ms,end_hold_ms);
  if(!(end_hold_ms>=0.0 && end_hold_ms<=max_hold_ms)) return out;

  const double tail_dt=(t1_ns-seg_start)*1e-9;
  if(tail_dt>0.0){
    if(!(tail_dt<0.1)) return out;
    const cv::Vec3d rv(rate.x*tail_dt,rate.y*tail_dt,rate.z*tail_dt);
    dR=dR*expSO3(rv);
    angle_sum+=cv::norm(rv);
    ++segs;
  }

  out.delta_R=dR;
  out.valid=segs>0;
  // For this causal variant the field records maximum endpoint hold age,
  // not a two-sided bracket gap.
  out.max_bracket_gap_ms=max_seen_hold_ms;
  out.integrated_angle_deg=angle_sum*180.0/3.14159265358979323846;
  out.segments=segs;
  return out;
}

inline double rotationDistanceDeg(const cv::Matx33d& a,const cv::Matx33d& b){
  const cv::Matx33d d=a.t()*b;
  double c=(d(0,0)+d(1,1)+d(2,2)-1.0)*0.5;
  c=std::max(-1.0,std::min(1.0,c));
  return std::acos(c)*180.0/3.14159265358979323846;
}

} // namespace metric_shadow
