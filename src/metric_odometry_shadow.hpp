#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace metric_shadow {

enum class RejectReason {
  NONE=0,
  BAD_DT,
  BAD_CORRESPONDENCES,
  BAD_ATTITUDE,
  BAD_RANGE,
  BAD_EXTRINSICS,
  DEGENERATE_PLANE,
  ROBUST_FIT
};

inline const char* rejectReasonName(RejectReason r){
  switch(r){
    case RejectReason::NONE:return "OK";
    case RejectReason::BAD_DT:return "BAD_DT";
    case RejectReason::BAD_CORRESPONDENCES:return "BAD_CORRESPONDENCES";
    case RejectReason::BAD_ATTITUDE:return "BAD_ATTITUDE";
    case RejectReason::BAD_RANGE:return "BAD_RANGE";
    case RejectReason::BAD_EXTRINSICS:return "BAD_EXTRINSICS";
    case RejectReason::DEGENERATE_PLANE:return "DEGENERATE_PLANE";
    case RejectReason::ROBUST_FIT:return "ROBUST_FIT";
  }
  return "UNKNOWN";
}

struct Attitude {
  double roll=0.0,pitch=0.0,yaw=0.0;
  int64_t sample_ns=0;
  bool valid=false;
};

struct Input {
  int64_t t0_ns=0,t1_ns=0;
  std::vector<cv::Point2f> px0,px1;
  cv::Mat K,D;
  Attitude a0,a1;
  double range0_m=0.0;
  double range1_m=0.0;
  bool range0_valid=false;
  bool range1_valid=false;
  cv::Matx33d body_R_camera_frd=cv::Matx33d::eye();
  bool body_R_camera_valid=false;
  cv::Vec3d camera_pos_body_frd{0.0625,0.0,0.050};
  cv::Vec3d range_pos_body_frd{0.0855,0.0,0.055};
  cv::Vec3d range_ray_body_frd{0.0,0.0,1.0};
};

struct Step {
  bool valid=false;
  RejectReason reason=RejectReason::NONE;
  double dt=0.0;
  cv::Vec3d delta_local_m{0,0,0};
  cv::Vec3d velocity_local_mps{0,0,0};
  int points=0;
  double residual_median_m=0.0;
  double residual_mad_m=0.0;
};

inline cv::Matx33d bodyToLocal(double roll,double pitch,double yaw){
  const double cr=std::cos(roll),sr=std::sin(roll);
  const double cp=std::cos(pitch),sp=std::sin(pitch);
  const double cy=std::cos(yaw),sy=std::sin(yaw);
  return cv::Matx33d(
    cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
    sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
    -sp,   cp*sr,          cp*cr);
}

inline double median(std::vector<double> v){
  if(v.empty()) return 0.0;
  const size_t n=v.size()/2;
  std::nth_element(v.begin(),v.begin()+n,v.end());
  double m=v[n];
  if((v.size()&1)==0){
    std::nth_element(v.begin(),v.begin()+n-1,v.end());
    m=0.5*(m+v[n-1]);
  }
  return m;
}

inline Step estimate(const Input& in){
  Step out;
  out.dt=(in.t1_ns-in.t0_ns)*1e-9;
  if(!(out.dt>0.0 && out.dt<0.2)){
    out.reason=RejectReason::BAD_DT; return out;
  }
  if(in.px0.size()!=in.px1.size() || in.px0.size()<20 || in.K.empty()){
    out.reason=RejectReason::BAD_CORRESPONDENCES; return out;
  }
  if(!in.a0.valid || !in.a1.valid){
    out.reason=RejectReason::BAD_ATTITUDE; return out;
  }
  if(!in.range0_valid || !in.range1_valid || !(in.range0_m>0.05) || !(in.range1_m>0.05)){
    out.reason=RejectReason::BAD_RANGE; return out;
  }
  if(!in.body_R_camera_valid){
    out.reason=RejectReason::BAD_EXTRINSICS; return out;
  }

  std::vector<cv::Point2f> q0,q1;
  cv::undistortPoints(in.px0,q0,in.K,in.D);
  cv::undistortPoints(in.px1,q1,in.K,in.D);

  const cv::Matx33d R0=bodyToLocal(in.a0.roll,in.a0.pitch,in.a0.yaw);
  const cv::Matx33d R1=bodyToLocal(in.a1.roll,in.a1.pitch,in.a1.yaw);
  const cv::Vec3d down(0,0,1);
  cv::Vec3d lidar_ray_body=in.range_ray_body_frd;
  const double lrnorm=cv::norm(lidar_ray_body);
  if(!(lrnorm>0.5) || !std::isfinite(lrnorm)){
    out.reason=RejectReason::BAD_EXTRINSICS; return out;
  }
  lidar_ray_body*=1.0/lrnorm;

  const double h0=down.dot(R0*(in.range_pos_body_frd-in.camera_pos_body_frd + in.range0_m*lidar_ray_body));
  const double h1=down.dot(R1*(in.range_pos_body_frd-in.camera_pos_body_frd + in.range1_m*lidar_ray_body));
  if(!(h0>0.03 && h1>0.03) || !std::isfinite(h0) || !std::isfinite(h1)){
    out.reason=RejectReason::BAD_RANGE; return out;
  }

  std::vector<cv::Vec3d> deltas;
  deltas.reserve(q0.size());
  for(size_t i=0;i<q0.size();++i){
    const cv::Vec3d c0(q0[i].x,q0[i].y,1.0), c1(q1[i].x,q1[i].y,1.0);
    const cv::Vec3d r0=R0*(in.body_R_camera_frd*c0);
    const cv::Vec3d r1=R1*(in.body_R_camera_frd*c1);
    const double z0=down.dot(r0), z1=down.dot(r1);
    if(z0<=0.08 || z1<=0.08) continue;
    const cv::Vec3d X0=(h0/z0)*r0;
    const cv::Vec3d X1=(h1/z1)*r1;
    // Same stationary world point: pC1-pC0 = X0-X1.
    deltas.push_back(X0-X1);
  }
  if(deltas.size()<20){ out.reason=RejectReason::DEGENERATE_PLANE; return out; }

  cv::Vec3d center;
  for(int axis=0;axis<3;++axis){
    std::vector<double> v; v.reserve(deltas.size());
    for(const auto& d:deltas) v.push_back(d[axis]);
    center[axis]=median(std::move(v));
  }

  // Robustly refine the common translation. The scale is data-derived from
  // MAD; no stationary deadband or surface-specific threshold is introduced.
  std::vector<double> rr; rr.reserve(deltas.size());
  for(const auto& d:deltas) rr.push_back(cv::norm(d-center));
  const double rmed=median(rr);
  std::vector<double> ad; ad.reserve(rr.size());
  for(double r:rr) ad.push_back(std::abs(r-rmed));
  const double mad=median(ad);
  const double scale=std::max(1e-5,1.4826*mad);
  cv::Vec3d sum(0,0,0); double wsum=0.0;
  for(size_t i=0;i<deltas.size();++i){
    const double e=cv::norm(deltas[i]-center);
    const double w=(e<=2.5*scale || e<=1e-12)?1.0:(2.5*scale/e);
    sum += w*deltas[i]; wsum += w;
  }
  if(!(wsum>0.0)){ out.reason=RejectReason::ROBUST_FIT; return out; }
  const cv::Vec3d delta_cam=sum*(1.0/wsum);

  // Convert camera-centre displacement to FC/IMU displacement. Camera position
  // in local coordinates changes under body rotation even if the IMU does not.
  const cv::Vec3d lever=(R1-R0)*in.camera_pos_body_frd;
  const cv::Vec3d delta_imu=delta_cam-lever;
  if(!std::isfinite(delta_imu[0]) || !std::isfinite(delta_imu[1])){
    out.reason=RejectReason::ROBUST_FIT; return out;
  }

  out.valid=true;
  out.reason=RejectReason::NONE;
  out.points=(int)deltas.size();
  out.delta_local_m=delta_imu;
  out.velocity_local_mps=delta_imu*(1.0/out.dt);
  out.residual_median_m=rmed;
  out.residual_mad_m=mad;
  return out;
}

struct Integrator {
  cv::Vec3d position_m{0,0,0};
  uint64_t accepted=0,rejected=0;
  double accepted_time_s=0.0,gap_time_s=0.0;
  bool complete=true;

  void reset(){ *this=Integrator{}; }
  void consume(const Step& s){
    if(s.valid){
      position_m+=s.delta_local_m;
      accepted_time_s+=std::max(0.0,s.dt);
      ++accepted;
    }else{
      gap_time_s+=std::max(0.0,s.dt);
      complete=false;
      ++rejected;
    }
  }
};

} // namespace metric_shadow
