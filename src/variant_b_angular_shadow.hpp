#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <cmath>
#include <vector>

// VARIANT_B_ANGULAR_SHADOW_V1
//
// Diagnostic-only angular optical-flow estimator.
// It uses the already accepted adjacent-frame correspondences and the same
// causal R0/R1 rotations as Variant B, but deliberately does NOT consume
// range0/range1. Current-frame rays are rotated back into the previous body
// attitude before a 4-parameter similarity fit. The fit estimates normalized
// image translation jointly with isotropic scale (vertical motion) and residual
// in-plane rotation.
//
// This file must remain independent of WORKED5 and must never publish MAVLink.
namespace variant_b_angular_shadow {

struct Step {
  bool valid=false;
  int points=0;
  double du_norm=0.0;
  double dv_norm=0.0;
  double scale_rate=0.0;
  double residual_rotation_rate=0.0;
  double flow_x=0.0;
  double flow_y=0.0;
  double rms_norm=0.0;
};

inline Step estimate(const std::vector<cv::Point2f>& px0,
                     const std::vector<cv::Point2f>& px1,
                     const cv::Mat& K,
                     const cv::Mat& D,
                     const cv::Matx33d& body_R_camera_frd,
                     const cv::Matx33d& R0,
                     const cv::Matx33d& R1,
                     double dt_s) {
  Step o;
  if(px0.size()!=px1.size() || px0.size()<20 || K.empty() ||
     !(dt_s>0.0 && dt_s<0.2)) return o;

  std::vector<cv::Point2f> q0,q1;
  cv::undistortPoints(px0,q0,K,D);
  cv::undistortPoints(px1,q1,K,D);
  if(q0.size()!=q1.size() || q0.size()<20) return o;

  // Transform current camera rays into the orientation of camera frame 0.
  // No range or metric reconstruction is involved.
  const cv::Matx33d camera_R_body=body_R_camera_frd.t();
  std::vector<cv::Point2d> a,b;
  a.reserve(q0.size());
  b.reserve(q0.size());
  for(size_t i=0;i<q0.size();++i){
    const cv::Vec3d c0(q0[i].x,q0[i].y,1.0);
    const cv::Vec3d c1(q1[i].x,q1[i].y,1.0);
    const cv::Vec3d c1_at_0 =
      camera_R_body * (R0.t() * (R1 * (body_R_camera_frd * c1)));
    if(!std::isfinite(c1_at_0[0]) || !std::isfinite(c1_at_0[1]) ||
       !std::isfinite(c1_at_0[2]) || std::abs(c1_at_0[2])<1e-6) continue;
    a.emplace_back(c0[0]/c0[2],c0[1]/c0[2]);
    b.emplace_back(c1_at_0[0]/c1_at_0[2],c1_at_0[1]/c1_at_0[2]);
  }
  if(a.size()<20) return o;

  // b-a = [du,dv] + isotropic_scale*[x,y] + residual_rotation*[-y,x].
  // Solving scale together with translation prevents true vertical motion
  // (image expansion/contraction) from being forced into horizontal flow.
  cv::Mat A((int)a.size()*2,4,CV_64F);
  cv::Mat rhs((int)a.size()*2,1,CV_64F);
  for(size_t i=0;i<a.size();++i){
    const double x=a[i].x,y=a[i].y;
    const double du=b[i].x-a[i].x;
    const double dv=b[i].y-a[i].y;
    const int r=(int)(2*i);
    A.at<double>(r,0)=1.0; A.at<double>(r,1)=0.0;
    A.at<double>(r,2)=x;   A.at<double>(r,3)=-y;
    rhs.at<double>(r,0)=du;
    A.at<double>(r+1,0)=0.0; A.at<double>(r+1,1)=1.0;
    A.at<double>(r+1,2)=y;   A.at<double>(r+1,3)=x;
    rhs.at<double>(r+1,0)=dv;
  }

  cv::Mat sol;
  if(!cv::solve(A,rhs,sol,cv::DECOMP_SVD) || sol.rows!=4) return o;

  const double du=sol.at<double>(0,0);
  const double dv=sol.at<double>(1,0);
  const double sc=sol.at<double>(2,0);
  const double rr=sol.at<double>(3,0);

  double sse=0.0;
  for(size_t i=0;i<a.size();++i){
    const double pu=du+sc*a[i].x-rr*a[i].y;
    const double pv=dv+sc*a[i].y+rr*a[i].x;
    const double eu=(b[i].x-a[i].x)-pu;
    const double ev=(b[i].y-a[i].y)-pv;
    sse+=eu*eu+ev*ev;
  }

  o.points=(int)a.size();
  o.du_norm=du;
  o.dv_norm=dv;
  o.scale_rate=sc/dt_s;
  o.residual_rotation_rate=rr/dt_s;
  // Match the existing Variant-B/AP body-flow sign convention for the
  // near-level downward camera: flow_x=du/dt, flow_y=dv/dt.
  o.flow_x=du/dt_s;
  o.flow_y=dv/dt_s;
  o.rms_norm=std::sqrt(sse/(2.0*a.size()));
  o.valid=std::isfinite(o.flow_x) && std::isfinite(o.flow_y) &&
          std::isfinite(o.scale_rate) &&
          std::isfinite(o.residual_rotation_rate) &&
          std::isfinite(o.rms_norm);
  return o;
}

} // namespace variant_b_angular_shadow
