#include "../src/metric_odometry_shadow.hpp"

#include <opencv2/core.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

cv::Point2f project(const cv::Vec3d& world_point,
                    const cv::Vec3d& imu_pos_local,
                    const cv::Matx33d& local_R_body,
                    const cv::Matx33d& body_R_camera,
                    const cv::Vec3d& camera_pos_body,
                    const cv::Mat& K){
  const cv::Vec3d camera_pos_local=imu_pos_local+local_R_body*camera_pos_body;
  const cv::Vec3d ray_local=world_point-camera_pos_local;
  const cv::Vec3d ray_body=local_R_body.t()*ray_local;
  const cv::Vec3d ray_cam=body_R_camera.t()*ray_body;
  const double x=ray_cam[0]/ray_cam[2];
  const double y=ray_cam[1]/ray_cam[2];
  return cv::Point2f(
    (float)(K.at<double>(0,0)*x+K.at<double>(0,2)),
    (float)(K.at<double>(1,1)*y+K.at<double>(1,2)));
}

metric_shadow::Input makeCase(const cv::Vec3d& p0,const cv::Vec3d& p1,
                              double r0,double pch0,double y0,
                              double r1,double pch1,double y1){
  metric_shadow::Input in;
  in.t0_ns=1000000000LL;
  in.t1_ns=1020000000LL;
  in.K=(cv::Mat_<double>(3,3)<<568.53170752165227,0,315.98271077441063,
                                0,569.68005562865858,239.88148589100641,
                                0,0,1);
  in.D=cv::Mat::zeros(1,5,CV_64F);
  in.a0={r0,pch0,y0,in.t0_ns,true};
  in.a1={r1,pch1,y1,in.t1_ns,true};
  in.camera_pos_body_frd=cv::Vec3d(0.0625,0.0,0.050);
  in.range_pos_body_frd=cv::Vec3d(0.0855,0.0,0.055);
  // Synthetic camera looks body-down: camera +Z -> body +Z, camera +X -> body +Y,
  // camera +Y -> body -X. This is a proper rotation (det=+1).
  in.body_R_camera_frd=cv::Matx33d(0,-1,0, 1,0,0, 0,0,1);
  in.body_R_camera_valid=true;
  in.range_ray_body_frd=cv::Vec3d(0,0,1);

  const auto R0=metric_shadow::bodyToLocal(r0,pch0,y0);
  const auto R1=metric_shadow::bodyToLocal(r1,pch1,y1);
  const double plane_z=0.30; // local NED: ground plane below IMU
  // The synthetic range is chosen so the lidar ray intersects the same plane.
  const cv::Vec3d l0=p0+R0*in.range_pos_body_frd;
  const cv::Vec3d l1=p1+R1*in.range_pos_body_frd;
  const cv::Vec3d d0=R0*in.range_ray_body_frd;
  const cv::Vec3d d1=R1*in.range_ray_body_frd;
  in.range0_m=(plane_z-l0[2])/d0[2];
  in.range1_m=(plane_z-l1[2])/d1[2];
  in.range0_valid=in.range0_m>0.05;
  in.range1_valid=in.range1_m>0.05;

  for(int iy=-3;iy<=3;++iy){
    for(int ix=-4;ix<=4;++ix){
      const cv::Vec3d X(0.04*ix,0.04*iy,plane_z);
      const auto a=project(X,p0,R0,in.body_R_camera_frd,in.camera_pos_body_frd,in.K);
      const auto b=project(X,p1,R1,in.body_R_camera_frd,in.camera_pos_body_frd,in.K);
      if(a.x>20&&a.x<620&&a.y>20&&a.y<460&&b.x>20&&b.x<620&&b.y>20&&b.y<460){
        in.px0.push_back(a); in.px1.push_back(b);
      }
    }
  }
  return in;
}

bool run(const char* name,const metric_shadow::Input& in,const cv::Vec3d& expected,double tol_m){
  const auto s=metric_shadow::estimate(in);
  const double err=s.valid?cv::norm(s.delta_local_m-expected):1e9;
  std::cout<<std::left<<std::setw(24)<<name
           <<" valid="<<s.valid
           <<" reason="<<metric_shadow::rejectReasonName(s.reason)
           <<" points="<<s.points
           <<" d=["<<s.delta_local_m[0]<<","<<s.delta_local_m[1]<<","<<s.delta_local_m[2]<<"]"
           <<" expected=["<<expected[0]<<","<<expected[1]<<","<<expected[2]<<"]"
           <<" err_mm="<<err*1000.0<<"\n";
  return s.valid && err<=tol_m;
}

} // namespace

int main(){
  constexpr double d2r=M_PI/180.0;
  bool ok=true;
  ok &= run("forward +N 40mm",makeCase({0,0,0},{0.040,0,0},0,0,0,0,0,0),{0.040,0,0},0.0005);
  ok &= run("right +E 30mm",makeCase({0,0,0},{0,0.030,0},0,0,0,0,0,0),{0,0.030,0},0.0005);
  ok &= run("return -N 40mm",makeCase({0.040,0,0},{0,0,0},0,0,0,0,0,0),{-0.040,0,0},0.0005);
  ok &= run("pure yaw 12deg",makeCase({0,0,0},{0,0,0},0,0,0,0,0,12*d2r),{0,0,0},0.0008);
  ok &= run("roll/pitch only",makeCase({0,0,0},{0,0,0},2*d2r,-3*d2r,0,-4*d2r,5*d2r,0),{0,0,0},0.0010);
  ok &= run("move + attitude",makeCase({0,0,0},{0.025,-0.018,0},1*d2r,-2*d2r,5*d2r,-3*d2r,4*d2r,13*d2r),{0.025,-0.018,0},0.0010);
  std::cout<<(ok?"PASS":"FAIL")<<"\n";
  return ok?0:1;
}
