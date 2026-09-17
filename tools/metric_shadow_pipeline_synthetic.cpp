#include "metric_odometry_shadow.hpp"
#include "metric_shadow_sync.hpp"
#include "metric_shadow_range_sync.hpp"
#include "runtime.hpp"

#include <opencv2/calib3d.hpp>
#include <cmath>
#include <iostream>
#include <vector>

static cv::Point2f project(const cv::Vec3d& p,const cv::Mat& K,const cv::Mat& D){
  std::vector<cv::Point3f> obj{{(float)p[0],(float)p[1],(float)p[2]}};
  std::vector<cv::Point2f> img;
  cv::projectPoints(obj,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),K,D,img);
  return img[0];
}

int main(){
  const cv::Mat K=(cv::Mat_<double>(3,3)<<568.5317075,0,315.9827108,0,569.6800556,239.8814859,0,0,1);
  const cv::Mat D=(cv::Mat_<double>(1,5)<<.0735691922,-.0952538938,-.0108105308,-.00228433736,.0821774008);
  const cv::Matx33d BRC(0,-1,0, 1,0,0, 0,0,1);
  const cv::Vec3d cam(0.0625,0,0.050), luna(0.0855,0,0.055);
  const int64_t t0=1000000000LL, t1=1020000000LL;

  std::deque<FlowFcGyro> ah;
  for(int k=-1;k<=3;k++){
    FlowFcGyro g{}; g.valid=true; g.sample_ns=t0+k*10000000LL;
    g.roll=0.02; g.pitch=-0.03; g.yaw=0.15;
    ah.push_back(g);
  }
  const auto a0s=metric_shadow_sync::interpolateAttitude(ah,t0,25.0,15.0);
  const auto a1s=metric_shadow_sync::interpolateAttitude(ah,t1,25.0,15.0);

  std::deque<LunaSample> rh;
  for(int k=-1;k<=3;k++) rh.push_back(LunaSample{0.20,100,t0+k*10000000LL});
  const auto r0s=metric_shadow_range_sync::interpolateRange(rh,t0,25.0,15.0);
  const auto r1s=metric_shadow_range_sync::interpolateRange(rh,t1,25.0,15.0);

  if(!a0s.valid||!a1s.valid||!r0s.valid||!r1s.valid){
    std::cerr<<"FAIL sync gate\n"; return 2;
  }

  metric_shadow::Attitude a0{a0s.roll,a0s.pitch,a0s.yaw,t0,true};
  metric_shadow::Attitude a1{a1s.roll,a1s.pitch,a1s.yaw,t1,true};
  const cv::Matx33d R0=metric_shadow::bodyToLocal(a0.roll,a0.pitch,a0.yaw);
  const cv::Matx33d R1=metric_shadow::bodyToLocal(a1.roll,a1.pitch,a1.yaw);
  const cv::Vec3d dp(0.035,-0.022,0.0);
  const cv::Vec3d down(0,0,1);
  const double h0=down.dot(R0*(luna-cam+r0s.distance_m*cv::Vec3d(0,0,1)));
  const double h1=down.dot(R1*(luna-cam+r1s.distance_m*cv::Vec3d(0,0,1)));

  std::vector<cv::Point2f> p0,p1;
  for(int y=-2;y<=2;y++) for(int x=-3;x<=3;x++){
    cv::Vec3d ray0=BRC*cv::Vec3d(0.10*x,0.08*y,1.0);
    ray0=R0*ray0;
    const cv::Vec3d Xw=(h0/ray0[2])*ray0;
    const cv::Vec3d ray1w=Xw-dp;
    const cv::Vec3d ray1c=BRC.t()*(R1.t()*ray1w);
    if(ray1c[2]<=0.1) continue;
    p0.push_back(project(cv::Vec3d(0.10*x,0.08*y,1.0),K,D));
    p1.push_back(project(ray1c*(1.0/ray1c[2]),K,D));
  }

  metric_shadow::Input in;
  in.t0_ns=t0; in.t1_ns=t1; in.px0=p0; in.px1=p1; in.K=K; in.D=D;
  in.a0=a0; in.a1=a1; in.range0_m=r0s.distance_m; in.range1_m=r1s.distance_m;
  in.range0_valid=in.range1_valid=true; in.body_R_camera_frd=BRC; in.body_R_camera_valid=true;
  in.camera_pos_body_frd=cam; in.range_pos_body_frd=luna;
  const auto s=metric_shadow::estimate(in);
  const double err_mm=cv::norm(s.delta_local_m-dp)*1000.0;
  std::cout<<"valid="<<s.valid<<" reason="<<metric_shadow::rejectReasonName(s.reason)
           <<" points="<<s.points<<" d=["<<s.delta_local_m[0]<<","<<s.delta_local_m[1]<<","<<s.delta_local_m[2]
           <<"] expected=["<<dp[0]<<","<<dp[1]<<","<<dp[2]<<"] err_mm="<<err_mm<<"\n";
  if(!s.valid || err_mm>0.05){ std::cerr<<"FAIL metric pipeline\n"; return 3; }

  auto broken=ah; broken.clear(); broken.push_back(ah.front()); broken.push_back(ah.back());
  const auto bad=metric_shadow_sync::interpolateAttitude(broken,t0,15.0,15.0);
  std::cout<<"broken-attitude valid="<<bad.valid<<" gap_ms="<<bad.bracket_gap_ms<<"\n";
  if(bad.valid){ std::cerr<<"FAIL reject gate\n"; return 4; }

  std::cout<<"PASS\n";
  return 0;
}
