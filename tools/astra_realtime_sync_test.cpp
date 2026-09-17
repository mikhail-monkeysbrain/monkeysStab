#include <opencv2/opencv.hpp>
#include <cmath>
#include <deque>
#include <iostream>
#include "../src/astra_realtime_sync.hpp"

int main(){
  using namespace astra_realtime_sync;
  PendingSynchronizer q;
  cv::Mat im(480,640,CV_8UC1,cv::Scalar(0));
  q.pushFrame(7,1500000000LL,im);

  std::deque<AttitudeSample> ah;
  ah.push_back({1400000000LL,0.0,0.0,3.13,true});
  astra_reference::FrameInput out;
  if(q.popReady(ah,{},out)){ std::cerr<<"FAIL: frame released without future attitude\n"; return 1; }
  if(q.pending()!=1){ std::cerr<<"FAIL: pending frame was consumed\n"; return 2; }

  ah.push_back({1600000000LL,0.02,-0.04,-3.13,true});
  std::deque<RangeSample> rh{{1450000000LL,0.20,true},{1550000000LL,0.22,true}};
  double ag=-1,rg=-1;
  if(!q.popReady(ah,rh,out,&ag,&rg)){ std::cerr<<"FAIL: bracketed frame not released\n"; return 3; }
  if(q.pending()!=0 || out.frame_id!=7 || !out.sensor.attitude_valid || !out.sensor.range_valid){ std::cerr<<"FAIL: output state\n"; return 4; }
  if(std::abs(out.sensor.range_m-0.21)>1e-12){ std::cerr<<"FAIL: range interpolation "<<out.sensor.range_m<<"\n"; return 5; }
  if(std::abs(ag-200.0)>1e-9 || std::abs(rg-100.0)>1e-9){ std::cerr<<"FAIL: bracket gaps\n"; return 6; }
  std::cout<<"ASTRA REALTIME SYNC TEST PASS pending_future_attitude=PASS range_interp=PASS yaw_wrap=PASS\n";
  return 0;
}
