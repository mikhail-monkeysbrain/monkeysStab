#include "../src/astra_reference_pipeline.hpp"

#include <iostream>

int main(){
  astra_reference::Pipeline p;
  astra_reference::FrameInput f;
  f.frame_id=0;
  f.camera_ts_ns=1;
  f.gray=cv::Mat(480,640,CV_8UC1,cv::Scalar(0));
  f.sensor.body_to_ned=astra_metric::M3::eye();
  f.sensor.range_m=0.20;
  f.sensor.attitude_valid=true;
  f.sensor.range_valid=true;
  const auto s=p.process(f);
  if(s.status!=astra_reference::Status::STARTED || !p.started() || p.anchorId()!=0){
    std::cerr<<"FAIL\n";
    return 1;
  }
  std::cout<<"ASTRA REFERENCE PIPELINE COMPILE GATE PASS\n";
  return 0;
}
