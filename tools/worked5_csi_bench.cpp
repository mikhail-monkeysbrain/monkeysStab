// Bench-only CSI -> production estimateRawFlow() bridge.
// Never opens a serial port, never sends MAVLink or motor commands.
#include "../src/worked5_flow_isolated.hpp"
#include "../src/csi_camera.hpp"
#include <iostream>
#include <chrono>
int main(int argc,char** argv) {
  try {
    const int seconds=argc>1?std::stoi(argv[1]):10;
    if(seconds<1 || seconds>60) throw std::runtime_error("duration must be 1..60 s");
    // Bench camera intrinsics from OV5647 2026-10-24; mount rotation not measured.
    // Identity B_R_C is only a placeholder: body-frame outputs are NOT flight valid.
    CameraCalib calib;
    calib.fx=1097.679850; calib.fy=1090.160283;
    calib.cx=320; calib.cy=240;
    calib.B_R_C=cv::Matx33d::eye();
    calib.K=(cv::Mat_<double>(3,3)<<calib.fx,0,calib.cx,
                                        0,calib.fy,calib.cy,0,0,1);
    calib.D=(cv::Mat_<double>(1,5)<<-0.55462130,1.63595601,
                                    -0.00507743,0.00303382,0.0);
    cv::setNumThreads(2);
    CsiCamera camera;
    camera.open(30);
    cv::Mat previous;
    int64_t previous_ns=0;
    uint64_t received=0,attempted=0,valid=0,skipped=0;
    auto end=std::chrono::steady_clock::now()+std::chrono::seconds(seconds);
    while(std::chrono::steady_clock::now()<end){
      CsiCamera::Frame f;
      if(!camera.read(f,250)) continue;
      ++received;
      if(previous_ns>0 && f.sensor_ns>previous_ns) {
        const double dt=(f.sensor_ns-previous_ns)*1e-9;
        // Use EXACT production WORKED5 feature/LK/RANSAC function.
        const FlowStep step=estimateRawFlow(previous,f.gray,dt,calib);
        ++attempted;
        if(step.valid) ++valid;
        else ++skipped;
        if(attempted%30==0)
          std::cout<<"WORKED5_CSI frames="<<received
                   <<" valid="<<valid
                   <<" invalid="<<skipped
                   <<" reason="<<step.invalid_reason
                   <<" tracked="<<step.tracked
                   <<" inliers="<<step.inliers
                   <<" dt_ms="<<dt*1000.0
                   <<"\n";
      }
      previous=std::move(f.gray);
      previous_ns=f.sensor_ns;
    }
    camera.close();
    std::cout<<"WORKED5_CSI_RESULT frames="<<received
             <<" attempted="<<attempted<<" valid="<<valid
             <<" invalid="<<skipped<<"\n";
    std::cout<<"BENCH_ONLY no_fc no_mavlink no_range no_rotation_compensation\n";
    return attempted?0:1;
  }catch(const std::exception& e){
    std::cerr<<"WORKED5_CSI_ERROR "<<e.what()<<"\n";
    return 1;
  }
}
