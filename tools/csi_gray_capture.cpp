// CSI grayscale capture validation, no FC/MAVLink activity.
#include "../src/csi_camera.hpp"
#include <iostream>
#include <chrono>
int main(){
  try {
    CsiCamera cam;
    cam.open(30);
    int n=0; int64_t first=0,last=0;
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
    while(std::chrono::steady_clock::now()<deadline){
      CsiCamera::Frame frame;
      if(!cam.read(frame,500)) continue;
      if(frame.gray.empty() || frame.gray.cols!=640 || frame.gray.rows!=480)
        throw std::runtime_error("invalid grayscale frame");
      if(!first) first=frame.sensor_ns;
      last=frame.sensor_ns;
      ++n;
    }
    cam.close();
    std::cout<<"CSI_GRAY_FRAMES "<<n<<"\n";
    std::cout<<"CSI_GRAY_SENSOR_FPS "
             <<((n>1 && last>first)?(n-1)*1e9/double(last-first):0)<<"\n";
    return n>0?0:1;
  } catch(const std::exception& e){
    std::cerr<<"CSI_GRAY_ERROR "<<e.what()<<"\n";
    return 1;
  }
}
