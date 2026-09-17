#include <opencv2/opencv.hpp>
#include <iomanip>
#include <iostream>
#include <string>
#include "../src/astra_shadow.hpp"

int main(int argc,char** argv){
  if(argc<3 || argc>4){
    std::cerr<<"usage: "<<argv[0]<<" PREV.jpg CURR.jpg [force_sift=0|1]\n";
    return 2;
  }
  cv::setNumThreads(1);
  cv::setRNGSeed(716);
  cv::Mat a=cv::imread(argv[1],cv::IMREAD_GRAYSCALE);
  cv::Mat b=cv::imread(argv[2],cv::IMREAD_GRAYSCALE);
  if(a.empty()||b.empty()){
    std::cerr<<"cannot read input image(s)\n";
    return 2;
  }
  const bool force=(argc==4 && std::string(argv[3])!="0");
  const auto r=astra_shadow::registerFrames(a,b,force);
  std::cout<<std::fixed<<std::setprecision(9)
           <<"ASTRA C++ FRONTEND PARITY\n"
           <<"method="<<r.info.method<<" ok="<<(r.info.ok?1:0)
           <<" n="<<r.info.n<<" inliers="<<r.info.inliers
           <<" cells="<<r.info.cells<<" residual="<<r.info.residual<<"\n";
  for(size_t i=0;i<r.a.size();++i)
    std::cout<<i<<','<<r.a[i].x<<','<<r.a[i].y<<','<<r.b[i].x<<','<<r.b[i].y<<'\n';
  return r.info.ok?0:1;
}
