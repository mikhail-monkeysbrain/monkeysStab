#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
static double ms(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}

struct R{int features=0,tracked=0,inliers=0; double feat_ms=0,lk_ms=0,ransac_ms=0;};
static R replay(const cv::Mat& prev,const cv::Mat& curr){
  R o;
  const double rx0=.20,ry0=.32,rx1=.80,ry1=.90;
  const int maxf=500;
  int x0=std::clamp((int)std::lround(rx0*prev.cols),0,prev.cols-1);
  int y0=std::clamp((int)std::lround(ry0*prev.rows),0,prev.rows-1);
  int x1=std::clamp((int)std::lround(rx1*prev.cols),x0+1,prev.cols);
  int y1=std::clamp((int)std::lround(ry1*prev.rows),y0+1,prev.rows);
  std::vector<cv::Point2f> p0,p1;
  auto t0=Clock::now();
  constexpr int G=3;
  const int per=std::max(1,(maxf+G*G-1)/(G*G));
  p0.reserve(maxf);
  for(int gy=0;gy<G;gy++){
    int cy0=y0+(y1-y0)*gy/G, cy1=y0+(y1-y0)*(gy+1)/G;
    for(int gx=0;gx<G;gx++){
      int cx0=x0+(x1-x0)*gx/G, cx1=x0+(x1-x0)*(gx+1)/G;
      if(cx1<=cx0||cy1<=cy0) continue;
      cv::Rect cell(cx0,cy0,cx1-cx0,cy1-cy0);
      std::vector<cv::Point2f> local;
      cv::goodFeaturesToTrack(prev(cell),local,per,0.01,7);
      for(auto p:local){p.x+=cell.x;p.y+=cell.y;p0.push_back(p);if((int)p0.size()>=maxf)break;}
      if((int)p0.size()>=maxf)break;
    }
    if((int)p0.size()>=maxf)break;
  }
  if(p0.size()<30){
    cv::Mat mask(prev.size(),CV_8UC1,cv::Scalar(0));
    int fx0=std::clamp((int)std::lround(.05*prev.cols),0,prev.cols-1);
    int fy0=std::clamp((int)std::lround(.25*prev.rows),0,prev.rows-1);
    int fx1=std::clamp((int)std::lround(.95*prev.cols),fx0+1,prev.cols);
    int fy1=std::clamp((int)std::lround(.98*prev.rows),fy0+1,prev.rows);
    mask(cv::Rect(fx0,fy0,fx1-fx0,fy1-fy0)).setTo(255);
    std::vector<cv::Point2f> pf; cv::goodFeaturesToTrack(prev,pf,maxf,.005,5,mask);
    if(pf.size()>p0.size())p0.swap(pf);
  }
  auto t1=Clock::now(); o.feat_ms=ms(t0,t1); o.features=(int)p0.size();
  if(p0.size()<30)return o;
  std::vector<uchar> st;std::vector<float> err;
  auto l0=Clock::now();
  cv::calcOpticalFlowPyrLK(prev,curr,p0,p1,st,err,{21,21},3,
    cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.01),0,1e-4);
  auto l1=Clock::now();o.lk_ms=ms(l0,l1);
  std::vector<cv::Point2f>a,b;
  for(size_t i=0;i<p0.size();i++)if(st[i]){a.push_back(p0[i]);b.push_back(p1[i]);}
  o.tracked=(int)a.size();if(a.size()<20)return o;
  cv::Mat mask;auto r0=Clock::now();
  cv::findHomography(a,b,cv::RANSAC,2.0,mask,350,.99);
  auto r1=Clock::now();o.ransac_ms=ms(r0,r1);
  if(!mask.empty())for(int i=0;i<mask.rows;i++)if(mask.at<uchar>(i))o.inliers++;
  return o;
}
static std::map<int,std::vector<std::string>> prod(const fs::path&p){
  std::ifstream f(p);std::string h,s;std::getline(f,h);std::vector<std::string> hh;std::stringstream hs(h);
  while(std::getline(hs,s,','))hh.push_back(s);
  std::map<int,std::vector<std::string>> m;
  while(std::getline(f,s)){std::vector<std::string>v;std::stringstream ss(s);std::string x;while(std::getline(ss,x,','))v.push_back(x);
    if(v.size()==hh.size()){int fi=-1;for(size_t i=0;i<hh.size();i++)if(hh[i]=="frame")fi=std::stoi(v[i]);if(fi>=0)m[fi]=v;}}
  m[-1]=hh;return m;
}
static std::string col(const std::map<int,std::vector<std::string>>&m,int fr,const std::string&n){
 auto &h=m.at(-1);auto it=m.find(fr);if(it==m.end())return "-";for(size_t i=0;i<h.size();i++)if(h[i]==n)return it->second[i];return "-";
}
int main(int argc,char**argv){
 if(argc<4){std::cerr<<"usage: exact_replay RUN START END [opencv_threads]\n";return 2;}
 fs::path run=argv[1],cap=run/"lk_forensic";int a=std::stoi(argv[2]),b=std::stoi(argv[3]);
 if(argc>=5) cv::setNumThreads(std::max(1,std::stoi(argv[4])));
 std::cout<<"OpenCV requested_threads="<<(argc>=5?std::stoi(argv[4]):-1)
          <<" effective_threads="<<cv::getNumThreads()<<" CPUs="<<cv::getNumberOfCPUs()<<"\n";
 auto p=prod(run/"optical_flow_mavlink.csv");
 std::cout<<"frame | production F/T/I LKms | exact-replay F/T/I LKms | delta F/T/I\n";
 for(int fr=a;fr<=b;fr++){
  cv::Mat prev=cv::imread((cap/("frame_"+std::to_string(fr-1)+".jpg")).string(),0);
  cv::Mat curr=cv::imread((cap/("frame_"+std::to_string(fr)+".jpg")).string(),0);
  if(prev.empty()||curr.empty()){std::cout<<fr<<" missing\n";continue;}
  R q=replay(prev,curr);
  int pf=std::stoi(col(p,fr,"features")),pt=std::stoi(col(p,fr,"tracked")),pi=std::stoi(col(p,fr,"inliers"));
  std::cout<<fr<<" | "<<pf<<"/"<<pt<<"/"<<pi<<" "<<col(p,fr,"t_lk_ms")
    <<" | "<<q.features<<"/"<<q.tracked<<"/"<<q.inliers<<" "<<std::fixed<<std::setprecision(3)<<q.lk_ms
    <<" | "<<q.features-pf<<"/"<<q.tracked-pt<<"/"<<q.inliers-pi<<"\n";
 }
}
