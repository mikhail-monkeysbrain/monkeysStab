#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
struct CsvRow { int frame=-1; int luna_mm=0; };
struct RawFrame { uint64_t ts=0; cv::Mat gray; };

std::vector<std::string> split(const std::string& s){ std::vector<std::string> o; std::string x; std::stringstream ss(s); while(std::getline(ss,x,',')) o.push_back(x); return o; }

std::vector<CsvRow> readCsv(const std::string& path){
 std::ifstream f(path); if(!f) throw std::runtime_error("cannot open "+path);
 std::string line; if(!std::getline(f,line)) throw std::runtime_error("empty CSV");
 auto hv=split(line); std::map<std::string,size_t> h; for(size_t i=0;i<hv.size();++i) h[hv[i]]=i;
 if(!h.count("frame")||!h.count("luna_m")) throw std::runtime_error("CSV needs frame,luna_m");
 std::vector<CsvRow> out;
 while(std::getline(f,line)){
  if(line.empty()) continue; auto v=split(line);
  if(h["frame"]>=v.size()||h["luna_m"]>=v.size()||v[h["frame"]].empty()||v[h["luna_m"]].empty()) continue;
  CsvRow r; r.frame=std::stoi(v[h["frame"]]); r.luna_mm=(int)std::llround(std::stod(v[h["luna_m"]])*1000.0); out.push_back(r);
 }
 return out;
}

bool readFrame(std::ifstream& f,RawFrame& out){
 uint32_t n=0; f.read((char*)&out.ts,8); if(!f){ if(f.eof()&&f.gcount()==0) return false; throw std::runtime_error("partial MJPG timestamp"); }
 f.read((char*)&n,4); if(!f) throw std::runtime_error("partial MJPG size");
 std::vector<uchar> b(n); f.read((char*)b.data(),n); if((uint32_t)f.gcount()!=n) throw std::runtime_error("partial JPEG");
 out.gray=cv::imdecode(b,cv::IMREAD_GRAYSCALE); if(out.gray.empty()) throw std::runtime_error("JPEG decode failed"); return true;
}

struct Estimate { bool ok=false; double scale=0,rot_deg=0,tx=0,ty=0; int matches=0,inliers=0; };
Estimate estimate(const cv::Mat& a,const cv::Mat& b){
 Estimate e; std::vector<cv::Point2f> p0; cv::goodFeaturesToTrack(a,p0,600,0.01,7.0,cv::noArray(),7,false,0.04); if(p0.size()<30) return e;
 std::vector<cv::Point2f> p1; std::vector<uchar> st; std::vector<float> err;
 cv::calcOpticalFlowPyrLK(a,b,p0,p1,st,err,cv::Size(21,21),3,cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01));
 std::vector<cv::Point2f> q0,q1; for(size_t i=0;i<st.size();++i) if(st[i]&&p1[i].x>=0&&p1[i].y>=0&&p1[i].x<b.cols&&p1[i].y<b.rows){q0.push_back(p0[i]);q1.push_back(p1[i]);}
 e.matches=(int)q0.size(); if(q0.size()<20) return e;
 cv::Mat mask; cv::Mat M=cv::estimateAffinePartial2D(q0,q1,mask,cv::RANSAC,1.5,3000,0.995,10); if(M.empty()) return e;
 e.inliers=cv::countNonZero(mask); if(e.inliers<15) return e;
 double m00=M.at<double>(0,0),m10=M.at<double>(1,0); e.scale=std::hypot(m00,m10); e.rot_deg=std::atan2(m10,m00)*180.0/CV_PI; e.tx=M.at<double>(0,2); e.ty=M.at<double>(1,2); e.ok=std::isfinite(e.scale); return e;
}

double median(std::vector<double> v){ if(v.empty()) return NAN; std::sort(v.begin(),v.end()); size_t n=v.size(); return n%2?v[n/2]:0.5*(v[n/2-1]+v[n/2]); }
}

int main(int argc,char** argv){
 try{
  if(argc!=2){std::cerr<<"usage: "<<argv[0]<<" DATASET_DIR\n";return 2;} cv::setNumThreads(1); cv::setRNGSeed(716);
  std::string root=argv[1]; auto csv=readCsv(root+"/optical_flow_mavlink.csv"); if(csv.size()<2) throw std::runtime_error("too few CSV rows");
  std::ifstream f(root+"/frames.mjpgbin",std::ios::binary); if(!f) throw std::runtime_error("cannot open frames.mjpgbin");
  std::vector<RawFrame> raw; RawFrame rf; while(readFrame(f,rf)) raw.push_back({rf.ts,rf.gray.clone()});
  size_t n=std::min(csv.size(),raw.size()); if(n<2) throw std::runtime_error("too few aligned rows/frames");
  std::vector<double> up,down; int transitions=0,valid=0;
  std::cout<<std::fixed<<std::setprecision(6);
  std::cout<<"LUNA IMAGE-SCALE TRANSITIONS\n"<<"dataset: "<<root<<"\n"<<"CSV rows="<<csv.size()<<" RAW frames="<<raw.size()<<" aligned="<<n<<"\n";
  std::cout<<"frame_pair,luna_mm,scale,scale_pct,rot_deg,tx_px,ty_px,matches,inliers\n";
  for(size_t i=1;i<n;++i){
   int l0=csv[i-1].luna_mm,l1=csv[i].luna_mm; if(l0==l1) continue; ++transitions;
   auto e=estimate(raw[i-1].gray,raw[i].gray);
   std::cout<<csv[i-1].frame<<"->"<<csv[i].frame<<","<<l0<<"->"<<l1<<",";
   if(!e.ok){std::cout<<"INVALID,INVALID,INVALID,INVALID,INVALID,"<<e.matches<<","<<e.inliers<<"\n";continue;}
   ++valid; double pct=(e.scale-1.0)*100.0;
   std::cout<<e.scale<<","<<pct<<","<<e.rot_deg<<","<<e.tx<<","<<e.ty<<","<<e.matches<<","<<e.inliers<<"\n";
   if(l0==190&&l1==200) up.push_back(e.scale); if(l0==200&&l1==190) down.push_back(e.scale);
  }
  std::cout<<"\nSUMMARY\ntransitions="<<transitions<<" valid="<<valid<<"\n";
  std::cout<<"190->200 n="<<up.size()<<" median_scale="<<median(up)<<" median_change_pct="<<(median(up)-1.0)*100.0<<"\n";
  std::cout<<"200->190 n="<<down.size()<<" median_scale="<<median(down)<<" median_change_pct="<<(median(down)-1.0)*100.0<<"\n";
  std::cout<<"If physical range really changes 190->200 mm between adjacent views, pure height scaling would be about 190/200=0.950000 for A->B image coordinates (inverse 1.052632 depending mapping direction).\n";
  return 0;
 }catch(const std::exception& e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}
}
