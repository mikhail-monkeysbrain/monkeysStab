#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "../src/astra_reference_pipeline.hpp"

namespace {
struct Row { int64_t camera_ts_ns=0, flow_send_ns=0, mono_ns=0; double age_ms=0, roll=0,pitch=0,yaw=0,luna_age_ms=0,luna_m=0; int event=0; };
struct TimedRPY { double t=0,r=0,p=0,y=0; };
struct TimedRange { double t=0,v=0; };

std::vector<std::string> split(const std::string& s){ std::vector<std::string> o; std::string x; std::stringstream ss(s); while(std::getline(ss,x,',')) o.push_back(x); return o; }
double dval(const std::vector<std::string>& v,const std::map<std::string,size_t>& h,const char* k){ auto it=h.find(k); if(it==h.end()||it->second>=v.size()||v[it->second].empty()) throw std::runtime_error(std::string("missing CSV field: ")+k); return std::stod(v[it->second]); }
int64_t ival(const std::vector<std::string>& v,const std::map<std::string,size_t>& h,const char* k){ return (int64_t)std::llround(dval(v,h,k)); }
std::vector<Row> readCsv(const std::string& path){ std::ifstream f(path); if(!f) throw std::runtime_error("cannot open "+path); std::string line; if(!std::getline(f,line)) throw std::runtime_error("empty CSV"); auto hv=split(line); std::map<std::string,size_t> h; for(size_t i=0;i<hv.size();++i) h[hv[i]]=i; std::vector<Row> out; while(std::getline(f,line)){ if(line.empty()) continue; auto v=split(line); Row r; r.camera_ts_ns=ival(v,h,"camera_ts_ns"); r.flow_send_ns=ival(v,h,"flow_send_ns"); r.mono_ns=ival(v,h,"mono_ns"); r.age_ms=dval(v,h,"fc_gyro_age_ms"); r.roll=dval(v,h,"fc_roll"); r.pitch=dval(v,h,"fc_pitch"); r.yaw=dval(v,h,"fc_yaw"); r.luna_age_ms=dval(v,h,"luna_age_ms"); r.luna_m=dval(v,h,"luna_m"); auto e=h.find("return_event"); if(e!=h.end()&&e->second<v.size()&&!v[e->second].empty()) r.event=(int)std::llround(std::stod(v[e->second])); out.push_back(r); } return out; }

double interp(double t,const std::vector<double>& ts,const std::vector<double>& vs){ if(ts.empty()) throw std::runtime_error("empty timeline"); if(t<=ts.front()) return vs.front(); if(t>=ts.back()) return vs.back(); auto it=std::upper_bound(ts.begin(),ts.end(),t); size_t j=(size_t)(it-ts.begin()),i=j-1; double a=(t-ts[i])/(ts[j]-ts[i]); return vs[i]+a*(vs[j]-vs[i]); }
void unwrap(std::vector<double>& v){ for(size_t i=1;i<v.size();++i){ double d=v[i]-v[i-1]; while(d>M_PI){v[i]-=2*M_PI;d-=2*M_PI;} while(d<-M_PI){v[i]+=2*M_PI;d+=2*M_PI;} } }

template<class T> void sortUnique(std::vector<double>& t,std::vector<T>& v){ std::vector<size_t> ix(t.size()); for(size_t i=0;i<ix.size();++i) ix[i]=i; std::stable_sort(ix.begin(),ix.end(),[&](size_t a,size_t b){return t[a]<t[b];}); std::vector<double> nt; std::vector<T> nv; for(size_t k:ix) if(nt.empty()||t[k]-nt.back()>1e-8){nt.push_back(t[k]);nv.push_back(v[k]);} t.swap(nt);v.swap(nv); }

bool readFrame(std::ifstream& f,uint64_t& ts,cv::Mat& gray){ uint32_t n=0; f.read((char*)&ts,8); if(!f){ if(f.eof()&&f.gcount()==0) return false; throw std::runtime_error("partial MJPG header ts"); } f.read((char*)&n,4); if(!f) throw std::runtime_error("partial MJPG header size"); std::vector<uchar> b(n); f.read((char*)b.data(),n); if((uint32_t)f.gcount()!=n) throw std::runtime_error("partial JPEG"); gray=cv::imdecode(b,cv::IMREAD_GRAYSCALE); if(gray.empty()) throw std::runtime_error("JPEG decode failed"); return true; }
}

int main(int argc,char** argv){
 try{
  if(argc!=2){ std::cerr<<"usage: "<<argv[0]<<" DATASET_DIR\n"; return 2; }
  cv::setNumThreads(1); cv::setRNGSeed(716);
  const std::string root=argv[1]; auto rows=readCsv(root+"/optical_flow_mavlink.csv");
  if(rows.empty()) throw std::runtime_error("no CSV rows");
  int A=-1,B=-1; for(size_t i=0;i<rows.size();++i){if(rows[i].event==1)A=(int)i;if(rows[i].event==2)B=(int)i;} if(A<0||B<0||B<=A) throw std::runtime_error("A/B events not found");

  std::vector<double> at; std::vector<TimedRPY> av; std::vector<double> rt,rv;
  for(const auto& r:rows){ if(r.age_ms>=0&&r.flow_send_ns>0){ double t=r.flow_send_ns*1e-9-r.age_ms*1e-3; at.push_back(t); av.push_back({t,r.roll,r.pitch,r.yaw}); } rt.push_back(r.mono_ns*1e-9-r.luna_age_ms*1e-3); rv.push_back(r.luna_m); }
  sortUnique(at,av); sortUnique(rt,rv); std::vector<double> rr,rp,ry; for(auto& a:av){rr.push_back(a.r);rp.push_back(a.p);ry.push_back(a.y);} unwrap(rr);unwrap(rp);unwrap(ry);

  std::ifstream mf(root+"/frames.mjpgbin",std::ios::binary); if(!mf) throw std::runtime_error("cannot open frames.mjpgbin");
  astra_reference::Pipeline pipe; std::vector<cv::Vec3d> pos(rows.size(),cv::Vec3d(0,0,0)); std::vector<double> camt(rows.size()); std::vector<char> coverage(rows.size(),0); int recovered=0,bad=0; int last_anchor=0; cv::Vec3d last_anchor_pos(0,0,0); double last_anchor_t=rows[0].camera_ts_ns*1e-9;

  for(int j=0;j<=B;++j){ uint64_t rawts=0; cv::Mat gray; if(!readFrame(mf,rawts,gray)) throw std::runtime_error("fewer RAW frames than CSV rows"); const double t=rows[j].camera_ts_ns*1e-9; camt[j]=t; astra_reference::FrameInput in; in.frame_id=j; in.camera_ts_ns=rows[j].camera_ts_ns; in.gray=gray; const double r=interp(t,at,rr),p=interp(t,at,rp),y=interp(t,at,ry); in.sensor.body_to_ned=astra_metric::rotation(r,p,y); in.sensor.range_m=interp(t,rt,rv); in.sensor.attitude_valid=true; in.sensor.range_valid=true; auto s=pipe.process(in);
    if(s.status==astra_reference::Status::ACCEPTED){ pos[j]=s.position_ned_m; if(j>last_anchor){ const cv::Vec3d delta=s.position_ned_m-last_anchor_pos; const double den=t-last_anchor_t; for(int k=last_anchor+1;k<=j;++k){ double f=den!=0?(camt[k]-last_anchor_t)/den:1.0; pos[k]=last_anchor_pos+f*delta; coverage[k]=1; } } if(s.registration.method=="SIFT_RECOVERY"||j-last_anchor>1) ++recovered; last_anchor=j;last_anchor_pos=s.position_ned_m;last_anchor_t=t; }
    else if(s.status==astra_reference::Status::RESET){ ++bad; pos[j]=(j?pos[j-1]:cv::Vec3d(0,0,0)); last_anchor=j;last_anchor_pos=pos[j];last_anchor_t=t; }
    else if(s.status!=astra_reference::Status::STARTED){ ++bad; }
    if(j&&j%250==0) std::cout<<"frame "<<j<<"/"<<B<<" bad="<<bad<<" recovered="<<recovered<<"\n";
  }
  const cv::Vec3d d=pos[B]-pos[A]; const double yawA=interp(camt[A],at,ry); const auto local=astra_metric::rotation(0,0,yawA).t()*d; const double mag=std::hypot(d[0],d[1])*1000.0; int covn=0; for(int i=A+1;i<=B;++i) covn+=coverage[i]?1:0; double cov=100.0*covn/std::max(1,B-A);
  std::cout<<std::fixed<<std::setprecision(6)<<"========================================================================\nASTRA C++ FULL RAW REPLAY\n"<<"dataset: "<<root<<"\nA/B zero-based indexes: "<<A<<"/"<<B<<"; report frames: "<<A+1<<"/"<<B+1<<"\nN/E = ("<<(d[0]*1000)<<", "<<(d[1]*1000)<<") mm\nlocal X/Y = ("<<(local[0]*1000)<<", "<<(local[1]*1000)<<") mm\nmagnitude = "<<mag<<" mm\ncoverage = "<<cov<<"%  bad_attempts="<<bad<<" recovered="<<recovered<<"\nPython C++-port reference: magnitude=325.260115 mm\nOriginal Astra locked reference: magnitude=325.2898884 mm\n========================================================================\n";
  return 0;
 }catch(const std::exception& e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}
}
