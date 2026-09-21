// RAW replay parity gate for the frozen WORKED5 frontend.
// This diagnostic deliberately compiles the production optical_flow_mavlink.cpp
// into the same translation unit so estimateRawFlow() cannot silently diverge.
#define JTZERO_OPTFLOW_LIBRARY 1
#include "../src/optical_flow_mavlink.cpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {
struct FrameMeta { uint64_t camera_ns=0, mono_ns=0; };
struct RangeSample { int64_t recv_ns=0; double m=0.0; bool ok=false; };

std::vector<std::string> split(const std::string& s){
  std::vector<std::string> v; std::string x; std::stringstream ss(s);
  while(std::getline(ss,x,',')) v.push_back(x);
  return v;
}
std::map<std::string,size_t> header(const std::string& s){
  auto v=split(s); std::map<std::string,size_t> h;
  for(size_t i=0;i<v.size();++i) h[v[i]]=i;
  return h;
}
std::vector<FrameMeta> readFramesCsv(const std::string& p){
  std::ifstream f(p); if(!f) throw std::runtime_error("cannot open "+p);
  std::string line; if(!std::getline(f,line)) throw std::runtime_error("empty frames.csv");
  auto h=header(line); std::vector<FrameMeta> out;
  while(std::getline(f,line)){
    if(line.empty()) continue; auto v=split(line);
    FrameMeta r;
    r.camera_ns=std::stoull(v.at(h.at("camera_ts_ns")));
    r.mono_ns=std::stoull(v.at(h.at("mono_ns")));
    out.push_back(r);
  }
  return out;
}
std::vector<RangeSample> readLuna(const std::string& p){
  std::ifstream f(p); if(!f) throw std::runtime_error("cannot open "+p);
  std::string line; if(!std::getline(f,line)) throw std::runtime_error("empty luna_raw.csv");
  auto h=header(line); std::vector<RangeSample> out;
  while(std::getline(f,line)){
    if(line.empty()) continue; auto v=split(line);
    RangeSample r;
    r.recv_ns=std::stoll(v.at(h.at("recv_mono_ns")));
    const bool checksum=std::stoi(v.at(h.at("checksum_ok")))!=0;
    const double cm=std::stod(v.at(h.at("distance_cm")));
    r.ok=checksum && cm>0.0;
    r.m=cm/100.0;
    out.push_back(r);
  }
  std::sort(out.begin(),out.end(),[](const auto&a,const auto&b){return a.recv_ns<b.recv_ns;});
  return out;
}
bool readRawFrame(std::ifstream& f,uint64_t& ts,cv::Mat& gray){
  uint32_t n=0;
  f.read(reinterpret_cast<char*>(&ts),sizeof(ts));
  if(!f){ if(f.eof()) return false; throw std::runtime_error("partial frame timestamp"); }
  f.read(reinterpret_cast<char*>(&n),sizeof(n));
  if(!f || n==0 || n>16*1024*1024) throw std::runtime_error("bad frame size");
  std::vector<uchar> b(n);
  f.read(reinterpret_cast<char*>(b.data()),n);
  if((uint32_t)f.gcount()!=n) throw std::runtime_error("partial JPEG");
  gray=cv::imdecode(b,cv::IMREAD_GRAYSCALE);
  if(gray.empty()) throw std::runtime_error("JPEG decode failed");
  return true;
}
double geometryDz(const std::string& p){
  cv::FileStorage fs(p,cv::FileStorage::READ);
  if(!fs.isOpened()) throw std::runtime_error("cannot open "+p);
  const double cz=(double)fs["camera"]["z"];
  const double rz=(double)fs["rangefinder"]["z"];
  return cz-rz;
}
}

int main(int argc,char** argv){
  try{
    if(argc<2 || argc>3){
      std::cerr<<"Использование: "<<argv[0]<<" DATASET_DIR [FOCAL_SCALE]\n";
      return 2;
    }
    const std::string root=argv[1];
    const std::string out_csv=root+"/worked5_replay.csv";
    const double focal_scale=(argc>=3)?std::stod(argv[2]):0.931;
    if(argc==8){
      g_feature_roi.x0=std::stod(argv[3]);
      g_feature_roi.y0=std::stod(argv[4]);
      g_feature_roi.x1=std::stod(argv[5]);
      g_feature_roi.y1=std::stod(argv[6]);
      g_max_features=std::stoi(argv[7]);
    } else {
      // Frozen current production defaults from config/runtime.json.
      g_feature_roi.x0=0.20;
      g_feature_roi.y0=0.32;
      g_feature_roi.x1=0.80;
      g_feature_roi.y1=0.90;
      g_max_features=500;
    }
    auto meta=readFramesCsv(root+"/frames.csv");
    auto luna=readLuna(root+"/luna_raw.csv");
    if(meta.empty()) throw std::runtime_error("frames.csv has no frames");
    if(luna.empty()) throw std::runtime_error("luna_raw.csv has no packets");

    CameraCalib calib=loadCameraCalib(root+"/camera_calibration.yaml");
    calib.fx*=focal_scale; calib.fy*=focal_scale;
    calib.K=(cv::Mat_<double>(3,3)<<calib.fx,0,calib.cx,0,calib.fy,calib.cy,0,0,1);
    const double dz=geometryDz(root+"/mount_geometry.json");

    cv::setNumThreads(4);
    g_fb_shadow_max_px=0.0;
    g_obs_shadow_enabled=false;

    std::ifstream raw(root+"/frames.mjpgbin",std::ios::binary);
    if(!raw) throw std::runtime_error("cannot open frames.mjpgbin");

    cv::Mat prev; int64_t prev_ts=0;
    double prev_h=0.0; bool prev_h_valid=false;
    size_t ri=0, decoded=0, frontend_valid=0, w5_attempt=0, w5_valid=0;
    std::array<uint64_t,7> reason{};
    double acc_x=0.0,acc_y=0.0;
    std::ofstream out(out_csv,std::ios::trunc);
    if(!out) throw std::runtime_error("cannot open "+out_csv);
    out<<"dataset_frame,camera_ts_ns,frame_mono_ns,dt_s,luna_recv_ns,luna_m,luna_age_ms,hcam_m,"
          "frontend_valid,invalid_reason,points,worked5_valid,du_norm,dv_norm,dx_m,dy_m,acc_x_m,acc_y_m\n";

    for(size_t i=0;i<meta.size();++i){
      uint64_t raw_ts=0; cv::Mat gray;
      if(!readRawFrame(raw,raw_ts,gray)) throw std::runtime_error("frames.mjpgbin shorter than frames.csv");
      ++decoded;
      if(raw_ts!=meta[i].camera_ns)
        throw std::runtime_error("RAW timestamp != frames.csv at dataset frame "+std::to_string(i+1));

      while(ri+1<luna.size() && luna[ri+1].recv_ns<=(int64_t)meta[i].mono_ns) ++ri;
      bool hvalid=false; double h=0.0;
      if(luna[ri].recv_ns<=(int64_t)meta[i].mono_ns && luna[ri].ok){
        const double age_ms=((int64_t)meta[i].mono_ns-luna[ri].recv_ns)*1e-6;
        hvalid=age_ms<100.0;
        h=luna[ri].m-dz;
        hvalid=hvalid && h>0.05 && std::isfinite(h);
      }

      const double dt=prev_ts?((int64_t)meta[i].camera_ns-prev_ts)*1e-9:0.0;
      FlowStep s;
      if(!prev.empty()) s=estimateRawFlow(prev,gray,dt,calib,
                                           prev_h_valid?prev_h:0.0,
                                           hvalid?h:0.0);
      if(s.invalid_reason>=0 && s.invalid_reason<(int)reason.size()) ++reason[s.invalid_reason];
      worked5::Step w5{};
      if(s.valid){
        ++frontend_valid;
        if(hvalid && dt>0.0 && dt<0.2){
          ++w5_attempt;
          w5=worked5::estimate(s.metric_prev_points,s.metric_curr_points,
                               calib.K,focal_scale,calib.D,h,dt);
          if(w5.valid){
            ++w5_valid;
            acc_x+=w5.dx_m; acc_y+=w5.dy_m;
          }
        }
      }
      const double luna_age_ms=(luna[ri].recv_ns<=(int64_t)meta[i].mono_ns)
        ? ((int64_t)meta[i].mono_ns-luna[ri].recv_ns)*1e-6 : -1.0;
      out<<i+1<<','<<meta[i].camera_ns<<','<<meta[i].mono_ns<<','<<dt<<','
         <<luna[ri].recv_ns<<','<<luna[ri].m<<','<<luna_age_ms<<','<<h<<','
         <<(s.valid?1:0)<<','<<s.invalid_reason<<','
         <<std::min(s.metric_prev_points.size(),s.metric_curr_points.size())<<','
         <<(w5.valid?1:0)<<','<<w5.du_norm<<','<<w5.dv_norm<<','
         <<w5.dx_m<<','<<w5.dy_m<<','<<acc_x<<','<<acc_y<<'\n';
      prev=gray; prev_ts=(int64_t)meta[i].camera_ns;
      prev_h=h; prev_h_valid=hvalid;
    }

    uint64_t extra_ts=0; cv::Mat extra;
    if(readRawFrame(raw,extra_ts,extra))
      throw std::runtime_error("frames.mjpgbin has more frames than frames.csv");

    std::cout<<std::fixed<<std::setprecision(6)
      <<"======================================================================\n"
      <<"WORKED5 FULL RAW REPLAY\n"
      <<"dataset       = "<<root<<"\n"
      <<"frames        = "<<meta.size()<<"\n"
      <<"decoded       = "<<decoded<<"\n"
      <<"frontend_valid= "<<frontend_valid<<"\n"
      <<"w5_attempt    = "<<w5_attempt<<"\n"
      <<"w5_valid      = "<<w5_valid<<"\n"
      <<"coverage      = "<<(w5_attempt?100.0*w5_valid/w5_attempt:0.0)<<" %\n"
      <<"acc_body_x_mm = "<<acc_x*1000.0<<"\n"
      <<"acc_body_y_mm = "<<acc_y*1000.0<<"\n"
      <<"endpoint_mm   = "<<std::hypot(acc_x,acc_y)*1000.0<<"\n"
      <<"invalid reasons: DT="<<reason[1]
      <<" FEATURES="<<reason[2]
      <<" TRACKED="<<reason[3]
      <<" HOMOGRAPHY="<<reason[4]
      <<" INLIERS="<<reason[5]
      <<" MAGNITUDE="<<reason[6]<<"\n"
      <<"focal_scale   = "<<focal_scale<<"\n"
      <<"camera-range dz = "<<dz<<" m\n"
      <<"======================================================================\n";
    return 0;
  }catch(const std::exception& e){
    std::cerr<<"ERROR: "<<e.what()<<"\n";
    return 1;
  }
}
