// Standalone realtime gate for the verified Astra frontend + metric.
// IMPORTANT: this executable never publishes OPTICAL_FLOW or DISTANCE_SENSOR.
// Interactive blind A->B->A mode: press ENTER to mark A, B and A2.
#define JTZERO_OPTFLOW_LIBRARY
#include "../src/optical_flow_mavlink.cpp"
#include "../src/astra_shadow.hpp"
#include "../src/astra_metric.hpp"

#include <atomic>
#include <chrono>
#include <thread>

namespace {
static double lerpAngle(double a,double b,double u){ return a+u*std::remainder(b-a,2.0*M_PI); }
static bool attitudeAt(FlowFc& fc,int64_t ts,FlowFcGyro* out,double* gap_ms){
  std::lock_guard<std::mutex> l(fc.mu); if(fc.attitude_history.empty()) return false;
  const auto& h=fc.attitude_history; if(ts<h.front().sample_ns||ts>h.back().sample_ns) return false;
  auto it=std::lower_bound(h.begin(),h.end(),ts,[](const FlowFcGyro& q,int64_t t){return q.sample_ns<t;});
  if(it==h.begin()){*out=*it;if(gap_ms)*gap_ms=std::abs((double)(it->sample_ns-ts))*1e-6;return true;}
  if(it==h.end()) return false; const auto& b=*it; const auto& a=*(it-1); const int64_t span=b.sample_ns-a.sample_ns; if(span<=0)return false;
  const double u=std::clamp((double)(ts-a.sample_ns)/(double)span,0.0,1.0); *out=a;
  out->roll=lerpAngle(a.roll,b.roll,u); out->pitch=lerpAngle(a.pitch,b.pitch,u); out->yaw=lerpAngle(a.yaw,b.yaw,u);
  out->x=a.x+u*(b.x-a.x);out->y=a.y+u*(b.y-a.y);out->z=a.z+u*(b.z-a.z);out->sample_ns=ts;out->recv_ns=ts;out->valid=true;
  if(gap_ms)*gap_ms=std::max(ts-a.sample_ns,b.sample_ns-ts)*1e-6; return true;
}
static bool latestGray(Camera& cam,cv::Mat* gray,int64_t* ts,uint64_t* dropped){
  std::vector<uint8_t> jpeg;int64_t best_ts=0;uint64_t drop=0;
  while(true){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}
    const int64_t bts=(int64_t)b.timestamp.tv_sec*1000000000LL+(int64_t)b.timestamp.tv_usec*1000LL;const int64_t dq=monoNs();const bool mt=(b.flags&V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)!=0;
    if(!jpeg.empty())++drop;const uint8_t* p=reinterpret_cast<const uint8_t*>(cam.bufs[b.index].p);jpeg.assign(p,p+b.bytesused);best_ts=(mt&&bts>0)?bts:dq;if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");}
  if(jpeg.empty())return false;cv::Mat raw(1,(int)jpeg.size(),CV_8UC1,jpeg.data());*gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);if(gray->empty())return false;*ts=best_ts;if(dropped)*dropped=drop;return true;
}
static void printVec(const char* name,const cv::Vec3d& v){std::cout<<name<<" N/E=("<<v[0]*1000.0<<", "<<v[1]*1000.0<<") mm magnitude="<<std::hypot(v[0],v[1])*1000.0<<" mm\n";}
}

int main(int argc,char** argv){
  if(argc<4){std::cerr<<"usage: astra_realtime_shadow <camera> <luna> <fc> [seconds]\n";return 2;}
  const std::string camdev=argv[1],lunadev=argv[2],fcdev=argv[3];const double seconds=(argc>=5)?std::stod(argv[4]):0.0;const bool blind=(seconds<=0.0);
  if(!blind&&!(seconds>=3.0&&seconds<=300.0)){std::cerr<<"seconds must be 0 or 3..300\n";return 2;}
  try{
    cv::setNumThreads(1);Camera cam;cam.openDev(camdev);LunaReader luna;luna.start(lunadev);FlowFc fc;fc.start(fcdev);std::this_thread::sleep_for(std::chrono::milliseconds(700));
    cv::Mat anchor;FlowFcGyro anchor_att{};double anchor_range=0.0;bool anchor_valid=false;cv::Vec3d total(0,0,0);
    uint64_t frames=0,accepted=0,recovered=0,failed=0,att_bad=0,range_bad=0,dropped=0;double max_att_gap_ms=0.0;const int64_t begin=monoNs();
    std::atomic<int> mark_requests{0};std::atomic<bool> input_done{false};std::thread input_thread;
    if(blind){
      std::cerr<<"\n=== ASTRA BLIND A-B-A ===\n"
               <<"1. Стойте неподвижно в A и нажмите ENTER.\n";
      input_thread=std::thread([&]{std::string line;for(int i=0;i<3&&std::getline(std::cin,line);++i)mark_requests.fetch_add(1);input_done.store(true);});
    }
    int marks=0;cv::Vec3d mark_pos[3];uint64_t mark_frame[3]{};
    while(g_running&&(blind?marks<3:(monoNs()-begin)*1e-9<seconds)){
      pollfd p{cam.fd,POLLIN,0};const int pr=poll(&p,1,20);if(pr<0){if(errno==EINTR)continue;fail("camera poll");}if(pr<=0)continue;
      cv::Mat gray;int64_t ts=0;uint64_t dd=0;if(!latestGray(cam,&gray,&ts,&dd))continue;dropped+=dd;++frames;
      double range=0.0;int strength=0;int64_t rns=0;const bool rok=luna.latest(&range,&strength,&rns)&&range>0.05&&std::abs((double)(monoNs()-rns))*1e-6<100.0;
      FlowFcGyro att{};double att_gap=1e9;const bool aok=attitudeAt(fc,ts,&att,&att_gap)&&att_gap<=30.0;if(aok)max_att_gap_ms=std::max(max_att_gap_ms,att_gap);
      if(!anchor_valid){if(!rok){++range_bad;continue;}if(!aok){++att_bad;continue;}anchor=gray.clone();anchor_att=att;anchor_range=range;anchor_valid=true;}
      else if(!aok){++att_bad;}
      else{auto reg=astra_shadow::registerFrames(anchor,gray,false);if(!reg.info.ok){++failed;}else{const auto R0=astra_metric::rotation(anchor_att.roll,anchor_att.pitch,anchor_att.yaw);const auto R1=astra_metric::rotation(att.roll,att.pitch,att.yaw);const cv::Vec3d d=astra_metric::metric(reg.a,reg.b,R0,R1,anchor_range);
        if(!std::isfinite(d[0])||!std::isfinite(d[1]))++failed;else{total+=d;++accepted;if(reg.info.method=="SIFT_RECOVERY")++recovered;anchor=gray.clone();anchor_att=att;if(rok)anchor_range=range;else++range_bad;}}}
      if(blind&&mark_requests.load()>0&&anchor_valid&&aok&&rok){mark_pos[marks]=total;mark_frame[marks]=frames;++marks;mark_requests.fetch_sub(1);
        if(marks==1)std::cerr<<"A зафиксирована.\n2. Переместите в B, остановитесь и нажмите ENTER.\n";
        else if(marks==2)std::cerr<<"B зафиксирована.\n3. Верните точно в A, остановитесь и нажмите ENTER.\n";
        else std::cerr<<"A2 зафиксирована. Тест завершён.\n\n";
      }
    }
    g_running=false;fc.stop();luna.stop();if(input_thread.joinable()){if(marks>=3&&!input_done.load())input_thread.detach();else input_thread.join();}
    std::cout<<std::fixed<<std::setprecision(6)<<"ASTRA REALTIME SHADOW RESULT\n"<<"frames="<<frames<<" accepted="<<accepted<<" failed="<<failed<<" recovered="<<recovered<<" dropped="<<dropped<<" att_bad="<<att_bad<<" range_bad="<<range_bad<<"\n"<<"max_att_interp_gap_ms="<<max_att_gap_ms<<"\n";
    if(blind&&marks==3){const cv::Vec3d ab=mark_pos[1]-mark_pos[0],ba=mark_pos[2]-mark_pos[1],closure=mark_pos[2]-mark_pos[0];std::cout<<"marks: A="<<mark_frame[0]<<" B="<<mark_frame[1]<<" A2="<<mark_frame[2]<<"\n";printVec("A_TO_B",ab);printVec("B_TO_A",ba);printVec("CLOSURE_A_TO_A2",closure);const double mab=std::hypot(ab[0],ab[1]),mba=std::hypot(ba[0],ba[1]);std::cout<<"reciprocity_magnitude_ratio="<<(mab>1e-12?mba/mab:0.0)<<"\n";if(mab>1e-12&&mba>1e-12){double c=(ab[0]*ba[0]+ab[1]*ba[1])/(mab*mba);c=std::clamp(c,-1.0,1.0);std::cout<<"forward_reverse_angle_deg="<<std::acos(c)*180.0/M_PI<<"\n";}}
    else printVec("TOTAL",total);return 0;
  }catch(const std::exception& e){g_running=false;std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}
}
