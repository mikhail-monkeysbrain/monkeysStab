// monkeysStab — independent FC MAVLink blackbox logger.
// Lives outside the optical-flow runtime so STOP/START of Variant B does not
// create a blind interval in FC telemetry.
#include "ardupilotmega/mavlink.h"
#include <arpa/inet.h>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <netdb.h>
#include <poll.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <algorithm>

using Clock=std::chrono::steady_clock;

[[noreturn]] static void die(const std::string& s){std::cerr<<"ОШИБКА: "<<s<<"\n";std::exit(2);}
static uint64_t monoNs(){return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();}
static uint64_t wallNs(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}

static int openTcp(const std::string& ep){
  if(ep.rfind("tcp://",0)!=0)die("поддерживается только tcp://host:port");
  std::string hp=ep.substr(6);auto p=hp.rfind(':');
  if(p==std::string::npos)die("ожидается tcp://host:port");
  std::string host=hp.substr(0,p),port=hp.substr(p+1);
  addrinfo h{},*res=nullptr;h.ai_family=AF_UNSPEC;h.ai_socktype=SOCK_STREAM;
  int gr=getaddrinfo(host.c_str(),port.c_str(),&h,&res);
  if(gr!=0)die(std::string("getaddrinfo: ")+gai_strerror(gr));
  int fd=-1;
  for(auto*q=res;q;q=q->ai_next){
    fd=::socket(q->ai_family,q->ai_socktype,q->ai_protocol);if(fd<0)continue;
    if(::connect(fd,q->ai_addr,q->ai_addrlen)==0)break;
    ::close(fd);fd=-1;
  }
  freeaddrinfo(res);if(fd<0)die("не удалось подключиться к "+ep);
  int fl=fcntl(fd,F_GETFL,0);if(fl>=0)fcntl(fd,F_SETFL,fl|O_NONBLOCK);
  return fd;
}
static void writeRow(std::ostream& out,const mavlink_message_t& m){
  std::string f[33];
  auto set=[&](int i,const auto& v){std::ostringstream s;s<<std::setprecision(10)<<v;f[i]=s.str();};
  set(0,monoNs());set(1,wallNs());set(2,m.msgid);set(3,(int)m.sysid);set(4,(int)m.compid);
  if(m.msgid==MAVLINK_MSG_ID_HEARTBEAT){mavlink_heartbeat_t q{};mavlink_msg_heartbeat_decode(&m,&q);set(6,(q.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)?1:0);set(7,q.custom_mode);}
  else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t q{};mavlink_msg_attitude_decode(&m,&q);set(5,q.time_boot_ms);set(8,q.roll);set(9,q.pitch);set(10,q.yaw);set(11,q.rollspeed);set(12,q.pitchspeed);set(13,q.yawspeed);}
  else if(m.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED){mavlink_local_position_ned_t q{};mavlink_msg_local_position_ned_decode(&m,&q);set(5,q.time_boot_ms);set(14,q.x);set(15,q.y);set(16,q.z);set(17,q.vx);set(18,q.vy);set(19,q.vz);}
  else if(m.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT){mavlink_ekf_status_report_t q{};mavlink_msg_ekf_status_report_decode(&m,&q);set(20,q.flags);set(21,q.velocity_variance);set(22,q.pos_horiz_variance);set(23,q.pos_vert_variance);set(24,q.compass_variance);set(25,q.terrain_alt_variance);}
  else if(m.msgid==MAVLINK_MSG_ID_OPTICAL_FLOW){mavlink_optical_flow_t q{};mavlink_msg_optical_flow_decode(&m,&q);set(26,q.flow_comp_m_x);set(27,q.flow_comp_m_y);set(28,(int)q.quality);set(29,q.ground_distance);}
  else if(m.msgid==MAVLINK_MSG_ID_DISTANCE_SENSOR){mavlink_distance_sensor_t q{};mavlink_msg_distance_sensor_decode(&m,&q);set(5,q.time_boot_ms);set(30,q.current_distance);set(31,(int)q.orientation);set(32,(int)q.covariance);}
  for(int i=0;i<33;i++){if(i)out<<",";out<<f[i];}out<<"\n";
}

int main(int argc,char**argv){
  std::string ep=argc>1?argv[1]:"tcp://127.0.0.1:5760";
  std::string path=argc>2?argv[2]:"continuous_fc.csv";
  int fd=openTcp(ep);
  const uint64_t segment_ns=3600ULL*1000000000ULL;
  const uint64_t retention_ns=12ULL*3600ULL*1000000000ULL;
  const uintmax_t hard_limit=1100ULL*1024ULL*1024ULL;
  std::filesystem::path base(path),dir=base.parent_path();
  std::string stem=base.stem().string();
  auto segmentPath=[&](uint64_t wn){
    std::time_t t=(std::time_t)(wn/1000000000ULL);std::tm tm{};localtime_r(&t,&tm);
    char b[32];std::strftime(b,sizeof(b),"%Y%m%d_%H00",&tm);
    return dir/(stem+"_"+b+".csv");
  };
  auto cleanup=[&](uint64_t now){
    struct E{std::filesystem::path p;uint64_t ns;uintmax_t sz;};std::vector<E> v;uintmax_t total=0;
    for(const auto& e:std::filesystem::directory_iterator(dir)){
      if(!e.is_regular_file())continue;auto n=e.path().filename().string();
      if(n.rfind(stem+"_",0)!=0||e.path().extension()!=".csv")continue;
      auto ft=e.last_write_time();auto sys=std::chrono::time_point_cast<std::chrono::system_clock::duration>(ft-std::filesystem::file_time_type::clock::now()+std::chrono::system_clock::now());
      uint64_t ns=std::chrono::duration_cast<std::chrono::nanoseconds>(sys.time_since_epoch()).count();uintmax_t sz=e.file_size();
      if(now>ns&&now-ns>retention_ns){std::error_code ec;std::filesystem::remove(e.path(),ec);continue;}
      v.push_back({e.path(),ns,sz});total+=sz;
    }
    std::sort(v.begin(),v.end(),[](const E&a,const E&b){return a.ns<b.ns;});
    for(const auto&e:v){if(total<=hard_limit)break;std::error_code ec;if(std::filesystem::remove(e.p,ec))total-=e.sz;}
  };
  uint64_t seg_start=0;std::filesystem::path current;std::ofstream out;
  auto ensureOut=[&](uint64_t wn){
    uint64_t s=(wn/segment_ns)*segment_ns;if(out.is_open()&&s==seg_start)return;
    if(out.is_open()){out.flush();out.close();}seg_start=s;current=segmentPath(wn);
    out.open(current,std::ios::app);if(!out)die("не удалось открыть "+current.string());
    if(out.tellp()==0)out<<"recv_mono_ns,wall_ns,msgid,sysid,compid,time_boot_ms,armed,custom_mode,roll,pitch,yaw,rollspeed,pitchspeed,yawspeed,x,y,z,vx,vy,vz,ekf_flags,vel_var,pos_h_var,pos_v_var,compass_var,terrain_var,flow_x,flow_y,flow_quality,flow_ground_m,range_cm,range_orientation,range_covariance\\n";
    out<<std::setprecision(10);cleanup(wn);
  };
  ensureOut(wallNs());
  mavlink_status_t st{};mavlink_message_t m{};uint8_t buf[8192];
  uint64_t last_flush=monoNs();
  while(true){
    pollfd p{fd,POLLIN,0};int pr=poll(&p,1,500);
    if(pr<0&&errno==EINTR)continue;if(pr<0)die(std::string("poll: ")+std::strerror(errno));
    if(pr==0){if(monoNs()-last_flush>1000000000ULL){out.flush();last_flush=monoNs();}continue;}
    for(;;){
      ssize_t n=::read(fd,buf,sizeof(buf));
      if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<0&&errno==EINTR)continue;
      if(n<=0)die("MAVLink TCP закрыт");
      for(ssize_t i=0;i<n;i++){
        if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st))continue;
        if(m.msgid!=MAVLINK_MSG_ID_HEARTBEAT && m.msgid!=MAVLINK_MSG_ID_ATTITUDE &&
           m.msgid!=MAVLINK_MSG_ID_LOCAL_POSITION_NED && m.msgid!=MAVLINK_MSG_ID_EKF_STATUS_REPORT &&
           m.msgid!=MAVLINK_MSG_ID_OPTICAL_FLOW && m.msgid!=MAVLINK_MSG_ID_DISTANCE_SENSOR)continue;
        ensureOut(wn); writeRow(out,m);

      }
    }
    if(monoNs()-last_flush>1000000000ULL){out.flush();last_flush=monoNs();}
  }
}
