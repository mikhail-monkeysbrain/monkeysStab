// monkeysStab — web FC control helper.
// Commands: status | arm | disarm | mode stabilize|poshold|guided|land | takeoff <relative_alt_m>
#include "ardupilotmega/mavlink.h"
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <string>
#include <chrono>

static int64_t nowMs(){
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
[[noreturn]] static void die(const std::string& s){
  std::cerr<<"ОШИБКА: "<<s<<"\n"; std::exit(2);
}
static int openTcp(const std::string& ep){
  if(ep.rfind("tcp://",0)!=0) die("поддерживается только tcp://host:port");
  std::string hp=ep.substr(6); auto p=hp.rfind(':');
  if(p==std::string::npos) die("ожидается tcp://host:port");
  std::string host=hp.substr(0,p), port=hp.substr(p+1);
  addrinfo h{},*res=nullptr; h.ai_family=AF_UNSPEC; h.ai_socktype=SOCK_STREAM;
  int gr=getaddrinfo(host.c_str(),port.c_str(),&h,&res);
  if(gr!=0) die(std::string("getaddrinfo: ")+gai_strerror(gr));
  int fd=-1;
  for(auto* q=res;q;q=q->ai_next){
    fd=::socket(q->ai_family,q->ai_socktype,q->ai_protocol);
    if(fd<0) continue;
    if(::connect(fd,q->ai_addr,q->ai_addrlen)==0) break;
    ::close(fd); fd=-1;
  }
  freeaddrinfo(res);
  if(fd<0) die("не удалось подключиться к "+ep);
  int fl=fcntl(fd,F_GETFL,0); if(fl>=0) fcntl(fd,F_SETFL,fl|O_NONBLOCK);
  return fd;
}
static void writeAll(int fd,const uint8_t* p,size_t n){
  size_t o=0;
  while(o<n){
    ssize_t k=::write(fd,p+o,n-o);
    if(k>0){o+=(size_t)k;continue;}
    if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd q{fd,POLLOUT,0};poll(&q,1,50);continue;}
    if(k<0&&errno==EINTR)continue;
    die(std::string("write: ")+std::strerror(errno));
  }
}
struct Hb {
  uint8_t sys=0,comp=0;
  uint32_t custom_mode=0;
  uint8_t base_mode=0;
  bool ok=false;
};
static bool waitHb(int fd,Hb* out,int timeout_ms){
  mavlink_status_t st{}; mavlink_message_t m{}; uint8_t buf[2048];
  int64_t end=nowMs()+timeout_ms;
  while(nowMs()<end){
    pollfd p{fd,POLLIN,0};
    int pr=poll(&p,1,100);
    if(pr<0&&errno==EINTR)continue;
    if(pr<=0)continue;
    for(;;){
      ssize_t n=::read(fd,buf,sizeof(buf));
      if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
      if(n<=0)break;
      for(ssize_t i=0;i<n;i++){
        if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st))continue;
        if(m.msgid!=MAVLINK_MSG_ID_HEARTBEAT)continue;
        mavlink_heartbeat_t h{}; mavlink_msg_heartbeat_decode(&m,&h);
        if(h.autopilot!=MAV_AUTOPILOT_ARDUPILOTMEGA)continue;
        out->sys=m.sysid; out->comp=m.compid; out->custom_mode=h.custom_mode;
        out->base_mode=h.base_mode; out->ok=true; return true;
      }
    }
  }
  return false;
}
static const char* modeName(uint32_t m){
  switch(m){
    case 0:return "Stabilize";
    case 4:return "Guided";
    case 9:return "Land";
    case 16:return "PosHold";
    default:return "Other";
  }
}
static void printStatus(const Hb& h){
  bool armed=(h.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)!=0;
  std::cout<<"{"
           <<"\"ok\":true,"
           <<"\"sysid\":"<<(int)h.sys<<","
           <<"\"compid\":"<<(int)h.comp<<","
           <<"\"armed\":"<<(armed?"true":"false")<<","
           <<"\"custom_mode\":"<<h.custom_mode<<","
           <<"\"mode\":\""<<modeName(h.custom_mode)<<"\""
           <<"}\n";
}
int main(int argc,char** argv){
  if(argc<3){
    std::cerr<<"Использование: "<<argv[0]<<" tcp://host:port status|arm|disarm|mode [stabilize|poshold|guided|land] | takeoff <relative_alt_m>\n";
    return 2;
  }
  std::string ep=argv[1],cmd=argv[2];
  int fd=openTcp(ep);
  Hb h{};
  if(!waitHb(fd,&h,3500)) die("не получен HEARTBEAT ArduPilot");

  if(cmd=="status"){
    printStatus(h); ::close(fd); return 0;
  }

  if(cmd=="arm"||cmd=="disarm"){
    const bool want=cmd=="arm";
    mavlink_message_t m{};
    mavlink_msg_command_long_pack(
      191,199,&m,h.sys,h.comp,
      MAV_CMD_COMPONENT_ARM_DISARM,0,
      want?1.0f:0.0f, 0,0,0,0,0,0);
    uint8_t b[MAVLINK_MAX_PACKET_LEN];
    auto n=mavlink_msg_to_send_buffer(b,&m); writeAll(fd,b,n);

    int64_t end=nowMs()+5000;
    while(nowMs()<end){
      Hb q{};
      if(!waitHb(fd,&q,700))continue;
      bool armed=(q.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)!=0;
      if(armed==want){ printStatus(q); ::close(fd); return 0; }
    }
    die(want?"FC не подтвердил ARM (проверь pre-arm ошибки)":
             "FC не подтвердил DISARM (в полёте обычный disarm может быть запрещён)");
  }

  if(cmd=="mode"){
    if(argc<4) die("для mode укажите stabilize|poshold|guided|land");
    std::string name=argv[3];
    uint32_t mode=0;
    if(name=="stabilize") mode=0;
    else if(name=="guided") mode=4;
    else if(name=="land") mode=9;
    else if(name=="poshold") mode=16;
    else die("разрешены только stabilize|poshold|guided|land");

    mavlink_message_t m{};
    mavlink_msg_set_mode_pack(191,199,&m,h.sys,MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode);
    uint8_t b[MAVLINK_MAX_PACKET_LEN];
    auto n=mavlink_msg_to_send_buffer(b,&m); writeAll(fd,b,n);

    int64_t end=nowMs()+5000;
    while(nowMs()<end){
      Hb q{};
      if(!waitHb(fd,&q,700))continue;
      if(q.custom_mode==mode){ printStatus(q); ::close(fd); return 0; }
    }
    die("FC не подтвердил смену режима");
  }

  if(cmd=="takeoff"){
    if(argc<4) die("для takeoff укажите относительную высоту в метрах");
    char* endp=nullptr;
    const double alt=std::strtod(argv[3],&endp);
    if(endp==argv[3] || *endp!='\\0' || alt<0.10 || alt>10.0)
      die("высота takeoff должна быть числом 0.10..10.0 м");
    const bool armed=(h.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)!=0;
    if(!armed) die("TAKEOFF запрещён: FC должен быть ARMED");

    // Automated takeoff is a Guided command. Switch and confirm Guided first.
    {
      mavlink_message_t m{};
      mavlink_msg_set_mode_pack(191,199,&m,h.sys,MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,4);
      uint8_t b[MAVLINK_MAX_PACKET_LEN];
      auto n=mavlink_msg_to_send_buffer(b,&m); writeAll(fd,b,n);
      bool guided=false;
      int64_t end=nowMs()+5000;
      while(nowMs()<end){
        Hb q{};
        if(!waitHb(fd,&q,700))continue;
        if(q.custom_mode==4){ h=q; guided=true; break; }
      }
      if(!guided) die("FC не подтвердил переход в Guided");
    }

    mavlink_message_t m{};
    mavlink_msg_command_long_pack(
      191,199,&m,h.sys,h.comp,
      MAV_CMD_NAV_TAKEOFF,0,
      0,0,0,0,0,0,(float)alt);
    uint8_t b[MAVLINK_MAX_PACKET_LEN];
    auto n=mavlink_msg_to_send_buffer(b,&m); writeAll(fd,b,n);

    mavlink_status_t st{}; mavlink_message_t rx{}; uint8_t buf[2048];
    int64_t end=nowMs()+5000;
    while(nowMs()<end){
      pollfd p{fd,POLLIN,0};
      int pr=poll(&p,1,100);
      if(pr<0&&errno==EINTR)continue;
      if(pr<=0)continue;
      for(;;){
        ssize_t nr=::read(fd,buf,sizeof(buf));
        if(nr<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
        if(nr<=0)break;
        for(ssize_t i=0;i<nr;i++){
          if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&rx,&st))continue;
          if(rx.msgid!=MAVLINK_MSG_ID_COMMAND_ACK)continue;
          mavlink_command_ack_t ack{}; mavlink_msg_command_ack_decode(&rx,&ack);
          if(ack.command!=MAV_CMD_NAV_TAKEOFF)continue;
          if(ack.result!=MAV_RESULT_ACCEPTED)
            die("FC отклонил MAV_CMD_NAV_TAKEOFF, result="+std::to_string((int)ack.result));
          printStatus(h); ::close(fd); return 0;
        }
      }
    }
    die("FC не прислал COMMAND_ACK для MAV_CMD_NAV_TAKEOFF");
  }

  die("неизвестная команда");
}
