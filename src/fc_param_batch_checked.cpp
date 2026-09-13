// monkeysStab — robust MAVLink PARAM read utility for MatekH743 on /dev/ttyAMA0.
// It intentionally does NOT wait for HEARTBEAT. sysid/compid are explicit and
// PARAM_VALUE is the acknowledgement. Supports robust read and verified set.
#include "ardupilotmega/mavlink.h"
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>

static mavlink_status_t g_status{};

[[noreturn]] static void die(const std::string& s){
  std::cerr<<"ОШИБКА: "<<s<<"\n";
  std::exit(2);
}

static int open_serial(const std::string& dev,int baud){
  if(baud!=460800) die("эта утилита поддерживает baud=460800");
  int fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(fd<0) die("open "+dev+": "+std::strerror(errno));
  termios t{};
  if(tcgetattr(fd,&t)<0) die("tcgetattr: "+std::string(std::strerror(errno)));
  cfmakeraw(&t);
  cfsetispeed(&t,B460800); cfsetospeed(&t,B460800);
  t.c_cflag|=CLOCAL|CREAD;
  t.c_cflag&=~CRTSCTS; t.c_cflag&=~PARENB; t.c_cflag&=~CSTOPB;
  t.c_cflag&=~CSIZE; t.c_cflag|=CS8;
  if(tcsetattr(fd,TCSANOW,&t)<0) die("tcsetattr: "+std::string(std::strerror(errno)));
  tcflush(fd,TCIFLUSH);
  return fd;
}

static void write_all(int fd,const uint8_t* p,size_t n){
  size_t off=0;
  while(off<n){
    ssize_t k=::write(fd,p+off,n-off);
    if(k>0){off+=(size_t)k;continue;}
    if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd q{fd,POLLOUT,0};poll(&q,1,20);continue;}
    if(k<0&&errno==EINTR)continue;
    die("serial write: "+std::string(std::strerror(errno)));
  }
}

static std::string param_id_string(const mavlink_param_value_t& q){
  char id[17]{}; std::memcpy(id,q.param_id,16); return std::string(id);
}

static int64_t mono_us(){
  timespec ts{}; clock_gettime(CLOCK_MONOTONIC,&ts);
  return (int64_t)ts.tv_sec*1000000LL+ts.tv_nsec/1000;
}

static bool wait_param_value(int fd,uint8_t target_sys,const std::string& wanted,
                             mavlink_param_value_t* out,int timeout_ms){
  const int64_t end_us=mono_us()+(int64_t)timeout_ms*1000LL;
  uint8_t buf[2048];
  while(mono_us()<end_us){
    int remain_ms=(int)std::max<int64_t>(1,(end_us-mono_us())/1000);
    pollfd p{fd,POLLIN,0};
    int pr=poll(&p,1,std::min(remain_ms,50));
    if(pr<0){if(errno==EINTR)continue;die("poll: "+std::string(std::strerror(errno)));}
    if(pr==0)continue;
    for(;;){
      ssize_t n=::read(fd,buf,sizeof(buf));
      if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
      if(n<0&&errno==EINTR)continue;
      if(n<=0)break;
      for(ssize_t i=0;i<n;i++){
        mavlink_message_t m{};
        if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&g_status))continue;
        if(m.sysid!=target_sys||m.msgid!=MAVLINK_MSG_ID_PARAM_VALUE)continue;
        mavlink_param_value_t q{}; mavlink_msg_param_value_decode(&m,&q);
        if(param_id_string(q)==wanted){*out=q;return true;}
      }
    }
  }
  return false;
}

static void send_param_request(int fd,uint8_t target_sys,uint8_t target_comp,const std::string& name){
  mavlink_message_t m{};
  mavlink_msg_param_request_read_pack(191,199,&m,target_sys,target_comp,name.c_str(),-1);
  uint8_t b[MAVLINK_MAX_PACKET_LEN]; auto n=mavlink_msg_to_send_buffer(b,&m); write_all(fd,b,n);
}

static bool read_param(int fd,uint8_t target_sys,uint8_t target_comp,const std::string& name,
                       mavlink_param_value_t* out){
  for(int attempt=0;attempt<5;attempt++){
    send_param_request(fd,target_sys,target_comp,name);
    if(wait_param_value(fd,target_sys,name,out,900))return true;
  }
  return false;
}

static void send_param_set(int fd,uint8_t target_sys,uint8_t target_comp,
                           const std::string& name,float value){
  mavlink_message_t m{};
  mavlink_msg_param_set_pack(191,199,&m,target_sys,target_comp,
                             name.c_str(),value,MAV_PARAM_TYPE_REAL32);
  uint8_t b[MAVLINK_MAX_PACKET_LEN];
  auto n=mavlink_msg_to_send_buffer(b,&m);
  write_all(fd,b,n);
}

static bool set_param_verified(int fd,uint8_t target_sys,uint8_t target_comp,
                               const std::string& name,float value,
                               mavlink_param_value_t* out){
  for(int attempt=0;attempt<5;attempt++){
    send_param_set(fd,target_sys,target_comp,name,value);
    mavlink_param_value_t p{};
    if(!wait_param_value(fd,target_sys,name,&p,1200))continue;
    const float tol=std::max(1.0e-6f,std::fabs(value)*1.0e-5f);
    if(std::fabs(p.param_value-value)<=tol){
      if(out)*out=p;
      return true;
    }
  }
  return false;
}

int main(int argc,char** argv){
  if(argc<7){
    std::cerr<<"Использование:\n"
             <<"  "<<argv[0]<<" <device> <baud> <sysid> <compid> read NAME [NAME...]\n"
             <<"  "<<argv[0]<<" <device> <baud> <sysid> <compid> set NAME VALUE [NAME VALUE...]\n";
    return 2;
  }
  const std::string dev=argv[1];
  const int baud=std::stoi(argv[2]);
  const uint8_t sys=(uint8_t)std::stoi(argv[3]);
  const uint8_t comp=(uint8_t)std::stoi(argv[4]);
  const std::string mode=argv[5];
  int fd=open_serial(dev,baud);
  std::cout<<"TARGET FC sys="<<(int)sys<<" comp="<<(int)comp<<"\n";

  if(mode=="read"){
    for(int i=6;i<argc;i++){
      mavlink_param_value_t p{};
      if(!read_param(fd,sys,comp,argv[i],&p))die(std::string("параметр не прочитан: ")+argv[i]);
      std::cout<<argv[i]<<"="<<p.param_value<<"\n";
    }
  } else if(mode=="set"){
    if((argc-6)%2!=0)die("для set нужны пары NAME VALUE");
    for(int i=6;i<argc;i+=2){
      const std::string name=argv[i];
      const float value=std::stof(argv[i+1]);
      mavlink_param_value_t p{};
      if(!set_param_verified(fd,sys,comp,name,value,&p))
        die("параметр не записан/не подтверждён: "+name);
      std::cout<<name<<"="<<p.param_value<<" VERIFIED\n";
    }
  } else {
    die("неизвестный режим: "+mode);
  }

  ::close(fd);
  return 0;
}
