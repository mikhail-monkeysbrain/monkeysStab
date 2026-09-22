// Shadow-only gravity observer diagnostic with progress.
// Compares gyro-only gravity propagation against accelerometer-corrected observers.
// No production state or persistent FC parameters are changed.
#include "ardupilotmega/mavlink.h"
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>

struct V3 { double x=0,y=0,z=0; };
static V3 add(V3 a,V3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
static V3 sub(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
static V3 mul(V3 a,double s){return {a.x*s,a.y*s,a.z*s};}
static V3 cross(V3 a,V3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
static double norm(V3 a){return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);}
static V3 unit_to(V3 a,double mag){double n=norm(a);return n>1e-12?mul(a,mag/n):a;}

static int conn(){
  addrinfo h{},*r=nullptr;h.ai_family=AF_UNSPEC;h.ai_socktype=SOCK_STREAM;
  if(getaddrinfo("127.0.0.1","5760",&h,&r))throw std::runtime_error("getaddrinfo");
  int fd=-1;for(auto*p=r;p;p=p->ai_next){fd=socket(p->ai_family,p->ai_socktype,p->ai_protocol);
    if(fd>=0&&connect(fd,p->ai_addr,p->ai_addrlen)==0)break;if(fd>=0)close(fd);fd=-1;}
  freeaddrinfo(r);if(fd<0)throw std::runtime_error("connect tcp://127.0.0.1:5760");return fd;
}
static void sendmsg(int fd,const mavlink_message_t&m){
  uint8_t b[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(b,&m);size_t o=0;
  while(o<n){auto q=write(fd,b+o,n-o);if(q<=0)throw std::runtime_error("write");o+=size_t(q);}
}
static void rate(int fd,uint8_t sys,uint8_t comp,uint32_t id,int hz){
  mavlink_message_t m{};mavlink_msg_command_long_pack(192,191,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,
    float(id),1000000.0f/float(hz),0,0,0,0,0);sendmsg(fd,m);
}
static std::string bar(double f){
  f=std::clamp(f,0.0,1.0);const int w=20,n=int(std::floor(f*w+1e-9));std::string s="[";
  for(int i=0;i<w;i++)s+=(i<n?"#":"-");return s+"]";
}
static std::string mmss(int sec){
  char b[32];std::snprintf(b,sizeof(b),"%02d:%02d",sec/60,sec%60);return b;
}
struct Obs { double tau; V3 g; V3 sum{}; double maxr=0; uint64_t n=0; };

int main(int argc,char**argv){
 try{
  const int seconds=argc>1?std::max(20,std::atoi(argv[1])):300;
  int fd=conn();mavlink_status_t st{};mavlink_message_t m{};uint8_t b[4096],sys=0,comp=0;
  auto hb=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  while(!sys&&std::chrono::steady_clock::now()<hb){
    pollfd p{fd,POLLIN,0};if(poll(&p,1,200)<=0)continue;auto n=read(fd,b,sizeof(b));if(n<=0)continue;
    for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT&&m.sysid!=192){sys=m.sysid;comp=m.compid;break;}
  }
  if(!sys)throw std::runtime_error("heartbeat timeout");
  rate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,100);rate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50);

  V3 asum{},wsum{};uint64_t initn=0;double ar=0,ap=0;uint64_t an=0;
  std::cout<<"Gravity observer shadow; keep stand stationary. duration="<<seconds<<" s\n";
  std::cout<<"Initial 2 s: gravity + gyro bias. Observers: gyro-only, tau=1/3/10/30 s.\n";
  auto ie=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(std::chrono::steady_clock::now()<ie){
    pollfd p{fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;auto n=read(fd,b,sizeof(b));if(n<=0)continue;
    for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)&&m.sysid==sys){
      if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t q{};mavlink_msg_highres_imu_decode(&m,&q);asum=add(asum,{q.xacc,q.yacc,q.zacc});wsum=add(wsum,{q.xgyro,q.ygyro,q.zgyro});initn++;}
      else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t q{};mavlink_msg_attitude_decode(&m,&q);ar+=q.roll;ap+=q.pitch;an++;}
    }
  }
  if(initn<20)throw std::runtime_error("not enough HIGHRES_IMU samples");
  const V3 g0=mul(asum,1.0/double(initn)), wb=mul(wsum,1.0/double(initn));const double gmag=norm(g0);
  const double ar0=an?ar/an:0,ap0=an?ap/an:0;
  std::array<Obs,5> o{{{0,g0},{1,g0},{3,g0},{10,g0},{30,g0}}};
  std::cout<<std::fixed<<std::setprecision(6)<<"INIT n="<<initn<<" |g|="<<gmag<<" gyro_bias=["<<wb.x<<","<<wb.y<<","<<wb.z<<"] ATT0=["<<ar0*180/M_PI<<","<<ap0*180/M_PI<<"] deg\n";

  uint64_t last=0;double sr=0,sp=0;uint64_t sn=0;
  auto start=std::chrono::steady_clock::now(),next=start+std::chrono::seconds(10),end=start+std::chrono::seconds(seconds);
  while(std::chrono::steady_clock::now()<end){
    pollfd p{fd,POLLIN,0};if(poll(&p,1,100)>0){auto n=read(fd,b,sizeof(b));if(n<=0)break;
      for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)&&m.sysid==sys){
        if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t q{};mavlink_msg_highres_imu_decode(&m,&q);
          if(last&&q.time_usec>last){double dt=(q.time_usec-last)*1e-6;if(dt>0&&dt<0.1){
            V3 a{q.xacc,q.yacc,q.zacc},w=sub({q.xgyro,q.ygyro,q.zgyro},wb);
            for(auto&x:o){
              x.g=add(x.g,mul(cross(w,x.g),-dt));x.g=unit_to(x.g,gmag);
              if(x.tau>0){double alpha=1.0-std::exp(-dt/x.tau);V3 target=unit_to(a,gmag);x.g=unit_to(add(mul(x.g,1-alpha),mul(target,alpha)),gmag);}
              V3 r=sub(a,x.g);x.sum=add(x.sum,r);x.maxr=std::max(x.maxr,norm(r));x.n++;
            }
          }}last=q.time_usec;
        }else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t q{};mavlink_msg_attitude_decode(&m,&q);sr+=q.roll;sp+=q.pitch;sn++;}
      }
    }
    if(std::chrono::steady_clock::now()>=next){
      int elapsed=int(std::lround(std::chrono::duration<double>(next-start).count()));double f=double(elapsed)/seconds;
      std::cout<<"PROGRESS "<<bar(f)<<" "<<std::setw(3)<<int(std::round(100*f))<<"% "<<mmss(elapsed)<<"/"<<mmss(seconds)<<" left "<<mmss(std::max(0,seconds-elapsed))<<"\n";
      for(auto&x:o){V3 rm=x.n?mul(x.sum,1.0/x.n):V3{};std::cout<<"  "<<(x.tau==0?"GYRO":("TAU"+std::to_string(int(x.tau))))<<" |mean|="<<norm(rm)<<" mean=["<<rm.x<<","<<rm.y<<","<<rm.z<<"] max="<<x.maxr<<"\n";x.sum={};x.maxr=0;x.n=0;}
      double rr=sn?sr/sn:0,pp=sn?sp/sn:0;std::cout<<"  ATT=["<<rr*180/M_PI<<","<<pp*180/M_PI<<"] dATT=["<<(rr-ar0)*180/M_PI<<","<<(pp-ap0)*180/M_PI<<"] deg\n";
      sr=sp=0;sn=0;next+=std::chrono::seconds(10);
    }
  }
  close(fd);std::cout<<"DONE\n";return 0;
 }catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}
}
