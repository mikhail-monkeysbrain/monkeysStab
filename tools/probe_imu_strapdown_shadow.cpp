// Shadow-only IMU gravity/strapdown diagnostic.
// Uses HIGHRES_IMU accel+gyro coherently in one sensor frame.
// It does not change production state or persistent FC parameters.
#include "ardupilotmega/mavlink.h"
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>

struct V3 { double x=0,y=0,z=0; };
static V3 add(V3 a,V3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
static V3 sub(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
static V3 mul(V3 a,double s){return {a.x*s,a.y*s,a.z*s};}
static V3 cross(V3 a,V3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
static double norm(V3 a){return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);}

static int conn(){
  addrinfo h{},*r=nullptr; h.ai_family=AF_UNSPEC; h.ai_socktype=SOCK_STREAM;
  if(getaddrinfo("127.0.0.1","5760",&h,&r)) throw std::runtime_error("getaddrinfo");
  int fd=-1;
  for(auto*p=r;p;p=p->ai_next){
    fd=socket(p->ai_family,p->ai_socktype,p->ai_protocol);
    if(fd>=0 && connect(fd,p->ai_addr,p->ai_addrlen)==0) break;
    if(fd>=0) close(fd); fd=-1;
  }
  freeaddrinfo(r);
  if(fd<0) throw std::runtime_error("connect tcp://127.0.0.1:5760");
  return fd;
}
static void sendmsg(int fd,const mavlink_message_t&m){
  uint8_t b[MAVLINK_MAX_PACKET_LEN]; auto n=mavlink_msg_to_send_buffer(b,&m);
  size_t o=0; while(o<n){auto q=write(fd,b+o,n-o); if(q<=0) throw std::runtime_error("write"); o+=size_t(q);}
}
static void rate(int fd,uint8_t sys,uint8_t comp,uint32_t id,int hz){
  mavlink_message_t m{};
  mavlink_msg_command_long_pack(192,191,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,
    float(id),1000000.0f/float(hz),0,0,0,0,0);
  sendmsg(fd,m);
}

int main(int argc,char**argv){
 try{
  const int seconds=argc>1?std::max(10,std::atoi(argv[1])):120;
  int fd=conn(); mavlink_status_t st{}; mavlink_message_t m{}; uint8_t b[4096],sys=0,comp=0;
  auto hb=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  while(!sys && std::chrono::steady_clock::now()<hb){
    pollfd p{fd,POLLIN,0}; if(poll(&p,1,200)<=0) continue; auto n=read(fd,b,sizeof(b)); if(n<=0) continue;
    for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st) &&
      m.msgid==MAVLINK_MSG_ID_HEARTBEAT && m.sysid!=192){sys=m.sysid;comp=m.compid;break;}
  }
  if(!sys) throw std::runtime_error("heartbeat timeout");
  rate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,100);
  rate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50);

  std::cout<<"Shadow IMU probe. Keep stand stationary. duration="<<seconds<<" s\n";
  std::cout<<"Initial 2 s estimate gyro bias and gravity vector; then gyro propagates gravity in HIGHRES frame.\n";

  V3 asum{},gsum{}; uint64_t initn=0; double ar=0,ap=0; uint64_t attn=0;
  auto init_end=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(std::chrono::steady_clock::now()<init_end){
    pollfd p{fd,POLLIN,0}; if(poll(&p,1,100)<=0) continue; auto n=read(fd,b,sizeof(b)); if(n<=0) continue;
    for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st) && m.sysid==sys){
      if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t q{};mavlink_msg_highres_imu_decode(&m,&q);
        asum=add(asum,{q.xacc,q.yacc,q.zacc}); gsum=add(gsum,{q.xgyro,q.ygyro,q.zgyro}); initn++;}
      else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t q{};mavlink_msg_attitude_decode(&m,&q);ar+=q.roll;ap+=q.pitch;attn++;}
    }
  }
  if(initn<20) throw std::runtime_error("not enough HIGHRES_IMU samples");
  V3 gravity=mul(asum,1.0/double(initn));
  V3 gbias=mul(gsum,1.0/double(initn));
  const double ar0=attn?ar/double(attn):0, ap0=attn?ap/double(attn):0;
  std::cout<<std::fixed<<std::setprecision(6)
    <<"INIT n="<<initn<<" gravity=["<<gravity.x<<","<<gravity.y<<","<<gravity.z<<"] |g|="<<norm(gravity)
    <<" gyro_bias=["<<gbias.x<<","<<gbias.y<<","<<gbias.z<<"]"
    <<" ATT0=["<<ar0*180/M_PI<<","<<ap0*180/M_PI<<"] deg\n";

  uint64_t last_us=0,total=0,bin_n=0; V3 rsum{}; double rmax=0,att_r=0,att_p=0; uint64_t att_bin=0;
  auto start=std::chrono::steady_clock::now(), next=start+std::chrono::seconds(10), end=start+std::chrono::seconds(seconds);
  while(std::chrono::steady_clock::now()<end){
    pollfd p{fd,POLLIN,0}; if(poll(&p,1,100)<=0) continue; auto n=read(fd,b,sizeof(b)); if(n<=0) continue;
    for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st) && m.sysid==sys){
      if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
        mavlink_highres_imu_t q{};mavlink_msg_highres_imu_decode(&m,&q);
        if(last_us && q.time_usec>last_us){
          double dt=(q.time_usec-last_us)*1e-6;
          if(dt>0 && dt<0.1){
            V3 w{sub({q.xgyro,q.ygyro,q.zgyro},gbias)};
            // A fixed inertial gravity vector expressed in body coordinates obeys dg_b/dt = -omega x g_b.
            gravity=add(gravity,mul(cross(w,gravity),-dt));
            const double gn=norm(gravity); if(gn>1e-9) gravity=mul(gravity,norm(mul(asum,1.0/double(initn)))/gn);
            V3 res=sub({q.xacc,q.yacc,q.zacc},gravity);
            rsum=add(rsum,res); rmax=std::max(rmax,norm(res)); bin_n++; total++;
          }
        }
        last_us=q.time_usec;
      } else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){
        mavlink_attitude_t q{};mavlink_msg_attitude_decode(&m,&q); att_r+=q.roll;att_p+=q.pitch;att_bin++;
      }
    }
    if(std::chrono::steady_clock::now()>=next){
      const double t=std::chrono::duration<double>(next-start).count();
      V3 rm=bin_n?mul(rsum,1.0/double(bin_n)):V3{};
      const double rr=att_bin?att_r/double(att_bin):0, pp=att_bin?att_p/double(att_bin):0;
      std::cout<<"T="<<std::setw(5)<<t<<"s residual_mean=["<<rm.x<<","<<rm.y<<","<<rm.z<<"] |mean|="<<norm(rm)
        <<" max="<<rmax<<" ATT=["<<rr*180/M_PI<<","<<pp*180/M_PI<<"] dATT=["
        <<(rr-ar0)*180/M_PI<<","<<(pp-ap0)*180/M_PI<<"] deg samples="<<bin_n<<"\n";
      rsum={};rmax=0;bin_n=0;att_r=att_p=0;att_bin=0; next+=std::chrono::seconds(10);
    }
  }
  close(fd);
  std::cout<<"DONE samples="<<total<<"\n";
  return 0;
 }catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}
}
