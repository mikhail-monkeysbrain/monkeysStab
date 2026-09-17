// Live diagnostic: request HIGHRES_IMU + ATTITUDE from the existing MAVLink router,
// rotate measured body acceleration into NED using ATTITUDE, and show both possible
// gravity-removal sign conventions. This does not modify runtime or FC parameters.
#include "ardupilotmega/mavlink.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>

static int connectTcp(){
  addrinfo h{},*r=nullptr; h.ai_family=AF_UNSPEC; h.ai_socktype=SOCK_STREAM;
  if(getaddrinfo("127.0.0.1","5760",&h,&r)!=0) throw std::runtime_error("getaddrinfo");
  int fd=-1; for(auto*p=r;p;p=p->ai_next){fd=socket(p->ai_family,p->ai_socktype,p->ai_protocol); if(fd>=0&&connect(fd,p->ai_addr,p->ai_addrlen)==0)break; if(fd>=0)close(fd);fd=-1;} freeaddrinfo(r);
  if(fd<0) throw std::runtime_error("connect tcp:5760"); return fd;
}
static void sendRate(int fd,uint8_t sys,uint8_t comp,uint32_t id,int hz){
  mavlink_message_t m{}; mavlink_msg_command_long_pack(192,191,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,(float)id,1000000.0f/hz,0,0,0,0,0);
  uint8_t b[MAVLINK_MAX_PACKET_LEN]; auto n=mavlink_msg_to_send_buffer(b,&m); if(write(fd,b,n)!=(ssize_t)n) throw std::runtime_error("write");
}
int main(){
 try{
  int fd=connectTcp(); mavlink_status_t st{}; mavlink_message_t m{}; uint8_t b[4096]; uint8_t sys=0,comp=0;
  double roll=0,pitch=0,yaw=0; bool att=false; uint64_t imu_n=0,att_n=0; double sumN=0,sumE=0,sumD=0,sumP=0,sumM=0;
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  while(!sys&&std::chrono::steady_clock::now()<deadline){pollfd p{fd,POLLIN,0};if(poll(&p,1,200)<=0)continue;auto n=read(fd,b,sizeof(b));for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT&&m.sysid!=192){sys=m.sysid;comp=m.compid;break;}}
  if(!sys)throw std::runtime_error("heartbeat timeout"); sendRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100); sendRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,50);
  std::cout<<"FC sys="<<(int)sys<<" comp="<<(int)comp<<"; collecting stationary diagnostic 10 s...\n";
  auto end=std::chrono::steady_clock::now()+std::chrono::seconds(10);
  while(std::chrono::steady_clock::now()<end){
   pollfd p{fd,POLLIN,0};if(poll(&p,1,200)<=0)continue;auto n=read(fd,b,sizeof(b));if(n<=0)continue;
   for(ssize_t i=0;i<n;i++){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)||m.sysid!=sys)continue;
    if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t q{};mavlink_msg_attitude_decode(&m,&q);roll=q.roll;pitch=q.pitch;yaw=q.yaw;att=true;att_n++;}
    else if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU&&att){mavlink_highres_imu_t q{};mavlink_msg_highres_imu_decode(&m,&q);
     const double cr=cos(roll),sr=sin(roll),cp=cos(pitch),sp=sin(pitch),cy=cos(yaw),sy=sin(yaw);
     const double r00=cy*cp,r01=cy*sp*sr-sy*cr,r02=cy*sp*cr+sy*sr;
     const double r10=sy*cp,r11=sy*sp*sr+cy*cr,r12=sy*sp*cr-cy*sr;
     const double r20=-sp,r21=cp*sr,r22=cp*cr;
     const double N=r00*q.xacc+r01*q.yacc+r02*q.zacc,E=r10*q.xacc+r11*q.yacc+r12*q.zacc,D=r20*q.xacc+r21*q.yacc+r22*q.zacc;
     sumN+=N;sumE+=E;sumD+=D;sumP+=(D+9.80665);sumM+=(D-9.80665);imu_n++;
    }
   }
  }
  close(fd); std::cout<<std::fixed<<std::setprecision(6)<<"\n===== STATIONARY IMU/NED =====\nATTITUDE packets: "<<att_n<<"\nHIGHRES_IMU packets: "<<imu_n<<"\n";
  if(imu_n){std::cout<<"mean rotated NED [m/s^2]: N="<<sumN/imu_n<<" E="<<sumE/imu_n<<" D="<<sumD/imu_n<<"\nD + g = "<<sumP/imu_n<<" m/s^2\nD - g = "<<sumM/imu_n<<" m/s^2\n";}
 }catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;} return 0;
}
