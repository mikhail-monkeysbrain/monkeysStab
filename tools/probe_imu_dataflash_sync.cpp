// Synchronized read-only MAVLink IMU + ArduPilot remote DataFlash capture.
// Diagnostic only: does not publish navigation data or touch camera/TF-Luna.
#include "ardupilotmega/mavlink.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

static constexpr uint8_t SELF_SYS=190;
static constexpr uint8_t SELF_COMP=MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
static constexpr size_t BLOCK=MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN;

static int conn(){
  addrinfo h{},*r=nullptr; h.ai_family=AF_UNSPEC; h.ai_socktype=SOCK_STREAM;
  if(getaddrinfo("127.0.0.1","5760",&h,&r)) throw std::runtime_error("getaddrinfo");
  int fd=-1;
  for(auto*p=r;p;p=p->ai_next){ fd=socket(p->ai_family,p->ai_socktype,p->ai_protocol);
    if(fd>=0 && connect(fd,p->ai_addr,p->ai_addrlen)==0) break;
    if(fd>=0) close(fd); fd=-1;
  }
  freeaddrinfo(r); if(fd<0) throw std::runtime_error("connect tcp://127.0.0.1:5760"); return fd;
}
static void sendmsg(int fd,mavlink_message_t&m){
  uint8_t b[MAVLINK_MAX_PACKET_LEN]; auto n=mavlink_msg_to_send_buffer(b,&m);
  size_t o=0; while(o<n){ auto k=write(fd,b+o,n-o); if(k<0){if(errno==EINTR)continue; throw std::runtime_error("write");} o+=size_t(k); }
}
static void rate(int fd,uint8_t sys,uint8_t comp,uint32_t id,int hz){
  mavlink_message_t m{}; mavlink_msg_command_long_pack(SELF_SYS,SELF_COMP,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,float(id),1000000.0f/hz,0,0,0,0,0); sendmsg(fd,m);
}
static void remote(int fd,uint8_t sys,uint8_t comp,uint32_t seq,uint8_t status){
  mavlink_message_t m{}; mavlink_msg_remote_log_block_status_pack(SELF_SYS,SELF_COMP,&m,sys,comp,seq,status); sendmsg(fd,m);
}
int main(int argc,char**argv){
 try{
  if(argc!=2){std::cerr<<"usage: "<<argv[0]<<" OUT_DIR\n"; return 2;}
  std::string dir=argv[1], bin=dir+"/fc_dataflash.bin", csv=dir+"/live_imu.csv";
  std::ofstream bo(bin,std::ios::binary|std::ios::trunc), co(csv,std::ios::trunc);
  if(!bo||!co) throw std::runtime_error("cannot open output files");
  co<<"kind,fc_time_us,ax,ay,az,roll,pitch,yaw\n";
  int fd=conn(); mavlink_status_t st{}; mavlink_message_t m{}; uint8_t b[8192],sys=0,comp=0;
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  while(!sys && std::chrono::steady_clock::now()<deadline){
    pollfd p{fd,POLLIN,0}; if(poll(&p,1,200)<=0)continue; auto n=read(fd,b,sizeof(b)); if(n<=0)continue;
    for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT&&m.sysid!=SELF_SYS){sys=m.sysid;comp=m.compid;break;}
  }
  if(!sys) throw std::runtime_error("heartbeat timeout");
  rate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50); rate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,50);
  rate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU,50); rate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU2,50);
  remote(fd,sys,comp,MAV_REMOTE_LOG_DATA_BLOCK_START,MAV_REMOTE_LOG_DATA_BLOCK_ACK);
  std::map<uint32_t,std::array<uint8_t,BLOCK>> pending; uint32_t expected=0; uint64_t rx=0,written=0,dup=0;
  uint64_t att0=0,att1=0,hi0=0,hi1=0; unsigned natt=0,nhi=0,ni1=0,ni2=0;
  std::cout<<"Keep stand stationary: synchronized capture 20 s\n";
  auto end=std::chrono::steady_clock::now()+std::chrono::seconds(20);
  while(std::chrono::steady_clock::now()<end){
    pollfd p{fd,POLLIN,0}; if(poll(&p,1,100)<=0)continue; auto n=read(fd,b,sizeof(b)); if(n<=0)continue;
    for(ssize_t j=0;j<n;j++){
      if(!mavlink_parse_char(MAVLINK_COMM_0,b[j],&m,&st)||m.sysid!=sys)continue;
      if(m.msgid==MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK){
        mavlink_remote_log_data_block_t q{}; mavlink_msg_remote_log_data_block_decode(&m,&q);
        if(q.target_system!=SELF_SYS||q.target_component!=SELF_COMP)continue; ++rx;
        if(q.seqno<expected||pending.count(q.seqno))++dup; else {std::array<uint8_t,BLOCK>a{};std::memcpy(a.data(),q.data,BLOCK);pending.emplace(q.seqno,a);}
        remote(fd,sys,comp,q.seqno,MAV_REMOTE_LOG_DATA_BLOCK_ACK);
        while(true){auto it=pending.find(expected);if(it==pending.end())break;bo.write((char*)it->second.data(),BLOCK);pending.erase(it);++expected;++written;}
      } else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){
        mavlink_attitude_t q{};mavlink_msg_attitude_decode(&m,&q);uint64_t t=uint64_t(q.time_boot_ms)*1000ULL;if(!att0)att0=t;att1=t;++natt;
        co<<"ATTITUDE,"<<t<<",,,, "<<q.roll<<","<<q.pitch<<","<<q.yaw<<"\n";
      } else if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
        mavlink_highres_imu_t q{};mavlink_msg_highres_imu_decode(&m,&q);if(!hi0)hi0=q.time_usec;hi1=q.time_usec;++nhi;
        co<<"HIGHRES_IMU,"<<q.time_usec<<","<<q.xacc<<","<<q.yacc<<","<<q.zacc<<",,,\n";
      } else if(m.msgid==MAVLINK_MSG_ID_SCALED_IMU){
        mavlink_scaled_imu_t q{};mavlink_msg_scaled_imu_decode(&m,&q);++ni1;co<<"SCALED_IMU1,"<<uint64_t(q.time_boot_ms)*1000ULL<<","<<q.xacc<<","<<q.yacc<<","<<q.zacc<<",,,\n";
      } else if(m.msgid==MAVLINK_MSG_ID_SCALED_IMU2){
        mavlink_scaled_imu2_t q{};mavlink_msg_scaled_imu2_decode(&m,&q);++ni2;co<<"SCALED_IMU2,"<<uint64_t(q.time_boot_ms)*1000ULL<<","<<q.xacc<<","<<q.yacc<<","<<q.zacc<<",,,\n";
      }
    }
  }
  remote(fd,sys,comp,MAV_REMOTE_LOG_DATA_BLOCK_STOP,MAV_REMOTE_LOG_DATA_BLOCK_ACK); close(fd); bo.flush(); co.flush();
  std::cout<<"ATTITUDE boot_us window=["<<att0<<","<<att1<<"] n="<<natt<<"\n";
  std::cout<<"HIGHRES boot_us window=["<<hi0<<","<<hi1<<"] n="<<nhi<<"\n";
  std::cout<<"SCALED_IMU1 n="<<ni1<<" SCALED_IMU2 n="<<ni2<<"\n";
  std::cout<<"REMOTE blocks rx="<<rx<<" written="<<written<<" duplicates="<<dup<<" pending="<<pending.size()<<" bytes="<<(written*BLOCK)<<"\n";
  std::cout<<"BIN="<<bin<<"\nCSV="<<csv<<"\n";
  if(written==0){std::cerr<<"ERROR: no REMOTE_LOG_DATA_BLOCK received. Check FC LOG_BACKEND_TYPE and reboot requirement.\n";return 3;}
  if(!pending.empty()){std::cerr<<"WARNING: missing remote blocks; BIN is not contiguous.\n";return 4;}
  return 0;
 }catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}
}
