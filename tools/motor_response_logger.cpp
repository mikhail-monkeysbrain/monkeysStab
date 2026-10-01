// JT-ZERO: инструментальный тест знака стабилизации Roll/Pitch.
// Логирует ATTITUDE, HIGHRES_IMU и SERVO_OUTPUT_RAW в один CSV.
#include "ardupilotmega/mavlink.h"
#include <arpa/inet.h>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <netdb.h>
#include <poll.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

static volatile sig_atomic_t run=1;
static void sig(int){run=0;}
static uint64_t mono_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static uint64_t wall_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}
static int open_tcp(const std::string& ep){
  std::string hp=ep.rfind("tcp://",0)==0?ep.substr(6):ep; auto p=hp.rfind(':');
  if(p==std::string::npos) return -1; std::string host=hp.substr(0,p),port=hp.substr(p+1);
  addrinfo h{},*r=nullptr; h.ai_family=AF_UNSPEC; h.ai_socktype=SOCK_STREAM;
  if(getaddrinfo(host.c_str(),port.c_str(),&h,&r)!=0) return -1;
  int fd=-1; for(auto*q=r;q;q=q->ai_next){fd=socket(q->ai_family,q->ai_socktype,q->ai_protocol);if(fd>=0&&connect(fd,q->ai_addr,q->ai_addrlen)==0)break;if(fd>=0)close(fd);fd=-1;} freeaddrinfo(r); return fd;
}
static void send_interval(int fd,uint32_t msgid,int usec){
  mavlink_message_t m{}; uint8_t b[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(250,190,&m,1,1,MAV_CMD_SET_MESSAGE_INTERVAL,0,(float)msgid,(float)usec,0,0,0,0,0);
  auto n=mavlink_msg_to_send_buffer(b,&m); (void)!write(fd,b,n);
}
int main(int argc,char**argv){
  signal(SIGINT,sig); signal(SIGTERM,sig);
  std::string ep=argc>1?argv[1]:"tcp://127.0.0.1:5760", outp=argc>2?argv[2]:"motor_response.csv";
  int fd=open_tcp(ep); if(fd<0){std::cerr<<"ОШИБКА: нет MAVLink TCP "<<ep<<"\n";return 2;}
  int fl=fcntl(fd,F_GETFL,0); if(fl>=0)fcntl(fd,F_SETFL,fl|O_NONBLOCK);
  send_interval(fd,MAVLINK_MSG_ID_ATTITUDE,20000);       // 50 Hz
  send_interval(fd,MAVLINK_MSG_ID_HIGHRES_IMU,10000);    // 100 Hz
  send_interval(fd,MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,10000);// 100 Hz
  send_interval(fd,MAVLINK_MSG_ID_HEARTBEAT,200000);
  std::ofstream o(outp); if(!o){std::cerr<<"ОШИБКА: не открыть "<<outp<<"\n";return 3;}
  o<<"mono_ns,wall_ns,msgid,armed,roll,pitch,yaw,rollspeed,pitchspeed,yawspeed,imu_time_usec,gx,gy,gz,ax,ay,az,servo_time_usec,port,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16\n";
  mavlink_status_t st{}; mavlink_message_t m{}; uint8_t buf[8192]; bool armed=false; uint64_t last=mono_ns();
  while(run){
    pollfd p{fd,POLLIN,0}; int pr=poll(&p,1,200); if(pr<=0){if(mono_ns()-last>200000000ULL){o.flush();last=mono_ns();}continue;}
    ssize_t n=read(fd,buf,sizeof(buf)); if(n<=0)continue;
    for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)){
      if(m.msgid==MAVLINK_MSG_ID_HEARTBEAT){mavlink_heartbeat_t q{};mavlink_msg_heartbeat_decode(&m,&q);armed=(q.base_mode&MAV_MODE_FLAG_SAFETY_ARMED);continue;}
      std::ostringstream row; row<<mono_ns()<<","<<wall_ns()<<","<<m.msgid<<","<<(armed?1:0);
      if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t q{};mavlink_msg_attitude_decode(&m,&q);row<<","<<q.roll<<","<<q.pitch<<","<<q.yaw<<","<<q.rollspeed<<","<<q.pitchspeed<<","<<q.yawspeed<<",,,,,,,,,,,,,,,,,,,,,,,,";}
      else if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t q{};mavlink_msg_highres_imu_decode(&m,&q);row<<",,,,,,,"<<q.time_usec<<","<<q.xgyro<<","<<q.ygyro<<","<<q.zgyro<<","<<q.xacc<<","<<q.yacc<<","<<q.zacc<<",,,,,,,,,,,,,,,,,,,";}
      else if(m.msgid==MAVLINK_MSG_ID_SERVO_OUTPUT_RAW){mavlink_servo_output_raw_t q{};mavlink_msg_servo_output_raw_decode(&m,&q);row<<",,,,,,,,,,,,,"<<q.time_usec<<","<<(int)q.port<<","<<q.servo1_raw<<","<<q.servo2_raw<<","<<q.servo3_raw<<","<<q.servo4_raw<<","<<q.servo5_raw<<","<<q.servo6_raw<<","<<q.servo7_raw<<","<<q.servo8_raw<<","<<q.servo9_raw<<","<<q.servo10_raw<<","<<q.servo11_raw<<","<<q.servo12_raw<<","<<q.servo13_raw<<","<<q.servo14_raw<<","<<q.servo15_raw<<","<<q.servo16_raw;}
      else continue;
      o<<row.str()<<"\n";
    }
    if(mono_ns()-last>200000000ULL){o.flush();last=mono_ns();}
  }
  o.flush(); std::cout<<"LOG="<<outp<<"\n"; return 0;
}
