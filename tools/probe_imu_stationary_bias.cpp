// Stationary IMU bias stability probe. Temporary MAVLink requests only.
// Measures 1-second means/stddev of gravity-removed NED acceleration.
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
#include <vector>

struct Stats{double s=0,s2=0;int n=0;void add(double x){s+=x;s2+=x*x;n++;}double mean()const{return n?s/n:0;}double sd()const{if(n<2)return 0;double v=(s2-s*s/n)/(n-1);return std::sqrt(v>0?v:0);}};
static int conn(){addrinfo h{},*r=nullptr;h.ai_family=AF_UNSPEC;h.ai_socktype=SOCK_STREAM;if(getaddrinfo("127.0.0.1","5760",&h,&r))throw std::runtime_error("getaddrinfo");int fd=-1;for(auto*p=r;p;p=p->ai_next){fd=socket(p->ai_family,p->ai_socktype,p->ai_protocol);if(fd>=0&&connect(fd,p->ai_addr,p->ai_addrlen)==0)break;if(fd>=0)close(fd);fd=-1;}freeaddrinfo(r);if(fd<0)throw std::runtime_error("connect");return fd;}
static void rate(int fd,uint8_t sys,uint8_t comp,uint32_t id,int hz){mavlink_message_t m{};mavlink_msg_command_long_pack(192,191,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,(float)id,1000000.0f/hz,0,0,0,0,0);uint8_t b[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(b,&m);if(write(fd,b,n)!=(ssize_t)n)throw std::runtime_error("write");}
int main(){try{int fd=conn();mavlink_status_t st{};mavlink_message_t m{};uint8_t b[4096],sys=0,comp=0;double r=0,p=0,y=0;bool att=false;auto dl=std::chrono::steady_clock::now()+std::chrono::seconds(5);while(!sys&&std::chrono::steady_clock::now()<dl){pollfd q{fd,POLLIN,0};if(poll(&q,1,200)<=0)continue;auto n=read(fd,b,sizeof(b));for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT&&m.sysid!=192){sys=m.sysid;comp=m.compid;break;}}if(!sys)throw std::runtime_error("heartbeat timeout");rate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);rate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,50);
std::cout<<"Keep stand stationary: 20 s, reporting 1 s windows\n";Stats total[3],win[3],raw[3],attst[2];int sec=0;auto next=std::chrono::steady_clock::now()+std::chrono::seconds(1),end=std::chrono::steady_clock::now()+std::chrono::seconds(20);while(std::chrono::steady_clock::now()<end){pollfd q{fd,POLLIN,0};if(poll(&q,1,100)>0){auto n=read(fd,b,sizeof(b));for(ssize_t i=0;i<n;i++){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)||m.sysid!=sys)continue;if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&m,&a);r=a.roll;p=a.pitch;y=a.yaw;att=true;}else if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU&&att){mavlink_highres_imu_t a{};mavlink_msg_highres_imu_decode(&m,&a);double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p),cy=cos(y),sy=sin(y);double N=cy*cp*a.xacc+(cy*sp*sr-sy*cr)*a.yacc+(cy*sp*cr+sy*sr)*a.zacc;double E=sy*cp*a.xacc+(sy*sp*sr+cy*cr)*a.yacc+(sy*sp*cr-cy*sr)*a.zacc;double D=-sp*a.xacc+cp*sr*a.yacc+cp*cr*a.zacc+9.80665;double v[3]={N,E,D};for(int k=0;k<3;k++){win[k].add(v[k]);total[k].add(v[k]);} raw[0].add(a.xacc);raw[1].add(a.yacc);raw[2].add(a.zacc);attst[0].add(r);attst[1].add(p);}}}
if(std::chrono::steady_clock::now()>=next){++sec;std::cout<<std::fixed<<std::setprecision(5)<<"t="<<std::setw(2)<<sec<<"s mean=["<<win[0].mean()<<","<<win[1].mean()<<","<<win[2].mean()<<"] sd=["<<win[0].sd()<<","<<win[1].sd()<<","<<win[2].sd()<<"] n="<<win[0].n<<"\n";for(auto&x:win)x=Stats{};next+=std::chrono::seconds(1);}}
close(fd);std::cout<<"TOTAL mean N/E/D linear = ["<<total[0].mean()<<","<<total[1].mean()<<","<<total[2].mean()<<"] m/s^2\nTOTAL sd   N/E/D linear = ["<<total[0].sd()<<","<<total[1].sd()<<","<<total[2].sd()<<"] m/s^2\nRAW mean body xyz = ["<<raw[0].mean()<<","<<raw[1].mean()<<","<<raw[2].mean()<<"] m/s^2\nATT mean roll/pitch = ["<<attst[0].mean()*180.0/M_PI<<","<<attst[1].mean()*180.0/M_PI<<"] deg\n";}catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}return 0;}
