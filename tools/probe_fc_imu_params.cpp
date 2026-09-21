// Read-only probe for AHRS/EKF/INS parameters relevant to IMU attitude mismatch.
#include "ardupilotmega/mavlink.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

static int conn(){addrinfo h{},*r=nullptr;h.ai_family=AF_UNSPEC;h.ai_socktype=SOCK_STREAM;if(getaddrinfo("127.0.0.1","5760",&h,&r))throw std::runtime_error("getaddrinfo");int fd=-1;for(auto*p=r;p;p=p->ai_next){fd=socket(p->ai_family,p->ai_socktype,p->ai_protocol);if(fd>=0&&connect(fd,p->ai_addr,p->ai_addrlen)==0)break;if(fd>=0)close(fd);fd=-1;}freeaddrinfo(r);if(fd<0)throw std::runtime_error("connect");return fd;}
static void request_param(int fd,uint8_t sys,uint8_t comp,const std::string& name){mavlink_message_t m{};char id[16]{};std::strncpy(id,name.c_str(),sizeof(id));mavlink_msg_param_request_read_pack(192,191,&m,sys,comp,id,-1);uint8_t b[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(b,&m);if(write(fd,b,n)!=(ssize_t)n)throw std::runtime_error("write");}
int main(){try{
 const std::vector<std::string> names={"AHRS_TRIM_X","AHRS_TRIM_Y","AHRS_TRIM_Z","AHRS_ORIENTATION","EK3_PRIMARY","EK3_IMU_MASK","INS_USE","INS_USE2","INS_USE3","INS_ACC_ID","INS_ACC2_ID","INS_ACC3_ID","INS_GYR_ID","INS_GYR2_ID","INS_GYR3_ID"};
 int fd=conn();mavlink_status_t st{};mavlink_message_t m{};uint8_t b[4096],sys=0,comp=0;
 auto hb_end=std::chrono::steady_clock::now()+std::chrono::seconds(5);
 while(!sys&&std::chrono::steady_clock::now()<hb_end){pollfd q{fd,POLLIN,0};if(poll(&q,1,200)<=0)continue;auto n=read(fd,b,sizeof(b));for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT&&m.sysid!=192){sys=m.sysid;comp=m.compid;break;}}
 if(!sys)throw std::runtime_error("heartbeat timeout");
 for(const auto& name:names){request_param(fd,sys,comp,name);usleep(20000);}
 std::map<std::string,double> got;
 auto end=std::chrono::steady_clock::now()+std::chrono::seconds(8);
 while(std::chrono::steady_clock::now()<end&&got.size()<names.size()){pollfd q{fd,POLLIN,0};if(poll(&q,1,250)<=0)continue;auto n=read(fd,b,sizeof(b));for(ssize_t i=0;i<n;i++){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st)||m.sysid!=sys||m.msgid!=MAVLINK_MSG_ID_PARAM_VALUE)continue;mavlink_param_value_t p{};mavlink_msg_param_value_decode(&m,&p);char id[17]{};std::memcpy(id,p.param_id,16);std::string name(id);for(const auto& wanted:names)if(name==wanted)got[name]=p.param_value;}}
 close(fd);
 std::cout<<"===== FC IMU / AHRS PARAMETERS (READ ONLY) =====\n"<<std::setprecision(10);
 for(const auto& name:names){auto it=got.find(name);std::cout<<std::left<<std::setw(20)<<name<<" = ";if(it==got.end())std::cout<<"NOT RECEIVED";else std::cout<<it->second;std::cout<<"\n";}
 }catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}return 0;
}
