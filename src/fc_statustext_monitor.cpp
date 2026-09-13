// monkeysStab — stream ArduPilot STATUSTEXT as JSON lines over MAVLink TCP.
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
    if(fd<0)continue;
    if(::connect(fd,q->ai_addr,q->ai_addrlen)==0)break;
    ::close(fd);fd=-1;
  }
  freeaddrinfo(res);
  if(fd<0)die("не удалось подключиться к "+ep);
  int fl=fcntl(fd,F_GETFL,0); if(fl>=0)fcntl(fd,F_SETFL,fl|O_NONBLOCK);
  return fd;
}
static std::string esc(const char* s,size_t n){
  std::string o;
  for(size_t i=0;i<n && s[i];++i){
    unsigned char c=(unsigned char)s[i];
    if(c=='"'||c=='\\'){o.push_back('\\');o.push_back((char)c);}
    else if(c=='\n')o+="\\n";
    else if(c=='\r')o+="\\r";
    else if(c=='\t')o+="\\t";
    else if(c>=32)o.push_back((char)c);
  }
  return o;
}
int main(int argc,char** argv){
  std::string ep=argc>1?argv[1]:"tcp://127.0.0.1:5760";
  int fd=openTcp(ep);
  mavlink_status_t st{};mavlink_message_t m{};uint8_t buf[4096];
  while(true){
    pollfd p{fd,POLLIN,0};
    int pr=poll(&p,1,500);
    if(pr<0&&errno==EINTR)continue;
    if(pr<0)die(std::string("poll: ")+std::strerror(errno));
    if(pr==0)continue;
    for(;;){
      ssize_t n=::read(fd,buf,sizeof(buf));
      if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
      if(n<0&&errno==EINTR)continue;
      if(n<=0)die("MAVLink TCP закрыт");
      for(ssize_t i=0;i<n;i++){
        if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st))continue;
        if(m.msgid!=MAVLINK_MSG_ID_STATUSTEXT)continue;
        mavlink_statustext_t q{};mavlink_msg_statustext_decode(&m,&q);
        std::cout<<"{\"sysid\":"<<(int)m.sysid
                 <<",\"compid\":"<<(int)m.compid
                 <<",\"severity\":"<<(int)q.severity
                 <<",\"id\":"<<q.id
                 <<",\"chunk_seq\":"<<(int)q.chunk_seq
                 <<",\"text\":\""<<esc(q.text,sizeof(q.text))<<"\"}\n"
                 <<std::flush;
      }
    }
  }
}
