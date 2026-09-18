// monkeysStab — passive synchronized FC HIGHRES_IMU / ATTITUDE + external MPU capture.
#include "ardupilotmega/mavlink.h"
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <linux/i2c-dev.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

[[noreturn]] static void die(const std::string& s){
  std::cerr<<"ОШИБКА: "<<s<<"\n"; std::exit(2);
}
static int64_t monoNs(){
  timespec ts{}; clock_gettime(CLOCK_MONOTONIC,&ts);
  return int64_t(ts.tv_sec)*1000000000LL+ts.tv_nsec;
}
static int openTcp(const std::string& ep){
  if(ep.rfind("tcp://",0)!=0) die("поддерживается только tcp://host:port");
  std::string hp=ep.substr(6); auto p=hp.rfind(':');
  if(p==std::string::npos) die("ожидается tcp://host:port");
  std::string host=hp.substr(0,p),port=hp.substr(p+1);
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
static int openI2c(const std::string& dev,int addr){
  int fd=::open(dev.c_str(),O_RDWR);
  if(fd<0) die("open "+dev+": "+std::strerror(errno));
  if(ioctl(fd,I2C_SLAVE,addr)<0) die("I2C_SLAVE: "+std::string(std::strerror(errno)));
  auto wr=[&](uint8_t reg,uint8_t val){
    uint8_t b[2]{reg,val};
    if(::write(fd,b,2)!=2) die("MPU write register");
  };
  wr(0x6B,0x01); wr(0x1A,0x03); wr(0x1B,0x00); wr(0x1C,0x00);
  usleep(200000);
  return fd;
}
static int16_t be16(const uint8_t* p){
  return int16_t((uint16_t(p[0])<<8)|uint16_t(p[1]));
}
struct Writer{
  std::mutex mu;
  std::ofstream f;
  uint64_t fc_imu=0,att=0,sy=0;
  explicit Writer(const std::string& path):f(path){
    if(!f) die("не удалось создать "+path);
    f<<"type,mono_ns,sensor_time,ax,ay,az,gx,gy,gz,roll,pitch,yaw,rollspeed,pitchspeed,yawspeed,read_us\n";
    f<<std::setprecision(12);
  }
  void fcImu(int64_t ns,const mavlink_highres_imu_t& q){
    std::lock_guard<std::mutex> l(mu);
    f<<"FC_IMU,"<<ns<<","<<q.time_usec<<","
     <<q.xacc<<","<<q.yacc<<","<<q.zacc<<","
     <<q.xgyro<<","<<q.ygyro<<","<<q.zgyro<<",,,,,,,\n";
    ++fc_imu;
  }
  void attitude(int64_t ns,const mavlink_attitude_t& q){
    std::lock_guard<std::mutex> l(mu);
    f<<"ATT,"<<ns<<","<<q.time_boot_ms<<",,,,,,,"
     <<q.roll<<","<<q.pitch<<","<<q.yaw<<","
     <<q.rollspeed<<","<<q.pitchspeed<<","<<q.yawspeed<<",\n";
    ++att;
  }
  void syImu(int64_t ns,double read_us,const int16_t* v){
    std::lock_guard<std::mutex> l(mu);
    f<<"SY_IMU,"<<ns<<",,"
     <<double(v[0])/16384.0<<","<<double(v[1])/16384.0<<","<<double(v[2])/16384.0<<","
     <<double(v[4])/131.0<<","<<double(v[5])/131.0<<","<<double(v[6])/131.0
     <<",,,,,,,"<<read_us<<"\n";
    ++sy;
  }
};
int main(int argc,char** argv){
  double seconds=15.0;
  std::string out="dual_imu_sync.csv";
  std::string ep="tcp://127.0.0.1:5760";
  std::string i2c="/dev/i2c-1";
  int addr=0x68;
  for(int i=1;i<argc;i++){
    std::string a=argv[i];
    auto need=[&](){if(i+1>=argc)die("нет значения для "+a);return std::string(argv[++i]);};
    if(a=="--seconds") seconds=std::stod(need());
    else if(a=="--out") out=need();
    else if(a=="--tcp") ep=need();
    else if(a=="--i2c") i2c=need();
    else if(a=="--addr") addr=std::stoi(need(),nullptr,0);
    else die("неизвестный аргумент: "+a);
  }
  if(seconds<=0) die("--seconds должен быть > 0");

  int mpu=openI2c(i2c,addr);
  Writer w(out);
  std::atomic<bool> run{true};

  std::cout<<"DUAL IMU SYNC CAPTURE\n"
           <<"TCP passive: "<<ep<<"\n"
           <<"External MPU: "<<i2c<<" @ 0x"<<std::hex<<addr<<std::dec<<"\n"
           <<"CSV: "<<out<<"\n\n"
           <<"После ENTER: 2-3 с покоя -> один сдвиг -> полная остановка -> покой.\n"
           <<"Нажми ENTER для начала записи..."<<std::flush;
  std::string line; std::getline(std::cin,line);

  // Connect only after ENTER. Otherwise the router can fill the TCP receive
  // buffer while the operator waits, and capture starts by draining old MAVLink.
  int tcp=openTcp(ep);

  const int64_t start_ns=monoNs();
  const int64_t end_ns=start_ns+int64_t(seconds*1e9);

  std::thread ft([&]{
    mavlink_status_t st{}; mavlink_message_t m{}; uint8_t buf[4096];
    while(run.load()){
      pollfd p{tcp,POLLIN,0};
      int pr=poll(&p,1,20);
      if(pr<0&&errno==EINTR) continue;
      if(pr<0){run=false;break;}
      if(pr==0) continue;
      for(;;){
        ssize_t n=::read(tcp,buf,sizeof(buf));
        if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)) break;
        if(n<0&&errno==EINTR) continue;
        if(n<=0){run=false;break;}
        for(ssize_t i=0;i<n;i++){
          if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)) continue;
          const int64_t ns=monoNs();
          if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
            mavlink_highres_imu_t q{}; mavlink_msg_highres_imu_decode(&m,&q);
            w.fcImu(ns,q);
          }else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){
            mavlink_attitude_t q{}; mavlink_msg_attitude_decode(&m,&q);
            w.attitude(ns,q);
          }
        }
      }
    }
  });

  std::thread mt([&]{
    const int64_t period_ns=5000000LL; // target 200 Hz; actual timestamps are authoritative.
    int64_t next=monoNs();
    while(run.load()){
      uint8_t reg=0x3b,b[14]{};
      int64_t t0=monoNs();
      if(::write(mpu,&reg,1)!=1 || ::read(mpu,b,14)!=14){run=false;break;}
      int64_t t1=monoNs();
      int16_t v[7];
      for(int i=0;i<7;i++) v[i]=be16(b+2*i);
      w.syImu((t0+t1)/2,double(t1-t0)/1000.0,v);
      next+=period_ns;
      int64_t now=monoNs();
      if(next>now){
        timespec ts{(next-now)/1000000000LL,(next-now)%1000000000LL};
        nanosleep(&ts,nullptr);
      }else next=now;
    }
  });

  while(run.load() && monoNs()<end_ns) usleep(10000);
  run=false;
  ft.join(); mt.join();
  ::close(tcp); ::close(mpu);

  const double actual=(monoNs()-start_ns)/1e9;
  std::cout<<"\nЗапись завершена: "<<actual<<" s\n"
           <<"FC_IMU: "<<w.fc_imu<<" samples ("<<(w.fc_imu/actual)<<" Hz)\n"
           <<"ATT:    "<<w.att<<" samples ("<<(w.att/actual)<<" Hz)\n"
           <<"SY_IMU: "<<w.sy<<" samples ("<<(w.sy/actual)<<" Hz)\n"
           <<"CSV: "<<out<<"\n";
  if(w.fc_imu<10) return 3;
  if(w.sy<10) return 4;
  return 0;
}
