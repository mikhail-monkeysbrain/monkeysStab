// Compare FC ATTITUDE with gravity direction from MAVLink IMU streams.
// Diagnostic only. Keep vehicle stationary.
#include "ardupilotmega/mavlink.h"
#include <arpa/inet.h>
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

struct Stats {
    double s = 0;
    int n = 0;
    void add(double v) { s += v; ++n; }
    double mean() const { return n ? s / n : 0; }
};
struct ImuStats { Stats ax, ay, az; uint64_t t_first_us=0, t_last_us=0; };

static int conn() {
    addrinfo h{}, *r = nullptr;
    h.ai_family = AF_UNSPEC;
    h.ai_socktype = SOCK_STREAM;
    if (getaddrinfo("127.0.0.1", "5760", &h, &r)) throw std::runtime_error("getaddrinfo");
    int fd = -1;
    for (auto *p = r; p; p = p->ai_next) {
        fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (fd >= 0 && connect(fd, p->ai_addr, p->ai_addrlen) == 0) break;
        if (fd >= 0) close(fd);
        fd = -1;
    }
    freeaddrinfo(r);
    if (fd < 0) throw std::runtime_error("connect");
    return fd;
}

static void rate(int fd, uint8_t sys, uint8_t comp, uint32_t id, int hz) {
    mavlink_message_t m{};
    mavlink_msg_command_long_pack(192,191,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,
                                  (float)id,1000000.0f/hz,0,0,0,0,0);
    uint8_t b[MAVLINK_MAX_PACKET_LEN];
    auto n = mavlink_msg_to_send_buffer(b,&m);
    if (write(fd,b,n) != (ssize_t)n) throw std::runtime_error("write");
}

static void add(ImuStats& s, double ax, double ay, double az) {
    s.ax.add(ax); s.ay.add(ay); s.az.add(az);
}

static void rot321(double r, double p, double y,
                   double x, double yy, double z,
                   double& ox, double& oy, double& oz) {
    const double cr=cos(r), sr=sin(r), cp=cos(p), sp=sin(p), cy=cos(y), sy=sin(y);
    ox = cy*cp*x + (cy*sp*sr-sy*cr)*yy + (cy*sp*cr+sy*sr)*z;
    oy = sy*cp*x + (sy*sp*sr+cy*cr)*yy + (sy*sp*cr-cy*sr)*z;
    oz = -sp*x + cp*sr*yy + cp*cr*z;
}

static void report(const char* name, const ImuStats& s,
                   double ar, double ap, double ayaw,
                   double trimx, double trimy) {
    if (!s.ax.n) { std::cout << name << ": no samples\n"; return; }
    const double ax=s.ax.mean(), ay=s.ay.mean(), az=s.az.mean();
    const double rg=atan2(-ay,-az);
    const double pg=atan2(ax,sqrt(ay*ay+az*az));
    const double k=180.0/M_PI;
    std::cout << name << " n=" << s.ax.n
              << " raw=[" << ax << "," << ay << "," << az << "]"
              << " |a|=" << sqrt(ax*ax+ay*ay+az*az)
              << " gravity roll/pitch=[" << rg*k << "," << pg*k << "] deg"
              << " delta_to_ATT=[" << (rg-ar)*k << "," << (pg-ap)*k << "] deg\n";

    auto eval = [&](const char* label, double tr, double tp, bool trim_first) {
        double x1=ax,y1=ay,z1=az,N,E,D;
        if (trim_first) {
            rot321(tr,tp,0,ax,ay,az,x1,y1,z1);
            rot321(ar,ap,ayaw,x1,y1,z1,N,E,D);
        } else {
            rot321(ar,ap,ayaw,ax,ay,az,x1,y1,z1);
            rot321(tr,tp,0,x1,y1,z1,N,E,D);
        }
        D += 9.80665;
        std::cout << "  " << label << " N/E/D=[" << N << "," << E << "," << D
                  << "] |NE|=" << hypot(N,E) << " m/s^2\n";
    };
    // DataFlash cross-run audit showed the physically consistent chain is
    // R_ATT * R_TRIM * a_raw. Keep legacy variants for comparison, but make
    // the confirmed candidate explicit in live output.
    eval("BASE_ATT",0,0,true);
    eval("RATT_RTRIM_CONFIRMED",trimx,trimy,true);
    eval("LEGACY_NEGTRIM_TO_ATT",-trimx,-trimy,true);
    eval("RTRIM_RATT",trimx,trimy,false);
    eval("NEGTRIM_RATT",-trimx,-trimy,false);
}

int main() {
    try {
        int fd=conn();
        mavlink_status_t st{};
        mavlink_message_t m{};
        uint8_t b[4096], sys=0, comp=0;
        auto dl=std::chrono::steady_clock::now()+std::chrono::seconds(5);
        while(!sys && std::chrono::steady_clock::now()<dl) {
            pollfd q{fd,POLLIN,0};
            if(poll(&q,1,200)<=0) continue;
            auto n=read(fd,b,sizeof(b));
            for(ssize_t i=0;i<n;i++)
                if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&m,&st) &&
                   m.msgid==MAVLINK_MSG_ID_HEARTBEAT && m.sysid!=192) {
                    sys=m.sysid; comp=m.compid; break;
                }
        }
        if(!sys) throw std::runtime_error("heartbeat timeout");

        rate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50);
        rate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,50);
        rate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU,50);
        rate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU2,50);
        rate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU3,50);

        Stats roll,pitch,yaw;\n        uint64_t att_first_us=0, att_last_us=0;
        ImuStats hi,i1,i2,i3;
        std::cout << "Keep stand stationary: 20 s\n";
        auto end=std::chrono::steady_clock::now()+std::chrono::seconds(20);
        while(std::chrono::steady_clock::now()<end) {
            pollfd q{fd,POLLIN,0};
            if(poll(&q,1,100)<=0) continue;
            auto n=read(fd,b,sizeof(b));
            for(ssize_t j=0;j<n;j++) {
                if(!mavlink_parse_char(MAVLINK_COMM_0,b[j],&m,&st) || m.sysid!=sys) continue;
                if(m.msgid==MAVLINK_MSG_ID_ATTITUDE) {
                    mavlink_attitude_t a{}; mavlink_msg_attitude_decode(&m,&a);
                    roll.add(a.roll); pitch.add(a.pitch); yaw.add(a.yaw);
                    const uint64_t tu=uint64_t(a.time_boot_ms)*1000ULL;
                    if(!att_first_us) att_first_us=tu; att_last_us=tu;
                } else if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU) {
                    mavlink_highres_imu_t a{}; mavlink_msg_highres_imu_decode(&m,&a);
                    add(hi,a.xacc,a.yacc,a.zacc,a.time_usec);
                } else if(m.msgid==MAVLINK_MSG_ID_SCALED_IMU) {
                    mavlink_scaled_imu_t a{}; mavlink_msg_scaled_imu_decode(&m,&a);
                    add(i1,a.xacc*9.80665/1000.0,a.yacc*9.80665/1000.0,a.zacc*9.80665/1000.0);
                } else if(m.msgid==MAVLINK_MSG_ID_SCALED_IMU2) {
                    mavlink_scaled_imu2_t a{}; mavlink_msg_scaled_imu2_decode(&m,&a);
                    add(i2,a.xacc*9.80665/1000.0,a.yacc*9.80665/1000.0,a.zacc*9.80665/1000.0);
                } else if(m.msgid==MAVLINK_MSG_ID_SCALED_IMU3) {
                    mavlink_scaled_imu3_t a{}; mavlink_msg_scaled_imu3_decode(&m,&a);
                    add(i3,a.xacc*9.80665/1000.0,a.yacc*9.80665/1000.0,a.zacc*9.80665/1000.0);
                }
            }
        }
        close(fd);

        const double ar=roll.mean(), ap=pitch.mean(), ayaw=yaw.mean();
        const double trimx=-0.03428453207, trimy=-0.0220823437;
        const double k=180.0/M_PI;
        std::cout << std::fixed << std::setprecision(5);
        std::cout << "ATTITUDE n=" << roll.n << " roll/pitch/yaw=["
                  << ar*k << "," << ap*k << "," << ayaw*k << "] deg\n";
        std::cout << "ATTITUDE boot_us window=[" << att_first_us << "," << att_last_us << "]\n";
        std::cout << "HIGHRES boot_us window=[" << hi.t_first_us << "," << hi.t_last_us << "]\n";
        std::cout << "AHRS trim used roll/pitch=["
                  << trimx*k << "," << trimy*k << "] deg\n";
        report("HIGHRES_IMU",hi,ar,ap,ayaw,trimx,trimy);
        report("SCALED_IMU1",i1,ar,ap,ayaw,trimx,trimy);
        report("SCALED_IMU2",i2,ar,ap,ayaw,trimx,trimy);
        report("SCALED_IMU3",i3,ar,ap,ayaw,trimx,trimy);
    } catch(const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
