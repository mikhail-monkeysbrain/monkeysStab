// Standalone MAVLink IMU message probe for monkeysStab.
// Connects to the existing mavlink-router TCP endpoint and reports which
// raw/scaled IMU messages are actually present. Does not modify FC state.
#include "ardupilotmega/mavlink.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

namespace {

int connectTcp(const std::string& host, const std::string& port) {
  addrinfo hints{}, *res = nullptr;
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  const int rc = getaddrinfo(host.c_str(), port.c_str(), &hints, &res);
  if (rc != 0) throw std::runtime_error(gai_strerror(rc));
  int fd = -1;
  for (addrinfo* p = res; p; p = p->ai_next) {
    fd = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    if (fd < 0) continue;
    if (::connect(fd, p->ai_addr, p->ai_addrlen) == 0) break;
    ::close(fd);
    fd = -1;
  }
  freeaddrinfo(res);
  if (fd < 0) throw std::runtime_error("cannot connect to mavlink-router");
  return fd;
}

struct Seen {
  uint64_t count = 0;
  mavlink_message_t last{};
};

void printScaled(const char* name, const Seen& s, uint32_t id) {
  std::cout << std::left << std::setw(13) << name << ": " << std::right << std::setw(6) << s.count;
  if (!s.count) { std::cout << "\n"; return; }
  mavlink_scaled_imu_t v{};
  if (id == MAVLINK_MSG_ID_SCALED_IMU) mavlink_msg_scaled_imu_decode(&s.last, &v);
  else if (id == MAVLINK_MSG_ID_SCALED_IMU2) {
    mavlink_scaled_imu2_t q{}; mavlink_msg_scaled_imu2_decode(&s.last, &q);
    std::cout << "  acc_mg=[" << q.xacc << ',' << q.yacc << ',' << q.zacc << "]"
              << " gyro_mrad_s=[" << q.xgyro << ',' << q.ygyro << ',' << q.zgyro << "]\n"; return;
  } else {
    mavlink_scaled_imu3_t q{}; mavlink_msg_scaled_imu3_decode(&s.last, &q);
    std::cout << "  acc_mg=[" << q.xacc << ',' << q.yacc << ',' << q.zacc << "]"
              << " gyro_mrad_s=[" << q.xgyro << ',' << q.ygyro << ',' << q.zgyro << "]\n"; return;
  }
  std::cout << "  acc_mg=[" << v.xacc << ',' << v.yacc << ',' << v.zacc << "]"
            << " gyro_mrad_s=[" << v.xgyro << ',' << v.ygyro << ',' << v.zgyro << "]\n";
}

} // namespace

int main(int argc, char** argv) {
  try {
    const int seconds = argc > 1 ? std::max(1, std::atoi(argv[1])) : 10;
    const int fd = connectTcp("127.0.0.1", "5760");
    std::map<uint32_t, Seen> seen;
    mavlink_status_t status{};
    mavlink_message_t msg{};
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    std::cout << "MAVLink IMU probe: tcp://127.0.0.1:5760, " << seconds << " s\n";
    while (std::chrono::steady_clock::now() < end) {
      pollfd p{fd, POLLIN, 0};
      const int pr = ::poll(&p, 1, 250);
      if (pr < 0 && errno == EINTR) continue;
      if (pr < 0) throw std::runtime_error(std::strerror(errno));
      if (pr == 0) continue;
      uint8_t buf[4096];
      const ssize_t n = ::read(fd, buf, sizeof(buf));
      if (n <= 0) continue;
      for (ssize_t i = 0; i < n; ++i) {
        if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) continue;
        switch (msg.msgid) {
          case MAVLINK_MSG_ID_HIGHRES_IMU:
          case MAVLINK_MSG_ID_RAW_IMU:
          case MAVLINK_MSG_ID_SCALED_IMU:
          case MAVLINK_MSG_ID_SCALED_IMU2:
          case MAVLINK_MSG_ID_SCALED_IMU3:
            seen[msg.msgid].count++;
            seen[msg.msgid].last = msg;
            break;
          default: break;
        }
      }
    }
    ::close(fd);

    std::cout << "\n===== IMU MAVLINK =====\n";
    const Seen empty{};
    auto get = [&](uint32_t id) -> const Seen& { auto it=seen.find(id); return it==seen.end()?empty:it->second; };
    const Seen& hi = get(MAVLINK_MSG_ID_HIGHRES_IMU);
    std::cout << std::left << std::setw(13) << "HIGHRES_IMU" << ": " << std::right << std::setw(6) << hi.count;
    if (hi.count) {
      mavlink_highres_imu_t v{}; mavlink_msg_highres_imu_decode(&hi.last, &v);
      std::cout << "  acc_m_s2=[" << v.xacc << ',' << v.yacc << ',' << v.zacc << "]"
                << " gyro_rad_s=[" << v.xgyro << ',' << v.ygyro << ',' << v.zgyro << ']';
    }
    std::cout << "\n";

    const Seen& raw = get(MAVLINK_MSG_ID_RAW_IMU);
    std::cout << std::left << std::setw(13) << "RAW_IMU" << ": " << std::right << std::setw(6) << raw.count;
    if (raw.count) {
      mavlink_raw_imu_t v{}; mavlink_msg_raw_imu_decode(&raw.last, &v);
      std::cout << "  acc_raw=[" << v.xacc << ',' << v.yacc << ',' << v.zacc << "]"
                << " gyro_raw=[" << v.xgyro << ',' << v.ygyro << ',' << v.zgyro << ']';
    }
    std::cout << "\n";
    printScaled("SCALED_IMU",  get(MAVLINK_MSG_ID_SCALED_IMU),  MAVLINK_MSG_ID_SCALED_IMU);
    printScaled("SCALED_IMU2", get(MAVLINK_MSG_ID_SCALED_IMU2), MAVLINK_MSG_ID_SCALED_IMU2);
    printScaled("SCALED_IMU3", get(MAVLINK_MSG_ID_SCALED_IMU3), MAVLINK_MSG_ID_SCALED_IMU3);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
}
