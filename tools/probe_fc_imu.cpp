// Standalone MAVLink IMU message probe for monkeysStab.
// Connects to the existing mavlink-router TCP endpoint, requests IMU streams
// temporarily with MAV_CMD_SET_MESSAGE_INTERVAL, and reports what FC provides.
// It does not change persistent ArduPilot parameters.
#include "ardupilotmega/mavlink.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cstdlib>
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

void sendMessage(int fd, const mavlink_message_t& msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::write(fd, buf + off, len - off);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) throw std::runtime_error("MAVLink write failed");
    off += static_cast<size_t>(n);
  }
}

void requestInterval(int fd, uint8_t target_sys, uint8_t target_comp,
                     uint32_t msgid, float hz) {
  mavlink_message_t out{};
  const float interval_us = hz > 0.0f ? 1000000.0f / hz : -1.0f;
  mavlink_msg_command_long_pack(
      192, 191, &out,
      target_sys, target_comp,
      MAV_CMD_SET_MESSAGE_INTERVAL, 0,
      static_cast<float>(msgid), interval_us,
      0, 0, 0, 0, 0);
  sendMessage(fd, out);
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
    uint8_t target_sys = 0, target_comp = 0;

    std::cout << "MAVLink IMU probe: tcp://127.0.0.1:5760\n";
    std::cout << "Waiting for FC heartbeat...\n";
    const auto hb_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < hb_deadline && !target_sys) {
      pollfd p{fd, POLLIN, 0};
      if (::poll(&p, 1, 250) <= 0) continue;
      uint8_t buf[4096];
      const ssize_t n = ::read(fd, buf, sizeof(buf));
      if (n <= 0) continue;
      for (ssize_t i = 0; i < n; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status) &&
            msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && msg.sysid != 192) {
          target_sys = msg.sysid;
          target_comp = msg.compid;
          break;
        }
      }
    }
    if (!target_sys) throw std::runtime_error("FC heartbeat not found");
    std::cout << "FC: sysid=" << unsigned(target_sys) << " compid=" << unsigned(target_comp) << "\n";
    std::cout << "Requesting IMU messages at 50 Hz (temporary MAVLink interval request)...\n";

    requestInterval(fd, target_sys, target_comp, MAVLINK_MSG_ID_HIGHRES_IMU, 50.0f);
    requestInterval(fd, target_sys, target_comp, MAVLINK_MSG_ID_RAW_IMU, 50.0f);
    requestInterval(fd, target_sys, target_comp, MAVLINK_MSG_ID_SCALED_IMU, 50.0f);
    requestInterval(fd, target_sys, target_comp, MAVLINK_MSG_ID_SCALED_IMU2, 50.0f);
    requestInterval(fd, target_sys, target_comp, MAVLINK_MSG_ID_SCALED_IMU3, 50.0f);

    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
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
