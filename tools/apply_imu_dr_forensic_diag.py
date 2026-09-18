#!/usr/bin/env python3
from pathlib import Path

def replace_once(path, old, new):
    p=Path(path); s=p.read_text()
    if old not in s: raise SystemExit(f"PATTERN NOT FOUND: {path}")
    if s.count(old)!=1: raise SystemExit(f"PATTERN NOT UNIQUE ({s.count(old)}): {path}")
    p.write_text(s.replace(old,new,1))
    print("patched",path)

replace_once("src/imu_dead_reckoning.hpp",
"""  int stationary_samples=0;
  uint64_t last_time_usec=0;""",
"""  int stationary_samples=0;
  double diag_amag=0,diag_gmag=0,diag_dt=0;
  bool diag_acc_ok=false,diag_gyro_ok=false,diag_stationary=false;
  uint64_t diag_acc_rejects=0,diag_gyro_rejects=0;
  uint64_t last_time_usec=0;""")

replace_once("src/imu_dead_reckoning.hpp",
"""  const double gmag=std::sqrt(gx*gx+gy*gy+gz*gz);
  const bool stationary=(amag<0.12&&gmag<0.02);
  if(stationary) ++s.stationary_samples; else s.stationary_samples=0;""",
"""  const double gmag=std::sqrt(gx*gx+gy*gy+gz*gz);
  const bool acc_ok=amag<0.12;
  const bool gyro_ok=gmag<0.02;
  const bool stationary=acc_ok&&gyro_ok;
  s.diag_amag=amag; s.diag_gmag=gmag; s.diag_dt=dt;
  s.diag_acc_ok=acc_ok; s.diag_gyro_ok=gyro_ok; s.diag_stationary=stationary;
  if(!acc_ok) ++s.diag_acc_rejects;
  if(!gyro_ok) ++s.diag_gyro_rejects;
  if(stationary) ++s.stationary_samples; else s.stationary_samples=0;""")

replace_once("src/optical_flow_mavlink.cpp",
"""              if(gyro.valid) imu_dr::update(imu_dr_state,q.xacc,q.yacc,q.zacc,q.xgyro,q.ygyro,q.zgyro,gyro.roll,gyro.pitch,gyro.yaw,q.time_usec);""",
"""              if(gyro.valid) {
                imu_dr::update(imu_dr_state,q.xacc,q.yacc,q.zacc,q.xgyro,q.ygyro,q.zgyro,
                               gyro.roll,gyro.pitch,gyro.yaw,q.time_usec);
              }""")

replace_once("src/optical_flow_mavlink.cpp",
"""            <<",\\\"imu_dr_stationary_samples\\\":"<<fc.imu_dr_state.stationary_samples
            <<",\\\"rc_zero_seq\\\":"<<rc_zero_seq""",
"""            <<",\\\"imu_dr_stationary_samples\\\":"<<fc.imu_dr_state.stationary_samples
            <<",\\\"imu_dr_amag\\\":"<<jsonNumber(fc.imu_dr_state.diag_amag)
            <<",\\\"imu_dr_gmag\\\":"<<jsonNumber(fc.imu_dr_state.diag_gmag)
            <<",\\\"imu_dr_dt\\\":"<<jsonNumber(fc.imu_dr_state.diag_dt)
            <<",\\\"imu_dr_acc_ok\\\":"<<(fc.imu_dr_state.diag_acc_ok?"true":"false")
            <<",\\\"imu_dr_gyro_ok\\\":"<<(fc.imu_dr_state.diag_gyro_ok?"true":"false")
            <<",\\\"imu_dr_stationary\\\":"<<(fc.imu_dr_state.diag_stationary?"true":"false")
            <<",\\\"imu_dr_acc_rejects\\\":"<<fc.imu_dr_state.diag_acc_rejects
            <<",\\\"imu_dr_gyro_rejects\\\":"<<fc.imu_dr_state.diag_gyro_rejects
            <<",\\\"imu_dr_attitude_age_ms\\\":"<<jsonNumber(fc.gyro.valid?(monoNs()-fc.gyro.recv_ns)*1e-6:-1.0)
            <<",\\\"rc_zero_seq\\\":"<<rc_zero_seq""")
print("OK")
