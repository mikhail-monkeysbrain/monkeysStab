#!/usr/bin/env python3
from pathlib import Path

p=Path("src/imu_dead_reckoning.hpp")
s=p.read_text()

old="""inline void update(State& s,double ax,double ay,double az,double gx,double gy,double gz,
                   double roll,double pitch,double yaw,uint64_t time_usec){"""
new="""inline void update(State& s,double ax,double ay,double az,double gx,double gy,double gz,
                   double roll,double pitch,double yaw,uint64_t time_usec,
                   bool external_zupt_allow=true){"""
if s.count(old)!=1: raise SystemExit(f"ERROR signature anchor: {s.count(old)}")
s=s.replace(old,new,1)

old="""  const bool stationary=acc_ok&&gyro_ok;
  s.diag_amag=amag; s.diag_gmag=gmag; s.diag_dt=dt;
  s.diag_acc_ok=acc_ok; s.diag_gyro_ok=gyro_ok; s.diag_stationary=stationary;
  if(!acc_ok) ++s.diag_acc_rejects;
  if(!gyro_ok) ++s.diag_gyro_rejects;
  if(stationary) ++s.stationary_samples; else s.stationary_samples=0;"""
new="""  const bool imu_stationary=acc_ok&&gyro_ok;
  const bool stationary=imu_stationary&&external_zupt_allow;
  s.diag_amag=amag; s.diag_gmag=gmag; s.diag_dt=dt;
  s.diag_acc_ok=acc_ok; s.diag_gyro_ok=gyro_ok; s.diag_stationary=imu_stationary;
  if(!acc_ok) ++s.diag_acc_rejects;
  if(!gyro_ok) ++s.diag_gyro_rejects;
  if(stationary) ++s.stationary_samples; else s.stationary_samples=0;"""
if s.count(old)!=1: raise SystemExit(f"ERROR stationary anchor: {s.count(old)}")
s=s.replace(old,new,1)
p.write_text(s)

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()
needle="imu_dr::update("
i=s.find(needle)
if i<0: raise SystemExit("ERROR imu update not found")
semi=s.find(";",i)
call=s[i:semi+1]
if "external_zupt_allow" in call: raise SystemExit("ERROR already patched")
# Append camera confirmation argument before closing call.
j=call.rfind(")")
if j<0: raise SystemExit("ERROR call close not found")
replacement=call[:j]+",\n                (imu_cam_valid && (monoNs()-imu_cam_recv_ns)>=0 &&\n                 (monoNs()-imu_cam_recv_ns)<100000000LL &&\n                 std::hypot(imu_cam_vn,imu_cam_ve)<0.01)"+call[j:]
s=s[:i]+replacement+s[semi+1:]
p.write_text(s)
print("OK: real IMU ZUPT now requires fresh WORKED5 camera-stationary confirmation")
