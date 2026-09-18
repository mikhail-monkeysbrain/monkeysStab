#!/usr/bin/env python3
from pathlib import Path
import sys

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()

def once(old,new,label):
    global s
    n=s.count(old)
    if n!=1:
        raise SystemExit(f"ERROR {label}: expected 1 anchor, got {n}")
    s=s.replace(old,new,1)

once(
"  imu_dr::State imu_dr_state{};\n",
"""  imu_dr::State imu_dr_state{};
  // Shadow-only camera gate for IMU ZUPT diagnostics.
  double imu_cam_vn=0.0,imu_cam_ve=0.0;
  int64_t imu_cam_recv_ns=0;
  bool imu_cam_valid=false;
  bool imu_zupt_cam_fresh=false;
  bool imu_zupt_cam_stationary=false;
  bool imu_zupt_shadow=false;
  uint64_t imu_zupt_shadow_accepts=0;
  uint64_t imu_zupt_shadow_blocks=0;
""","FlowFc state")

once(
"              web_raw_step_valid=true;\n",
"""              web_raw_step_valid=true;
              {
                std::lock_guard<std::mutex> l(fc.mu);
                fc.imu_cam_vn=web_raw_vn;
                fc.imu_cam_ve=web_raw_ve;
                fc.imu_cam_recv_ns=monoNs();
                fc.imu_cam_valid=true;
              }
""","WORKED5 publish")

# Insert shadow logic immediately after the local imu_dr::update statement,
# regardless of its argument formatting.
needle="imu_dr::update("
i=s.find(needle)
if i<0: raise SystemExit("ERROR imu update: anchor not found")
line0=s.rfind("\n",0,i)+1
semi=s.find(";",i)
if semi<0: raise SystemExit("ERROR imu update: semicolon not found")
end=s.find("\n",semi)+1
if end<=0: end=len(s)
block="""              const int64_t zupt_now_ns=monoNs();
              const double cam_age_ms=imu_cam_valid
                ? (zupt_now_ns-imu_cam_recv_ns)*1e-6 : 1e9;
              imu_zupt_cam_fresh=imu_cam_valid && cam_age_ms>=0.0 && cam_age_ms<100.0;
              const double cam_speed=std::hypot(imu_cam_vn,imu_cam_ve);
              imu_zupt_cam_stationary=imu_zupt_cam_fresh && cam_speed<0.01;
              const bool imu_stationary=imu_dr_state.diag_stationary;
              imu_zupt_shadow=imu_stationary && imu_zupt_cam_stationary;
              if(imu_stationary){
                if(imu_zupt_shadow) ++imu_zupt_shadow_accepts;
                else ++imu_zupt_shadow_blocks;
              }
"""
s=s[:end]+block+s[end:]

once(
'            <<",\\\"imu_dr_gyro_rejects\\\":"<<fc.imu_dr_state.diag_gyro_rejects\n',
'''            <<",\\\"imu_dr_gyro_rejects\\\":"<<fc.imu_dr_state.diag_gyro_rejects
            <<",\\\"imu_cam_vn\\\":"<<jsonNumber(fc.imu_cam_vn)
            <<",\\\"imu_cam_ve\\\":"<<jsonNumber(fc.imu_cam_ve)
            <<",\\\"imu_cam_speed\\\":"<<jsonNumber(std::hypot(fc.imu_cam_vn,fc.imu_cam_ve))
            <<",\\\"imu_cam_age_ms\\\":"<<jsonNumber(fc.imu_cam_valid?(monoNs()-fc.imu_cam_recv_ns)*1e-6:-1.0)
            <<",\\\"imu_cam_fresh\\\":"<<(fc.imu_zupt_cam_fresh?"true":"false")
            <<",\\\"imu_cam_stationary\\\":"<<(fc.imu_zupt_cam_stationary?"true":"false")
            <<",\\\"imu_zupt_shadow\\\":"<<(fc.imu_zupt_shadow?"true":"false")
            <<",\\\"imu_zupt_shadow_accepts\\\":"<<fc.imu_zupt_shadow_accepts
            <<",\\\"imu_zupt_shadow_blocks\\\":"<<fc.imu_zupt_shadow_blocks
''',"web json")

p.write_text(s)
print("OK: shadow camera-gated ZUPT diagnostic applied")
