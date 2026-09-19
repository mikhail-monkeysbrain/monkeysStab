#!/usr/bin/env python3
from pathlib import Path

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()

old=r'''    auto best=fused_v2_imu_history.begin();
    int64_t best_abs=std::llabs(best->recv_ns-cam_ns);
    for(auto it=fused_v2_imu_history.begin();it!=fused_v2_imu_history.end();++it){
      const int64_t d=std::llabs(it->recv_ns-cam_ns);
      if(d<best_abs){best=it;best_abs=d;}
    }
'''
new=r'''    // FUSED_V2_CAUSAL_CAPTURE_V1
    // Realtime-causal lookup: never use an IMU sample newer than this camera frame.
    auto best=fused_v2_imu_history.end();
    for(auto it=fused_v2_imu_history.begin();it!=fused_v2_imu_history.end();++it){
      if(it->recv_ns<=cam_ns && (best==fused_v2_imu_history.end() ||
                                it->recv_ns>best->recv_ns)){
        best=it;
      }
    }
    if(best==fused_v2_imu_history.end()) return;
'''

if "// FUSED_V2_CAUSAL_CAPTURE_V1" in s:
    print("already applied")
    raise SystemExit(0)

if old not in s:
    raise SystemExit("nearest-IMU capture block not found; source not modified")

s=s.replace(old,new,1)
s=s.replace(
    "// Snapshot nearest buffered IMU DR state for every camera frame, including",
    "// Snapshot latest causal buffered IMU DR state for every camera frame, including",
    1)
s=s.replace(
    "// mono timestamp 'ts' is the same frame clock used by production CSV.",
    "// 'now' is the monotonic frame timestamp used for causal IMU lookup.",
    1)

p.write_text(s)
print("patched",p)
print("FUSED-V2 frame capture now uses latest imu.recv_ns <= cam_ns; production unchanged")
