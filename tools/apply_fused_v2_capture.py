#!/usr/bin/env python3
from pathlib import Path

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()
marker="// FUSED_V2_CAPTURE_V1"
if marker in s:
    print("already applied")
    raise SystemExit(0)

anchor='''  // FUSED-V1 event-driven shadow.
  uint64_t imu_cam_seq=0;
  double imu_cam_dN=0.0,imu_cam_dE=0.0,imu_cam_dt=0.0;
'''
if anchor not in s:
    raise SystemExit("state anchor not found; source not modified")
state=anchor+'''
  // FUSED_V2_CAPTURE_V1
  // Diagnostic-only time-aligned IMU history for a future V2 bridge.
  struct FusedV2ImuSample {
    int64_t recv_ns=0;
    double pos_n=0.0,pos_e=0.0;
    double vel_n=0.0,vel_e=0.0;
  };
  std::deque<FusedV2ImuSample> fused_v2_imu_history;
'''
s=s.replace(anchor,state,1)

imu_anchor='''        ++fused_v1_imu_predictions;
      }
'''
if imu_anchor not in s:
    raise SystemExit("IMU anchor not found; source not modified")
imu=imu_anchor+'''      // FUSED_V2_CAPTURE_V1: keep 500 ms of the exact DR state, keyed
      // by RPi monotonic receive time. No production state is modified.
      if(imu_dr_state.calibrated){
        const int64_t v2_now_ns=imu.recv_ns;
        fused_v2_imu_history.push_back({
          v2_now_ns,
          imu_dr_state.pos_n,imu_dr_state.pos_e,
          imu_dr_state.vel_n,imu_dr_state.vel_e});
        while(!fused_v2_imu_history.empty() &&
              v2_now_ns-fused_v2_imu_history.front().recv_ns>500000000LL)
          fused_v2_imu_history.pop_front();
      }
'''
s=s.replace(imu_anchor,imu,1)

cam_anchor='''                fc.imu_cam_valid=true;

                // FUSED-V1 visual update happens HERE, once per unique WORKED5
'''
if cam_anchor not in s:
    raise SystemExit("camera anchor not found; source not modified")
cam='''                fc.imu_cam_valid=true;

                // FUSED_V2_CAPTURE_V1
                // Capture-only probe: nearest IMU state to this visual event.
                // This intentionally does NOT alter WORKED5/FUSED-V1.
                if(!fc.fused_v2_imu_history.empty()){
                  const int64_t v2_cam_ns=fc.imu_cam_recv_ns;
                  auto best=fc.fused_v2_imu_history.begin();
                  int64_t best_abs=std::llabs(best->recv_ns-v2_cam_ns);
                  for(auto it=fc.fused_v2_imu_history.begin();it!=fc.fused_v2_imu_history.end();++it){
                    const int64_t d=std::llabs(it->recv_ns-v2_cam_ns);
                    if(d<best_abs){best=it;best_abs=d;}
                  }
                  static std::ofstream v2_capture_csv;
                  static bool v2_capture_header=false;
                  if(!v2_capture_csv.is_open()){
                    const std::filesystem::path production_csv_path(csvpath);
                    v2_capture_csv.open(production_csv_path.parent_path()/"fused_v2_capture.csv",
                                        std::ios::out|std::ios::trunc);
                  }
                  if(v2_capture_csv.is_open()){
                    if(!v2_capture_header){
                      v2_capture_csv<<"cam_seq,cam_recv_ns,imu_recv_ns,age_ms,imu_n_m,imu_e_m,imu_vn,imu_ve,dN_m,dE_m,dt_s\\n";
                      v2_capture_header=true;
                    }
                    v2_capture_csv<<fc.imu_cam_seq<<','<<v2_cam_ns<<','<<best->recv_ns<<','
                      <<(v2_cam_ns-best->recv_ns)*1e-6<<','
                      <<best->pos_n<<','<<best->pos_e<<','<<best->vel_n<<','<<best->vel_e<<','
                      <<dN<<','<<dE<<','<<dt<<'\\n';
                    v2_capture_csv.flush();
                  }
                }

                // FUSED-V1 visual update happens HERE, once per unique WORKED5
'''
s=s.replace(cam_anchor,cam,1)
p.write_text(s)
print("patched",p)
print("added FUSED_V2_CAPTURE_V1; production WORKED5/FUSED-V1 unchanged")
