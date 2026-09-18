#!/usr/bin/env python3
from pathlib import Path

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()

def once(old,new,label):
    global s
    n=s.count(old)
    if n!=1:
        raise SystemExit(f"ERROR {label}: expected 1 anchor, got {n}")
    s=s.replace(old,new,1)

# FlowFc state.
once(
"""  bool imu_cam_valid=false;
""",
"""  bool imu_cam_valid=false;

  // FUSED-V1 event-driven shadow.
  uint64_t imu_cam_seq=0;
  double imu_cam_dN=0.0,imu_cam_dE=0.0,imu_cam_dt=0.0;

  uint64_t fused_v1_seen_cam_seq=0;
  uint64_t fused_v1_visual_updates=0;
  uint64_t fused_v1_imu_predictions=0;
  uint64_t fused_v1_stop_constraints=0;
  double fused_v1_n=0.0,fused_v1_e=0.0;
  double fused_v1_vn=0.0,fused_v1_ve=0.0;
  double fused_v1_vn_hist[5]{};
  double fused_v1_ve_hist[5]{};
  int fused_v1_vhist_count=0;
  int fused_v1_vhist_head=0;
  bool fused_v1_stationary=false;
  int fused_v1_stop_confirm=0;
""","FlowFc state")

# Find the real/current IMU DR update call robustly, independent of formatting.
needle="imu_dr::update("
i=s.find(needle)
if i<0:
    raise SystemExit("ERROR IMU prediction: imu_dr::update not found")
semi=s.find(";",i)
if semi<0:
    raise SystemExit("ERROR IMU prediction: update semicolon not found")
end=s.find("\n",semi)
if end<0: end=len(s)
else: end+=1

block="""      // FUSED-V1 IMU velocity prediction. Position remains WORKED5-only in V1.
      if(imu_dr_state.calibrated &&
         imu_dr_state.diag_dt>0.0 &&
         imu_dr_state.diag_dt<0.1){
        fused_v1_vn += imu_dr_state.acc_n * imu_dr_state.diag_dt;
        fused_v1_ve += imu_dr_state.acc_e * imu_dr_state.diag_dt;
        ++fused_v1_imu_predictions;
      }
"""
s=s[:end]+block+s[end:]

# Consume each unique WORKED5 event once, after camera freshness/stationary is known.
anchor="      const bool imu_stationary=imu_dr_state.diag_stationary;\n"
if s.count(anchor)!=1:
    raise SystemExit(f"ERROR visual consumer: expected 1 imu_stationary anchor, got {s.count(anchor)}")
consumer=anchor+"""
      // FUSED-V1: event-driven visual update; no Web/latest-value resampling.
      if(imu_cam_seq != fused_v1_seen_cam_seq){
        fused_v1_seen_cam_seq=imu_cam_seq;
        ++fused_v1_visual_updates;

        fused_v1_n += imu_cam_dN;
        fused_v1_e += imu_cam_dE;

        if(imu_cam_dt>0.0 && imu_cam_dt<0.2){
          const double vobs_n=imu_cam_dN/imu_cam_dt;
          const double vobs_e=imu_cam_dE/imu_cam_dt;
          fused_v1_vn_hist[fused_v1_vhist_head]=vobs_n;
          fused_v1_ve_hist[fused_v1_vhist_head]=vobs_e;
          fused_v1_vhist_head=(fused_v1_vhist_head+1)%5;
          if(fused_v1_vhist_count<5) ++fused_v1_vhist_count;

          double sn=0.0,se=0.0;
          for(int k=0;k<fused_v1_vhist_count;++k){
            sn+=fused_v1_vn_hist[k];
            se+=fused_v1_ve_hist[k];
          }
          fused_v1_vn=sn/fused_v1_vhist_count;
          fused_v1_ve=se/fused_v1_vhist_count;
        }

        const double fused_cam_speed=std::hypot(imu_cam_vn,imu_cam_ve);
        if(fused_v1_stationary){
          if(fused_cam_speed>0.010){
            fused_v1_stationary=false;
            fused_v1_stop_confirm=0;
          }
        }else{
          if(fused_cam_speed<0.005){
            ++fused_v1_stop_confirm;
            if(fused_v1_stop_confirm>=3){
              fused_v1_stationary=true;
              fused_v1_stop_confirm=3;
              ++fused_v1_stop_constraints;
            }
          }else{
            fused_v1_stop_confirm=0;
          }
        }
      }

      if(fused_v1_stationary){
        fused_v1_vn=0.0;
        fused_v1_ve=0.0;
      }
"""
s=s.replace(anchor,consumer,1)

# Publish the metric observation and sequence from the WORKED5 producer.
old="""                fc.imu_cam_vn=web_raw_vn;
                fc.imu_cam_ve=web_raw_ve;
                fc.imu_cam_recv_ns=monoNs();
                fc.imu_cam_valid=true;
"""
new="""                fc.imu_cam_vn=web_raw_vn;
                fc.imu_cam_ve=web_raw_ve;
                fc.imu_cam_dN=dN;
                fc.imu_cam_dE=dE;
                fc.imu_cam_dt=dt;
                ++fc.imu_cam_seq;
                fc.imu_cam_recv_ns=monoNs();
                fc.imu_cam_valid=true;
"""
once(old,new,"WORKED5 producer")

p.write_text(s)
print("OK: FUSED-V1 event-driven shadow applied")
