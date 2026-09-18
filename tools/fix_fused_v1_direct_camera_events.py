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

# Remove the IMU-side latest-event consumer. It loses WORKED5 observations when
# camera events arrive faster than HIGHRES_IMU.
start='      // FUSED-V1: event-driven visual update; no Web/latest-value resampling.\n'
end='      if(fused_v1_stationary){\n        fused_v1_vn=0.0;\n        fused_v1_ve=0.0;\n      }\n'
i=s.find(start)
if i<0:
    raise SystemExit("ERROR: old FUSED-V1 visual consumer not found")
j=s.find(end,i)
if j<0:
    raise SystemExit("ERROR: old FUSED-V1 consumer end not found")
j+=len(end)
replacement='''      // FUSED-V1 horizontal stop constraint is maintained by unique
      // WORKED5 events in the camera producer below.
      if(fused_v1_stationary){
        fused_v1_vn=0.0;
        fused_v1_ve=0.0;
      }
'''
s=s[:i]+replacement+s[j:]

# Replace producer payload with direct event-driven fusion under the same fc.mu.
old='''                fc.imu_cam_vn=web_raw_vn;
                fc.imu_cam_ve=web_raw_ve;
                fc.imu_cam_dN=dN;
                fc.imu_cam_dE=dE;
                fc.imu_cam_dt=dt;
                ++fc.imu_cam_seq;
                fc.imu_cam_recv_ns=monoNs();
                fc.imu_cam_valid=true;
'''
new='''                fc.imu_cam_vn=web_raw_vn;
                fc.imu_cam_ve=web_raw_ve;
                fc.imu_cam_dN=dN;
                fc.imu_cam_dE=dE;
                fc.imu_cam_dt=dt;
                ++fc.imu_cam_seq;
                fc.imu_cam_recv_ns=monoNs();
                fc.imu_cam_valid=true;

                // FUSED-V1 visual update happens HERE, once per unique WORKED5
                // observation. No latest-value mailbox is consumed by IMU.
                ++fc.fused_v1_visual_updates;
                fc.fused_v1_seen_cam_seq=fc.imu_cam_seq;
                fc.fused_v1_n += dN;
                fc.fused_v1_e += dE;

                const double vobs_n=dN/dt;
                const double vobs_e=dE/dt;
                fc.fused_v1_vn_hist[fc.fused_v1_vhist_head]=vobs_n;
                fc.fused_v1_ve_hist[fc.fused_v1_vhist_head]=vobs_e;
                fc.fused_v1_vhist_head=(fc.fused_v1_vhist_head+1)%5;
                if(fc.fused_v1_vhist_count<5) ++fc.fused_v1_vhist_count;

                double fused_sn=0.0,fused_se=0.0;
                for(int k=0;k<fc.fused_v1_vhist_count;++k){
                  fused_sn+=fc.fused_v1_vn_hist[k];
                  fused_se+=fc.fused_v1_ve_hist[k];
                }

                const double fused_cam_speed=std::hypot(web_raw_vn,web_raw_ve);
                if(fc.fused_v1_stationary){
                  if(fused_cam_speed>0.010){
                    fc.fused_v1_stationary=false;
                    fc.fused_v1_stop_confirm=0;
                  }
                }else{
                  if(fused_cam_speed<0.005){
                    ++fc.fused_v1_stop_confirm;
                    if(fc.fused_v1_stop_confirm>=3){
                      fc.fused_v1_stationary=true;
                      fc.fused_v1_stop_confirm=3;
                      ++fc.fused_v1_stop_constraints;
                    }
                  }else{
                    fc.fused_v1_stop_confirm=0;
                  }
                }

                if(fc.fused_v1_stationary){
                  fc.fused_v1_vn=0.0;
                  fc.fused_v1_ve=0.0;
                }else{
                  fc.fused_v1_vn=fused_sn/fc.fused_v1_vhist_count;
                  fc.fused_v1_ve=fused_se/fc.fused_v1_vhist_count;
                }
'''
once(old,new,"WORKED5 direct FUSED-V1 producer")

p.write_text(s)
print("OK: FUSED-V1 visual fusion moved directly into WORKED5 producer")
