#!/usr/bin/env python3
"""Add diagnostic WORKED5 magnitude-gate A/B shadow to the local production source.

The production path is untouched. Shadow B runs the frozen worked5::estimate()
on the exact production RANSAC correspondences whenever >=20 pairs exist,
including invalid_reason=6 frames rejected only by the mag<4 gate.
Writes worked5_mag_shadow.csv next to the production CSV.
"""
from pathlib import Path
import sys

p = Path("src/optical_flow_mavlink.cpp")
s = p.read_text()

marker = "// WORKED5_MAG_SHADOW_V1"
if marker in s:
    print("already patched:", p)
    raise SystemExit(0)

anchor = """        FlowFcGyro fg{}; double fg_age=1e9; uint64_t fg_samples=0;
        const bool fg_ok=fc.consumeGyroAverage(&fg,&fg_age,&fg_samples);

        // Production lever-arm candidate."""
if s.count(anchor) != 1:
    print(f"expected exactly one fg_ok anchor, found {s.count(anchor)}; source not modified", file=sys.stderr)
    raise SystemExit(2)

insert = """        FlowFcGyro fg{}; double fg_age=1e9; uint64_t fg_samples=0;
        const bool fg_ok=fc.consumeGyroAverage(&fg,&fg_age,&fg_samples);

        // WORKED5_MAG_SHADOW_V1
        // Diagnostic B arm only. Production s.valid, MAVLink and WORKED5-A are untouched.
        // Reuse the exact production RANSAC correspondences before the mag<4 gate.
        {
          static double mag_shadow_n=0.0, mag_shadow_e=0.0;
          static uint64_t mag_shadow_attempts=0, mag_shadow_valid=0;
          static std::ofstream mag_shadow_csv;
          static bool mag_shadow_header=false;

          if(!mag_shadow_csv.is_open()){
            const std::filesystem::path production_csv_path(csvpath);
            mag_shadow_csv.open(
              production_csv_path.parent_path() / "worked5_mag_shadow.csv",
              std::ios::out | std::ios::trunc);
          }
          if(mag_shadow_csv.is_open() && !mag_shadow_header){
            mag_shadow_csv
              <<"frame,mono_ns,production_valid,invalid_reason,pairs,dt_s,hcam_m,"
              <<"shadow_attempted,shadow_valid,dN_m,dE_m,pN_m,pE_m,"
              <<"du_norm,dv_norm,dx_m,dy_m\\n";
            mag_shadow_header=true;
          }

          double shadow_hcam=0.0;
          if(bench_true_camera_height>0.0) shadow_hcam=bench_true_camera_height;
          else if(current_camera_height_valid) shadow_hcam=current_camera_height_m;

          const int shadow_pairs=(int)std::min(
            s.metric_prev_points.size(),s.metric_curr_points.size());
          const bool shadow_attempt =
            fg_ok && dt>0.0 && dt<0.2 &&
            shadow_hcam>0.02 && std::isfinite(shadow_hcam) &&
            shadow_pairs>=20;

          bool shadow_valid=false;
          double shadow_dN=0.0,shadow_dE=0.0;
          worked5::Result shadow_w5{};
          if(shadow_attempt){
            ++mag_shadow_attempts;
            shadow_w5=worked5::estimate(
              s.metric_prev_points,s.metric_curr_points,
              calib.K,focal_scale,calib.D,shadow_hcam,dt);
            if(shadow_w5.valid){
              const double cr=std::cos(fg.roll),  sr=std::sin(fg.roll);
              const double cp=std::cos(fg.pitch), sp=std::sin(fg.pitch);
              const double cy=std::cos(fg.yaw),   sy=std::sin(fg.yaw);
              const double r00=cy*cp;
              const double r01=cy*sp*sr-sy*cr;
              const double r10=sy*cp;
              const double r11=sy*sp*sr+cy*cr;
              shadow_dN=r00*shadow_w5.dx_m+r01*shadow_w5.dy_m;
              shadow_dE=r10*shadow_w5.dx_m+r11*shadow_w5.dy_m;
              if(std::isfinite(shadow_dN) && std::isfinite(shadow_dE)){
                shadow_valid=true;
                ++mag_shadow_valid;
                mag_shadow_n+=shadow_dN;
                mag_shadow_e+=shadow_dE;
              }
            }
          }

          if(mag_shadow_csv.is_open()){
            mag_shadow_csv
              <<frame<<','<<ts<<','<<(s.valid?1:0)<<','<<s.invalid_reason<<','
              <<shadow_pairs<<','<<dt<<','<<shadow_hcam<<','
              <<(shadow_attempt?1:0)<<','<<(shadow_valid?1:0)<<','
              <<shadow_dN<<','<<shadow_dE<<','
              <<mag_shadow_n<<','<<mag_shadow_e<<','
              <<shadow_w5.du_norm<<','<<shadow_w5.dv_norm<<','
              <<shadow_w5.dx_m<<','<<shadow_w5.dy_m<<'\\n';
            mag_shadow_csv.flush();
          }
        }

        // Production lever-arm candidate."""

s = s.replace(anchor, insert, 1)
p.write_text(s)
print("patched", p)
print("added WORKED5_MAG_SHADOW_V1 after fg_ok; production path unchanged")
