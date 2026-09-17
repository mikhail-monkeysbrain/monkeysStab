#!/usr/bin/env python3
from pathlib import Path

p = Path("src/optical_flow_mavlink.cpp")
s = p.read_text()

inc_old = '#include "metric_shadow_range_sync.hpp"\n'
inc_new = inc_old + '#include "worked5_estimator.hpp"\n'
if '#include "worked5_estimator.hpp"' not in s:
    if inc_old not in s:
        raise SystemExit("include anchor not found")
    s = s.replace(inc_old, inc_new, 1)

old = '''        // Independent RAW Optical Flow diagnostic for Web UI.
        // Use only accepted flow intervals and the physical camera height.
        // This is intentionally diagnostic-only and does not alter publisher/EKF.
        web_raw_step_valid=false;
        web_raw_vn=web_raw_ve=0.0;
        if(s.valid && flow_sent && fg_ok && dt>0.0 && dt<0.2){
          double hcam=0.0;
          if(bench_true_camera_height>0.0){
            hcam=bench_true_camera_height;
          } else if(current_camera_height_valid){
            hcam=current_camera_height_m;
          }
          if(hcam>0.02 && std::isfinite(hcam)){
            // Diagnostic RAW must integrate the exact same production flow
            // that is sent to ArduPilot.  In particular, include the production
            // lever-arm correction so EKF-vs-RAW compares only the downstream
            // integration/fusion paths, not two different OF estimators.
            const double production_fx=flow_send_x;
            const double production_fy=flow_send_y;
            const double comp_x=-production_fx + fg.x;
            const double comp_y=-production_fy + fg.y;
            const double vbx=(-comp_y)*hcam;
            const double vby=( comp_x)*hcam;

            const double cr=std::cos(fg.roll),  sr=std::sin(fg.roll);
            const double cp=std::cos(fg.pitch), sp=std::sin(fg.pitch);
            const double cy=std::cos(fg.yaw),   sy=std::sin(fg.yaw);
            const double r00=cy*cp;
            const double r01=cy*sp*sr-sy*cr;
            const double r10=sy*cp;
            const double r11=sy*sp*sr+cy*cr;

            web_raw_vn=r00*vbx + r01*vby;
            web_raw_ve=r10*vbx + r11*vby;
            web_raw_n += web_raw_vn*dt;
            web_raw_e += web_raw_ve*dt;
            web_raw_step_valid=true;
          }
        }
'''

new = '''        // WORKED 5% closure estimator for Web UI / HOME return.
        // The estimator itself is frozen in worked5_estimator.hpp.  This layer
        // only maps its metric camera-plane delta into the existing N/E HOME
        // coordinate system.  The ArduPilot OPTICAL_FLOW publisher above is
        // intentionally untouched and remains an independent comparison path.
        web_raw_step_valid=false;
        web_raw_vn=web_raw_ve=0.0;
        if(s.valid && fg_ok && dt>0.0 && dt<0.2){
          double hcam=0.0;
          if(bench_true_camera_height>0.0){
            hcam=bench_true_camera_height;
          } else if(current_camera_height_valid){
            hcam=current_camera_height_m;
          }
          if(hcam>0.02 && std::isfinite(hcam)){
            const auto w5=worked5::estimate(
              s.metric_prev_points,s.metric_curr_points,
              calib.K,focal_scale,calib.D,hcam,dt);
            if(w5.valid){
              // Frozen blind convention gives a metric displacement in the
              // camera/body horizontal plane: X=+dv*H, Y=-du*H.  Rotate that
              // already-metric delta into NED using the FC attitude.  No extra
              // empirical scale, axis correction or gyro subtraction is added.
              const double cr=std::cos(fg.roll),  sr=std::sin(fg.roll);
              const double cp=std::cos(fg.pitch), sp=std::sin(fg.pitch);
              const double cy=std::cos(fg.yaw),   sy=std::sin(fg.yaw);
              const double r00=cy*cp;
              const double r01=cy*sp*sr-sy*cr;
              const double r10=sy*cp;
              const double r11=sy*sp*sr+cy*cr;

              const double dN=r00*w5.dx_m + r01*w5.dy_m;
              const double dE=r10*w5.dx_m + r11*w5.dy_m;
              web_raw_n += dN;
              web_raw_e += dE;
              web_raw_vn=dN/dt;
              web_raw_ve=dE/dt;
              web_raw_step_valid=true;
            }
          }
        }
'''

if new not in s:
    if old not in s:
        raise SystemExit("web_raw integration anchor not found")
    s = s.replace(old, new, 1)

p.write_text(s)
print("WORKED5 closure integration applied to", p)
