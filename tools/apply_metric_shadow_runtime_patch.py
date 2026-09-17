#!/usr/bin/env python3
from pathlib import Path

p = Path("src/optical_flow_mavlink.cpp")
s = p.read_text()

repls = []

repls.append((
'''#include "runtime.hpp"\n#include "mavlink_io.hpp"\n''',
'''#include "runtime.hpp"\n#include "mavlink_io.hpp"\n#include "metric_odometry_shadow.hpp"\n#include "metric_shadow_sync.hpp"\n#include "metric_shadow_range_sync.hpp"\n'''))

repls.append((
'''  std::vector<cv::Point2f> inlier_points; // current-frame RANSAC inliers for web diagnostics\n''',
'''  std::vector<cv::Point2f> inlier_points; // current-frame RANSAC inliers for web diagnostics\n  // Metric-shadow input: preserve BOTH sides of the exact production\n  // homography-RANSAC inlier correspondences. Diagnostic only.\n  std::vector<cv::Point2f> metric_prev_points;\n  std::vector<cv::Point2f> metric_curr_points;\n'''))

repls.append((
'''  o.inlier_points=bi;\n  if(ai.size()<20){ o.invalid_reason=5; return o; }\n''',
'''  o.inlier_points=bi;\n  o.metric_prev_points=ai;\n  o.metric_curr_points=bi;\n  if(ai.size()<20){ o.invalid_reason=5; return o; }\n'''))

anchor = '''        web_live.sendPreview(now,gray,s.inlier_points,g_feature_roi);\n\n        // Consume the FC gyro for THIS processed camera interval before any\n'''
insert = '''        web_live.sendPreview(now,gray,s.inlier_points,g_feature_roi);\n\n        // Metric odometry shadow. This path is diagnostic only: it consumes\n        // the exact production RANSAC correspondences but never changes\n        // flow_send_x/y and never calls sendOpticalFlow().\n        static metric_shadow::Integrator metric_shadow_integrator;\n        static uint64_t metric_shadow_interval_id=0;\n        static int64_t metric_shadow_last_print_ns=0;\n        metric_shadow::Step metric_step;\n        bool metric_attempted=false;\n        double metric_att_gap0_ms=-1.0,metric_att_gap1_ms=-1.0;\n        double metric_range_gap0_ms=-1.0,metric_range_gap1_ms=-1.0;\n        if(!prev.empty() && prev_ts>0 && ts>prev_ts){\n          metric_attempted=true;\n          ++metric_shadow_interval_id;\n\n          std::deque<metric_shadow::TimedAttitude> ah;\n          {\n            std::lock_guard<std::mutex> lock(fc.mu);\n            ah.clear();\n            ah.resize(fc.attitude_history.size());\n            for(size_t i=0;i<fc.attitude_history.size();++i){\n              const auto& g=fc.attitude_history[i];\n              ah[i]={g.roll,g.pitch,g.yaw,g.sample_ns,g.valid};\n            }\n          }\n          const auto a0=metric_shadow::interpolateAttitude(ah,prev_ts,30.0);\n          const auto a1=metric_shadow::interpolateAttitude(ah,ts,30.0);\n          metric_att_gap0_ms=a0.bracket_gap_ms;\n          metric_att_gap1_ms=a1.bracket_gap_ms;\n\n          const auto rh=luna.historySnapshot();\n          const auto r0=metric_shadow_sync::interpolateRange(rh,prev_ts,40.0,25.0);\n          const auto r1=metric_shadow_sync::interpolateRange(rh,ts,40.0,25.0);\n          metric_range_gap0_ms=r0.bracket_gap_ms;\n          metric_range_gap1_ms=r1.bracket_gap_ms;\n\n          metric_shadow::Input mi;\n          mi.t0_ns=prev_ts; mi.t1_ns=ts;\n          mi.px0=s.metric_prev_points; mi.px1=s.metric_curr_points;\n          mi.K=calib.K; mi.D=calib.D;\n          if(a0.valid) mi.a0=a0.attitude;\n          if(a1.valid) mi.a1=a1.attitude;\n          mi.range0_m=r0.distance_m; mi.range1_m=r1.distance_m;\n          mi.range0_valid=r0.valid; mi.range1_valid=r1.valid;\n          mi.body_R_camera_frd=calib.B_R_C;\n          mi.body_R_camera_valid=true;\n          mi.camera_pos_body_frd=cv::Vec3d(diag_camera_x_m,diag_camera_y_m,diag_camera_z_m);\n          mi.range_pos_body_frd=cv::Vec3d(diag_range_x_m,diag_range_y_m,diag_range_z_m);\n\n          metric_step=metric_shadow::estimate(mi);\n          metric_shadow_integrator.consume(metric_step);\n\n          if(metric_shadow_last_print_ns==0 || now-metric_shadow_last_print_ns>=500000000LL){\n            metric_shadow_last_print_ns=now;\n            const auto& mp=metric_shadow_integrator.position_m;\n            std::cerr<<"METRIC_SHADOW interval="<<metric_shadow_interval_id\n                     <<" valid="<<(metric_step.valid?1:0)\n                     <<" reason="<<metric_shadow::rejectReasonName(metric_step.reason)\n                     <<" pairs="<<s.metric_prev_points.size()\n                     <<" used="<<metric_step.points\n                     <<" dNE_mm=["<<metric_step.delta_local_m[0]*1000.0\n                     <<","<<metric_step.delta_local_m[1]*1000.0<<"]"\n                     <<" posNE_mm=["<<mp[0]*1000.0<<","<<mp[1]*1000.0<<"]"\n                     <<" complete="<<(metric_shadow_integrator.complete?1:0)\n                     <<" accepted="<<metric_shadow_integrator.accepted\n                     <<" rejected="<<metric_shadow_integrator.rejected\n                     <<" att_gap_ms=["<<metric_att_gap0_ms<<","<<metric_att_gap1_ms<<"]"\n                     <<" range_gap_ms=["<<metric_range_gap0_ms<<","<<metric_range_gap1_ms<<"]"\n                     <<" residual_med_mm="<<metric_step.residual_median_m*1000.0\n                     <<"\\n";\n          }\n        }\n\n        // Consume the FC gyro for THIS processed camera interval before any\n'''
repls.append((anchor, insert))

for old, new in repls:
    n = s.count(old)
    if n != 1:
        raise SystemExit(f"PATCH ABORT: expected exactly one anchor, got {n}: {old[:80]!r}")
    s = s.replace(old, new, 1)

p.write_text(s)
print("PATCH OK: src/optical_flow_mavlink.cpp")
print("Added metric correspondences + synchronized diagnostic Metric Shadow; OPTICAL_FLOW send path untouched.")
