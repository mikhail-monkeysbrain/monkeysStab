#!/usr/bin/env python3
from pathlib import Path

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()

marker='"fused_v1_visual_updates"'
if marker in s:
    print("OK: FUSED-V1 telemetry already present")
    raise SystemExit(0)

anchor='            <<",\\\"imu_zupt_shadow_blocks\\\":"<<fc.imu_zupt_shadow_blocks\n'
if s.count(anchor)!=1:
    raise SystemExit(f"ERROR: telemetry anchor count={s.count(anchor)}")

new='''            <<",\\\"imu_zupt_shadow_blocks\\\":"<<fc.imu_zupt_shadow_blocks
            <<",\\\"imu_cam_seq\\\":"<<fc.imu_cam_seq
            <<",\\\"fused_v1_visual_updates\\\":"<<fc.fused_v1_visual_updates
            <<",\\\"fused_v1_imu_predictions\\\":"<<fc.fused_v1_imu_predictions
            <<",\\\"fused_v1_stop_constraints\\\":"<<fc.fused_v1_stop_constraints
            <<",\\\"fused_v1_stationary\\\":"<<(fc.fused_v1_stationary?"true":"false")
            <<",\\\"fused_v1_stop_confirm\\\":"<<fc.fused_v1_stop_confirm
            <<",\\\"fused_v1_n_mm\\\":"<<jsonNumber(fc.fused_v1_n*1000.0)
            <<",\\\"fused_v1_e_mm\\\":"<<jsonNumber(fc.fused_v1_e*1000.0)
            <<",\\\"fused_v1_vn\\\":"<<jsonNumber(fc.fused_v1_vn)
            <<",\\\"fused_v1_ve\\\":"<<jsonNumber(fc.fused_v1_ve)
'''
s=s.replace(anchor,new,1)
p.write_text(s)
print("OK: FUSED-V1 telemetry-only patch applied")
