#!/usr/bin/env python3
from pathlib import Path

p=Path("tools/web_service.py")
s=p.read_text()
anchor='        "raw_of_ve":raw.get("raw_of_ve"),\n'
insert='''        "raw_of_ve":raw.get("raw_of_ve"),
        "imu_cam_vn":raw.get("imu_cam_vn"),
        "imu_cam_ve":raw.get("imu_cam_ve"),
        "imu_cam_speed":raw.get("imu_cam_speed"),
        "imu_cam_age_ms":raw.get("imu_cam_age_ms"),
        "imu_cam_fresh":bool(raw.get("imu_cam_fresh",False)),
        "imu_cam_stationary":bool(raw.get("imu_cam_stationary",False)),
        "imu_zupt_shadow":bool(raw.get("imu_zupt_shadow",False)),
        "imu_zupt_shadow_accepts":raw.get("imu_zupt_shadow_accepts",0),
        "imu_zupt_shadow_blocks":raw.get("imu_zupt_shadow_blocks",0),
'''
n=s.count(anchor)
if n!=1:
    raise SystemExit(f"ERROR web mapping anchor: expected 1, got {n}")
s=s.replace(anchor,insert,1)
p.write_text(s)
print("OK: ZUPT shadow fields mapped into /api/telemetry")
