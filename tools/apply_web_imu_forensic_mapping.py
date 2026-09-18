#!/usr/bin/env python3
from pathlib import Path

p=Path("tools/web_service.py")
s=p.read_text()
old='''        "imu_dr_stationary_samples":raw.get("imu_dr_stationary_samples",0),
        "rc_zero_event":rc_zero_event,'''

new='''        "imu_dr_stationary_samples":raw.get("imu_dr_stationary_samples",0),
        "imu_dr_amag":raw.get("imu_dr_amag"),
        "imu_dr_gmag":raw.get("imu_dr_gmag"),
        "imu_dr_dt":raw.get("imu_dr_dt"),
        "imu_dr_acc_ok":raw.get("imu_dr_acc_ok"),
        "imu_dr_gyro_ok":raw.get("imu_dr_gyro_ok"),
        "imu_dr_stationary":raw.get("imu_dr_stationary"),
        "imu_dr_acc_rejects":raw.get("imu_dr_acc_rejects",0),
        "imu_dr_gyro_rejects":raw.get("imu_dr_gyro_rejects",0),
        "imu_dr_attitude_age_ms":raw.get("imu_dr_attitude_age_ms"),
        "rc_zero_event":rc_zero_event,'''

if s.count(old)!=1:
    raise SystemExit(f"PATTERN count={s.count(old)}, expected 1")
p.write_text(s.replace(old,new,1))
print("patched tools/web_service.py")
