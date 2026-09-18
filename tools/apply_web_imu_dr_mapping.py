#!/usr/bin/env python3
from pathlib import Path

p=Path("tools/web_service.py")
s=p.read_text()

fields='''        "imu_dr_n_mm":raw.get("imu_dr_n_mm"),
        "imu_dr_e_mm":raw.get("imu_dr_e_mm"),
        "imu_dr_d_mm":raw.get("imu_dr_d_mm"),
        "imu_dr_vn":raw.get("imu_dr_vn"),
        "imu_dr_ve":raw.get("imu_dr_ve"),
        "imu_dr_vd":raw.get("imu_dr_vd"),
        "imu_dr_stationary":bool(raw.get("imu_dr_stationary",False)),
        "imu_dr_acc_n":raw.get("imu_dr_acc_n"),
        "imu_dr_acc_e":raw.get("imu_dr_acc_e"),
        "imu_dr_acc_d":raw.get("imu_dr_acc_d"),
        "imu_dr_dt":raw.get("imu_dr_dt"),
'''

if '"imu_dr_n_mm":raw.get("imu_dr_n_mm")' in s:
    print("OK: IMU DR web mapping already present")
    raise SystemExit(0)

anchor='        "raw_of_ve":raw.get("raw_of_ve"),\n'
n=s.count(anchor)
if n!=1:
    raise SystemExit(f"ERROR: raw_of_ve mapping anchor count={n}")

s=s.replace(anchor,anchor+fields,1)
p.write_text(s)
print("OK: IMU DR fields mapped into /api/telemetry")
