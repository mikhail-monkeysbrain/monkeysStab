#!/usr/bin/env python3
from pathlib import Path

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()

if '"imu_dr_n_mm"' in s:
    print("OK: IMU DR position telemetry already present")
    raise SystemExit(0)

anchor='            <<",\\\"imu_dr_acc_n\\\":"<<jsonNumber(fc.imu_dr_state.acc_n)\n'
if s.count(anchor)!=1:
    raise SystemExit(f"ERROR: IMU DR telemetry anchor count={s.count(anchor)}")

insert='''            <<",\\\"imu_dr_n_mm\\\":"<<jsonNumber(fc.imu_dr_state.pos_n*1000.0)
            <<",\\\"imu_dr_e_mm\\\":"<<jsonNumber(fc.imu_dr_state.pos_e*1000.0)
            <<",\\\"imu_dr_d_mm\\\":"<<jsonNumber(fc.imu_dr_state.pos_d*1000.0)
'''
s=s.replace(anchor,insert+anchor,1)
p.write_text(s)
print("OK: IMU DR position telemetry added")
