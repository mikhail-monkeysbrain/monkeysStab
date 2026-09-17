#!/usr/bin/env python3
from pathlib import Path
p=Path('src/optical_flow_mavlink.cpp'); s=p.read_text()
old='''            <<",\\\"raw_of_ve\\\":"<<jsonNumber(web_raw_ve)
            <<",\\\"rc_zero_seq\\\":"<<rc_zero_seq'''
new='''            <<",\\\"raw_of_ve\\\":"<<jsonNumber(web_raw_ve)
            <<",\\\"imu_dr_calibrated\\\":"<<(fc.imu_dr_state.calibrated?"true":"false")
            <<",\\\"imu_dr_calibrating\\\":"<<(fc.imu_dr_state.calibrating?"true":"false")
            <<",\\\"imu_dr_bias_samples\\\":"<<fc.imu_dr_state.bias_samples
            <<",\\\"imu_dr_bias_n\\\":"<<jsonNumber(fc.imu_dr_state.bias_n)
            <<",\\\"imu_dr_bias_e\\\":"<<jsonNumber(fc.imu_dr_state.bias_e)
            <<",\\\"imu_dr_bias_d\\\":"<<jsonNumber(fc.imu_dr_state.bias_d)
            <<",\\\"imu_dr_acc_n\\\":"<<jsonNumber(fc.imu_dr_state.acc_n)
            <<",\\\"imu_dr_acc_e\\\":"<<jsonNumber(fc.imu_dr_state.acc_e)
            <<",\\\"imu_dr_acc_d\\\":"<<jsonNumber(fc.imu_dr_state.acc_d)
            <<",\\\"imu_dr_vn\\\":"<<jsonNumber(fc.imu_dr_state.vel_n)
            <<",\\\"imu_dr_ve\\\":"<<jsonNumber(fc.imu_dr_state.vel_e)
            <<",\\\"imu_dr_vd\\\":"<<jsonNumber(fc.imu_dr_state.vel_d)
            <<",\\\"imu_dr_n\\\":"<<jsonNumber(fc.imu_dr_state.pos_n)
            <<",\\\"imu_dr_e\\\":"<<jsonNumber(fc.imu_dr_state.pos_e)
            <<",\\\"imu_dr_d\\\":"<<jsonNumber(fc.imu_dr_state.pos_d)
            <<",\\\"imu_dr_stationary_samples\\\":"<<fc.imu_dr_state.stationary_samples
            <<",\\\"rc_zero_seq\\\":"<<rc_zero_seq'''
assert old in s, 'web telemetry anchor not found'
s=s.replace(old,new,1)
p.write_text(s)
print('OK: IMU DR state published in runtime WebSocket telemetry')
