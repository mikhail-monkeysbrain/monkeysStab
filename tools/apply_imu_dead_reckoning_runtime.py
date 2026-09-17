#!/usr/bin/env python3
from pathlib import Path
p=Path('src/optical_flow_mavlink.cpp'); s=p.read_text()

def once(old,new):
 global s
 assert old in s, 'anchor not found: '+old[:80]
 s=s.replace(old,new,1)

once('#include "worked5_estimator.hpp"','#include "worked5_estimator.hpp"\n#include "imu_dead_reckoning.hpp"')
once('  FlowFcImu imu{};\n  FlowFcTarget target{};','  FlowFcImu imu{};\n  imu_dr::State imu_dr_state{};\n  FlowFcTarget target{};')
# Feed every HIGHRES_IMU sample while FlowFc mutex already protects attitude/imu state.
once('''              imu.time_usec=q.time_usec; imu.recv_ns=monoNs(); imu.valid=true; ++imu_count;''','''              imu.time_usec=q.time_usec; imu.recv_ns=monoNs(); imu.valid=true; ++imu_count;
              if(gyro.valid) imu_dr::update(imu_dr_state,q.xacc,q.yacc,q.zacc,q.xgyro,q.ygyro,q.zgyro,gyro.roll,gyro.pitch,gyro.yaw,q.time_usec);''')
# Physical RC HOME is the canonical C++ reset already used for WORKED5 closure.
once('''        web_raw_n=0.0; web_raw_e=0.0; web_raw_vn=0.0; web_raw_ve=0.0; web_raw_step_valid=false;''','''        web_raw_n=0.0; web_raw_e=0.0; web_raw_vn=0.0; web_raw_ve=0.0; web_raw_step_valid=false;
        { std::lock_guard<std::mutex> l(fc.mu); imu_dr::reset(fc.imu_dr_state); }''')
p.write_text(s)
print('OK: independent IMU DR core integrated into runtime')
print('HOME: 50-sample bias calibration; then NED integration + conservative ZUPT.')
print('No Web payload/fusion changes in this patch.')
