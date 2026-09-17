#!/usr/bin/env python3
from pathlib import Path

p=Path('src/optical_flow_mavlink.cpp')
s=p.read_text()

old='''struct FlowFcTarget {\n'''
new='''struct FlowFcImu {\n  double ax=0,ay=0,az=0;       // HIGHRES_IMU body acceleration, m/s^2\n  double gx=0,gy=0,gz=0;       // HIGHRES_IMU body gyro, rad/s\n  uint64_t time_usec=0;\n  int64_t recv_ns=0;\n  bool valid=false;\n};\n\nstruct FlowFcTarget {\n'''
assert old in s
s=s.replace(old,new,1)

old='''  FlowFcGyro gyro{};\n  FlowFcTarget target{};'''
new='''  FlowFcGyro gyro{};\n  FlowFcImu imu{};\n  FlowFcTarget target{};'''
assert old in s
s=s.replace(old,new,1)

old='''  uint64_t gyro_count=0;\n  double gyro_sum_x=0,gyro_sum_y=0,gyro_sum_z=0;'''
new='''  uint64_t gyro_count=0;\n  uint64_t imu_count=0;\n  double gyro_sum_x=0,gyro_sum_y=0,gyro_sum_z=0;'''
assert old in s
s=s.replace(old,new,1)

old='''      requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);\n      requestRate(fd,sys,comp,MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,20);'''
new='''      requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);\n      requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,50);\n      requestRate(fd,sys,comp,MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,20);'''
assert old in s
s=s.replace(old,new,1)

old='''            } else if(m.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED){\n'''
new='''            } else if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){\n              mavlink_highres_imu_t q{}; mavlink_msg_highres_imu_decode(&m,&q);\n              std::lock_guard<std::mutex> l(mu);\n              imu.ax=q.xacc; imu.ay=q.yacc; imu.az=q.zacc;\n              imu.gx=q.xgyro; imu.gy=q.ygyro; imu.gz=q.zgyro;\n              imu.time_usec=q.time_usec; imu.recv_ns=monoNs(); imu.valid=true; ++imu_count;\n            } else if(m.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED){\n'''
assert old in s
s=s.replace(old,new,1)

p.write_text(s)
print('OK: HIGHRES_IMU 50 Hz receiver installed in src/optical_flow_mavlink.cpp')
print('This patch only receives/stores IMU; it does not integrate position or change WORKED5/EKF.')
