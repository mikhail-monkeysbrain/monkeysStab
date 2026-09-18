#!/usr/bin/env python3
from pathlib import Path
p=Path('tools/web_service.py')
s=p.read_text()

old=""" // Fusion v1: WORKED5 horizontal position + independent IMU vertical DR.
 const finalX=camX;
 const finalY=camY;
 const finalZ=imuReady&&t.imu_dr_d_mm!=null?t.imu_dr_d_mm:null;"""
new=""" // Fusion v1: WORKED5 horizontal position + independent IMU vertical DR.
 // camX/camY are DOM element IDs, not numeric variables; use telemetry values directly.
 const finalX=t.raw_of_n_mm==null?null:t.raw_of_n_mm;
 const finalY=t.raw_of_e_mm==null?null:t.raw_of_e_mm;
 const finalZ=imuReady&&t.imu_dr_d_mm!=null?t.imu_dr_d_mm:null;"""
assert old in s, 'actual Fusion v1 block not found'
s=s.replace(old,new,1)

p.write_text(s)
print('OK: final X/Y now use numeric WORKED5 telemetry instead of DOM globals.')
