#!/usr/bin/env python3
from pathlib import Path
p=Path('tools/web_service.py'); s=p.read_text()
# Pass runtime IMU DR fields through live_payload, converting position to mm.
anchor='''        "raw_of_ve":raw.get("raw_of_ve"),\n        "rc_zero_event":rc_zero_event,'''
insert='''        "raw_of_ve":raw.get("raw_of_ve"),
        "imu_dr_calibrated":bool(raw.get("imu_dr_calibrated",False)),
        "imu_dr_calibrating":bool(raw.get("imu_dr_calibrating",False)),
        "imu_dr_bias_samples":raw.get("imu_dr_bias_samples",0),
        "imu_dr_n_mm":float(raw.get("imu_dr_n",0.0))*1000.0 if raw.get("imu_dr_n") is not None else None,
        "imu_dr_e_mm":float(raw.get("imu_dr_e",0.0))*1000.0 if raw.get("imu_dr_e") is not None else None,
        "imu_dr_d_mm":float(raw.get("imu_dr_d",0.0))*1000.0 if raw.get("imu_dr_d") is not None else None,
        "imu_dr_vn":raw.get("imu_dr_vn"),"imu_dr_ve":raw.get("imu_dr_ve"),"imu_dr_vd":raw.get("imu_dr_vd"),
        "imu_dr_acc_n":raw.get("imu_dr_acc_n"),"imu_dr_acc_e":raw.get("imu_dr_acc_e"),"imu_dr_acc_d":raw.get("imu_dr_acc_d"),
        "imu_dr_bias_n":raw.get("imu_dr_bias_n"),"imu_dr_bias_e":raw.get("imu_dr_bias_e"),"imu_dr_bias_d":raw.get("imu_dr_bias_d"),
        "imu_dr_stationary_samples":raw.get("imu_dr_stationary_samples",0),
        "rc_zero_event":rc_zero_event,'''
assert anchor in s, 'live_payload anchor not found'; s=s.replace(anchor,insert,1)
# Current locally installed navigation panel has these explicit placeholder lines.
old=""" // A standalone IMU position is not currently published by runtime. Do not label FC EKF as IMU.\n ['imuX','imuY','imuZ'].forEach(id=>{if($(id))$(id).textContent='—'});"""
new=""" const imuReady=!!t.imu_dr_calibrated;\n if($('imuX'))$('imuX').textContent=imuReady&&t.imu_dr_n_mm!=null?fmt(t.imu_dr_n_mm,1)+' мм':'—';\n if($('imuY'))$('imuY').textContent=imuReady&&t.imu_dr_e_mm!=null?fmt(t.imu_dr_e_mm,1)+' мм':'—';\n if($('imuZ'))$('imuZ').textContent=imuReady&&t.imu_dr_d_mm!=null?fmt(t.imu_dr_d_mm,1)+' мм':'—';"""
assert old in s, 'navigation IMU placeholder anchor not found'; s=s.replace(old,new,1)
# Update explanatory note, but deliberately keep fusion row unavailable.
s=s.replace('Камера: WORKED5 X/Y. Отдельная IMU-позиция и fusion XYZ пока не публикуются runtime и поэтому не подменяются EKF-данными.','Камера: WORKED5 X/Y. IMU: независимое dead reckoning после HOME-калибровки. Итоговая fusion XYZ пока не реализована и не подменяется EKF-данными.',1)
p.write_text(s)
print('OK: IMU DR telemetry mapped to navigation Web UI; fusion row remains unavailable.')
