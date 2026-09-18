#!/usr/bin/env python3
from pathlib import Path
p=Path('tools/web_service.py')
s=p.read_text()

# Add one compact diagnostic line below the existing navigation note.
anchor='''<div class="navEstimateNote">'''
i=s.find(anchor)
assert i>=0, 'navEstimateNote not found'
end=s.find('</div>',i)
assert end>=0, 'navEstimateNote closing div not found'
end+=len('</div>')
diag='''
  <div class="navEstimateNote" id="imuDiag">IMU diag: —</div>'''
if 'id="imuDiag"' not in s:
    s=s[:end]+diag+s[end:]

# Populate it in updateHud using telemetry already published by the runtime.
hud_anchor=""" if($('imuZ'))$('imuZ').textContent=imuReady&&t.imu_dr_d_mm!=null?fmt(t.imu_dr_d_mm,1)+' мм':'—';"""
assert hud_anchor in s, 'imuZ HUD anchor not found'
diag_js=""" if($('imuDiag')){
   const an=t.imu_dr_acc_n, ae=t.imu_dr_acc_e, ad=t.imu_dr_acc_d;
   const vn=t.imu_dr_vn, ve=t.imu_dr_ve, vd=t.imu_dr_vd;
   const ss=t.imu_dr_stationary_samples;
   const bs=t.imu_dr_bias_samples;
   $('imuDiag').textContent='IMU diag: aN/E/D '+fmt(an,3)+' / '+fmt(ae,3)+' / '+fmt(ad,3)+' m/s² · V N/E/D '+fmt(vn,3)+' / '+fmt(ve,3)+' / '+fmt(vd,3)+' m/s · stationary_samples '+(ss??'—')+' · bias_samples '+(bs??'—');
 }"""
if "const an=t.imu_dr_acc_n" not in s:
    s=s.replace(hud_anchor,hud_anchor+'\n'+diag_js,1)

p.write_text(s)
print('OK: IMU acceleration, velocity and stationary_samples exposed in navigation panel.')
