#!/usr/bin/env python3
from pathlib import Path
p=Path('tools/web_service.py'); s=p.read_text()

# Fusion v1 deliberately lives in the common HOME-relative Web frame:
# camera WORKED5 is the position anchor for horizontal N/E; IMU supplies Z.
# We do NOT average IMU horizontal dead-reckoning into the proven camera metric position yet.
old=""" if($('fusionX'))$('fusionX').textContent='—';
 if($('fusionY'))$('fusionY').textContent='—';
 if($('fusionZ'))$('fusionZ').textContent='—';
 if($('corrX'))$('corrX').textContent=camX!=null?fmt(camX,1)+' мм':'—';
 if($('corrY'))$('corrY').textContent=camY!=null?fmt(camY,1)+' мм':'—';
 if($('corrZ'))$('corrZ').textContent='—';"""
new=""" // Fusion v1: WORKED5 is the validated horizontal metric anchor; IMU contributes vertical DR.
 // Horizontal IMU DR remains visible above for comparison, but is not averaged into X/Y without validation.
 const fusionX=camX;
 const fusionY=camY;
 const fusionZ=imuReady&&t.imu_dr_d_mm!=null?t.imu_dr_d_mm:null;
 if($('fusionX'))$('fusionX').textContent=fusionX!=null?fmt(fusionX,1)+' мм':'—';
 if($('fusionY'))$('fusionY').textContent=fusionY!=null?fmt(fusionY,1)+' мм':'—';
 if($('fusionZ'))$('fusionZ').textContent=fusionZ!=null?fmt(fusionZ,1)+' мм':'—';
 if($('corrX'))$('corrX').textContent=fusionX!=null?fmt(fusionX,1)+' мм':'—';
 if($('corrY'))$('corrY').textContent=fusionY!=null?fmt(fusionY,1)+' мм':'—';
 if($('corrZ'))$('corrZ').textContent=fusionZ!=null?fmt(fusionZ,1)+' мм':'—';"""
assert old in s, 'fusion/correction placeholder anchor not found'
s=s.replace(old,new,1)
s=s.replace('Камера: WORKED5 X/Y. IMU: независимое dead reckoning после HOME-калибровки. Итоговая fusion XYZ пока не реализована и не подменяется EKF-данными.',
'''Камера: WORKED5 X/Y. IMU: независимое dead reckoning. Итог v1: WORKED5 X/Y + IMU Z; горизонтальный IMU DR пока показан отдельно и не усредняется без валидации.''',1)
p.write_text(s)
print('OK: fusion v1 shown as WORKED5 X/Y + IMU Z; correction row uses fused XYZ.')
