#!/usr/bin/env python3
from pathlib import Path
p=Path('tools/web_service.py'); s=p.read_text()

# Exact anchors taken from the actual local grep supplied after v2 failed.
imu_anchor=" if($('imuZ'))$('imuZ').textContent=imuReady&&t.imu_dr_d_mm!=null?fmt(t.imu_dr_d_mm,1)+' мм':'—';"
assert imu_anchor in s, 'imuZ anchor not found'
if 'const finalX=camX;' not in s:
    s=s.replace(imu_anchor, imu_anchor+"\n // Fusion v1: WORKED5 horizontal position + independent IMU vertical DR.\n const finalX=camX;\n const finalY=camY;\n const finalZ=imuReady&&t.imu_dr_d_mm!=null?t.imu_dr_d_mm:null;",1)

group=" ['finalX','finalY','finalZ'].forEach(id=>{if($(id))$(id).textContent='—'});"
assert group in s, 'grouped final XYZ placeholder not found'
s=s.replace(group,""" if($('finalX'))$('finalX').textContent=finalX!=null?fmt(finalX,1)+' мм':'—';
 if($('finalY'))$('finalY').textContent=finalY!=null?fmt(finalY,1)+' мм':'—';
 if($('finalZ'))$('finalZ').textContent=finalZ!=null?fmt(finalZ,1)+' мм':'—';""",1)

repls={
 " if($('corrX'))$('corrX').textContent=t.raw_of_n_mm==null?'—':fmt(t.raw_of_n_mm,1)+' мм';":" if($('corrX'))$('corrX').textContent=finalX!=null?fmt(finalX,1)+' мм':'—';",
 " if($('corrY'))$('corrY').textContent=t.raw_of_e_mm==null?'—':fmt(t.raw_of_e_mm,1)+' мм';":" if($('corrY'))$('corrY').textContent=finalY!=null?fmt(finalY,1)+' мм':'—';",
 " if($('corrZ'))$('corrZ').textContent='—';":" if($('corrZ'))$('corrZ').textContent=finalZ!=null?fmt(finalZ,1)+' мм':'—';",
}
for old,new in repls.items():
    assert old in s, 'correction anchor not found: '+old
    s=s.replace(old,new,1)

notes=[
 'Камера: WORKED5 X/Y. IMU: независимое dead reckoning после HOME-калибровки. Итоговая fusion XYZ пока не реализована и не подменяется EKF-данными.',
 'Камера: WORKED5 X/Y. IMU: независимое dead reckoning. Итоговая fusion XYZ пока не реализована и не подменяется EKF-данными.'
]
for old in notes:
    if old in s:
        s=s.replace(old,'Камера: WORKED5 X/Y. IMU: независимое dead reckoning. Итог v1: WORKED5 X/Y + IMU Z; горизонтальный IMU DR пока показан отдельно и не усредняется без валидации.',1)
        break

p.write_text(s)
print('OK: grouped final XYZ placeholder replaced; correction row now follows final XYZ.')
