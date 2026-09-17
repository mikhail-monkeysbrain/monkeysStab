#!/usr/bin/env python3
from pathlib import Path
import re
p=Path('tools/web_service.py'); s=p.read_text()

# Current navigation panel may have formatting changes from the IMU/HOME patches.
# Replace the six assignments by semantic IDs rather than one exact multiline block.
patterns={
 'fusionX': " if($('fusionX'))$('fusionX').textContent=fusionX!=null?fmt(fusionX,1)+' мм':'—';",
 'fusionY': " if($('fusionY'))$('fusionY').textContent=fusionY!=null?fmt(fusionY,1)+' мм':'—';",
 'fusionZ': " if($('fusionZ'))$('fusionZ').textContent=fusionZ!=null?fmt(fusionZ,1)+' мм':'—';",
 'corrX': " if($('corrX'))$('corrX').textContent=fusionX!=null?fmt(fusionX,1)+' мм':'—';",
 'corrY': " if($('corrY'))$('corrY').textContent=fusionY!=null?fmt(fusionY,1)+' мм':'—';",
 'corrZ': " if($('corrZ'))$('corrZ').textContent=fusionZ!=null?fmt(fusionZ,1)+' мм':'—';",
}
# Insert fusion variables immediately after the IMU Z display line.
anchor=" if($('imuZ'))$('imuZ').textContent=imuReady&&t.imu_dr_d_mm!=null?fmt(t.imu_dr_d_mm,1)+' мм':'—';"
assert anchor in s, 'IMU Z anchor not found'
insert=anchor+"\n // Fusion v1: validated WORKED5 horizontal position + independent IMU vertical DR.\n const fusionX=camX;\n const fusionY=camY;\n const fusionZ=imuReady&&t.imu_dr_d_mm!=null?t.imu_dr_d_mm:null;"
s=s.replace(anchor,insert,1)
for ident,repl in patterns.items():
    rx=r"^\s*if\(\$\('"+re.escape(ident)+r"'\)\).*?;$"
    s,n=re.subn(rx,repl,s,count=1,flags=re.M)
    assert n==1, f'{ident} assignment anchor not found'

old_notes=[
 'Камера: WORKED5 X/Y. IMU: независимое dead reckoning после HOME-калибровки. Итоговая fusion XYZ пока не реализована и не подменяется EKF-данными.',
 'Камера: WORKED5 X/Y. IMU: независимое dead reckoning. Итоговая fusion XYZ пока не реализована и не подменяется EKF-данными.'
]
for note in old_notes:
    if note in s:
        s=s.replace(note,'Камера: WORKED5 X/Y. IMU: независимое dead reckoning. Итог v1: WORKED5 X/Y + IMU Z; горизонтальный IMU DR пока показан отдельно и не усредняется без валидации.',1)
        break
p.write_text(s)
print('OK: fusion v1 shown as WORKED5 X/Y + IMU Z; correction row uses fused XYZ.')
