#!/usr/bin/env python3
from pathlib import Path

p = Path(__file__).resolve().parents[1] / "tools" / "web_service.py"
s = p.read_text(encoding="utf-8")

old_css = '.metrics{display:grid;grid-template-columns:repeat(5,1fr);gap:8px;margin-top:10px}\n.metric{background:#09151f;border:1px solid #163147;border-radius:7px;padding:9px}.metric span{font-size:11px;color:#7fa1ba}.metric b{display:block;font-size:18px;margin-top:2px}\n'
new_css = old_css + '.navEstimateCard{margin-top:10px;padding:10px 12px}.navEstimateCard h3{margin:0 0 8px}.navEstimateRow{display:grid;grid-template-columns:220px repeat(3,minmax(90px,1fr));gap:8px;align-items:center;padding:8px 0;border-top:1px solid #142d40}.navEstimateRow:first-of-type{border-top:0}.navEstimateLabel{color:#9fb7ca;font-size:12px}.navEstimateAxis{background:#09151f;border:1px solid #163147;border-radius:6px;padding:7px 9px}.navEstimateAxis span{display:block;color:#6f91aa;font-size:10px}.navEstimateAxis b{display:block;margin-top:2px;font-size:15px}.navEstimateNote{margin-top:7px;color:#6f91aa;font-size:10px}\n'
if old_css not in s:
    raise SystemExit('ERROR: metrics CSS anchor not found')
s = s.replace(old_css, new_css, 1)

old_html = '''  <div class="metrics">\n   <div class="metric"><span>X</span><b id="mx">—</b></div>\n   <div class="metric"><span>Y</span><b id="my">—</b></div>\n   <div class="metric"><span>Z</span><b id="mz">—</b></div>\n   <div class="metric"><span>TF-Luna</span><b id="mr">—</b></div>\n   <div class="metric"><span>Flow quality</span><b id="mq">—</b></div>\n  </div>\n\n  <div class="card" style="margin-top:10px">\n   <h3>Дрейф от HOME — EKF vs RAW Optical Flow</h3>\n   <div class="kv" style="grid-template-columns:155px 1fr 155px 1fr 155px 1fr">\n    <span>EKF ΔN / ΔE</span><span id="ekfNE">—</span>\n    <span>EKF |XY|</span><span id="ekfDrift">—</span>\n    <span>EKF vN / vE</span><span id="ekfVel">—</span>\n    <span>RAW OF ΔN / ΔE</span><span id="rawNE">—</span>\n    <span>RAW OF |XY|</span><span id="rawDrift">—</span>\n    <span>RAW OF vN / vE</span><span id="rawVel">—</span>\n   </div>\n  </div>\n'''
new_html = '''  <div class="metrics">\n   <div class="metric"><span>EKF X</span><b id="mx">—</b></div>\n   <div class="metric"><span>EKF Y</span><b id="my">—</b></div>\n   <div class="metric"><span>EKF Z</span><b id="mz">—</b></div>\n   <div class="metric"><span>TF-Luna</span><b id="mr">—</b></div>\n   <div class="metric"><span>Flow quality</span><b id="mq">—</b></div>\n  </div>\n\n  <div class="card navEstimateCard">\n   <h3>Навигационная оценка от HOME</h3>\n   <div class="navEstimateRow">\n    <div class="navEstimateLabel">Оценка по камере</div>\n    <div class="navEstimateAxis"><span>X</span><b id="camX">—</b></div>\n    <div class="navEstimateAxis"><span>Y</span><b id="camY">—</b></div>\n    <div class="navEstimateAxis"><span>Z</span><b id="camZ">—</b></div>\n   </div>\n   <div class="navEstimateRow">\n    <div class="navEstimateLabel">Оценка по IMU</div>\n    <div class="navEstimateAxis"><span>X</span><b id="imuX">—</b></div>\n    <div class="navEstimateAxis"><span>Y</span><b id="imuY">—</b></div>\n    <div class="navEstimateAxis"><span>Z</span><b id="imuZ">—</b></div>\n   </div>\n   <div class="navEstimateRow">\n    <div class="navEstimateLabel">Итоговая оценка</div>\n    <div class="navEstimateAxis"><span>X</span><b id="finalX">—</b></div>\n    <div class="navEstimateAxis"><span>Y</span><b id="finalY">—</b></div>\n    <div class="navEstimateAxis"><span>Z</span><b id="finalZ">—</b></div>\n   </div>\n   <div class="navEstimateRow">\n    <div class="navEstimateLabel">Поправка / удаление от старта</div>\n    <div class="navEstimateAxis"><span>X</span><b id="corrX">—</b></div>\n    <div class="navEstimateAxis"><span>Y</span><b id="corrY">—</b></div>\n    <div class="navEstimateAxis"><span>Z</span><b id="corrZ">—</b></div>\n   </div>\n   <div class="navEstimateNote">Камера: WORKED5 X/Y. Отдельная IMU-позиция и fusion XYZ пока не публикуются runtime и поэтому не подменяются EKF-данными.</div>\n  </div>\n'''
if old_html not in s:
    raise SystemExit('ERROR: flight metrics/drift HTML anchor not found')
s = s.replace(old_html, new_html, 1)

old_clear = "   ['ekfNE','ekfDrift','ekfVel','rawNE','rawDrift','rawVel'].forEach(id=>{if($(id))$(id).textContent='—'});"
new_clear = "   ['camX','camY','camZ','imuX','imuY','imuZ','finalX','finalY','finalZ','corrX','corrY','corrZ'].forEach(id=>{if($(id))$(id).textContent='—'});"
if old_clear not in s:
    raise SystemExit('ERROR: HUD clear anchor not found')
s = s.replace(old_clear, new_clear, 1)

old_update = ''' if($('ekfNE'))$('ekfNE').textContent=fmt(t.x_mm,1)+' / '+fmt(t.y_mm,1)+' мм';\n if($('ekfDrift'))$('ekfDrift').textContent=fmt(t.ekf_drift_mm,1)+' мм';\n if($('ekfVel'))$('ekfVel').textContent=fmt((t.vx||0)*1000,1)+' / '+fmt((t.vy||0)*1000,1)+' мм/с';\n if($('rawNE'))$('rawNE').textContent=t.raw_of_n_mm==null?'—':fmt(t.raw_of_n_mm,1)+' / '+fmt(t.raw_of_e_mm,1)+' мм';\n if($('rawDrift'))$('rawDrift').textContent=t.raw_of_drift_mm==null?'—':fmt(t.raw_of_drift_mm,1)+' мм';\n if($('rawVel'))$('rawVel').textContent=t.raw_of_vn==null?'—':fmt(t.raw_of_vn*1000,1)+' / '+fmt(t.raw_of_ve*1000,1)+' мм/с';\n'''
new_update = ''' if($('camX'))$('camX').textContent=t.raw_of_n_mm==null?'—':fmt(t.raw_of_n_mm,1)+' мм';\n if($('camY'))$('camY').textContent=t.raw_of_e_mm==null?'—':fmt(t.raw_of_e_mm,1)+' мм';\n if($('camZ'))$('camZ').textContent='—';\n // A standalone IMU position is not currently published by runtime. Do not label FC EKF as IMU.\n ['imuX','imuY','imuZ'].forEach(id=>{if($(id))$(id).textContent='—'});\n // No camera+IMU fusion position exists yet. Keep the row explicit instead of fabricating a value.\n ['finalX','finalY','finalZ'].forEach(id=>{if($(id))$(id).textContent='—'});\n // Until fusion is implemented, correction uses the only independent metric displacement: WORKED5 X/Y.\n if($('corrX'))$('corrX').textContent=t.raw_of_n_mm==null?'—':fmt(t.raw_of_n_mm,1)+' мм';\n if($('corrY'))$('corrY').textContent=t.raw_of_e_mm==null?'—':fmt(t.raw_of_e_mm,1)+' мм';\n if($('corrZ'))$('corrZ').textContent='—';\n'''
if old_update not in s:
    raise SystemExit('ERROR: HUD drift update anchor not found')
s = s.replace(old_update, new_update, 1)

p.write_text(s, encoding='utf-8')
print('OK: navigation estimate panel installed in tools/web_service.py')
print('Camera X/Y = WORKED5. IMU XYZ and fused XYZ remain unavailable until runtime publishes real estimators.')
