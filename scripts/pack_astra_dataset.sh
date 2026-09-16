#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
D="${1:-}"
[[ -n "$D" ]] || { echo "Usage: $0 /path/to/ASTRA_AB_RAW"; exit 2; }
D="$(readlink -f "$D")"
[[ -d "$D" ]] || { echo "ERROR: dataset not found: $D"; exit 2; }
for f in frames.csv frames.mjpgbin optical_flow_mavlink.csv fc_dataflash.bin capture_manifest.txt; do
  [[ -s "$D/$f" ]] || { echo "ERROR: missing/empty $D/$f"; exit 3; }
done
python3 - "$D" <<'PY'
from pathlib import Path
import csv,json,sys
d=Path(sys.argv[1])
with (d/"frames.csv").open(newline="") as f: frames=list(csv.DictReader(f))
with (d/"optical_flow_mavlink.csv").open(newline="") as f: flow=list(csv.DictReader(f))
events=[{"frame":int(r["frame"]),"event":int(r["return_event"])} for r in flow if r.get("return_event","0") not in ("","0")]
info={
 "dataset":d.name,
 "purpose":"Raw synchronized evidence for Astra to develop an offline metric distance estimator. Existing production optical-flow metric is not ground truth.",
 "physical_gt_mm":292,
 "protocol":"A to B on table; measured A-B 292 mm; then physical return toward A.",
 "camera_frames":len(frames),
 "persisted_return_events":events,
 "known_limitation":"CSV persisted events 1 and 2 only. Reverse-leg start/final-A markers are absent; raw camera and FC DataFlash continued recording.",
 "files":{
  "frames.mjpgbin":"Original OV9281 MJPEG payloads. Records: uint64 LE camera_ts_ns, uint32 LE jpeg_size, JPEG bytes.",
  "frames.csv":"dataset_frame,camera_ts_ns,mono_ns,jpeg_size",
  "optical_flow_mavlink.csv":"Synchronized production diagnostic CSV; estimator output is NOT ground truth.",
  "fc_dataflash.bin":"ArduPilot remote DataFlash captured during run.",
  "capture_manifest.txt":"Capture provenance and git commit."
 }}
(d/"ASTRA_TASK.json").write_text(json.dumps(info,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
readme="""# ASTRA raw A/B dataset

## Задача
Разработать офлайн-алгоритм, который по исходным данным эксперимента оценивает физическое горизонтальное перемещение камеры/БПЛА. Не подгонять существующий production optical-flow estimator под 292 мм. Разрешено заново извлекать признаки/поток из исходных кадров и объединять их с данными FC/IMU и TF-Luna.

## Ground truth
Физически измеренный проход A->B: 292 мм. Аппарат перемещался руками по столу; возможны небольшие roll/pitch/yaw и небольшой поперечный уход.

Сохранённые маркеры CSV:
- event 1 — начало A->B;
- event 2 — конец A->B / точка B.

Маркеры начала обратного прохода и финальной A в CSV не сохранились. Первичная метрическая задача — A->B между events 1 и 2. Обратный участок использовать только после независимого определения его границ по raw данным.

## Данные
- frames.mjpgbin — оригинальные JPEG OV9281 без повторного кодирования.
- frames.csv — номер кадра и временные метки.
- fc_dataflash.bin — DataFlash FC.
- optical_flow_mavlink.csv — синхронная диагностическая телеметрия.
- capture_manifest.txt — provenance.
- ASTRA_TASK.json — машинно-читаемое описание.
- SHA256SUMS — контроль целостности.

## Важное ограничение
Production optical-flow output не считать эталоном. В этом прогоне существующий estimator существенно недооценил 292 мм. Цель датасета — найти причину и построить estimator из raw evidence.

## Что требуется от Astra
1. Проверить целостность и временные шкалы.
2. Декодировать исходные кадры.
3. Разобрать DataFlash и перечислить реально доступные raw IMU/gyro/accel/attitude/range каналы и частоты.
4. Определить синхронизацию camera <-> FC.
5. Построить несколько независимых оценок A->B, начиная с чисто визуальной.
6. Использовать TF-Luna/геометрию для метрического масштаба явно, с формулой.
7. Показать покадровые/интервальные ошибки и места потери наблюдаемости, не только итог.
8. Не менять GT и не калибровать свободный scale только по единственному GT.
9. Вернуть воспроизводимый код анализа и список дополнительных прогонов для cross-validation.
"""
(d/"README_ASTRA.md").write_text(readme,encoding="utf-8")
PY
(cd "$D" && sha256sum frames.csv frames.mjpgbin optical_flow_mavlink.csv fc_dataflash.bin capture_manifest.txt ASTRA_TASK.json README_ASTRA.md > SHA256SUMS)
BASE="$(basename "$D")"
OUT="$(dirname "$D")/${BASE}.tar.gz"
tar -C "$(dirname "$D")" -czf "$OUT" "$BASE"
echo "===== ASTRA ARCHIVE READY ====="
ls -lh "$OUT"
sha256sum "$OUT"
echo "Archive: $OUT"
