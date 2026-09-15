#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "======================================================================"
echo "monkeysStab — DATASET CAPTURE ПО ТИПУ ПОВЕРХНОСТИ"
echo "======================================================================"
echo
echo "Выберите поверхность:"
echo "  1) Дерево"
echo "  2) Бетон"
echo "  3) Кафель"
echo
read -r -p "Номер [1-3]: " choice

case "$choice" in
  1) SURFACE="wood"; SURFACE_RU="дерево" ;;
  2) SURFACE="concrete"; SURFACE_RU="бетон" ;;
  3) SURFACE="tile"; SURFACE_RU="кафель" ;;
  *) echo "ОШИБКА: допустимы только 1, 2 или 3." >&2; exit 2 ;;
esac

DURATION="${MONKEYS_DATASET_DURATION_SEC:-60}"
DATA_ROOT="${MONKEYS_DATASET_ROOT:-$HOME/monkeysStab_datasets}"
STAMP="$(date +%Y%m%d_%H%M%S)"
DATASET_DIR="$DATA_ROOT/${STAMP}_${SURFACE^^}"
mkdir -p "$DATASET_DIR"

cp -a config/runtime.json "$DATASET_DIR/runtime.json" 2>/dev/null || true
cp -a config/ov9281_current_mount.yaml "$DATASET_DIR/ov9281_current_mount.yaml" 2>/dev/null || true
cp -a config/mount_geometry.json "$DATASET_DIR/mount_geometry.json" 2>/dev/null || true
cp -a config/fc_profile.json "$DATASET_DIR/fc_profile.json" 2>/dev/null || true

python3 - "$DATASET_DIR/metadata.json" "$SURFACE" "$SURFACE_RU" "$DURATION" "$(git rev-parse HEAD)" <<'PY'
import json,sys,datetime,platform
out,surface,surface_ru,duration,commit=sys.argv[1:]
d={
  "schema":1,
  "surface":surface,
  "surface_ru":surface_ru,
  "duration_sec":float(duration),
  "git_commit":commit,
  "created_local":datetime.datetime.now().astimezone().isoformat(),
  "hostname":platform.node(),
  "capture_note":"Raw processed-latest OV9281 MJPEG frames + production telemetry. Across surfaces keep height, roll and pitch as similar as possible. Yaw may differ and is treated as a recorded covariate, not a controlled constant. Lighting may differ between locations."
}
with open(out,"w",encoding="utf-8") as f:
    json.dump(d,f,ensure_ascii=False,indent=2)
    f.write("\n")
PY

cat <<EOF

======================================================================
ПОВЕРХНОСТЬ: $SURFACE_RU
ДАТАСЕТ:     $DATASET_DIR
ДЛИТЕЛЬНОСТЬ: $DURATION с
======================================================================

Для сравнения покрытий:
  - во время одной 60-секундной записи аппарат НЕ двигать;
  - между WOOD / CONCRETE / TILE сохранять одинаковую высоту;
  - roll и pitch сохранять максимально одинаковыми;
  - yaw МОЖЕТ отличаться между поверхностями — он записывается в CSV;
  - освещение МОЖЕТ отличаться между поверхностями — это часть реального сценария;
  - поверхность должна занимать основную рабочую область кадра.

ВАЖНО:
  этот эксперимент сравнивает реальные условия "дерево / бетон / кафель".
  Он НЕ изолирует только коэффициент отражения как единственную переменную.

Запись остановится автоматически через $DURATION с.
Ctrl+C также корректно завершает запись.

Файлы датасета:
  frames.mjpgbin           — исходные MJPEG кадры, обработанные pipeline
  frames.csv               — timestamps и размер каждого кадра
  optical_flow_mavlink.csv — полный production telemetry/flow CSV
  metadata.json            — тип поверхности, commit, время
  snapshots config         — точная конфигурация эксперимента
======================================================================

EOF

export MONKEYS_RUN_DIR="$DATASET_DIR"
export MONKEYS_DATASET_DIR="$DATASET_DIR"
export MONKEYS_DATASET_SURFACE="$SURFACE"
export MONKEYS_DATASET_DURATION_SEC="$DURATION"
export MONKEYS_LOCAL_GUI=0

bash "$ROOT/scripts/run.sh"

# Add measured capture conditions to metadata after the run.  These values are
# observational: yaw and lighting are allowed to differ between surfaces.
python3 - "$DATASET_DIR/metadata.json" "$DATASET_DIR/optical_flow_mavlink.csv" <<'PY'
import csv,json,math,statistics,sys
meta_path,csv_path=sys.argv[1:]
with open(meta_path,"r",encoding="utf-8") as f:
    meta=json.load(f)

vals={k:[] for k in (
    "luna_m","fc_roll","fc_pitch","fc_yaw",
    "features","tracked","inliers","inlier_ratio",
    "flow_send_x","flow_send_y"
)}
try:
    with open(csv_path,newline="") as f:
        for r in csv.DictReader(f):
            for k in vals:
                try:
                    v=float(r.get(k,"nan"))
                    if math.isfinite(v):
                        vals[k].append(v)
                except Exception:
                    pass
except FileNotFoundError:
    pass

def stat(a):
    if not a:
        return None
    return {
        "mean":statistics.fmean(a),
        "median":statistics.median(a),
        "min":min(a),
        "max":max(a)
    }

meta["measured_conditions"]={
    "height_luna_m":stat(vals["luna_m"]),
    "roll_rad":stat(vals["fc_roll"]),
    "pitch_rad":stat(vals["fc_pitch"]),
    "yaw_rad":stat(vals["fc_yaw"]),
    "features":stat(vals["features"]),
    "tracked":stat(vals["tracked"]),
    "inliers":stat(vals["inliers"]),
    "inlier_ratio":stat(vals["inlier_ratio"]),
    "flow_send_x_rad_s":stat(vals["flow_send_x"]),
    "flow_send_y_rad_s":stat(vals["flow_send_y"]),
}
with open(meta_path,"w",encoding="utf-8") as f:
    json.dump(meta,f,ensure_ascii=False,indent=2)
    f.write("\n")
PY

echo
echo "======================================================================"
echo "ДАТАСЕТ ГОТОВ: $DATASET_DIR"
du -sh "$DATASET_DIR" || true
find "$DATASET_DIR" -maxdepth 1 -type f -printf '%f %s bytes\n' | sort
echo "======================================================================"
