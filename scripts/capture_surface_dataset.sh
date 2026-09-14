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
  "capture_note":"Raw processed-latest OV9281 MJPEG frames + production telemetry. Keep geometry and lighting fixed across surfaces."
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
  - аппарат и камеру не перемещать;
  - высоту не менять;
  - освещение не менять;
  - поверхность должна занимать ту же область кадра.

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

echo
echo "======================================================================"
echo "ДАТАСЕТ ГОТОВ: $DATASET_DIR"
du -sh "$DATASET_DIR" || true
find "$DATASET_DIR" -maxdepth 1 -type f -printf '%f %s bytes\n' | sort
echo "======================================================================"
