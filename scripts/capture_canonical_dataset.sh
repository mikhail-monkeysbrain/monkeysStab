#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

STAMP="$(date +%Y%m%d_%H%M%S)"
DATASET_ROOT="${MONKEYS_CANONICAL_ROOT:-$HOME/monkeysStab_datasets}"
DATASET_DIR="$DATASET_ROOT/${STAMP}_CANONICAL"
SURFACE="${MONKEYS_DATASET_SURFACE:-manual_ab_return}"
mkdir -p "$DATASET_DIR/config"

for f in runtime.json mount_geometry.json ov9281_current_mount.yaml fc_profile.json; do
  [[ -f "$ROOT/config/$f" ]] && cp "$ROOT/config/$f" "$DATASET_DIR/config/$f"
done

{
  echo "schema=monkeysStab-canonical-capture-v1"
  echo "created_local=$(date --iso-8601=seconds)"
  echo "created_utc=$(date -u --iso-8601=seconds)"
  echo
  echo "repository=$ROOT"
  echo "branch=$(git branch --show-current)"
  echo "head=$(git rev-parse HEAD)"
  echo
  echo "===== GIT STATUS ====="
  git status -sb
  echo
  echo "===== LAST COMMIT ====="
  git log -1 --oneline
} > "$DATASET_DIR/MANIFEST.txt"

{
  uname -a
  echo
  cat /proc/device-tree/model 2>/dev/null || true
  echo
  v4l2-ctl --all -d "${MONKEYS_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}" 2>&1 || true
} > "$DATASET_DIR/HARDWARE.txt"

cat > "$DATASET_DIR/PHYSICAL_TEST_NOTES.txt" <<EOF
surface=$SURFACE
protocol=A -> B -> physical measurement -> A
arm=DISARMED
physical_distance_mm=
notes=
EOF

echo
echo "ПОЛОЖИ БПЛА В ТОЧКУ A."
echo "После полной остановки нажми SPACE."
echo
echo "Далее: перенеси в B, после остановки нажми B."
echo "В B НЕ ДВИГАЙ аппарат и измерь A→B рулеткой."
echo "Фактическое расстояние запиши в PHYSICAL_TEST_NOTES.txt."
echo "После измерения верни аппарат в A и нажми H."
echo "Для завершения нажми Q."
echo

set +e
MONKEYS_RUN_DIR="$DATASET_DIR" \
MONKEYS_DATASET_DIR="$DATASET_DIR" \
MONKEYS_DATASET_SURFACE="$SURFACE" \
MONKEYS_RETURN_GUI=0 \
MONKEYS_RETURN_CLI=1 \
MONKEYS_RETURN_MANUAL_TARGET=1 \
MONKEYS_LOCAL_GUI=0 \
bash "$ROOT/scripts/run.sh" 2>&1 \
  | tee "$DATASET_DIR/runtime.log" \
  | awk \'
      BEGIN { fflush() }
      /CANONICAL A MARK:/ {
        print "\nТОЧКА A ЗАФИКСИРОВАНА."
        print "Перенеси БПЛА в точку B. После полной остановки нажми B."
        fflush(); next
      }
      /CANONICAL B MARK:/ {
        print "\nТОЧКА B ЗАФИКСИРОВАНА."
        print "НЕ ДВИГАЙ БПЛА. Измерь рулеткой фактическое расстояние A→B."
        print "После измерения верни БПЛА физически в точку A и после полной остановки нажми H."
        fflush(); next
      }
      /CANONICAL H MARK:/ {
        print "\nВОЗВРАТ В A ЗАФИКСИРОВАН."
        print "Нажми Q для завершения теста."
        fflush(); next
      }
      /ОШИБКА:|DATASET CAPTURE COMPLETE:|Остановлено\. CSV:/ {
        print; fflush(); next
      }
    '
RC=${PIPESTATUS[0]}
set -e

{
  echo
  echo "===== CAPTURE RESULT ====="
  echo "exit_code=$RC"
  echo "finished_local=$(date --iso-8601=seconds)"
  echo
  echo "===== FILES ====="
  find "$DATASET_DIR" -maxdepth 2 -type f -printf '%s bytes  %P\n' | sort
} >> "$DATASET_DIR/MANIFEST.txt"

echo
echo "CANONICAL CAPTURE FINISHED: $DATASET_DIR"
echo "exit code: $RC"
exit "$RC"
