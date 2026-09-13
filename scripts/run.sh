#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CAMERA="${MONKEYS_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${MONKEYS_LUNA:-/dev/ttyAMA2}"
FC="${MONKEYS_FC:-/dev/ttyAMA0}"
CAMERA_YAML="${MONKEYS_CAMERA_YAML:-$ROOT/config/ov9281_current_mount.yaml}"
FOCAL_SCALE="${MONKEYS_FOCAL_SCALE:-0.931}"
FEATURE_ROI="${MONKEYS_FEATURE_ROI:-0.20 0.32 0.80 0.90}"
MAX_FEATURES="${MONKEYS_MAX_FEATURES:-500}"
CAMERA_Z_M="${MONKEYS_CAMERA_Z_M:-0.050}"
RANGE_Z_M="${MONKEYS_RANGE_Z_M:-0.055}"
if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  for d in "$ROOT/third_party/mavlink" /usr/local/include/mavlink/v2.0 /usr/include/mavlink/v2.0; do
    if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then
      MAVLINK_ROOT="$d"
      break
    fi
  done
fi
if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  bash "$ROOT/scripts/bootstrap_dependencies.sh"
  MAVLINK_ROOT="$ROOT/third_party/mavlink"
fi
export MAVLINK_ROOT
RUN_ROOT="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}"

bash "$ROOT/scripts/audit_geometry.sh"
bash "$ROOT/scripts/audit_fc_params.sh"

[[ -e "$CAMERA" ]] || { echo "ОШИБКА: камера не найдена: $CAMERA" >&2; exit 2; }
[[ -e "$LUNA" ]] || { echo "ОШИБКА: TF-Luna не найден: $LUNA" >&2; exit 2; }
[[ -e "$FC" ]] || { echo "ОШИБКА: FC не найден: $FC" >&2; exit 2; }
[[ -f "$CAMERA_YAML" ]] || { echo "ОШИБКА: camera YAML не найден: $CAMERA_YAML" >&2; exit 2; }
[[ -f "$MAVLINK_ROOT/ardupilotmega/mavlink.h" ]] || { echo "ОШИБКА: MAVLink headers не найдены: $MAVLINK_ROOT" >&2; exit 2; }
[[ -f "$ROOT/src/optical_flow_mavlink.cpp" ]] || {
  echo "ОШИБКА: отсутствует production source src/optical_flow_mavlink.cpp" >&2
  echo "Репозиторий перенесён не полностью. Не используйте старый jtzero-kimera как скрытую зависимость." >&2
  exit 2
}

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$RUN_ROOT/${STAMP}_OPTICAL_FLOW"
mkdir -p "$RUN_DIR"
BIN="$RUN_DIR/monkeysstab_optical_flow"
CSV="$RUN_DIR/optical_flow_mavlink.csv"
BUILD_LOG="$RUN_DIR/build.log"

read -r RX0 RY0 RX1 RY1 <<< "$FEATURE_ROI"

if ! g++ -std=c++17 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member \
  $(pkg-config --cflags opencv4) -I"$MAVLINK_ROOT" -I"$ROOT/src" \
  "$ROOT/src/optical_flow_mavlink.cpp" -o "$BIN" \
  $(pkg-config --libs opencv4) -lpthread >"$BUILD_LOG" 2>&1; then
  echo "ОШИБКА СБОРКИ. Последние 80 строк:"
  tail -80 "$BUILD_LOG"
  exit 1
fi

cat <<EOF
======================================================================
monkeysStab — OPTICAL FLOW
======================================================================
OV9281 -> OPTICAL_FLOW -> ArduPilot EKF3
TF-Luna -> DISTANCE_SENSOR

camera: $CAMERA
luna:   $LUNA
fc:     $FC
CSV:    $CSV

focal_scale: $FOCAL_SCALE
feature ROI: $FEATURE_ROI
max features: $MAX_FEATURES

Launcher НЕ ARM-ит FC и НЕ переключает режим полёта.
======================================================================
EOF

exec "$BIN" "$CAMERA" "$LUNA" "$FC" "$CSV" "$CAMERA_YAML" "$FOCAL_SCALE" \
  --feature-roi "$RX0" "$RY0" "$RX1" "$RY1" \
  --max-features "$MAX_FEATURES" \
  --rotation-gui \
  --diag-camera-z-m "$CAMERA_Z_M" \
  --diag-range-z-m "$RANGE_Z_M"
