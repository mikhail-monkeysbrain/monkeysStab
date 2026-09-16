#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CAMERA="${MONKEYS_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${MONKEYS_LUNA:-/dev/ttyAMA2}"
FC="${MONKEYS_FC:-tcp://127.0.0.1:5760}"
CAMERA_YAML="${MONKEYS_CAMERA_YAML:-$ROOT/config/ov9281_current_mount.yaml}"
RUNTIME_JSON="${MONKEYS_RUNTIME_JSON:-$ROOT/config/runtime.json}"

CFG_FOCAL_SCALE="0.931"
CFG_FEATURE_ROI="0.20 0.32 0.80 0.90"
CFG_MAX_FEATURES="500"
CFG_LOCAL_GUI="1"
if [[ -f "$RUNTIME_JSON" ]]; then
  read -r CFG_FOCAL_SCALE RX0C RY0C RX1C RY1C CFG_MAX_FEATURES CFG_LOCAL_GUI < <(
    python3 - "$RUNTIME_JSON" <<'PY'
import json,sys
with open(sys.argv[1],"r",encoding="utf-8") as f:
    d=json.load(f)
roi=d.get("feature_roi",[0.20,0.32,0.80,0.90])
print(
    d.get("focal_scale",0.931),
    roi[0],roi[1],roi[2],roi[3],
    d.get("max_features",500),
    1 if d.get("local_gui",True) else 0
)
PY
  )
  CFG_FEATURE_ROI="$RX0C $RY0C $RX1C $RY1C"
fi

FOCAL_SCALE="${MONKEYS_FOCAL_SCALE:-$CFG_FOCAL_SCALE}"
FEATURE_ROI="${MONKEYS_FEATURE_ROI:-$CFG_FEATURE_ROI}"
MAX_FEATURES="${MONKEYS_MAX_FEATURES:-$CFG_MAX_FEATURES}"
LOCAL_GUI="${MONKEYS_LOCAL_GUI:-$CFG_LOCAL_GUI}"
GEOMETRY_JSON="${MONKEYS_GEOMETRY_JSON:-$ROOT/config/mount_geometry.json}"
[[ -f "$GEOMETRY_JSON" ]] || { echo "ОШИБКА: geometry config не найден: $GEOMETRY_JSON" >&2; exit 2; }
read -r CFG_CAMERA_X CFG_CAMERA_Y CFG_CAMERA_Z CFG_RANGE_Z < <(python3 - "$GEOMETRY_JSON" <<'PY'
import json,sys
with open(sys.argv[1], "r", encoding="utf-8") as f:
    g=json.load(f)
print(g["camera"]["x"], g["camera"]["y"], g["camera"]["z"], g["rangefinder"]["z"])
PY
)
CAMERA_X_M="${MONKEYS_CAMERA_X_M:-$CFG_CAMERA_X}"
CAMERA_Y_M="${MONKEYS_CAMERA_Y_M:-$CFG_CAMERA_Y}"
CAMERA_Z_M="${MONKEYS_CAMERA_Z_M:-$CFG_CAMERA_Z}"
RANGE_Z_M="${MONKEYS_RANGE_Z_M:-$CFG_RANGE_Z}"
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
mkdir -p "$RUN_ROOT"

FREE_KB="$(df -Pk "$RUN_ROOT" | awk 'NR==2 {print $4}')"
FREE_MB=$((FREE_KB / 1024))
if (( FREE_MB < 300 )); then
  echo "ОШИБКА: недостаточно свободного места: ${FREE_MB} MB." >&2
  echo "Для запуска monkeysStab требуется минимум 300 MB." >&2
  exit 3
elif (( FREE_MB < 1024 )); then
  echo "ПРЕДУПРЕЖДЕНИЕ: на диске осталось только ${FREE_MB} MB (< 1 GB)." >&2
  echo "CSV будет автоматически остановлен при достижении лимита." >&2
fi

bash "$ROOT/scripts/audit_geometry.sh"
bash "$ROOT/scripts/audit_fc_params.sh"

[[ -e "$CAMERA" ]] || { echo "ОШИБКА: камера не найдена: $CAMERA" >&2; exit 2; }
[[ -e "$LUNA" ]] || { echo "ОШИБКА: TF-Luna не найден: $LUNA" >&2; exit 2; }
if [[ "$FC" != tcp://* ]]; then
  [[ -e "$FC" ]] || { echo "ОШИБКА: FC не найден: $FC" >&2; exit 2; }
fi
[[ -f "$CAMERA_YAML" ]] || { echo "ОШИБКА: camera YAML не найден: $CAMERA_YAML" >&2; exit 2; }
[[ -f "$MAVLINK_ROOT/ardupilotmega/mavlink.h" ]] || { echo "ОШИБКА: MAVLink headers не найдены: $MAVLINK_ROOT" >&2; exit 2; }
[[ -f "$ROOT/src/optical_flow_mavlink.cpp" ]] || {
  echo "ОШИБКА: отсутствует production source src/optical_flow_mavlink.cpp" >&2
  echo "Репозиторий перенесён не полностью. Не используйте старый jtzero-kimera как скрытую зависимость." >&2
  exit 2
}

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${MONKEYS_RUN_DIR:-$RUN_ROOT/${STAMP}_OPTICAL_FLOW}"
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

ARGS=(
  "$CAMERA" "$LUNA" "$FC" "$CSV" "$CAMERA_YAML" "$FOCAL_SCALE"
  --feature-roi "$RX0" "$RY0" "$RX1" "$RY1"
  --max-features "$MAX_FEATURES"
  --diag-camera-x-m "$CAMERA_X_M"
  --diag-camera-y-m "$CAMERA_Y_M"
  --diag-camera-z-m "$CAMERA_Z_M"
  --diag-range-z-m "$RANGE_Z_M"
)
if [[ "$LOCAL_GUI" == "1" || "$LOCAL_GUI" == "true" || "$LOCAL_GUI" == "yes" ]]; then
  ARGS+=(--rotation-gui)
fi
if [[ "${MONKEYS_RETURN_GUI:-0}" == "1" || "${MONKEYS_RETURN_GUI:-0}" == "true" || "${MONKEYS_RETURN_GUI:-0}" == "yes" ]]; then
  ARGS+=(--return-gui)
fi
if [[ "${MONKEYS_RETURN_CLI:-0}" == "1" || "${MONKEYS_RETURN_CLI:-0}" == "true" || "${MONKEYS_RETURN_CLI:-0}" == "yes" ]]; then
  ARGS+=(--return-cli)
fi
if [[ "${MONKEYS_RETURN_MANUAL_TARGET:-0}" == "1" || "${MONKEYS_RETURN_MANUAL_TARGET:-0}" == "true" || "${MONKEYS_RETURN_MANUAL_TARGET:-0}" == "yes" ]]; then
  ARGS+=(--return-manual-target)
fi
if [[ -n "${MONKEYS_DATASET_DIR:-}" ]]; then
  ARGS+=(--dataset-dir "$MONKEYS_DATASET_DIR")
fi
if [[ -n "${MONKEYS_DATASET_SURFACE:-}" ]]; then
  ARGS+=(--dataset-surface "$MONKEYS_DATASET_SURFACE")
fi
if [[ -n "${MONKEYS_DATASET_DURATION_SEC:-}" ]]; then
  ARGS+=(--dataset-duration-sec "$MONKEYS_DATASET_DURATION_SEC")
fi
if [[ -n "${MONKEYS_REMOTE_LOG:-}" ]]; then
  ARGS+=(--remote-log "$MONKEYS_REMOTE_LOG")
fi
if [[ -n "${MONKEYS_FB_SHADOW_MAX_PX:-}" ]]; then
  ARGS+=(--fb-shadow-max-px "$MONKEYS_FB_SHADOW_MAX_PX")
fi
if [[ "${MONKEYS_NO_OBS_SHADOW:-0}" == "1" ]]; then
  ARGS+=(--no-obs-shadow)
fi
if [[ -n "${MONKEYS_GUIDED_MM:-}" ]]; then
  ARGS+=(--guided-mm "$MONKEYS_GUIDED_MM")
fi
if [[ -n "${MONKEYS_CONTINUOUS_LEGS:-}" ]]; then
  ARGS+=(--continuous-legs "$MONKEYS_CONTINUOUS_LEGS")
fi
if [[ -n "${MONKEYS_PRE_STATIC_SEC:-}" ]]; then
  ARGS+=(--pre-static-sec "$MONKEYS_PRE_STATIC_SEC")
fi
if [[ -n "${MONKEYS_POST_STATIC_SEC:-}" ]]; then
  ARGS+=(--post-static-sec "$MONKEYS_POST_STATIC_SEC")
fi

exec "$BIN" "${ARGS[@]}"
