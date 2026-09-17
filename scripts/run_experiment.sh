#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CAMERA="${MONKEYS_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${MONKEYS_LUNA:-/dev/ttyAMA2}"
FC="tcp://127.0.0.1:5760"
RUN_ROOT="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$RUN_ROOT/${STAMP}_EXPERIMENT"
mkdir -p "$RUN_DIR"

ROUTER_PID=""
cleanup() {
  [[ -n "$ROUTER_PID" ]] && kill -TERM "$ROUTER_PID" 2>/dev/null || true
  [[ -n "$ROUTER_PID" ]] && wait "$ROUTER_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if fuser "$CAMERA" >/dev/null 2>&1; then
  echo "ОШИБКА: камера занята. Закрой start.sh и повтори запуск."
  exit 2
fi

if ! python3 - <<'PY'
import socket,sys
try:
    s=socket.create_connection(("127.0.0.1",5760),timeout=.15)
    s.close(); sys.exit(0)
except OSError:
    sys.exit(1)
PY
then
  bash "$ROOT/scripts/run_mavlink_wifi.sh" >"$RUN_DIR/router.log" 2>&1 &
  ROUTER_PID=$!
  for _ in {1..20}; do
    if python3 - <<'PY'
import socket,sys
try:
    s=socket.create_connection(("127.0.0.1",5760),timeout=.1)
    s.close(); sys.exit(0)
except OSError:
    sys.exit(1)
PY
    then break; fi
    sleep .1
  done
fi

CACHE_DIR="$HOME/.cache/monkeysStab"
BIN="$CACHE_DIR/monkeysstab_optical_flow"
mkdir -p "$CACHE_DIR"
SRC="$ROOT/src/optical_flow_mavlink.cpp"
if [[ ! -x "$BIN" || "$SRC" -nt "$BIN" ]]; then
  MAVLINK_ROOT="${MAVLINK_ROOT:-$ROOT/third_party/mavlink}"
  g++ -std=c++17 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member \
    $(pkg-config --cflags opencv4) -I"$MAVLINK_ROOT" -I"$ROOT/src" \
    "$SRC" -o "$BIN" $(pkg-config --libs opencv4) -lpthread \
    >"$RUN_DIR/build.log" 2>&1
fi

CSV="$RUN_DIR/optical_flow_mavlink.csv"
CAMERA_YAML="$ROOT/config/ov9281_current_mount.yaml"
read -r FOCAL RX0 RY0 RX1 RY1 MAXF < <(python3 - "$ROOT/config/runtime.json" <<'PY'
import json,sys
with open(sys.argv[1],encoding='utf-8') as f: d=json.load(f)
r=d.get('feature_roi',[.20,.32,.80,.90])
print(d.get('focal_scale',.931),*r,d.get('max_features',500))
PY
)
read -r CX CY CZ RZ < <(python3 - "$ROOT/config/mount_geometry.json" <<'PY'
import json,sys
with open(sys.argv[1],encoding='utf-8') as f: g=json.load(f)
print(g['camera']['x'],g['camera']['y'],g['camera']['z'],g['rangefinder']['z'])
PY
)

clear 2>/dev/null || true
echo "========================================"
echo " ЭКСПЕРИМЕНТ"
echo "========================================"
echo "После строки 'СИСТЕМА ГОТОВА':"
echo "SPACE  — точка A"
echo "перемести стенд"
echo "SPACE  — точка B"
echo "Ctrl+C — закончить"
echo "========================================"

# Runtime MUST stay in foreground so it owns the operator TTY and receives SPACE.
# Keep stderr in a file, but do not redirect stdin/stdout away from the terminal.
exec "$BIN" "$CAMERA" "$LUNA" "$FC" "$CSV" "$CAMERA_YAML" "$FOCAL" \
  --feature-roi "$RX0" "$RY0" "$RX1" "$RY1" --max-features "$MAXF" \
  --diag-camera-x-m "$CX" --diag-camera-y-m "$CY" --diag-camera-z-m "$CZ" \
  --diag-range-z-m "$RZ" --blind4-cli 2>"$RUN_DIR/runtime.err.log"
