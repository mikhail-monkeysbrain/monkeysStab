#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "======================================================================"
echo "monkeysStab — ARMED MOTION → STOP DRIFT TEST"
echo "======================================================================"
echo "Цель: определить, где живёт послемоушн-дрейф:"
echo "  1) RAW Optical Flow продолжает движение после физической остановки"
echo "  2) RAW уже остановился, но EKF продолжает ползти"
echo
echo "ТРЕБОВАНИЯ:"
echo "  - ПРОПЕЛЛЕРЫ СНЯТЫ"
echo "  - FC должен быть ARMED до запуска теста"
echo "  - поверхность произвольная"
echo "  - FC получит СИНТЕТИЧЕСКУЮ высоту 0.60 м, чтобы ArduPilot не занулял"
echo "    optical flow до takeoff ниже 0.5 м"
echo "  - реальный TF-Luna продолжит использоваться для метрического масштаба flow"
echo
echo "ПРОТОКОЛ:"
echo "  PRE: 5 с неподвижно"
echo "  MOVE: активно двигать аппарат 20–30 с; затем ПОЛНОСТЬЮ остановить"
echo "        и сразу нажать Enter"
echo "  STOP: 15 с вообще не трогать аппарат"
echo
echo "Тест завершится сам и напечатает RAW/EKF decay summary."
echo "======================================================================"
echo

read -r -p "FC ARMED, винты сняты, готовы? [y/N]: " ans
case "${ans,,}" in
  y|yes|д|да) ;;
  *) echo "Отмена."; exit 1 ;;
esac

# The test must be self-contained. The Web service normally owns the MAVLink
# router, so closing the Web UI can also remove tcp://127.0.0.1:5760. Start a
# temporary router here when needed and stop only the router started by us.
TCP_HOST="${MONKEYS_FC_TCP_HOST:-127.0.0.1}"
TCP_PORT="${MONKEYS_FC_TCP_PORT:-5760}"
TEST_ROUTER_PID=""
STARTED_TEST_ROUTER=0
tcp_ready() {
  python3 - "$TCP_HOST" "$TCP_PORT" <<'PY' >/dev/null 2>&1
import socket,sys
s=socket.socket(); s.settimeout(0.35)
try:
    s.connect((sys.argv[1],int(sys.argv[2])))
except OSError:
    raise SystemExit(1)
finally:
    s.close()
PY
}
cleanup_router() {
  if [[ "$STARTED_TEST_ROUTER" == "1" && -n "$TEST_ROUTER_PID" ]]; then
    kill "$TEST_ROUTER_PID" 2>/dev/null || true
    wait "$TEST_ROUTER_PID" 2>/dev/null || true
  fi
}
trap cleanup_router EXIT INT TERM

if ! tcp_ready; then
  echo
  echo "MAVLink router не запущен — запускаю временный router для теста..."
  ROUTER_LOG="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}/motion_stop_router.log"
  mkdir -p "$(dirname "$ROUTER_LOG")"
  bash "$ROOT/scripts/run_mavlink_wifi.sh" >"$ROUTER_LOG" 2>&1 &
  TEST_ROUTER_PID=$!
  STARTED_TEST_ROUTER=1
  for _ in $(seq 1 60); do
    if tcp_ready; then break; fi
    if ! kill -0 "$TEST_ROUTER_PID" 2>/dev/null; then
      echo "ОШИБКА: временный MAVLink router завершился." >&2
      cat "$ROUTER_LOG" >&2
      exit 2
    fi
    sleep 0.1
  done
  if ! tcp_ready; then
    echo "ОШИБКА: tcp://$TCP_HOST:$TCP_PORT не открылся." >&2
    cat "$ROUTER_LOG" >&2
    exit 2
  fi
fi

export MONKEYS_FC="tcp://$TCP_HOST:$TCP_PORT"

# Do not compete for OV9281 with the normal Web flight runtime.
CAMERA="${MONKEYS_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
if fuser "$CAMERA" >/tmp/monkeys_motion_stop_camera_owner 2>/dev/null; then
  echo "ОШИБКА: OV9281 уже занята другим процессом:" >&2
  fuser -v "$CAMERA" >&2 || true
  echo "Остановите только flight runtime, но Web UI можно оставить открытым." >&2
  exit 2
fi

# Confirm ARMED through the same router before opening the camera.
ARMED_JSON="$(bash "$ROOT/scripts/fc_control.sh" status 2>/dev/null || true)"
if [[ -z "$ARMED_JSON" ]]; then
  echo "ОШИБКА: FC не отвечает через MAVLink router." >&2
  exit 2
fi
if ! python3 - "$ARMED_JSON" <<'PY'
import json,sys
try:
    d=json.loads(sys.argv[1].splitlines()[-1])
except Exception:
    raise SystemExit(2)
raise SystemExit(0 if d.get("armed") else 1)
PY
then
  echo "ОШИБКА: FC отвечает, но DISARMED. Заармьте FC и повторите тест." >&2
  exit 2
fi
echo "ARMED подтверждён через MAVLink."

export MONKEYS_REQUIRE_ARMED=1
export MONKEYS_GUIDED_MM=175
export MONKEYS_PRE_STATIC_SEC=5
export MONKEYS_POST_STATIC_SEC=15
export MONKEYS_LOCAL_GUI=0

# ArduPilot EKF3 intentionally forces optical-flow measurements to zero when
# takeoff has not been detected and AGL is below 0.5 m. This hand-carried,
# props-off test never triggers takeoff, so publish 0.60 m to FC only.
# The production estimator still reads real TF-Luna and run.cpp scales only
# the translational flow residual so metric motion remains based on real range.
export MONKEYS_BENCH_HEIGHT=0.60
unset MONKEYS_BENCH_TRUE_CAMERA_HEIGHT || true

before="$(find "${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}" -mindepth 1 -maxdepth 1 -type d -name '*_OPTICAL_FLOW' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2- || true)"

set +e
bash "$ROOT/scripts/run.sh"
rc=$?
set -e

after="$(find "${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}" -mindepth 1 -maxdepth 1 -type d -name '*_OPTICAL_FLOW' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2- || true)"

if [[ -z "$after" || "$after" == "$before" || ! -f "$after/optical_flow_mavlink.csv" ]]; then
  echo "ОШИБКА: новый CSV теста не найден." >&2
  exit "${rc:-2}"
fi

echo
echo "===== MOTION → STOP FORENSIC ====="
python3 "$ROOT/tools/analyze_motion_stop_drift.py" "$after/optical_flow_mavlink.csv"
echo
echo "Dataset: $after"

exit "$rc"
