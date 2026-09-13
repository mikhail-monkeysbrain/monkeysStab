#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

TCP_HOST="${MONKEYS_FC_TCP_HOST:-127.0.0.1}"
TCP_PORT="${MONKEYS_FC_TCP_PORT:-5760}"
RUN_ROOT="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}"
mkdir -p "$RUN_ROOT"
ROUTER_LOG="$RUN_ROOT/mavlink_router_gui.log"
ROUTER_PID=""
STARTED_ROUTER=0

tcp_ready() {
  python3 - "$TCP_HOST" "$TCP_PORT" <<'PY' >/dev/null 2>&1
import socket,sys
s=socket.socket()
s.settimeout(0.35)
try:
    s.connect((sys.argv[1],int(sys.argv[2])))
except OSError:
    raise SystemExit(1)
finally:
    s.close()
PY
}

cleanup() {
  if [[ "$STARTED_ROUTER" == "1" && -n "$ROUTER_PID" ]]; then
    kill "$ROUTER_PID" 2>/dev/null || true
    wait "$ROUTER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if tcp_ready; then
  echo "MAVLink router уже работает: tcp://$TCP_HOST:$TCP_PORT"
else
  echo "MAVLink router не запущен. Запускаю scripts/run_mavlink_wifi.sh..."
  : >"$ROUTER_LOG"
  MONKEYS_FC_TCP_PORT="$TCP_PORT" bash "$ROOT/scripts/run_mavlink_wifi.sh" >"$ROUTER_LOG" 2>&1 &
  ROUTER_PID=$!
  STARTED_ROUTER=1

  for _ in $(seq 1 40); do
    if tcp_ready; then break; fi
    if ! kill -0 "$ROUTER_PID" 2>/dev/null; then
      echo "ОШИБКА: MAVLink router завершился при запуске." >&2
      cat "$ROUTER_LOG" >&2
      exit 2
    fi
    sleep 0.1
  done

  if ! tcp_ready; then
    echo "ОШИБКА: локальный MAVLink TCP endpoint не открылся: tcp://$TCP_HOST:$TCP_PORT" >&2
    cat "$ROUTER_LOG" >&2
    exit 2
  fi

  sleep 0.2
  cat "$ROUTER_LOG"
fi

export MONKEYS_FC="tcp://$TCP_HOST:$TCP_PORT"

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

mkdir -p "$ROOT/build"
g++ -std=c++17 -O2 -DNDEBUG -Wno-address-of-packed-member \
  -I"$MAVLINK_ROOT" "$ROOT/src/fc_param_batch_checked.cpp" \
  -o "$ROOT/build/fc_param_tool"

if ! python3 - <<'PY' >/dev/null 2>&1
import tkinter
PY
then
  echo "ОШИБКА: не установлен python3-tk." >&2
  echo "Установите: sudo apt install python3-tk" >&2
  exit 2
fi

python3 "$ROOT/tools/sensor_geometry_gui.py"
