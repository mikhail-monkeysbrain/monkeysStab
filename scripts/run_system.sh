#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

TCP_HOST="${MONKEYS_FC_TCP_HOST:-127.0.0.1}"
TCP_PORT="${MONKEYS_FC_TCP_PORT:-5760}"
RUN_ROOT="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}"
mkdir -p "$RUN_ROOT"
ROUTER_LOG="$RUN_ROOT/mavlink_router.log"
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
  echo "Запускаю MAVLink router..."
  : >"$ROUTER_LOG"
  python3 -u "$ROOT/tools/mavlink_wifi_router.py"     --serial "${MONKEYS_FC_UART:-/dev/ttyAMA0}"     --baud "${MONKEYS_FC_BAUD:-460800}"     --tcp-port "$TCP_PORT"     --udp-port "${MONKEYS_GCS_UDP_PORT:-14550}"     ${MONKEYS_GCS_IP:+--gcs-ip "$MONKEYS_GCS_IP"}     >"$ROUTER_LOG" 2>&1 &
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
    echo "ОШИБКА: локальный MAVLink TCP endpoint не открылся." >&2
    cat "$ROUTER_LOG" >&2
    exit 2
  fi

  sleep 0.2
  cat "$ROUTER_LOG"
fi

export MONKEYS_FC="tcp://$TCP_HOST:$TCP_PORT"

echo
echo "======================================================================"
echo "monkeysStab — ПОЛНЫЙ ЗАПУСК"
echo "======================================================================"
echo "FC transport: $MONKEYS_FC"
echo "Mission Planner: UDPCl -> IP Raspberry Pi : ${MONKEYS_GCS_UDP_PORT:-14550}"
echo "UART /dev/ttyAMA0 принадлежит только MAVLink router."
echo "======================================================================"
echo

bash "$ROOT/scripts/run.sh"
