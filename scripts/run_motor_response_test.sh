#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
MAVLINK_ROOT="${MAVLINK_ROOT:-$ROOT/third_party/mavlink}"
[[ -f "$MAVLINK_ROOT/ardupilotmega/mavlink.h" ]] || { echo "ОШИБКА: нет MAVLink headers: $MAVLINK_ROOT"; exit 2; }
mkdir -p build "$HOME/monkeysStab_runs/motor_response"
BIN="$ROOT/build/motor_response_logger"
g++ -std=c++17 -O2 -Wno-address-of-packed-member -I"$MAVLINK_ROOT" tools/motor_response_logger.cpp -o "$BIN"
TS="$(date +%Y%m%d_%H%M%S)"
OUT="$HOME/monkeysStab_runs/motor_response/motor_response_${TS}.csv"
echo "============================================================"
echo " JT-ZERO — MOTOR RESPONSE FORENSIC"
echo " Пропеллеры СНЯТЫ. Режим STABILIZE."
echo " Лог: $OUT"
echo " Ctrl+C после DISARM завершит запись."
echo "============================================================"
exec "$BIN" "${MONKEYS_FC:-tcp://127.0.0.1:5760}" "$OUT"
