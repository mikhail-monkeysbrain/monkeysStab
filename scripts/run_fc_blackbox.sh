#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
EP="${MONKEYS_FC:-tcp://127.0.0.1:5760}"
RUN_ROOT="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}"
mkdir -p "$RUN_ROOT" "$ROOT/build"
OUT="${MONKEYS_FC_BLACKBOX:-$RUN_ROOT/continuous_fc.csv}"
BIN="$ROOT/build/fc_blackbox_logger"
SRC="$ROOT/src/fc_blackbox_logger.cpp"

if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  for d in "$ROOT/third_party/mavlink" /usr/local/include/mavlink/v2.0 /usr/include/mavlink/v2.0; do
    if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_ROOT="$d"; break; fi
  done
fi
if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  bash "$ROOT/scripts/bootstrap_dependencies.sh"
  MAVLINK_ROOT="$ROOT/third_party/mavlink"
fi

if [[ ! -x "$BIN" || "$SRC" -nt "$BIN" ]]; then
  g++ -std=c++17 -O2 -DNDEBUG -Wno-address-of-packed-member -I"$MAVLINK_ROOT" "$SRC" -o "$BIN"
fi

exec "$BIN" "$EP" "$OUT"
