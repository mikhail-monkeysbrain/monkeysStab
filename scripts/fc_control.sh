#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
EP="${MONKEYS_FC:-tcp://127.0.0.1:5760}"

if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  for d in "$ROOT/third_party/mavlink" /usr/local/include/mavlink/v2.0 /usr/include/mavlink/v2.0; do
    if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_ROOT="$d"; break; fi
  done
fi
if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  bash "$ROOT/scripts/bootstrap_dependencies.sh"
  MAVLINK_ROOT="$ROOT/third_party/mavlink"
fi

mkdir -p "$ROOT/build"
BIN="$ROOT/build/fc_control"
SRC="$ROOT/src/fc_control.cpp"
if [[ ! -x "$BIN" || "$SRC" -nt "$BIN" ]]; then
  g++ -std=c++17 -O2 -DNDEBUG -Wno-address-of-packed-member \
    -I"$MAVLINK_ROOT" "$SRC" -o "$BIN"
fi
exec "$BIN" "$EP" "$@"
