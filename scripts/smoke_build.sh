#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  for d in /usr/local/include/mavlink/v2.0 /usr/include/mavlink/v2.0; do
    if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then
      MAVLINK_ROOT="$d"
      break
    fi
  done
fi
MAVLINK_ROOT="${MAVLINK_ROOT:-/usr/local/include/mavlink/v2.0}"

[[ -f "$MAVLINK_ROOT/ardupilotmega/mavlink.h" ]] || {
  echo "ОШИБКА: MAVLink headers не найдены. Задайте MAVLINK_ROOT." >&2
  exit 2
}

OUT="${1:-/tmp/monkeysstab_optical_flow_buildcheck}"
g++ -std=c++17 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member \
  $(pkg-config --cflags opencv4) -I"$MAVLINK_ROOT" -I"$ROOT/src" \
  "$ROOT/src/optical_flow_mavlink.cpp" -o "$OUT" \
  $(pkg-config --libs opencv4) -lpthread

echo "BUILD PASS: $OUT"
