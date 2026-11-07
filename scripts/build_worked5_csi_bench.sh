#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-/tmp/monkeysstab_worked5_csi_bench}"
if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  for d in "$ROOT/third_party/mavlink" /usr/local/include/mavlink/v2.0 /usr/include/mavlink/v2.0; do
    if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then
      MAVLINK_ROOT="$d"
      break
    fi
  done
fi
if [[ ! -f "${MAVLINK_ROOT:-}/ardupilotmega/mavlink.h" ]]; then
  echo "ERROR: MAVLink headers not found. Set MAVLINK_ROOT or install project dependencies." >&2
  exit 2
fi
g++ -std=c++20 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member \
  -Wno-deprecated-enum-enum-conversion \
  -I"$ROOT/src" -I"$MAVLINK_ROOT" \
  "$ROOT/tools/worked5_csi_bench.cpp" -o "$OUT" \
  $(pkg-config --cflags --libs opencv4 libcamera) -lpthread
echo "BUILD PASS: $OUT"
