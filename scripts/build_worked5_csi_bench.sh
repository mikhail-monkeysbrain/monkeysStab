#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAVLINK_ROOT="$ROOT/third_party/mavlink/include/mavlink/v2.0"
OUT="${1:-/tmp/monkeysstab_worked5_csi_bench}"
g++ -std=c++20 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member \
  -Wno-deprecated-enum-enum-conversion \
  -I"$ROOT/src" -I"$MAVLINK_ROOT" \
  "$ROOT/tools/worked5_csi_bench.cpp" -o "$OUT" \
  $(pkg-config --cflags --libs opencv4 libcamera) -lpthread
echo "BUILD PASS: $OUT"
