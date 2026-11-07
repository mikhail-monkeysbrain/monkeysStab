#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-/tmp/monkeysstab_csi_gray_capture}"
g++ -std=c++20 -O2 -pthread -I"$ROOT/src" \
  "$ROOT/tools/csi_gray_capture.cpp" -o "$OUT" \
  $(pkg-config --cflags --libs libcamera opencv4)
echo "BUILD PASS: $OUT"
