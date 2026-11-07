#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-/tmp/monkeysstab_worked5_csi_bench}"
DIR="${TMPDIR:-/tmp}/monkeysstab_csi_parts"
mkdir -p "$DIR"
FLAGS=(-std=c++20 -O0 -g0 -pthread -Wno-deprecated-enum-enum-conversion -I"$ROOT/src")
read -r -a CVFLAGS <<< "$(pkg-config --cflags opencv4)"
read -r -a LCFLAGS <<< "$(pkg-config --cflags libcamera)"
echo "STEP 1/3: WORKED5 estimator only"
g++ "${FLAGS[@]}" "${CVFLAGS[@]}" -c "$ROOT/src/worked5_flow_isolated.cpp" -o "$DIR/worked5_flow.o"
echo "STEP 2/3: CSI capture and bench"
g++ "${FLAGS[@]}" "${CVFLAGS[@]}" "${LCFLAGS[@]}" -c "$ROOT/tools/worked5_csi_bench.cpp" -o "$DIR/csi_bench.o"
echo "STEP 3/3: Link"
g++ "$DIR/worked5_flow.o" "$DIR/csi_bench.o" -o "$OUT" $(pkg-config --libs opencv4 libcamera) -pthread
echo "BUILD PASS: $OUT"
