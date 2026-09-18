#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "===== APPLY FUSED-V1 ====="
python3 tools/apply_fused_v1_shadow.py

echo
echo "===== FUSED-V1 STATE ====="
grep -n -A35 -B5 'FUSED-V1 event-driven shadow' src/optical_flow_mavlink.cpp

echo
echo "===== FUSED-V1 IMU PREDICTION ====="
grep -n -A16 -B5 'FUSED-V1 IMU velocity prediction' src/optical_flow_mavlink.cpp

echo
echo "===== FUSED-V1 VISUAL CONSUMER ====="
grep -n -A70 -B5 'FUSED-V1: event-driven visual update' src/optical_flow_mavlink.cpp

echo
echo "===== WORKED5 PRODUCER ====="
grep -n -A12 -B4 'fc.imu_cam_dN=dN' src/optical_flow_mavlink.cpp

echo
echo "===== BUILD CHECK ====="
MAVLINK_ROOT="${MAVLINK_ROOT:-$ROOT/third_party/mavlink}"
g++ -std=c++17 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member \
  $(pkg-config --cflags opencv4) \
  -I"$MAVLINK_ROOT" -I"$ROOT/src" \
  src/optical_flow_mavlink.cpp \
  -o /tmp/fused_v1_build_check \
  $(pkg-config --libs opencv4) -lpthread

echo
echo "OK: FUSED-V1 source audit and build passed"
