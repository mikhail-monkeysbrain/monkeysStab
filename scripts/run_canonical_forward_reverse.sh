#!/usr/bin/env bash
set -euo pipefail
D="${1:-}"
if [[ -z "$D" || ! -d "$D" ]]; then
  echo "Usage: $0 DATASET_DIR" >&2
  exit 2
fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for f in frames.mjpgbin frames.csv optical_flow_mavlink.csv; do
  [[ -f "$D/$f" ]] || { echo "Missing: $D/$f" >&2; exit 2; }
done
BIN="/tmp/monkeysstab_canonical_forward_reverse"
echo "===== BUILD ====="
g++ -std=c++17 -O2 "$ROOT/tools/canonical_forward_reverse.cpp" -o "$BIN" $(pkg-config --cflags --libs opencv4)
echo "BUILD OK"
echo "===== SAME-FRAME FORWARD/REVERSE ====="
"$BIN" "$D"
