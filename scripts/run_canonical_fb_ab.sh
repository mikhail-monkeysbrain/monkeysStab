#!/usr/bin/env bash
set -euo pipefail
D="${1:-}"
[[ -d "$D" ]] || { echo "Usage: $0 DATASET_DIR" >&2; exit 2; }
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN=/tmp/monkeysstab_canonical_fb_ab
echo "===== BUILD ====="
g++ -std=c++17 -O2 "$ROOT/tools/canonical_fb_ab.cpp" -o "$BIN" $(pkg-config --cflags --libs opencv4)
echo "BUILD OK"
time "$BIN" "$D"
