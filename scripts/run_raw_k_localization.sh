#!/usr/bin/env bash
set -euo pipefail
D="${1:-/home/vio/monkeysStab_datasets/20260916_192841_ASTRA_AB_RAW}"
GT="${2:-292}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN=/tmp/monkeys_replay_canonical_flow
echo "======================================================================"
echo "monkeysStab — OFFLINE RAW SAME-PAIR K/D LOCALIZATION"
echo "Dataset: $D"
echo "GT: $GT mm"
echo "Ничего в production/FC не изменяется."
echo "======================================================================"
for f in frames.csv frames.mjpgbin optical_flow_mavlink.csv; do
  [[ -s "$D/$f" ]] || { echo "ОШИБКА: нет $D/$f"; exit 2; }
done
g++ -std=c++17 -O2 "$ROOT/tools/replay_canonical_flow.cpp" -o "$BIN" $(pkg-config --cflags --libs opencv4)
python3 "$ROOT/tools/analyze_raw_k_sweep.py" "$D" "$GT" "$BIN"
