#!/usr/bin/env bash
set -euo pipefail
D="${1:-}"
if [[ -z "$D" || ! -d "$D" ]]; then echo "Usage: $0 DATASET_DIR" >&2; exit 2; fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/tools/replay_canonical_flow.cpp"
BIN="/tmp/monkeysstab_canonical_cpp_replay"
echo "===== BUILD ====="
echo "Source: $SRC"
echo "Dataset: $D"
echo "OpenCV: $(pkg-config --modversion opencv4)"
g++ -std=c++17 -O2 "$SRC" -o "$BIN" $(pkg-config --cflags --libs opencv4)
echo
echo "===== REPLAY ====="
"$BIN" "$D"
echo
echo "===== FIRST MISMATCHES ====="
python3 - "$D/canonical_cpp_replay.csv" <<'PY'
import csv,sys
p=sys.argv[1]
rows=list(csv.DictReader(open(p,newline='')))
bad=[r for r in rows if r['replay_features']!=r['prod_features'] or r['replay_tracked']!=r['prod_tracked'] or r['replay_inliers']!=r['prod_inliers']]
print("count:",len(bad))
for r in bad[:20]:
 print("frame",r["frame"],"F",r["replay_features"],r["prod_features"],"T",r["replay_tracked"],r["prod_tracked"],"I",r["replay_inliers"],r["prod_inliers"],"du",r["replay_du_norm"],r["prod_du_norm"],"dv",r["replay_dv_norm"],r["prod_dv_norm"])
PY