#!/usr/bin/env bash
set -euo pipefail
D="${1:-}"
if [[ -z "$D" || ! -d "$D" ]]; then echo "Usage: $0 DATASET_DIR" >&2; exit 2; fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/monkeysstab_canonical_cpp_replay"
g++ -std=c++17 -O2 "$ROOT/tools/replay_canonical_flow.cpp" -o "$BIN" $(pkg-config --cflags --libs opencv4)
for FS in 0.931 1.000 1.0925 1.10561; do
  echo "===== focal_scale=$FS ====="
  "$BIN" "$D" "$FS" | tail -3
done
python3 - "$D" <<'PY'
import csv,math,sys
from pathlib import Path
D=Path(sys.argv[1])
with (D/"optical_flow_mavlink.csv").open(newline="") as f: prod=list(csv.DictReader(f))
events={}
for r in prod:
    try:e=int(float(r["return_event"]))
    except:e=0
    if e in (1,2,3) and e not in events: events[e]=int(float(r["frame"]))
A,B=events[1],events[2]
print("\n===== FOCAL SCALE SWEEP: A->B NATIVE CAMERA/BODY TRANSLATION =====")
print("Uses replay du/dv on exact production inliers and hcam=range_to_fc+0.005.")
print("No GT fitting; listed scales were selected before this calculation.")
byframe={int(float(r["frame"])):r for r in prod}
for p in sorted(D.glob("canonical_cpp_replay_fs_*.csv")):
    fs=float(p.stem.rsplit("_",1)[1]); x=y=0.; n=0
    with p.open(newline="") as f:
      for r in csv.DictReader(f):
        fr=int(r["frame"])
        if not (A < fr <= B) or r["replay_valid"]!="1": continue
        pr=byframe.get(fr)
        if not pr: continue
        dt=float(r["dt_s"]); h=float(pr["range_to_fc_m"])+.005
        # Current mount: Cx->+Y_FRD, Cy->-X_FRD.
        # flow_cam=(dv/dt,-du/dt), hence body_xy=(du/dt,dv/dt).
        x += float(r["replay_du_norm"])*h
        y += float(r["replay_dv_norm"])*h
        n+=1
    mag=math.hypot(x,y)
    print(f"fs={fs:.6f}  X={x*1000:+.3f}  Y={y*1000:+.3f}  mag={mag*1000:.3f} mm  samples={n}")
PY
