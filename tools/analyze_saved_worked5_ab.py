#!/usr/bin/env python3
import csv
import math
import sys
from pathlib import Path

def f(row, key, default=float("nan")):
    try:
        v=row.get(key, "")
        return float(v) if v not in ("", None) else default
    except (TypeError, ValueError):
        return default

def main():
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} DATASET_DIR", file=sys.stderr)
        return 2
    root=Path(sys.argv[1])
    path=root/"optical_flow_mavlink.csv"
    if not path.exists():
        raise SystemExit(f"ERROR: not found: {path}")

    events={}
    rows=0
    w5_valid=0
    with path.open(newline="") as fh:
        rd=csv.DictReader(fh)
        required={"return_event","worked5_valid","worked5_dN_m","worked5_dE_m","worked5_pN_m","worked5_pE_m"}
        missing=required-set(rd.fieldnames or [])
        if missing:
            raise SystemExit("ERROR: CSV lacks frozen WORKED5 fields: "+", ".join(sorted(missing)))
        for row in rd:
            rows+=1
            if int(f(row,"worked5_valid",0)) == 1:
                w5_valid+=1
            ev=int(f(row,"return_event",0))
            if ev:
                events[ev]={
                    "frame": int(f(row,"frame",rows)),
                    "pN": f(row,"worked5_pN_m"),
                    "pE": f(row,"worked5_pE_m"),
                }

    if 11 not in events or 12 not in events:
        raise SystemExit(f"ERROR: need return_event 11 and 12, found {sorted(events)}")
    a,b=events[11],events[12]
    dN=b["pN"]-a["pN"]
    dE=b["pE"]-a["pE"]
    mag=math.hypot(dN,dE)
    print("="*72)
    print("FROZEN WORKED5 ONLINE — SAVED A -> B")
    print(f"dataset: {root}")
    print(f"rows={rows} worked5_valid={w5_valid}")
    print(f"A event=11 frame={a['frame']} pNE=({a['pN']*1000:.3f}, {a['pE']*1000:.3f}) mm")
    print(f"B event=12 frame={b['frame']} pNE=({b['pN']*1000:.3f}, {b['pE']*1000:.3f}) mm")
    print(f"A -> B dNE=({dN*1000:+.3f}, {dE*1000:+.3f}) mm")
    print(f"A -> B magnitude={mag*1000:.3f} mm")
    print("="*72)
    return 0

if __name__=="__main__":
    raise SystemExit(main())
