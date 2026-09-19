#!/usr/bin/env python3
import csv
import math
import sys
from pathlib import Path

def num(r, k, d=0.0):
    try: return float(r.get(k, d))
    except (TypeError, ValueError): return d

def integer(r, k, d=0):
    try: return int(float(r.get(k, d)))
    except (TypeError, ValueError): return d

def mm(n, e):
    return 1000.0 * math.hypot(n, e)

if len(sys.argv) != 2:
    raise SystemExit("usage: analyze_false_bad_stationary.py /path/to/fused_v2_realtime_shadow.csv")

path = Path(sys.argv[1])
with path.open(newline="", encoding="utf-8", errors="replace") as f:
    rows = list(csv.DictReader(f))
if not rows:
    raise SystemExit("empty CSV")

bad_idx = [i for i,r in enumerate(rows)
           if num(r,"inlier_ratio",1.0) < 0.50 and integer(r,"inliers",999999) < 100]
print("FALSE-BAD FORENSIC")
print("===================")
print("CSV:", path)
print("rows:", len(rows), "BAD frames:", len(bad_idx))

events=[]
seen=set()
for i,r in enumerate(rows):
    ev=integer(r,"event_count")
    if ev>0 and ev not in seen:
        seen.add(ev); events.append((ev,i))

for ev,i0 in events:
    r0=rows[i0]
    badf=integer(r0,"bad_frame")
    anchf=integer(r0,"anchor_frame")
    ibad=next((i for i,r in enumerate(rows) if integer(r,"frame")==badf), i0)
    ia=next((i for i,r in enumerate(rows) if integer(r,"frame")==anchf), max(0,ibad-20))
    # recovery = first transition bridge 1 -> 0 after bad
    ir=None
    for j in range(ibad+1,len(rows)):
        if integer(rows[j-1],"bridge")==1 and integer(rows[j],"bridge")==0:
            ir=j; break
    if ir is None: ir=min(len(rows)-1, ibad+120)

    pre=max(0,ibad-120); post=min(len(rows)-1,ir+120)
    print(f"\nEVENT {ev}: anchor={anchf} bad={badf} recovery~={integer(rows[ir],'frame')}")
    print("frame  rel_ms  valid reason tracked inliers ratio dt_ms newW5  w5_dNmm w5_dEmm bridge imuAge shadowNmm shadowEmm endpointmm")
    tbad=num(rows[ibad],"cam_ns")
    for j in range(pre,post+1):
        r=rows[j]
        interesting=(j>=ibad-8 and j<=ir+8) or j==ia
        if not interesting: continue
        rel=(num(r,"cam_ns")-tbad)/1e6
        print(f"{integer(r,'frame'):5d} {rel:7.1f} {integer(r,'production_valid'):5d} "
              f"{integer(r,'invalid_reason'):6d} {integer(r,'tracked'):7d} {integer(r,'inliers'):7d} "
              f"{num(r,'inlier_ratio'):5.3f} {1000*num(r,'dt_s'):6.2f} {integer(r,'new_w5'):5d} "
              f"{1000*num(r,'w5_dN_m'):8.3f} {1000*num(r,'w5_dE_m'):8.3f} {integer(r,'bridge'):6d} "
              f"{num(r,'imu_age_ms'):6.2f} {1000*num(r,'shadow_n_m'):9.3f} "
              f"{1000*num(r,'shadow_e_m'):9.3f} {1000*num(r,'shadow_endpoint_m'):10.3f}")

    a=rows[ia]; b=rows[ibad]; q=rows[ir]
    removed_n=removed_e=0.0
    for r in rows[ia+1:ir+1]:
        if integer(r,"new_w5"):
            removed_n += num(r,"w5_dN_m")
            removed_e += num(r,"w5_dE_m")
    shadow_bridge_n=num(q,"shadow_n_m")-num(a,"shadow_n_m")
    shadow_bridge_e=num(q,"shadow_e_m")-num(a,"shadow_e_m")
    print("\nEVENT DELTAS")
    print(f"anchor->recovery removed W5: dN={1000*removed_n:.3f} dE={1000*removed_e:.3f} |dXY|={mm(removed_n,removed_e):.3f} mm")
    print(f"anchor->recovery V2 shadow: dN={1000*shadow_bridge_n:.3f} dE={1000*shadow_bridge_e:.3f} |dXY|={mm(shadow_bridge_n,shadow_bridge_e):.3f} mm")
    print(f"replacement delta (V2-W5): dN={1000*(shadow_bridge_n-removed_n):.3f} "
          f"dE={1000*(shadow_bridge_e-removed_e):.3f} "
          f"|vector|={mm(shadow_bridge_n-removed_n,shadow_bridge_e-removed_e):.3f} mm")

# Whole-run decomposition: sum W5 increments vs final V2 shadow displacement.
w5n=sum(num(r,"w5_dN_m") for r in rows if integer(r,"new_w5"))
w5e=sum(num(r,"w5_dE_m") for r in rows if integer(r,"new_w5"))
v2n=num(rows[-1],"shadow_n_m")-num(rows[0],"shadow_n_m")
v2e=num(rows[-1],"shadow_e_m")-num(rows[0],"shadow_e_m")
print("\nWHOLE RUN")
print(f"W5 summed: dN={1000*w5n:.3f} dE={1000*w5e:.3f} |dXY|={mm(w5n,w5e):.3f} mm")
print(f"V2 shadow: dN={1000*v2n:.3f} dE={1000*v2e:.3f} |dXY|={mm(v2n,v2e):.3f} mm")
print(f"V2-W5 vector: dN={1000*(v2n-w5n):.3f} dE={1000*(v2e-w5e):.3f} |vector|={mm(v2n-w5n,v2e-w5e):.3f} mm")

# Time-window decomposition. Uses camera monotonic timestamps, so each row is
# assigned to a real elapsed-time window rather than an equal row-count bucket.
WINDOW_S = 300.0
t0_ns = num(rows[0], "cam_ns")
t1_ns = num(rows[-1], "cam_ns")
duration_s = max(0.0, (t1_ns - t0_ns) / 1e9)
print("\n5-MINUTE WINDOWS")
print("window        frames   BAD  events    W5_dN    W5_dE   W5_|d|    V2_dN    V2_dE   V2_|d|   V2-W5")
start_s = 0.0
while start_s < duration_s + 1e-9:
    end_s = min(duration_s, start_s + WINDOW_S)
    idx = [i for i,r in enumerate(rows)
           if start_s <= (num(r,"cam_ns")-t0_ns)/1e9 < end_s + (1e-9 if end_s == duration_s else 0.0)]
    if not idx:
        start_s += WINDOW_S
        continue
    i0, i1 = idx[0], idx[-1]
    wr = rows[i0:i1+1]
    wn = sum(num(r,"w5_dN_m") for r in wr if integer(r,"new_w5"))
    we = sum(num(r,"w5_dE_m") for r in wr if integer(r,"new_w5"))
    sn = num(rows[i1],"shadow_n_m") - num(rows[i0],"shadow_n_m")
    se = num(rows[i1],"shadow_e_m") - num(rows[i0],"shadow_e_m")
    bad = sum(1 for r in wr if num(r,"inlier_ratio",1.0)<0.50 and integer(r,"inliers",999999)<100)
    evs = len({integer(r,"event_count") for r in wr if integer(r,"event_count")>0})
    print(f"{start_s/60:4.0f}-{end_s/60:4.0f} min {len(wr):7d} {bad:5d} {evs:7d} "
          f"{1000*wn:8.3f} {1000*we:8.3f} {mm(wn,we):8.3f} "
          f"{1000*sn:8.3f} {1000*se:8.3f} {mm(sn,se):8.3f} "
          f"{mm(sn-wn,se-we):7.3f}")
    start_s += WINDOW_S
