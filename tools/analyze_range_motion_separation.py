#!/usr/bin/env python3
"""Compare TF-Luna height-change evidence on stationary and guided vertical runs.

No XY ground truth and no threshold fitting.  Reports rolling-window height
excursions so we can see whether stationary sensor wander and real vertical
motion are naturally separable before defining any gate.
"""
import csv, sys, math
from pathlib import Path
from collections import deque

if len(sys.argv) != 3:
    raise SystemExit("usage: analyze_range_motion_separation.py STATIONARY_RUN VERTICAL_RUN")

WINDOWS=(0.25,0.5,1.0,2.0,3.0)

def load(run):
    p=Path(run)/"stationary_balanced_shadow.csv"
    if not p.exists(): raise SystemExit(f"missing: {p}")
    out=[]
    with p.open(newline="",encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                dt=float(r["dt_s"]); h=float(r["camera_height_m"])
                valid=int(r["camera_height_valid"])==1
                if valid and 0<dt<0.2 and h>0.05 and math.isfinite(h):
                    out.append((dt,h))
            except (KeyError,ValueError):
                pass
    if not out: raise SystemExit(f"no valid height rows: {p}")
    t=0.0; a=[]
    for dt,h in out:
        t+=dt; a.append((t,h))
    return a

def rolling_delta(a,w):
    q=deque(); vals=[]
    for t,h in a:
        q.append((t,h))
        while len(q)>1 and q[1][0] <= t-w: q.popleft()
        if t-q[0][0] >= 0.9*w:
            vals.append(abs(h-q[0][1]))
    return vals

def pct(v,p):
    if not v:return float("nan")
    s=sorted(v); x=(len(s)-1)*p; i=int(x); f=x-i
    return s[i]*(1-f)+s[min(i+1,len(s)-1)]*f

sta=load(sys.argv[1]); vert=load(sys.argv[2])
print("TF-LUNA MOTION SEPARATION")
print("=========================")
print(f"stationary rows={len(sta)} duration={sta[-1][0]:.1f}s")
print(f"vertical   rows={len(vert)} duration={vert[-1][0]:.1f}s")
print()
print("window | stationary p95 / p99 / max | vertical p95 / p99 / max | max-ratio")
for w in WINDOWS:
    s=rolling_delta(sta,w); v=rolling_delta(vert,w)
    sm=max(s) if s else float("nan"); vm=max(v) if v else float("nan")
    ratio=vm/sm if sm>1e-12 else float("inf")
    print(f"{w:4.2f}s | {pct(s,.95)*1000:6.2f} {pct(s,.99)*1000:6.2f} {sm*1000:6.2f} mm"
          f" | {pct(v,.95)*1000:6.2f} {pct(v,.99)*1000:6.2f} {vm*1000:6.2f} mm"
          f" | {ratio:6.2f}x")
print()
print("No gate threshold is selected here. This is separation measurement only.")
