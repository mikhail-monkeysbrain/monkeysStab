#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path
if len(sys.argv)!=2: raise SystemExit("usage: analyze_stationary_fit_coupling.py /path/to/optical_flow_mavlink.csv")
p=Path(sys.argv[1])
def f(r,k,d=0.0):
    try:return float(r.get(k,d))
    except:return d
def i(r,k,d=0):
    try:return int(float(r.get(k,d)))
    except:return d
def mean(v):return statistics.fmean(v) if v else 0.0
def med(v):return statistics.median(v) if v else 0.0
with p.open(newline="",encoding="utf-8",errors="replace") as fh:
    rd=csv.DictReader(fh)
    need={"mono_ns","worked5_valid","dt_s","du_norm","dv_norm","scale_rate","yaw_rate_cam_z",
          "worked5_du_norm","worked5_dv_norm"}
    miss=need-set(rd.fieldnames or [])
    if miss: raise SystemExit("missing columns: "+", ".join(sorted(miss)))
    rows=[r for r in rd if i(r,"worked5_valid")==1 and f(r,"dt_s")>0]
if not rows: raise SystemExit("no valid rows")
t0=f(rows[0],"mono_ns")
for r in rows:r["_t"]=(f(r,"mono_ns")-t0)/1e9
dur=max(r["_t"] for r in rows)

def rep(label,rr):
    tx=[f(r,"du_norm") for r in rr]; ty=[f(r,"dv_norm") for r in rr]
    sc=[f(r,"scale_rate") for r in rr]; wz=[f(r,"yaw_rate_cam_z") for r in rr]
    sdt=[f(r,"scale_rate")*f(r,"dt_s") for r in rr]
    wdt=[f(r,"yaw_rate_cam_z")*f(r,"dt_s") for r in rr]
    wtx=[f(r,"worked5_du_norm") for r in rr]; wty=[f(r,"worked5_dv_norm") for r in rr]
    print(f"\n{label} rows={len(rr)}")
    print(f" fit translation sum: tx={sum(tx):+.8f} ty={sum(ty):+.8f}")
    print(f" WORKED5 sum        : du={sum(wtx):+.8f} dv={sum(wty):+.8f}")
    print(f" scale: mean_rate={mean(sc):+.8e}/s median_rate={med(sc):+.8e}/s sum(s*dt)={sum(sdt):+.8f}")
    print(f" yaw  : mean_rate={mean(wz):+.8e}rad/s median_rate={med(wz):+.8e}rad/s sum(w*dt)={sum(wdt):+.8f}rad ({math.degrees(sum(wdt)):+.4f}deg)")
    # Sign/co-movement diagnostic, not causality.
    def corr(a,b):
        if len(a)<2:return 0.0
        ma,mb=mean(a),mean(b); da=[x-ma for x in a]; db=[x-mb for x in b]
        den=math.sqrt(sum(x*x for x in da)*sum(y*y for y in db))
        return sum(x*y for x,y in zip(da,db))/den if den else 0.0
    print(f" corr(tx,scale_rate)={corr(tx,sc):+.4f} corr(ty,scale_rate)={corr(ty,sc):+.4f}")
    print(f" corr(tx,yaw_rate)  ={corr(tx,wz):+.4f} corr(ty,yaw_rate)  ={corr(ty,wz):+.4f}")

print("STATIONARY 4-PARAMETER FIT COUPLING")
print("===================================")
print("CSV:",p)
print(f"rows={len(rows)} duration={dur:.1f}s")
rep("WHOLE RUN",rows)
s=0.0
while s<dur+1e-9:
    e=min(dur,s+300)
    rr=[r for r in rows if s<=r["_t"]<e+(1e-9 if e==dur else 0)]
    if rr:rep(f"{s/60:.0f}-{e/60:.0f} min",rr)
    s+=300
print("\nNOTE")
print("scale_rate and yaw_rate_cam_z are the production 4-parameter fit components, not FC gyro.")
print("Correlation is descriptive only; it does not prove that scale/yaw causes translation.")
