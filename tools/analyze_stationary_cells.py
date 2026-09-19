#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_stationary_cells.py /path/to/optical_flow_mavlink.csv")
p=Path(sys.argv[1])

def f(r,k,d=0.0):
    try:return float(r.get(k,d))
    except (TypeError,ValueError):return d
def i(r,k,d=0):
    try:return int(float(r.get(k,d)))
    except (TypeError,ValueError):return d
def mean(v): return statistics.fmean(v) if v else 0.0
def med(v): return statistics.median(v) if v else 0.0

with p.open(newline="",encoding="utf-8",errors="replace") as fh:
    rd=csv.DictReader(fh)
    need={"mono_ns","worked5_valid","du_px","dv_px","du_norm","dv_norm",
          "worked5_du_norm","worked5_dv_norm"}
    for c in range(9):
        need |= {f"c{c}_n",f"c{c}_bx",f"c{c}_by"}
    miss=need-set(rd.fieldnames or [])
    if miss: raise SystemExit("missing columns: "+", ".join(sorted(miss)))
    rows=[r for r in rd if i(r,"worked5_valid")==1]
if not rows: raise SystemExit("no WORKED5-valid rows")
t0=f(rows[0],"mono_ns")
for r in rows:r["_t"]=(f(r,"mono_ns")-t0)/1e9
dur=max(r["_t"] for r in rows)

def report(label,rr):
    print(f"\n{label} rows={len(rr)}")
    print("cell       samples   mean_n   med_n       sum_bx_mm       sum_by_mm      mean_bx_um      mean_by_um")
    cell_sumx=cell_sumy=0.0
    for c in range(9):
        active=[r for r in rr if i(r,f"c{c}_n")>0]
        ns=[i(r,f"c{c}_n") for r in active]
        sx=sum(f(r,f"c{c}_bx") for r in active)
        sy=sum(f(r,f"c{c}_by") for r in active)
        cell_sumx+=sx; cell_sumy+=sy
        print(f"c{c} {len(active):12d} {mean(ns):8.2f} {med(ns):7.1f} "
              f"{1000*sx:+15.3f} {1000*sy:+15.3f} "
              f"{1e6*mean([f(r,f'c{c}_bx') for r in active]):+15.4f} "
              f"{1e6*mean([f(r,f'c{c}_by') for r in active]):+15.4f}")
    print(f"cell-vector sum (diagnostic, not estimator integral): bx={1000*cell_sumx:+.3f} by={1000*cell_sumy:+.3f} mm")
    du=[f(r,"du_px") for r in rr]; dv=[f(r,"dv_px") for r in rr]
    dun=[f(r,"du_norm") for r in rr]; dvn=[f(r,"dv_norm") for r in rr]
    wdu=[f(r,"worked5_du_norm") for r in rr]; wdv=[f(r,"worked5_dv_norm") for r in rr]
    print(f"production flow px mean/median: du={mean(du):+.6f}/{med(du):+.6f} dv={mean(dv):+.6f}/{med(dv):+.6f}")
    print(f"production norm mean/sum      : du={mean(dun):+.9f}/{sum(dun):+.6f} dv={mean(dvn):+.9f}/{sum(dvn):+.6f}")
    print(f"WORKED5 norm mean/sum         : du={mean(wdu):+.9f}/{sum(wdu):+.6f} dv={mean(wdv):+.9f}/{sum(wdv):+.6f}")

print("STATIONARY CELL FORENSIC")
print("========================")
print("CSV:",p)
print(f"valid rows={len(rows)} duration={dur:.1f}s")
report("WHOLE RUN",rows)
start=0.0
while start<dur+1e-9:
    end=min(dur,start+300.0)
    rr=[r for r in rows if start<=r["_t"]<end+(1e-9 if end==dur else 0)]
    if rr: report(f"{start/60:.0f}-{end/60:.0f} min",rr)
    start+=300.0

print("\nCELL LAYOUT")
print("c0 c1 c2")
print("c3 c4 c5")
print("c6 c7 c8")
print("\nREADING")
print("A coherent same-sign bias across most cells supports image-wide apparent translation.")
print("Opposing/spatially structured cell vectors with a biased WORKED5 fit supports model/geometry coupling.")
