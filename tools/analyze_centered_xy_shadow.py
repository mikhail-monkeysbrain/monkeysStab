#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_centered_xy_shadow.py RUN_DIR")
p=Path(sys.argv[1])/"stationary_balanced_shadow.csv"
if not p.exists(): raise SystemExit(f"missing: {p}")
rows=[]
with p.open(newline="",encoding="utf-8") as f:
    for r in csv.DictReader(f):
        try:
            if int(r["production_valid"])!=1 or int(r["centered_xy_valid"])!=1: continue
            dt=float(r["dt_s"])
            if not (0<dt<0.2): continue
            rows.append((dt,float(r["prod_du_norm"]),float(r["prod_dv_norm"]),
                         float(r["trans_du_norm"]),float(r["trans_dv_norm"]),
                         float(r["centered_du_norm"]),float(r["centered_dv_norm"])))
        except (KeyError,ValueError): pass
if not rows: raise SystemExit("no centered XY rows")
def integ(ix):
    return sum(r[0]*r[ix]/r[0] for r in rows)
# du/dv are per-frame normalized increments, so integrate by direct sum.
vals={}
for name,i,j in (("production",1,2),("translation_only",3,4),("centered_xy",5,6)):
    du=sum(r[i] for r in rows); dv=sum(r[j] for r in rows)
    vals[name]=(du,dv,math.hypot(du,dv))
print("CENTERED SIMILARITY XY SHADOW")
print("=============================")
print(f"rows={len(rows)} duration={sum(r[0] for r in rows):.1f}s")
for name,(du,dv,m) in vals.items():
    print(f"{name:18s} du={du:+.8f} dv={dv:+.8f} |sum|={m:.8f}")
a=vals["production"][2]; c=vals["centered_xy"][2]
if a>1e-12: print(f"centered reduction vs production: {(1-c/a)*100:+.2f}%")
print("\nNormalized image displacement only; diagnostic shadow, not metric WORKED5.")
