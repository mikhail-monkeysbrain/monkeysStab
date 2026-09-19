#!/usr/bin/env python3
import csv, math, sys
from pathlib import Path
if len(sys.argv)!=2: raise SystemExit("usage: analyze_range_xy_shadow.py RUN_DIR")
p=Path(sys.argv[1])/"stationary_balanced_shadow.csv"
if not p.exists(): raise SystemExit(f"missing: {p}")
rows=[]
with p.open(newline="",encoding="utf-8") as f:
    for r in csv.DictReader(f):
        try:
            if int(r["production_valid"])!=1 or int(r["range_xy_valid"])!=1: continue
            dt=float(r["dt_s"])
            if not 0<dt<0.2: continue
            rows.append({k:float(r[k]) for k in ("dt_s","prod_du_norm","prod_dv_norm",
                "trans_du_norm","trans_dv_norm","centered_du_norm","centered_dv_norm",
                "range_du_norm","range_dv_norm","range_scale_rate","range_filtered_height_m")})
        except (KeyError,ValueError): pass
if not rows: raise SystemExit("no range-constrained XY rows")
def total(a,b):
    du=sum(r[a] for r in rows); dv=sum(r[b] for r in rows)
    return du,dv,math.hypot(du,dv)
sets=[("production","prod_du_norm","prod_dv_norm"),
      ("translation_only","trans_du_norm","trans_dv_norm"),
      ("centered_xy","centered_du_norm","centered_dv_norm"),
      ("range_xy","range_du_norm","range_dv_norm")]
print("RANGE-CONSTRAINED XY SHADOW")
print("===========================")
print(f"rows={len(rows)} duration={sum(r['dt_s'] for r in rows):.1f}s")
out={}
for name,a,b in sets:
    out[name]=total(a,b); du,dv,m=out[name]
    print(f"{name:18s} du={du:+.8f} dv={dv:+.8f} |sum|={m:.8f}")
base=out["production"][2]
if base>1e-12:
    print(f"range reduction vs production: {(1-out['range_xy'][2]/base)*100:+.2f}%")
print(f"range scale integral     : {sum(r['range_scale_rate']*r['dt_s'] for r in rows):+.8f}")
hs=[r["range_filtered_height_m"] for r in rows if r["range_filtered_height_m"]>0]
if hs: print(f"filtered height span     : {min(hs):.4f} .. {max(hs):.4f} m")
print("\nDiagnostic only. Fixed causal height filter tau=0.25 s; WORKED5/FC unchanged.")
