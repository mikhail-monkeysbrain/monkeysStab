#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_centered_scale_shadow.py RUN_DIR")
p=Path(sys.argv[1])/"stationary_balanced_shadow.csv"
if not p.exists(): raise SystemExit(f"missing: {p}")
rows=[]
with p.open(newline="",encoding="utf-8") as f:
    for r in csv.DictReader(f):
        try:
            if int(r["production_valid"])!=1 or int(r["centered_valid"])!=1: continue
            dt=float(r["dt_s"])
            if not (0<dt<0.2): continue
            regs=[]
            ok=True
            for n in ("left","right","top","bottom"):
                if int(r[f"centered_{n}_valid"])!=1: ok=False; break
                regs.append(float(r[f"centered_{n}_scale_rate"]))
            if not ok: continue
            rows.append((dt,float(r["ts_scale_rate"]),float(r["centered_scale_rate"]),regs))
        except (KeyError,ValueError):
            pass
if not rows: raise SystemExit("no centered-scale rows")
T=sum(x[0] for x in rows)
old=sum(x[0]*x[1] for x in rows)
new=sum(x[0]*x[2] for x in rows)
sp=[max(x[3])-min(x[3]) for x in rows]
agree=[]
for _,_,g,rr in rows:
    agree.append(sum(1 for v in rr if v*g>0)/4.0)
print("CENTERED PROCRUSTES SCALE SHADOW")
print("================================")
print(f"rows={len(rows)} duration={T:.1f}s")
print(f"legacy TS integrated scale : {old:+.8f}")
print(f"centered integrated scale  : {new:+.8f}")
if abs(old)>1e-12:
    print(f"|centered| reduction       : {(1-abs(new)/abs(old))*100:+.2f}%")
print(f"centered regional spread median: {statistics.median(sp):.8e}/s")
print(f"centered regional sign agreement: {sum(agree)/len(agree):.3f}")
print("\nDiagnostic only. WORKED5 and FC output are unchanged.")
