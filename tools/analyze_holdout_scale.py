#!/usr/bin/env python3
import csv, math, os, sys, statistics

def load(path):
    if os.path.isdir(path):
        path=os.path.join(path,"stationary_balanced_shadow.csv")
    rows=[]
    with open(path,newline="") as f:
        for r in csv.DictReader(f):
            try:
                if int(float(r["production_valid"]))!=1 or int(float(r["holdout_valid"]))!=1:
                    continue
                g=float(r["holdout_g"]); e0=float(r["holdout_e0"]); e1=float(r["holdout_e1"])
                sc=float(r["holdout_scale"]); dt=float(r["dt_s"])
                if all(map(math.isfinite,[g,e0,e1,sc,dt])) and 0<dt<0.2:
                    rows.append((g,e0,e1,sc,dt))
            except (KeyError,ValueError):
                pass
    return rows

def pct(v,p):
    if not v:return float("nan")
    x=sorted(v); k=(len(x)-1)*p; a=int(math.floor(k)); b=int(math.ceil(k))
    return x[a] if a==b else x[a]*(b-k)+x[b]*(k-a)

def report(name,rows):
    print(f"\n{name}: rows={len(rows)}")
    if not rows:return
    g=[x[0] for x in rows]; e0=[x[1] for x in rows]; e1=[x[2] for x in rows]
    sc=[x[3] for x in rows]
    print(f"G median={statistics.median(g):+.6f} p05={pct(g,.05):+.6f} p25={pct(g,.25):+.6f} p75={pct(g,.75):+.6f} p95={pct(g,.95):+.6f}")
    print(f"G positive fraction={sum(x>0 for x in g)/len(g):.4f}")
    print(f"E0 median={statistics.median(e0):.9g}  E1 median={statistics.median(e1):.9g}")
    print(f"holdout scale median={statistics.median(sc):+.9g}")
    # Signal-conditioned summaries prevent 'large motion is easier' from being
    # mistaken for spatial coherence. Fixed bins, no fitted threshold.
    amag=[abs(x) for x in sc]
    cuts=[pct(amag,.25),pct(amag,.50),pct(amag,.75)]
    lo=0.0
    for j,hi in enumerate(cuts+[float("inf")]):
        z=[g[i] for i,a in enumerate(amag) if a>=lo and a<(hi if math.isfinite(hi) else float("inf"))]
        if z: print(f"|scale| Q{j+1}: n={len(z)} G_med={statistics.median(z):+.6f} G_pos={sum(x>0 for x in z)/len(z):.3f}")
        lo=hi

if len(sys.argv)!=3:
    print("usage: analyze_holdout_scale.py STATIONARY_RUN VERTICAL_RUN")
    raise SystemExit(2)
a=load(sys.argv[1]); b=load(sys.argv[2])
print("SPATIAL HOLDOUT SCALE SHADOW")
print("============================")
report("STATIONARY",a)
report("VERTICAL",b)
print("\nDiagnostic only. No gate threshold is selected; WORKED5/FC unchanged.")
