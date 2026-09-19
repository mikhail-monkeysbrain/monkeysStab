#!/usr/bin/env python3
import csv, math, os, statistics, sys

def pct(v,p):
    if not v: return float("nan")
    x=sorted(v); k=(len(x)-1)*p; a=int(math.floor(k)); b=int(math.ceil(k))
    return x[a] if a==b else x[a]*(b-k)+x[b]*(k-a)

def main(run):
    p=os.path.join(run,"stationary_balanced_shadow.csv") if os.path.isdir(run) else run
    rows=[]
    with open(p,newline="") as f:
        for r in csv.DictReader(f):
            try:
                if int(float(r["production_valid"]))!=1 or int(float(r["holdout_valid"]))!=1: continue
                dt=float(r["dt_s"]); g=float(r["holdout_g"]); s=abs(float(r["holdout_scale"]))
                if 0<dt<.2 and math.isfinite(g) and math.isfinite(s): rows.append((g,s))
            except (KeyError,ValueError): pass
    if not rows: raise SystemExit("no valid holdout rows")
    gs=[x[0] for x in rows]; ss=[x[1] for x in rows]
    print("LONG STATIONARY HOLDOUT TAIL")
    print("============================")
    print(f"rows={len(rows)}")
    print(f"|scale| median={statistics.median(ss):.9g} p90={pct(ss,.90):.9g} p95={pct(ss,.95):.9g} p99={pct(ss,.99):.9g} p99.9={pct(ss,.999):.9g} max={max(ss):.9g}")
    print(f"G median={statistics.median(gs):+.6f} p95={pct(gs,.95):+.6f} p99={pct(gs,.99):+.6f} max={max(gs):+.6f}")
    # Frozen validation reference from the prior guided run. It is NOT a production gate.
    ref=0.00084979075
    tail=[(g,s) for g,s in rows if s>=ref]
    print(f"\nREFERENCE REGION |scale| >= {ref:.9g} (validation reference only, not a gate)")
    print(f"count={len(tail)} fraction={len(tail)/len(rows):.8f}")
    if tail:
        tg=[x[0] for x in tail]; ts=[x[1] for x in tail]
        print(f"|scale| median={statistics.median(ts):.9g} p95={pct(ts,.95):.9g} max={max(ts):.9g}")
        print(f"G median={statistics.median(tg):+.6f} p25={pct(tg,.25):+.6f} p75={pct(tg,.75):+.6f} p95={pct(tg,.95):+.6f}")
        print(f"G positive fraction={sum(x>0 for x in tg)/len(tg):.4f}")
    else:
        print("No stationary samples entered the reference region.")
    print("\nDiagnostic only. WORKED5/FC unchanged; no threshold is selected.")

if len(sys.argv)!=2:
    print("usage: analyze_long_stationary_holdout_tail.py RUN"); raise SystemExit(2)
main(sys.argv[1])
