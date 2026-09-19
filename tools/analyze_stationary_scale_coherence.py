#!/usr/bin/env python3
import csv, math, os, statistics, sys

if len(sys.argv)!=2:
    print("usage: analyze_stationary_scale_coherence.py RUN_DIR_or_stationary_balanced_shadow.csv")
    sys.exit(2)

p=sys.argv[1]
if os.path.isdir(p):
    p=os.path.join(p,"stationary_balanced_shadow.csv")
if not os.path.exists(p):
    raise SystemExit(f"missing: {p}")

regions=("left","right","top","bottom")
rows=[]
with open(p,newline="") as f:
    for r in csv.DictReader(f):
        try:
            if int(r["production_valid"])!=1:
                continue
            item={
                "t":int(r["mono_ns"])*1e-9,
                "dt":float(r["dt_s"]),
                "global":float(r["ts_scale_rate"]),
            }
            ok=True
            for name in regions:
                item[name+"_valid"]=int(r["scale_"+name+"_valid"])
                item[name+"_n"]=int(r["scale_"+name+"_n"])
                item[name]=float(r["scale_"+name+"_rate"])
                if item[name+"_valid"]!=1:
                    ok=False
            if ok:
                rows.append(item)
        except (KeyError,ValueError):
            pass

if not rows:
    raise SystemExit("no rows with all four regional scale fits valid")

def corr(a,b):
    n=len(a)
    if n<2: return float("nan")
    ma=sum(a)/n; mb=sum(b)/n
    da=[x-ma for x in a]; db=[x-mb for x in b]
    va=sum(x*x for x in da); vb=sum(x*x for x in db)
    if va<=0 or vb<=0: return float("nan")
    return sum(x*y for x,y in zip(da,db))/math.sqrt(va*vb)

def report(label,rr):
    if not rr: return
    integ={k:sum(x[k]*x["dt"] for x in rr) for k in ("global",)+regions}
    means={k:sum(x[k] for x in rr)/len(rr) for k in ("global",)+regions}
    counts={k:sum(x[k+"_n"] for x in rr)/len(rr) for k in regions}
    per_frame_spread=[]
    per_frame_sign_agree=[]
    for x in rr:
        vals=[x[k] for k in regions]
        per_frame_spread.append(max(vals)-min(vals))
        g=x["global"]
        if abs(g)>1e-12:
            per_frame_sign_agree.append(sum(1 for v in vals if v*g>0)/4.0)
    print(f"\n{label} rows={len(rr)}")
    print(" integrated scale:")
    print(f"   global={integ['global']:+.8f}")
    for k in regions:
        print(f"   {k:6s}={integ[k]:+.8f}   mean_n={counts[k]:.1f}")
    print(" mean scale rate (/s):")
    print(f"   global={means['global']:+.8e}")
    for k in regions:
        print(f"   {k:6s}={means[k]:+.8e}")
    print(f" median per-frame regional spread={statistics.median(per_frame_spread):.8e}/s")
    if per_frame_sign_agree:
        print(f" mean fraction of regions with global sign={sum(per_frame_sign_agree)/len(per_frame_sign_agree):.3f}")
    print(f" corr(global,left)  ={corr([x['global'] for x in rr],[x['left'] for x in rr]):+.4f}")
    print(f" corr(global,right) ={corr([x['global'] for x in rr],[x['right'] for x in rr]):+.4f}")
    print(f" corr(global,top)   ={corr([x['global'] for x in rr],[x['top'] for x in rr]):+.4f}")
    print(f" corr(global,bottom)={corr([x['global'] for x in rr],[x['bottom'] for x in rr]):+.4f}")

print("STATIONARY SCALE COHERENCE")
print("==========================")
print("CSV:",p)
print(f"rows={len(rows)} duration={rows[-1]['t']-rows[0]['t']:.1f}s")
print("Regions use the same production RANSAC inliers and independent translation+scale fits.")
report("WHOLE RUN",rows)

t0=rows[0]["t"]; end=rows[-1]["t"]; k=0
while t0+k*60.0 < end:
    a=t0+k*60.0; b=min(a+60.0,end+1e-9)
    rr=[x for x in rows if a<=x["t"]<b]
    if rr: report(f"{k}-{k+1} min",rr)
    k+=1

print("\nINTERPRETATION")
print("A real isotropic image-scale signal should be spatially coherent: regional fits should usually agree in sign and track one another.")
print("Persistent large disagreement/opposite signs supports the hypothesis that global scale is absorbing structured non-scale image deformation.")
print("No coherence threshold is applied here; this is measurement only.")
print("Diagnostic only; production WORKED5 and MAVLink flow are unchanged.")
