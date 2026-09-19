#!/usr/bin/env python3
import csv, math, os, sys

if len(sys.argv)!=2:
    print("usage: analyze_stationary_translation_only.py RUN_DIR_or_stationary_balanced_shadow.csv")
    sys.exit(2)

p=sys.argv[1]
if os.path.isdir(p):
    p=os.path.join(p,"stationary_balanced_shadow.csv")
if not os.path.exists(p):
    raise SystemExit(f"missing: {p}")

rows=[]
with open(p,newline="") as f:
    for r in csv.DictReader(f):
        try:
            if int(r["production_valid"])!=1 or int(r["translation_only_valid"])!=1:
                continue
            rows.append({
                "t":int(r["mono_ns"])*1e-9,
                "pdu":float(r["prod_du_norm"]),"pdv":float(r["prod_dv_norm"]),
                "bdu":float(r["bal_du_norm"]),"bdv":float(r["bal_dv_norm"]),
                "tdu":float(r["trans_du_norm"]),"tdv":float(r["trans_dv_norm"]),
                "ps":float(r["prod_scale_rate"]),"pw":float(r["prod_yaw_rate"]),
                "dt":float(r["dt_s"]),
            })
        except (KeyError,ValueError):
            pass

if not rows:
    raise SystemExit("no rows with production_valid=1 and translation_only_valid=1")

def mag(du,dv): return math.hypot(du,dv)

def report(label,rr):
    if not rr: return
    pdu=sum(x["pdu"] for x in rr); pdv=sum(x["pdv"] for x in rr)
    bdu=sum(x["bdu"] for x in rr); bdv=sum(x["bdv"] for x in rr)
    tdu=sum(x["tdu"] for x in rr); tdv=sum(x["tdv"] for x in rr)
    pm,bm,tm=mag(pdu,pdv),mag(bdu,bdv),mag(tdu,tdv)
    print(f"\n{label} rows={len(rr)}")
    print(f" A production 4-param : du={pdu:+.8f} dv={pdv:+.8f} |v|={pm:.8f}")
    print(f" B balanced 4-param   : du={bdu:+.8f} dv={bdv:+.8f} |v|={bm:.8f}")
    print(f" C translation-only   : du={tdu:+.8f} dv={tdv:+.8f} |v|={tm:.8f}")
    if pm>0:
        print(f" B vs A magnitude     : {(1-bm/pm)*100:+.2f}% reduction")
        print(f" C vs A magnitude     : {(1-tm/pm)*100:+.2f}% reduction")
    print(f" A integrated scale   : {sum(x['ps']*x['dt'] for x in rr):+.8f}")
    print(f" A integrated yaw     : {sum(x['pw']*x['dt'] for x in rr):+.8f} rad")

print("STATIONARY TRANSLATION-ONLY A/B/C")
print("================================")
print("CSV:",p)
print(f"rows={len(rows)} duration={rows[-1]['t']-rows[0]['t']:.1f}s")
report("WHOLE RUN",rows)

t0=rows[0]["t"]; end=rows[-1]["t"]; k=0
while t0+k*60.0 < end:
    a=t0+k*60.0; b=min(a+60.0,end+1e-9)
    rr=[x for x in rows if a<=x["t"]<b]
    if rr: report(f"{k}-{k+1} min",rr)
    k+=1

print("\nINTERPRETATION")
print("A: unchanged production translation+scale+yaw fit.")
print("B: same 4-param model with equal total weight per occupied 3x3 cell.")
print("C: median translation of the same production RANSAC inliers, with no scale/yaw decomposition.")
print("If C is near zero while A is biased, the stationary bias is introduced mainly by 4-param decomposition.")
print("If C is similar to A, the bias already exists in the tracked/RANSAC correspondences before decomposition.")
print("This script is diagnostic only; no result is sent to the FC.")
