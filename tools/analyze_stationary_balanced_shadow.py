#!/usr/bin/env python3
import csv, math, os, sys

if len(sys.argv) != 2:
    print("usage: analyze_stationary_balanced_shadow.py RUN_DIR_or_stationary_balanced_shadow.csv")
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
            if int(r["production_valid"]) != 1 or int(r["balanced_valid"]) != 1:
                continue
            rows.append({
                "t": int(r["mono_ns"])*1e-9,
                "pdu": float(r["prod_du_norm"]), "pdv": float(r["prod_dv_norm"]),
                "ps": float(r["prod_scale_rate"]), "pw": float(r["prod_yaw_rate"]),
                "bdu": float(r["bal_du_norm"]), "bdv": float(r["bal_dv_norm"]),
                "bs": float(r["bal_scale_rate"]), "bw": float(r["bal_yaw_rate"]),
                "dt": float(r["dt_s"]),
            })
        except (KeyError,ValueError):
            pass

if not rows:
    raise SystemExit("no rows with production_valid=1 and balanced_valid=1")

t0=rows[0]["t"]

def corr(a,b):
    n=len(a)
    if n<2: return float("nan")
    ma=sum(a)/n; mb=sum(b)/n
    da=[x-ma for x in a]; db=[x-mb for x in b]
    va=sum(x*x for x in da); vb=sum(x*x for x in db)
    if va<=0 or vb<=0: return float("nan")
    return sum(x*y for x,y in zip(da,db))/math.sqrt(va*vb)

def report(label, rr):
    if not rr: return
    pdu=sum(x["pdu"] for x in rr); pdv=sum(x["pdv"] for x in rr)
    bdu=sum(x["bdu"] for x in rr); bdv=sum(x["bdv"] for x in rr)
    pm=math.hypot(pdu,pdv); bm=math.hypot(bdu,bdv)
    red=(1-bm/pm)*100 if pm>0 else float("nan")
    ps=sum(x["ps"]*x["dt"] for x in rr); pw=sum(x["pw"]*x["dt"] for x in rr)
    bs=sum(x["bs"]*x["dt"] for x in rr); bw=sum(x["bw"]*x["dt"] for x in rr)
    print(f"\n{label} rows={len(rr)}")
    print(f" production translation: du={pdu:+.8f} dv={pdv:+.8f} |v|={pm:.8f}")
    print(f" balanced   translation: du={bdu:+.8f} dv={bdv:+.8f} |v|={bm:.8f}")
    print(f" balanced magnitude change vs production: {red:+.2f}% reduction")
    print(f" production integrated scale={ps:+.8f} yaw={pw:+.8f} rad ({math.degrees(pw):+.4f} deg)")
    print(f" balanced   integrated scale={bs:+.8f} yaw={bw:+.8f} rad ({math.degrees(bw):+.4f} deg)")
    print(f" corr(prod_dv,prod_scale_rate)={corr([x['pdv'] for x in rr],[x['ps'] for x in rr]):+.4f}")
    print(f" corr(bal_dv,bal_scale_rate)  ={corr([x['bdv'] for x in rr],[x['bs'] for x in rr]):+.4f}")
    print(f" corr(prod_du,prod_scale_rate)={corr([x['pdu'] for x in rr],[x['ps'] for x in rr]):+.4f}")
    print(f" corr(bal_du,bal_scale_rate)  ={corr([x['bdu'] for x in rr],[x['bs'] for x in rr]):+.4f}")

print("STATIONARY EQUAL-CELL BALANCED FIT A/B")
print("=====================================")
print("CSV:",p)
print(f"rows={len(rows)} duration={rows[-1]['t']-rows[0]['t']:.1f}s")
report("WHOLE RUN",rows)

end=rows[-1]["t"]
minute=60.0
k=0
while t0+k*minute < end:
    a=t0+k*minute; b=min(a+minute,end+1e-9)
    rr=[x for x in rows if a <= x["t"] < b]
    if rr:
        report(f"{k}-{k+1} min",rr)
    k+=1

print("\nINTERPRETATION")
print("A = unchanged production 4-parameter fit.")
print("B = same production RANSAC inliers, but every occupied 3x3 ROI cell has equal total weight.")
print("A large reduction of stationary translation in B, with the same frames/inliers, supports spatial feature-count coupling.")
print("No reduction (or a larger bias) means unequal cell counts alone do not explain the stationary translation.")
print("This shadow is diagnostic only and is never sent to the FC.")
