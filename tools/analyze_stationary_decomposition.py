#!/usr/bin/env python3
import csv, math, os, sys

if len(sys.argv)!=2:
    print("usage: analyze_stationary_decomposition.py RUN_DIR_or_stationary_balanced_shadow.csv")
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
            if int(r["production_valid"])!=1:
                continue
            if int(r["translation_only_valid"])!=1 or int(r["trans_scale_valid"])!=1 or int(r["trans_yaw_valid"])!=1:
                continue
            rows.append({
                "t":int(r["mono_ns"])*1e-9,
                "adu":float(r["prod_du_norm"]),"adv":float(r["prod_dv_norm"]),
                "cdu":float(r["trans_du_norm"]),"cdv":float(r["trans_dv_norm"]),
                "sdu":float(r["ts_du_norm"]),"sdv":float(r["ts_dv_norm"]),
                "ydu":float(r["ty_du_norm"]),"ydv":float(r["ty_dv_norm"]),
                "as":float(r["prod_scale_rate"]),"aw":float(r["prod_yaw_rate"]),
                "ss":float(r["ts_scale_rate"]),"yw":float(r["ty_yaw_rate"]),
                "dt":float(r["dt_s"]),
            })
        except (KeyError,ValueError):
            pass

if not rows:
    raise SystemExit("no valid decomposition-shadow rows")

def vecsum(rr,u,v):
    du=sum(x[u] for x in rr); dv=sum(x[v] for x in rr)
    return du,dv,math.hypot(du,dv)

def report(label,rr):
    if not rr: return
    A=vecsum(rr,"adu","adv")
    C=vecsum(rr,"cdu","cdv")
    S=vecsum(rr,"sdu","sdv")
    Y=vecsum(rr,"ydu","ydv")
    print(f"\n{label} rows={len(rr)}")
    print(f" A full 4-param       : du={A[0]:+.8f} dv={A[1]:+.8f} |v|={A[2]:.8f}")
    print(f" C translation-only   : du={C[0]:+.8f} dv={C[1]:+.8f} |v|={C[2]:.8f}")
    print(f" S translation+scale  : du={S[0]:+.8f} dv={S[1]:+.8f} |v|={S[2]:.8f}")
    print(f" Y translation+yaw    : du={Y[0]:+.8f} dv={Y[1]:+.8f} |v|={Y[2]:.8f}")
    if A[2]>0:
        print(f" C vs A reduction     : {(1-C[2]/A[2])*100:+.2f}%")
        print(f" S vs A reduction     : {(1-S[2]/A[2])*100:+.2f}%")
        print(f" Y vs A reduction     : {(1-Y[2]/A[2])*100:+.2f}%")
    print(f" A integrated scale   : {sum(x['as']*x['dt'] for x in rr):+.8f}")
    print(f" A integrated yaw     : {sum(x['aw']*x['dt'] for x in rr):+.8f} rad")
    print(f" S integrated scale   : {sum(x['ss']*x['dt'] for x in rr):+.8f}")
    print(f" Y integrated yaw     : {sum(x['yw']*x['dt'] for x in rr):+.8f} rad")

print("STATIONARY DECOMPOSITION: SCALE vs YAW")
print("=====================================")
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
print("A = production translation+scale+yaw.")
print("C = median translation only.")
print("S = translation+scale with yaw forced to zero.")
print("Y = translation+yaw with scale forced to zero.")
print("If S stays close to A while Y stays close to C, scale coupling dominates.")
print("If Y stays close to A while S stays close to C, yaw coupling dominates.")
print("If both are intermediate, both nuisance terms contribute.")
print("Diagnostic only; no shadow is sent to the FC.")
