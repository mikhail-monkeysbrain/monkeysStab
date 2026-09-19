#!/usr/bin/env python3
import csv, math, os, statistics, sys

if len(sys.argv)!=2:
    print("usage: analyze_scale_physical_consistency.py RUN_DIR_or_stationary_balanced_shadow.csv")
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
            if int(r["camera_height_valid"])!=1 or int(r["prev_camera_height_valid"])!=1:
                continue
            dt=float(r["dt_s"])
            h0=float(r["prev_camera_height_m"])
            h1=float(r["camera_height_m"])
            if not (dt>0 and h0>0.05 and h1>0.05):
                continue
            global_scale=float(r["ts_scale_rate"])
            # Ground-plane pinhole expectation: normalized image coordinates
            # scale approximately inversely with camera height.
            # For small steps: s ~= -d(h)/h/dt.  Use log ratio for symmetry.
            expected=-math.log(h1/h0)/dt
            regs={}
            ok=True
            for name in ("left","right","top","bottom"):
                if int(r["scale_"+name+"_valid"])!=1:
                    ok=False; break
                regs[name]=float(r["scale_"+name+"_rate"])
            if not ok:
                continue
            vals=list(regs.values())
            spread=max(vals)-min(vals)
            mean_reg=sum(vals)/4.0
            sign_agree=sum(1 for v in vals if v*global_scale>0)/4.0 if abs(global_scale)>1e-12 else 0.0
            rows.append({
                "t":int(r["mono_ns"])*1e-9,"dt":dt,"h0":h0,"h1":h1,
                "global":global_scale,"expected":expected,
                "spread":spread,"mean_reg":mean_reg,"sign_agree":sign_agree,
                **regs
            })
        except (KeyError,ValueError,ZeroDivisionError):
            pass

if not rows:
    raise SystemExit("no rows with valid visual scale + camera height")

def corr(a,b):
    n=len(a)
    if n<2: return float("nan")
    ma=sum(a)/n; mb=sum(b)/n
    da=[x-ma for x in a]; db=[x-mb for x in b]
    va=sum(x*x for x in da); vb=sum(x*x for x in db)
    if va<=0 or vb<=0: return float("nan")
    return sum(x*y for x,y in zip(da,db))/math.sqrt(va*vb)

def q(v,p):
    if not v: return float("nan")
    s=sorted(v); x=(len(s)-1)*p; i=int(math.floor(x)); j=min(len(s)-1,i+1); f=x-i
    return s[i]*(1-f)+s[j]*f

def report(label,rr):
    if not rr: return
    gv=[x["global"] for x in rr]
    ev=[x["expected"] for x in rr]
    sp=[x["spread"] for x in rr]
    dh=[x["h1"]-x["h0"] for x in rr]
    abs_err=[abs(x["global"]-x["expected"]) for x in rr]
    print(f"\n{label} rows={len(rr)}")
    print(f" height mean={sum(x['h1'] for x in rr)/len(rr):.6f} m  span={min(x['h1'] for x in rr):.6f}..{max(x['h1'] for x in rr):.6f}")
    print(f" dh/frame median={statistics.median(dh)*1000:+.4f} mm  p95_abs={q([abs(x) for x in dh],0.95)*1000:.4f} mm")
    print(f" visual global scale mean={sum(gv)/len(gv):+.8e}/s")
    print(f" range-expected scale mean={sum(ev)/len(ev):+.8e}/s")
    print(f" visual-vs-range corr={corr(gv,ev):+.4f}")
    print(f" median |visual-expected|={statistics.median(abs_err):.8e}/s")
    print(f" median regional spread={statistics.median(sp):.8e}/s")
    print(f" mean regional sign agreement={sum(x['sign_agree'] for x in rr)/len(rr):.3f}")
    print(f" integrated visual scale={sum(x['global']*x['dt'] for x in rr):+.8f}")
    print(f" integrated expected scale={sum(x['expected']*x['dt'] for x in rr):+.8f}")

print("VISUAL SCALE vs PHYSICAL HEIGHT")
print("===============================")
print("CSV:",p)
print(f"rows={len(rows)} duration={rows[-1]['t']-rows[0]['t']:.1f}s")
print("Expected scale uses -log(h1/h0)/dt from camera-height samples.")
report("WHOLE RUN",rows)

t0=rows[0]["t"]; end=rows[-1]["t"]; k=0
while t0+k*60.0<end:
    a=t0+k*60.0; b=min(a+60.0,end+1e-9)
    rr=[x for x in rows if a<=x["t"]<b]
    if rr: report(f"{k}-{k+1} min",rr)
    k+=1

print("\nINTERPRETATION")
print("At rest, real physical scale should be near zero. If visual scale accumulates while range-expected scale stays near zero, the visual scale is non-physical.")
print("During real height change, useful scale should both be spatially coherent and track the range-derived expected scale with the same sign and similar magnitude.")
print("No gate or production behavior is changed by this analyzer.")
