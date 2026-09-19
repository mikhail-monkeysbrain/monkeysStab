#!/usr/bin/env python3
import csv, math, os, statistics, sys
from pathlib import Path

if len(sys.argv)!=3:
    raise SystemExit("usage: analyze_scale_height_guided.py RUN_DIR MARKERS.csv")
run=Path(sys.argv[1]); mp=Path(sys.argv[2]); cp=run/"stationary_balanced_shadow.csv"
if not cp.exists(): raise SystemExit(f"missing: {cp}")

marks=list(csv.DictReader(mp.open(newline="",encoding="utf-8")))
if not marks: raise SystemExit("no markers")
wall0=float(marks[0]["wall_time"])-float(marks[0]["elapsed_s"])
stages={}
for r in marks:
    stages.setdefault(r["stage"],[]).append(float(r["wall_time"]))

rows=[]
with cp.open(newline="",encoding="utf-8") as f:
    for r in csv.DictReader(f):
        try:
            if int(r["production_valid"])!=1 or int(r["camera_height_valid"])!=1: continue
            vals=[float(r["scale_"+n+"_rate"]) for n in ("left","right","top","bottom")]
            if not all(int(r["scale_"+n+"_valid"])==1 for n in ("left","right","top","bottom")): continue
            rows.append({"wall":wall0+(int(r["mono_ns"])*1e-9-float(marks[0]["elapsed_s"])),
                         "dt":float(r["dt_s"]),"h":float(r["camera_height_m"]),
                         "scale":float(r["ts_scale_rate"]),"spread":max(vals)-min(vals),
                         "agree":sum(1 for v in vals if v*float(r["ts_scale_rate"])>0)/4.0})
        except (KeyError,ValueError): pass

# monotonic and wall clocks cannot be aligned from CSV directly; use stage marker
# elapsed timeline and map camera rows by relative order/duration instead.
if not rows: raise SystemExit("no valid rows")
cam0=rows[0]["wall"]
marker_min=min(float(r["elapsed_s"]) for r in marks)
def bounds(stage):
    a=[float(r["elapsed_s"]) for r in marks if r["stage"]==stage]
    return min(a),max(a)
# Reconstruct relative camera time from accumulated dt, avoiding clock-domain mixing.
t=0.0
for r in rows:
    t+=max(0.0,r["dt"]); r["rel"]=t
# Align first marker sample to first diagnostic row approximately; startup rows may precede Enter.
# Use total test end as anchor: guided profile is 40 s and rows continue only slightly around it.
end_rel=rows[-1]["rel"]
offset=end_rel-max(float(r["elapsed_s"]) for r in marks)
for r in rows: r["test_t"]=r["rel"]-offset

def sel(stage,trim=.8):
    a,b=bounds(stage); a+=trim; b-=trim
    return [r for r in rows if a<=r["test_t"]<=b]

def med(v): return statistics.median(v) if v else float("nan")

def transition_motion(stage, h_from, h_to):
    rr=sel(stage, trim=0.0)
    if len(rr)<20 or not math.isfinite(h_from) or not math.isfinite(h_to):
        return None
    dh=h_to-h_from
    if abs(dh)<0.005:
        return None

    # Robust height trace: rolling median over ~0.5 s, using timestamps
    # reconstructed from camera dt. Detection requires sustained departure
    # from the source plateau and sustained arrival at the target plateau.
    tt=[]; acc=0.0
    for x in rr:
        acc+=max(0.0,x["dt"]); tt.append(acc)

    half=0.25
    hs=[]
    j0=0; j1=0
    for i,t0 in enumerate(tt):
        while j0<len(rr) and tt[j0]<t0-half: j0+=1
        if j1<j0: j1=j0
        while j1+1<len(rr) and tt[j1+1]<=t0+half: j1+=1
        hs.append(med([rr[j]["h"] for j in range(j0,j1+1)]))

    direction=1.0 if dh>0 else -1.0
    depart=h_from+0.15*dh
    arrive=h_from+0.85*dh
    hold=0.35

    def sustained(i, predicate):
        t0=tt[i]; k=i
        while k<len(rr) and tt[k]-t0<hold:
            if not predicate(hs[k]): return False
            k+=1
        return k<len(rr) or (tt[-1]-t0)>=hold

    if direction>0:
        p_depart=lambda h: h>=depart
        p_arrive=lambda h: h>=arrive
    else:
        p_depart=lambda h: h<=depart
        p_arrive=lambda h: h<=arrive

    i0=next((i for i in range(len(rr)) if sustained(i,p_depart)),None)
    if i0 is None: return None
    i1=next((i for i in range(i0,len(rr)) if sustained(i,p_arrive)),None)
    if i1 is None or i1<=i0: return None

    active=rr[i0:i1+1]
    visual=sum(x["scale"]*x["dt"] for x in active)
    h0=hs[i0]; h1=hs[i1]
    expected=math.log(h0/h1) if h0>0 and h1>0 else float("nan")
    return {"n":len(active),"t":tt[i1]-tt[i0],"h0":h0,"h1":h1,
            "visual":visual,"expected":expected,
            "ratio":visual/expected if abs(expected)>1e-9 else float("nan"),
            "t0":tt[i0],"t1":tt[i1]}

def summary(stage):
    rr=sel(stage)
    if not rr: return None
    return {"n":len(rr),"h":med([x["h"] for x in rr]),"scale_int":sum(x["scale"]*x["dt"] for x in rr),
            "scale_mean":sum(x["scale"] for x in rr)/len(rr),"spread":med([x["spread"] for x in rr]),
            "agree":sum(x["agree"] for x in rr)/len(rr)}
print("GUIDED HEIGHT SCALE ANALYSIS")
print("============================")
S={}
for name in ("LOW-1 ПОКОЙ","LOW->HIGH","HIGH ПОКОЙ","HIGH->LOW","LOW-2 ПОКОЙ"):
    S[name]=summary(name)
    x=S[name]
    if not x: print(name,": NO DATA"); continue
    print(f"{name:18s} n={x['n']:5d} h_med={x['h']:.4f}m scale_int={x['scale_int']:+.6f} mean={x['scale_mean']:+.3e}/s spread_med={x['spread']:.3e}/s agree={x['agree']:.3f}")
lo=S["LOW-1 ПОКОЙ"]; hi=S["HIGH ПОКОЙ"]; lo2=S["LOW-2 ПОКОЙ"]
if lo and hi:
    expected=math.log(lo["h"]/hi["h"])
    print(f"\nLOW/HIGH median height: {lo['h']:.4f} -> {hi['h']:.4f} m")
    print(f"Expected LOW->HIGH log image scale: {expected:+.6f}")
if lo and lo2:
    print(f"LOW return height delta: {(lo2['h']-lo['h'])*1000:+.2f} mm")
print("\nPHYSICAL MOTION WINDOWS (0.5s median, sustained 15%->85%)")
if lo and hi and lo2:
    for name,h0,h1 in (("LOW->HIGH",lo["h"],hi["h"]),("HIGH->LOW",hi["h"],lo2["h"])):
        x=transition_motion(name,h0,h1)
        if not x:
            print(f"{name:18s} NO DATA")
            continue
        print(f"{name:18s} n={x['n']:5d} dt={x['t']:.3f}s local_t={x['t0']:.3f}->{x['t1']:.3f}s h={x['h0']:.4f}->{x['h1']:.4f}m")
        print(f"  visual_int={x['visual']:+.6f} expected={x['expected']:+.6f} visual/expected={x['ratio']:.3f}")

print("\nMotion windows use a 0.5 s rolling median and require 0.35 s sustained departure/arrival. Full transition windows remain useful for sign and context.")
print("No production gate or WORKED5 parameter is changed.")
