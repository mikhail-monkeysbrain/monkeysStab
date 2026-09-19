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
    if not rr or not math.isfinite(h_from) or not math.isfinite(h_to):
        return None
    dh=h_to-h_from
    if abs(dh)<0.005:
        return None
    p0=h_from+0.10*dh
    p1=h_from+0.90*dh
    lo_h=min(p0,p1); hi_h=max(p0,p1)
    idx=[i for i,x in enumerate(rr) if lo_h<=x["h"]<=hi_h]
    if not idx:
        return None
    active=rr[idx[0]:idx[-1]+1]
    visual=sum(x["scale"]*x["dt"] for x in active)
    k=max(1,min(30,len(active)))
    h0=med([x["h"] for x in active[:k]])
    h1=med([x["h"] for x in active[-k:]])
    expected=math.log(h0/h1) if h0>0 and h1>0 else float("nan")
    return {"n":len(active),"t":sum(x["dt"] for x in active),"h0":h0,"h1":h1,
            "visual":visual,"expected":expected,
            "ratio":visual/expected if abs(expected)>1e-9 else float("nan")}

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
print("\nPHYSICAL MOTION WINDOWS (10..90% of measured height step)")
if lo and hi and lo2:
    for name,h0,h1 in (("LOW->HIGH",lo["h"],hi["h"]),("HIGH->LOW",hi["h"],lo2["h"])):
        x=transition_motion(name,h0,h1)
        if not x:
            print(f"{name:18s} NO DATA")
            continue
        print(f"{name:18s} n={x['n']:5d} dt={x['t']:.3f}s h={x['h0']:.4f}->{x['h1']:.4f}m")
        print(f"  visual_int={x['visual']:+.6f} expected={x['expected']:+.6f} visual/expected={x['ratio']:.3f}")

print("\nUse physical-motion windows to judge magnitude; full transition windows remain useful for sign and context.")
print("No production gate or WORKED5 parameter is changed.")
