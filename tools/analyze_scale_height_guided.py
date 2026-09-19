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
            cv=float(r["centered_scale_rate"])
            cvals=[float(r["centered_"+n+"_scale_rate"]) for n in ("left","right","top","bottom")]
            cok=int(r["centered_valid"])==1 and all(int(r["centered_"+n+"_valid"])==1 for n in ("left","right","top","bottom"))
            rows.append({"wall":wall0+(int(r["mono_ns"])*1e-9-float(marks[0]["elapsed_s"])),
                         "dt":float(r["dt_s"]),"h":float(r["camera_height_m"]),
                         "scale":float(r["ts_scale_rate"]),"spread":max(vals)-min(vals),
                         "agree":sum(1 for v in vals if v*float(r["ts_scale_rate"])>0)/4.0,
                         "centered":cv if cok else float("nan"),
                         "cspread":max(cvals)-min(cvals) if cok else float("nan"),
                         "cagree":sum(1 for v in cvals if v*cv>0)/4.0 if cok else float("nan")})
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

def summary(stage):
    rr=sel(stage)
    if not rr: return None
    cr=[x for x in rr if math.isfinite(x["centered"])]
    return {"n":len(rr),"h":med([x["h"] for x in rr]),"scale_int":sum(x["scale"]*x["dt"] for x in rr),
            "scale_mean":sum(x["scale"] for x in rr)/len(rr),"spread":med([x["spread"] for x in rr]),
            "agree":sum(x["agree"] for x in rr)/len(rr),
            "centered_int":sum(x["centered"]*x["dt"] for x in cr) if cr else float("nan"),
            "centered_mean":sum(x["centered"] for x in cr)/len(cr) if cr else float("nan"),
            "cspread":med([x["cspread"] for x in cr]),"cagree":sum(x["cagree"] for x in cr)/len(cr) if cr else float("nan")}
print("GUIDED HEIGHT SCALE ANALYSIS")
print("============================")
S={}
for name in ("LOW-1 ПОКОЙ","LOW->HIGH","HIGH ПОКОЙ","HIGH->LOW","LOW-2 ПОКОЙ"):
    S[name]=summary(name)
    x=S[name]
    if not x: print(name,": NO DATA"); continue
    print(f"{name:18s} n={x['n']:5d} h_med={x['h']:.4f}m")
    print(f"  legacy   int={x['scale_int']:+.6f} mean={x['scale_mean']:+.3e}/s spread={x['spread']:.3e}/s agree={x['agree']:.3f}")
    print(f"  centered int={x['centered_int']:+.6f} mean={x['centered_mean']:+.3e}/s spread={x['cspread']:.3e}/s agree={x['cagree']:.3f}")
lo=S["LOW-1 ПОКОЙ"]; hi=S["HIGH ПОКОЙ"]; lo2=S["LOW-2 ПОКОЙ"]
if lo and hi:
    expected=math.log(lo["h"]/hi["h"])
    print(f"\nLOW/HIGH median height: {lo['h']:.4f} -> {hi['h']:.4f} m")
    print(f"Expected LOW->HIGH log image scale: {expected:+.6f}")
if lo and lo2:
    print(f"LOW return height delta: {(lo2['h']-lo['h'])*1000:+.2f} mm")
print("\nWHOLE PREDEFINED STAGE COMPARISON")
if lo and hi and lo2:
    expected_up=math.log(lo["h"]/hi["h"])
    expected_down=math.log(hi["h"]/lo2["h"])
    up=S["LOW->HIGH"]; down=S["HIGH->LOW"]
    print(f"Expected plateau scale UP   : {expected_up:+.6f}")
    print(f"Expected plateau scale DOWN : {expected_down:+.6f}")
    if up and down:
        print(f"Legacy transition closure   : {up['scale_int']+down['scale_int']:+.6f}")
        print(f"Centered transition closure : {up['centered_int']+down['centered_int']:+.6f}")
        print(f"Legacy UP/DOWN              : {up['scale_int']:+.6f} / {down['scale_int']:+.6f}")
        print(f"Centered UP/DOWN            : {up['centered_int']:+.6f} / {down['centered_int']:+.6f}")

print("\nStages are fixed by the guided test markers; TF-Luna is used for plateau medians only, not to detect motion boundaries.")
print("No production gate or WORKED5 parameter is changed.")
