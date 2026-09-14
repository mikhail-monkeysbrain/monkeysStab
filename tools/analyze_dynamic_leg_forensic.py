#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_dynamic_leg_forensic.py optical_flow_mavlink.csv")

path=Path(sys.argv[1])
rows=[]
with path.open(newline="") as f:
    for r in csv.DictReader(f):
        try:
            rows.append({k:float(v) for k,v in r.items() if v not in ("",None)})
        except ValueError:
            continue

def vals(rr,k):
    return [r[k] for r in rr if k in r and math.isfinite(r[k])]

def pct(v,p):
    if not v: return float("nan")
    s=sorted(v)
    i=min(len(s)-1,max(0,int(round((len(s)-1)*p))))
    return s[i]

def statline(name,v):
    if not v:
        print(f"{name:24s} no-data"); return
    print(f"{name:24s} mean={statistics.fmean(v): .6f} med={statistics.median(v): .6f} "
          f"p95={pct(v,.95): .6f} max={max(v): .6f}")

for leg in (1,2):
    rr=[r for r in rows if int(r.get("guide_leg",0))==leg and int(r.get("guide_stage",0))==1]
    print()
    print("="*72)
    print(f"LEG {leg} FORENSIC")
    print("="*72)
    if not rr:
        print("no rows"); continue

    t=vals(rr,"mono_ns")
    duration=(max(t)-min(t))*1e-9 if len(t)>=2 else float("nan")
    print(f"rows={len(rr)} duration_s={duration:.3f}")

    valid=sum(r.get("valid",0)>0.5 for r in rr)
    sent=sum(r.get("flow_sent",0)>0.5 for r in rr)
    fbv=sum(r.get("ab_fb_valid",0)>0.5 for r in rr)
    cv=sum(r.get("ab_robust_valid",0)>0.5 for r in rr)
    print(f"A valid={valid}/{len(rr)} ({100*valid/len(rr):.2f}%)  flow_sent={sent}/{len(rr)} ({100*sent/len(rr):.2f}%)")
    print(f"B valid={fbv}/{len(rr)} ({100*fbv/len(rr):.2f}%)  C valid={cv}/{len(rr)} ({100*cv/len(rr):.2f}%)")

    statline("dt_s", vals(rr,"dt_s"))
    statline("pipeline_latency_ms", vals(rr,"frame_pipeline_latency_ms"))
    statline("camera_queue_dropped", vals(rr,"camera_queue_dropped"))
    statline("luna_m", vals(rr,"luna_m"))
    statline("lk_height_scale", vals(rr,"lk_height_scale"))
    statline("features", vals(rr,"features"))
    statline("tracked", vals(rr,"tracked"))
    statline("inliers", vals(rr,"inliers"))
    statline("inlier_ratio", vals(rr,"inlier_ratio"))
    statline("fb_ratio", vals(rr,"ab_fb_ratio"))
    statline("fb_inliers", vals(rr,"ab_fb_inliers"))
    statline("robust_sigma", vals(rr,"ab_robust_sigma"))
    statline("robust_mean_weight", vals(rr,"ab_robust_mean_weight"))

    gs=vals(rr,"fc_gyro_samples")
    ga=vals(rr,"fc_gyro_age_ms")
    statline("fc_gyro_age_ms", ga)
    print(f"gyro new-sample rows={sum(x>=1 for x in gs)}/{len(gs)} ({100*sum(x>=1 for x in gs)/max(1,len(gs)):.2f}%)")
    statline("gyro_x", vals(rr,"fc_gyro_x"))
    statline("gyro_y", vals(rr,"fc_gyro_y"))
    statline("gyro_z", vals(rr,"fc_gyro_z"))

    for k in ("fc_roll","fc_pitch","fc_yaw"):
        v=vals(rr,k)
        if v:
            print(f"{k:24s} start={v[0]:+.6f} end={v[-1]:+.6f} delta={v[-1]-v[0]:+.6f} "
                  f"range={max(v)-min(v):.6f}")

    # Signed visual rates for all three arms, before gyro compensation.
    for name,xk,yk,vk in (
        ("A","flow_body_x","flow_body_y",None),
        ("B","ab_fb_flow_body_x","ab_fb_flow_body_y","ab_fb_valid"),
        ("C","ab_robust_flow_body_x","ab_robust_flow_body_y","ab_robust_valid"),
    ):
        sel=[r for r in rr if r.get("valid",0)>0.5 and (vk is None or r.get(vk,0)>0.5)]
        xv=[r.get(xk,float("nan")) for r in sel]
        yv=[r.get(yk,float("nan")) for r in sel]
        xv=[x for x in xv if math.isfinite(x)]
        yv=[y for y in yv if math.isfinite(y)]
        if xv and yv:
            print(f"{name} raw flow mean         x={statistics.fmean(xv):+.8f} y={statistics.fmean(yv):+.8f} n={min(len(xv),len(yv))}")

print()
print("NOTE: compare LEG1 vs LEG2. Large differences in gyro age/rate, height,")
print("latency, validity, FB ratio or attitude excursion make scale comparison non-causal.")
