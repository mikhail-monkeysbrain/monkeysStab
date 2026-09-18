#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Анализ срыва PyrLK по production CSV без изменения estimator."""
import argparse, csv, math, statistics
from pathlib import Path

def f(r,k,d=math.nan):
    try: return float(r.get(k,""))
    except (TypeError,ValueError): return d

def corr(xs,ys):
    p=[(x,y) for x,y in zip(xs,ys) if math.isfinite(x) and math.isfinite(y)]
    if len(p)<3:return math.nan
    ax=statistics.mean(x for x,_ in p); ay=statistics.mean(y for _,y in p)
    dx=[x-ax for x,_ in p]; dy=[y-ay for _,y in p]
    den=math.sqrt(sum(x*x for x in dx)*sum(y*y for y in dy))
    return sum(x*y for x,y in zip(dx,dy))/den if den else math.nan

ap=argparse.ArgumentParser()
ap.add_argument("csv",type=Path)
ap.add_argument("--slow-ms",type=float,default=20.0)
args=ap.parse_args()
with args.csv.open(newline="") as h: rows=list(csv.DictReader(h))
rows=[r for r in rows if math.isfinite(f(r,"t_lk_ms")) and f(r,"t_lk_ms")>0]
print(f"CSV: {args.csv}\nrows={len(rows)} slow_threshold={args.slow_ms:.1f}ms")

metrics=["dt_s","features","tracked","inliers","inlier_ratio","camera_queue_dropped","t_ransac_ms","frame_pipeline_latency_ms"]
print("\n===== CORRELATION WITH t_lk_ms =====")
lk=[f(r,"t_lk_ms") for r in rows]
for k in metrics:
    x=[f(r,k)*1000.0 if k=="dt_s" else f(r,k) for r in rows]
    print(f"{k:28s} r={corr(x,lk): .4f}")

print("\n===== FAST/SLOW COMPARISON =====")
for name,rr in [("FAST",[r for r in rows if f(r,"t_lk_ms")<=args.slow_ms]),
                ("SLOW",[r for r in rows if f(r,"t_lk_ms")>args.slow_ms])]:
    print(f"{name}: n={len(rr)}")
    for k in ["dt_s","features","tracked","inliers","inlier_ratio","camera_queue_dropped","t_lk_ms"]:
        x=[f(r,k) for r in rr if math.isfinite(f(r,k))]
        if k=="dt_s": x=[v*1000 for v in x]
        if x: print(f"  {k:24s} median={statistics.median(x):8.3f} mean={statistics.mean(x):8.3f} max={max(x):8.3f}")

print("\n===== FAILURE EPISODES =====")
slow=[i for i,r in enumerate(rows) if f(r,"t_lk_ms")>args.slow_ms]
episodes=[]
if slow:
    a=b=slow[0]
    for i in slow[1:]:
        if i==b+1:b=i
        else:episodes.append((a,b));a=b=i
    episodes.append((a,b))
for n,(a,b) in enumerate(episodes,1):
    rr=rows[a:b+1]
    print(f"episode {n}: frames {rr[0].get('frame')}..{rr[-1].get('frame')} n={len(rr)} "
          f"max_lk={max(f(r,'t_lk_ms') for r in rr):.3f}ms "
          f"max_dt={max(f(r,'dt_s') for r in rr)*1000:.3f}ms "
          f"min_inliers={min(f(r,'inliers') for r in rr):.0f}")

print("\n===== FRAME-LOCAL TIMELINE AROUND SLOWEST =====")
if rows:
    peak=max(range(len(rows)),key=lambda i:f(rows[i],"t_lk_ms"))
    for i in range(max(0,peak-12),min(len(rows),peak+13)):
        r=rows[i]
        print(f"{'*' if i==peak else ' '} frame={r.get('frame','?'):>6} "
              f"dt={f(r,'dt_s')*1000:6.1f}ms feat={f(r,'features'):4.0f} "
              f"trk={f(r,'tracked'):4.0f} inl={f(r,'inliers'):4.0f} "
              f"LK={f(r,'t_lk_ms'):7.2f}ms drop={f(r,'camera_queue_dropped'):3.0f} "
              f"lat={f(r,'frame_pipeline_latency_ms'):7.2f}ms")
