#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

if len(sys.argv) != 2:
    raise SystemExit("usage: analyze_fb_ab_dataset.py optical_flow_mavlink.csv")

path=Path(sys.argv[1])
rows=[]
with path.open(newline="") as f:
    for r in csv.DictReader(f):
        try:
            row={k:float(v) for k,v in r.items() if v not in ("",None)}
        except ValueError:
            continue
        rows.append(row)

if not rows:
    raise SystemExit("no rows")

required=["ab_fb_enabled","ab_fb_valid","ab_fb_flow_body_x","ab_fb_flow_body_y",
          "ab_robust_valid","ab_robust_flow_body_x","ab_robust_flow_body_y",
          "ab_obs_valid","ab_obs_flow_body_x","ab_obs_flow_body_y",
          "flow_body_x","flow_body_y","dt_s","luna_m"]
missing=[k for k in required if k not in rows[0]]
if missing:
    raise SystemExit("CSV has no A/B columns: "+", ".join(missing))

a=[r for r in rows if r.get("valid",0)>0.5]
paired=[r for r in a if r.get("ab_fb_valid",0)>0.5 and r.get("ab_robust_valid",0)>0.5 and r.get("ab_obs_valid",0)>0.5]

def mean(v):
    return statistics.fmean(v) if v else float("nan")
def rmsxy(rr,x,y):
    return math.sqrt(mean([r[x]*r[x]+r[y]*r[y] for r in rr])) if rr else float("nan")
def integ(rr,x,y):
    sx=sy=0.0
    n=0
    for r in rr:
        dt=r.get("dt_s",0.0); h=r.get("luna_m",0.0)
        if 0<dt<0.2 and 0.05<h<10:
            sx += r[x]*h*dt
            sy += r[y]*h*dt
            n += 1
    return sx,sy,math.hypot(sx,sy),n

ax,ay,an,na=integ(paired,"flow_body_x","flow_body_y")
bx,by,bn,nb=integ(paired,"ab_fb_flow_body_x","ab_fb_flow_body_y")
cx,cy,cn,nc=integ(paired,"ab_robust_flow_body_x","ab_robust_flow_body_y")
dx,dy,dn,nd=integ(paired,"ab_obs_flow_body_x","ab_obs_flow_body_y")

print(f"CSV: {path}")
print(f"rows={len(rows)}  A_valid={len(a)}  paired_A_B_C_D={len(paired)} ({100*len(paired)/max(1,len(a)):.2f}%)")
print(f"FB threshold px: {rows[0].get('ab_fb_max_px',float('nan')):g}")
print(f"mean FB pass ratio: {mean([r.get('ab_fb_ratio',0.0) for r in a]):.4f}")
print(f"mean A inliers:      {mean([r.get('inliers',0.0) for r in paired]):.2f}")
print(f"mean B FB-pass:      {mean([r.get('ab_fb_pass',0.0) for r in paired]):.2f}")
print(f"mean B inliers:      {mean([r.get('ab_fb_inliers',0.0) for r in paired]):.2f}")
print(f"mean shadow time ms: {mean([r.get('ab_fb_t_ms',0.0) for r in a]):.3f}")
print(f"mean C robust sigma:  {mean([r.get('ab_robust_sigma',0.0) for r in paired]):.8f}")
print(f"mean C weight:        {mean([r.get('ab_robust_mean_weight',0.0) for r in paired]):.4f}")
print(f"mean C downweighted:  {mean([r.get('ab_robust_downweighted',0.0) for r in paired]):.2f}")
print(f"mean C IRLS iters:    {mean([r.get('ab_robust_iters',0.0) for r in paired]):.2f}")
print(f"mean D eig ratio:     {mean([r.get('ab_obs_median_ratio',0.0) for r in paired]):.6f}")
print(f"mean D weight:        {mean([r.get('ab_obs_mean_weight',0.0) for r in paired]):.4f}")
print(f"mean D downweighted:  {mean([r.get('ab_obs_downweighted',0.0) for r in paired]):.2f}")
print()
print("PAIRED SAME-FRAME METRICS")
print(f"A mean flow X/Y rad/s: {mean([r['flow_body_x'] for r in paired]):+.8f}  {mean([r['flow_body_y'] for r in paired]):+.8f}")
print(f"B mean flow X/Y rad/s: {mean([r['ab_fb_flow_body_x'] for r in paired]):+.8f}  {mean([r['ab_fb_flow_body_y'] for r in paired]):+.8f}")
print(f"C mean flow X/Y rad/s: {mean([r['ab_robust_flow_body_x'] for r in paired]):+.8f}  {mean([r['ab_robust_flow_body_y'] for r in paired]):+.8f}")
print(f"D mean flow X/Y rad/s: {mean([r['ab_obs_flow_body_x'] for r in paired]):+.8f}  {mean([r['ab_obs_flow_body_y'] for r in paired]):+.8f}")
print(f"A RMS vector rad/s:    {rmsxy(paired,'flow_body_x','flow_body_y'):.8f}")
print(f"B RMS vector rad/s:    {rmsxy(paired,'ab_fb_flow_body_x','ab_fb_flow_body_y'):.8f}")
print(f"C RMS vector rad/s:    {rmsxy(paired,'ab_robust_flow_body_x','ab_robust_flow_body_y'):.8f}")
print(f"D RMS vector rad/s:    {rmsxy(paired,'ab_obs_flow_body_x','ab_obs_flow_body_y'):.8f}")
print(f"A integral X/Y/norm:   {ax*1000:+.3f} {ay*1000:+.3f} / {an*1000:.3f} mm")
print(f"B integral X/Y/norm:   {bx*1000:+.3f} {by*1000:+.3f} / {bn*1000:.3f} mm")
print(f"C integral X/Y/norm:   {cx*1000:+.3f} {cy*1000:+.3f} / {cn*1000:.3f} mm")
print(f"D integral X/Y/norm:   {dx*1000:+.3f} {dy*1000:+.3f} / {dn*1000:.3f} mm")
if math.isfinite(an) and an>0:
    print(f"B/A endpoint ratio:     {bn/an:.3f}")
if math.isfinite(rmsxy(paired,'flow_body_x','flow_body_y')) and rmsxy(paired,'flow_body_x','flow_body_y')>0:
    print(f"B/A RMS ratio:          {rmsxy(paired,'ab_fb_flow_body_x','ab_fb_flow_body_y')/rmsxy(paired,'flow_body_x','flow_body_y'):.3f}")

if math.isfinite(an) and an>0:
    print(f"C/A endpoint ratio:     {cn/an:.3f}")
if math.isfinite(bn) and bn>0:
    print(f"C/B endpoint ratio:     {cn/bn:.3f}")
ar=rmsxy(paired,'flow_body_x','flow_body_y')
br=rmsxy(paired,'ab_fb_flow_body_x','ab_fb_flow_body_y')
cr=rmsxy(paired,'ab_robust_flow_body_x','ab_robust_flow_body_y')
if math.isfinite(ar) and ar>0:
    print(f"C/A RMS ratio:          {cr/ar:.3f}")
if math.isfinite(br) and br>0:
    print(f"C/B RMS ratio:          {cr/br:.3f}")

dr=rmsxy(paired,'ab_obs_flow_body_x','ab_obs_flow_body_y')
if math.isfinite(an) and an>0:
    print(f"D/A endpoint ratio:     {dn/an:.3f}")
if math.isfinite(bn) and bn>0:
    print(f"D/B endpoint ratio:     {dn/bn:.3f}")
if math.isfinite(ar) and ar>0:
    print(f"D/A RMS ratio:          {dr/ar:.3f}")
if math.isfinite(br) and br>0:
    print(f"D/B RMS ratio:          {dr/br:.3f}")
