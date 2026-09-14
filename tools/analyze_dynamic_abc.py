#!/usr/bin/env python3
import csv, json, math, statistics, sys
from pathlib import Path

if len(sys.argv) < 2:
    raise SystemExit("usage: analyze_dynamic_abc.py optical_flow_mavlink.csv [target_mm]")
csv_path=Path(sys.argv[1])
target_mm=float(sys.argv[2]) if len(sys.argv)>2 else 175.0
mount_path=csv_path.parent/"mount_geometry.json"
camera_minus_range=0.0
if mount_path.exists():
    g=json.loads(mount_path.read_text(encoding="utf-8"))
    camera_minus_range=float(g["camera"]["z"])-float(g["rangefinder"]["z"])

rows=[]
with csv_path.open(newline="") as f:
    rd=csv.DictReader(f)
    for r in rd:
        try:
            d={k:float(v) for k,v in r.items() if v not in ("",None)}
        except ValueError:
            continue
        rows.append(d)

need=["guide_leg","guide_stage","dt_s","luna_m","flow_body_x","flow_body_y",
      "ab_fb_valid","ab_fb_flow_body_x","ab_fb_flow_body_y",
      "ab_robust_valid","ab_robust_flow_body_x","ab_robust_flow_body_y"]
missing=[k for k in need if not rows or k not in rows[0]]
if missing:
    raise SystemExit("missing columns: "+", ".join(missing))

arms={
    "A":("flow_body_x","flow_body_y",None),
    "B":("ab_fb_flow_body_x","ab_fb_flow_body_y","ab_fb_valid"),
    "C":("ab_robust_flow_body_x","ab_robust_flow_body_y","ab_robust_valid"),
}

def camera_h(r):
    h=r.get("luna_m",0.0)-camera_minus_range
    return h if math.isfinite(h) and 0.05<h<10 else None

def leg_integral(leg,xk,yk,validk):
    x=y=0.0; n=0
    vals=[]
    for r in rows:
        if int(r.get("guide_leg",0))!=leg or int(r.get("guide_stage",0))!=1:
            continue
        if r.get("valid",0)<0.5: continue
        if validk and r.get(validk,0)<0.5: continue
        dt=r.get("dt_s",0.0); h=camera_h(r)
        if h is None or not (0<dt<0.2): continue
        vx=r.get(xk,float("nan")); vy=r.get(yk,float("nan"))
        if not (math.isfinite(vx) and math.isfinite(vy)): continue
        dx=vx*h*dt; dy=vy*h*dt
        x+=dx; y+=dy; n+=1
        vals.append(math.hypot(vx,vy))
    return x,y,math.hypot(x,y),n,(math.sqrt(statistics.fmean([v*v for v in vals])) if vals else float("nan"))

print(f"CSV: {csv_path}")
print(f"target per leg: {target_mm:.1f} mm")
print(f"camera-range z correction: {camera_minus_range:+.4f} m")
print()

vecs={}
for arm,(xk,yk,vk) in arms.items():
    vecs[arm]=[]
    print(f"===== {arm} =====")
    for leg in (1,2):
        x,y,norm,n,rms=leg_integral(leg,xk,yk,vk)
        vecs[arm].append((x,y))
        err=norm*1000-target_mm
        scale=(norm*1000/target_mm) if target_mm else float("nan")
        print(f"LEG {leg}: dx={x*1000:+.3f} dy={y*1000:+.3f} norm={norm*1000:.3f} mm  err={err:+.3f} mm  scale={scale:.5f}  samples={n}  rateRMS={rms:.6f}")
    (x1,y1),(x2,y2)=vecs[arm]
    closure=math.hypot(x1+x2,y1+y2)
    dot=x1*x2+y1*y2
    n1=math.hypot(x1,y1); n2=math.hypot(x2,y2)
    ang=float("nan")
    if n1>0 and n2>0:
        c=max(-1.0,min(1.0,dot/(n1*n2)))
        ang=math.degrees(math.acos(c))
    mean_dist=(n1+n2)*500.0
    mean_abs_err=(abs(n1*1000-target_mm)+abs(n2*1000-target_mm))/2.0
    print(f"mean distance={mean_dist:.3f} mm  mean |error|={mean_abs_err:.3f} mm")
    print(f"reciprocal angle={ang:.3f} deg  closure={closure*1000:.3f} mm")
    print()

print("===== COMPARISON =====")
for arm in ("A","B","C"):
    vals=vecs[arm]
    mean_dist=sum(math.hypot(x,y) for x,y in vals)*500.0
    closure=math.hypot(vals[0][0]+vals[1][0],vals[0][1]+vals[1][1])*1000
    print(f"{arm}: mean_distance={mean_dist:.3f} mm  scale={mean_dist/target_mm:.5f}  closure={closure:.3f} mm")
