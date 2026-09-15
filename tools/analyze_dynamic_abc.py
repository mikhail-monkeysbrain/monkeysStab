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

need=["guide_leg","guide_stage","dt_s","luna_m",
      "fc_roll","fc_pitch","fc_yaw","fc_gyro_x","fc_gyro_y","fc_gyro_samples",
      "flow_body_x","flow_body_y",
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
    # Mirror ArduPilot / existing return-forensic conventions:
    #   flowComp = -rawFlow + bodyRate
    #   v_body_x = -flowComp.y * range
    #   v_body_y =  flowComp.x * range
    # Then rotate each body-FRD displacement into NED before accumulation.
    n=e=0.0
    samples=0
    comp_rates=[]
    gyro_missing=0
    for r in rows:
        if int(r.get("guide_leg",0))!=leg or int(r.get("guide_stage",0))!=1:
            continue
        if r.get("valid",0)<0.5:
            continue
        if validk and r.get(validk,0)<0.5:
            continue
        dt=r.get("dt_s",0.0); h=camera_h(r)
        if h is None or not (0<dt<0.2):
            continue
        # fc_gyro_samples==0 does NOT mean gyro is unavailable. In production
        # consumeGyroAverage() falls back to the latest valid ATTITUDE/body-rate
        # sample when no new samples accumulated since the previous processed
        # camera frame. The CSV still contains that valid fc_gyro_x/y value.
        # Therefore keep these rows; only count them diagnostically.
        if r.get("fc_gyro_samples",0)<1:
            gyro_missing += 1

        fx=r.get(xk,float("nan")); fy=r.get(yk,float("nan"))
        gx=r.get("fc_gyro_x",float("nan")); gy=r.get("fc_gyro_y",float("nan"))
        roll=r.get("fc_roll",float("nan")); pitch=r.get("fc_pitch",float("nan")); yaw=r.get("fc_yaw",float("nan"))
        if not all(math.isfinite(v) for v in (fx,fy,gx,gy,roll,pitch,yaw)):
            continue

        comp_x=-fx+gx
        comp_y=-fy+gy
        dbx=(-comp_y)*h*dt
        dby=( comp_x)*h*dt

        cr=math.cos(roll); sr=math.sin(roll)
        cp=math.cos(pitch); sp=math.sin(pitch)
        cy=math.cos(yaw); sy=math.sin(yaw)
        r00=cy*cp
        r01=cy*sp*sr-sy*cr
        r10=sy*cp
        r11=sy*sp*sr+cy*cr

        n += r00*dbx + r01*dby
        e += r10*dbx + r11*dby
        samples += 1
        comp_rates.append(math.hypot(comp_x,comp_y))

    rms=math.sqrt(statistics.fmean([v*v for v in comp_rates])) if comp_rates else float("nan")
    return n,e,math.hypot(n,e),samples,rms,gyro_missing

print(f"CSV: {csv_path}")
print(f"target per leg: {target_mm:.1f} mm")
print(f"camera-range z correction: {camera_minus_range:+.4f} m")
print("integration: gyro-compensated body displacement, rotated sample-by-sample to NED")
print()

vecs={}
for arm,(xk,yk,vk) in arms.items():
    vecs[arm]=[]
    print(f"===== {arm} =====")
    for leg in (1,2):
        n,e,norm,count,rms,gyro_missing=leg_integral(leg,xk,yk,vk)
        vecs[arm].append((n,e))
        err=norm*1000-target_mm
        scale=(norm*1000/target_mm) if target_mm else float("nan")
        print(f"LEG {leg}: dN={n*1000:+.3f} dE={e*1000:+.3f} norm={norm*1000:.3f} mm  err={err:+.3f} mm  scale={scale:.5f}  samples={count}  compRateRMS={rms:.6f}  gyro_missing={gyro_missing}")
    (n1,e1),(n2,e2)=vecs[arm]
    closure=math.hypot(n1+n2,e1+e2)
    dot=n1*n2+e1*e2
    l1=math.hypot(n1,e1); l2=math.hypot(n2,e2)
    ang=float("nan")
    if l1>0 and l2>0:
        cc=max(-1.0,min(1.0,dot/(l1*l2)))
        ang=math.degrees(math.acos(cc))
    mean_dist=(l1+l2)*500.0
    mean_abs_err=(abs(l1*1000-target_mm)+abs(l2*1000-target_mm))/2.0
    print(f"mean distance={mean_dist:.3f} mm  mean |error|={mean_abs_err:.3f} mm")
    print(f"reciprocal angle={ang:.3f} deg  closure={closure*1000:.3f} mm")
    print()

print("===== COMPARISON =====")
for arm in ("A","B","C"):
    vals=vecs[arm]
    mean_dist=sum(math.hypot(n,e) for n,e in vals)*500.0
    closure=math.hypot(vals[0][0]+vals[1][0],vals[0][1]+vals[1][1])*1000
    print(f"{arm}: mean_distance={mean_dist:.3f} mm  scale={mean_dist/target_mm:.5f}  closure={closure:.3f} mm")
