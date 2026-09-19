#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

if len(sys.argv) != 2:
    raise SystemExit("usage: analyze_stationary_bias_layers.py /path/to/optical_flow_mavlink.csv")
p=Path(sys.argv[1])

def f(r,k,d=0.0):
    try: return float(r.get(k,d))
    except (TypeError,ValueError): return d
def i(r,k,d=0):
    try: return int(float(r.get(k,d)))
    except (TypeError,ValueError): return d
def mag(a,b): return math.hypot(a,b)
def med(v): return statistics.median(v) if v else 0.0
def mean(v): return statistics.fmean(v) if v else 0.0

rows=[]
with p.open(newline="",encoding="utf-8",errors="replace") as fh:
    rd=csv.DictReader(fh)
    need={"mono_ns","worked5_valid","worked5_du_norm","worked5_dv_norm",
          "worked5_dx_m","worked5_dy_m","worked5_dN_m","worked5_dE_m",
          "fc_roll","fc_pitch","fc_yaw","fc_gyro_x","fc_gyro_y","fc_gyro_z",
          "luna_m","tracked","inliers","inlier_ratio"}
    missing=need-set(rd.fieldnames or [])
    if missing: raise SystemExit("missing columns: "+", ".join(sorted(missing)))
    for r in rd:
        if i(r,"worked5_valid")==1: rows.append(r)
if not rows: raise SystemExit("no WORKED5-valid rows")

t0=f(rows[0],"mono_ns")
for r in rows: r["_t"]=(f(r,"mono_ns")-t0)/1e9
duration=max(r["_t"] for r in rows)

def report(label, rr):
    if not rr: return
    sdx=sum(f(r,"worked5_dx_m") for r in rr)
    sdy=sum(f(r,"worked5_dy_m") for r in rr)
    sn=sum(f(r,"worked5_dN_m") for r in rr)
    se=sum(f(r,"worked5_dE_m") for r in rr)
    sdu=sum(f(r,"worked5_du_norm") for r in rr)
    sdv=sum(f(r,"worked5_dv_norm") for r in rr)
    rolls=[math.degrees(f(r,"fc_roll")) for r in rr]
    pitches=[math.degrees(f(r,"fc_pitch")) for r in rr]
    yaws=[math.degrees(f(r,"fc_yaw")) for r in rr]
    gx=[math.degrees(f(r,"fc_gyro_x")) for r in rr]
    gy=[math.degrees(f(r,"fc_gyro_y")) for r in rr]
    gz=[math.degrees(f(r,"fc_gyro_z")) for r in rr]
    rng=[1000*f(r,"luna_m") for r in rr if f(r,"luna_m")>0]
    ratios=[f(r,"inlier_ratio") for r in rr]
    print(f"\n{label}  rows={len(rr)}")
    print(f"  W5 camera metric : dx={1000*sdx:+.3f} dy={1000*sdy:+.3f} |d|={1000*mag(sdx,sdy):.3f} mm")
    print(f"  W5 N/E           : dN={1000*sn:+.3f} dE={1000*se:+.3f} |d|={1000*mag(sn,se):.3f} mm")
    print(f"  sum normalized   : du={sdu:+.8f} dv={sdv:+.8f}")
    print(f"  attitude mean deg: roll={mean(rolls):+.4f} pitch={mean(pitches):+.4f} yaw={mean(yaws):+.4f}")
    print(f"  attitude span deg: roll={min(rolls):+.4f}..{max(rolls):+.4f} pitch={min(pitches):+.4f}..{max(pitches):+.4f} yaw={min(yaws):+.4f}..{max(yaws):+.4f}")
    print(f"  gyro mean deg/s  : x={mean(gx):+.5f} y={mean(gy):+.5f} z={mean(gz):+.5f}")
    print(f"  range mm         : mean={mean(rng):.3f} median={med(rng):.3f} span={min(rng):.3f}..{max(rng):.3f}")
    print(f"  inlier ratio     : mean={mean(ratios):.4f} median={med(ratios):.4f}")

print("STATIONARY BIAS LAYERS")
print("======================")
print("CSV:",p)
print(f"WORKED5 valid rows={len(rows)} duration={duration:.1f}s")
report("WHOLE RUN",rows)

w=300.0
start=0.0
while start < duration+1e-9:
    end=min(duration,start+w)
    rr=[r for r in rows if start <= r["_t"] < end+(1e-9 if end==duration else 0)]
    report(f"{start/60:.0f}-{end/60:.0f} min",rr)
    start+=w

# Direction consistency: compare camera-plane and N/E accumulated vector angles.
sdx=sum(f(r,"worked5_dx_m") for r in rows); sdy=sum(f(r,"worked5_dy_m") for r in rows)
sn=sum(f(r,"worked5_dN_m") for r in rows); se=sum(f(r,"worked5_dE_m") for r in rows)
cam_ang=math.degrees(math.atan2(sdy,sdx))
ned_ang=math.degrees(math.atan2(se,sn))
print("\nDIRECTION")
print(f"camera metric vector angle atan2(dy,dx) = {cam_ang:+.3f} deg")
print(f"N/E vector angle atan2(dE,dN)          = {ned_ang:+.3f} deg")
print(f"angle difference                         = {((ned_ang-cam_ang+180)%360)-180:+.3f} deg")
print("\nINTERPRETATION KEY")
print("If camera-metric dx/dy already accumulates strongly while stationary, bias exists before N/E attitude rotation.")
print("If camera-metric dx/dy stays near zero but N/E accumulates, investigate attitude/mapping instead.")
