#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

if len(sys.argv) < 2:
    raise SystemExit("usage: analyze_ekf_runaway_forensic.py optical_flow_mavlink.csv [start_s end_s]")
path=Path(sys.argv[1])
start=float(sys.argv[2]) if len(sys.argv)>2 else 64.0
end=float(sys.argv[3]) if len(sys.argv)>3 else 81.0

rows=[]
with path.open(newline="") as f:
    for r in csv.DictReader(f):
        try:
            rows.append({k:float(v) for k,v in r.items() if v not in ("",None)})
        except ValueError:
            continue
if not rows: raise SystemExit("no data")
t0=rows[0]["mono_ns"]
def ts(r): return (r["mono_ns"]-t0)*1e-9

def raw_ned(r):
    try:
        if r.get("valid",0)<0.5: return None
        h=r["luna_m"]; dt=r["dt_s"]
        if not (0.05<h<10 and 0<dt<0.2): return None
        fx,fy=r["flow_body_x"],r["flow_body_y"]
        gx,gy=r["fc_gyro_x"],r["fc_gyro_y"]
        roll,pitch,yaw=r["fc_roll"],r["fc_pitch"],r["fc_yaw"]
        compx=-fx+gx; compy=-fy+gy
        vbx=-compy*h; vby=compx*h
        cr,sr=math.cos(roll),math.sin(roll); cp,sp=math.cos(pitch),math.sin(pitch)
        cy,sy=math.cos(yaw),math.sin(yaw)
        r00=cy*cp; r01=cy*sp*sr-sy*cr
        r10=sy*cp; r11=sy*sp*sr+cy*cr
        return r00*vbx+r01*vby, r10*vbx+r11*vby
    except Exception:
        return None

def mean(rr,k):
    v=[r[k] for r in rr if k in r and math.isfinite(r[k])]
    return statistics.fmean(v) if v else float("nan")
def rng(rr,k):
    v=[r[k] for r in rr if k in r and math.isfinite(r[k])]
    return (min(v),max(v)) if v else (float("nan"),float("nan"))
def frac(rr,k,thr=.5):
    return sum(r.get(k,0)>thr for r in rr)/max(1,len(rr))

print(f"CSV: {path}")
print(f"window={start:.1f}..{end:.1f}s")
print()
print("time,raw_vN,raw_vE,raw_spd,ekf_vN,ekf_vE,ekf_spd,ekf_accN,ekf_accE,roll_deg,pitch_deg,yaw_deg,gyro_mag,range_m,flow_sent,valid,inliers,ekf_flags,vel_var,pos_h_var,terrain_var,lat_ms,drop_frac")

prev=None
binw=.5
b=start
while b<end-1e-9:
    rr=[r for r in rows if b<=ts(r)<b+binw]
    if not rr:
        b+=binw; continue
    rv=[raw_ned(r) for r in rr]; rv=[v for v in rv if v]
    rvn=statistics.fmean(v[0] for v in rv) if rv else float("nan")
    rve=statistics.fmean(v[1] for v in rv) if rv else float("nan")
    rspd=math.hypot(rvn,rve) if rv else float("nan")
    evn=mean(rr,"ekf_vx_ned"); eve=mean(rr,"ekf_vy_ned")
    espd=math.hypot(evn,eve) if math.isfinite(evn) and math.isfinite(eve) else float("nan")
    accn=acce=float("nan")
    if prev and math.isfinite(evn) and math.isfinite(prev[0]):
        accn=(evn-prev[0])/binw; acce=(eve-prev[1])/binw
    prev=(evn,eve)
    roll=math.degrees(mean(rr,"fc_roll")); pitch=math.degrees(mean(rr,"fc_pitch")); yaw=math.degrees(mean(rr,"fc_yaw"))
    gx=mean(rr,"fc_gyro_x"); gy=mean(rr,"fc_gyro_y"); gz=mean(rr,"fc_gyro_z")
    gmag=math.sqrt(gx*gx+gy*gy+gz*gz) if all(math.isfinite(x) for x in (gx,gy,gz)) else float("nan")
    flags=int(round(mean(rr,"ekf_flags"))) if math.isfinite(mean(rr,"ekf_flags")) else -1
    print(f"{b:5.1f},{rvn:+.4f},{rve:+.4f},{rspd:.4f},{evn:+.4f},{eve:+.4f},{espd:.4f},"
          f"{accn:+.4f},{acce:+.4f},{roll:+.3f},{pitch:+.3f},{yaw:+.3f},{gmag:.4f},"
          f"{mean(rr,'luna_m'):.3f},{frac(rr,'flow_sent'):.2f},{frac(rr,'valid'):.2f},{mean(rr,'inliers'):.1f},"
          f"{flags},{mean(rr,'ekf_vel_var'):.6f},{mean(rr,'ekf_pos_h_var'):.6f},{mean(rr,'ekf_terrain_var'):.6f},"
          f"{mean(rr,'frame_pipeline_latency_ms'):.1f},{frac(rr,'camera_queue_dropped',0):.2f}")
    b+=binw

# Correlate apparent EKF acceleration during the quiet runaway interval with attitude.
q=[r for r in rows if 69<=ts(r)<76.5]
if q:
    print()
    print("===== 69..76.5 s QUIET-RUNAWAY SUMMARY =====")
    print(f"RAW speed mean={statistics.fmean(math.hypot(*raw_ned(r)) for r in q if raw_ned(r)):.6f} m/s")
    print(f"EKF vN start/end={q[0].get('ekf_vx_ned',float('nan')):+.4f} -> {q[-1].get('ekf_vx_ned',float('nan')):+.4f} m/s")
    print(f"EKF vE start/end={q[0].get('ekf_vy_ned',float('nan')):+.4f} -> {q[-1].get('ekf_vy_ned',float('nan')):+.4f} m/s")
    dt=(q[-1]["mono_ns"]-q[0]["mono_ns"])*1e-9
    if dt>0:
        an=(q[-1].get('ekf_vx_ned',0)-q[0].get('ekf_vx_ned',0))/dt
        ae=(q[-1].get('ekf_vy_ned',0)-q[0].get('ekf_vy_ned',0))/dt
        amag=math.hypot(an,ae)
        tilt_deg=math.degrees(math.asin(min(1.0,amag/9.80665)))
        print(f"apparent EKF horizontal acceleration ≈ {amag:.4f} m/s²")
        print(f"equivalent gravity tilt error ≈ {tilt_deg:.3f} deg")
    rr=rng(q,"fc_roll"); pr=rng(q,"fc_pitch")
    print(f"reported roll range={math.degrees(rr[0]):+.3f}..{math.degrees(rr[1]):+.3f} deg")
    print(f"reported pitch range={math.degrees(pr[0]):+.3f}..{math.degrees(pr[1]):+.3f} deg")
    print(f"flow_sent fraction={frac(q,'flow_sent'):.3f}, valid fraction={frac(q,'valid'):.3f}")
    print(f"mean inliers={mean(q,'inliers'):.1f}, mean latency={mean(q,'frame_pipeline_latency_ms'):.1f} ms")
    print(f"EKF flags unique={sorted(set(int(r.get('ekf_flags',-1)) for r in q if 'ekf_flags' in r))}")
    print(f"vel_var range={rng(q,'ekf_vel_var')}, pos_h_var range={rng(q,'ekf_pos_h_var')}, terrain_var range={rng(q,'ekf_terrain_var')}")

print()
print("Interpretation guide:")
print("- If flow_sent≈1 and RAW≈0 while EKF velocity accelerates, current OF measurements are not the source of that velocity.")
print("- If EKF acceleration is roughly constant after physical stop, suspect inertial/attitude/bias propagation or rejection of OF corrections.")
print("- A sudden velocity collapse/reset later suggests a delayed EKF correction/reset rather than optical-flow decay.")
