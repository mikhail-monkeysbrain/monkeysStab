#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_motion_stop_drift.py optical_flow_mavlink.csv")

path=Path(sys.argv[1])
rows=[]
with path.open(newline="") as f:
    for r in csv.DictReader(f):
        try:
            rows.append({k:float(v) for k,v in r.items() if v not in ("",None)})
        except ValueError:
            continue

if not rows:
    raise SystemExit("no data")

need=["mono_ns","guide_stage","valid","dt_s","luna_m",
      "flow_body_x","flow_body_y","fc_gyro_x","fc_gyro_y",
      "fc_roll","fc_pitch","fc_yaw",
      "ekf_local_valid","ekf_x_ned","ekf_y_ned","ekf_vx_ned","ekf_vy_ned"]
missing=[k for k in need if k not in rows[0]]
if missing:
    raise SystemExit("missing columns: "+", ".join(missing))

stop=[r for r in rows if int(r.get("guide_stage",0))==2]
move=[r for r in rows if int(r.get("guide_stage",0))==1]
if not stop:
    raise SystemExit("no post-stop stage found")

t0=min(r["mono_ns"] for r in stop)

def hcam(r):
    h=r.get("luna_m",0.0)
    return h if math.isfinite(h) and 0.05<h<10 else None

def raw_vel_ned(r):
    if r.get("valid",0)<0.5:
        return None
    h=hcam(r)
    dt=r.get("dt_s",0.0)
    if h is None or not (0<dt<0.2):
        return None
    vals=[r.get(k,float("nan")) for k in ("flow_body_x","flow_body_y","fc_gyro_x","fc_gyro_y","fc_roll","fc_pitch","fc_yaw")]
    if not all(math.isfinite(v) for v in vals):
        return None
    fx,fy,gx,gy,roll,pitch,yaw=vals
    comp_x=-fx+gx
    comp_y=-fy+gy
    vbx=(-comp_y)*h
    vby=( comp_x)*h
    cr=math.cos(roll); sr=math.sin(roll)
    cp=math.cos(pitch); sp=math.sin(pitch)
    cy=math.cos(yaw); sy=math.sin(yaw)
    r00=cy*cp
    r01=cy*sp*sr-sy*cr
    r10=sy*cp
    r11=sy*sp*sr+cy*cr
    vn=r00*vbx+r01*vby
    ve=r10*vbx+r11*vby
    return vn,ve

def ekf_vel(r):
    if r.get("ekf_local_valid",0)<0.5:
        return None
    vn=r.get("ekf_vx_ned",float("nan")); ve=r.get("ekf_vy_ned",float("nan"))
    if not (math.isfinite(vn) and math.isfinite(ve)):
        return None
    return vn,ve

def window(a,b):
    return [r for r in stop if a <= (r["mono_ns"]-t0)*1e-9 < b]

def stats(rr,fn):
    sp=[]
    vec=[]
    for r in rr:
        v=fn(r)
        if v is None: continue
        x,y=v
        vec.append((x,y,r))
        sp.append(math.hypot(x,y))
    if not sp:
        return None
    return {
      "n":len(sp),
      "mean":statistics.fmean(sp),
      "median":statistics.median(sp),
      "p95":sorted(sp)[min(len(sp)-1,int(0.95*(len(sp)-1)))],
      "max":max(sp),
      "mean_x":statistics.fmean([x for x,y,r in vec]),
      "mean_y":statistics.fmean([y for x,y,r in vec]),
    }

def disp_raw(rr):
    n=e=0.0; count=0
    for r in rr:
        v=raw_vel_ned(r)
        dt=r.get("dt_s",0.0)
        if v is None or not (0<dt<0.2): continue
        n+=v[0]*dt; e+=v[1]*dt; count+=1
    return n,e,math.hypot(n,e),count

def disp_ekf(rr):
    good=[r for r in rr if r.get("ekf_local_valid",0)>0.5]
    if len(good)<2: return None
    a,b=good[0],good[-1]
    dn=b["ekf_x_ned"]-a["ekf_x_ned"]
    de=b["ekf_y_ned"]-a["ekf_y_ned"]
    return dn,de,math.hypot(dn,de),len(good)

print(f"CSV: {path}")
print(f"move_rows={len(move)} stop_rows={len(stop)}")
if move:
    md=(max(r["mono_ns"] for r in move)-min(r["mono_ns"] for r in move))*1e-9
    print(f"move_duration_s={md:.3f}")
sd=(max(r["mono_ns"] for r in stop)-min(r["mono_ns"] for r in stop))*1e-9
print(f"stop_duration_s={sd:.3f}")
print()

windows=[("0–2 s",0,2),("2–5 s",2,5),("5–10 s",5,10),("10–15 s",10,15)]
print("===== SPEED DECAY AFTER PHYSICAL STOP =====")
for label,a,b in windows:
    rr=window(a,b)
    rs=stats(rr,raw_vel_ned)
    es=stats(rr,ekf_vel)
    def fmt(s):
        return "no-data" if s is None else f"mean={s['mean']:.4f} med={s['median']:.4f} p95={s['p95']:.4f} max={s['max']:.4f} m/s"
    print(f"{label:8s} RAW {fmt(rs)}")
    print(f"{'':8s} EKF {fmt(es)}")

all_raw=disp_raw(stop)
all_ekf=disp_ekf(stop)
print()
print("===== POSITION DRIFT DURING 15 s STOP =====")
print(f"RAW dN={all_raw[0]*1000:+.2f} dE={all_raw[1]*1000:+.2f} norm={all_raw[2]*1000:.2f} mm samples={all_raw[3]}")
if all_ekf:
    print(f"EKF dN={all_ekf[0]*1000:+.2f} dE={all_ekf[1]*1000:+.2f} norm={all_ekf[2]*1000:.2f} mm samples={all_ekf[3]}")
else:
    print("EKF no-data")

early_r=stats(window(0,2),raw_vel_ned); late_r=stats(window(10,15),raw_vel_ned)
early_e=stats(window(0,2),ekf_vel); late_e=stats(window(10,15),ekf_vel)

print()
print("===== VERDICT =====")
raw_decay=(late_r and early_r and early_r["mean"]>1e-9 and late_r["mean"]/early_r["mean"])
ekf_decay=(late_e and early_e and early_e["mean"]>1e-9 and late_e["mean"]/early_e["mean"])
if raw_decay is not False:
    print(f"RAW late/early speed ratio = {raw_decay:.3f}")
if ekf_decay is not False:
    print(f"EKF late/early speed ratio = {ekf_decay:.3f}")

# Conservative classification: avoid pretending certainty when both remain large.
raw_late = late_r["mean"] if late_r else float("nan")
ekf_late = late_e["mean"] if late_e else float("nan")
raw_early = early_r["mean"] if early_r else float("nan")
ekf_early = early_e["mean"] if early_e else float("nan")

if late_r and late_e:
    if raw_late < 0.01 and ekf_late >= 0.02:
        print("CLASS: RAW успокоился, EKF продолжает движение -> искать после frontend: EKF/fusion/IMU state.")
    elif raw_late >= 0.02:
        print("CLASS: RAW сам продолжает заметное движение после STOP -> причина до/на входе EKF: visual/gyro/timing.")
    elif raw_early >= 0.02 and raw_late < 0.01 and ekf_late < 0.01:
        print("CLASS: RAW имел послемоушн-хвост, но оба тракта к 10–15 с успокоились.")
    else:
        print("CLASS: пограничный случай; смотреть RAW и EKF decay вместе, без автоматического вывода.")
else:
    print("CLASS: недостаточно данных для автоматической классификации.")
