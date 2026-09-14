#!/usr/bin/env python3
import csv, math, statistics, sys
from pathlib import Path

if len(sys.argv)!=2:
    raise SystemExit("usage: analyze_runaway_transition.py optical_flow_mavlink.csv")

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

need=["mono_ns","valid","dt_s","luna_m","flow_body_x","flow_body_y",
      "fc_gyro_x","fc_gyro_y","fc_roll","fc_pitch","fc_yaw",
      "ekf_local_valid","ekf_x_ned","ekf_y_ned","ekf_vx_ned","ekf_vy_ned"]
missing=[k for k in need if k not in rows[0]]
if missing:
    raise SystemExit("missing columns: "+", ".join(missing))

tbase=rows[0]["mono_ns"]

def tsec(r): return (r["mono_ns"]-tbase)*1e-9

def raw_vel(r):
    if r.get("valid",0)<0.5: return None
    h=r.get("luna_m",0.0); dt=r.get("dt_s",0.0)
    if not (math.isfinite(h) and 0.05<h<10 and 0<dt<0.2): return None
    vals=[r.get(k,float("nan")) for k in ("flow_body_x","flow_body_y","fc_gyro_x","fc_gyro_y","fc_roll","fc_pitch","fc_yaw")]
    if not all(math.isfinite(v) for v in vals): return None
    fx,fy,gx,gy,roll,pitch,yaw=vals
    comp_x=-fx+gx; comp_y=-fy+gy
    vbx=-comp_y*h; vby=comp_x*h
    cr=math.cos(roll); sr=math.sin(roll)
    cp=math.cos(pitch); sp=math.sin(pitch)
    cy=math.cos(yaw); sy=math.sin(yaw)
    r00=cy*cp
    r01=cy*sp*sr-sy*cr
    r10=sy*cp
    r11=sy*sp*sr+cy*cr
    return r00*vbx+r01*vby, r10*vbx+r11*vby

def ekf_vel(r):
    if r.get("ekf_local_valid",0)<0.5: return None
    vn=r.get("ekf_vx_ned",float("nan")); ve=r.get("ekf_vy_ned",float("nan"))
    if not (math.isfinite(vn) and math.isfinite(ve)): return None
    return vn,ve

def mag(v):
    return math.hypot(v[0],v[1]) if v else float("nan")

# Build 0.5 s bins across whole run.
bin_s=0.5
T=max(tsec(r) for r in rows)
nb=int(math.ceil(T/bin_s))
bins=[]
for i in range(nb):
    a=i*bin_s; b=a+bin_s
    rr=[r for r in rows if a<=tsec(r)<b]
    if not rr: continue
    rv=[mag(raw_vel(r)) for r in rr]; rv=[x for x in rv if math.isfinite(x)]
    ev=[mag(ekf_vel(r)) for r in rr]; ev=[x for x in ev if math.isfinite(x)]
    gx=[abs(r.get("fc_gyro_x",0.0)) for r in rr]
    gy=[abs(r.get("fc_gyro_y",0.0)) for r in rr]
    gz=[abs(r.get("fc_gyro_z",0.0)) for r in rr]
    lat=[r.get("frame_pipeline_latency_ms",float("nan")) for r in rr]
    lat=[x for x in lat if math.isfinite(x)]
    drop=sum(1 for r in rr if r.get("camera_queue_dropped",0)>0)
    bins.append({
      "a":a,"b":b,
      "raw":statistics.fmean(rv) if rv else float("nan"),
      "ekf":statistics.fmean(ev) if ev else float("nan"),
      "gyro":statistics.fmean([math.sqrt(x*x+y*y+z*z) for x,y,z in zip(gx,gy,gz)]) if rr else float("nan"),
      "lat":statistics.fmean(lat) if lat else float("nan"),
      "drop_frac":drop/max(1,len(rr)),
      "n":len(rr)
    })

# Episodes where RAW is almost stopped but EKF still moving fast.
candidates=[x for x in bins if math.isfinite(x["raw"]) and math.isfinite(x["ekf"]) and x["raw"]<0.01 and x["ekf"]>0.05]
# Merge consecutive bins.
episodes=[]
for x in candidates:
    if not episodes or x["a"]-episodes[-1][-1]["b"]>1e-6:
        episodes.append([x])
    else:
        episodes[-1].append(x)

print(f"CSV: {path}")
print(f"duration_s={T:.3f} rows={len(rows)}")
print()
print("===== RAW≈0 BUT EKF MOVING EPISODES =====")
if not episodes:
    print("none with RAW<0.01 m/s and EKF>0.05 m/s")
else:
    for j,ep in enumerate(episodes,1):
        a=ep[0]["a"]; b=ep[-1]["b"]
        print(f"#{j}: {a:.2f}..{b:.2f}s  dur={b-a:.2f}s  "
              f"RAW={statistics.fmean(x['raw'] for x in ep):.4f} m/s  "
              f"EKF={statistics.fmean(x['ekf'] for x in ep):.4f} m/s  "
              f"gyro={statistics.fmean(x['gyro'] for x in ep):.4f} rad/s  "
              f"lat={statistics.fmean(x['lat'] for x in ep):.1f} ms  "
              f"drop_frac={statistics.fmean(x['drop_frac'] for x in ep):.2f}")

print()
print("===== TOP EKF/RAW DIVERGENCE BINS =====")
rank=[]
for x in bins:
    if not (math.isfinite(x["raw"]) and math.isfinite(x["ekf"])): continue
    score=x["ekf"]/(x["raw"]+0.002)
    rank.append((score,x))
for score,x in sorted(rank,reverse=True)[:20]:
    print(f"{x['a']:7.2f}..{x['b']:7.2f}s  score={score:7.1f}  "
          f"RAW={x['raw']:.4f} EKF={x['ekf']:.4f} gyro={x['gyro']:.4f} "
          f"lat={x['lat']:.1f}ms drop={x['drop_frac']:.2f}")

# Detect the first strong post-motion divergence: previous 2 s had meaningful raw motion,
# current bin has raw near zero and EKF still fast.
print()
print("===== FIRST MOTION→STOP DIVERGENCE =====")
first=None
for i,x in enumerate(bins):
    if not (math.isfinite(x["raw"]) and math.isfinite(x["ekf"])): continue
    prev=[p for p in bins[max(0,i-4):i] if math.isfinite(p["raw"])]
    prev_raw=statistics.fmean(p["raw"] for p in prev) if prev else 0.0
    if prev_raw>0.05 and x["raw"]<0.01 and x["ekf"]>0.05:
        first=(i,x,prev_raw); break
if first is None:
    print("not found by thresholds")
else:
    i,x,prev_raw=first
    print(f"transition at ~{x['a']:.2f}s: previous RAW mean={prev_raw:.4f} m/s, "
          f"now RAW={x['raw']:.4f}, EKF={x['ekf']:.4f} m/s")
    lo=max(0,i-6); hi=min(len(bins),i+21)
    print("time_s,raw_mps,ekf_mps,gyro_rps,lat_ms,drop_frac")
    for p in bins[lo:hi]:
        print(f"{p['a']:.2f},{p['raw']:.6f},{p['ekf']:.6f},{p['gyro']:.6f},{p['lat']:.3f},{p['drop_frac']:.3f}")

print()
print("Interpretation:")
print("- If RAW drops near zero but EKF remains high, the visible runaway is carried by EKF state.")
print("- Inspect gyro/latency/drop around the transition to identify what injected the bad state during motion.")
