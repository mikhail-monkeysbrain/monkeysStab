#!/usr/bin/env python3
# Offline FUSED-V2 shadow evaluator.
# Frozen policy:
# BAD: eligible && inlier_ratio < 0.50 && inliers < 100
# RECOVER: two consecutive eligible frames with ratio > 0.70 && inliers >= 100
# pre-roll: 150 ms
# IMU: causal capture only (age_ms >= 0)
#
# This script evaluates a complete trajectory:
# healthy WORKED5 + IMU replacement over unhealthy interval + healthy WORKED5.
# It does not modify production estimator state.

import csv, sys, math
from pathlib import Path

if len(sys.argv) != 3:
    raise SystemExit("usage: analyze_fused_v2_full_shadow.py fused_v2_frame_capture.csv optical_flow_mavlink.csv")

fp, op = map(Path, sys.argv[1:])
fr = list(csv.DictReader(fp.open(newline="")))
pr = list(csv.DictReader(op.open(newline="")))

def F(x,k):
    try: return float(x[k])
    except: return float("nan")

def I(x,k):
    try: return int(float(x[k]))
    except: return 0

def eligible(x):
    return F(x,"dt_s") > 0 and I(x,"tracked") >= 20

def bad(x):
    return eligible(x) and F(x,"inlier_ratio") < 0.50 and I(x,"inliers") < 100

def good(x):
    return eligible(x) and F(x,"inlier_ratio") > 0.70 and I(x,"inliers") >= 100

if not fr or not pr:
    raise SystemExit("empty input")
if any(F(x,"age_ms") < 0 for x in fr):
    raise SystemExit("non-causal frame capture detected: age_ms < 0")

# Restrict frame capture to the current production CSV frame domain. This also
# prevents an accidentally persistent diagnostic stream from selecting old runs.
prod_frames=[I(x,"frame") for x in pr if "frame" in x and I(x,"frame")>0]
if prod_frames:
    lo,hi=min(prod_frames),max(prod_frames)
    cur=[x for x in fr if lo <= I(x,"frame") <= hi]
else:
    # Fallback: camera timestamps, if production file does not expose frame.
    ts=[I(x,"now_ns") for x in pr if "now_ns" in x and I(x,"now_ns")>0]
    if not ts:
        ts=[I(x,"mono_ns") for x in pr if "mono_ns" in x and I(x,"mono_ns")>0]
    if not ts:
        raise SystemExit("cannot establish current-run frame/time domain from production CSV")
    lo,hi=min(ts),max(ts)
    cur=[x for x in fr if lo <= I(x,"cam_ns") <= hi]

if not cur:
    raise SystemExit("no frame-capture rows overlap current production run")

# Production W5 increments indexed by frame. Header names are the current
# production names used by the existing diagnostics.
w5={}
for x in pr:
    f=I(x,"frame")
    if f<=0 or I(x,"worked5_valid")!=1:
        continue
    dn=F(x,"worked5_dN_m"); de=F(x,"worked5_dE_m")
    if math.isfinite(dn) and math.isfinite(de):
        w5[f]=(dn,de)

# Baseline: full WORKED5 trajectory.
base_n=sum(v[0] for v in w5.values())
base_e=sum(v[1] for v in w5.values())

# Shadow starts from W5 and replaces only each detected unhealthy interval.
shadow_n=base_n
shadow_e=base_e
events=[]
i=0
last_end_frame=-1

while i < len(cur):
    if not bad(cur[i]):
        i+=1
        continue

    bi=i
    bad_ns=I(cur[bi],"cam_ns")
    target=bad_ns-150_000_000

    # causal pre-roll anchor: last camera frame at or before target
    candidates=[j for j in range(bi+1) if I(cur[j],"cam_ns") <= target]
    ai=candidates[-1] if candidates else 0

    # Avoid overlapping replacement intervals.
    if I(cur[ai],"frame") <= last_end_frame:
        i+=1
        continue

    ri=None
    for j in range(bi+1,len(cur)-1):
        if good(cur[j]) and good(cur[j+1]):
            ri=j+1
            break
    if ri is None:
        break

    af=I(cur[ai],"frame"); rf=I(cur[ri],"frame")

    # Remove all production W5 increments whose destination frame lies in the
    # replaced interval (anchor, recovery], then insert causal IMU delta.
    rem_n=rem_e=0.0
    for f,(dn,de) in w5.items():
        if af < f <= rf:
            rem_n+=dn; rem_e+=de

    imu_n=F(cur[ri],"imu_n_m")-F(cur[ai],"imu_n_m")
    imu_e=F(cur[ri],"imu_e_m")-F(cur[ai],"imu_e_m")

    shadow_n += imu_n-rem_n
    shadow_e += imu_e-rem_e

    events.append({
        "anchor":af,"bad":I(cur[bi],"frame"),"recover":rf,
        "removed_n":rem_n,"removed_e":rem_e,
        "imu_n":imu_n,"imu_e":imu_e,
        "anchor_age":F(cur[ai],"age_ms"),"recover_age":F(cur[ri],"age_ms")
    })
    last_end_frame=rf
    i=ri+1

print("frame capture current rows =",len(cur))
print("WORKED5 endpoint mm =",
      round(math.hypot(base_n,base_e)*1000,3),
      "N/E =",round(base_n*1000,3),round(base_e*1000,3))
print("bridge events =",len(events))

for k,e in enumerate(events,1):
    iv=math.hypot(e["imu_n"],e["imu_e"])*1000
    rv=math.hypot(e["removed_n"],e["removed_e"])*1000
    print(
        f"EVENT {k}: anchor={e['anchor']} bad={e['bad']} recover={e['recover']}",
        f"removed_W5={rv:.3f}mm",
        f"imu_delta={iv:.3f}mm",
        f"age_anchor/recover={e['anchor_age']:.3f}/{e['recover_age']:.3f}ms"
    )

print("FUSED-V2 FULL SHADOW endpoint mm =",
      round(math.hypot(shadow_n,shadow_e)*1000,3),
      "N/E =",round(shadow_n*1000,3),round(shadow_e*1000,3))
