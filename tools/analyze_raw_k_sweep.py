#!/usr/bin/env python3
# RAW A->B focal/K sweep. Uses replay output from the SAME saved JPEG pairs.
# It does not tune production and does not use GT while estimating motion.
import csv, math, os, subprocess, sys

if len(sys.argv) < 3:
    raise SystemExit("usage: analyze_raw_k_sweep.py DATASET_DIR GT_MM [REPLAY_BIN]")
d=os.path.abspath(sys.argv[1]); gt=float(sys.argv[2])
replay=sys.argv[3] if len(sys.argv)>3 else "/tmp/monkeys_replay_canonical_flow"
flow=os.path.join(d,"optical_flow_mavlink.csv")

with open(flow,newline="") as f: fr=list(csv.DictReader(f))
# Legacy capture used 1/2; current BLIND4 capture uses 11/12 for A1/B1.
def event_frame(*events):
    for event in events:
        for r in fr:
            if r.get("return_event")==event:
                return int(r["frame"])
    raise SystemExit(f"missing return_event; expected one of {events}")
A=event_frame("11","1")
B=event_frame("12","2")
if B <= A:
    raise SystemExit(f"invalid A/B event order: A={A}, B={B}")
range_by_frame={}
for r in fr:
    try: range_by_frame[int(r["frame"])]=float(r["range_to_fc_m"])
    except: pass

# Fixed, predeclared diagnostic scales. GT is NEVER used to choose them.
scales=[0.80,0.90,0.931,1.00,1.10,1.20]
print("="*78)
print("RAW SAME-PAIR K/D SWEEP")
print("dataset:",d)
print(f"A/B frames: {A}/{B}; GT disclosed only for final comparison: {gt:.3f} mm")
print("Estimator and image pairs are fixed; only fx/fy multiplier changes.")
print("="*78)
results=[]
for fs in scales:
    subprocess.run([replay,d,f"{fs:.6f}"],check=True,stdout=subprocess.DEVNULL)
    rp=os.path.join(d,f"canonical_cpp_replay_fs_{fs:.6f}.csv")
    with open(rp,newline="") as f: rr=list(csv.DictReader(f))
    X=Y=0.; n=0; bad=0
    for r in rr:
        frame=int(r["frame"])
        if not (A < frame <= B): continue
        if r["replay_valid"]!="1": bad+=1; continue
        h=range_by_frame.get(frame)
        if h is None: bad+=1; continue
        h += 0.005
        # estimate() stores normalized translation per frame:
        # flow_cam=[dv/dt,-du/dt], therefore integrated camera-plane vector
        # over this frame is [dv,-du]*h. Rotation of axes does not change magnitude.
        du=float(r["replay_du_norm"]); dv=float(r["replay_dv_norm"])
        X += dv*h; Y += -du*h; n+=1
    mag=1000*math.hypot(X,Y)
    err=mag-gt
    results.append((fs,mag,err,n,bad))
    print(f"fs={fs:5.3f}  metric={mag:9.3f} mm  error={err:+9.3f} mm ({100*err/gt:+7.3f}%)  valid={n} bad={bad}")

print("-"*78)
# Diagnostic only: interpolation tells whether K alone has enough leverage.
# This is explicitly NOT a calibration or a value to write into production.
cross=None
for a,b in zip(results,results[1:]):
    if (a[2]<=0<=b[2]) or (b[2]<=0<=a[2]):
        if b[1]!=a[1]:
            cross=a[0]+(gt-a[1])*(b[0]-a[0])/(b[1]-a[1])
        break
if cross is None:
    print("GT is not bracketed by the fixed K sweep -> K-only hypothesis lacks sufficient leverage in tested range.")
else:
    print(f"Diagnostic GT crossing by interpolation: fs~{cross:.6f} (DO NOT apply as calibration).")
print("Decision rule:")
print("  strong monotonic response + plausible independent fs => K/D branch remains plausible;")
print("  weak response / implausible crossing => reject K-only explanation and move to gyro/plane geometry.")
print("="*78)
