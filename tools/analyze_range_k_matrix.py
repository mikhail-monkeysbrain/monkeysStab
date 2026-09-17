#!/usr/bin/env python3
# Diagnostic A/B matrix on the SAME saved RAW-derived canonical replay.
# Compares recorded TF-Luna range with a fixed 190 mm TF-Luna reading.
# focal scales are predeclared independent hypotheses; GT is used only for final error reporting.
import csv, math, os, subprocess, sys

if len(sys.argv) < 3:
    raise SystemExit("usage: analyze_range_k_matrix.py DATASET_DIR GT_MM [REPLAY_BIN]")

d=os.path.abspath(sys.argv[1])
gt=float(sys.argv[2])
replay=sys.argv[3] if len(sys.argv)>3 else "/tmp/monkeys_replay_canonical_flow"
flow_csv=os.path.join(d,"optical_flow_mavlink.csv")

with open(flow_csv,newline="") as f:
    flow=list(csv.DictReader(f))

def event_frame(*events):
    for event in events:
        for r in flow:
            if r.get("return_event")==event:
                return int(r["frame"])
    raise SystemExit(f"missing return_event; expected one of {events}")

A=event_frame("11","1")
B=event_frame("12","2")
if B <= A:
    raise SystemExit(f"invalid A/B order: {A}->{B}")

range_by_frame={}
for r in flow:
    try:
        range_by_frame[int(r["frame"])]=float(r["range_to_fc_m"])
    except Exception:
        pass

# fs=1.0 is saved calibration; fs=1.10 is the earlier independent ~1.09-1.10 hypothesis.
scales=[1.00,1.10]
# The existing canonical metric convention adds +5 mm camera-vs-Luna Z geometry.
# Therefore fixed Luna=190 mm corresponds to h=195 mm in this simplified integration,
# exactly as recorded Luna values are converted below.
range_modes=[("RECORDED",None),("FIXED_LUNA_190MM",0.190)]

print("="*88)
print("RAW RANGE x K DIAGNOSTIC MATRIX")
print("dataset:",d)
print(f"A/B frames: {A}/{B}; GT={gt:.3f} mm (comparison only)")
print("Same RAW, same accepted image motion; only metric range model / predeclared K scale changes.")
print("FIXED_LUNA_190MM means Luna=0.190 m; camera-plane height convention adds +0.005 m.")
print("="*88)

for fs in scales:
    subprocess.run([replay,d,f"{fs:.6f}"],check=True,stdout=subprocess.DEVNULL)
    rp=os.path.join(d,f"canonical_cpp_replay_fs_{fs:.6f}.csv")
    with open(rp,newline="") as f:
        rr=list(csv.DictReader(f))

    for name,fixed_luna in range_modes:
        X=Y=0.0; n=bad=0; weighted_h_num=weighted_h_den=0.0
        for r in rr:
            frame=int(r["frame"])
            if not (A < frame <= B):
                continue
            if r["replay_valid"]!="1":
                bad+=1; continue
            if fixed_luna is None:
                luna=range_by_frame.get(frame)
                if luna is None:
                    bad+=1; continue
            else:
                luna=fixed_luna
            h=luna+0.005
            du=float(r["replay_du_norm"]); dv=float(r["replay_dv_norm"])
            step=math.hypot(du,dv)
            X += dv*h
            Y += -du*h
            weighted_h_num += h*step
            weighted_h_den += step
            n+=1
        mag=1000.0*math.hypot(X,Y)
        err=mag-gt
        wh=1000.0*weighted_h_num/weighted_h_den if weighted_h_den else float("nan")
        print(f"fs={fs:4.2f}  range={name:18s}  metric={mag:9.3f} mm  error={err:+8.3f} mm ({100*err/gt:+7.3f}%)  flow_weighted_h={wh:7.3f} mm  valid={n} bad={bad}")

print("="*88)
print("Interpretation: compare RECORDED vs FIXED at the SAME fs to isolate the false Luna dynamics;")
print("then compare fs=1.00 vs 1.10 at the SAME range mode to isolate the independent K hypothesis.")
print("No value is fitted to GT and nothing is written to production configuration.")
