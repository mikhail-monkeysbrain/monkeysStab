#!/usr/bin/env python3
import csv, math, os, sys

# CAUSAL SCALE-COHERENCE SHADOW V1
# Replay-only first step: every decision at frame k uses only values available
# at frame k or earlier. It never changes WORKED5/FUSED-V2/FC.
#
# Policy is deliberately parameter-free with respect to the discovery data:
#   coherent now  := holdout_valid && G > 0
#   coherent state:= two consecutive coherent frames to enter, two consecutive
#                    non-coherent frames to leave (symmetric debounce only).
# We do NOT use the discovered |scale|=8.4979e-4 boundary as a gate.
#
# This script does not yet replace WORKED5 scale. It measures when a causal
# classifier would label scale as spatially coherent and joins that decision to
# the exact realtime WORKED5 increments from fused_v2_realtime_shadow.csv.

def load_holdout(run):
    p=os.path.join(run,"stationary_balanced_shadow.csv")
    out={}
    with open(p,newline="") as f:
        for r in csv.DictReader(f):
            try:
                fr=int(float(r["frame"]))
                valid=int(float(r["holdout_valid"]))==1
                g=float(r["holdout_g"])
                sc=float(r["holdout_scale"])
                if math.isfinite(g) and math.isfinite(sc):
                    out[fr]=(valid,g,sc)
            except (KeyError,ValueError):
                pass
    return out

def load_w5(run):
    p=os.path.join(run,"fused_v2_realtime_shadow.csv")
    out=[]
    with open(p,newline="") as f:
        for r in csv.DictReader(f):
            try:
                fr=int(float(r["frame"]))
                new=int(float(r["new_w5"]))==1
                dn=float(r["w5_dN_m"]); de=float(r["w5_dE_m"])
                if math.isfinite(dn) and math.isfinite(de):
                    out.append((fr,new,dn,de))
            except (KeyError,ValueError):
                pass
    return out

def main(run):
    h=load_holdout(run); w=load_w5(run)
    state=False; pos_streak=0; neg_streak=0
    total=joined=coherent_frames=w5_steps=coherent_w5=0
    all_n=all_e=coh_n=coh_e=0.0
    transitions=0
    for fr,new,dn,de in w:
        total+=1
        x=h.get(fr)
        if x is None: continue
        joined+=1
        valid,g,sc=x
        now=valid and g>0.0
        if now:
            pos_streak+=1; neg_streak=0
        else:
            neg_streak+=1; pos_streak=0
        old=state
        if not state and pos_streak>=2: state=True
        elif state and neg_streak>=2: state=False
        if state!=old: transitions+=1
        if state: coherent_frames+=1
        if new:
            w5_steps+=1
            all_n+=dn; all_e+=de
            if state:
                coherent_w5+=1; coh_n+=dn; coh_e+=de

    print("CAUSAL SCALE-COHERENCE SHADOW V1")
    print("================================")
    print(f"run={run}")
    print(f"frames={total} joined={joined} ({joined/max(1,total):.4%})")
    print(f"coherent_state_frames={coherent_frames} fraction={coherent_frames/max(1,joined):.6f}")
    print(f"state_transitions={transitions}")
    print(f"w5_steps={w5_steps} coherent_w5_steps={coherent_w5} fraction={coherent_w5/max(1,w5_steps):.6f}")
    print(f"WORKED5 all increments: dN={all_n*1000:+.3f} mm dE={all_e*1000:+.3f} mm |d|={math.hypot(all_n,all_e)*1000:.3f} mm")
    print(f"WORKED5 increments while coherent: dN={coh_n*1000:+.3f} mm dE={coh_e*1000:+.3f} mm |d|={math.hypot(coh_n,coh_e)*1000:.3f} mm")
    print("NOTE: coherent-only displacement is diagnostic attribution, NOT a corrected trajectory.")
    print("No discovered |scale| threshold is used. WORKED5/FUSED-V2/FC unchanged.")

if len(sys.argv)!=2:
    print("usage: analyze_causal_scale_coherence_shadow.py RUN")
    raise SystemExit(2)
main(sys.argv[1])
