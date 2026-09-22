#!/usr/bin/env python3
import argparse,csv,math
from pathlib import Path

def f(r,k):
    try:return float(r[k])
    except:return float("nan")

def norm(x,y): return math.hypot(x,y)

def main():
    ap=argparse.ArgumentParser(description="Read-only WORKED5 pitch geometry forensic")
    ap.add_argument("csv",type=Path)
    ap.add_argument("--start-ns",type=int,required=True)
    ap.add_argument("--stop-ns",type=int,required=True)
    ap.add_argument("--gt-mm",type=float,default=None)
    ap.add_argument("--name",default="RUN")
    a=ap.parse_args()
    with a.csv.open(newline="") as fh:
        rows=[r for r in csv.DictReader(fh) if a.start_ns<=int(float(r["mono_ns"]))<=a.stop_ns]
    req=["worked5_valid","worked5_dx_m","worked5_dy_m","worked5_hcam_m","worked5_du_norm","worked5_dv_norm","fc_roll","fc_pitch"]
    if not rows: raise SystemExit("NO ROWS IN WINDOW")
    miss=[k for k in req if k not in rows[0]]
    if miss: raise SystemExit("MISSING COLUMNS: "+",".join(miss))
    v=[r for r in rows if int(float(r["worked5_valid"]))==1]
    if not v: raise SystemExit("NO VALID WORKED5 ROWS")
    sx=sum(f(r,"worked5_dx_m") for r in v); sy=sum(f(r,"worked5_dy_m") for r in v)
    # Diagnostic models only. No production correction is implied.
    # Model A: frozen W5.
    # Model B: divide each camera-plane step by cos(pitch).
    # Model C: divide by cos(roll)*cos(pitch).
    bx=by=cx=cy=0.0
    pitch=[]; roll=[]
    for r in v:
        dx=f(r,"worked5_dx_m"); dy=f(r,"worked5_dy_m")
        p=f(r,"fc_pitch"); q=f(r,"fc_roll")
        pitch.append(math.degrees(p)); roll.append(math.degrees(q))
        cp=math.cos(p); cr=math.cos(q)
        if abs(cp)>1e-6:
            bx+=dx/cp; by+=dy/cp
        if abs(cp*cr)>1e-6:
            cx+=dx/(cp*cr); cy+=dy/(cp*cr)
    def med(x):
        x=sorted(x); n=len(x); return x[n//2] if n%2 else .5*(x[n//2-1]+x[n//2])
    print("===== WORKED5 PITCH GEOMETRY FORENSIC =====")
    print("name =",a.name)
    print("rows =",len(rows),"valid =",len(v))
    print(f"roll_deg  median={med(roll):+.3f} min={min(roll):+.3f} max={max(roll):+.3f}")
    print(f"pitch_deg median={med(pitch):+.3f} min={min(pitch):+.3f} max={max(pitch):+.3f}")
    vals=[("FROZEN_W5",norm(sx,sy)),("PITCH_COS_SHADOW",norm(bx,by)),("ROLL_PITCH_COS_SHADOW",norm(cx,cy))]
    for n,m in vals:
        print(f"{n:24s} XY={m*1000.0:.3f} mm",end="")
        if a.gt_mm:
            print(f"  error={m*1000-a.gt_mm:+.3f} mm ({(m*1000/a.gt_mm-1)*100:+.2f}%)")
        else: print()
    print("NOTE: shadow models are diagnostics, not validated corrections.")

if __name__=="__main__": main()
