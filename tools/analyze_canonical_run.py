#!/usr/bin/env python3
"""Forensic analysis of a monkeysStab canonical A-B-H run.

Measurement-only: does not modify calibration, runtime config, or focal_scale.
"""
import argparse, csv, math
from collections import Counter
from pathlib import Path
from statistics import median

def fval(r,k):
    try: return float(r[k])
    except (KeyError,TypeError,ValueError): return float("nan")

def ival(r,k):
    try: return int(float(r[k]))
    except (KeyError,TypeError,ValueError): return 0

def height(r):
    # Current canonical CSV does not log current_camera_height_m separately.
    # Reconstruct the exact production geometry used in this mount:
    # hcam = range_to_fc - (camera_z-range_z) = range + 0.005 m.
    h=fval(r,"range_to_fc_m")
    return h+0.005 if math.isfinite(h) else float("nan")

def integrate_angular(rows,lo,hi,xcol,ycol):
    x=y=0.; n=0
    for r in rows[lo:hi+1]:
        if ival(r,"valid")!=1: continue
        dt,h,fx,fy=fval(r,"dt_s"),height(r),fval(r,xcol),fval(r,ycol)
        if not all(math.isfinite(v) for v in (dt,h,fx,fy)) or dt<=0: continue
        x+=fx*dt*h; y+=fy*dt*h; n+=1
    return x,y,n

def integrate_comp(rows,lo,hi):
    bx=by=nn=ee=0.; n=0
    for r in rows[lo:hi+1]:
        # Match production RAW diagnostic gate as closely as the CSV permits.
        if ival(r,"valid")!=1 or ival(r,"flow_sent")!=1: continue
        dt,h,fx,fy,gx,gy=(fval(r,k) for k in
            ("dt_s","range_to_fc_m","flow_send_x","flow_send_y","fc_gyro_x","fc_gyro_y"))
        h=height(r)
        roll,pitch,yaw=(fval(r,k) for k in ("fc_roll","fc_pitch","fc_yaw"))
        vals=(dt,h,fx,fy,gx,gy,roll,pitch,yaw)
        if not all(math.isfinite(v) for v in vals) or not (0<dt<0.2): continue
        # Exact equations from production web RAW diagnostic:
        comp_x=-fx+gx; comp_y=-fy+gy
        vbx=(-comp_y)*h; vby=comp_x*h
        cr,sr=math.cos(roll),math.sin(roll); cp,sp=math.cos(pitch),math.sin(pitch)
        cy,sy=math.cos(yaw),math.sin(yaw)
        r00=cy*cp; r01=cy*sp*sr-sy*cr
        r10=sy*cp; r11=sy*sp*sr+cy*cr
        vn=r00*vbx+r01*vby; ve=r10*vbx+r11*vby
        bx+=vbx*dt; by+=vby*dt; nn+=vn*dt; ee+=ve*dt; n+=1
    return bx,by,nn,ee,n

def basic_report(rows,ia,ib,ih,title,xcol,ycol,gt):
    def one(lo,hi): return integrate_angular(rows,lo,hi,xcol,ycol)
    abx,aby,nab=one(ia+1,ib); bax,bay,nba=one(ib+1,ih)
    ab=math.hypot(abx,aby); ba=math.hypot(bax,bay)
    print(f"\n===== {title} =====")
    print(f"A->B vector:       X={abx*1000:+.2f} Y={aby*1000:+.2f} mm")
    print(f"A->B magnitude:    {ab*1000:.2f} mm  samples={nab}")
    print(f"B->H magnitude:    {ba*1000:.2f} mm  samples={nba} (GT UNKNOWN)")
    print(f"A->B measured/GT:  {ab/gt:.6f}")
    print(f"A->B error:        {(ab-gt)*1000:+.2f} mm ({(ab/gt-1)*100:+.2f} %)")

def comp_report(rows,ia,ib,ih,gt):
    ab=integrate_comp(rows,ia+1,ib); ba=integrate_comp(rows,ib+1,ih)
    ab_b=math.hypot(ab[0],ab[1]); ab_n=math.hypot(ab[2],ab[3])
    ba_b=math.hypot(ba[0],ba[1]); ba_n=math.hypot(ba[2],ba[3])
    cn,ce=ab[2]+ba[2],ab[3]+ba[3]
    print("\n===== GYRO-COMPENSATED PRODUCTION MOTION =====")
    print("Uses the same flow_send + FC gyro + camera height equations as the production RAW diagnostic.")
    print(f"A->B BODY:         X={ab[0]*1000:+.2f} Y={ab[1]*1000:+.2f} mm")
    print(f"A->B BODY magnitude: {ab_b*1000:.2f} mm")
    print(f"A->B NED:          N={ab[2]*1000:+.2f} E={ab[3]*1000:+.2f} mm")
    print(f"A->B NED magnitude:  {ab_n*1000:.2f} mm  samples={ab[4]}")
    print(f"A->B measured/GT:    {ab_n/gt:.6f}")
    print(f"A->B error:          {(ab_n-gt)*1000:+.2f} mm ({(ab_n/gt-1)*100:+.2f} %)")
    print(f"B->H BODY magnitude: {ba_b*1000:.2f} mm (GT UNKNOWN)")
    print(f"B->H NED magnitude:  {ba_n*1000:.2f} mm  samples={ba[4]} (GT UNKNOWN)")
    print(f"NED closure residual: {math.hypot(cn,ce)*1000:.2f} mm")
    return ab_n

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("dataset",type=Path)
    ap.add_argument("--distance-mm",required=True,type=float)
    a=ap.parse_args()
    with (a.dataset/"optical_flow_mavlink.csv").open(newline="") as f: rows=list(csv.DictReader(f))
    ev={}
    for i,r in enumerate(rows):
        e=ival(r,"return_event")
        if e in (1,2,3) and e not in ev: ev[e]=i
    if any(e not in ev for e in (1,2,3)): raise SystemExit("missing A/B/H return_event markers")
    ia,ib,ih=ev[1],ev[2],ev[3]; gt=a.distance_mm/1000.
    print("===== CANONICAL FORENSIC =====")
    print(f"dataset: {a.dataset}")
    print(f"A row={ia+2}  B row={ib+2}  H row={ih+2}")
    print(f"ONLY exact ground truth: A->B={a.distance_mm:.2f} mm")
    basic_report(rows,ia,ib,ih,"NATIVE flow_body (angular flow × height; NOT final translation)","flow_body_x","flow_body_y",gt)
    basic_report(rows,ia,ib,ih,"ACTUAL flow_send (MAVLink angular flow × height; NOT final translation)","flow_send_x","flow_send_y",gt)
    comp_report(rows,ia,ib,ih,gt)
    seg=rows[ia+1:ih+1]
    bad=[r for r in seg if ival(r,"valid")!=1]
    dts=[fval(r,"dt_s") for r in seg if ival(r,"valid")==1 and math.isfinite(fval(r,"dt_s"))]
    print("\n===== DATA QUALITY A..H =====")
    print(f"rows={len(seg)} valid={len(seg)-len(bad)} invalid={len(bad)}")
    print(f"invalid reasons={dict(Counter(r.get('invalid_reason','') for r in bad))}")
    if dts: print(f"dt median/mean/max={median(dts)*1000:.3f}/{sum(dts)/len(dts)*1000:.3f}/{max(dts)*1000:.3f} ms")
    print("\n===== INTERPRETATION GUARD =====")
    print("The first two sections are angular-flow diagnostics, not final translational distance.")
    print("Use GYRO-COMPENSATED PRODUCTION MOTION for the 340 mm comparison.")
    print("B->H ground truth is unknown; closure includes hand-return error.")
    print("Do NOT change focal_scale from one measured A->B leg.")

if __name__=="__main__": main()
