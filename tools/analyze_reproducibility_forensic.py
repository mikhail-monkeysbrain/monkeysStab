#!/usr/bin/env python3
"""
Episode-based read-only forensic analyzer for monkeysStab.

IMPORTANT:
- does not change runtime, FC parameters, logs, or production code;
- does not infer physical direction or ground truth;
- detects motion episodes only from logged WORKED5 per-frame BODY displacement;
- reports where logged layers diverge: BODY -> N/E -> publish -> EKF3.

This V2 intentionally lives on a diagnostic-only branch.
"""
from __future__ import annotations
import argparse, csv, math, time
from pathlib import Path

def F(r,k):
    try:
        v=float(r.get(k,""))
        return v if math.isfinite(v) else None
    except (TypeError,ValueError): return None

def B(r,k):
    v=F(r,k); return v is not None and v!=0.0

def fmt(v,scale=1.0,n=3):
    return "N/A" if v is None or not math.isfinite(v) else f"{v*scale:.{n}f}"

def percentile(a,p):
    a=sorted(v for v in a if v is not None and math.isfinite(v))
    if not a:return None
    return a[min(len(a)-1,max(0,round((len(a)-1)*p)))]

def angle_delta(a,b):
    return math.atan2(math.sin(b-a),math.cos(b-a))

def unwrap_span(vals):
    a=[v for v in vals if v is not None]
    if len(a)<2:return None
    u=[a[0]]
    for v in a[1:]: u.append(u[-1]+angle_delta(u[-1],v))
    return max(u)-min(u)

def find_csv(p):
    p=Path(p).expanduser()
    if p.is_file(): return p
    q=p/"optical_flow_mavlink.csv"
    if q.is_file(): return q
    raise FileNotFoundError(p)

def load(path):
    path=find_csv(path)
    with path.open(newline="") as f:return path,list(csv.DictReader(f))

def mono_s(r):
    v=F(r,"mono_ns")
    return None if v is None else v/1e9

def step_body_mm(r):
    if not B(r,"worked5_valid"): return 0.0
    x=F(r,"worked5_dx_m") or 0.0; y=F(r,"worked5_dy_m") or 0.0
    return 1000.0*math.hypot(x,y)

def episodes(rows, start_mm, stop_mm, start_frames, stop_frames, pad_s, merge_gap_s, min_motion_mm):
    # Hysteresis over per-frame WORKED5 BODY displacement.
    raw=[]; active=False; s=None; hi=lo=0; last_motion=None
    for i,r in enumerate(rows):
        m=step_body_mm(r)
        hi = hi+1 if m>=start_mm else 0
        lo = lo+1 if m<=stop_mm else 0
        if not active and hi>=start_frames:
            s=max(0,i-start_frames+1); active=True; lo=0
        if active and m>stop_mm: last_motion=i
        if active and lo>=stop_frames:
            e=max(s,i-stop_frames)
            raw.append((s,e)); active=False; hi=lo=0; s=None
    if active and s is not None: raw.append((s,len(rows)-1))

    # pad using timestamps, then merge close episodes
    padded=[]
    for s,e in raw:
        ts=mono_s(rows[s]); te=mono_s(rows[e])
        while s>0 and ts is not None and mono_s(rows[s-1]) is not None and ts-mono_s(rows[s-1])<=pad_s:
            s-=1
        while e+1<len(rows) and te is not None and mono_s(rows[e+1]) is not None and mono_s(rows[e+1])-te<=pad_s:
            e+=1
        padded.append((s,e))
    merged=[]
    for s,e in padded:
        if merged:
            pe=merged[-1][1]; a=mono_s(rows[pe]); b=mono_s(rows[s])
            if a is not None and b is not None and b-a<=merge_gap_s:
                merged[-1]=(merged[-1][0],e); continue
        merged.append((s,e))

    out=[]
    for s,e in merged:
        dist=sum(step_body_mm(r) for r in rows[s:e+1])
        if dist>=min_motion_mm: out.append((s,e))
    return out

def endpoint(rows,keys):
    good=[]
    for r in rows:
        v=tuple(F(r,k) for k in keys)
        if all(x is not None for x in v):good.append(v)
    if len(good)<2:return None
    return tuple(b-a for a,b in zip(good[0],good[-1]))

def summarize(rows,s,e):
    a=rows[s:e+1]; w=[r for r in a if B(r,"worked5_valid")]
    dx=sum(F(r,"worked5_dx_m") or 0 for r in w); dy=sum(F(r,"worked5_dy_m") or 0 for r in w)
    dn=sum(F(r,"worked5_dN_m") or 0 for r in w); de=sum(F(r,"worked5_dE_m") or 0 for r in w)
    body=math.hypot(dx,dy); ne=math.hypot(dn,de)
    ek=endpoint([r for r in a if B(r,"ekf_local_valid")],("ekf_x_ned","ekf_y_ned","ekf_z_ned"))
    roll=[F(r,"fc_roll") for r in a]; pitch=[F(r,"fc_pitch") for r in a]; yaw=[F(r,"fc_yaw") for r in a]
    h=[F(r,"worked5_hcam_m") for r in w]; la=[F(r,"luna_age_ms") for r in a]
    aa=[F(r,"fc_gyro_age_ms") for r in a]; ea=[F(r,"ekf_age_ms") for r in a]
    sent=sum(B(r,"flow_sent") for r in a); ready=sum(B(r,"stabilised_publish_ready") for r in a)
    valid=sum(B(r,"valid") for r in a)
    t0=mono_s(a[0]); t1=mono_s(a[-1]); dur=(t1-t0) if t0 is not None and t1 is not None else None

    # Detect logged EKF jumps between adjacent valid samples; threshold is diagnostic, not GT.
    jumps=[]; prev=None
    for j,r in enumerate(a):
        cur=tuple(F(r,k) for k in ("ekf_x_ned","ekf_y_ned","ekf_z_ned"))
        if all(v is not None for v in cur):
            if prev is not None:
                d=math.sqrt(sum((x-y)**2 for x,y in zip(cur,prev)))
                if d>0.25:jumps.append((j,d))
            prev=cur

    flags=[]
    if len(a) and sent/len(a)<0.98:flags.append("PUBLISH_GAP")
    if percentile(aa,.95) is not None and percentile(aa,.95)>100:flags.append("ATTITUDE_STALE")
    if jumps:flags.append("EKF_DISCONTINUITY")
    rs=unwrap_span(roll); ps=unwrap_span(pitch); ys=unwrap_span(yaw)
    if (rs is not None and abs(math.degrees(rs))>10) or (ps is not None and abs(math.degrees(ps))>10):flags.append("HIGH_TILT")
    if ys is not None and abs(math.degrees(ys))>15:flags.append("YAW_CHANGE")
    if body>0 and abs(ne-body)/body>0.10:flags.append("BODY_NE_DIVERGENCE")

    return dict(s=s,e=e,n=len(a),dur=dur,valid=100*valid/len(a),w5=100*len(w)/len(a),
      sent=100*sent/len(a),ready=100*ready/len(a),dx=dx,dy=dy,body=body,dn=dn,de=de,ne=ne,
      ekdn=ek[0] if ek else None,ekde=ek[1] if ek else None,ekdz=ek[2] if ek else None,
      ekmag=math.hypot(ek[0],ek[1]) if ek else None,
      roll=math.degrees(rs) if rs is not None else None,pitch=math.degrees(ps) if ps is not None else None,
      yaw=math.degrees(ys) if ys is not None else None,hmed=percentile(h,.5),hmin=percentile(h,0),hmax=percentile(h,1),
      lap95=percentile(la,.95),aap95=percentile(aa,.95),eap95=percentile(ea,.95),jumps=len(jumps),flags=flags)

def progress(i,n,start,label):
    elapsed=time.monotonic()-start; rate=elapsed/i if i else 0; left=rate*(n-i)
    pct=100*i/n; bars=int(pct/5)
    print(f"\r[{'█'*bars}{'░'*(20-bars)}] {pct:5.1f}% | {i}/{n} | {elapsed:6.1f}s | осталось ~{left:6.1f}s | {label[:30]:30s}",end="",flush=True)

def main():
    p=argparse.ArgumentParser()
    p.add_argument("runs",nargs="+")
    p.add_argument("--start-mm",type=float,default=0.35,help="per-frame BODY threshold to enter motion")
    p.add_argument("--stop-mm",type=float,default=0.15,help="per-frame BODY threshold to leave motion")
    p.add_argument("--start-frames",type=int,default=3)
    p.add_argument("--stop-frames",type=int,default=12)
    p.add_argument("--pad-s",type=float,default=0.15)
    p.add_argument("--merge-gap-s",type=float,default=0.35)
    p.add_argument("--min-motion-mm",type=float,default=20.0)
    args=p.parse_args()

    start=time.monotonic(); all_ep=[]
    for i,x in enumerate(args.runs,1):
        path,rows=load(x)
        eps=episodes(rows,args.start_mm,args.stop_mm,args.start_frames,args.stop_frames,args.pad_s,args.merge_gap_s,args.min_motion_mm)
        ss=[summarize(rows,s,e) for s,e in eps]
        all_ep.append((path,ss))
        progress(i,len(args.runs),start,path.name)
    print()

    print("\n===== EPISODE FORENSIC V2 =====")
    total=0
    for path,ss in all_ep:
        print(f"\nFILE: {path}")
        print(f"episodes={len(ss)}")
        for k,z in enumerate(ss,1):
            total+=1
            flags=",".join(z["flags"]) if z["flags"] else "NONE"
            print(f"  E{k:03d} rows={z['s']}..{z['e']} n={z['n']} dur={fmt(z['dur'])}s flags={flags}")
            print(f"       BODY dX/dY/|XY|={fmt(z['dx'],1000)}/{fmt(z['dy'],1000)}/{fmt(z['body'],1000)} mm  W5={z['w5']:.2f}%")
            print(f"       N/E dN/dE/|NE|={fmt(z['dn'],1000)}/{fmt(z['de'],1000)}/{fmt(z['ne'],1000)} mm")
            print(f"       EKF dN/dE/dZ/|XY|={fmt(z['ekdn'],1000)}/{fmt(z['ekde'],1000)}/{fmt(z['ekdz'],1000)}/{fmt(z['ekmag'],1000)} mm jumps={z['jumps']}")
            print(f"       spans roll/pitch/yaw={fmt(z['roll'])}/{fmt(z['pitch'])}/{fmt(z['yaw'])} deg")
            print(f"       hcam med/min/max={fmt(z['hmed'],1000)}/{fmt(z['hmin'],1000)}/{fmt(z['hmax'],1000)} mm")
            print(f"       valid={z['valid']:.2f}% sent={z['sent']:.2f}% ready={z['ready']:.2f}% age95 Luna/ATT/EKF={fmt(z['lap95'])}/{fmt(z['aap95'])}/{fmt(z['eap95'])} ms")

    print(f"\nTOTAL EPISODES: {total}")
    print("\n===== FORENSIC CONTRACT =====")
    print("Эпизоды выделены только по logged WORKED5 BODY activity.")
    print("Это НЕ физический ground truth и НЕ утверждение о направлении аппарата.")
    print("Ошибка относительно реальной дистанции без отдельного GT не вычисляется.")

if __name__=="__main__":main()
