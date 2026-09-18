#!/usr/bin/env python3
import argparse, csv, time
from pathlib import Path
import cv2
import numpy as np

def detect_grid(gray, max_features):
    h,w=gray.shape[:2]; pts=[]
    per=max(1, max_features//9)
    for gy in range(3):
        for gx in range(3):
            x0=gx*w//3; x1=(gx+1)*w//3
            y0=gy*h//3; y1=(gy+1)*h//3
            roi=gray[y0:y1,x0:x1]
            q=cv2.goodFeaturesToTrack(roi, per, 0.01, 7)
            if q is not None:
                q=q.reshape(-1,2); q[:,0]+=x0; q[:,1]+=y0
                pts.extend(q.tolist())
    if len(pts)<30:
        q=cv2.goodFeaturesToTrack(gray,max_features,0.01,7)
        pts=[] if q is None else q.reshape(-1,2).tolist()
    if len(pts)>max_features: pts=pts[:max_features]
    return np.asarray(pts,np.float32).reshape(-1,1,2)

def run_pair(a,b,cfg):
    p0=detect_grid(a,cfg["features"])
    if len(p0)<20: return len(p0),0,0,0.0,0.0
    t=time.perf_counter_ns()
    p1,st,err=cv2.calcOpticalFlowPyrLK(
        a,b,p0,None,winSize=(cfg["win"],cfg["win"]),
        maxLevel=cfg["level"],
        criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,cfg["iters"],0.01),
        flags=0,minEigThreshold=1e-4)
    ms=(time.perf_counter_ns()-t)*1e-6
    ok=st.reshape(-1).astype(bool)
    x0=p0.reshape(-1,2)[ok]; x1=p1.reshape(-1,2)[ok]
    if len(x0)<4: return len(p0),len(x0),0,ms,0.0
    M,mask=cv2.estimateAffinePartial2D(x0,x1,method=cv2.RANSAC,
                                      ransacReprojThreshold=2.0,
                                      maxIters=2000,confidence=0.99,
                                      refineIters=10)
    inl=0 if mask is None else int(mask.sum())
    disp=0.0
    if mask is not None and inl:
        d=x1[mask.reshape(-1).astype(bool)]-x0[mask.reshape(-1).astype(bool)]
        disp=float(np.linalg.norm(np.median(d,axis=0)))
    return len(p0),len(x0),inl,ms,disp

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("run",type=Path)
    ap.add_argument("--start",type=int,default=None)
    ap.add_argument("--end",type=int,default=None)
    args=ap.parse_args()
    cap=args.run/"lk_forensic"
    meta=list(csv.DictReader(open(cap/"frames.csv",newline="")))
    prod={}
    with open(args.run/"optical_flow_mavlink.csv",newline="") as f:
        for r in csv.DictReader(f):
            try: prod[int(r["frame"])]=r
            except: pass
    configs=[
      ("PROD",21,3,30,500),
      ("L2",21,2,30,500),
      ("ITER15",21,3,15,500),
      ("WIN15",15,3,30,500),
      ("FEAT250",21,3,30,250),
    ]
    configs=[{"name":n,"win":w,"level":l,"iters":it,"features":ft} for n,w,l,it,ft in configs]
    rows=[]
    for i in range(1,len(meta)):
        frame=int(meta[i]["frame"])
        if args.start is not None and frame<args.start: continue
        if args.end is not None and frame>args.end: continue
        pa=cap/meta[i-1]["jpeg"]; pb=cap/meta[i]["jpeg"]
        a=cv2.imread(str(pa),cv2.IMREAD_GRAYSCALE); b=cv2.imread(str(pb),cv2.IMREAD_GRAYSCALE)
        if a is None or b is None: continue
        pr=prod.get(frame,{})
        for cfg in configs:
            nf,nt,ni,ms,disp=run_pair(a,b,cfg)
            rows.append({
              "frame":frame,"variant":cfg["name"],"features":nf,"tracked":nt,
              "inliers":ni,"inlier_ratio":ni/max(nt,1),"lk_ms":ms,
              "pixel_disp":disp,
              "prod_lk_ms":pr.get("t_lk_ms",""),"prod_features":pr.get("features",""),
              "prod_tracked":pr.get("tracked",""),"prod_inliers":pr.get("inliers",""),
              "prod_dt_s":pr.get("dt_s",""),"prod_drop":pr.get("camera_queue_dropped","")
            })
    out=args.run/"lk_forensic_ab.csv"
    with open(out,"w",newline="") as f:
        wr=csv.DictWriter(f,fieldnames=rows[0].keys()); wr.writeheader(); wr.writerows(rows)
    print("OUT",out)
    for cfg in configs:
        rr=[r for r in rows if r["variant"]==cfg["name"]]
        slow=sum(r["lk_ms"]>20 for r in rr)
        med=np.median([r["lk_ms"] for r in rr]) if rr else 0
        mx=max([r["lk_ms"] for r in rr],default=0)
        medir=np.median([r["inlier_ratio"] for r in rr]) if rr else 0
        print(f'{cfg["name"]:8s} n={len(rr):3d} slow>20={slow:3d} lk_med={med:7.3f} lk_max={mx:7.3f} inlier_ratio_med={medir:.3f}')
    print("\nCRITICAL 1013..1025")
    for frame in range(1013,1026):
        z=[r for r in rows if r["frame"]==frame]
        if not z: continue
        print("frame",frame,"prodLK",z[0]["prod_lk_ms"],"prodInl",z[0]["prod_inliers"],
              " | ".join(f'{r["variant"]}: {r["lk_ms"]:.2f}ms {r["inliers"]}/{r["tracked"]}' for r in z))
if __name__=="__main__": main()
