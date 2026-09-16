#!/usr/bin/env python3
"""Fast forensic for a few known bad canonical frame pairs.
Tests whether asymmetry comes from feature selection or survives with identical correspondences.
"""
import argparse, csv, struct, cv2, numpy as np
from pathlib import Path

PAIRS=[(2597,2598),(2600,2601),(2665,2666),(2666,2667)]
ROI=(.20,.32,.80,.90)

def load_frames(root, wanted):
    meta={}
    with open(root/"frames.csv",newline="") as f:
        for r in csv.DictReader(f):
            fr=int(r["frame"])
            if fr in wanted: meta[fr]=(int(r["camera_ts_ns"]),int(r["size"]))
    out={}
    with open(root/"frames.mjpgbin","rb") as f:
        while True:
            h=f.read(12)
            if not h: break
            ts,sz=struct.unpack("<QI",h)
            jpg=f.read(sz)
            for fr,(mts,msz) in meta.items():
                if fr not in out and ts==mts and sz==msz:
                    out[fr]=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_GRAYSCALE)
    return out

def features(im):
    x0,y0,x1,y1=ROI
    X0,Y0,X1,Y1=round(x0*im.shape[1]),round(y0*im.shape[0]),round(x1*im.shape[1]),round(y1*im.shape[0])
    pts=[]
    for gy in range(3):
      for gx in range(3):
        a=X0+(X1-X0)*gx//3;b=X0+(X1-X0)*(gx+1)//3
        c=Y0+(Y1-Y0)*gy//3;d=Y0+(Y1-Y0)*(gy+1)//3
        q=cv2.goodFeaturesToTrack(im[c:d,a:b],56,.01,7)
        if q is not None:
          q=q.reshape(-1,2);q[:,0]+=a;q[:,1]+=c;pts.extend(q.tolist())
    return np.float32(pts[:500]).reshape(-1,1,2)

def lk(a,b,p):
    q,st,_=cv2.calcOpticalFlowPyrLK(a,b,p,None,winSize=(21,21),maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,30,.01),flags=0,minEigThreshold=1e-4)
    st=st.ravel().astype(bool)
    return p[st].reshape(-1,2),q[st].reshape(-1,2)

def consensus(a,b):
    if len(a)<20:return 0,None
    H,m=cv2.findHomography(a,b,cv2.RANSAC,2.0,maxIters=350,confidence=.99)
    return (0 if m is None else int(m.sum())),m

def reciprocal_shared(im0,im1):
    p=features(im0); a,b=lk(im0,im1,p)
    # Forward-backward check creates IDENTICAL correspondence pairs for both model fits.
    br,st,_=cv2.calcOpticalFlowPyrLK(im1,im0,b.reshape(-1,1,2),None,winSize=(21,21),maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,30,.01),flags=0,minEigThreshold=1e-4)
    br=br.reshape(-1,2); fb=np.linalg.norm(br-a,axis=1)
    keep=fb<=2.0; a=a[keep];b=b[keep]
    fi,fm=consensus(a,b);ri,rm=consensus(b,a)
    overlap=0
    if fm is not None and rm is not None:
        F=fm.ravel().astype(bool);R=rm.ravel().astype(bool);overlap=int(np.logical_and(F,R).sum())
    return len(p),len(a),fi,ri,overlap

def main():
    ap=argparse.ArgumentParser();ap.add_argument("directory");args=ap.parse_args()
    root=Path(args.directory);wanted={x for p in PAIRS for x in p};ims=load_frames(root,wanted)
    missing=sorted(wanted-set(ims)); 
    if missing: raise SystemExit(f"missing frames: {missing}")
    print("===== FIXED-CORRESPONDENCE RANSAC FORENSIC =====")
    print("Same LK correspondences are fitted as A->B and B->A; no reverse feature re-detection.")
    print("pair seed_features fb_shared fwd_inliers rev_inliers shared_inlier_overlap")
    for a,b in PAIRS:
        vals=reciprocal_shared(ims[a],ims[b])
        print(f"{a}->{b} {vals[0]} {vals[1]} {vals[2]} {vals[3]} {vals[4]}")
if __name__=="__main__": main()
