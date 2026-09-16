#!/usr/bin/env python3
"""SERIES_D — high-FPS guided ChArUco intrinsic dataset capture.

Operator freely moves/tilts the UAV above a fixed horizontal board.
All 120-fps frames are inspected, but only geometrically useful, sharp and
sufficiently different observations are persisted. Guidance is based on
coverage and projective pose classes, not a fixed frame count.
"""
import argparse, csv, math, time
from pathlib import Path
from collections import deque
import cv2
import numpy as np

W,H=640,480
S=.027315
M=.020031
dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try: board=cv2.aruco.CharucoBoard((7,5),S,M,dic)
except TypeError: board=cv2.aruco.CharucoBoard_create(7,5,S,M,dic)
OBJ=np.asarray(board.getChessboardCorners(),np.float32)
# Approximate K is used ONLY for pose/guidance classification, never as calibration output.
K_GUIDE=np.array([[568.5317075,0,320.0],[0,569.6800556,240.0],[0,0,1]],np.float64)
D_GUIDE=np.zeros(5,np.float64)

def detect(g):
    mc,mi,_=cv2.aruco.detectMarkers(g,dic)
    if mi is None or len(mi)<2:return mc,mi,None,None
    _,cc,ci=cv2.aruco.interpolateCornersCharuco(mc,mi,g,board)
    return mc,mi,cc,ci

def homography_metrics(cc,ci):
    ids=ci.reshape(-1).astype(int)
    obj=OBJ[ids,:2].astype(np.float32)
    img=cc.reshape(-1,2).astype(np.float32)
    if len(obj)<6:return None
    Hm,mask=cv2.findHomography(obj,img,cv2.RANSAC,2.0)
    if Hm is None:return None
    # Vanishing/projective terms normalized by the affine scale.
    scale=max(1e-9,math.sqrt(Hm[0,0]**2+Hm[1,0]**2))
    px=float(Hm[2,0]/scale); py=float(Hm[2,1]/scale)
    p=img
    x0,y0=p.min(0);x1,y1=p.max(0);cx,cy=p.mean(0)
    area=max(1.,float((x1-x0)*(y1-y0)))
    return np.array([cx/W,cy/H,(x1-x0)/W,(y1-y0)/H,px*S,py*S],float),area

def pose_tilt(cc,ci):
    """Board-normal tilt from calibrated homography decomposition (guidance only)."""
    ids=ci.reshape(-1).astype(int)
    obj=OBJ[ids,:2].astype(np.float32)
    img=cc.reshape(-1,2).astype(np.float32)
    if len(obj)<6:return None
    Hm,_=cv2.findHomography(obj,img,cv2.RANSAC,2.0)
    if Hm is None:return None
    A=np.linalg.inv(K_GUIDE)@Hm
    a1=A[:,0]; a2=A[:,1]
    if np.linalg.norm(a1)<1e-9 or np.linalg.norm(a2)<1e-9:return None
    r1=a1/np.linalg.norm(a1); r2=a2/np.linalg.norm(a2)
    n=np.cross(r1,r2)
    nn=np.linalg.norm(n)
    if nn<1e-9:return None
    n=n/nn
    if n[2]<0:n=-n
    tx=math.degrees(math.atan2(float(n[0]),float(n[2])))
    ty=math.degrees(math.atan2(float(n[1]),float(n[2])))
    return tx,ty

def focus_score(g,cc):
    p=cc.reshape(-1,2);x0,y0=np.floor(p.min(0)-8).astype(int);x1,y1=np.ceil(p.max(0)+8).astype(int)
    x0=max(0,x0);y0=max(0,y0);x1=min(W,x1);y1=min(H,y1)
    roi=g[y0:y1,x0:x1]
    return 0. if roi.size<100 else float(cv2.Laplacian(roi,cv2.CV_64F).var())

def cell(v,n): return min(n-1,max(0,int(v*n)))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--camera",default="/dev/video0")
    ap.add_argument("--max-frames",type=int,default=600)
    ap.add_argument("--min-corners",type=int,default=8)
    ap.add_argument("--min-focus",type=float,default=35.)
    ap.add_argument("--min-interval",type=float,default=.12)
    ap.add_argument("--novelty",type=float,default=.055)
    a=ap.parse_args()
    out=Path(f"/home/vio/charuco_series_D_{time.strftime('%Y%m%d_%H%M%S')}")
    out.mkdir(parents=True)
    cap=cv2.VideoCapture(a.camera,cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"));cap.set(cv2.CAP_PROP_FRAME_WIDTH,W);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,H);cap.set(cv2.CAP_PROP_FPS,120)
    if not cap.isOpened():raise SystemExit("camera open failed")
    coverage=np.zeros((3,3),int)
    tilt=np.zeros(5,int) # frontal,left,right,up,down projective classes
    scale=np.zeros(3,int)
    desc=[];meta=[];last=0.;fpsq=deque(maxlen=120);frames_seen=0
    print("SERIES_D AUTO: move AND tilt UAV continuously; Q/ESC stops. No fixed target count.")
    while len(meta)<a.max_frames:
        ok,im=cap.read()
        if not ok:continue
        frames_seen+=1;now=time.monotonic();fpsq.append(now)
        g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);mc,mi,cc,ci=detect(g)
        nm=0 if mi is None else len(mi);nc=0 if ci is None else len(ci)
        reason=f"corners {nc}/{a.min_corners}";ready=False;d=None;nov=0.;fs=0.;area=0.;tc=-1;sc=-1;zy=zx=-1
        if cc is not None and nc>=a.min_corners:
            hm=homography_metrics(cc,ci)
            if hm is not None:
                d,area=hm;fs=focus_score(g,cc);zx=cell(d[0],3);zy=cell(d[1],3)
                sc=0 if d[2]<.28 else (1 if d[2]<.48 else 2)
                pt=pose_tilt(cc,ci)
                if pt is None:
                    reason="pose solve failed"
                    continue
                tx,ty=pt;thr=8.0
                if abs(tx)<thr and abs(ty)<thr:tc=0
                elif abs(tx)>=abs(ty):tc=1 if tx<0 else 2
                else:tc=3 if ty<0 else 4
                weights=np.array([1.4,1.4,1.5,1.5,7.,7.])
                nov=999. if not desc else min(float(np.linalg.norm((d-o)*weights)) for o in desc)
                # Capture is driven only by observable pose/scale diversity. Image-cell coverage\n                # is intentionally excluded because the fixed stand ring occludes part of the frame.\n                need=(tilt[tc]<60) or (scale[sc]<60)
                if fs<a.min_focus:reason=f"BLUR focus={fs:.0f}"
                elif nov<a.novelty:reason=f"DUPLICATE novelty={nov:.3f}"
                elif not need:reason="well-covered geometry"
                elif now-last<a.min_interval:reason="rate gate"
                else:ready=True;reason="AUTO SAVE"
        if ready:
            n=len(meta)+1;fn=f"frame_{n:04d}.jpg";cv2.imwrite(str(out/fn),im)
            coverage[zy,zx]+=1;tilt[tc]+=1;scale[sc]+=1;desc.append(d.copy());last=now
            meta.append([fn,frames_seen,nm,nc,fs,nov,*d,tx,ty,zy,zx,tc,sc])
            print(f"SAVE {n:04d} cell={zy},{zx} tilt={tc} pose=({tx:+.1f},{ty:+.1f})deg scale={sc} corners={nc} focus={fs:.0f} novelty={nov:.3f}")
        vis=im.copy()
        if mi is not None:cv2.aruco.drawDetectedMarkers(vis,mc,mi)
        if cc is not None:cv2.aruco.drawDetectedCornersCharuco(vis,cc,ci)
        for x in (W//3,2*W//3):cv2.line(vis,(x,0),(x,H-1),(255,255,255),1)
        for y in (H//3,2*H//3):cv2.line(vis,(0,y),(W-1,y),(255,255,255),1)
        fps=0 if len(fpsq)<2 else (len(fpsq)-1)/(fpsq[-1]-fpsq[0])
        cv2.putText(vis,f"seen {frames_seen} saved {len(meta)} detect {nc} fps {fps:.0f}",(8,20),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1)
        cv2.putText(vis,reason,(8,H-12),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,255,0) if ready else (0,0,255),2)
        # Guidance = least represented observable requirement.
        cy,cx=np.unravel_index(np.argmin(coverage),coverage.shape)
        tnames=["FRONTAL","ROLL/LEFT-TILT","ROLL/RIGHT-TILT","PITCH/UP-TILT","PITCH/DOWN-TILT"]
        snames=["FAR/SMALL","MID","NEAR/LARGE"]
        guides=[(coverage[cy,cx],f"NEED IMAGE AREA row={cy+1} col={cx+1}"),(tilt.min(),f"NEED {tnames[int(np.argmin(tilt))]}"),(scale.min(),f"NEED {snames[int(np.argmin(scale))]}")]
        guide=min(guides,key=lambda x:x[0])[1]
        cv2.putText(vis,guide,(8,42),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,255,255),2)
        cv2.imshow("ChArUco SERIES_D HIGH-FPS GUIDED",vis)
        k=cv2.waitKey(1)&255
        if k in (27,ord('q'),ord('Q')):break
    cap.release();cv2.destroyAllWindows()
    with (out/"capture.csv").open("w",newline="") as f:
        w=csv.writer(f);w.writerow(["file","source_frame","markers","corners","focus","novelty","cx_norm","cy_norm","w_norm","h_norm","proj_x","proj_y","pose_tilt_x_deg","pose_tilt_y_deg","cell_y","cell_x","tilt_class","scale_class"]);w.writerows(meta)
    np.savetxt(out/"coverage_3x3.csv",coverage,fmt="%d",delimiter=",")
    print("\nSaved:",out);print("source frames inspected:",frames_seen,"accepted:",len(meta));print("coverage 3x3:\n",coverage);print("tilt [front,L,R,U,D]:",tilt.tolist());print("scale [far,mid,near]:",scale.tolist())

if __name__=="__main__":main()
