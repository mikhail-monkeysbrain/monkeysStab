#!/usr/bin/env python3
"""Сбор независимой ChArUco SERIES_B/C.
SPACE сохраняет лучший кадр из стабильного 24-кадрового окна.
Критерий окна: median >= 6 corners. Дополнительно действует diversity gate.
Никакого GT/focal fitting.
"""
import argparse,time,csv
from pathlib import Path
from collections import deque
import cv2,numpy as np

S=.027315; M=.020031
dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try: board=cv2.aruco.CharucoBoard((7,5),S,M,dic)
except TypeError: board=cv2.aruco.CharucoBoard_create(7,5,S,M,dic)

def det(g):
    mc,mi,_=cv2.aruco.detectMarkers(g,dic)
    if mi is None or len(mi)<2: return mc,mi,None,None
    _,cc,ci=cv2.aruco.interpolateCornersCharuco(mc,mi,g,board)
    return mc,mi,cc,ci

def desc(cc):
    p=cc.reshape(-1,2); x0,y0=p.min(0); x1,y1=p.max(0); cx,cy=p.mean(0)
    area=max(1.,(x1-x0)*(y1-y0)); scale=np.sqrt(area/(640*480))
    sx=np.std(p[:,0])/640; sy=np.std(p[:,1])/480
    return np.array([cx/640,cy/480,scale,sx,sy],float)

def novel(d,old):
    if not old: return True,999.
    w=np.array([1.5,1.5,2.5,1.,1.])
    ds=[float(np.linalg.norm((d-o)*w)) for o in old]
    return min(ds)>=.055,min(ds)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--series",choices=["B","C"],required=True)
    ap.add_argument("--camera",default="/dev/video0")
    ap.add_argument("--count",type=int,default=40)
    a=ap.parse_args()
    out=Path(f"/home/vio/charuco_series_{a.series}_{time.strftime('%Y%m%d_%H%M%S')}")
    out.mkdir(parents=True)
    cap=cv2.VideoCapture(a.camera,cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480); cap.set(cv2.CAP_PROP_FPS,120)
    if not cap.isOpened(): raise SystemExit("camera open failed")
    saved=[]; descs=[]; meta=[]; window=deque(maxlen=24)
    print(f"SERIES_{a.series}: SPACE saves best stable frame from 24-frame window; median >=6 + diversity gate; Q finish.")
    while True:
        ok,im=cap.read()
        if not ok: continue
        g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        mc,mi,cc,ci=det(g); nm=0 if mi is None else len(mi); nc=0 if ci is None else len(ci)
        d=desc(cc) if cc is not None and nc>=6 else None
        window.append((nc,nm,im.copy(),d))
        counts=np.asarray([q[0] for q in window],dtype=int)
        med=float(np.median(counts)) if len(counts) else 0.
        stable=len(window)>=12 and med>=6
        candidates=[q for q in window if q[0]>=6 and q[3] is not None]
        best=max(candidates,key=lambda q:q[0]) if candidates else None
        bnov=False; bdist=0.
        if best is not None: bnov,bdist=novel(best[3],descs)
        good=stable and best is not None and bnov

        vis=im.copy()
        if mi is not None: cv2.aruco.drawDetectedMarkers(vis,mc,mi)
        if cc is not None: cv2.aruco.drawDetectedCornersCharuco(vis,cc,ci)
        bcnt=best[0] if best else 0
        msg=f"saved {len(saved)}/{a.count} now {nc}/24 med {med:.1f} best {bcnt}/24 novelty {bdist:.3f}"
        cv2.putText(vis,msg,(8,24),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,255,0) if good else (0,0,255),2)
        status="SPACE ACCEPT" if good else ("hold: need median >=6" if not stable else "move/tilt/scale board")
        cv2.putText(vis,status,(8,47),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,0) if good else (0,0,255),2)
        cv2.imshow(f"ChArUco SERIES_{a.series}",vis)
        k=cv2.waitKey(1)&255
        if k==32:
            if not good:
                print("REJECT",msg); continue
            bnc,bnm,bim,bd=best
            p=out/f"frame_{len(saved)+1:03d}.jpg"
            cv2.imwrite(str(p),bim)
            saved.append(p); descs.append(bd)
            meta.append([p.name,bnm,bnc,*bd,bdist])
            print("ACCEPT",p.name,"markers",bnm,"corners",bnc,f"median={med:.1f} novelty={bdist:.3f}")
            window.clear()
        if k in (27,ord("q"),ord("Q")) or len(saved)>=a.count: break
    cap.release(); cv2.destroyAllWindows()
    with (out/"capture.csv").open("w",newline="") as f:
        w=csv.writer(f)
        w.writerow(["file","markers","corners","cx_norm","cy_norm","scale","spread_x","spread_y","novelty"])
        w.writerows(meta)
    print("Saved directory:",out); print("accepted:",len(saved))

if __name__=="__main__": main()
