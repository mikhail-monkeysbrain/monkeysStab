#!/usr/bin/env python3
"""Строгая SERIES_B ChArUco capture для OV9281.
Требует >=18 углов; показывает bbox coverage и центр доски.
Физическая доска: 7x5, square 27.315 mm, marker 20.031 mm.
"""
import argparse,time
from pathlib import Path
import cv2,numpy as np
SQUARE=.027315;MARKER=.020031;SX,SY=7,5;DICT=cv2.aruco.DICT_4X4_50
def api():
 d=cv2.aruco.getPredefinedDictionary(DICT)
 try:b=cv2.aruco.CharucoBoard((SX,SY),SQUARE,MARKER,d)
 except TypeError:b=cv2.aruco.CharucoBoard_create(SX,SY,SQUARE,MARKER,d)
 return d,b
def det(g,d,b):
 mc,mi,_=cv2.aruco.detectMarkers(g,d)
 if mi is None or len(mi)<2:return mc,mi,None,None
 _,cc,ci=cv2.aruco.interpolateCornersCharuco(mc,mi,g,b);return mc,mi,cc,ci
def main():
 ap=argparse.ArgumentParser();ap.add_argument("--camera",default="/dev/video0");ap.add_argument("--count",type=int,default=24);ap.add_argument("--out",default="");a=ap.parse_args()
 out=Path(a.out or f"/home/vio/charuco_series_B_{time.strftime('%Y%m%d_%H%M%S')}");out.mkdir(parents=True,exist_ok=True)
 d,b=api();cap=cv2.VideoCapture(a.camera,cv2.CAP_V4L2);cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"));cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480);cap.set(cv2.CAP_PROP_FPS,120)
 if not cap.isOpened():raise SystemExit("Не удалось открыть "+a.camera)
 saved=[];print("SERIES_B: ПРОБЕЛ сохранить; Q/ESC закончить. Сохранение разрешено при >=18 ChArUco corners.")
 while True:
  ok,im=cap.read()
  if not ok:continue
  g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);mc,mi,cc,ci=det(g,d,b);vis=im.copy();n=0 if ci is None else len(ci)
  if mi is not None:cv2.aruco.drawDetectedMarkers(vis,mc,mi)
  cov=0;cx=cy=0
  if cc is not None:
   cv2.aruco.drawDetectedCornersCharuco(vis,cc,ci);p=cc.reshape(-1,2);x0,y0=p.min(0);x1,y1=p.max(0);cov=(x1-x0)*(y1-y0)/(640*480);cx,cy=p.mean(0)
   cv2.rectangle(vis,(int(x0),int(y0)),(int(x1),int(y1)),(255,255,255),1)
  good=n>=18
  cv2.putText(vis,f"saved {len(saved)}/{a.count} corners {n}/24 bbox {cov*100:.1f}% center {cx:.0f},{cy:.0f}",(8,25),cv2.FONT_HERSHEY_SIMPLEX,.52,(0,255,0) if good else (0,0,255),2)
  cv2.putText(vis,"SPACE enabled" if good else "Need >=18 corners",(8,48),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,0) if good else (0,0,255),2)
  cv2.imshow("OV9281 ChArUco SERIES_B",vis);k=cv2.waitKey(1)&255
  if k==32:
   if not good:print("SKIP: corners",n,"< 18");continue
   p=out/f"frame_{len(saved)+1:03d}.jpg";cv2.imwrite(str(p),im);saved.append(p);print("saved",p.name,"corners",n,f"bbox={cov*100:.1f}% center={cx:.0f},{cy:.0f}")
  if k in (27,ord("q"),ord("Q")) or len(saved)>=a.count:break
 cap.release();cv2.destroyAllWindows();print("\nSaved directory:",out);print("frames:",len(saved))
if __name__=="__main__":main()
