#!/usr/bin/env python3
"""Сбалансированный сбор ChArUco SERIES_C по 3x3 зонам изображения.
Цель: устранить пространственную необусловленность principal point/focal.
В каждой зоне сохраняются несколько разнообразных наблюдений автоматически.
"""
import argparse,time,csv
from pathlib import Path
import cv2,numpy as np
S=.027315; M=.020031
dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try: board=cv2.aruco.CharucoBoard((7,5),S,M,dic)
except TypeError: board=cv2.aruco.CharucoBoard_create(7,5,S,M,dic)
def detect(g):
 mc,mi,_=cv2.aruco.detectMarkers(g,dic)
 if mi is None or len(mi)<2:return mc,mi,None,None
 _,cc,ci=cv2.aruco.interpolateCornersCharuco(mc,mi,g,board);return mc,mi,cc,ci
def descriptor(cc):
 p=cc.reshape(-1,2); x0,y0=p.min(0); x1,y1=p.max(0)
 return np.array([(x1-x0)/640.,(y1-y0)/480.,np.std(p[:,0])/640.,np.std(p[:,1])/480.])
def main():
 ap=argparse.ArgumentParser();ap.add_argument("--camera",default="/dev/video0");ap.add_argument("--per-zone",type=int,default=4);ap.add_argument("--top-mask",type=int,default=190,help="ignore stand ring area above this y pixel");a=ap.parse_args()
 out=Path(f"/home/vio/charuco_series_C_{time.strftime('%Y%m%d_%H%M%S')}");out.mkdir(parents=True)
 cap=cv2.VideoCapture(a.camera,cv2.CAP_V4L2);cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"));cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480);cap.set(cv2.CAP_PROP_FPS,120)
 if not cap.isOpened():raise SystemExit("camera open failed")
 counts=np.zeros((3,3),int); descs=[[[] for _ in range(3)] for _ in range(3)];meta=[];last=0.;total=6*a.per_zone
 print(f"SERIES_C stand-aware: top y<{a.top_mask} is physically occluded by stand ring; target middle+bottom 3x2, {a.per_zone}/zone = {total}. AUTO capture. Q/ESC stop.")
 while counts.sum()<total:
  ok,im=cap.read()
  if not ok:continue
  g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);mc,mi,cc,ci=detect(g);nm=0 if mi is None else len(mi);nc=0 if ci is None else len(ci)
  zone=None;good=False;reason="need >=8 corners";cx=cy=0.;d=None;nov=0.
  if cc is not None and nc>=8:
   p=cc.reshape(-1,2);cx,cy=p.mean(0);zx=min(2,max(0,int(cx/(640/3))));zy=min(2,max(0,int(cy/(480/3))));zone=(zy,zx);d=descriptor(cc)
   if cy<a.top_mask: reason="stand-ring masked area"\n   elif counts[zy,zx]>=a.per_zone:reason="zone full"
   else:
    old=descs[zy][zx]
    nov=999. if not old else min(float(np.linalg.norm((d-o)*np.array([2.,2.,1.,1.]))) for o in old)
    if nov>=.035:good=True;reason="AUTO READY"
    else:reason=f"same geometry {nov:.3f}"
  vis=im.copy()
  if mi is not None:cv2.aruco.drawDetectedMarkers(vis,mc,mi)
  if cc is not None:cv2.aruco.drawDetectedCornersCharuco(vis,cc,ci)
  for x in (213,426):cv2.line(vis,(x,0),(x,479),(255,255,255),1)
  cv2.rectangle(vis,(0,0),(639,a.top_mask),(80,80,80),2)\n  cv2.putText(vis,"MASK: STAND RING / NOT CALIBRATION COVERAGE",(8,145),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),2)\n  cv2.line(vis,(0,320),(639,320),(255,255,255),1)
  for yy in range(1,3):
   for xx in range(3):
    cv2.putText(vis,f"{counts[yy,xx]}/{a.per_zone}",(xx*213+8,yy*160+22),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,0) if counts[yy,xx]>=a.per_zone else (255,255,255),2)
  cv2.putText(vis,f"TOTAL {counts.sum()}/{total} corners {nc} {reason}",(8,470),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,255,0) if good else (0,0,255),2)
  cv2.imshow("ChArUco SERIES_C BALANCED 3x3",vis);k=cv2.waitKey(1)&255
  now=time.time()
  if good and zone is not None and zone[0]>0 and now-last>.65:
   zy,zx=zone;n=int(counts.sum())+1;p=out/f"frame_{n:03d}.jpg";cv2.imwrite(str(p),im);counts[zy,zx]+=1;descs[zy][zx].append(d);meta.append([p.name,zy,zx,nm,nc,cx/640.,cy/480.,*d,nov]);last=now
   print(f"SAVED {p.name} zone={zy},{zx} zone_count={counts[zy,zx]}/{a.per_zone} corners={nc} novelty={nov:.3f}")
  if k in (27,ord('q'),ord('Q')):break
 cap.release();cv2.destroyAllWindows()
 with (out/"capture.csv").open("w",newline="") as f:
  w=csv.writer(f);w.writerow(["file","zone_y","zone_x","markers","corners","cx_norm","cy_norm","width_norm","height_norm","spread_x","spread_y","novelty"]);w.writerows(meta)
 print("Saved directory:",out);print("accepted:",int(counts.sum()),"/",total);print("zone counts:");print(counts)
if __name__=="__main__":main()
