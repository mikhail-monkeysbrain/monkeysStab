#!/usr/bin/env python3
"""Live forensic резкости OV9281 по области ChArUco.
Не калибрует, не сохраняет кадры и не меняет controls камеры.
SPACE печатает статистику последних 60 кадров.
"""
import cv2, numpy as np
from collections import deque
dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try: board=cv2.aruco.CharucoBoard((7,5),.027315,.020031,dic)
except TypeError: board=cv2.aruco.CharucoBoard_create(7,5,.027315,.020031,dic)
hist=deque(maxlen=60)
def focus(a):
 if a.size<100:return float("nan")
 return float(cv2.Laplacian(a,cv2.CV_64F).var())
def contrast(a):
 if a.size<100:return float("nan")
 return float(np.percentile(a,95)-np.percentile(a,5))
cap=cv2.VideoCapture("/dev/video0",cv2.CAP_V4L2);cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"));cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480);cap.set(cv2.CAP_PROP_FPS,120)
if not cap.isOpened():raise SystemExit("Не удалось открыть /dev/video0")
print("Доска неподвижно и почти фронтально. SPACE = статистика 60 кадров, Q = выход.")
while True:
 ok,im=cap.read()
 if not ok:continue
 g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);mc,mi,_=cv2.aruco.detectMarkers(g,dic)
 nm=0 if mi is None else len(mi);cc=ci=None
 if mi is not None and len(mi)>=2:_,cc,ci=cv2.aruco.interpolateCornersCharuco(mc,mi,g,board)
 nc=0 if ci is None else len(ci)
 # ROI по всем углам обнаруженных marker-квадратов, с небольшим запасом.
 if mc:
  p=np.concatenate([np.asarray(x).reshape(-1,2) for x in mc]);x0,y0=np.floor(p.min(0)-10).astype(int);x1,y1=np.ceil(p.max(0)+10).astype(int)
  x0=max(0,x0);y0=max(0,y0);x1=min(g.shape[1],x1);y1=min(g.shape[0],y1)
 else:x0,y0,x1,y1=0,0,g.shape[1],g.shape[0]
 roi=g[y0:y1,x0:x1];h,w=roi.shape
 # 3x3 zones inside detected-board envelope.
 vals=[]
 for yy in range(3):
  for xx in range(3):
   z=roi[yy*h//3:(yy+1)*h//3,xx*w//3:(xx+1)*w//3];vals.append(focus(z))
 F=focus(roi);C=contrast(roi);hist.append((nm,nc,F,C,*vals))
 vis=im.copy();cv2.rectangle(vis,(x0,y0),(x1,y1),(255,255,255),1)
 cv2.putText(vis,f"markers {nm}/17 corners {nc}/24 focus {F:.0f} contrast {C:.0f}",(8,24),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),2)
 labels=["TL","TC","TR","ML","MC","MR","BL","BC","BR"]
 for j,(lab,v) in enumerate(zip(labels,vals)):
  yy=j//3;xx=j%3;px=x0+xx*max(1,w)//3+4;py=y0+yy*max(1,h)//3+18
  cv2.putText(vis,f"{lab}:{v:.0f}",(px,py),cv2.FONT_HERSHEY_SIMPLEX,.38,(255,255,255),1)
 cv2.imshow("OV9281 ChArUco sharpness forensic",vis);k=cv2.waitKey(1)&255
 if k==32 and hist:
  a=np.asarray(hist,float);print("\n===== SHARPNESS SNAPSHOT (last %d) ====="%len(a))
  print("markers median/max: %.1f/%d"%(np.median(a[:,0]),int(a[:,0].max())))
  print("corners median/max: %.1f/%d"%(np.median(a[:,1]),int(a[:,1].max())))
  print("board focus median: %.1f  contrast median: %.1f"%(np.median(a[:,2]),np.median(a[:,3])))
  print("zone focus medians:")
  for r in range(3):print("  "+"  ".join("%s=%7.1f"%(labels[r*3+c],np.median(a[:,4+r*3+c])) for c in range(3)))
  z=np.array([np.median(a[:,4+i]) for i in range(9)]);print("zone max/min ratio: %.3f"%(np.nanmax(z)/max(np.nanmin(z),1e-9)))
 if k in (27,ord("q"),ord("Q")):break
cap.release();cv2.destroyAllWindows()
