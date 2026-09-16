#!/usr/bin/env python3
"""Диагностика соответствия физической ChArUco ожидаемому OpenCV board.
Ничего не калибрует и не меняет.
"""
import cv2, numpy as np
SX,SY=7,5; S=.027315; M=.020031
dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try: board=cv2.aruco.CharucoBoard((SX,SY),S,M,dic)
except TypeError: board=cv2.aruco.CharucoBoard_create(SX,SY,S,M,dic)
try:
    expected=np.asarray(board.getIds()).reshape(-1).astype(int)
except Exception:
    expected=np.asarray(board.ids).reshape(-1).astype(int)
print("Expected board marker IDs:",expected.tolist())
cap=cv2.VideoCapture("/dev/video0",cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"));cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480);cap.set(cv2.CAP_PROP_FPS,120)
if not cap.isOpened():raise SystemExit("Не удалось открыть /dev/video0")
while True:
 ok,im=cap.read()
 if not ok:continue
 g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);corners,ids,_=cv2.aruco.detectMarkers(g,dic)
 vis=im.copy(); found=[]
 if ids is not None:
  cv2.aruco.drawDetectedMarkers(vis,corners,ids);found=ids.reshape(-1).astype(int).tolist()
  pts=[]
  for c,i in zip(corners,found):
   x,y=np.asarray(c).reshape(-1,2).mean(0);pts.append((y,x,i))
  pts.sort()
  # group approximately by image rows for terminal readability
  rows=[]
  for y,x,i in pts:
   if not rows or abs(y-np.mean([q[0] for q in rows[-1]]))>35: rows.append([])
   rows[-1].append((y,x,i))
  layout=[" ".join(str(q[2]) for q in sorted(r,key=lambda z:z[1])) for r in rows]
 else:layout=[]
 cc=ci=None
 if ids is not None and len(ids)>=2:
  _,cc,ci=cv2.aruco.interpolateCornersCharuco(corners,ids,g,board)
 n=0 if ci is None else len(ci)
 exp=set(expected.tolist()); bad=sorted(set(found)-exp); missing=sorted(exp-set(found))
 cv2.putText(vis,f"markers {len(found)} charuco {n}/24 unexpected {bad}",(8,24),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,0,255) if bad else (0,255,0),2)
 cv2.imshow("ChArUco marker-layout diagnostic",vis);k=cv2.waitKey(1)&255
 if k==32:
  print("\n===== SNAPSHOT =====");print("Detected IDs:",sorted(found));print("Approx image rows:")
  for r in layout:print("  ",r)
  print("Unexpected IDs vs expected board:",bad);print("Expected but not visible:",missing);print("Charuco corners:",n)
 if k in (27,ord("q"),ord("Q")):break
cap.release();cv2.destroyAllWindows()
