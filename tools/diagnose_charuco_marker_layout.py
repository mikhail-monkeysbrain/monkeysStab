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
# Board marker centers in the board XY plane, keyed by marker ID.
try: obj=np.asarray(board.getObjPoints(),dtype=np.float32)
except Exception: obj=np.asarray(board.objPoints,dtype=np.float32)
board_center={int(i):np.asarray(q,dtype=np.float32).reshape(-1,3)[:,:2].mean(0) for i,q in zip(expected,obj)}
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
  common=[i for i in found if i in board_center]
  if len(common)>=4:
   src=np.asarray([board_center[i] for i in common],np.float32)
   dst=np.asarray([np.asarray(corners[found.index(i)]).reshape(-1,2).mean(0) for i in common],np.float32)
   H,mask=cv2.findHomography(src,dst,cv2.RANSAC,3.0)
   if H is not None:
    pred=cv2.perspectiveTransform(src.reshape(-1,1,2),H).reshape(-1,2)
    er=np.linalg.norm(pred-dst,axis=1)
    print("===== BOARD-ID HOMOGRAPHY =====")
    print("matched markers:",len(common),"RANSAC inliers:",int(mask.sum()) if mask is not None else -1)
    print("center reprojection px: median=%.3f max=%.3f RMS=%.3f"%(np.median(er),np.max(er),np.sqrt(np.mean(er*er))))
    for i,e in sorted(zip(common,er),key=lambda z:z[1],reverse=True):print("  ID %2d  err %.3f px"%(i,e))
    if mask is not None and int(mask.sum())>=max(4,len(common)-1) and np.median(er)<2.0:
     print("LAYOUT VERDICT: CONSISTENT with OpenCV board")
    else: print("LAYOUT VERDICT: NOT CONSISTENT / needs inspection")
   else: print("BOARD-ID HOMOGRAPHY: failed")
  else: print("BOARD-ID HOMOGRAPHY: need >=4 expected markers")
 if k in (27,ord("q"),ord("Q")):break
cap.release();cv2.destroyAllWindows()
