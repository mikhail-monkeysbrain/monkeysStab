#!/usr/bin/env python3
"""Сравнение legacy interpolateCornersCharuco и CharucoDetector OpenCV 4.10."""
import cv2, numpy as np
S=.027315; M=.020031; SX,SY=7,5
dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try: board=cv2.aruco.CharucoBoard((SX,SY),S,M,dic)
except TypeError: board=cv2.aruco.CharucoBoard_create(SX,SY,S,M,dic)
def legacy(g,refine):
 p=cv2.aruco.DetectorParameters()
 p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX if refine else cv2.aruco.CORNER_REFINE_NONE
 det=cv2.aruco.ArucoDetector(dic,p) if hasattr(cv2.aruco,"ArucoDetector") else None
 if det: mc,mi,_=det.detectMarkers(g)
 else: mc,mi,_=cv2.aruco.detectMarkers(g,dic,parameters=p)
 if mi is None or len(mi)<2:return mc,mi,None,None
 _,cc,ci=cv2.aruco.interpolateCornersCharuco(mc,mi,g,board)
 return mc,mi,cc,ci
def modern(g):
 if not hasattr(cv2.aruco,"CharucoDetector"):return None,None,None,None
 try:
  cp=cv2.aruco.CharucoParameters(); ap=cv2.aruco.DetectorParameters(); ap.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
  cd=cv2.aruco.CharucoDetector(board,cp,ap)
  cc,ci,mc,mi=cd.detectBoard(g);return mc,mi,cc,ci
 except Exception as e:
  print("CharucoDetector error:",e);return None,None,None,None
def ids(ci):return [] if ci is None else sorted(np.asarray(ci).reshape(-1).astype(int).tolist())
cap=cv2.VideoCapture("/dev/video0",cv2.CAP_V4L2);cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"));cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480);cap.set(cv2.CAP_PROP_FPS,120)
if not cap.isOpened():raise SystemExit("Не удалось открыть /dev/video0")
while True:
 ok,im=cap.read()
 if not ok:continue
 g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
 a=legacy(g,False);b=legacy(g,True);c=modern(g)
 na=0 if a[3] is None else len(a[3]);nb=0 if b[3] is None else len(b[3]);nc=0 if c[3] is None else len(c[3])
 vis=im.copy()
 if c[1] is not None:cv2.aruco.drawDetectedMarkers(vis,c[0],c[1])
 if c[2] is not None:cv2.aruco.drawDetectedCornersCharuco(vis,c[2],c[3])
 cv2.putText(vis,f"legacy none {na}/24  subpix {nb}/24  modern {nc}/24",(8,25),cv2.FONT_HERSHEY_SIMPLEX,.52,(0,255,0),2)
 cv2.imshow("ChArUco detector A/B",vis);k=cv2.waitKey(1)&255
 if k==32:
  print("\n===== DETECTOR A/B SNAPSHOT =====")
  print("OpenCV:",cv2.__version__)
  print("legacy NONE  :",na,"IDs",ids(a[3]))
  print("legacy SUBPIX:",nb,"IDs",ids(b[3]))
  print("modern       :",nc,"IDs",ids(c[3]))
  if ids(a[3])!=ids(c[3]):print("legacy vs modern ID difference:",sorted(set(ids(a[3]))^set(ids(c[3]))))
 if k in (27,ord("q"),ord("Q")):break
cap.release();cv2.destroyAllWindows()
