#!/usr/bin/env python3
"""Автоматический sweep экспозиции OV9281 по качеству ChArUco detection.
Для каждого значения ждёт стабилизацию, затем измеряет серию кадров.
Ничего не сохраняет и не калибрует.
"""
import cv2,time,subprocess,numpy as np
EXPOSURES=[10,15,20,25,35,50]
N=40; WARMUP=12
dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try: board=cv2.aruco.CharucoBoard((7,5),.027315,.020031,dic)
except TypeError: board=cv2.aruco.CharucoBoard_create(7,5,.027315,.020031,dic)
def setexp(v):
 subprocess.run(["v4l2-ctl","-d","/dev/video0","--set-ctrl=auto_exposure=1,exposure_time_absolute=%d,gain=0,backlight_compensation=0"%v],check=True,stdout=subprocess.DEVNULL)
def detect(g):
 mc,mi,_=cv2.aruco.detectMarkers(g,dic); nm=0 if mi is None else len(mi); nc=0
 if mi is not None and len(mi)>=2:
  _,cc,ci=cv2.aruco.interpolateCornersCharuco(mc,mi,g,board);nc=0 if ci is None else len(ci)
 return nm,nc
cap=cv2.VideoCapture("/dev/video0",cv2.CAP_V4L2);cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"));cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480);cap.set(cv2.CAP_PROP_FPS,120)
if not cap.isOpened():raise SystemExit("Не удалось открыть /dev/video0")
print("Держи доску НЕПОДВИЖНО, почти фронтально, целиком в кадре.")
print("Sweep:",EXPOSURES,"frames/value:",N)
rows=[]
for e in EXPOSURES:
 setexp(e)
 for _ in range(WARMUP):cap.read()
 ms=[];cs=[];means=[];clips=[]
 for _ in range(N):
  ok,im=cap.read()
  if not ok:continue
  g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);m,c=detect(g);ms.append(m);cs.append(c);means.append(float(g.mean()));clips.append(float(np.mean(g>=250))*100)
 rows.append((e,ms,cs,means,clips))
 print(f"exp={e:3d} markers med/max={np.median(ms):4.1f}/{max(ms):2d} charuco med/max={np.median(cs):4.1f}/{max(cs):2d} gray={np.median(means):6.1f} clip>=250={np.median(clips):5.2f}%")
cap.release()
print("\n===== EXPOSURE SWEEP SUMMARY =====")
print("exp  marker_med marker_max corner_med corner_max gray_med clip250_med")
for e,ms,cs,means,clips in rows:print(f"{e:3d} {np.median(ms):10.1f} {max(ms):10d} {np.median(cs):10.1f} {max(cs):10d} {np.median(means):8.1f} {np.median(clips):11.2f}%")
best=max(rows,key=lambda r:(np.median(r[2]),max(r[2]),np.median(r[1]),-np.median(r[4])))
print("\nBest by detection (not calibration): exposure",best[0])
