#!/usr/bin/env python3
"""Независимая калибровка OV9281 по ChArUco 7x5.
Физическая распечатка: square=27.315 mm, marker=20.031 mm.
Не использует Ground Motion, MOVE340/500 или focal_scale.
"""
import argparse, time
from pathlib import Path
import cv2, numpy as np

SQUARE=0.027315
MARKER=0.020031
SX,SY=7,5
DICT=cv2.aruco.DICT_4X4_50

def board_api():
    dic=cv2.aruco.getPredefinedDictionary(DICT)
    try: board=cv2.aruco.CharucoBoard((SX,SY),SQUARE,MARKER,dic)
    except TypeError: board=cv2.aruco.CharucoBoard_create(SX,SY,SQUARE,MARKER,dic)
    return dic,board

def detect(gray,dic,board):
    corners,ids,_=cv2.aruco.detectMarkers(gray,dic)
    if ids is None or len(ids)<2:return corners,ids,None,None
    _,cc,ci=cv2.aruco.interpolateCornersCharuco(corners,ids,gray,board)
    return corners,ids,cc,ci

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--camera",default="/dev/video0")
    ap.add_argument("--width",type=int,default=640);ap.add_argument("--height",type=int,default=480)
    ap.add_argument("--fps",type=int,default=120);ap.add_argument("--count",type=int,default=30)
    ap.add_argument("--out",default="")
    a=ap.parse_args()
    dic,board=board_api(); cap=cv2.VideoCapture(a.camera,cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"));cap.set(cv2.CAP_PROP_FRAME_WIDTH,a.width);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,a.height);cap.set(cv2.CAP_PROP_FPS,a.fps)
    if not cap.isOpened():raise SystemExit("Не удалось открыть камеру "+a.camera)
    out=Path(a.out or f"/home/vio/charuco_calib_{time.strftime('%Y%m%d_%H%M%S')}");out.mkdir(parents=True,exist_ok=True)
    saved=[]; print("ПРОБЕЛ — сохранить хороший кадр; Q/ESC — завершить и калибровать.")
    print("Нужно 20-30 кадров: центр, края, углы, разные наклоны и расстояния.")
    while True:
      ok,im=cap.read()
      if not ok:continue
      g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);mc,mi,cc,ci=detect(g,dic,board)
      vis=im.copy()
      if mi is not None:cv2.aruco.drawDetectedMarkers(vis,mc,mi)
      n=0 if ci is None else len(ci)
      if ci is not None:cv2.aruco.drawDetectedCornersCharuco(vis,cc,ci)
      cv2.putText(vis,f"saved {len(saved)}/{a.count} corners {n}",(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,0),2)
      cv2.imshow("OV9281 ChArUco calibration",vis);k=cv2.waitKey(1)&255
      if k==32 and n>=8:
        p=out/f"frame_{len(saved)+1:03d}.jpg";cv2.imwrite(str(p),im);saved.append(p);print("saved",p.name,"corners",n)
      if k in (27,ord('q'),ord('Q')) or len(saved)>=a.count:break
    cap.release();cv2.destroyAllWindows()
    if len(saved)<10:raise SystemExit(f"Недостаточно кадров: {len(saved)}")
    allc=[];alli=[];shape=None;used=[]
    for p in saved:
      im=cv2.imread(str(p));g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);shape=g.shape[::-1];_,_,cc,ci=detect(g,dic,board)
      if ci is not None and len(ci)>=8:allc.append(cc);alli.append(ci);used.append(p)
    flags=0
    ret,K,D,rvecs,tvecs=cv2.aruco.calibrateCameraCharuco(allc,alli,board,shape,None,None,flags=flags)
    print("\n===== RESULT =====");print("frames:",len(used),"RMS:",ret);print("K:\n",K);print("D:",D.ravel())
    print("fx/fy:",K[0,0],K[1,1]);print("cx/cy:",K[0,2],K[1,2]);print("fx/568.5317075:",K[0,0]/568.53170752165227);print("fy/569.6800556:",K[1,1]/569.68005562865858)
    fs=cv2.FileStorage(str(out/"ov9281_charuco_calibration.yaml"),cv2.FILE_STORAGE_WRITE)
    fs.write("image_width",shape[0]);fs.write("image_height",shape[1]);fs.write("square_length_m",SQUARE);fs.write("marker_length_m",MARKER);fs.write("camera_matrix",K);fs.write("distortion_coefficients",D);fs.write("rms",ret);fs.release()
    print("Saved:",out/"ov9281_charuco_calibration.yaml")
if __name__=="__main__":main()
