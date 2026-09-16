#!/usr/bin/env python3
"""Drop-worst trajectory для уже снятой независимой ChArUco серии.
На каждом шаге: калибровка -> per-view RMS -> удалить текущий худший view -> повторить.
Никакого GT fitting.
"""
import sys,glob,cv2,numpy as np
from pathlib import Path
if len(sys.argv)!=2:raise SystemExit("usage: analyze_charuco_drop_worst.py /path/to/series")
root=Path(sys.argv[1]);files=sorted(glob.glob(str(root/"frame_*.jpg")))
S=.027315;M=.020031;dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try:board=cv2.aruco.CharucoBoard((7,5),S,M,dic)
except TypeError:board=cv2.aruco.CharucoBoard_create(7,5,S,M,dic)
objall=np.asarray(board.getChessboardCorners(),np.float32);obs=[];shape=None
for fn in files:
 im=cv2.imread(fn);shape=(im.shape[1],im.shape[0]);g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);mc,mi,_=cv2.aruco.detectMarkers(g,dic)
 if mi is None or len(mi)<2:continue
 _,cc,ci=cv2.aruco.interpolateCornersCharuco(mc,mi,g,board)
 if ci is None or len(ci)<4:continue
 ids=ci.reshape(-1).astype(int);p=cc.reshape(-1,2)
 obs.append({"fn":Path(fn).name,"o":objall[ids].reshape(-1,1,3),"p":cc.astype(np.float32),"cx":p[:,0].mean()/shape[0],"cy":p[:,1].mean()/shape[1]})
def cal(a):
 r,K,D,rv,tv=cv2.calibrateCamera([x["o"] for x in a],[x["p"] for x in a],shape,None,None)
 er=[]
 for x,R,t in zip(a,rv,tv):
  q,_=cv2.projectPoints(x["o"],R,t,K,D);d=np.linalg.norm(q.reshape(-1,2)-x["p"].reshape(-1,2),axis=1);er.append(float(np.sqrt(np.mean(d*d))))
 return r,K,D,er
a=obs.copy();print("===== ITERATIVE DROP-WORST FREE_5D =====")
print("n  RMS     fx      fy      cx      cy      worst             worstRMS")
while len(a)>=20:
 r,K,D,e=cal(a);j=int(np.argmax(e))
 print(f"{len(a):2d} {r:7.4f} {K[0,0]:7.2f} {K[1,1]:7.2f} {K[0,2]:7.2f} {K[1,2]:7.2f} {a[j]['fn']:17s} {e[j]:8.4f}")
 a.pop(j)
print("\n===== COVERAGE ORIGINAL =====")
xs=np.array([x["cx"] for x in obs]);ys=np.array([x["cy"] for x in obs])
print(f"view-center x norm min/median/max: {xs.min():.3f} {np.median(xs):.3f} {xs.max():.3f}")
print(f"view-center y norm min/median/max: {ys.min():.3f} {np.median(ys):.3f} {ys.max():.3f}")
