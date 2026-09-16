#!/usr/bin/env python3
"""Forensic-калибровка независимой ChArUco серии B/C.
Физические размеры зафиксированы до расчёта: square=27.315 mm, marker=20.031 mm.
Считает несколько distortion-моделей, per-view errors и leave-one-out.
"""
import sys,glob,cv2,numpy as np
from pathlib import Path
if len(sys.argv)!=2: raise SystemExit("usage: analyze_charuco_independent_series.py /path/to/series")
root=Path(sys.argv[1]); files=sorted(glob.glob(str(root/"frame_*.jpg")))
if not files: raise SystemExit("no frame_*.jpg")
S=.027315;M=.020031
dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try: board=cv2.aruco.CharucoBoard((7,5),S,M,dic)
except TypeError: board=cv2.aruco.CharucoBoard_create(7,5,S,M,dic)
obj_all=np.asarray(board.getChessboardCorners(),np.float32)
obs=[]
for fn in files:
 im=cv2.imread(fn);g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);mc,mi,_=cv2.aruco.detectMarkers(g,dic)
 if mi is None or len(mi)<2:continue
 _,cc,ci=cv2.aruco.interpolateCornersCharuco(mc,mi,g,board)
 if ci is None or len(ci)<4:continue
 ids=ci.reshape(-1).astype(int);obs.append((fn,obj_all[ids].reshape(-1,1,3),cc.astype(np.float32)))
shape=(im.shape[1],im.shape[0])
print("===== INPUT =====");print("directory:",root);print("images:",len(files),"usable:",len(obs),"shape:",shape)
print("corners min/median/max:",min(len(x[2]) for x in obs),np.median([len(x[2]) for x in obs]),max(len(x[2]) for x in obs))
def cal(items,flags):
 obj=[x[1] for x in items];img=[x[2] for x in items]
 return cv2.calibrateCamera(obj,img,shape,None,None,flags=flags)
models=[("FREE_5D",0),("ZERO_TANGENT",cv2.CALIB_ZERO_TANGENT_DIST),("FIX_K3",cv2.CALIB_FIX_K3),("ZERO_TANGENT_FIX_K3",cv2.CALIB_ZERO_TANGENT_DIST|cv2.CALIB_FIX_K3)]
results={}
print("\n===== DISTORTION MODELS =====")
for name,flags in models:
 rms,K,D,rv,tv=cal(obs,flags);results[name]=(rms,K,D,rv,tv,flags)
 print(f"{name:20s} RMS={rms:.4f} fx={K[0,0]:.3f} fy={K[1,1]:.3f} cx={K[0,2]:.3f} cy={K[1,2]:.3f} D={np.array2string(D.ravel(),precision=4)}")
rms,K,D,rv,tv,flags=results["FREE_5D"]
errs=[]
for (fn,o,p),r,t in zip(obs,rv,tv):
 q,_=cv2.projectPoints(o,r,t,K,D);e=np.linalg.norm(q.reshape(-1,2)-p.reshape(-1,2),axis=1)
 errs.append((Path(fn).name,float(np.sqrt(np.mean(e*e))),len(e)))
print("\n===== WORST VIEWS FREE_5D =====")
for x in sorted(errs,key=lambda z:z[1],reverse=True)[:10]:print(f"{x[0]} RMS={x[1]:.4f}px corners={x[2]}")
print("\n===== LEAVE-ONE-OUT FREE_5D =====")
loo=[]
for i in range(len(obs)):
 try:
  rr,kk,dd,_,_=cal(obs[:i]+obs[i+1:],0);loo.append([kk[0,0],kk[1,1],kk[0,2],kk[1,2],rr])
 except cv2.error:pass
a=np.asarray(loo)
for j,n in enumerate(["fx","fy","cx","cy","rms"]):print(f"{n}: min={a[:,j].min():.3f} median={np.median(a[:,j]):.3f} max={a[:,j].max():.3f} std={a[:,j].std():.3f}")
print("\n===== REFERENCE RATIOS (report only; NOT fitted) =====")
print("FREE_5D fx/568.5317075 =",K[0,0]/568.5317075)
print("FREE_5D fy/569.6800556 =",K[1,1]/569.6800556)
print("FREE_5D fx/621.12 =",K[0,0]/621.12)
print("FREE_5D fx/628.57 =",K[0,0]/628.57)
