#!/usr/bin/env python3
"""Forensic устойчивости ChArUco-калибровки по уже сохранённым кадрам."""
import argparse, csv
from pathlib import Path
import cv2, numpy as np
SQUARE=.027315; MARKER=.020031; SX,SY=7,5; DICT=cv2.aruco.DICT_4X4_50
def api():
 d=cv2.aruco.getPredefinedDictionary(DICT)
 try:b=cv2.aruco.CharucoBoard((SX,SY),SQUARE,MARKER,d)
 except TypeError:b=cv2.aruco.CharucoBoard_create(SX,SY,SQUARE,MARKER,d)
 return d,b
def det(g,d,b):
 c,i,_=cv2.aruco.detectMarkers(g,d)
 if i is None or len(i)<2:return None,None
 _,cc,ci=cv2.aruco.interpolateCornersCharuco(c,i,g,b);return cc,ci
def cal(C,I,b,shape,flags=0,K=None,D=None):
 return cv2.aruco.calibrateCameraCharuco(C,I,b,shape,K,D,flags=flags)
def stats(name,C,I,b,shape,flags=0,K=None,D=None):
 try:
  r,k,d,rv,tv=cal(C,I,b,shape,flags,K,D)
  return dict(name=name,n=len(C),rms=r,fx=k[0,0],fy=k[1,1],cx=k[0,2],cy=k[1,2],k1=d.ravel()[0],k2=d.ravel()[1],p1=d.ravel()[2],p2=d.ravel()[3],k3=d.ravel()[4] if d.size>4 else 0)
 except cv2.error:return None
def main():
 ap=argparse.ArgumentParser();ap.add_argument("dir");a=ap.parse_args();root=Path(a.dir);d,b=api()
 C=[];I=[];names=[];shape=None
 for p in sorted(root.glob("frame_*.jpg")):
  im=cv2.imread(str(p));g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);shape=g.shape[::-1];cc,ci=det(g,d,b)
  if ci is not None and len(ci)>=8:C.append(cc);I.append(ci);names.append(p.name)
 print("===== INPUT =====");print("frames:",len(C),"shape:",shape,"corners min/median/max:",min(map(len,I)),np.median(list(map(len,I))),max(map(len,I)))
 models=[]
 models.append(stats("FREE_5D",C,I,b,shape))
 models.append(stats("ZERO_TANGENT",C,I,b,shape,cv2.CALIB_ZERO_TANGENT_DIST))
 models.append(stats("FIX_K3",C,I,b,shape,cv2.CALIB_FIX_K3))
 models.append(stats("ZERO_TANGENT_FIX_K3",C,I,b,shape,cv2.CALIB_ZERO_TANGENT_DIST|cv2.CALIB_FIX_K3))
 print("\n===== DISTORTION MODELS =====")
 for x in models:
  if x:print(f'{x["name"]:20s} RMS={x["rms"]:.4f} fx={x["fx"]:.2f} fy={x["fy"]:.2f} cx={x["cx"]:.2f} cy={x["cy"]:.2f} D=[{x["k1"]:.3g},{x["k2"]:.3g},{x["p1"]:.3g},{x["p2"]:.3g},{x["k3"]:.3g}]')
 base=models[0]; K=np.array([[base["fx"],0,base["cx"]],[0,base["fy"],base["cy"]],[0,0,1.]],float);D=np.array([base["k1"],base["k2"],base["p1"],base["p2"],base["k3"]],float)
 # Per-view reprojection using base model.
 r,k,dd,rv,tv=cal(C,I,b,shape)
 errs=[]
 obj=np.asarray(b.getChessboardCorners() if hasattr(b,"getChessboardCorners") else b.chessboardCorners)
 for n,cc,ci,rvec,tvec in zip(names,C,I,rv,tv):
  ids=ci.reshape(-1).astype(int); op=obj[ids].reshape(-1,1,3);pr,_=cv2.projectPoints(op,rvec,tvec,k,dd)
  e=float(np.sqrt(np.mean(np.sum((pr.reshape(-1,2)-cc.reshape(-1,2))**2,axis=1))));errs.append((e,n))
 print("\n===== WORST VIEWS (FREE_5D) =====")
 for e,n in sorted(errs,reverse=True)[:10]:print(f"{n}: {e:.4f} px")
 # Leave-one-out.
 loo=[]
 for j in range(len(C)):
  x=stats(names[j],[v for i,v in enumerate(C) if i!=j],[v for i,v in enumerate(I) if i!=j],b,shape)
  if x:loo.append(x)
 print("\n===== LEAVE-ONE-OUT FREE_5D =====")
 for key in ("fx","fy","cx","cy","rms"):
  v=np.array([x[key] for x in loo]);print(f"{key}: min={v.min():.3f} median={np.median(v):.3f} max={v.max():.3f} std={v.std():.3f}")
 # Progressive rejection of worst base reprojection views, fixed ranking.
 print("\n===== DROP WORST VIEWS =====")
 bad=[n for _,n in sorted(errs,reverse=True)]
 for drop in (0,2,4,6,8,10):
  keep=[i for i,n in enumerate(names) if n not in set(bad[:drop])]
  if len(keep)<10:continue
  x=stats(f"drop{drop}",[C[i] for i in keep],[I[i] for i in keep],b,shape)
  print(f'drop={drop:2d} n={x["n"]:2d} RMS={x["rms"]:.4f} fx={x["fx"]:.2f} fy={x["fy"]:.2f} cx={x["cx"]:.2f} cy={x["cy"]:.2f}')
 out=root/"charuco_forensic.csv"
 with out.open("w",newline="") as f:
  w=csv.DictWriter(f,fieldnames=["excluded","fx","fy","cx","cy","rms"]);w.writeheader()
  for x in loo:w.writerow(dict(excluded=x["name"],fx=x["fx"],fy=x["fy"],cx=x["cx"],cy=x["cy"],rms=x["rms"]))
 print("\nSaved:",out)
if __name__=="__main__":main()
