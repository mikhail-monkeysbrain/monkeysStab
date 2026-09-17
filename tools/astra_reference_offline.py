#!/usr/bin/env python3
"""Offline reference port of Astra raw-plane-v1 for monkeysStab.

Purpose: reproduce the original Astra Python estimator on the original RAW dataset
before any C++/realtime port. GT is never used by the estimator.
"""
import argparse, csv, math, struct, time
from pathlib import Path

import cv2
import numpy as np

cv2.setNumThreads(1)
cv2.setRNGSeed(716)

K0=np.array([[568.5317075216523,0,315.98271077441063],
             [0,569.6800556286586,239.8814858910064],[0,0,1.]],dtype=np.float64)
DIST=np.array([.07356919219402849,-.095253893789117,-.0108105307571873,
               -.002284337357697,.08217740080275748],dtype=np.float64)
BC=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]],dtype=np.float64)
CAM=np.array([.0625,0.,.05],dtype=np.float64)
LUNA=np.array([.0855,0.,.055],dtype=np.float64)
ROI=(.10,.44,.90,.96)


def rotation(roll,pitch,yaw):
    cr,sr=np.cos(roll),np.sin(roll); cp,sp=np.cos(pitch),np.sin(pitch); cy,sy=np.cos(yaw),np.sin(yaw)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                     [-sp,cp*sr,cp*cr]],dtype=np.float64)


def roi_mask(im):
    h,w=im.shape; m=np.zeros_like(im); x0,y0,x1,y1=ROI
    m[int(h*y0):int(h*y1),int(w*x0):int(w*x1)]=255
    return m


def features(im):
    m=roi_mask(im); out=[]
    for y in range(3):
        for x in range(4):
            cell=np.zeros_like(im)
            cell[y*160:(y+1)*160,x*160:(x+1)*160]=m[y*160:(y+1)*160,x*160:(x+1)*160]
            p=cv2.goodFeaturesToTrack(im,36,.01,7,mask=cell)
            if p is not None: out.extend(p[:,0])
    return np.asarray(out,np.float32).reshape(-1,1,2)


def fit(a,b):
    info={'n':len(a),'inliers':0,'residual':999.,'cells':0,'ok':False}
    if len(a)<8: return None,None,info
    H,m=cv2.findHomography(a,b,cv2.RANSAC,1.,maxIters=2000,confidence=.999)
    if H is None: return None,None,info
    m=m.ravel().astype(bool); pred=cv2.perspectiveTransform(a.reshape(-1,1,2),H)[:,0]
    e=np.linalg.norm(pred-b,axis=1); n=int(m.sum())
    cells=len(set((int(x//160),int(y//80)) for x,y in a[m])); med=float(np.median(e[m])) if n else 999.
    info.update(inliers=n,residual=med,cells=cells,ok=bool(n>=24 and cells>=4 and med<.65))
    return H,m,info


def lk(a,b):
    p=features(a)
    if len(p)<8: return np.empty((0,2),np.float32),np.empty((0,2),np.float32)
    args=dict(winSize=(31,31),maxLevel=4,criteria=(3,30,.01))
    q,s,_=cv2.calcOpticalFlowPyrLK(a,b,p,None,**args)
    if q is None: return np.empty((0,2),np.float32),np.empty((0,2),np.float32)
    back,sb,_=cv2.calcOpticalFlowPyrLK(b,a,q,None,**args)
    if back is None: return np.empty((0,2),np.float32),np.empty((0,2),np.float32)
    p=p[:,0]; q=q[:,0]; back=back[:,0]
    good=s.ravel().astype(bool)&sb.ravel().astype(bool)&(np.linalg.norm(back-p,axis=1)<.8)
    good&=(q[:,0]>32)&(q[:,0]<608)&(q[:,1]>205)&(q[:,1]<472)
    return p[good],q[good]


def sift(a,b):
    detector=cv2.SIFT_create(nfeatures=1200,contrastThreshold=.015,edgeThreshold=12)
    ka,da=detector.detectAndCompute(a,roi_mask(a)); kb,db=detector.detectAndCompute(b,roi_mask(b))
    if da is None or db is None: return np.empty((0,2),np.float32),np.empty((0,2),np.float32)
    matcher=cv2.BFMatcher(); ms=matcher.knnMatch(da,db,k=2); rev=matcher.knnMatch(db,da,k=2)
    revgood={(m.trainIdx,m.queryIdx) for pair in rev if len(pair)==2 for m,n in [pair] if m.distance<.72*n.distance}
    good=[m for pair in ms if len(pair)==2 for m,n in [pair] if m.distance<.72*n.distance and (m.queryIdx,m.trainIdx) in revgood]
    return np.float32([ka[m.queryIdx].pt for m in good]).reshape(-1,2),np.float32([kb[m.trainIdx].pt for m in good]).reshape(-1,2)


def register(a,b,force_sift=False):
    pa,pb=lk(a,b); _,m,info=fit(pa,pb); method='LK4_FB'
    if not info['ok'] or force_sift:
        sa,sb=sift(a,b); _,sm,si=fit(sa,sb)
        if si['ok'] and (not info['ok'] or si['inliers']>info['inliers']): pa,pb,m,info,method=sa,sb,sm,si,'SIFT_RECOVERY'
    info['method']=method
    if m is None: return pa,pb,info
    return pa[m],pb[m],info


def rays(points):
    r=cv2.undistortPoints(np.asarray(points,np.float32).reshape(-1,1,2),K0,DIST)[:,0]
    return np.c_[r,np.ones(len(r))]


def astra_metric(pa,pb,Rb0,Rb1,range0,normal=np.array([0.,0.,1.])):
    # Literal semantic port of Astra estimator/estimator.py::metric().
    C0=Rb0@BC; C1=Rb1@BC; R=C1.T@C0; n=C0.T@normal
    d=float(normal@(Rb0@(LUNA-CAM+np.array([0.,0.,range0]))))
    a=rays(pa); b=rays(pb); denom=a@n
    if d<=0 or np.min(denom)<.2: return np.full(3,np.nan)
    P=(a*(d/denom)[:,None])@R.T
    A=np.zeros((len(b),2,3)); A[:,0,0]=1; A[:,1,1]=1; A[:,:,2]=-b[:,:2]
    z=P[:,:2]-b[:,:2]*P[:,2:]; w=np.ones(len(b))
    for _ in range(4):
        sw=np.sqrt(w)[:,None,None]
        t=np.linalg.lstsq((A*sw).reshape(-1,3),(z*np.sqrt(w)[:,None]).ravel(),rcond=None)[0]
        e=np.linalg.norm(z-np.einsum('nij,j->ni',A,t),axis=1)
        sc=max(.00003,1.4826*np.median(abs(e-np.median(e))))
        w=np.minimum(1,2.5*sc/np.maximum(e,1e-9))
    dc=C1@t
    return dc-(Rb1-Rb0)@CAM


def read_raw(path):
    out=[]
    with open(path,'rb') as f:
        while True:
            h=f.read(12)
            if not h: break
            if len(h)!=12: raise RuntimeError('partial MJPG header')
            ts,n=struct.unpack('<QI',h); blob=f.read(n)
            if len(blob)!=n: raise RuntimeError('partial JPEG')
            out.append((ts,blob))
    return out


def interp_unwrapped(t,st,v):
    order=np.argsort(st); st=np.asarray(st)[order]; v=np.asarray(v)[order]
    unique=np.r_[True,np.diff(st)>1e-8]; st=st[unique]; v=np.unwrap(v[unique],axis=0)
    return np.array([np.interp(t,st,v[:,k]) for k in range(3)]).T


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('dataset'); ap.add_argument('--gt-mm',type=float,default=None); args=ap.parse_args()
    p=Path(args.dataset)
    rows=list(csv.DictReader(open(p/'optical_flow_mavlink.csv',newline='')))
    frames=read_raw(p/'frames.mjpgbin')
    if len(rows)!=len(frames): raise RuntimeError(f'row/frame mismatch {len(rows)} != {len(frames)}')
    t=np.array([int(r['camera_ts_ns']) for r in rows],np.float64)*1e-9
    st=[]; rpy=[]; rt=[]; rv=[]; events={}
    for idx,r in enumerate(rows):
        age=float(r['fc_gyro_age_ms']); send=int(r['flow_send_ns'])
        if age>=0 and send>0:
            st.append(send*1e-9-age*.001); rpy.append([float(r['fc_roll']),float(r['fc_pitch']),float(r['fc_yaw'])])
        rt.append(int(r['mono_ns'])*1e-9-float(r['luna_age_ms'])*.001); rv.append(float(r['luna_m']))
        ev=str(r.get('return_event','')).strip()
        if ev and ev not in ('0','0.0'): events[int(float(ev))]=idx
    ang=interp_unwrapped(t,np.array(st),np.array(rpy)); rb=np.array([rotation(*a) for a in ang])
    order=np.argsort(rt); rt=np.asarray(rt)[order]; rv=np.asarray(rv)[order]; unique=np.r_[True,np.diff(rt)>1e-8]
    rng=np.interp(t,rt[unique],rv[unique])
    if 1 not in events or 2 not in events: raise RuntimeError(f'A/B events not found: {events}')
    Aidx,Bidx=events[1],events[2]
    pos=np.zeros((len(frames),3)); coverage=np.zeros(len(frames),bool)
    anchor=0; im=cv2.imdecode(np.frombuffer(frames[0][1],np.uint8),0); recovered=0; bad=0
    start=time.time()
    for j in range(1,Bidx+1):
        curr=cv2.imdecode(np.frombuffer(frames[j][1],np.uint8),0)
        dt=abs(t[j]-t[anchor]); pa,pb,info=register(im,curr,force_sift=dt>.1)
        if info['ok']:
            delta=astra_metric(pa,pb,rb[anchor],rb[j],rng[anchor])
            if np.all(np.isfinite(delta)):
                pos[j]=pos[anchor]+delta
                if j>anchor:
                    f=(t[anchor+1:j+1]-t[anchor])/(t[j]-t[anchor]); pos[anchor+1:j+1]=pos[anchor]+f[:,None]*delta
                    coverage[anchor+1:j+1]=True
                if info['method']=='SIFT_RECOVERY' or j-anchor>1: recovered+=1
                anchor=j; im=curr
            else: bad+=1
        else:
            bad+=1
            if dt>2.0:
                anchor=j; im=curr; pos[j]=pos[j-1]
        if j%250==0: print(f'frame {j}/{Bidx} bad={bad} recovered={recovered}',flush=True)
    d=pos[Bidx]-pos[Aidx]; yaw=ang[Aidx,2]; local=rotation(0,0,yaw).T@d; mag=float(np.linalg.norm(d[:2]))*1000
    cov=100*float(np.mean(coverage[Aidx+1:Bidx+1]))
    print('='*72)
    print('ASTRA REFERENCE OFFLINE — monkeysStab')
    print(f'dataset: {p}')
    print(f'A/B zero-based indexes: {Aidx}/{Bidx}; report frames: {Aidx+1}/{Bidx+1}')
    print(f'N/E = ({d[0]*1000:+.6f}, {d[1]*1000:+.6f}) mm')
    print(f'local X/Y = ({local[0]*1000:+.6f}, {local[1]*1000:+.6f}) mm')
    print(f'magnitude = {mag:.6f} mm')
    print(f'coverage = {cov:.3f}%  bad_attempts={bad} recovered={recovered}')
    print('Astra locked reference: X/Y=324.3130605/25.1902813 mm; magnitude=325.2898884 mm')
    if args.gt_mm is not None:
        err=mag-args.gt_mm; print(f'GT comparison only: {args.gt_mm:.3f} mm; error={err:+.3f} mm ({err/args.gt_mm*100:+.3f}%)')
    print(f'elapsed = {time.time()-start:.1f} s')
    print('='*72)

if __name__=='__main__': main()
