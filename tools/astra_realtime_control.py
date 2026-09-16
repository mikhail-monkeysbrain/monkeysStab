#!/usr/bin/env python3
# ASTRA realtime shadow control run for OV9281 + TF-Luna/FC.
# Does NOT publish optical flow to FC. It only estimates A->B metric displacement.
import math, queue, sys, threading, time
from dataclasses import dataclass
import cv2
import numpy as np
from pymavlink import mavutil

CAM_DEV="/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0"
FC="tcp:127.0.0.1:5760"
K0=np.array([[568.5317075216523,0,315.98271077441063],
             [0,569.6800556286586,239.8814858910064],[0,0,1.]],dtype=np.float64)
DIST=np.array([.07356919219402849,-.095253893789117,-.0108105307571873,
               -.002284337357697,.08217740080275748],dtype=np.float64)
BC=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
CAM=np.array([.0625,0.,.05]); LUNA=np.array([.0855,0.,.055])
ROI=(.10,.44,.90,.96)

def rotation(roll,pitch,yaw):
    cr,sr=math.cos(roll),math.sin(roll); cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                     [-sp,cp*sr,cp*cr]])

def mask(im):
    h,w=im.shape
    m=np.zeros_like(im)
    x0,y0,x1,y1=ROI
    m[int(h*y0):int(h*y1),int(w*x0):int(w*x1)]=255
    return m

def fit(a,b):
    if len(a)<8: return None,None,False,0,999.
    H,m=cv2.findHomography(a,b,cv2.RANSAC,1.0,maxIters=2000,confidence=.999)
    if H is None: return None,None,False,0,999.
    m=m.ravel().astype(bool)
    pred=cv2.perspectiveTransform(a.reshape(-1,1,2),H)[:,0]
    e=np.linalg.norm(pred-b,axis=1); n=int(m.sum())
    cells=len(set((int(x//160),int(y//80)) for x,y in a[m]))
    med=float(np.median(e[m])) if n else 999.
    return H,m,bool(n>=24 and cells>=4 and med<.65),n,med

def fast_register(prev,curr):
    s0=cv2.resize(prev,(320,240)); s1=cv2.resize(curr,(320,240))
    p=cv2.goodFeaturesToTrack(s0,120,.01,5,mask=mask(s0))
    if p is None: return None
    args=dict(winSize=(21,21),maxLevel=3,criteria=(3,25,.01))
    q,s,_=cv2.calcOpticalFlowPyrLK(s0,s1,p,None,**args)
    if q is None: return None
    back,sb,_=cv2.calcOpticalFlowPyrLK(s1,s0,q,None,**args)
    if back is None: return None
    good=s.ravel().astype(bool)&sb.ravel().astype(bool)
    good &= np.linalg.norm(back[:,0]-p[:,0],axis=1)<.4
    good &= (q[:,0,0]>16)&(q[:,0,0]<304)&(q[:,0,1]>102.5)&(q[:,0,1]<236)
    a=p[good,0]*2.; b=q[good,0]*2.
    H,m,ok,n,res=fit(a,b)
    if not ok: return None
    return a[m],b[m],n,res

def rays(points):
    r=cv2.undistortPoints(np.asarray(points,np.float32).reshape(-1,1,2),K0,DIST)[:,0]
    return np.c_[r,np.ones(len(r))]

def metric(pa,pb,Rb0,Rb1,range0,normal=np.array([0.,0.,1.])):
    C0=Rb0@BC; C1=Rb1@BC; R=C1.T@C0; n=C0.T@normal
    d=float(normal@(Rb0@(LUNA-CAM+np.array([0.,0.,range0]))))
    a=rays(pa); b=rays(pb); denom=a@n
    if d<=0 or np.min(denom)<.2: return None
    P=(a*(d/denom)[:,None])@R.T
    A=np.zeros((len(b),2,3)); A[:,0,0]=1; A[:,1,1]=1; A[:,:,2]=-b[:,:2]
    z=P[:,:2]-b[:,:2]*P[:,2:]; w=np.ones(len(b))
    for _ in range(4):
        sw=np.sqrt(w)[:,None,None]
        t=np.linalg.lstsq((A*sw).reshape(-1,3),(z*np.sqrt(w)[:,None]).ravel(),rcond=None)[0]
        e=np.linalg.norm(z-np.einsum('nij,j->ni',A,t),axis=1)
        med=np.median(e); sc=max(.00003,1.4826*np.median(np.abs(e-med)))
        w=np.minimum(1.,2.5*sc/np.maximum(e,1e-9))
    dc=C1@t
    return dc-(Rb1-Rb0)@CAM

@dataclass
class Sensors:
    roll: float=0.; pitch: float=0.; yaw: float=0.; rng: float=float("nan")
    att_t: float=0.; rng_t: float=0.

class SensorThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True); self.s=Sensors(); self.lock=threading.Lock(); self.ok=False
    def run(self):
        m=mavutil.mavlink_connection(FC)
        m.wait_heartbeat(timeout=10); self.ok=True
        try:
            m.mav.request_data_stream_send(m.target_system,m.target_component,
                                           mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,100,1)
            m.mav.request_data_stream_send(m.target_system,m.target_component,
                                           mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,20,1)
        except Exception: pass
        while True:
            x=m.recv_match(blocking=True,timeout=1)
            if x is None: continue
            now=time.monotonic(); typ=x.get_type()
            with self.lock:
                if typ=="ATTITUDE":
                    self.s.roll=float(x.roll); self.s.pitch=float(x.pitch); self.s.yaw=float(x.yaw); self.s.att_t=now
                elif typ=="DISTANCE_SENSOR":
                    # ArduPilot DISTANCE_SENSOR current_distance is cm.
                    self.s.rng=float(x.current_distance)*.01; self.s.rng_t=now
    def get(self):
        with self.lock: return Sensors(**self.s.__dict__)

class CameraThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True); self.q=queue.Queue(maxsize=2); self.dropped=0
    def run(self):
        cap=cv2.VideoCapture(CAM_DEV,cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        cap.set(cv2.CAP_PROP_FPS,100)
        if not cap.isOpened(): raise RuntimeError("OV9281 open failed")
        while True:
            ok,im=cap.read()
            if not ok: continue
            if im.ndim==3: im=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
            item=(time.monotonic(),im)
            if self.q.full():
                try: self.q.get_nowait(); self.dropped+=1
                except queue.Empty: pass
            self.q.put_nowait(item)

def wait_space(text):
    print("\n"+text,flush=True)
    while True:
        s=input().strip()
        if s=="": return

def main():
    cv2.setNumThreads(1); cv2.setRNGSeed(716)
    print("======================================================================")
    print("ASTRA REALTIME SHADOW — CONTROL A->B")
    print("GT не используется estimator-ом. В FC ничего не публикуется.")
    print("======================================================================")
    st=SensorThread(); st.start()
    t0=time.monotonic()
    while not st.ok and time.monotonic()-t0<12: time.sleep(.05)
    if not st.ok: raise RuntimeError("нет heartbeat FC")
    ct=CameraThread(); ct.start()
    t,im=ct.q.get(timeout=10)
    while True:
        s=st.get()
        if time.monotonic()-s.att_t<.15 and time.monotonic()-s.rng_t<.25 and .10<s.rng<7: break
        time.sleep(.05)
    wait_space("СИСТЕМА ГОТОВА. Поставь БПЛА в A, полностью останови и нажми ENTER (SPACE тоже можно, затем ENTER).")
    # input() is line based in this compact control tool.
    while not ct.q.empty():
        try: ct.q.get_nowait()
        except queue.Empty: break
    ta,prev=ct.q.get(timeout=2); sp=st.get(); Rp=rotation(sp.roll,sp.pitch,sp.yaw); rp=sp.rng
    total=np.zeros(3); accepted=failed=0; ms=[]
    print("\nA ЗАФИКСИРОВАНА. Двигай аппарат по столу в B.")
    print("После полной остановки нажми ENTER.",flush=True)
    done=threading.Event()
    def keywait(): input(); done.set()
    threading.Thread(target=keywait,daemon=True).start()
    while not done.is_set():
        try: tc,curr=ct.q.get(timeout=.2)
        except queue.Empty: continue
        sc=st.get()
        if time.monotonic()-sc.att_t>.15 or time.monotonic()-sc.rng_t>.25: failed+=1; continue
        R1=rotation(sc.roll,sc.pitch,sc.yaw)
        q0=time.perf_counter(); reg=fast_register(prev,curr); ms.append((time.perf_counter()-q0)*1000)
        if reg is None:
            failed+=1
            # Astra rule: keep last accepted anchor; do not silently zero displacement.
            continue
        pa,pb,n,res=reg
        d=metric(pa,pb,Rp,R1,rp)
        if d is None or not np.all(np.isfinite(d)):
            failed+=1; continue
        total += d; accepted+=1
        prev=curr; Rp=R1; rp=sc.rng
    mag=float(np.linalg.norm(total[:2]))
    print("\nТОЧКА B ЗАФИКСИРОВАНА.")
    gt=float(input("Измерь физическое A->B и введи GT в мм: ").strip().replace(",","."))
    err=mag*1000-gt
    cov=100*accepted/max(1,accepted+failed)
    print("\n======================================================================")
    print("ASTRA REALTIME CONTROL RESULT")
    print(f"estimate N/E = ({total[0]*1000:+.1f}, {total[1]*1000:+.1f}) mm")
    print(f"estimate magnitude = {mag*1000:.1f} mm")
    print(f"GT = {gt:.1f} mm")
    print(f"error = {err:+.1f} mm ({err/gt*100:+.2f} %)")
    print(f"accepted/failed = {accepted}/{failed}; interval coverage proxy = {cov:.1f} %")
    if ms: print(f"fast tracker latency median/p95 = {np.median(ms):.2f}/{np.percentile(ms,95):.2f} ms")
    print(f"camera queue dropped = {ct.dropped}")
    print("======================================================================")
if __name__=="__main__":
    try: main()
    except KeyboardInterrupt: pass
