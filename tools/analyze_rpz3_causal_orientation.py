#!/usr/bin/env python3
"""RPZ3: проверка causal абсолютной ориентации ATTITUDE+corrected HIGHRES.

Не меняет production. Для каждого camera timestamp берёт только ATTITUDE/HIGHRES,
которые уже были получены к camera_dequeue_ns. ATTITUDE задаёт абсолютный anchor,
corrected HIGHRES распространяет его до camera timestamp. Сравнение — с
постфактум интерполированным ATTITUDE по mapped FC sample time.
"""
import argparse,csv,bisect,math
import numpy as np

def exp_so3(v):
    th=float(np.linalg.norm(v))
    if th<1e-12:return np.eye(3)
    k=v/th; K=np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]],float)
    return np.eye(3)+math.sin(th)*K+(1-math.cos(th))*(K@K)
def euler(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                     [-sp,cp*sr,cp*cr]],float)
def rd(a,b):
    c=max(-1.,min(1.,(float(np.trace(a.T@b))-1.)*.5));return math.degrees(math.acos(c))
def fv(r,k):
    try:return float(r[k])
    except:return float("nan")
def iv(r,k):
    try:return int(float(r[k]))
    except:return 0
def pct(v,p):
    v=sorted(x for x in v if math.isfinite(x))
    if not v:return float("nan")
    x=(len(v)-1)*p/100.;a=int(x);b=min(a+1,len(v)-1);return v[a]+(x-a)*(v[b]-v[a])
def interp_angle(a,b,u):
    d=(b-a+math.pi)%(2*math.pi)-math.pi;return a+u*d
def oracle(att,t):
    ts=[x[1] for x in att]; k=bisect.bisect_left(ts,t)
    if k<=0 or k>=len(att):return None
    a,b=att[k-1],att[k]
    if b[1]<=a[1] or (b[1]-a[1])*1e-6>40:return None
    u=(t-a[1])/(b[1]-a[1])
    return euler(interp_angle(a[2],b[2],u),interp_angle(a[3],b[3],u),interp_angle(a[4],b[4],u))
def propagate(R,t0,t1,hr):
    if t1<=t0:return R
    times=[x[1] for x in hr]; k=bisect.bisect_right(times,t0)-1
    if k<0:return None
    cur_t=t0; w=hr[k][2]; out=R.copy()
    k+=1
    while k<len(hr) and hr[k][1]<t1:
        tt=hr[k][1]; dt=(tt-cur_t)*1e-9
        if dt>0:out=out@exp_so3(w*dt)
        cur_t=tt;w=hr[k][2];k+=1
    dt=(t1-cur_t)*1e-9
    if dt>0:out=out@exp_so3(w*dt)
    return out

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("main_csv");ap.add_argument("attitude_csv");ap.add_argument("highres_csv")
    ap.add_argument("--start-s",type=float,default=36965.416638);ap.add_argument("--end-s",type=float,default=36973.928712)
    a=ap.parse_args()
    with open(a.main_csv,newline="") as f: main=list(csv.DictReader(f))
    with open(a.attitude_csv,newline="") as f:
        att=[(iv(r,"recv_ns"),iv(r,"mapped_sample_ns"),fv(r,"roll_rad"),fv(r,"pitch_rad"),fv(r,"yaw_rad"))
             for r in csv.DictReader(f) if iv(r,"clock_map_valid")==1 and iv(r,"mapped_sample_ns")>0]
    with open(a.highres_csv,newline="") as f:
        raw_hr=[]
        for r in csv.DictReader(f):
            vals=(r.get("corr_gx_rad_s"),r.get("corr_gy_rad_s"),r.get("corr_gz_rad_s"))
            try:
                if any(v is None or not str(v).strip() for v in vals): continue
                w=np.array([float(x) for x in vals],float)
            except (TypeError,ValueError):
                continue
            fc=iv(r,"fc_time_usec")*1000
            recv=iv(r,"recv_ns")
            if fc>0 and recv>0 and np.all(np.isfinite(w)):
                raw_hr.append((recv,fc,w))
    raw_hr.sort(key=lambda x:x[0])

    # Same lower-envelope affine FC->RPi mapper used by the validated RPZ2
    # causal replay: one minimum receive offset per FC second, OLS after bins.
    class ClockMap:
        def __init__(self):
            self.valid=False; self.fc0=0; self.bin=-1; self.bin_min=0
            self.n=0; self.st=self.so=self.stt=self.sto=0.0
            self.off0=0.0; self.drift=0.0
        def update(self,fc_ns,recv_ns):
            off=recv_ns-fc_ns
            if not self.valid:
                self.valid=True; self.fc0=fc_ns; self.bin=0
                self.bin_min=off; self.off0=float(off); return
            tt=(fc_ns-self.fc0)*1e-9
            b=int(math.floor(max(0.0,tt)))
            if b==self.bin:
                self.bin_min=min(self.bin_min,off); return
            if b>self.bin:
                tb=float(self.bin)+0.5; ob=float(self.bin_min)
                self.n+=1; self.st+=tb; self.so+=ob
                self.stt+=tb*tb; self.sto+=tb*ob
                if self.n>=3:
                    den=self.n*self.stt-self.st*self.st
                    if abs(den)>1e-9:
                        self.drift=(self.n*self.sto-self.st*self.so)/den
                        self.off0=(self.so-self.drift*self.st)/self.n
                else:
                    self.off0=min(self.off0,float(self.bin_min))
                self.bin=b; self.bin_min=off
        def map(self,fc_ns):
            tt=(fc_ns-self.fc0)*1e-9
            return fc_ns+int(round(self.off0+self.drift*tt))

    mapper=ClockMap()
    hr=[]
    for recv,fc,w in raw_hr:
        mapper.update(fc,recv)
        hr.append((recv,mapper.map(fc),w))
    att.sort();hr.sort()
    arecv=[x[0] for x in att]; hrecv=[x[0] for x in hr]
    targets=[]
    for r in main:
        ts=fv(r,"mono_ns")*1e-9
        if a.start_s<=ts<=a.end_s and iv(r,"worked5_valid")==1 and iv(r,"stabilised_publish_ready")==0:
            dq=iv(r,"camera_dequeue_ns") or iv(r,"mono_ns"); targets.append((r,dq,iv(r,"mono_ns")))
    print(f"targets={len(targets)} attitude={len(att)} highres={len(hr)}")
    if not hr:raise SystemExit("No valid HIGHRES rows after fc_time_usec mapping.")
    limits=(15,20,25,30,35,40)
    for lim in limits:
        errs=[];ok=0;no_oracle=0;no_hr=0
        for r,dq,tcam in targets:
            ka=bisect.bisect_right(arecv,dq)-1
            if ka<0:continue
            an=att[ka]; age=(dq-an[0])*1e-6
            if age<0 or age>lim:continue
            # Strict causality: HIGHRES history is truncated by receive time.
            kh=bisect.bisect_right(hrecv,dq)
            causal_hr=hr[:kh]
            R0=euler(an[2],an[3],an[4])
            Rp=propagate(R0,an[1],tcam,causal_hr)
            if Rp is None:no_hr+=1;continue
            Ro=oracle(sorted(att,key=lambda x:x[1]),tcam)
            if Ro is None:no_oracle+=1;continue
            ok+=1;errs.append(rd(Rp,Ro))
        print(f"anchor<={lim}ms: valid={ok}/{len(targets)} no_hr={no_hr} no_oracle={no_oracle} "
              f"err_deg median={pct(errs,50):.5f} p95={pct(errs,95):.5f} max={(max(errs) if errs else float('nan')):.5f}")

if __name__=="__main__":main()
