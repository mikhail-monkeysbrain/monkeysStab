#!/usr/bin/env python3
# Offline regression for HIGHRES_CAUSAL15_SHADOW_V1.
# Replays the runtime causal FC->RPi clock map and limits HIGHRES availability
# to samples whose recv_ns is not newer than the camera dequeue timestamp.

import argparse, csv, math
from collections import deque
import numpy as np

def exp_so3(v):
    a=float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k=v/a
    K=np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]],float)
    return np.eye(3)+math.sin(a)*K+(1-math.cos(a))*(K@K)

def rot_dist_deg(a,b):
    d=a.T@b
    c=max(-1.0,min(1.0,(float(np.trace(d))-1.0)*0.5))
    return math.degrees(math.acos(c))

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
        t=(fc_ns-self.fc0)*1e-9
        b=int(math.floor(max(0.0,t)))
        if b==self.bin:
            self.bin_min=min(self.bin_min,off); return
        if b>self.bin:
            tb=float(self.bin)+0.5; ob=float(self.bin_min)
            self.n+=1; self.st+=tb; self.so+=ob; self.stt+=tb*tb; self.sto+=tb*ob
            if self.n>=3:
                den=self.n*self.stt-self.st*self.st
                if abs(den)>1e-9:
                    self.drift=(self.n*self.sto-self.st*self.so)/den
                    self.off0=(self.so-self.drift*self.st)/self.n
            else:
                self.off0=min(self.off0,float(self.bin_min))
            self.bin=b; self.bin_min=off
    def map(self,fc_ns):
        t=(fc_ns-self.fc0)*1e-9
        return fc_ns+int(round(self.off0+self.drift*t))

def integrate_strict(samples,t0,t1,max_gap_ms=30.0):
    if len(samples)<2: return None
    def interp(t):
        hi=next((i for i,x in enumerate(samples) if x[0]>=t),None)
        if hi is None or hi==0: return None
        lo=hi-1; a=samples[lo]; b=samples[hi]
        gap=(b[0]-a[0])*1e-6
        if gap>max_gap_ms or b[0]<=a[0]: return None
        u=(t-a[0])/(b[0]-a[0])
        w=(1-u)*a[1]+u*b[1]
        return w,gap
    e0=interp(t0); e1=interp(t1)
    if e0 is None or e1 is None: return None
    knots=[(t0,e0[0])]
    knots += [(t,w) for t,w in samples if t0<t<t1]
    knots.append((t1,e1[0]))
    R=np.eye(3); angle=0.0
    for (ta,wa),(tb,wb) in zip(knots,knots[1:]):
        dt=(tb-ta)*1e-9
        wm=(wa+wb)*0.5
        rv=wm*dt; R=R@exp_so3(rv); angle+=float(np.linalg.norm(rv))
    return R,math.degrees(angle),len(knots)-1,max(e0[1],e1[1])

def integrate_hold(samples,t0,t1,max_hold_ms=15.0):
    before=[x for x in samples if x[0]<=t0]
    if not before: return None, "no_sample_before_t0", float("nan"), float("nan")
    rate_t,rate=before[-1]
    start=(t0-rate_t)*1e-6
    if start<0 or start>max_hold_ms:
        return None, "start_hold_limit", start, float("nan")
    R=np.eye(3); angle=0.0; seg=0; seg_start=t0
    for t,w in samples:
        if t<=t0 or t>=t1: continue
        dt=(t-seg_start)*1e-9
        if dt<=0 or dt>=0.1: return None, "bad_inner_dt", start, float("nan")
        rv=rate*dt; R=R@exp_so3(rv); angle+=float(np.linalg.norm(rv)); seg+=1
        rate_t=t; rate=w; seg_start=t
    end=(t1-rate_t)*1e-6
    if end<0 or end>max_hold_ms:
        return None, "end_hold_limit", start, end
    dt=(t1-seg_start)*1e-9
    if dt>0:
        rv=rate*dt; R=R@exp_so3(rv); angle+=float(np.linalg.norm(rv)); seg+=1
    return (R,math.degrees(angle),seg,max(start,end)), "ok", start, end

def read_csv(path):
    with open(path,newline='') as f: return list(csv.DictReader(f))

def pct(v,p):
    if not v:return float('nan')
    a=np.asarray(v,float)
    return float(np.percentile(a,p))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("main_csv"); ap.add_argument("deltar_csv"); ap.add_argument("highres_csv")
    ap.add_argument("--start-ns",type=int,required=True); ap.add_argument("--end-ns",type=int,required=True)
    ap.add_argument("--hold-ms",type=float,default=15.0)
    a=ap.parse_args()
    main_rows=read_csv(a.main_csv); dr=read_csv(a.deltar_csv); hr=read_csv(a.highres_csv)
    # optical_flow_mavlink.csv first columns are now,ts,v4l2_ts,dq_mono,...
    by_frame={int(r["frame"]):r for r in main_rows if r.get("frame","").strip()}
    targets=[]
    for r in dr:
        t0=int(r["t0_ns"]); t1=int(r["t1_ns"])
        if not(a.start_ns<=t1<=a.end_ns): continue
        if int(r.get("w5_valid","0"))!=1: continue
        if int(r.get("stabilised_unified_shadow_valid","0"))!=0: continue
        targets.append(r)
    raw=[]
    for r in hr:
        if not r.get("fc_time_usec","").strip(): continue
        fc=int(r["fc_time_usec"])*1000; recv=int(r["recv_ns"])
        vals=[r.get("corr_gx_rad_s"),r.get("corr_gy_rad_s"),r.get("corr_gz_rad_s")]
        # The live HIGHRES logger can leave a truncated/incomplete final row
        # when the process is still running or the file is copied mid-write.
        # Such a row is not a valid gyro sample and must not abort the replay.
        try:
            if any(v is None or not str(v).strip() for v in vals):
                continue
            w=np.array([float(v) for v in vals])
        except (TypeError,ValueError):
            continue
        if not np.all(np.isfinite(w)):
            continue
        raw.append((recv,fc,w))
    raw.sort(key=lambda x:x[0])
    mapper=ClockMap(); hist=deque(); hi=0
    recovered=0; strict_rt=0; diffs=[]; holds=[]; rows=[]
    buckets={"causal15":[],"causal20_only":[],"causal25_only":[],"over25":[]}
    missing_w5_columns=set()
    for r in sorted(targets,key=lambda x:int(x["t1_ns"])):
        frame=int(r["frame"]); t0=int(r["t0_ns"]); t1=int(r["t1_ns"])
        m=by_frame.get(frame)
        if m is None: continue
        dq=int(m.get("selected_dq_mono_ns") or m.get("dq_mono_ns") or list(m.values())[3])
        while hi<len(raw) and raw[hi][0]<=dq:
            recv,fc,w=raw[hi]; mapper.update(fc,recv)
            hist.append((fc,w)); hi+=1
        while hist and mapper.map(hist[0][0])<t0-100_000_000: hist.popleft()
        mapped=[(mapper.map(fc),w) for fc,w in hist]
        mapped.sort(key=lambda x:x[0])
        st=integrate_strict(mapped,t0,t1)
        c15,_,_,_=integrate_hold(mapped,t0,t1,15.0)
        c20,_,_,_=integrate_hold(mapped,t0,t1,20.0)
        c25,_,_,_=integrate_hold(mapped,t0,t1,25.0)
        def pick(keys):
            for k in keys:
                v=r.get(k)
                if v is not None and str(v).strip():
                    return float(v)
            return None
        # Deltar logs may not carry per-frame W5 dN/dE. Prefer them when
        # present; otherwise derive the W5 step from consecutive accumulated
        # positions in optical_flow_mavlink.csv using frame-1 -> frame.
        wn=pick(("w5_dN_m","worked5_dN_m","w5_dn_m","worked5_dn_m"))
        we=pick(("w5_dE_m","worked5_dE_m","w5_de_m","worked5_de_m"))
        if wn is None or we is None:
            prevm=by_frame.get(frame-1)
            nkeys=("worked5_n_m","w5_n_m","worked5_n","w5_n")
            ekeys=("worked5_e_m","w5_e_m","worked5_e","w5_e")
            def pickrow(row,keys):
                if row is None: return None
                for k in keys:
                    v=row.get(k)
                    if v is not None and str(v).strip():
                        return float(v)
                return None
            n1,e1=pickrow(m,nkeys),pickrow(m,ekeys)
            n0,e0=pickrow(prevm,nkeys),pickrow(prevm,ekeys)
            if None not in (n1,e1,n0,e0):
                wn,we=n1-n0,e1-e0
            else:
                missing_w5_columns.update(k for k in (*nkeys,*ekeys) if k not in m)
        if wn is not None and we is not None:
            key="causal15" if c15 else ("causal20_only" if c20 else ("causal25_only" if c25 else "over25"))
            buckets[key].append((wn,we))
        ca,ca_reason,start_hold,end_hold=integrate_hold(mapped,t0,t1,a.hold_ms)
        if st: strict_rt+=1
        if ca:
            recovered+=1; holds.append(ca[3])
        # Oracle strict uses future samples, but the same causal map state.
        oracle_raw=[]
        j=hi
        while j<len(raw) and raw[j][0]<=dq+50_000_000:
            oracle_raw.append((mapper.map(raw[j][1]),raw[j][2])); j+=1
        oracle=integrate_strict(mapped+oracle_raw,t0,t1)
        diff=float('nan')
        if ca and oracle:
            diff=rot_dist_deg(oracle[0],ca[0]); diffs.append(diff)
        rows.append((frame,t0,t1,0 if st is None else 1,0 if ca is None else 1,
                     -1 if ca is None else ca[3],start_hold,end_hold,ca_reason,diff))
    print(f"targets={len(targets)} evaluated={len(rows)}")
    print(f"strict_realtime_valid={strict_rt}")
    print(f"causal{a.hold_ms:g}_valid={recovered}/{len(rows)}")
    if holds:
        print(f"hold_ms median={pct(holds,50):.3f} p95={pct(holds,95):.3f} max={max(holds):.3f}")
    if diffs:
        print(f"deltaR_error_deg median={pct(diffs,50):.6f} p95={pct(diffs,95):.6f} max={max(diffs):.6f}")
    from collections import Counter
    print("causal_invalid_reasons="+str(dict(Counter(x[8] for x in rows if x[4]==0))))
    print("===== W5 MOVEMENT BY CAUSAL COVERAGE =====")\n    if sum(len(v) for v in buckets.values())==0:\n        print("w5_movement_status=UNAVAILABLE columns="+",".join(sorted(missing_w5_columns)))
    total=np.array([0.0,0.0]); cumulative=np.array([0.0,0.0])
    for name in ("causal15","causal20_only","causal25_only","over25"):
        arr=buckets[name]
        vec=np.sum(np.asarray(arr,float),axis=0) if arr else np.array([0.0,0.0])
        total+=vec
        if name!="over25": cumulative+=vec
        print(f"{name}: frames={len(arr)} dN={vec[0]*1000:.3f}mm dE={vec[1]*1000:.3f}mm vec={np.linalg.norm(vec)*1000:.3f}mm")
        if name=="causal15": print(f"cumulative15: vec={np.linalg.norm(cumulative)*1000:.3f}mm")
        elif name=="causal20_only": print(f"cumulative20: vec={np.linalg.norm(cumulative)*1000:.3f}mm")
        elif name=="causal25_only": print(f"cumulative25: vec={np.linalg.norm(cumulative)*1000:.3f}mm")
    print(f"all_targets_vector: dN={total[0]*1000:.3f}mm dE={total[1]*1000:.3f}mm vec={np.linalg.norm(total)*1000:.3f}mm")
    print("frame,t0_ns,t1_ns,strict_rt,causal,hold_ms,start_hold_ms,end_hold_ms,reason,oracle_diff_deg")
    for x in rows: print(",".join(str(v) for v in x))

if __name__=="__main__": main()
