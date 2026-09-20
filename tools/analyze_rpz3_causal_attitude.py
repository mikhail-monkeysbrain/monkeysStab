#!/usr/bin/env python3
"""RPZ3 causal ATTITUDE timing audit. Standard library only.

Answers one narrow question before production changes: when Variant B was
not ready during measured A->B, was a recent ATTITUDE anchor already causally
available, and how old was its FC sample / receive time?
"""
import argparse,csv,bisect,math,statistics

def f(r,k):
    try:return float(r[k])
    except (KeyError,TypeError,ValueError):return float("nan")
def i(r,k):
    try:return int(float(r[k]))
    except (KeyError,TypeError,ValueError):return 0
def pct(v,p):
    v=sorted(x for x in v if math.isfinite(x))
    if not v:return float("nan")
    x=(len(v)-1)*p/100.0; a=int(math.floor(x)); b=int(math.ceil(x))
    return v[a] if a==b else v[a]+(x-a)*(v[b]-v[a])
def stats(name,v):
    v=[x for x in v if math.isfinite(x)]
    if not v:print(f"{name}: n=0");return
    print(f"{name}: n={len(v)} median={pct(v,50):.3f} p95={pct(v,95):.3f} "
          f"p99={pct(v,99):.3f} min={min(v):.3f} max={max(v):.3f}")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("main_csv");ap.add_argument("attitude_csv")
    ap.add_argument("--start-s",type=float,default=36965.416638)
    ap.add_argument("--end-s",type=float,default=36973.928712)
    a=ap.parse_args()
    with open(a.main_csv,newline="") as fh: main=list(csv.DictReader(fh))
    with open(a.attitude_csv,newline="") as fh: att=list(csv.DictReader(fh))
    A=[]
    for r in att:
        recv=i(r,"recv_ns"); mapped=i(r,"mapped_sample_ns")
        if recv<=0:continue
        A.append((recv,mapped,i(r,"clock_map_valid"),f(r,"mapped_transport_ms")))
    A.sort()
    recvs=[x[0] for x in A]
    intervals=[(A[k][0]-A[k-1][0])*1e-6 for k in range(1,len(A))]
    transport=[x[3] for x in A if x[2]==1]
    print("===== ATTITUDE STREAM =====")
    print(f"rows={len(A)} clock_map_valid={sum(x[2] for x in A)}")
    stats("recv_interval_ms",intervals);stats("mapped_transport_ms",transport)
    neg=sum(1 for x in transport if x<0)
    print(f"mapped_transport_negative={neg}/{len(transport)} ({100*neg/len(transport) if transport else 0:.2f}%)")

    targets=[]
    for r in main:
        ts=f(r,"mono_ns")*1e-9
        if not(a.start_s<=ts<=a.end_s):continue
        if i(r,"worked5_valid")!=1 or i(r,"stabilised_publish_ready")!=0:continue
        # camera_dequeue_ns is the causal processing availability boundary.
        dq=i(r,"camera_dequeue_ns")
        if dq<=0:dq=i(r,"mono_ns")
        targets.append((r,dq))
    print("\n===== B-MISSING A->B TARGETS =====")
    print(f"targets={len(targets)}")
    recv_age=[];sample_age=[];future_sample=[];no_anchor=0
    within={10:0,15:0,20:0,25:0,30:0,40:0,50:0}
    rows=[]
    for r,dq in targets:
        k=bisect.bisect_right(recvs,dq)-1
        if k<0:
            no_anchor+=1;continue
        recv,mapped,valid,tr=A[k]
        ra=(dq-recv)*1e-6; recv_age.append(ra)
        sa=float("nan")
        if valid and mapped>0:
            sa=(dq-mapped)*1e-6;sample_age.append(sa)
            if sa<0:future_sample.append(sa)
            for lim in within:
                if 0<=sa<=lim:within[lim]+=1
        rows.append((i(r,"frame"),f(r,"mono_ns")*1e-9,ra,sa,tr,valid))
    stats("latest_anchor_recv_age_ms",recv_age)
    stats("latest_anchor_mapped_sample_age_ms",sample_age)
    print(f"mapped_sample_in_future={len(future_sample)}/{len(sample_age)}")
    print("causal_mapped_sample_age_coverage="+
          " ".join(f"<= {k}ms:{v}/{len(targets)}" for k,v in within.items()))
    print(f"no_received_anchor={no_anchor}")
    print("\n===== TARGET DETAIL =====")
    print("frame,mono_s,recv_age_ms,mapped_sample_age_ms,mapped_transport_ms,map_valid")
    for x in rows:print(",".join(str(v) for v in x))

if __name__=="__main__":main()
