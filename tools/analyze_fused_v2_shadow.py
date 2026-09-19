#!/usr/bin/env python3
# Offline diagnostic only. Does not modify production estimator.
import argparse,csv,math

def f(x,k):
    try:return float(x.get(k) or 0)
    except:return 0.0

ap=argparse.ArgumentParser(description="FUSED-V2 offline shadow: fixed visual-health detector + IMU short-horizon reference")
ap.add_argument("runtime_csv")
ap.add_argument("guided_csv")
a=ap.parse_args()
p=list(csv.DictReader(open(a.runtime_csv,newline="")))
g=list(csv.DictReader(open(a.guided_csv,newline="")))

# Frozen detector, preregistered before independent FAST-464 validation.
def eligible(x): return f(x,"dt_s")>0 and int(f(x,"tracked"))>=20
def bad(x): return eligible(x) and f(x,"inlier_ratio")<0.50 and int(f(x,"inliers"))<100
def recovered(x): return eligible(x) and f(x,"inlier_ratio")>0.70 and int(f(x,"inliers"))>=100

# Production WORKED5 endpoint.
pn=pe=0.0
cum=[]
for x in p:
    if int(f(x,"worked5_valid"))==1:
        pn+=f(x,"worked5_dN_m")*1000
        pe+=f(x,"worked5_dE_m")*1000
    cum.append((pn,pe))

bi=next((i for i,x in enumerate(p) if bad(x)),None)
ri=None
if bi is not None:
    streak=0
    for i in range(bi+1,len(p)):
        if recovered(p[i]): streak+=1
        else: streak=0
        if streak>=2:
            ri=i-1
            break

# Development-only pre-roll report: fixed 150 ms maximum; do not optimize against GT.
anchor=bi
if bi is not None:
    budget=0.0
    j=bi
    while j>0 and budget<0.150:
        j-=1
        budget+=max(0.0,f(p[j+1],"dt_s"))
        # Keep walking through the fixed window; report candidate anchor.
    anchor=j

# IMU peak relative to guided start: reference diagnostic, not fused production output.
n0,e0=f(g[0],"imu_n_mm"),f(g[0],"imu_e_mm")
peak=max(((math.hypot(f(x,"imu_n_mm")-n0,f(x,"imu_e_mm")-e0),i,x) for i,x in enumerate(g)),key=lambda z:z[0])

print("===== FUSED-V2 OFFLINE SHADOW =====")
print("WORKED5 endpoint mm =",round(math.hypot(pn,pe),3),"N/E =",round(pn,3),round(pe,3))
if bi is None:
    print("BAD = NONE")
else:
    x=p[bi]
    print("BAD frame =",int(f(x,"frame")),"ratio =",round(f(x,"inlier_ratio"),3),"inliers =",int(f(x,"inliers")),"reason =",int(f(x,"invalid_reason")))
    ax=p[anchor]
    print("fixed 150ms pre-roll anchor frame =",int(f(ax,"frame")),"pre-roll ms =",round(sum(max(0.0,f(p[k],"dt_s")) for k in range(anchor+1,bi+1))*1000,2))
    if ri is None:
        print("RECOVER = NONE")
    else:
        x=p[ri]
        print("RECOVER frame =",int(f(x,"frame")),"ratio =",round(f(x,"inlier_ratio"),3),"inliers =",int(f(x,"inliers")))
d,i,x=peak
print("IMU peak mm =",round(d,3),"gidx =",i,"cam_seq =",int(f(x,"cam_seq")))
print("NOTE: IMU peak is a reference diagnostic; no visual/IMU position summation is performed.")
print("NOTE: detector thresholds and 150ms pre-roll are fixed for the next blind validation.")
