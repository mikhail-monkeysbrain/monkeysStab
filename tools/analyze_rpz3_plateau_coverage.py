#!/usr/bin/env python3
"""RPZ3: независимые наземные плато, A->B и temporal coverage Variant B.

Границы выбираются по высоте/покою WORKED5. Известные 480 мм используются
только после выбора границ для отчёта ошибки A->B.
"""
import argparse,csv,math,statistics

def fv(r,k):
    try:return float(r[k])
    except (KeyError,TypeError,ValueError):return float("nan")
def iv(r,k):
    try:return int(float(r[k]))
    except (KeyError,TypeError,ValueError):return 0
def med(v):
    v=[x for x in v if math.isfinite(x)]
    return statistics.median(v) if v else float("nan")
def vec(dn,de):return math.hypot(dn,de)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("main_csv")
    ap.add_argument("--physical-ab-mm",type=float,default=480.0)
    ap.add_argument("--ground-max-m",type=float,default=0.22)
    ap.add_argument("--quiet-step-mm",type=float,default=0.8)
    ap.add_argument("--min-plateau-s",type=float,default=1.5)
    ap.add_argument("--seed1",type=float,default=36969.44)
    ap.add_argument("--seed2",type=float,default=36987.63)
    a=ap.parse_args()
    with open(a.main_csv,newline="") as f: rows=list(csv.DictReader(f))
    if not rows:raise SystemExit("empty CSV")
    t=[fv(r,"mono_ns")*1e-9 for r in rows]
    n=[fv(r,"worked5_acc_n_m") for r in rows]; e=[fv(r,"worked5_acc_e_m") for r in rows]
    h=[fv(r,"worked5_hcam_m") for r in rows]
    # Ground+quiet mask. Per-frame W5 step is diagnostic only and does not use 480 mm.
    q=[]
    for i,r in enumerate(rows):
        step=vec(fv(r,"worked5_dN_m"),fv(r,"worked5_dE_m"))*1000
        q.append(iv(r,"worked5_valid")==1 and math.isfinite(h[i]) and h[i]<=a.ground_max_m
                 and math.isfinite(step) and step<=a.quiet_step_mm)
    runs=[]; i=0
    while i<len(rows):
        if not q[i]: i+=1; continue
        j=i
        while j+1<len(rows) and q[j+1]: j+=1
        if t[j]-t[i]>=a.min_plateau_s:runs.append((i,j,t[i],t[j]))
        i=j+1
    print("===== GROUND QUIET PLATEAUS =====")
    for k,(i,j,t0,t1) in enumerate(runs):
        print(f"P{k}: {t0:.6f}->{t1:.6f} dur={t1-t0:.2f}s "
              f"W5=({med(n[i:j+1])*1000:.2f},{med(e[i:j+1])*1000:.2f})mm "
              f"h={med(h[i:j+1])*1000:.1f}mm")
    before=[x for x in runs if x[3]<a.seed1]
    between=[x for x in runs if x[2]>a.seed1 and x[3]<a.seed2]
    after=[x for x in runs if x[2]>a.seed2]
    if not before or not between:
        raise SystemExit("Не удалось автоматически найти плато A/B; пороги не менять вслепую.")
    A=max(before,key=lambda x:x[3]); B=max(between,key=lambda x:x[3])
    C=min(after,key=lambda x:x[2]) if after else None
    def state(run):
        i,j,_,_=run
        return (med(n[i:j+1]),med(e[i:j+1]),
                med([fv(r,"ekf_x_ned") for r in rows[i:j+1]]),
                med([fv(r,"ekf_y_ned") for r in rows[i:j+1]]))
    sa=state(A); sb=state(B)
    print("\n===== SELECTED A/B PLATEAUS =====")
    print(f"A: {A[2]:.6f}->{A[3]:.6f}")
    print(f"B: {B[2]:.6f}->{B[3]:.6f}")
    wdn,wde=sb[0]-sa[0],sb[1]-sa[1]; wmag=vec(wdn,wde)
    edn,ede=sb[2]-sa[2],sb[3]-sa[3]; emag=vec(edn,ede)
    gt=a.physical_ab_mm/1000
    print(f"W5 A->B: dN={wdn*1000:.2f} dE={wde*1000:.2f} vec={wmag*1000:.2f}mm "
          f"error={(wmag/gt-1)*100:+.2f}%")
    print(f"EKF A->B: dN={edn*1000:.2f} dE={ede*1000:.2f} vec={emag*1000:.2f}mm "
          f"error={(emag/gt-1)*100:+.2f}%")
    # Full transfer interval: end of A plateau -> start of B plateau.
    lo=A[1]+1; hi=B[0]-1
    all_dn=all_de=miss_dn=miss_de=0.0; valid=miss=ready=0
    src={}
    for r in rows[lo:hi+1]:
        if iv(r,"worked5_valid")!=1:continue
        dn=fv(r,"worked5_dN_m"); de=fv(r,"worked5_dE_m")
        if not(math.isfinite(dn) and math.isfinite(de)):continue
        valid+=1; all_dn+=dn; all_de+=de
        rd=iv(r,"stabilised_publish_ready")
        if rd:ready+=1
        else:miss+=1; miss_dn+=dn; miss_de+=de
        s=iv(r,"stabilised_publish_source"); src[s]=src.get(s,0)+1
    print("\n===== A->B VARIANT-B COVERAGE =====")
    print(f"interval={t[lo]:.6f}->{t[hi]:.6f} W5_valid={valid} ready={ready} missing={miss} "
          f"ready_fraction={(ready/valid if valid else 0)*100:.2f}% sources={src}")
    print(f"W5 all-frame step sum: dN={all_dn*1000:.2f} dE={all_de*1000:.2f} vec={vec(all_dn,all_de)*1000:.2f}mm")
    print(f"W5 on B-missing frames: dN={miss_dn*1000:.2f} dE={miss_de*1000:.2f} "
          f"vec={vec(miss_dn,miss_de)*1000:.2f}mm")
    if C:
        sc=state(C)
        print("\n===== RETURN (NO 480-mm ERROR CLAIM) =====")
        print(f"C: {C[2]:.6f}->{C[3]:.6f}")
        print(f"W5 B->C: dN={(sc[0]-sb[0])*1000:.2f} dE={(sc[1]-sb[1])*1000:.2f} "
              f"vec={vec(sc[0]-sb[0],sc[1]-sb[1])*1000:.2f}mm")

if __name__=="__main__":main()
