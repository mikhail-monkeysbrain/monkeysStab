#!/usr/bin/env python3
"""Выделяет крупные ручные перемещения RPZ3 по frozen WORKED5 без pandas/numpy."""
import argparse, csv, math

def f(row,key):
    try: return float(row[key])
    except (KeyError,TypeError,ValueError): return float("nan")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("main_csv")
    ap.add_argument("--window-s",type=float,default=0.5)
    ap.add_argument("--top",type=int,default=80)
    ap.add_argument("--cluster-gap-s",type=float,default=2.0)
    ap.add_argument("--edge-window-s",type=float,default=0.20)
    ap.add_argument("--edge-threshold-mm",type=float,default=2.0)
    ap.add_argument("--quiet-windows",type=int,default=3)
    a=ap.parse_args()
    with open(a.main_csv,newline="") as fh: rows=list(csv.DictReader(fh))
    if not rows: raise SystemExit("empty CSV")
    need=("mono_ns","worked5_acc_n_m","worked5_acc_e_m")
    miss=[k for k in need if k not in rows[0]]
    if miss: raise SystemExit("missing columns: "+",".join(miss))
    t=[f(r,"mono_ns")*1e-9 for r in rows]
    n=[f(r,"worked5_acc_n_m") for r in rows]
    e=[f(r,"worked5_acc_e_m") for r in rows]
    wins=[]; j=0
    for i in range(len(rows)):
        j=max(j,i+1)
        while j<len(rows) and t[j]-t[i]<a.window_s: j+=1
        if j>=len(rows): break
        vals=(t[i],t[j],n[i],e[i],n[j],e[j])
        if not all(math.isfinite(v) for v in vals): continue
        wins.append((math.hypot(n[j]-n[i],e[j]-e[i]),t[i],t[j],i,j))
    top=sorted(wins,reverse=True)[:a.top]
    print("===== TOP WINDOWS =====")
    for mag,t0,t1,i,j in top:
        print(f"{mag*1000:8.2f} mm  {t0:.6f} -> {t1:.6f}  rows {i}->{j}")
    spans=sorted((x[1],x[2]) for x in top)
    clusters=[]
    for lo,hi in spans:
        if not clusters or lo-clusters[-1][1]>a.cluster_gap_s: clusters.append([lo,hi])
        else: clusters[-1][1]=max(clusters[-1][1],hi)
    # Расширяем каждый seed-кластер до границ покой->движение->покой.
    # Известные 480 мм здесь намеренно нигде не используются.
    edge=[]
    j=0
    for i in range(len(rows)):
        j=max(j,i+1)
        while j<len(rows) and t[j]-t[i]<a.edge_window_s:
            j+=1
        if j>=len(rows): break
        vals=(n[i],e[i],n[j],e[j])
        if all(math.isfinite(v) for v in vals):
            edge.append((i,j,t[i],t[j],math.hypot(n[j]-n[i],e[j]-e[i])))

    def motion_bounds(seed_lo,seed_hi):
        overlap=[q for q,x in enumerate(edge) if x[2]<=seed_hi and x[3]>=seed_lo]
        if not overlap: return None
        left=min(overlap); right=max(overlap)
        quiet=0; q=left-1
        while q>=0:
            quiet=quiet+1 if edge[q][4]<a.edge_threshold_mm*1e-3 else 0
            if quiet>=a.quiet_windows: break
            left=q; q-=1
        quiet=0; q=right+1
        while q<len(edge):
            quiet=quiet+1 if edge[q][4]<a.edge_threshold_mm*1e-3 else 0
            if quiet>=a.quiet_windows: break
            right=q; q+=1
        i0=edge[left][0]; i1=edge[right][1]
        return i0,i1,t[i0],t[i1]

    print("\n===== TOP-WINDOW CLUSTERS =====")
    for k,(lo,hi) in enumerate(clusters,1):
        peak=max((x[0] for x in top if x[1]>=lo and x[2]<=hi),default=0.0)
        print(f"cluster{k}: {lo:.6f} -> {hi:.6f}  peak_{a.window_s:g}s={peak*1000:.2f} mm")
    print("\n===== MOTION BOUNDS (W5 ONLY; NO 480-mm FIT) =====")
    for k,(lo,hi) in enumerate(clusters,1):
        b=motion_bounds(lo,hi)
        if b is None:
            print(f"cluster{k}: unavailable")
            continue
        i0,i1,t0,t1=b
        dn=n[i1]-n[i0]; de=e[i1]-e[i0]
        print(f"cluster{k}: {t0:.6f} -> {t1:.6f} rows {i0}->{i1} "
              f"dN={dn*1000:.2f}mm dE={de*1000:.2f}mm "
              f"vec={math.hypot(dn,de)*1000:.2f}mm")
    print("\n===== W5 POSITION AROUND SEEDS =====")
    # Показываем абсолютную W5-позицию вокруг каждого переноса. Это позволяет
    # увидеть плато до/после движения и проверить, не обрезал ли edge detector
    # медленные участки. Никакой физической длины здесь также нет.
    for k,(lo,hi) in enumerate(clusters,1):
        print(f"cluster{k}:")
        for off in (-5,-3,-2,-1,0,1,2,3,5):
            target=(lo if off<0 else hi)+off
            idx=min(range(len(t)),key=lambda q:abs(t[q]-target))
            print(f"  t={t[idx]:.6f} off={off:+d}s "
                  f"N={n[idx]*1000:.2f}mm E={e[idx]*1000:.2f}mm "
                  f"h={f(rows[idx],'worked5_hcam_m')*1000:.1f}mm "
                  f"valid={rows[idx].get('worked5_valid','')}")
if __name__=="__main__": main()
