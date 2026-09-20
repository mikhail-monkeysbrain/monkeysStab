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
    print("\n===== TOP-WINDOW CLUSTERS =====")
    for k,(lo,hi) in enumerate(clusters,1):
        peak=max((x[0] for x in top if x[1]>=lo and x[2]<=hi),default=0.0)
        print(f"cluster{k}: {lo:.6f} -> {hi:.6f}  peak_{a.window_s:g}s={peak*1000:.2f} mm")
if __name__=="__main__": main()
