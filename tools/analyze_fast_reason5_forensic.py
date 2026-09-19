#!/usr/bin/env python3
import csv, math, sys
from collections import Counter

if len(sys.argv) != 2:
    print("usage: analyze_fast_reason5_forensic.py RUN_DIR")
    raise SystemExit(2)

run_dir=sys.argv[1].rstrip("/")
p=run_dir+"/optical_flow_mavlink.csv"

def f(x,k):
    try: return float(x.get(k) or 0)
    except: return 0.0

rows=list(csv.DictReader(open(p,newline="")))
r5=[x for x in rows if int(f(x,"invalid_reason"))==5]
r6=[x for x in rows if int(f(x,"invalid_reason"))==6]

print("===== FAST REASON5 FORENSIC =====")
print("run =",run_dir)
print("rows =",len(rows))
print("reason5 =",len(r5),"reason6 =",len(r6))
print()
print("===== REASON5 ROWS =====")
for x in r5:
    print(
        "frame",int(f(x,"frame")),
        "dt_ms",round(f(x,"dt_s")*1000,3),
        "features",int(f(x,"features")),
        "tracked",int(f(x,"tracked")),
        "inliers",int(f(x,"inliers")),
        "ratio",round(f(x,"inlier_ratio"),3),
        "flow_x",round(f(x,"flow_body_x"),4),
        "flow_y",round(f(x,"flow_body_y"),4),
        "LK_ms",round(f(x,"t_lk_ms"),3),
        "drop",int(f(x,"camera_queue_dropped")),
    )

print()
print("===== REASON5 SUMMARY =====")
if r5:
    ins=[int(f(x,"inliers")) for x in r5]
    dts=[f(x,"dt_s")*1000 for x in r5]
    print("inliers min/max =",min(ins),max(ins))
    print("inliers counts  =",dict(sorted(Counter(ins).items())))
    print("dt min/max ms   =",round(min(dts),3),round(max(dts),3))
    print("near threshold 15..19 =",sum(15 <= i <= 19 for i in ins))
    print("weak 10..14          =",sum(10 <= i <= 14 for i in ins))
    print("very weak <10        =",sum(i < 10 for i in ins))

print()
print("NOTE: reason5 rows cannot be reconstructed metrically from the production CSV alone.")
print("Next instrumentation must preserve their RANSAC inlier point pairs before the <20 return.")
