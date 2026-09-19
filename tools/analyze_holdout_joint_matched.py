#!/usr/bin/env python3
import csv, math, os, sys, statistics

def pct(v,p):
    if not v: return float("nan")
    x=sorted(v); k=(len(x)-1)*p; a=int(math.floor(k)); b=int(math.ceil(k))
    return x[a] if a==b else x[a]*(b-k)+x[b]*(k-a)

def load_rows(run):
    p=os.path.join(run,"stationary_balanced_shadow.csv") if os.path.isdir(run) else run
    out=[]
    with open(p,newline="") as f:
        for r in csv.DictReader(f):
            try:
                if int(float(r["production_valid"]))!=1 or int(float(r["holdout_valid"]))!=1: continue
                dt=float(r["dt_s"]); g=float(r["holdout_g"]); s=abs(float(r["holdout_scale"]))
                if 0<dt<.2 and math.isfinite(g) and math.isfinite(s):
                    out.append((int(float(r["frame"])),g,s))
            except (KeyError,ValueError): pass
    return out

def load_marks(path):
    out=[]
    with open(path,newline="",encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try: out.append((int(float(r["frame"])),r["stage"]))
            except (KeyError,ValueError,TypeError): pass
    return sorted(out)

def assign(rows,marks):
    out=[]; j=0
    for fr,g,s in rows:
        while j+1<len(marks) and abs(marks[j+1][0]-fr)<=abs(marks[j][0]-fr): j+=1
        out.append((marks[j][1],g,s))
    return out

def report(label,z,edges):
    print("\n"+label)
    print("-"*len(label))
    for lo,hi in zip(edges[:-1],edges[1:]):
        q=[(g,s) for _,g,s in z if s>=lo and (s<hi or math.isinf(hi))]
        if not q:
            print(f"|scale| [{lo:.9g},{hi:.9g}): n=0"); continue
        gs=[g for g,_ in q]
        print(f"|scale| [{lo:.9g},{hi:.9g}): n={len(q):5d} G_med={statistics.median(gs):+.6f} G_p25={pct(gs,.25):+.6f} G_p75={pct(gs,.75):+.6f} G_p95={pct(gs,.95):+.6f} G_pos={sum(x>0 for x in gs)/len(gs):.3f}")

if len(sys.argv)!=3:
    print("usage: analyze_holdout_joint_matched.py RUN MARKERS"); raise SystemExit(2)
z=assign(load_rows(sys.argv[1]),load_marks(sys.argv[2]))
rest_names={"LOW-1 ПОКОЙ","HIGH ПОКОЙ","LOW-2 ПОКОЙ"}
move_names={"LOW->HIGH","HIGH->LOW"}
rest=[x for x in z if x[0] in rest_names]
move=[x for x in z if x[0] in move_names]
if not rest or not move: raise SystemExit("missing predefined REST or TRANSITION stages")

# Common bins are derived ONCE from pooled |scale|, never separately by class.
# This prevents comparing different amplitude ranges while selecting no G gate.
pool=[x[2] for x in rest+move]
edges=[0.0,pct(pool,.50),pct(pool,.75),pct(pool,.90),pct(pool,.95),float("inf")]
# Remove accidental duplicate edges.
e=[edges[0]]
for x in edges[1:]:
    if x>e[-1]: e.append(x)
if not math.isinf(e[-1]): e.append(float("inf"))

print("JOINT HOLDOUT: G AT MATCHED |scale|")
print("===================================")
print("Classes are fixed by guided markers: REST=LOW/HIGH plateaus, TRANSITION=LOW->HIGH/HIGH->LOW.")
print("Bins are common pooled |scale| quantiles: 0-50%, 50-75%, 75-90%, 90-95%, 95-100%.")
print("No G threshold is selected.")
report("REST",rest,e)
report("TRANSITION",move,e)

print("\nMATCHED-BIN DELTA (TRANSITION - REST)")
print("-------------------------------------")
for lo,hi in zip(e[:-1],e[1:]):
    a=[g for _,g,s in rest if s>=lo and (s<hi or math.isinf(hi))]
    b=[g for _,g,s in move if s>=lo and (s<hi or math.isinf(hi))]
    if a and b:
        print(f"|scale| [{lo:.9g},{hi:.9g}): dG_med={statistics.median(b)-statistics.median(a):+.6f}  n_rest={len(a)} n_transition={len(b)}")
print("\nDiagnostic only; WORKED5/FC unchanged.")
