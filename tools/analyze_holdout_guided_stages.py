#!/usr/bin/env python3
import csv, math, os, sys, statistics

def pct(v,p):
    if not v:return float("nan")
    x=sorted(v); k=(len(x)-1)*p; a=int(math.floor(k)); b=int(math.ceil(k))
    return x[a] if a==b else x[a]*(b-k)+x[b]*(k-a)

def load_markers(path):
    rows=[]
    with open(path,newline="",encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try: rows.append((float(r["wall_time"]),r["stage"]))
            except (KeyError,ValueError): pass
    if not rows: raise SystemExit("markers empty")
    # Marker stage order is fixed by guided protocol; wall_time gives exact
    # stage windows independently of holdout result.
    out={}
    for t,s in rows:
        if s not in out: out[s]=[t,t]
        else: out[s][1]=t
    return out

def load_run(run):
    p=os.path.join(run,"stationary_balanced_shadow.csv") if os.path.isdir(run) else run
    st=os.stat(p)
    data=[]
    with open(p,newline="") as f:
        rr=csv.DictReader(f)
        for r in rr:
            try:
                if int(float(r["production_valid"]))!=1 or int(float(r["holdout_valid"]))!=1: continue
                data.append({
                    "mono":int(r["mono_ns"]),"g":float(r["holdout_g"]),
                    "e0":float(r["holdout_e0"]),"e1":float(r["holdout_e1"]),
                    "scale":float(r["holdout_scale"]),"dt":float(r["dt_s"])
                })
            except (KeyError,ValueError): pass
    if not data: raise SystemExit("no valid holdout rows")
    # Align monotonic CSV time to marker wall time using file/run timing:
    # guided markers begin after readiness; map relative monotonic seconds from
    # first valid CSV row to run file mtime interval is unsafe. Instead markers
    # include telemetry frame, but shadow CSV has frame too; reload frame map.
    return p,data

def frame_stage_map(markers_path):
    pts=[]
    with open(markers_path,newline="",encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                fr=int(float(r["frame"])); st=r["stage"]
                pts.append((fr,st))
            except (KeyError,ValueError,TypeError): pass
    return pts

def load_by_frame(run):
    p=os.path.join(run,"stationary_balanced_shadow.csv") if os.path.isdir(run) else run
    out=[]
    with open(p,newline="") as f:
        for r in csv.DictReader(f):
            try:
                if int(float(r["production_valid"]))!=1 or int(float(r["holdout_valid"]))!=1: continue
                dt=float(r["dt_s"]); vals=[float(r[x]) for x in ("holdout_g","holdout_e0","holdout_e1","holdout_scale")]
                if 0<dt<.2 and all(map(math.isfinite,vals)):
                    out.append((int(float(r["frame"])),*vals))
            except (KeyError,ValueError): pass
    return out

def report(name,z):
    print(f"\n{name}: n={len(z)}")
    if not z:return
    g=[x[1] for x in z]; sc=[abs(x[4]) for x in z]
    print(f"G median={statistics.median(g):+.6f} p05={pct(g,.05):+.6f} p25={pct(g,.25):+.6f} p75={pct(g,.75):+.6f} p95={pct(g,.95):+.6f}")
    print(f"G positive fraction={sum(x>0 for x in g)/len(g):.4f}")
    print(f"|scale| median={statistics.median(sc):.9g} p95={pct(sc,.95):.9g}")

if len(sys.argv)!=3:
    print("usage: analyze_holdout_guided_stages.py RUN MARKERS")
    raise SystemExit(2)
rows=load_by_frame(sys.argv[1]); marks=frame_stage_map(sys.argv[2])
if not marks: raise SystemExit("no marker frames")
# Assign each frame to nearest marker sample's stage. Markers are ~10 Hz and
# cover every predefined stage, so midpoint boundaries stay independent of G.
marks.sort()
stages=["LOW-1 ПОКОЙ","LOW->HIGH","HIGH ПОКОЙ","HIGH->LOW","LOW-2 ПОКОЙ"]
groups={s:[] for s in stages}
j=0
for row in rows:
    fr=row[0]
    while j+1<len(marks) and abs(marks[j+1][0]-fr)<=abs(marks[j][0]-fr): j+=1
    st=marks[j][1]
    if st in groups: groups[st].append(row)
print("SPATIAL HOLDOUT BY PREDEFINED GUIDED STAGE")
print("==========================================")
for s in stages: report(s,groups[s])
print("\nStage boundaries come only from guided markers; no G threshold is selected.")
