#!/usr/bin/env python3
import csv, math, sys
fn=sys.argv[1]; gt=float(sys.argv[2])
with open(fn,newline="") as f: rows=list(csv.DictReader(f))
def event(code):
    return next(i for i,r in enumerate(rows) if r.get("return_event","0")==str(code))
a,b=event(1),event(2); seg=rows[a:b]
def F(r,*ns):
    for n in ns:
        try:
            if r.get(n,"")!="": return float(r[n])
        except ValueError: pass
    return None
acc={k:{"v":[0.,0.],"n":0,"dt":0.} for k in ("NATIVE production flow_body","FB SHADOW","ROBUST/HUBER SHADOW")}
fb_checked=fb_pass=fb_inliers=0; invalid={}
for r in seg:
    if r.get("valid")!="1":
        q=r.get("invalid_reason","?"); invalid[q]=invalid.get(q,0)+1; continue
    dt=F(r,"dt_s"); rng=F(r,"range_to_fc_m","luna_m")
    if dt is None or rng is None or not 0<dt<.2: continue
    h=rng+0.005
    def add(k,xn,yn,ok=True):
        if not ok:return
        x,y=F(r,xn),F(r,yn)
        if x is None or y is None:return
        z=acc[k]; z["v"][0]+=x*h*dt; z["v"][1]+=y*h*dt; z["n"]+=1; z["dt"]+=dt
    add("NATIVE production flow_body","flow_body_x","flow_body_y")
    add("FB SHADOW","ab_fb_flow_body_x","ab_fb_flow_body_y",r.get("ab_fb_valid")=="1")
    add("ROBUST/HUBER SHADOW","ab_robust_flow_body_x","ab_robust_flow_body_y",r.get("ab_robust_valid")=="1")
    fb_checked+=int(F(r,"ab_fb_checked") or 0); fb_pass+=int(F(r,"ab_fb_pass") or 0); fb_inliers+=int(F(r,"ab_fb_inliers") or 0)
def report(k,z):
    if not z["n"]: print(k+": NO VALID SAMPLES"); return
    m=1000*math.hypot(*z["v"]); e=m-gt
    print(k); print(f"  X/Y=({z['v'][0]*1000:+.3f}, {z['v'][1]*1000:+.3f}) mm")
    print(f"  magnitude={m:.3f} mm"); print(f"  error={e:+.3f} mm ({100*e/gt:+.3f} %)")
    print(f"  accepted={z['n']}, dt_sum={z['dt']:.6f} s")
print("="*70); print("A->B ONLY OFFLINE RESULT"); print("CSV:",fn); print(f"GT: {gt:.3f} mm")
print(f"A row/frame: {a} / {rows[a].get('frame','?')}"); print(f"B row/frame: {b} / {rows[b].get('frame','?')}"); print("="*70)
for k,z in acc.items(): report(k,z); print("-"*70)
nn=acc["NATIVE production flow_body"]["n"]
print("SHADOW COVERAGE / GATES")
for k in ("FB SHADOW","ROBUST/HUBER SHADOW"):
    print(f"  {k}: {acc[k]['n']}/{nn} ({100*acc[k]['n']/max(1,nn):.2f} %)")
print(f"  FB tracks checked/pass/inliers totals: {fb_checked}/{fb_pass}/{fb_inliers}")
print("  invalid production reasons:",invalid); print("="*70)
