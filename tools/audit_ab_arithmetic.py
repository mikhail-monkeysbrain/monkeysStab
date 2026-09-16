#!/usr/bin/env python3
# Independent arithmetic audit. Deliberately does NOT import analyze_ab_only.py.
import csv, math, statistics, sys
fn=sys.argv[1]; gt=float(sys.argv[2])
with open(fn,newline="") as f: rows=list(csv.DictReader(f))
A=next(i for i,r in enumerate(rows) if r.get("return_event")=="1")
B=next(i for i,r in enumerate(rows) if r.get("return_event")=="2")
seg=rows[A:B]
def f(r,k):
    try:return float(r[k])
    except:return None
def audit(label,vcol,xcol,ycol):
    X=Y=angx=angy=dts=hs=0.; n=0; ranges=[]; heights=[]
    for r in seg:
        if r.get("valid")!="1": continue
        dt=f(r,"dt_s"); rng=f(r,"range_to_fc_m")
        if dt is None or rng is None or not 0<dt<.2: continue
        if vcol and r.get(vcol)!="1": continue
        x=f(r,xcol); y=f(r,ycol)
        if x is None or y is None: continue
        h=rng+0.005 # exact current C++ geometry: lm-(0.050-0.055)
        X+=x*h*dt; Y+=y*h*dt
        angx+=x*dt; angy+=y*dt
        dts+=dt; hs+=h*dt; ranges.append(rng); heights.append(h); n+=1
    mag=1000*math.hypot(X,Y); amag=math.hypot(angx,angy)
    print(label)
    print(f" rows={n} dt_sum={dts:.9f}s")
    print(f" range min/med/max={min(ranges):.6f}/{statistics.median(ranges):.6f}/{max(ranges):.6f} m")
    print(f" hcam min/med/max={min(heights):.6f}/{statistics.median(heights):.6f}/{max(heights):.6f} m")
    print(f" integral angular X/Y=({angx:+.9f},{angy:+.9f}) rad mag={amag:.9f}")
    print(f" metric X/Y=({X*1000:+.3f},{Y*1000:+.3f}) mm mag={mag:.3f}")
    print(f" GT error={mag-gt:+.3f} mm ({100*(mag-gt)/gt:+.3f}%)")
    print()
print("="*72)
print("INDEPENDENT A->B ARITHMETIC AUDIT")
print("file:",fn); print("GT:",gt,"mm")
print("A row/frame",A,rows[A].get("frame"),"B row/frame",B,rows[B].get("frame"))
print("segment rows",len(seg))
print("Formula copied from C++ canonical integration:")
print(" hcam = range_to_fc_m - (camera_z-range_z) = range_to_fc_m + 0.005")
print(" dX = flow_body_x*hcam*dt ; dY = flow_body_y*hcam*dt")
print("="*72)
audit("NATIVE",None,"flow_body_x","flow_body_y")
audit("FB","ab_fb_valid","ab_fb_flow_body_x","ab_fb_flow_body_y")
audit("ROBUST","ab_robust_valid","ab_robust_flow_body_x","ab_robust_flow_body_y")
