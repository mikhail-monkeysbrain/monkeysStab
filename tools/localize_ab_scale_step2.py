#!/usr/bin/env python3
# Step 2: localize A->B scale loss using only the existing CSV.
# No tuning and no production changes.
import csv, math, statistics, sys
fn=sys.argv[1]; gt=float(sys.argv[2])
with open(fn,newline="") as f: rows=list(csv.DictReader(f))
A=next(i for i,r in enumerate(rows) if r.get("return_event")=="1")
B=next(i for i,r in enumerate(rows) if r.get("return_event")=="2")
seg=rows[A:B]
def F(r,k):
    try:return float(r[k])
    except:return None
def integ(xcol,ycol,validcol=None):
    X=Y=AX=AY=0.; n=0; hs=[]
    for r in seg:
        if r.get("valid")!="1": continue
        if validcol and r.get(validcol)!="1": continue
        dt=F(r,"dt_s"); rng=F(r,"range_to_fc_m"); x=F(r,xcol); y=F(r,ycol)
        if None in (dt,rng,x,y) or not 0<dt<.2: continue
        h=rng+0.005
        X+=x*h*dt; Y+=y*h*dt; AX+=x*dt; AY+=y*dt; hs.append(h); n+=1
    return n,math.hypot(X,Y),math.hypot(AX,AY),statistics.median(hs) if hs else float("nan")
for name,v,x,y in [
 ("NATIVE",None,"flow_body_x","flow_body_y"),
 ("FB","ab_fb_valid","ab_fb_flow_body_x","ab_fb_flow_body_y"),
 ("ROBUST","ab_robust_valid","ab_robust_flow_body_x","ab_robust_flow_body_y")]:
    n,m,a,h=integ(x,y,v)
    print(f"{name:8s} n={n:3d} metric={m*1000:8.3f} mm angular={a:9.6f} rad h={h*1000:7.3f} mm")
    if a>0:
        print(f"         h_required_if_angular_correct = {gt/1000/a*1000:.3f} mm")
        print(f"         angular_required_if_height_correct = {gt/1000/h:.6f} rad")
        print(f"         missing_scale = {gt/(m*1000):.6f}x")
print()
# Production 4-param fit diagnostics already logged per frame.
for k in ("scale_rate","yaw_rate_cam_z"):
    vals=[F(r,k) for r in seg if r.get("valid")=="1" and F(r,k) is not None]
    if vals:
        print(f"{k}: median={statistics.median(vals):+.6f} min={min(vals):+.6f} max={max(vals):+.6f}")
# Spatial 3x3 flow diagnostic: if present, integrate each cell independently.
print("\n3x3 CELL INTEGRALS (same accepted production frames; diagnostic medians):")
found=False
for ci in range(9):
    nx=f"cell{ci}_n"; xx=f"cell{ci}_body_x"; yy=f"cell{ci}_body_y"
    if nx not in rows[0] or xx not in rows[0] or yy not in rows[0]: continue
    X=Y=0.; used=0
    for r in seg:
        if r.get("valid")!="1": continue
        dt=F(r,"dt_s"); rng=F(r,"range_to_fc_m"); n=F(r,nx); x=F(r,xx); y=F(r,yy)
        if None in (dt,rng,n,x,y) or n<3 or not 0<dt<.2: continue
        h=rng+0.005; X+=x*h*dt; Y+=y*h*dt; used+=1
    if used:
        found=True; print(f" cell{ci}: {1000*math.hypot(X,Y):8.3f} mm over {used} frames")
if not found: print(" cell columns are not present in this CSV")
print("\nINTERPRETATION:")
print(" - required height near measured height => metric-height branch remains plausible.")
print(" - large disagreement among cell integrals => planar/global fit or scene observability is suspect.")
print(" - uniform cell deficit => investigate pixel->normalized K/D conversion or common scale, not FB/Huber.")
