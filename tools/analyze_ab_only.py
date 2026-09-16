#!/usr/bin/env python3
import csv, math, sys

fn=sys.argv[1]
gt=float(sys.argv[2])
with open(fn,newline="") as f:
    rows=list(csv.DictReader(f))

def event_index(code):
    for i,r in enumerate(rows):
        if r.get("return_event","0")==str(code):
            return i
    raise SystemExit(f"ОШИБКА: return_event={code} не найден")

a=event_index(1); b=event_index(2)
seg=rows[a:b]

def F(r,*names):
    for n in names:
        if n in r and r[n] not in ("",None):
            try:return float(r[n])
            except:pass
    return None

native=[0.,0.]; fb=[0.,0.]
n_native=n_fb=0
dt_native=dt_fb=0.
for r in seg:
    if r.get("valid")!="1": continue
    dt=F(r,"dt_s")
    fx=F(r,"flow_body_x"); fy=F(r,"flow_body_y")
    rng=F(r,"range_to_fc_m","range_m","luna_m")
    if dt is None or fx is None or fy is None or rng is None or not (0<dt<.2):
        continue
    # Current mount: camera Z=.050, TF-Luna Z=.055 (body FRD).
    # Existing C++ canonical metric uses hcam = range-(cam_z-range_z)=range+0.005.
    h=rng+0.005
    native[0]+=fx*h*dt; native[1]+=fy*h*dt
    n_native+=1; dt_native+=dt

    fbok=r.get("fb_shadow_valid","0")
    fbx=F(r,"fb_flow_body_x"); fby=F(r,"fb_flow_body_y")
    if fbok=="1" and fbx is not None and fby is not None:
        fb[0]+=fbx*h*dt; fb[1]+=fby*h*dt
        n_fb+=1; dt_fb+=dt

def report(name,v,n,dt):
    mag=1000*math.hypot(*v); err=mag-gt
    print(name)
    print(f"  X/Y=({v[0]*1000:+.3f}, {v[1]*1000:+.3f}) mm")
    print(f"  magnitude={mag:.3f} mm")
    print(f"  error={err:+.3f} mm ({100*err/gt:+.3f} %)")
    print(f"  accepted={n}, dt_sum={dt:.6f} s")

print("======================================================================")
print("A->B ONLY OFFLINE RESULT")
print(f"CSV: {fn}")
print(f"GT: {gt:.3f} mm")
print(f"A row/frame: {a} / {rows[a].get('frame','?')}")
print(f"B row/frame: {b} / {rows[b].get('frame','?')}")
print("======================================================================")
report("NATIVE production flow_body",native,n_native,dt_native)
print("----------------------------------------------------------------------")
if n_fb:
    report("FB SHADOW <= configured threshold",fb,n_fb,dt_fb)
    print(f"  FB interval coverage={100*n_fb/max(1,n_native):.2f} % of native accepted rows")
else:
    print("FB SHADOW: в CSV нет валидных fb_shadow samples")
print("======================================================================")
