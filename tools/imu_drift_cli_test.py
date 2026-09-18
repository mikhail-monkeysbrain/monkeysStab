#!/usr/bin/env python3
import argparse, json, math, time, urllib.request

def get(url):
    with urllib.request.urlopen(url, timeout=1.0) as r:
        return json.load(r)

def snap(t, elapsed=0.0):
    an=float(t.get("imu_dr_acc_n") or 0); ae=float(t.get("imu_dr_acc_e") or 0); ad=float(t.get("imu_dr_acc_d") or 0)
    return dict(
        t=elapsed, a=math.sqrt(an*an+ae*ae+ad*ad),
        g=float(t.get("imu_dr_gmag") or 0),
        an=an, ae=ae, ad=ad,
        roll=float(t.get("roll_deg") or 0), pitch=float(t.get("pitch_deg") or 0),
        vn=float(t.get("imu_dr_vn") or 0), ve=float(t.get("imu_dr_ve") or 0), vd=float(t.get("imu_dr_vd") or 0),
        x=float(t.get("imu_dr_n_mm") or 0), y=float(t.get("imu_dr_e_mm") or 0), z=float(t.get("imu_dr_d_mm") or 0),
        st=int(t.get("imu_dr_stationary_samples") or 0),
        stationary=bool(t.get("imu_dr_stationary",False)),
        acc_ok=bool(t.get("imu_dr_acc_ok",False)), gyro_ok=bool(t.get("imu_dr_gyro_ok",False)),
    )

def avg(vs,k):
    return sum(v[k] for v in vs)/len(vs) if vs else 0.0

def dv(a,b,k):
    return b[k]-a[k]

def collect(url, seconds, label, stop_at=None):
    print(f"\n{label}  ({seconds:.0f} с)")
    vs=[]; t0=time.monotonic(); next_print=t0; stop_printed=False
    while True:
        now=time.monotonic(); elapsed=now-t0
        if elapsed>=seconds: break
        if stop_at is not None and elapsed>=stop_at and not stop_printed:
            print("\r\aСТОП — НЕ ДВИГАТЬ!                 ")
            stop_printed=True
        try:
            vs.append(snap(get(url),elapsed))
        except Exception:
            pass
        if now>=next_print:
            if stop_at is not None and elapsed<stop_at:
                msg=f"  ДВИЖЕНИЕ: осталось {max(0,math.ceil(stop_at-elapsed)):2d} с"
            elif stop_at is not None:
                msg=f"  ПОКОЙ: осталось {max(0,math.ceil(seconds-elapsed)):2d} с"
            else:
                msg=f"  осталось {max(0,math.ceil(seconds-elapsed)):2d} с"
            print("\r"+msg+" "*12,end="",flush=True)
            next_print=now+1
        time.sleep(.05)
    print("\r  готово                              ")
    return vs

ap=argparse.ArgumentParser(description="Canonical IMU DR move-stop drift test")
ap.add_argument("--url",default="http://127.0.0.1:8080/api/telemetry")
ap.add_argument("--rest-before",type=float,default=5)
ap.add_argument("--move",type=float,default=3)
ap.add_argument("--rest-after",type=float,default=12)
a=ap.parse_args()

try:
    first=get(a.url)
except Exception as e:
    raise SystemExit(f"Нет телеметрии: {e}")
if not first.get("imu_dr_calibrated"):
    raise SystemExit("IMU DR не откалиброван. Выполни физический RC HOME и дождись калибровки.")

print("IMU DR — MOVE/STOP")
print("==================")
print("Стенд неподвижен. Перед тестом нужен физический RC HOME.")
input("После HOME и полной остановки нажми ENTER... ")

ready=get(a.url)
if not ready.get("imu_dr_calibrated"):
    raise SystemExit("После HOME калибровка ещё не завершена.")
p0=collect(a.url,a.rest_before,"ИСХОДНЫЙ ПОКОЙ")
start=p0[-1] if p0 else snap(ready)
if start["st"]<10:
    raise SystemExit(f"ZUPT не захвачен перед движением (stat={start['st']}). Стенд не двигать; повторить после устойчивого покоя.")

print("\nПлавно перемести стенд на 250–300 мм БЕЗ ОТРЫВА.")
print(f"Двигай первые {a.move:.0f} с. После сигнала СТОП сразу отпусти стенд.")
input("Нажми ENTER и сразу начинай движение... ")

seq=collect(a.url,a.move+a.rest_after,"ДВИЖЕНИЕ → АВТОМАТИЧЕСКИЙ ПОКОЙ",stop_at=a.move)
move=[v for v in seq if v["t"]<a.move]
after=[v for v in seq if v["t"]>=a.move]
stop=min(seq,key=lambda v:abs(v["t"]-a.move)) if seq else start
end=seq[-1] if seq else start
m=max(move,key=lambda v:math.sqrt(v["vn"]**2+v["ve"]**2+v["vd"]**2)) if move else start

zupt=None
for v in after:
    if v["st"]>=10 and abs(v["vn"])<1e-9 and abs(v["ve"])<1e-9 and abs(v["vd"])<1e-9:
        zupt=v["t"]-a.move
        break

post_dx=dv(stop,end,"x"); post_dy=dv(stop,end,"y"); post_dz=dv(stop,end,"z")
move_dx=dv(start,stop,"x"); move_dy=dv(start,stop,"y"); move_dz=dv(start,stop,"z")
total_dx=dv(start,end,"x"); total_dy=dv(start,end,"y"); total_dz=dv(start,end,"z")

print("\nРЕЗУЛЬТАТ")
print("=========")
print(f"Покой до: |a|={avg(p0,'a'):.4f} m/s²  stat={start['st']}  V=({start['vn']:+.3f},{start['ve']:+.3f},{start['vd']:+.3f})")
print(f"Движение: max|V|={math.sqrt(m['vn']**2+m['ve']**2+m['vd']**2):.3f} m/s")
def integ(vs,k):
    s=0.0
    for u,v in zip(vs,vs[1:]):
        dt=v["t"]-u["t"]
        if 0<dt<0.2: s += .5*(u[k]+v[k])*dt
    return s
pre_a=integ(move,"an"); post_a=integ(after[:max(1,next((i for i,v in enumerate(after) if v["st"]>=10),len(after)))],"an")
print(f"Импульс aN: движение={pre_a:+.4f} m/s   после STOP до ZUPT={post_a:+.4f} m/s")
print(f"aN диапазон: движение [{min((v['an'] for v in move),default=0):+.3f},{max((v['an'] for v in move),default=0):+.3f}] m/s²")
print(f"ATTITUDE: roll [{min((v['roll'] for v in seq),default=0):+.2f},{max((v['roll'] for v in seq),default=0):+.2f}]°  pitch [{min((v['pitch'] for v in seq),default=0):+.2f},{max((v['pitch'] for v in seq),default=0):+.2f}]°")
print(f"Δ движение: X={move_dx:+.1f}  Y={move_dy:+.1f}  Z={move_dz:+.1f} mm")
print(f"Δ после STOP: X={post_dx:+.1f}  Y={post_dy:+.1f}  Z={post_dz:+.1f} mm")
print(f"Δ всего:     X={total_dx:+.1f}  Y={total_dy:+.1f}  Z={total_dz:+.1f} mm")
print("ZUPT после STOP:", f"{zupt:.2f} с" if zupt is not None else "НЕ ЗАХВАЧЕН")
print(f"Конец: |a|={avg(after[-40:] if len(after)>40 else after,'a'):.4f} m/s²  stat={end['st']}  V=({end['vn']:+.3f},{end['ve']:+.3f},{end['vd']:+.3f})")
