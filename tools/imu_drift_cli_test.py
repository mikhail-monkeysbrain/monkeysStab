#!/usr/bin/env python3
import argparse, json, math, time, urllib.request

def get(url):
    with urllib.request.urlopen(url,timeout=1.0) as r: return json.load(r)

def snap(t):
    an=float(t.get("imu_dr_acc_n") or 0); ae=float(t.get("imu_dr_acc_e") or 0); ad=float(t.get("imu_dr_acc_d") or 0)
    return dict(a=math.sqrt(an*an+ae*ae+ad*ad), an=an,ae=ae,ad=ad,
      vn=float(t.get("imu_dr_vn") or 0),ve=float(t.get("imu_dr_ve") or 0),vd=float(t.get("imu_dr_vd") or 0),
      x=float(t.get("imu_dr_n_mm") or 0),y=float(t.get("imu_dr_e_mm") or 0),z=float(t.get("imu_dr_d_mm") or 0),
      st=int(t.get("imu_dr_stationary_samples") or 0))

def avg(vs,k): return sum(v[k] for v in vs)/len(vs) if vs else 0.0
def phase(url,seconds,label):
    print(f"\n{label}  ({seconds:.0f} с)")
    vs=[]; t0=time.monotonic(); nxt=t0
    while time.monotonic()-t0<seconds:
        try: vs.append(snap(get(url)))
        except Exception as e: print(f"\r  ошибка телеметрии: {e}",end="",flush=True)
        now=time.monotonic()
        if now>=nxt:
            left=max(0,math.ceil(seconds-(now-t0)))
            print(f"\r  осталось {left:2d} с",end="",flush=True); nxt=now+1
        time.sleep(.05)
    print("\r  готово        ")
    return vs

ap=argparse.ArgumentParser(description="Guided IMU drift test via monkeysStab Web telemetry")
ap.add_argument("--url",default="http://127.0.0.1:8080/api/telemetry")
ap.add_argument("--rest-before",type=float,default=5)
ap.add_argument("--move",type=float,default=3)
ap.add_argument("--rest-after",type=float,default=12)
a=ap.parse_args()

try:
    t=get(a.url)
except Exception as e:
    raise SystemExit(f"Нет телеметрии: {e}")
if not t.get("imu_dr_calibrated"):
    raise SystemExit("IMU DR ещё не откалиброван. Дождись imu_dr_calibrated=true.")

print("IMU DR — тест дрейфа")
print("====================")
print("1. Стенд неподвижен.")
input("Нажми ENTER для начала... ")

p0=phase(a.url,a.rest_before,"ПОКОЙ ДО ДВИЖЕНИЯ")
print("\n2. Плавно перемести стенд на 250–300 мм БЕЗ ОТРЫВА.")
input("Положи руку на стенд и нажми ENTER — сразу начинай движение... ")
pm=phase(a.url,a.move,"ДВИЖЕНИЕ")
print("\n3. ОСТАНОВИ стенд и больше не трогай.")
input("Когда стенд полностью остановлен, нажми ENTER... ")
p1=phase(a.url,a.rest_after,"ПОКОЙ ПОСЛЕ ДВИЖЕНИЯ")

allv=p0+pm+p1
m=max(pm,key=lambda v: math.sqrt(v["vn"]**2+v["ve"]**2+v["vd"]**2)) if pm else snap(t)
end=p1[-1] if p1 else m
start=p0[-1] if p0 else snap(t)

print("\nРЕЗУЛЬТАТ")
print("=========")
print(f"Покой до:   |a|={avg(p0,'a'):.4f} m/s²   V=({start['vn']:+.3f},{start['ve']:+.3f},{start['vd']:+.3f}) m/s   stat={start['st']}")
print(f"Движение:   max|V|={math.sqrt(m['vn']**2+m['ve']**2+m['vd']**2):.3f} m/s   V=({m['vn']:+.3f},{m['ve']:+.3f},{m['vd']:+.3f})")
print(f"Покой после:|a|={avg(p1[-40:] if len(p1)>40 else p1,'a'):.4f} m/s²   V=({end['vn']:+.3f},{end['ve']:+.3f},{end['vd']:+.3f}) m/s   stat={end['st']}")
print(f"IMU position: X={end['x']:+.1f} mm  Y={end['y']:+.1f} mm  Z={end['z']:+.1f} mm")
