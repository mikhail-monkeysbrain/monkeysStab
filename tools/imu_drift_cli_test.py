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
ap.add_argument("--max-test",type=float,default=30)
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

print("\nТеперь двигай стенд руками как удобно, затем полностью останови и убери руки.")
input("Нажми ENTER и начинай движение... ")

print("\nЗАПИСЬ — ожидаю движение и последующий устойчивый покой")
seq=[]; t0=time.monotonic(); moved=False; quiet_since=None; done_reason="таймаут"
while time.monotonic()-t0 < a.max_test:
    elapsed=time.monotonic()-t0
    try:
        v=snap(get(a.url),elapsed); seq.append(v)
    except Exception:
        time.sleep(.05); continue

    speed=math.sqrt(v["vn"]**2+v["ve"]**2+v["vd"]**2)
    dynamic=(not v["stationary"]) or speed>0.01
    if dynamic:
        moved=True
        quiet_since=None
    elif moved:
        if quiet_since is None: quiet_since=elapsed
        if elapsed-quiet_since >= 2.0 and v["st"]>=10:
            done_reason="устойчивый конечный покой"
            break
    time.sleep(.05)
print(f"  запись завершена: {done_reason}")

if not moved:
    raise SystemExit("Движение не обнаружено.")
end=seq[-1]
# Конечный покой определяется данными, а не таймером/реакцией оператора.
q0=next((i for i in range(len(seq)) if all(x["stationary"] for x in seq[i:i+10]) and len(seq[i:i+10])==10 and seq[i]["t"]>0.2),None)
# Первый такой участок может быть паузой до начала движения; ищем последний переход в устойчивый покой после динамики.
candidates=[]
for i in range(1,len(seq)-9):
    if (not seq[i-1]["stationary"]) and all(x["stationary"] for x in seq[i:i+10]):
        candidates.append(i)
settle_i=candidates[-1] if candidates else max(0,len(seq)-1)
settle=seq[settle_i]
move_end=settle
m=max(seq[:settle_i+1],key=lambda v: math.sqrt(v["vn"]**2+v["ve"]**2+v["vd"]**2))
total_dx=dv(start,end,"x"); total_dy=dv(start,end,"y"); total_dz=dv(start,end,"z")
settle_dx=dv(start,settle,"x"); settle_dy=dv(start,settle,"y"); settle_dz=dv(start,settle,"z")

print("\nРЕЗУЛЬТАТ")
print("=========")
print(f"Покой до: |a|={avg(p0,'a'):.4f} m/s²  stat={start['st']}  V=({start['vn']:+.3f},{start['ve']:+.3f},{start['vd']:+.3f})")
print(f"Запись: {len(seq)} samples, {end['t']:.2f} с; max|V|={math.sqrt(m['vn']**2+m['ve']**2+m['vd']**2):.3f} m/s")
print(f"Устойчивый конечный покой с t={settle['t']:.2f} с, stat={settle['st']}")
print(f"Δ к покою: X={settle_dx:+.1f}  Y={settle_dy:+.1f}  Z={settle_dz:+.1f} mm")
print(f"Δ всего:   X={total_dx:+.1f}  Y={total_dy:+.1f}  Z={total_dz:+.1f} mm")
print(f"Конец: |a|={avg(seq[-40:] if len(seq)>40 else seq,'a'):.4f} m/s²  stat={end['st']}  V=({end['vn']:+.3f},{end['ve']:+.3f},{end['vd']:+.3f})")
