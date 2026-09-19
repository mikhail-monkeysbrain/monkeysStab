#!/usr/bin/env python3
import csv, json, os, signal, socket, subprocess, time, urllib.request
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
RUNS=Path("/home/vio/monkeysStab_runs")
URL="http://127.0.0.1:8080/api/telemetry"
SERVICE_LOG=ROOT/"scale_height_guided_service.log"
MARKERS=ROOT/"scale_height_guided_markers.csv"

def get():
    with urllib.request.urlopen(URL,timeout=0.8) as r: return json.load(r)

def busy(port):
    s=socket.socket(); s.settimeout(.2)
    try: return s.connect_ex(("127.0.0.1",port))==0
    finally: s.close()

def newest_run_after(t):
    if not RUNS.exists(): return None
    a=[p for p in RUNS.iterdir() if p.is_dir() and p.stat().st_mtime>=t-5]
    return max(a,key=lambda p:p.stat().st_mtime) if a else None

def mark(rows,stage,t0):
    try:
        d=get()
    except Exception:
        d={}
    rows.append({"wall_time":time.time(),"elapsed_s":time.monotonic()-t0,
                 "stage":stage,"range_m":d.get("range_m"),"frame":d.get("frame")})

def countdown(label,seconds,rows,t0):
    start=time.monotonic()
    while True:
        e=time.monotonic()-start
        if e>=seconds: break
        mark(rows,label,t0)
        print(f"\r{label:<18} {e:4.1f}/{seconds}s  осталось {seconds-e:4.1f}s",end="",flush=True)
        time.sleep(.1)
    print()
    mark(rows,label,t0)

print("\033[2J\033[H",end="")
print("JT-Zero — GUIDED SCALE HEIGHT diagnostic")
print("========================================")
print("Профиль: LOW 10s -> ПОДЪЁМ 5s -> HIGH 10s -> ОПУСКАНИЕ 5s -> LOW 10s")
print("На LOW/HIGH аппарат должен стоять сам. WORKED5/FUSED-V2 не изменяются.\n")
if busy(8080) or busy(5760):
    raise SystemExit("ОШИБКА: старый Web/router ещё работает. Останови его и повтори.")

start_wall=time.time()
log=SERVICE_LOG.open("w",encoding="utf-8")
env=os.environ.copy(); env["MONKEYS_LOCAL_GUI"]="0"; env["MONKEYS_WEB_TELEMETRY_UDP_PORT"]="8766"
proc=subprocess.Popen(["bash",str(ROOT/"scripts"/"run_web.sh")],cwd=ROOT,env=env,
                      stdout=log,stderr=subprocess.STDOUT,text=True)
markers=[]
try:
    print("Запуск router + Web + runtime...")
    t=time.monotonic(); ready=False; state="WAIT"
    while time.monotonic()-t<90:
        if proc.poll() is not None: raise RuntimeError(f"service exited {proc.returncode}; {SERVICE_LOG}")
        try:
            d=get(); state=f"fresh={d.get('imu_cam_fresh')} speed={d.get('imu_cam_speed')} frame={d.get('frame')}"
            if d.get("imu_cam_fresh") is True and d.get("imu_cam_speed") is not None:
                ready=True; break
        except Exception as e: state=f"{type(e).__name__}: {e}"
        print(f"\rПОДГОТОВКА {time.monotonic()-t:4.1f}/90s | {state}",end="",flush=True)
        time.sleep(.25)
    print()
    if not ready: raise RuntimeError(f"telemetry not ready: {state}")
    print("СИСТЕМА ГОТОВА")
    input("\nLOW: аппарат устойчиво стоит на столе, руки убраны. Нажми Enter... ")

    t0=time.monotonic()
    countdown("LOW-1 ПОКОЙ",10,markers,t0)
    print("\aСЕЙЧАС: плавно подними ВЕСЬ аппарат и поставь на проставки.")
    countdown("LOW->HIGH",5,markers,t0)
    print("\aHIGH: руки убрать, аппарат должен стоять сам.")
    countdown("HIGH ПОКОЙ",10,markers,t0)
    print("\aСЕЙЧАС: плавно сними ВЕСЬ аппарат с проставок и поставь на стол.")
    countdown("HIGH->LOW",5,markers,t0)
    print("\aLOW: руки убрать, аппарат снова стоит сам.")
    countdown("LOW-2 ПОКОЙ",10,markers,t0)
finally:
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired: proc.kill()
    log.close()

if markers:
    with MARKERS.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=list(markers[0].keys())); w.writeheader(); w.writerows(markers)

run=newest_run_after(start_wall)
print("\nРЕЗУЛЬТАТ")
print("Run:",run if run else "НЕ НАЙДЕН")
print("Markers:",MARKERS)
if run:
    print("\nЗапусти:")
    print(f"python3 tools/analyze_scale_height_guided.py '{run}' '{MARKERS}'")
