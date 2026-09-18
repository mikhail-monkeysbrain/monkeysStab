#!/usr/bin/env python3
import csv, json, os, signal, socket, subprocess, sys, time, urllib.request
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
URL="http://127.0.0.1:8080/api/telemetry"
OUT=ROOT/"imu_zupt_test_latest.csv"
LOG=ROOT/"imu_zupt_test_service.log"
FORENSIC=ROOT/"fast_motion_forensic_latest.log"

def bar(label,e,total):
    width=30; p=max(0,min(1,e/total)); n=int(width*p)
    print("\r%-16s [%s%s] %3d%%"%(label,"#"*n,"-"*(width-n),int(100*p)),end="",flush=True)

def get():
    with urllib.request.urlopen(URL,timeout=.8) as r:return json.load(r)

def busy(port):
    s=socket.socket();s.settimeout(.2)
    try:return s.connect_ex(("127.0.0.1",port))==0
    finally:s.close()

def pick(d,*ks):
    for k in ks:
        if d.get(k) is not None:return d[k]

def phase(name,sec,rows):
    t0=time.monotonic()
    while time.monotonic()-t0<sec:
        e=time.monotonic()-t0
        try:
            d=get()
            rows.append({"phase":name,"t":time.time(),
              "cam_speed":pick(d,"imu_cam_speed"),
              "cam_fresh":pick(d,"imu_cam_fresh"),
              "cam_stationary":pick(d,"imu_cam_stationary"),
              "imu_stationary":pick(d,"imu_dr_stationary"),
              "zupt_shadow":pick(d,"imu_zupt_shadow"),
              "zupt_accepts":pick(d,"imu_zupt_shadow_accepts"),
              "zupt_blocks":pick(d,"imu_zupt_shadow_blocks"),
              "imu_n_mm":pick(d,"imu_dr_n_mm"),"imu_e_mm":pick(d,"imu_dr_e_mm"),
              "imu_d_mm":pick(d,"imu_dr_d_mm"),"imu_vn":pick(d,"imu_dr_vn"),
              "imu_ve":pick(d,"imu_dr_ve"),"imu_vd":pick(d,"imu_dr_vd"),
              "imu_acc_n":pick(d,"imu_dr_acc_n"),"imu_acc_e":pick(d,"imu_dr_acc_e"),
              "imu_acc_d":pick(d,"imu_dr_acc_d"),"imu_dt":pick(d,"imu_dr_dt"),
              "cam_vn":pick(d,"imu_cam_vn"),"cam_ve":pick(d,"imu_cam_ve"),
              "cam_seq":pick(d,"imu_cam_seq"),
              "fused_v1_visual_updates":pick(d,"fused_v1_visual_updates"),
              "fused_v1_imu_predictions":pick(d,"fused_v1_imu_predictions"),
              "fused_v1_stop_constraints":pick(d,"fused_v1_stop_constraints"),
              "fused_v1_stationary":pick(d,"fused_v1_stationary"),
              "fused_v1_stop_confirm":pick(d,"fused_v1_stop_confirm"),
              "fused_v1_n_mm":pick(d,"fused_v1_n_mm"),
              "fused_v1_e_mm":pick(d,"fused_v1_e_mm"),
              "fused_v1_vn":pick(d,"fused_v1_vn"),
              "fused_v1_ve":pick(d,"fused_v1_ve")})
        except Exception:pass
        bar(name,e,sec);time.sleep(.1)
    bar(name,sec,sec);print()

print("\033[2J\033[H",end="")
print("JT-Zero — тест IMU / camera-gated ZUPT")
print("======================================")
print("Нужен только этот терминал.")
print("Движение может быть дёрганым. Отрыв от стола допустим.")
print("HOME во время теста не нажимать.\n")

if busy(8080) or busy(5760):
    print("ОШИБКА: старый Web/router ещё работает.")
    print("Закрой старые процессы и запусти тест снова.")
    raise SystemExit(2)

log=open(LOG,"w",encoding="utf-8")
env=os.environ.copy();env["MONKEYS_LOCAL_GUI"]="0";env["MONKEYS_WEB_TELEMETRY_UDP_PORT"]="8766"
proc=subprocess.Popen(["bash",str(ROOT/"scripts"/"run_web.sh")],cwd=ROOT,env=env,
                      stdout=log,stderr=subprocess.STDOUT,text=True)
try:
    print("Запуск router + Web + runtime...")
    t0=time.monotonic();ready=False
    while time.monotonic()-t0<90:
        e=time.monotonic()-t0;bar("Подготовка",e,90)
        if proc.poll() is not None:
            print("\nОШИБКА запуска. Лог:",LOG);raise SystemExit(3)
        try:
            d=get()
            if d.get("imu_cam_fresh") is True and d.get("imu_cam_speed") is not None:
                ready=True;break
        except Exception:pass
        time.sleep(.25)
    print()
    if not ready:
        print("ОШИБКА: система не готова за 90 секунд. Лог:",LOG);raise SystemExit(4)

    print("СИСТЕМА ГОТОВА")
    input("\nПоставь аппарат в исходное положение и НЕ ТРОГАЙ. Нажми Enter... ")
    rows=[]
    print("\nЭТАП 1/3 — ПОКОЙ. Не трогай аппарат.")
    phase("ПОКОЙ ДО",5,rows)
    print("\nЭТАП 2/3 — ДВИГАЙ СЕЙЧАС.")
    print("Двигай как получается: рывки и отрыв допустимы.")
    phase("ДВИЖЕНИЕ",7,rows)
    print("\nЭТАП 3/3 — СТОП. Положи аппарат и не трогай.")
    phase("ПОКОЙ ПОСЛЕ",6,rows)

    with open(OUT,"w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    move=[r for r in rows if r["phase"]=="ДВИЖЕНИЕ"]
    false_imu=sum(1 for r in move if r["imu_stationary"] is True and r["cam_stationary"] is False)
    moving=sum(1 for r in move if r["cam_stationary"] is False)
    speeds=[]
    for r in move:
        try:speeds.append(float(r["cam_speed"]))
        except:pass
    print("\nРЕЗУЛЬТАТ")
    print("---------")
    print("Отсчётов с движением камеры:",moving)
    print("Ложных «стоим» от IMU во время движения:",false_imu)
    if speeds:print("Макс. скорость камеры: %.1f мм/с"%(1000*max(speeds)))
    print("CSV:",OUT)
    print("\nГотово. Сервисы останавливаются автоматически.")
finally:
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=12)
        except subprocess.TimeoutExpired:proc.terminate()
    log.close()
