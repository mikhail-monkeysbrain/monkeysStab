#!/usr/bin/env python3
import csv
import json
import os
import signal
import socket
import subprocess
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNS = Path("/home/vio/monkeysStab_runs")
URL = "http://127.0.0.1:8080/api/telemetry"
SERVICE_LOG = ROOT / "false_bad_stationary_service.log"
TELEMETRY_CSV = ROOT / "false_bad_stationary_telemetry.csv"
DURATION_S = int(os.environ.get("FALSE_BAD_STATIONARY_SECONDS", "300"))

def get():
    with urllib.request.urlopen(URL, timeout=0.8) as r:
        return json.load(r)

def busy(port):
    s = socket.socket()
    s.settimeout(0.2)
    try:
        return s.connect_ex(("127.0.0.1", port)) == 0
    finally:
        s.close()

def pick(d, *keys):
    for k in keys:
        if d.get(k) is not None:
            return d[k]
    return None

def newest_run_after(start_wall):
    if not RUNS.exists():
        return None
    candidates = []
    for p in RUNS.iterdir():
        if not p.is_dir():
            continue
        try:
            if p.stat().st_mtime >= start_wall - 5:
                candidates.append(p)
        except OSError:
            pass
    return max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None

def read_fused(path):
    rows = []
    if not path or not path.exists():
        return rows
    with path.open(newline="", encoding="utf-8", errors="replace") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows

def fnum(r, key, default=0.0):
    try:
        return float(r.get(key, default))
    except (TypeError, ValueError):
        return default

def inum(r, key, default=0):
    try:
        return int(float(r.get(key, default)))
    except (TypeError, ValueError):
        return default

print("\033[2J\033[H", end="")
print("JT-Zero — FALSE BAD / STATIONARY diagnostic")
print("==========================================")
print("Тест: 5 минут полного покоя.")
print("WORKED5/FUSED-V2 и пороги НЕ изменяются.")
print("Нужен только этот терминал.\n")

if busy(8080) or busy(5760):
    raise SystemExit("ОШИБКА: старый Web/router ещё работает. Останови его и запусти тест снова.")

start_wall = time.time()
log = SERVICE_LOG.open("w", encoding="utf-8")
env = os.environ.copy()
env["MONKEYS_LOCAL_GUI"] = "0"
env["MONKEYS_WEB_TELEMETRY_UDP_PORT"] = "8766"
proc = subprocess.Popen(
    ["bash", str(ROOT / "scripts" / "run_web.sh")],
    cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, text=True
)

samples = []
try:
    print("Запуск router + Web + runtime...")
    ready = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 90:
        if proc.poll() is not None:
            raise RuntimeError(f"service exited: {proc.returncode}; log={SERVICE_LOG}")
        try:
            d = get()
            if d.get("imu_cam_fresh") is True and d.get("imu_cam_speed") is not None:
                ready = True
                break
        except Exception:
            pass
        time.sleep(0.25)

    if not ready:
        raise RuntimeError(f"telemetry not ready in 90 s; log={SERVICE_LOG}")

    print("СИСТЕМА ГОТОВА")
    input("\nПоставь аппарат на стол. Не трогай его весь тест. Нажми Enter... ")

    t0 = time.monotonic()
    while True:
        elapsed = time.monotonic() - t0
        if elapsed >= DURATION_S:
            break
        try:
            d = get()
            samples.append({
                "wall_time": time.time(),
                "elapsed_s": elapsed,
                "cam_speed": pick(d, "imu_cam_speed"),
                "cam_fresh": pick(d, "imu_cam_fresh"),
                "cam_stationary": pick(d, "imu_cam_stationary"),
                "cam_seq": pick(d, "imu_cam_seq"),
                "imu_stationary": pick(d, "imu_dr_stationary"),
                "imu_n_mm": pick(d, "imu_dr_n_mm"),
                "imu_e_mm": pick(d, "imu_dr_e_mm"),
                "imu_vn": pick(d, "imu_dr_vn"),
                "imu_ve": pick(d, "imu_dr_ve"),
                "imu_acc_n": pick(d, "imu_dr_acc_n"),
                "imu_acc_e": pick(d, "imu_dr_acc_e"),
                "imu_acc_d": pick(d, "imu_dr_acc_d"),
                "imu_dt": pick(d, "imu_dr_dt"),
                "fused_v1_n_mm": pick(d, "fused_v1_n_mm"),
                "fused_v1_e_mm": pick(d, "fused_v1_e_mm"),
            })
        except Exception:
            pass

        left = max(0, DURATION_S - elapsed)
        print(
            f"\rПОКОЙ {elapsed:6.1f}/{DURATION_S}s | осталось {left:6.1f}s | samples={len(samples)}",
            end="", flush=True
        )
        time.sleep(0.1)
    print()

finally:
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
    log.close()

if samples:
    with TELEMETRY_CSV.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(samples[0].keys()))
        w.writeheader()
        w.writerows(samples)

run_dir = newest_run_after(start_wall)
fused_csv = run_dir / "fused_v2_realtime_shadow.csv" if run_dir else None
rows = read_fused(fused_csv)

print("\nРЕЗУЛЬТАТ")
print("---------")
print("Run:", run_dir if run_dir else "НЕ НАЙДЕН")
print("Telemetry:", TELEMETRY_CSV)
print("FUSED-V2 CSV:", fused_csv if fused_csv and fused_csv.exists() else "НЕ НАЙДЕН")

if not rows:
    print("Нет fused_v2_realtime_shadow.csv — диагностический результат неполный.")
    raise SystemExit(0)

bad_rows = [
    r for r in rows
    if fnum(r, "inlier_ratio", 1.0) < 0.50 and inum(r, "inliers", 999999) < 100
]
bridge_rows = [r for r in rows if inum(r, "bridge", 0) == 1]
events = {}
for r in rows:
    ev = inum(r, "event_count", 0)
    if ev > 0:
        events.setdefault(ev, []).append(r)

first = rows[0]
last = rows[-1]
shadow_dn_mm = 1000.0 * (fnum(last, "shadow_n_m") - fnum(first, "shadow_n_m"))
shadow_de_mm = 1000.0 * (fnum(last, "shadow_e_m") - fnum(first, "shadow_e_m"))
shadow_dxy_mm = (shadow_dn_mm ** 2 + shadow_de_mm ** 2) ** 0.5

print("Frames:", len(rows))
print("BAD frames:", len(bad_rows))
print("Bridge frames:", len(bridge_rows))
print("Bridge events:", max((inum(r, "event_count", 0) for r in rows), default=0))
print(f"FUSED-V2 stationary displacement: {shadow_dxy_mm:.3f} mm  dN={shadow_dn_mm:.3f} dE={shadow_de_mm:.3f}")

if bad_rows:
    ratios = [fnum(r, "inlier_ratio") for r in bad_rows]
    tracked = [inum(r, "tracked") for r in bad_rows]
    inliers = [inum(r, "inliers") for r in bad_rows]
    dts = [1000.0 * fnum(r, "dt_s") for r in bad_rows]
    print(
        "BAD ranges:"
        f" ratio={min(ratios):.3f}..{max(ratios):.3f}"
        f" tracked={min(tracked)}..{max(tracked)}"
        f" inliers={min(inliers)}..{max(inliers)}"
        f" dt_ms={min(dts):.3f}..{max(dts):.3f}"
    )

print("\nEVENTS")
seen = set()
for r in rows:
    ev = inum(r, "event_count", 0)
    if ev <= 0 or ev in seen:
        continue
    seen.add(ev)
    print(
        f"event={ev}"
        f" anchor={inum(r,'anchor_frame')}"
        f" bad={inum(r,'bad_frame')}"
        f" frame={inum(r,'frame')}"
        f" tracked={inum(r,'tracked')}"
        f" inliers={inum(r,'inliers')}"
        f" ratio={fnum(r,'inlier_ratio'):.3f}"
        f" dt_ms={1000.0*fnum(r,'dt_s'):.3f}"
        f" endpoint_mm={1000.0*fnum(r,'shadow_endpoint_m'):.3f}"
    )

print("\nТест завершён. Аппарат во время этого прогона должен был оставаться полностью неподвижным.")
