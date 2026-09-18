#!/usr/bin/env python3
import argparse
import csv
import json
import math
import time
import urllib.request
from datetime import datetime
from pathlib import Path


def get(url):
    with urllib.request.urlopen(url, timeout=1.0) as r:
        return json.load(r)


def sample(t, elapsed):
    keys = [
        "imu_dr_amag", "imu_dr_gmag",
        "imu_dr_acc_n", "imu_dr_acc_e", "imu_dr_acc_d",
        "imu_dr_vn", "imu_dr_ve", "imu_dr_vd",
        "imu_dr_n_mm", "imu_dr_e_mm", "imu_dr_d_mm",
        "imu_dr_stationary_samples", "imu_dr_stationary",
        "imu_dr_acc_ok", "imu_dr_gyro_ok",
        "raw_of_vn", "raw_of_ve", "raw_of_valid",
        "roll_deg", "pitch_deg", "yaw_deg",
    ]
    row = {"t": elapsed}
    for k in keys:
        row[k] = t.get(k)
    return row


ap = argparse.ArgumentParser(description="Capture synchronized IMU/OF diagnostic profile")
ap.add_argument("--url", default="http://127.0.0.1:8080/api/telemetry")
ap.add_argument("--seconds", type=float, default=15.0)
ap.add_argument("--out", default=None)
a = ap.parse_args()

try:
    first = get(a.url)
except Exception as e:
    raise SystemExit(f"Нет телеметрии: {e}")

if not first.get("imu_dr_calibrated"):
    raise SystemExit("IMU DR не откалиброван. Выполни физический RC HOME и дождись калибровки.")

out = Path(a.out) if a.out else Path(
    "imu_drift_profile_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
)

print("IMU/OF PROFILE CAPTURE")
print("======================")
print("После ENTER: начни с покоя, затем двигай стенд преимущественно по одной оси,")
print("полностью останови и убери руки. Точные тайминги не нужны.")
input("Нажми ENTER для начала записи... ")

rows = []
t0 = time.monotonic()
while True:
    now = time.monotonic()
    elapsed = now - t0
    if elapsed >= a.seconds:
        break
    try:
        rows.append(sample(get(a.url), elapsed))
    except Exception:
        pass
    time.sleep(0.05)

if not rows:
    raise SystemExit("Не получено ни одного отсчёта.")

fields = list(rows[0].keys())
with out.open("w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=fields)
    w.writeheader()
    w.writerows(rows)

of_speeds = []
for r in rows:
    if r.get("raw_of_valid"):
        try:
            of_speeds.append(math.hypot(float(r.get("raw_of_vn") or 0), float(r.get("raw_of_ve") or 0)))
        except (TypeError, ValueError):
            pass

print()
print(f"Записано: {len(rows)} samples за {rows[-1]['t']:.2f} с")
print(f"CSV: {out}")
if of_speeds:
    print(f"Camera OF: valid, max speed={max(of_speeds):.4f}")
else:
    print("Camera OF: нет valid samples")
