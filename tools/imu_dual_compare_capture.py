#!/usr/bin/env python3
import argparse
import csv
import fcntl
import json
import math
import os
import struct
import time
import urllib.request
from datetime import datetime
from pathlib import Path

I2C_SLAVE = 0x0703


class Mpu6050:
    def __init__(self, dev="/dev/i2c-1", addr=0x68):
        self.fd = os.open(dev, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, addr)
        self.write_reg(0x6B, 0x01)  # wake, PLL X gyro
        self.write_reg(0x1A, 0x03)  # DLPF=3
        self.write_reg(0x1B, 0x00)  # gyro +/-250 dps
        self.write_reg(0x1C, 0x00)  # accel +/-2 g
        time.sleep(0.2)

    def write_reg(self, reg, value):
        os.write(self.fd, bytes((reg, value)))

    def read(self):
        t0_ns = time.monotonic_ns()
        os.write(self.fd, b"\x3b")
        b = os.read(self.fd, 14)
        t1_ns = time.monotonic_ns()
        if len(b) != 14:
            raise OSError(f"MPU short read: {len(b)}")
        ax, ay, az, temp, gx, gy, gz = struct.unpack(">hhhhhhh", b)
        return {
            "mpu_mono_ns": (t0_ns + t1_ns) // 2,
            "mpu_read_us": (t1_ns - t0_ns) / 1000.0,
            "mpu_ax_raw": ax,
            "mpu_ay_raw": ay,
            "mpu_az_raw": az,
            "mpu_gx_raw": gx,
            "mpu_gy_raw": gy,
            "mpu_gz_raw": gz,
            "mpu_ax_g": ax / 16384.0,
            "mpu_ay_g": ay / 16384.0,
            "mpu_az_g": az / 16384.0,
            "mpu_gx_dps": gx / 131.0,
            "mpu_gy_dps": gy / 131.0,
            "mpu_gz_dps": gz / 131.0,
        }

    def close(self):
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


def get_json(url):
    with urllib.request.urlopen(url, timeout=1.0) as r:
        return json.load(r)


TELEMETRY_KEYS = [
    "mono_ns",
    "imu_raw_ax", "imu_raw_ay", "imu_raw_az",
    "imu_dr_acc_n", "imu_dr_acc_e", "imu_dr_acc_d",
    "imu_dr_bias_n", "imu_dr_bias_e", "imu_dr_bias_d",
    "imu_dr_vn", "imu_dr_ve", "imu_dr_vd",
    "imu_dr_n_mm", "imu_dr_e_mm", "imu_dr_d_mm",
    "imu_dr_stationary_samples", "imu_dr_stationary",
    "imu_dr_acc_ok", "imu_dr_gyro_ok",
    "raw_of_vn", "raw_of_ve", "raw_of_valid",
    "roll_deg", "pitch_deg", "yaw_deg",
]


def make_row(t0_ns, telem, mpu):
    now_ns = time.monotonic_ns()
    row = {
        "t": (now_ns - t0_ns) / 1e9,
        "capture_mono_ns": now_ns,
    }
    for k in TELEMETRY_KEYS:
        row[k] = telem.get(k)
    row.update(mpu)
    api_ns = telem.get("mono_ns")
    try:
        row["mpu_minus_api_ms"] = (int(mpu["mpu_mono_ns"]) - int(api_ns)) / 1e6
    except (TypeError, ValueError):
        row["mpu_minus_api_ms"] = None
    return row


ap = argparse.ArgumentParser(
    description="Synchronized FC HIGHRES_IMU / ATTITUDE / OV9281 OF / external MPU diagnostic capture"
)
ap.add_argument("--url", default="http://127.0.0.1:8080/api/telemetry")
ap.add_argument("--seconds", type=float, default=15.0)
ap.add_argument("--hz", type=float, default=20.0)
ap.add_argument("--i2c", default="/dev/i2c-1")
ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x68)
ap.add_argument("--out", default=None)
a = ap.parse_args()

if a.seconds <= 0 or a.hz <= 0:
    raise SystemExit("--seconds and --hz must be > 0")

try:
    first = get_json(a.url)
except Exception as e:
    raise SystemExit(f"Нет Web-телеметрии: {e}")

required = ("raw_of_vn", "raw_of_ve", "roll_deg", "pitch_deg", "yaw_deg")
missing = [k for k in required if k not in first]
if missing:
    raise SystemExit("В Web-телеметрии отсутствуют поля: " + ", ".join(missing))

try:
    mpu = Mpu6050(a.i2c, a.addr)
    probe = mpu.read()
except Exception as e:
    raise SystemExit(f"MPU/I2C ошибка: {e}")

out = Path(a.out) if a.out else Path(
    "imu_dual_compare_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
)

print("DUAL IMU / CAMERA CAPTURE")
print("=========================")
print(f"Web telemetry: {a.url}")
print(f"External MPU: {a.i2c} @ 0x{a.addr:02x}")
print(f"MPU probe |a|={math.sqrt(probe['mpu_ax_g']**2 + probe['mpu_ay_g']**2 + probe['mpu_az_g']**2):.4f} g")
print()
print("После ENTER: 2-3 с покоя -> движение по красной стрелке -> полная остановка -> покой.")
input("Нажми ENTER для начала записи... ")

rows = []
period = 1.0 / a.hz
t0_ns = time.monotonic_ns()
next_t = time.monotonic()

try:
    while (time.monotonic_ns() - t0_ns) / 1e9 < a.seconds:
        try:
            telem = get_json(a.url)
            ext = mpu.read()
            rows.append(make_row(t0_ns, telem, ext))
        except Exception as e:
            print(f"sample warning: {e}")
        next_t += period
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.monotonic()
finally:
    mpu.close()

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
            of_speeds.append(math.hypot(float(r.get("raw_of_vn") or 0.0),
                                        float(r.get("raw_of_ve") or 0.0)))
        except (TypeError, ValueError):
            pass

print()
print(f"Записано: {len(rows)} samples за {rows[-1]['t']:.2f} с")
print(f"CSV: {out}")
if of_speeds:
    print(f"Camera OF: valid, max speed={max(of_speeds):.4f} m/s")
else:
    print("Camera OF: нет valid samples")
