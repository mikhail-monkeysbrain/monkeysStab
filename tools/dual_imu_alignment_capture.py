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

class Mpu:
    def __init__(self, dev, addr):
        self.fd=os.open(dev,os.O_RDWR)
        fcntl.ioctl(self.fd,I2C_SLAVE,addr)
        self.wr(0x6b,0x01); self.wr(0x1a,0x03)
        self.wr(0x1b,0x00); self.wr(0x1c,0x00)
        time.sleep(.2)
    def wr(self,r,v): os.write(self.fd,bytes((r,v)))
    def read(self):
        t0=time.monotonic_ns()
        os.write(self.fd,b"\x3b"); b=os.read(self.fd,14)
        t1=time.monotonic_ns()
        if len(b)!=14: raise OSError("MPU short read")
        ax,ay,az,temp,gx,gy,gz=struct.unpack(">hhhhhhh",b)
        return ((t0+t1)//2,ax/16384.,ay/16384.,az/16384.,
                gx/131.,gy/131.,gz/131.)
    def close(self): os.close(self.fd)

def api(url):
    with urllib.request.urlopen(url,timeout=1) as r: return json.load(r)

def f(x):
    try:return float(x)
    except:return float("nan")

ap=argparse.ArgumentParser()
ap.add_argument("--url",default="http://127.0.0.1:8080/api/telemetry")
ap.add_argument("--i2c",default="/dev/i2c-1")
ap.add_argument("--addr",type=lambda x:int(x,0),default=0x68)
ap.add_argument("--seconds",type=float,default=3.0,
                help="seconds recorded per static pose")
ap.add_argument("--hz",type=float,default=20.0)
ap.add_argument("--out",default=None)
a=ap.parse_args()

first=api(a.url)
required=("imu_raw_ax","imu_raw_ay","imu_raw_az","roll_deg","pitch_deg","yaw_deg")
missing=[k for k in required if k not in first]
if missing: raise SystemExit("Web telemetry missing: "+", ".join(missing))

mpu=Mpu(a.i2c,a.addr)
out=Path(a.out) if a.out else Path(
    "dual_imu_alignment_"+datetime.now().strftime("%Y%m%d_%H%M%S")+".csv")

poses=[
    "1/5: обычное исходное положение",
    "2/5: наклон примерно +15 deg вокруг первой горизонтальной оси",
    "3/5: наклон примерно -15 deg вокруг первой горизонтальной оси",
    "4/5: наклон примерно +15 deg вокруг второй горизонтальной оси",
    "5/5: наклон примерно -15 deg вокруг второй горизонтальной оси",
]

rows=[]
print("DUAL IMU STATIC ALIGNMENT")
print("=========================")
print("Точный угол НЕ нужен. В каждом положении конструкция должна быть полностью неподвижна.")
print("Не вращай только по yaw: нужен именно наклон относительно гравитации.")

try:
    for pose,label in enumerate(poses,1):
        print()
        print(label)
        input("Установи положение, полностью останови и нажми ENTER... ")
        print(f"Запись {a.seconds:.1f} с. НЕ ДВИГАТЬ.")
        t0=time.monotonic_ns(); next_t=time.monotonic()
        while (time.monotonic_ns()-t0)/1e9<a.seconds:
            q=api(a.url)
            mn,ax,ay,az,gx,gy,gz=mpu.read()
            rows.append({
                "pose":pose,
                "pose_t":(time.monotonic_ns()-t0)/1e9,
                "capture_mono_ns":time.monotonic_ns(),
                "api_mono_ns":q.get("mono_ns"),
                "fc_ax_mps2":q.get("imu_raw_ax"),
                "fc_ay_mps2":q.get("imu_raw_ay"),
                "fc_az_mps2":q.get("imu_raw_az"),
                "roll_deg":q.get("roll_deg"),
                "pitch_deg":q.get("pitch_deg"),
                "yaw_deg":q.get("yaw_deg"),
                "raw_of_vn":q.get("raw_of_vn"),
                "raw_of_ve":q.get("raw_of_ve"),
                "mpu_mono_ns":mn,
                "mpu_ax_g":ax,"mpu_ay_g":ay,"mpu_az_g":az,
                "mpu_gx_dps":gx,"mpu_gy_dps":gy,"mpu_gz_dps":gz,
            })
            next_t+=1.0/a.hz
            d=next_t-time.monotonic()
            if d>0: time.sleep(d)
            else: next_t=time.monotonic()
finally:
    mpu.close()

if not rows: raise SystemExit("No samples")
with out.open("w",newline="",encoding="utf-8") as fp:
    w=csv.DictWriter(fp,fieldnames=list(rows[0]))
    w.writeheader(); w.writerows(rows)

print()
print("===== POSE SUMMARY =====")
for pose in range(1,6):
    rr=[r for r in rows if r["pose"]==pose]
    def mean(k): return sum(f(r[k]) for r in rr)/len(rr)
    of=max(math.hypot(f(r["raw_of_vn"]),f(r["raw_of_ve"])) for r in rr)
    print(f"pose {pose}: n={len(rr)} "
          f"FC=[{mean('fc_ax_mps2'):+.4f},{mean('fc_ay_mps2'):+.4f},{mean('fc_az_mps2'):+.4f}] "
          f"SY=[{mean('mpu_ax_g'):+.4f},{mean('mpu_ay_g'):+.4f},{mean('mpu_az_g'):+.4f}] "
          f"RPY=[{mean('roll_deg'):+.2f},{mean('pitch_deg'):+.2f},{mean('yaw_deg'):+.2f}] "
          f"OFmax={of:.4f}")
print("CSV:",out)
