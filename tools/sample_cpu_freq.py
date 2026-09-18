#!/usr/bin/env python3
"""Independent CPU-frequency sampler for LK runtime forensic tests.

Reads cpufreq from sysfs outside the production LK path and writes monotonic
timestamps plus all available CPU frequencies to CSV.
"""
import argparse
import csv
import glob
import os
import signal
import time

running = True

def stop(*_):
    global running
    running = False

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--hz", type=float, default=50.0)
args = ap.parse_args()

paths = sorted(glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq"))
if not paths:
    raise SystemExit("No scaling_cur_freq sysfs entries found")

cpus = [p.split("/cpu")[-1].split("/")[0] for p in paths]
period = 1.0 / max(args.hz, 1.0)
next_t = time.monotonic()

with open(args.out, "w", newline="", buffering=1) as f:
    w = csv.writer(f)
    w.writerow(["mono_ns"] + [f"cpu{c}_khz" for c in cpus])
    while running:
        now_ns = time.monotonic_ns()
        vals = []
        for p in paths:
            try:
                with open(p) as sf:
                    vals.append(int(sf.read().strip()))
            except Exception:
                vals.append("")
        w.writerow([now_ns] + vals)
        next_t += period
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.monotonic()
