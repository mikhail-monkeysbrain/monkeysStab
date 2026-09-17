#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

root = Path(__file__).resolve().parents[1]
dataset = Path(os.environ["MONKEYS_DATASET_DIR"])
log_path = dataset / "capture_process.log"

env = os.environ.copy()
env["MONKEYS_RETURN_CLI"] = "0"
env["MONKEYS_BLIND4_CLI"] = "1"
env["MONKEYS_LOCAL_GUI"] = "0"

print("\nПодготовка системы...", flush=True)

with log_path.open("w", encoding="utf-8", errors="replace") as log:
    p = subprocess.Popen(
        ["bash", str(root / "scripts" / "run.sh")],
        cwd=root,
        env=env,
        stdin=None,                 # keep operator TTY for SPACE
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
    )

    stop_started = False

    def stop_after_b():
        # B marker is assigned before the CSV row is written. Give the camera
        # loop time to persist event 12, then request normal SIGINT shutdown.
        time.sleep(0.75)
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)

    assert p.stdout is not None
    for line in p.stdout:
        log.write(line)
        log.flush()
        s = line.strip()

        if "СИСТЕМА ГОТОВА." in s:
            print("\nСИСТЕМА ГОТОВА", flush=True)
            print("Аппарат неподвижен в точке A -> нажми SPACE", flush=True)
        elif "BLIND4 A1 ЗАФИКСИРОВАНА" in s:
            print("\nA ЗАФИКСИРОВАНА", flush=True)
            print("Перемести аппарат в B, полностью останови -> нажми SPACE", flush=True)
        elif "BLIND4 B1 ЗАФИКСИРОВАНА" in s:
            print("\nB ЗАФИКСИРОВАНА", flush=True)
            print("Аппарат не двигать. RAW сохранён, завершаю запись...", flush=True)
            if not stop_started:
                stop_started = True
                threading.Thread(target=stop_after_b, daemon=True).start()
        elif s.startswith("ОШИБКА") or "ERROR" in s or "terminate called" in s:
            print(s, flush=True)

    rc = p.wait()

# SIGINT after B is an expected controlled stop. Anything else is returned so
# the shell integrity gate can decide whether the capture is usable.
if stop_started:
    sys.exit(0)
sys.exit(rc)
