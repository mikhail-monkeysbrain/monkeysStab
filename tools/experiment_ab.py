#!/usr/bin/env python3
import os
import pty
import select
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

root = Path(__file__).resolve().parents[1]
master, slave = pty.openpty()
env = os.environ.copy()

p = subprocess.Popen(
    ["bash", str(root / "scripts" / "run_experiment.sh")],
    cwd=root,
    env=env,
    stdin=slave,
    stdout=slave,
    stderr=slave,
    start_new_session=True,
    close_fds=True,
)
os.close(slave)

state = {"a": False, "b": False, "failed": False}
log_lines = []

def reader():
    buf = b""
    while p.poll() is None or buf:
        try:
            r, _, _ = select.select([master], [], [], 0.1)
            if not r:
                continue
            data = os.read(master, 4096)
            if not data:
                break
            buf += data
        except OSError:
            break
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            s = raw.decode("utf-8", "replace").rstrip("\r")
            log_lines.append(s)
            if "BLIND4 A1 ЗАФИКСИРОВАНА" in s:
                state["a"] = True
                print("\nA ЗАФИКСИРОВАНА")
                print("Перемести стенд в B и полностью останови.")
            elif "BLIND4 B1 ЗАФИКСИРОВАНА" in s:
                state["b"] = True
                print("\nB ЗАФИКСИРОВАНА")
            elif s.startswith("ОШИБКА") or "Device or resource busy" in s:
                state["failed"] = True
                print(s)

threading.Thread(target=reader, daemon=True).start()

print("ЗАПУСК...", flush=True)
time.sleep(2.0)
if p.poll() is not None or state["failed"]:
    print("ОШИБКА ЗАПУСКА")
    sys.exit(p.returncode or 1)

print("\nГОТОВО")
input("Нажми ENTER, чтобы зафиксировать A: ")
os.write(master, b" ")

for _ in range(30):
    if state["a"] or p.poll() is not None:
        break
    time.sleep(0.1)
if not state["a"]:
    print("ОШИБКА: A не зафиксирована")
    os.killpg(p.pid, signal.SIGINT)
    sys.exit(2)

input("После полной остановки в B нажми ENTER: ")
os.write(master, b" ")
for _ in range(30):
    if state["b"] or p.poll() is not None:
        break
    time.sleep(0.1)
if not state["b"]:
    print("ОШИБКА: B не зафиксирована")
    os.killpg(p.pid, signal.SIGINT)
    sys.exit(3)

# Give the event-12 CSV row time to flush, then stop the runtime normally.
time.sleep(0.8)
os.killpg(p.pid, signal.SIGINT)
try:
    p.wait(timeout=5)
except subprocess.TimeoutExpired:
    os.killpg(p.pid, signal.SIGTERM)
    p.wait(timeout=2)

print("\nПРОХОД A -> B СОХРАНЁН")
print("GT пока не сообщай.")
