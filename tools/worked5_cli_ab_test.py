#!/usr/bin/env python3
"""Minimal interactive A->B->A test for the frozen WORKED5 online estimator.

Reads the newest live optical_flow_mavlink.csv. It does not use the Web UI,
does not reset the runtime accumulator, and does not modify estimator state.
The operator marks A, B and returned A by pressing Enter.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
import time
from pathlib import Path

REQUIRED = (
    "frame",
    "worked5_valid",
    "worked5_hcam_m",
    "worked5_acc_n_m",
    "worked5_acc_e_m",
)


def fail(msg: str) -> None:
    print(f"ERROR: {msg}", file=sys.stderr)
    raise SystemExit(2)


def running_csv() -> Path:
    try:
        out = subprocess.check_output(
            ["pgrep", "-af", "monkeysstab_optical_flow"], text=True
        )
    except subprocess.CalledProcessError:
        fail("monkeysstab_optical_flow is not running")
    rows = [x for x in out.splitlines() if "optical_flow_mavlink.csv" in x]
    if len(rows) != 1:
        fail(f"expected exactly one runtime, found {len(rows)}")
    parts = rows[0].split()
    csv_paths = [Path(x) for x in parts if x.endswith("optical_flow_mavlink.csv")]
    if len(csv_paths) != 1:
        fail("cannot determine runtime CSV from process command line")
    return csv_paths[0]


def header(path: Path) -> list[str]:
    deadline = time.time() + 5.0
    while time.time() < deadline:
        try:
            with path.open("r", encoding="utf-8", newline="") as f:
                row = next(csv.reader(f), None)
            if row:
                return row
        except (FileNotFoundError, OSError):
            pass
        time.sleep(0.1)
    fail(f"CSV is not readable: {path}")


def last_row(path: Path, names: list[str]) -> dict[str, str]:
    # The runtime CSV is append-only. Seek near EOF to avoid loading a long run.
    with path.open("rb") as f:
        f.seek(0, os.SEEK_END)
        size = f.tell()
        f.seek(max(0, size - 256 * 1024), os.SEEK_SET)
        data = f.read().decode("utf-8", "replace")
    lines = data.splitlines()
    if size > 256 * 1024 and lines:
        lines = lines[1:]
    for line in reversed(lines):
        try:
            vals = next(csv.reader([line]))
        except Exception:
            continue
        if len(vals) == len(names):
            return dict(zip(names, vals))
    fail("no complete CSV row found")


def f(row: dict[str, str], key: str) -> float:
    try:
        v = float(row[key])
    except (KeyError, ValueError):
        fail(f"invalid {key} in CSV")
    if not math.isfinite(v):
        fail(f"non-finite {key} in CSV")
    return v


def snapshot(path: Path, names: list[str], label: str) -> dict[str, float]:
    row = last_row(path, names)
    s = {
        "frame": f(row, "frame"),
        "valid": f(row, "worked5_valid"),
        "h": f(row, "worked5_hcam_m"),
        "n": f(row, "worked5_acc_n_m"),
        "e": f(row, "worked5_acc_e_m"),
    }
    print(
        f"{label}: frame={int(s['frame'])}  "
        f"N={s['n']*1000:+.1f} mm  E={s['e']*1000:+.1f} mm  "
        f"h={s['h']*1000:.1f} mm"
    )
    return s


def delta(a: dict[str, float], b: dict[str, float]) -> tuple[float, float, float]:
    dn = (b["n"] - a["n"]) * 1000.0
    de = (b["e"] - a["e"]) * 1000.0
    return dn, de, math.hypot(dn, de)


def wait_enter(text: str) -> None:
    try:
        input(text)
    except (EOFError, KeyboardInterrupt):
        print("\nABORT")
        raise SystemExit(130)


def main() -> None:
    ap = argparse.ArgumentParser(description="WORKED5 clean CLI A-B-A test")
    ap.add_argument("--distance-mm", type=float, default=None,
                    help="known horizontal A->B distance; optional")
    args = ap.parse_args()
    if args.distance_mm is not None and args.distance_mm <= 0:
        fail("--distance-mm must be > 0")

    path = running_csv()
    names = header(path)
    missing = [x for x in REQUIRED if x not in names]
    if missing:
        fail("runtime CSV lacks forensic columns: " + ", ".join(missing))

    print("WORKED5 CLI A-B-A")
    print(f"CSV: {path}")
    print("Web UI is not used. Runtime accumulator is not reset.")
    print()

    wait_enter("Place stand at A, keep still, press ENTER: ")
    a0 = snapshot(path, names, "A START")

    wait_enter("Move A -> B, stop at B, press ENTER: ")
    b = snapshot(path, names, "B")
    ab_n, ab_e, ab = delta(a0, b)
    print(f"A->B: dN={ab_n:+.1f} mm  dE={ab_e:+.1f} mm  distance={ab:.1f} mm")
    if args.distance_mm is not None:
        err = ab - args.distance_mm
        print(f"GT: {args.distance_mm:.1f} mm  error={err:+.1f} mm ({err/args.distance_mm*100:+.2f}%)")
    print()

    wait_enter("Return B -> A, stop at A, press ENTER: ")
    a1 = snapshot(path, names, "A FINISH")
    ba_n, ba_e, ba = delta(b, a1)
    cl_n, cl_e, closure = delta(a0, a1)
    print(f"B->A: dN={ba_n:+.1f} mm  dE={ba_e:+.1f} mm  distance={ba:.1f} mm")
    print(f"CLOSURE: dN={cl_n:+.1f} mm  dE={cl_e:+.1f} mm  residual={closure:.1f} mm")
    print(f"HEIGHT: A={a0['h']*1000:.1f} mm  B={b['h']*1000:.1f} mm  A2={a1['h']*1000:.1f} mm")
    print(f"FRAMES: A={int(a0['frame'])}  B={int(b['frame'])}  A2={int(a1['frame'])}")


if __name__ == "__main__":
    main()
