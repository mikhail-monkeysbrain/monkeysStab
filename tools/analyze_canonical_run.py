#!/usr/bin/env python3
"""Forensic analysis of a monkeysStab canonical A-B-H run.

This tool is deliberately measurement-only. It does not modify calibration,
runtime configuration, or focal_scale.
"""
import argparse
import csv
import math
from collections import Counter
from pathlib import Path
from statistics import median


def fval(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def ival(row, key):
    try:
        return int(float(row[key]))
    except (KeyError, TypeError, ValueError):
        return 0


def integrate(rows, lo, hi, xcol, ycol):
    x = y = 0.0
    used = 0
    for r in rows[lo:hi + 1]:
        if ival(r, "valid") != 1:
            continue
        dt = fval(r, "dt_s")
        h = fval(r, "range_to_fc_m")
        fx = fval(r, xcol)
        fy = fval(r, ycol)
        if not all(math.isfinite(v) for v in (dt, h, fx, fy)) or dt <= 0:
            continue
        # flow_body / flow_send are rates [rad/s]. Integrate rate*dt*range.
        x += fx * dt * h
        y += fy * dt * h
        used += 1
    return x, y, used


def vector_report(rows, ia, ib, ih, title, xcol, ycol, gt_m):
    abx, aby, nab = integrate(rows, ia + 1, ib, xcol, ycol)
    bax, bay, nba = integrate(rows, ib + 1, ih, xcol, ycol)
    ab = math.hypot(abx, aby)
    ba = math.hypot(bax, bay)
    cx, cy = abx + bax, aby + bay
    closure = math.hypot(cx, cy)
    denom = ab * ba
    angle = float("nan")
    if denom:
        c = max(-1.0, min(1.0, (abx * bax + aby * bay) / denom))
        angle = math.degrees(math.acos(c))

    print(f"\n===== {title} =====")
    print(f"A->B vector:       X={abx*1000:+.2f} Y={aby*1000:+.2f} mm")
    print(f"A->B distance:     {ab*1000:.2f} mm  samples={nab}")
    print(f"B->H vector:       X={bax*1000:+.2f} Y={bay*1000:+.2f} mm")
    print(f"B->H magnitude:    {ba*1000:.2f} mm  samples={nba}")
    print("B->H ground truth: UNKNOWN (return is not assumed equal to A->B)")
    print(f"reversal angle:    {angle:.2f} deg")
    print(f"closure vector:    X={cx*1000:+.2f} Y={cy*1000:+.2f} mm")
    print(f"closure residual:  {closure*1000:.2f} mm")
    print("closure note:      includes physical error of returning by hand to A")
    print(f"A->B ground truth: {gt_m*1000:.2f} mm")
    print(f"A->B measured/GT:  {ab/gt_m:.6f}")
    print(f"A->B error:        {(ab-gt_m)*1000:+.2f} mm ({(ab/gt_m-1)*100:+.2f} %)")
    return ab


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--distance-mm", required=True, type=float,
                    help="measured physical A->B distance; no assumption is made for B->H")
    args = ap.parse_args()

    path = args.dataset / "optical_flow_mavlink.csv"
    with path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))

    events = {}
    for i, r in enumerate(rows):
        ev = ival(r, "return_event")
        if ev in (1, 2, 3) and ev not in events:
            events[ev] = i
    missing = [e for e in (1, 2, 3) if e not in events]
    if missing:
        raise SystemExit(f"missing return_event markers: {missing}")

    ia, ib, ih = events[1], events[2], events[3]
    gt_m = args.distance_mm / 1000.0

    print("===== CANONICAL FORENSIC =====")
    print(f"dataset: {args.dataset}")
    print(f"A row={ia+2}  B row={ib+2}  H row={ih+2}")
    print(f"ONLY exact ground truth: A->B={args.distance_mm:.2f} mm")

    vector_report(rows, ia, ib, ih, "NATIVE flow_body",
                  "flow_body_x", "flow_body_y", gt_m)
    vector_report(rows, ia, ib, ih, "ACTUAL flow_send",
                  "flow_send_x", "flow_send_y", gt_m)

    seg = rows[ia + 1:ih + 1]
    dts = [fval(r, "dt_s") for r in seg
           if ival(r, "valid") == 1 and math.isfinite(fval(r, "dt_s"))]
    hs = [fval(r, "range_to_fc_m") for r in seg
          if ival(r, "valid") == 1 and math.isfinite(fval(r, "range_to_fc_m"))]
    bad = [r for r in seg if ival(r, "valid") != 1]

    print("\n===== DATA QUALITY A..H =====")
    print(f"rows={len(seg)} valid={len(seg)-len(bad)} invalid={len(bad)}")
    print(f"invalid reasons={dict(Counter(r.get('invalid_reason','') for r in bad))}")
    if dts:
        print(f"dt median/mean/max={median(dts)*1000:.3f}/{sum(dts)/len(dts)*1000:.3f}/{max(dts)*1000:.3f} ms")
        print(f"dt >50ms={sum(x > .050 for x in dts)}  >100ms={sum(x > .100 for x in dts)}")
    if hs:
        print(f"range median/min/max={median(hs)*1000:.2f}/{min(hs)*1000:.2f}/{max(hs)*1000:.2f} mm")
    print(f"lever production rows={sum(ival(r,'lever_production_applied') for r in seg)}/{len(seg)}")

    print("\n===== INTERPRETATION GUARD =====")
    print("Do NOT infer B->H scale error: its physical path length was not measured.")
    print("Do NOT treat closure residual as pure estimator error: hand-return error is unknown.")
    print("Do NOT change focal_scale from one measured A->B leg.")


if __name__ == "__main__":
    main()
