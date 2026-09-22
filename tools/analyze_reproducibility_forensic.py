#!/usr/bin/env python3
"""
Послойный read-only анализ воспроизводимости monkeysStab.

Скрипт НЕ изменяет runtime, параметры FC или логи. Он читает один или несколько
optical_flow_mavlink.csv и сравнивает слои:
  visual frontend -> WORKED5 BODY -> BODY->N/E -> published flow -> EKF3.

Физическое направление и ошибка относительно ground truth намеренно НЕ
выводятся: без отдельного GT это неизвестно.
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path


def f(row, key):
    try:
        v = float(row.get(key, ""))
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def truth(row, key):
    v = f(row, key)
    return v is not None and v != 0.0


def pct(n, d):
    return 100.0 * n / d if d else float("nan")


def q(values, frac):
    a = sorted(v for v in values if v is not None and math.isfinite(v))
    if not a:
        return None
    i = min(len(a) - 1, max(0, round((len(a) - 1) * frac)))
    return a[i]


def span(values):
    a = [v for v in values if v is not None and math.isfinite(v)]
    return (max(a) - min(a)) if a else None


def endpoint_delta(rows, keys):
    good = []
    for r in rows:
        vals = tuple(f(r, k) for k in keys)
        if all(v is not None for v in vals):
            good.append(vals)
    if len(good) < 2:
        return None
    return tuple(b - a for a, b in zip(good[0], good[-1]))


def norm2(x, y):
    if x is None or y is None:
        return None
    return math.hypot(x, y)


def fmt(v, scale=1.0, digits=3):
    if v is None or not math.isfinite(v):
        return "N/A"
    return f"{v * scale:.{digits}f}"


def find_csv(p):
    p = Path(p).expanduser()
    if p.is_file():
        return p
    candidate = p / "optical_flow_mavlink.csv"
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(f"не найден optical_flow_mavlink.csv: {p}")


def analyze(path):
    path = find_csv(path)
    with path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise ValueError(f"пустой CSV: {path}")

    n = len(rows)
    valid = [r for r in rows if truth(r, "valid")]
    w5 = [r for r in rows if truth(r, "worked5_valid")]
    sent = [r for r in rows if truth(r, "flow_sent")]
    ready = [r for r in rows if truth(r, "stabilised_publish_ready")]
    ekf = [r for r in rows if truth(r, "ekf_local_valid")]

    w5_dx = sum((f(r, "worked5_dx_m") or 0.0) for r in w5)
    w5_dy = sum((f(r, "worked5_dy_m") or 0.0) for r in w5)
    w5_dn = sum((f(r, "worked5_dN_m") or 0.0) for r in w5)
    w5_de = sum((f(r, "worked5_dE_m") or 0.0) for r in w5)

    # flow_send_x/y are angular rates. Integrate only rows actually sent.
    pub_x = pub_y = 0.0
    pub_steps = 0
    for r in sent:
        dt = f(r, "dt_s")
        x = f(r, "flow_send_x")
        y = f(r, "flow_send_y")
        if dt is not None and x is not None and y is not None and 0.0 < dt < 0.2:
            pub_x += x * dt
            pub_y += y * dt
            pub_steps += 1

    ekf_d = endpoint_delta(ekf, ("ekf_x_ned", "ekf_y_ned", "ekf_z_ned"))
    roll = [f(r, "fc_roll") for r in rows]
    pitch = [f(r, "fc_pitch") for r in rows]
    yaw = [f(r, "fc_yaw") for r in rows]

    inliers = [f(r, "inliers") for r in valid]
    ratios = [f(r, "inlier_ratio") for r in valid]
    heights = [f(r, "worked5_hcam_m") for r in w5]
    luna = [f(r, "luna_m") for r in rows]
    luna_age = [f(r, "luna_age_ms") for r in rows]
    att_age = [f(r, "fc_gyro_age_ms") for r in rows]
    ekf_age = [f(r, "ekf_age_ms") for r in rows]
    latency = [f(r, "frame_pipeline_latency_ms") for r in rows]
    dtvals = [f(r, "dt_s") for r in valid]
    drops = f(rows[-1], "camera_queue_dropped_total")

    return {
        "path": path, "rows": n,
        "valid_pct": pct(len(valid), n),
        "w5_pct": pct(len(w5), n),
        "sent_pct": pct(len(sent), n),
        "ready_pct": pct(len(ready), n),
        "ekf_pct": pct(len(ekf), n),
        "w5_dx": w5_dx, "w5_dy": w5_dy,
        "w5_body_mag": norm2(w5_dx, w5_dy),
        "w5_dn": w5_dn, "w5_de": w5_de,
        "w5_ne_mag": norm2(w5_dn, w5_de),
        "pub_x_int": pub_x, "pub_y_int": pub_y, "pub_steps": pub_steps,
        "ekf_dn": ekf_d[0] if ekf_d else None,
        "ekf_de": ekf_d[1] if ekf_d else None,
        "ekf_dz": ekf_d[2] if ekf_d else None,
        "ekf_mag": norm2(ekf_d[0], ekf_d[1]) if ekf_d else None,
        "roll_span_deg": math.degrees(span(roll)) if span(roll) is not None else None,
        "pitch_span_deg": math.degrees(span(pitch)) if span(pitch) is not None else None,
        "yaw_span_deg": math.degrees(span(yaw)) if span(yaw) is not None else None,
        "inliers_med": q(inliers, .5), "inliers_min": q(inliers, 0),
        "ratio_med": q(ratios, .5),
        "height_med": q(heights, .5), "height_min": q(heights, 0), "height_max": q(heights, 1),
        "luna_med": q(luna, .5), "luna_min": q(luna, 0), "luna_max": q(luna, 1),
        "luna_age_p95": q(luna_age, .95),
        "att_age_p95": q(att_age, .95),
        "ekf_age_p95": q(ekf_age, .95),
        "latency_p95": q(latency, .95),
        "dt_med_ms": (q(dtvals, .5) * 1000.0) if q(dtvals, .5) is not None else None,
        "drops": drops,
    }


def cv_pct(vals):
    a = [v for v in vals if v is not None and math.isfinite(v)]
    if len(a) < 2:
        return None
    mean = statistics.fmean(a)
    return 100.0 * statistics.pstdev(a) / abs(mean) if abs(mean) > 1e-12 else None


def print_run(i, s):
    print(f"\n===== RUN {i}: {s['path']} =====")
    print("LAYER 1 — FRONTEND")
    print(f"  rows={s['rows']} valid={s['valid_pct']:.2f}%  inliers med/min={fmt(s['inliers_med'])}/{fmt(s['inliers_min'])}  ratio med={fmt(s['ratio_med'])}")
    print(f"  dt median={fmt(s['dt_med_ms'])} ms  pipeline p95={fmt(s['latency_p95'])} ms  queue_drops_total={fmt(s['drops'])}")
    print("LAYER 2 — WORKED5 BODY")
    print(f"  coverage={s['w5_pct']:.2f}%  dX={fmt(s['w5_dx'],1000)} mm  dY={fmt(s['w5_dy'],1000)} mm  |XY|={fmt(s['w5_body_mag'],1000)} mm")
    print(f"  hcam med/min/max={fmt(s['height_med'],1000)}/{fmt(s['height_min'],1000)}/{fmt(s['height_max'],1000)} mm")
    print("LAYER 3 — BODY -> N/E")
    print(f"  dN={fmt(s['w5_dn'],1000)} mm  dE={fmt(s['w5_de'],1000)} mm  |NE|={fmt(s['w5_ne_mag'],1000)} mm")
    print(f"  roll/pitch/yaw span={fmt(s['roll_span_deg'])}/{fmt(s['pitch_span_deg'])}/{fmt(s['yaw_span_deg'])} deg")
    print("LAYER 4 — PUBLISH")
    print(f"  sent={s['sent_pct']:.2f}% ready={s['ready_pct']:.2f}% integrated angular x/y={fmt(s['pub_x_int'],1,6)}/{fmt(s['pub_y_int'],1,6)} rad  steps={s['pub_steps']}")
    print("LAYER 5 — EKF3")
    print(f"  coverage={s['ekf_pct']:.2f}%  dN={fmt(s['ekf_dn'],1000)} mm  dE={fmt(s['ekf_de'],1000)} mm  dZ={fmt(s['ekf_dz'],1000)} mm  |XY|={fmt(s['ekf_mag'],1000)} mm")
    print("TIMING / RANGE")
    print(f"  Luna med/min/max={fmt(s['luna_med'],1000)}/{fmt(s['luna_min'],1000)}/{fmt(s['luna_max'],1000)} mm")
    print(f"  age p95: Luna={fmt(s['luna_age_p95'])} ms ATT={fmt(s['att_age_p95'])} ms EKF={fmt(s['ekf_age_p95'])} ms")


def main():
    ap = argparse.ArgumentParser(description="Послойный анализ воспроизводимости monkeysStab")
    ap.add_argument("runs", nargs="+", help="CSV или каталог прогона")
    args = ap.parse_args()

    stats = [analyze(p) for p in args.runs]
    for i, s in enumerate(stats, 1):
        print_run(i, s)

    print("\n===== CROSS-RUN REPRODUCIBILITY =====")
    if len(stats) < 2:
        print("Нужны >=2 прогона для cross-run статистики.")
    else:
        for label, key in [
            ("WORKED5 BODY |XY|", "w5_body_mag"),
            ("WORKED5 N/E  |XY|", "w5_ne_mag"),
            ("EKF3         |XY|", "ekf_mag"),
            ("yaw span", "yaw_span_deg"),
            ("pitch span", "pitch_span_deg"),
            ("roll span", "roll_span_deg"),
        ]:
            vals = [s[key] for s in stats]
            scale = 1000.0 if "XY" in label else 1.0
            unit = "mm" if "XY" in label else "deg"
            rendered = ", ".join(fmt(v, scale) for v in vals)
            cv = cv_pct(vals)
            print(f"{label:20s}: [{rendered}] {unit}  CV={fmt(cv)}%")

    print("\n===== FORENSIC CONTRACT =====")
    print("Это RAW/LOG-derived facts. Физическое направление, истинная дистанция")
    print("и ошибка относительно GT из этих CSV не выводятся.")
    print("Классификация причины требует сравнения слоёв и, для ошибки %, отдельного GT.")


if __name__ == "__main__":
    main()
