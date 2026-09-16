#!/usr/bin/env python3
"""Fast two-height ChArUco metric-scale test.

Acquisition and analysis are deliberately separated:
SPACE -> short camera burst into RAM -> camera acquisition stops -> ChArUco/PnP analysis.
Physical protocol: LOW -> HIGH(+49..50 mm) -> LOW -> HIGH(+49..50 mm).
"""
import argparse
import time
import cv2
import numpy as np

SQUARE_M = 0.027315
MARKER_M = 0.020031
FX, FY, CX, CY = 558.982, 561.532, 318.225, 249.733
D = np.array([-0.15313, 0.87674, -0.00029407, -0.0048752, -1.6608], np.float64)
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], np.float64)
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
BOARD = cv2.aruco.CharucoBoard((5, 7), SQUARE_M, MARKER_M, DICT)
DET = cv2.aruco.CharucoDetector(BOARD)
WINDOW = "JT-Zero ChArUco two-height BURST"


def get_z(frame, min_corners):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cc, ci, _, _ = DET.detectBoard(gray)
    if cc is None or ci is None or len(cc) < min_corners:
        return None
    ids = ci.reshape(-1).astype(int)
    obj = BOARD.getChessboardCorners()[ids].astype(np.float32)
    img = cc.reshape(-1, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    cam = -R.T @ tvec
    return abs(float(cam[2, 0])) * 1000.0, len(ids)


def wait_space(cap, text):
    print("\n" + text)
    print("Установи положение, дождись неподвижности и нажми SPACE в окне камеры.")
    while True:
        ok, fr = cap.read()
        if not ok:
            continue
        view = fr.copy()
        cv2.putText(view, text, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                    (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(view, "SPACE = BURST   Q/ESC = STOP", (12, 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW, view)
        k = cv2.waitKey(1) & 0xff
        if k == 32:
            return
        if k in (27, ord("q")):
            raise KeyboardInterrupt


def capture_burst(cap, n):
    frames = []
    t0 = time.perf_counter()
    for _ in range(n):
        ok, fr = cap.read()
        if ok:
            frames.append(fr)
    dt = time.perf_counter() - t0
    fps = len(frames) / dt if dt > 0 else 0.0
    print(f"BURST DONE: {len(frames)}/{n} frames in {dt:.3f}s ({fps:.1f} fps)")
    return frames


def analyze_burst(frames, label, min_corners):
    print(f"ANALYZING {label}: {len(frames)} frames ...")
    vals = []
    corner_counts = []
    t0 = time.perf_counter()
    for i, fr in enumerate(frames, 1):
        r = get_z(fr, min_corners)
        if r is not None:
            vals.append(r[0])
            corner_counts.append(r[1])
        if i % 30 == 0 or i == len(frames):
            print(f"  analyzed {i}/{len(frames)}, valid={len(vals)}", flush=True)
    dt = time.perf_counter() - t0
    if not vals:
        raise RuntimeError(
            f"{label}: 0 valid frames with >= {min_corners} ChArUco corners; "
            "physical position is NOT consumed, repeat this same position."
        )
    vals = np.asarray(vals, dtype=np.float64)
    med = float(np.median(vals))
    mad = float(np.median(np.abs(vals - med)))
    cmed = float(np.median(corner_counts))
    print(
        f"{label} DONE: valid={len(vals)}/{len(frames)} "
        f"corners_med={cmed:.1f} Zmedian={med:.2f} mm MAD={mad:.2f} mm "
        f"analysis={dt:.1f}s"
    )
    return vals


def acquire_position(cap, label, instruction, burst_frames, min_corners):
    while True:
        wait_space(cap, instruction)
        frames = capture_burst(cap, burst_frames)
        vals = analyze_burst(frames, label, min_corners)
        if len(vals) >= max(10, burst_frames // 10):
            return vals
        print(
            f"{label}: валидных кадров мало ({len(vals)}/{burst_frames}). "
            "Положение НЕ меняй; нажми SPACE для повторного burst."
        )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="/dev/video0")
    ap.add_argument("--step-min-mm", type=float, default=49.0)
    ap.add_argument("--step-max-mm", type=float, default=50.0)
    ap.add_argument("--burst-frames", type=int, default=120)
    ap.add_argument("--min-corners", type=int, default=8)
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit("Cannot open camera " + a.camera)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 120)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print("===== CAMERA =====")
    print(
        "requested 640x480 MJPG @120; backend reports "
        f"{cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} "
        f"@ {cap.get(cv2.CAP_PROP_FPS):.1f} fps"
    )
    print(
        "SPACE captures a RAM burst first. ChArUco/PnP runs only AFTER the burst, "
        "so analysis speed cannot stretch the physical capture interval."
    )

    try:
        low1 = acquire_position(
            cap, "LOW-1", "1/4 НИЖНЕЕ ПОЛОЖЕНИЕ",
            a.burst_frames, a.min_corners
        )
        high1 = acquire_position(
            cap, "HIGH-1", "2/4 ПОДНИМИ НА 49-50 mm",
            a.burst_frames, a.min_corners
        )
        low2 = acquire_position(
            cap, "LOW-2", "3/4 ВЕРНИ В НИЖНЕЕ ПОЛОЖЕНИЕ",
            a.burst_frames, a.min_corners
        )
        high2 = acquire_position(
            cap, "HIGH-2", "4/4 СНОВА ПОДНИМИ НА 49-50 mm",
            a.burst_frames, a.min_corners
        )
    except KeyboardInterrupt:
        print("\nABORTED")
        return
    finally:
        cap.release()
        cv2.destroyAllWindows()

    m = [float(np.median(x)) for x in (low1, high1, low2, high2)]
    d1 = m[1] - m[0]
    d2 = m[3] - m[2]
    dm = (d1 + d2) / 2.0
    physical_mid = (a.step_min_mm + a.step_max_mm) / 2.0

    print("\n===== RESULT =====")
    print(f"LOW-1  median Z: {m[0]:.2f} mm")
    print(f"HIGH-1 median Z: {m[1]:.2f} mm")
    print(f"LOW-2  median Z: {m[2]:.2f} mm")
    print(f"HIGH-2 median Z: {m[3]:.2f} mm")
    print(f"delta #1: {d1:.2f} mm")
    print(f"delta #2: {d2:.2f} mm")
    print(f"delta mean: {dm:.2f} mm")
    print(f"physical step: {a.step_min_mm:.1f} .. {a.step_max_mm:.1f} mm")
    print(f"repeatability |d1-d2|: {abs(d1-d2):.2f} mm")
    print(
        f"scale vs {physical_mid:.1f}-mm midpoint: {dm/physical_mid:.4f}  "
        f"error={(dm/physical_mid-1.0)*100:+.2f}%"
    )


if __name__ == "__main__":
    main()
