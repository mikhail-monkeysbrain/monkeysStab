#!/usr/bin/env python3
"""Two-height ChArUco metric-scale test for the fixed JT-Zero 7x5 board."""
import argparse
import time
import cv2
import numpy as np

W, H = 640, 480
SQUARE_M = 0.027315
MARKER_M = 0.020031
FX, FY, CX, CY = 558.982, 561.532, 318.225, 249.733
K = np.array([[FX,0,CX],[0,FY,CY],[0,0,1]], np.float64)
D = np.array([-0.15313,0.87674,-0.00029407,-0.0048752,-1.6608], np.float64)
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try:
    BOARD = cv2.aruco.CharucoBoard((7,5), SQUARE_M, MARKER_M, DICT)
except TypeError:
    BOARD = cv2.aruco.CharucoBoard_create(7,5,SQUARE_M,MARKER_M,DICT)
OBJ = np.asarray(BOARD.getChessboardCorners(), np.float32)
WINDOW = "JT-Zero ChArUco two-height"

def detect(gray):
    mc, mi, _ = cv2.aruco.detectMarkers(gray, DICT)
    if mi is None or len(mi) < 2:
        return mc, mi, None, None
    _, cc, ci = cv2.aruco.interpolateCornersCharuco(mc, mi, gray, BOARD)
    return mc, mi, cc, ci

def frame_z(frame, min_corners):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, _, cc, ci = detect(gray)
    if cc is None or ci is None or len(ci) < min_corners:
        return None
    ids = ci.reshape(-1).astype(int)
    obj = OBJ[ids]
    img = cc.reshape(-1,2).astype(np.float32)
    ok, _, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    return abs(float(tvec[2,0]))*1000.0, len(ids)

def wait_for_space(cap, instruction, min_corners):
    print("\n" + instruction, flush=True)
    print("Держи положение неподвижно. SPACE = снять burst; Q/ESC = выход.", flush=True)
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mc, mi, cc, ci = detect(gray)
        nc = 0 if ci is None else len(ci)
        vis = frame.copy()
        if mi is not None:
            cv2.aruco.drawDetectedMarkers(vis, mc, mi)
        if cc is not None:
            cv2.aruco.drawDetectedCornersCharuco(vis, cc, ci)
        cv2.putText(vis, instruction, (8,25), cv2.FONT_HERSHEY_SIMPLEX, .48, (255,255,255), 1)
        cv2.putText(vis, f"corners {nc} / need {min_corners}", (8,48),
                    cv2.FONT_HERSHEY_SIMPLEX, .48,
                    (0,255,0) if nc >= min_corners else (0,0,255), 2)
        cv2.imshow(WINDOW, vis)
        key = cv2.waitKey(1) & 255
        if key == 32:
            return
        if key in (27, ord("q"), ord("Q")):
            raise KeyboardInterrupt

def capture_burst(cap, n):
    frames = []
    t0 = time.perf_counter()
    for _ in range(n):
        ok, frame = cap.read()
        if ok:
            frames.append(frame)
    dt = time.perf_counter() - t0
    fps = len(frames)/dt if dt > 0 else 0.0
    print(f"BURST: {len(frames)}/{n} frames, {dt:.3f}s, {fps:.1f} fps", flush=True)
    return frames

def analyze(frames, label, min_corners):
    vals = []
    counts = []
    for frame in frames:
        r = frame_z(frame, min_corners)
        if r is not None:
            vals.append(r[0]); counts.append(r[1])
    if not vals:
        print(f"{label}: valid=0/{len(frames)} -> REPEAT SAME POSITION", flush=True)
        return None
    a = np.asarray(vals)
    med = float(np.median(a))
    mad = float(np.median(np.abs(a-med)))
    print(f"{label}: valid={len(a)}/{len(frames)} corners_med={np.median(counts):.1f} "
          f"Zmedian={med:.2f}mm MAD={mad:.2f}mm", flush=True)
    return a

def acquire(cap, label, instruction, n, min_corners):
    while True:
        wait_for_space(cap, instruction, min_corners)
        frames = capture_burst(cap, n)
        vals = analyze(frames, label, min_corners)
        if vals is not None and len(vals) >= max(10, n//10):
            return vals
        print("Недостаточно валидных кадров. Положение НЕ меняй; повтори SPACE.", flush=True)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="/dev/video0")
    ap.add_argument("--step-min-mm", type=float, default=49.0)
    ap.add_argument("--step-max-mm", type=float, default=50.0)
    ap.add_argument("--burst-frames", type=int, default=120)
    ap.add_argument("--min-corners", type=int, default=8)
    args = ap.parse_args()

    print("START test_charuco_two_height_scale", flush=True)
    print("BOARD=7x5 DICT_4X4_50 square=27.315mm marker=20.031mm", flush=True)
    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, 120)
    if not cap.isOpened():
        raise SystemExit("camera open failed")
    print(f"CAMERA backend: {cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x"
          f"{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} @ "
          f"{cap.get(cv2.CAP_PROP_FPS):.1f} fps", flush=True)

    try:
        low1 = acquire(cap, "LOW-1", "1/4 LOW", args.burst_frames, args.min_corners)
        high1 = acquire(cap, "HIGH-1", "2/4 RAISE UAV 49-50 mm", args.burst_frames, args.min_corners)
        low2 = acquire(cap, "LOW-2", "3/4 RETURN TO LOW", args.burst_frames, args.min_corners)
        high2 = acquire(cap, "HIGH-2", "4/4 RAISE UAV 49-50 mm AGAIN", args.burst_frames, args.min_corners)
    except KeyboardInterrupt:
        print("\nABORTED", flush=True)
        return
    finally:
        cap.release()
        cv2.destroyAllWindows()

    z0,z1,z2,z3 = [float(np.median(v)) for v in (low1,high1,low2,high2)]
    # Raising camera away from a horizontal board normally increases distance.
    d1 = abs(z1-z0)
    d2 = abs(z3-z2)
    dm = (d1+d2)/2.0
    ref = (args.step_min_mm+args.step_max_mm)/2.0
    print("\n===== RESULT =====")
    print(f"LOW-1={z0:.2f} HIGH-1={z1:.2f} delta1={d1:.2f} mm")
    print(f"LOW-2={z2:.2f} HIGH-2={z3:.2f} delta2={d2:.2f} mm")
    print(f"delta mean={dm:.2f} mm")
    print(f"repeatability |d1-d2|={abs(d1-d2):.2f} mm")
    print(f"physical step={args.step_min_mm:.1f}..{args.step_max_mm:.1f} mm")
    print(f"scale vs midpoint {ref:.1f}mm = {dm/ref:.5f} ({(dm/ref-1)*100:+.2f}%)")

if __name__ == "__main__":
    main()
