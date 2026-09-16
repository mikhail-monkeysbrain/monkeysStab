#!/usr/bin/env python3
"""Validate ChArUco metric scale at known camera-to-board distances.

Uses a fixed intrinsic matrix and board dimensions. It does NOT recalibrate K.
For each saved frame it estimates board pose with solvePnP and reports camera
distance to the board plane. Intended to separate intrinsic calibration from
metric-scale errors elsewhere in the pipeline.
"""
import argparse
import csv
import glob
from pathlib import Path
import cv2
import numpy as np

SQUARE_M = 0.027315
MARKER_M = 0.020031
FX = 558.982
FY = 561.532
CX = 318.225
CY = 249.733
D = np.array([-0.15313, 0.87674, -0.00029407, -0.0048752, -1.6608], dtype=np.float64)
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
BOARD = cv2.aruco.CharucoBoard((5, 7), SQUARE_M, MARKER_M, DICT)
DETECTOR = cv2.aruco.CharucoDetector(BOARD)
K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]], dtype=np.float64)

def detect(gray):
    cc, ci, mc, mi = DETECTOR.detectBoard(gray)
    if cc is None or ci is None or len(cc) < 6:
        return None
    ids = ci.reshape(-1).astype(int)
    obj = BOARD.getChessboardCorners()[ids].astype(np.float32)
    img = cc.reshape(-1, 2).astype(np.float32)
    return obj, img

def estimate(path):
    im = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if im is None:
        return None
    z = detect(im)
    if z is None:
        return None
    obj, img = z
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    cam_in_board = -R.T @ tvec
    plane_distance = abs(float(cam_in_board[2, 0]))
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    err = np.linalg.norm(proj.reshape(-1,2) - img, axis=1)
    return len(obj), plane_distance, float(np.sqrt(np.mean(err * err)))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("directory")
    ap.add_argument("--known-mm", type=float, required=True,
                    help="physical perpendicular distance from camera optical center to board plane")
    args = ap.parse_args()
    root = Path(args.directory)
    files = sorted(glob.glob(str(root / "frame_*.jpg")))
    rows = []
    for p in files:
        r = estimate(p)
        if r is not None:
            rows.append((Path(p).name, *r))
    if not rows:
        raise SystemExit("No usable ChArUco frames")
    d = np.array([r[2] for r in rows])
    e = np.array([r[3] for r in rows])
    known = args.known_mm / 1000.0
    rel = d / known
    print("===== METRIC PNP VALIDATION =====")
    print("K fixed: fx=%.3f fy=%.3f cx=%.3f cy=%.3f" % (FX,FY,CX,CY))
    print("known perpendicular camera-center -> board-plane: %.3f mm" % args.known_mm)
    print("usable:", len(rows), "/", len(files))
    print("PnP plane distance mm min/median/max: %.2f %.2f %.2f" % tuple(d[[np.argmin(d), len(d)//2, np.argmax(d)]]*1000) if False else (d.min()*1000,np.median(d)*1000,d.max()*1000))
    print("scale estimated/known min/median/max: %.5f %.5f %.5f" % (rel.min(),np.median(rel),rel.max()))
    print("median metric error: %+.2f mm (%+.2f%%)" % ((np.median(d)-known)*1000,(np.median(rel)-1)*100))
    print("reprojection RMS px median/max: %.3f %.3f" % (np.median(e),e.max()))
    out = root / ("pnp_metric_%.1fmm.csv" % args.known_mm)
    with out.open("w", newline="") as f:
        w=csv.writer(f); w.writerow(["frame","corners","plane_distance_m","reproj_rms_px"])
        w.writerows(rows)
    print("Saved:", out)

if __name__ == "__main__":
    main()
