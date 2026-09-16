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
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, DICT)
    if marker_ids is None or len(marker_ids) < 2:
        return None
    _, cc, ci = cv2.aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, gray, BOARD
    )
    if cc is None or ci is None or len(ci) < min_corners:
        return None
    ids = ci.reshape(-1).astype(int)
    obj = OBJ[ids].astype(np.float32)
    img = cc.reshape(-1, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(
        obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok:
        return None
    return abs(float(tvec[2, 0])) * 1000.0, len(ids)

