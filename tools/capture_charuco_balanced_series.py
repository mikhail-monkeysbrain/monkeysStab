#!/usr/bin/env python3
"""ChArUco SERIES_C: сбалансированный сбор по реально наблюдаемой нижней части кадра.

Кольцо стенда не моделируется прямоугольной маской и не используется как зона
калибровки. Для устранения горизонтального spatial bias собираются три полосы
LEFT/CENTER/RIGHT по центру обнаруженных ChArUco corners. Вертикальное положение,
масштаб и наклон должны различаться внутри каждой полосы.
"""
import argparse, csv, time
from pathlib import Path
import cv2
import numpy as np

S = .027315
M = .020031
W, H = 640, 480
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try:
    board = cv2.aruco.CharucoBoard((7, 5), S, M, dic)
except TypeError:
    board = cv2.aruco.CharucoBoard_create(7, 5, S, M, dic)

def detect(gray):
    mc, mi, _ = cv2.aruco.detectMarkers(gray, dic)
    if mi is None or len(mi) < 2:
        return mc, mi, None, None
    _, cc, ci = cv2.aruco.interpolateCornersCharuco(mc, mi, gray, board)
    return mc, mi, cc, ci

def descriptor(cc):
    p = cc.reshape(-1, 2)
    x0, y0 = p.min(0)
    x1, y1 = p.max(0)
    return np.array([
        (x1 - x0) / W, (y1 - y0) / H,
        p[:, 1].mean() / H,
        np.std(p[:, 0]) / W, np.std(p[:, 1]) / H
    ], dtype=float)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="/dev/video0")
    ap.add_argument("--per-band", type=int, default=8)
    ap.add_argument("--min-corners", type=int, default=8)
    args = ap.parse_args()

    out = Path(f"/home/vio/charuco_series_C_{time.strftime('%Y%m%d_%H%M%S')}")
    out.mkdir(parents=True)

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, 120)
    if not cap.isOpened():
        raise SystemExit("camera open failed")

    counts = np.zeros(3, dtype=int)
    descs = [[] for _ in range(3)]
    meta = []
    last = 0.0
    total = 3 * args.per_band
    names = ("LEFT", "CENTER", "RIGHT")
    print(f"SERIES_C stand-aware horizontal coverage: {args.per_band}/band = {total}.")
    print("Кольцо стенда НЕ считается прямоугольной маской. AUTO capture; Q/ESC stop.")

    while counts.sum() < total:
        ok, im = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        mc, mi, cc, ci = detect(gray)
        nm = 0 if mi is None else len(mi)
        nc = 0 if ci is None else len(ci)
        good = False
        reason = f"need >={args.min_corners} corners"
        band = None
        cx = cy = nov = 0.0
        d = None

        if cc is not None and nc >= args.min_corners:
            p = cc.reshape(-1, 2)
            cx, cy = p.mean(0)
            band = min(2, max(0, int(cx / (W / 3))))
            d = descriptor(cc)
            if counts[band] >= args.per_band:
                reason = f"{names[band]} full"
            else:
                old = descs[band]
                nov = 999.0 if not old else min(
                    float(np.linalg.norm((d - o) * np.array([2., 2., 1.5, 1., 1.])))
                    for o in old
                )
                if nov >= .040:
                    good = True
                    reason = f"AUTO READY {names[band]}"
                else:
                    reason = f"same geometry {nov:.3f}"

        vis = im.copy()
        if mi is not None:
            cv2.aruco.drawDetectedMarkers(vis, mc, mi)
        if cc is not None:
            cv2.aruco.drawDetectedCornersCharuco(vis, cc, ci)
        for x in (W // 3, 2 * W // 3):
            cv2.line(vis, (x, 0), (x, H - 1), (255, 255, 255), 1)
        for i, name in enumerate(names):
            color = (0, 255, 0) if counts[i] >= args.per_band else (255, 255, 255)
            cv2.putText(vis, f"{name} {counts[i]}/{args.per_band}",
                        (i * (W // 3) + 8, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, .50, color, 2)
        cv2.putText(vis, f"TOTAL {counts.sum()}/{total} corners {nc} {reason}",
                    (8, H - 12), cv2.FONT_HERSHEY_SIMPLEX, .46,
                    (0, 255, 0) if good else (0, 0, 255), 2)

        cv2.imshow("ChArUco SERIES_C STAND-AWARE", vis)
        key = cv2.waitKey(1) & 255
        now = time.time()
        if good and band is not None and now - last > .65:
            n = int(counts.sum()) + 1
            path = out / f"frame_{n:03d}.jpg"
            cv2.imwrite(str(path), im)
            counts[band] += 1
            descs[band].append(d)
            meta.append([path.name, names[band], nm, nc, cx / W, cy / H, *d, nov])
            last = now
            print(f"SAVED {path.name} band={names[band]} "
                  f"{counts[band]}/{args.per_band} corners={nc} novelty={nov:.3f}")
        if key in (27, ord("q"), ord("Q")):
            break

    cap.release()
    cv2.destroyAllWindows()
    with (out / "capture.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["file", "band", "markers", "corners", "cx_norm", "cy_norm",
                    "width_norm", "height_norm", "mean_y_norm",
                    "spread_x", "spread_y", "novelty"])
        w.writerows(meta)
    print("Saved directory:", out)
    print("accepted:", int(counts.sum()), "/", total)
    print("band counts:", dict(zip(names, counts.tolist())))

if __name__ == "__main__":
    main()
