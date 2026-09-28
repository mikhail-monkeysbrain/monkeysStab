#!/usr/bin/env python3
import argparse
import subprocess
import time
import cv2
import numpy as np

DEFAULT_CAMERA = "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0"

def ctrl(dev, expr):
    subprocess.run(["v4l2-ctl", "-d", dev, "--set-ctrl=" + expr], check=True,
                   stdout=subprocess.DEVNULL)

def main():
    ap=argparse.ArgumentParser(description="OV9281 exposure sweep for optical-flow ROI")
    ap.add_argument("--camera", default=DEFAULT_CAMERA)
    ap.add_argument("--values", default="2,3,5,8,12,16,20,30,40,50")
    ap.add_argument("--settle", type=int, default=20)
    ap.add_argument("--frames", type=int, default=40)
    args=ap.parse_args()
    values=[int(x) for x in args.values.split(",") if x.strip()]

    ctrl(args.camera, "auto_exposure=1")
    ctrl(args.camera, "gain=0")

    cap=cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 100)
    if not cap.isOpened():
        raise SystemExit("ERROR: camera open failed")

    print("exp frames median p90 p95 p99 clip250% dark5% grad_mean corners")
    for exp in values:
        ctrl(args.camera, f"exposure_time_absolute={exp}")
        for _ in range(args.settle):
            cap.grab()
        med=[]; p90=[]; p95=[]; p99=[]; clip=[]; dark=[]; grad=[]; corners=[]
        got=0
        while got < args.frames:
            ok, frame=cap.read()
            if not ok:
                continue
            if frame.ndim == 3:
                gray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray=frame
            h,w=gray.shape[:2]
            roi=gray[int(.32*h):int(.90*h), int(.20*w):int(.80*w)]
            f=roi.astype(np.float32)
            med.append(float(np.median(roi)))
            p90.append(float(np.percentile(roi,90)))
            p95.append(float(np.percentile(roi,95)))
            p99.append(float(np.percentile(roi,99)))
            clip.append(float(np.mean(roi>=250)*100.0))
            dark.append(float(np.mean(roi<=5)*100.0))
            gx=cv2.Sobel(f,cv2.CV_32F,1,0,ksize=3)
            gy=cv2.Sobel(f,cv2.CV_32F,0,1,ksize=3)
            grad.append(float(np.mean(cv2.magnitude(gx,gy))))
            pts=cv2.goodFeaturesToTrack(roi,500,0.01,7)
            corners.append(0 if pts is None else len(pts))
            got+=1
        def M(a): return float(np.median(a))
        print(f"{exp:3d} {got:6d} {M(med):6.1f} {M(p90):6.1f} {M(p95):6.1f} {M(p99):6.1f} "
              f"{M(clip):8.3f} {M(dark):7.3f} {M(grad):9.2f} {int(round(M(corners))):7d}", flush=True)

    cap.release()
    ctrl(args.camera, "exposure_time_absolute=50")
    ctrl(args.camera, "gain=0")
    print("DONE: restored manual exposure=50 gain=0")

if __name__ == "__main__":
    main()
