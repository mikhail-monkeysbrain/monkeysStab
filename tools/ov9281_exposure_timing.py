#!/usr/bin/env python3
import argparse
import os
import select
import subprocess
import time
import cv2
import numpy as np

DEFAULT_CAMERA="/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0"

def set_ctrl(dev, expr):
    subprocess.run(["v4l2-ctl","-d",dev,"--set-ctrl="+expr],
                   check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

def get_ctrl(dev, name):
    p=subprocess.run(["v4l2-ctl","-d",dev,"--get-ctrl="+name],
                     check=True,text=True,capture_output=True)
    return p.stdout.strip()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--camera",default=DEFAULT_CAMERA)
    ap.add_argument("--values",default="20,30,40,50,60,70,80,90,100,125,150,200")
    ap.add_argument("--seconds",type=float,default=1.5)
    args=ap.parse_args()
    vals=[int(x) for x in args.values.split(",") if x.strip()]

    set_ctrl(args.camera,"auto_exposure=1")
    set_ctrl(args.camera,"exposure_dynamic_framerate=0")
    set_ctrl(args.camera,"gain=0")

    cap=cv2.VideoCapture(args.camera,cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    cap.set(cv2.CAP_PROP_FPS,100)
    if not cap.isOpened():
        raise SystemExit("ERROR: camera open failed")

    print("requested_fps=",cap.get(cv2.CAP_PROP_FPS))
    print("exp frames fps dt_med_ms dt_p95_ms dt_max_ms accepted")
    for exp in vals:
        set_ctrl(args.camera,f"exposure_time_absolute={exp}")
        accepted=get_ctrl(args.camera,"exposure_time_absolute").split(":")[-1].strip()
        for _ in range(20):
            cap.grab()
        stamps=[]
        t_end=time.monotonic()+args.seconds
        while time.monotonic()<t_end:
            ok,_=cap.read()
            if ok:
                stamps.append(time.monotonic_ns())
        if len(stamps)<2:
            print(f"{exp:3d} {len(stamps):6d} ERROR")
            continue
        d=np.diff(np.asarray(stamps,dtype=np.int64))*1e-6
        span=(stamps[-1]-stamps[0])*1e-9
        fps=(len(stamps)-1)/span if span>0 else 0.0
        print(f"{exp:3d} {len(stamps):6d} {fps:7.2f} {np.median(d):9.3f} "
              f"{np.percentile(d,95):9.3f} {np.max(d):9.3f} {accepted}")

    cap.release()
    set_ctrl(args.camera,"exposure_time_absolute=50")
    set_ctrl(args.camera,"gain=0")
    print("DONE: restored manual exposure=50 gain=0")

if __name__=="__main__":
    main()
