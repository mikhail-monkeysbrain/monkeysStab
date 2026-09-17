#!/usr/bin/env python3
"""Export one RAW frame pair and print the Python Astra registration reference."""
import argparse
from pathlib import Path
import cv2,numpy as np
import astra_reference_offline as a

ap=argparse.ArgumentParser()
ap.add_argument('dataset'); ap.add_argument('i',type=int); ap.add_argument('j',type=int)
ap.add_argument('outdir'); ap.add_argument('--force-sift',action='store_true')
x=ap.parse_args(); p=Path(x.dataset); out=Path(x.outdir); out.mkdir(parents=True,exist_ok=True)
f=a.read_raw(p/'frames.mjpgbin')
if not (0<=x.i<len(f) and 0<=x.j<len(f)): raise SystemExit(f'frame indexes out of range: {len(f)}')
im0=cv2.imdecode(np.frombuffer(f[x.i][1],np.uint8),0); im1=cv2.imdecode(np.frombuffer(f[x.j][1],np.uint8),0)
cv2.imwrite(str(out/'prev.png'),im0); cv2.imwrite(str(out/'curr.png'),im1)
pa,pb,info=a.register(im0,im1,force_sift=x.force_sift)
print('ASTRA PYTHON FRONTEND REFERENCE')
print(f"i/j={x.i}/{x.j} method={info['method']} ok={int(info['ok'])} n={info['n']} inliers={info['inliers']} cells={info['cells']} residual={info['residual']:.9f}")
for k,(u,v) in enumerate(zip(pa,pb)): print(f'{k},{u[0]:.9f},{u[1]:.9f},{v[0]:.9f},{v[1]:.9f}')
print(f'images -> {out}')
