#!/usr/bin/env python3
"""Export exactly the accepted A->B Astra correspondences and sensor state for C++ metric parity."""
import argparse,csv
from pathlib import Path
import cv2,numpy as np
import astra_reference_offline as a

def main():
 ap=argparse.ArgumentParser();ap.add_argument('dataset');ap.add_argument('output');x=ap.parse_args();p=Path(x.dataset)
 rows=list(csv.DictReader(open(p/'optical_flow_mavlink.csv',newline='')));frames=a.read_raw(p/'frames.mjpgbin');t=np.array([int(r['camera_ts_ns']) for r in rows],np.float64)*1e-9
 st=[];rpy=[];rt=[];rv=[];events={}
 for idx,r in enumerate(rows):
  age=float(r['fc_gyro_age_ms']);send=int(r['flow_send_ns'])
  if age>=0 and send>0:st.append(send*1e-9-age*.001);rpy.append([float(r['fc_roll']),float(r['fc_pitch']),float(r['fc_yaw'])])
  rt.append(int(r['mono_ns'])*1e-9-float(r['luna_age_ms'])*.001);rv.append(float(r['luna_m']))
  ev=str(r.get('return_event','')).strip()
  if ev and ev not in ('0','0.0'):events[int(float(ev))]=idx
 ang=a.interp_unwrapped(t,np.array(st),np.array(rpy));rb=np.array([a.rotation(*z) for z in ang]);order=np.argsort(rt);rt=np.asarray(rt)[order];rv=np.asarray(rv)[order];u=np.r_[True,np.diff(rt)>1e-8];rng=np.interp(t,rt[u],rv[u]);A,B=events[1],events[2]
 anchor=0;im=cv2.imdecode(np.frombuffer(frames[0][1],np.uint8),0);accepted=[]
 for j in range(1,B+1):
  curr=cv2.imdecode(np.frombuffer(frames[j][1],np.uint8),0);pa,pb,info=a.register(im,curr,force_sift=abs(t[j]-t[anchor])>.1)
  if info['ok']:
   d=a.astra_metric(pa,pb,rb[anchor],rb[j],rng[anchor])
   if np.all(np.isfinite(d)):
    if j>A:
     # For a bridge crossing A, Astra position interpolation means only the
     # post-A fraction belongs to A->B. Canonical A is itself an accepted anchor,
     # so this is normally a full interval.
     accepted.append((anchor,j,pa,pb,ang[anchor],ang[j],rng[anchor]))
    anchor=j;im=curr
  elif abs(t[j]-t[anchor])>2.0:anchor=j;im=curr
 with open(x.output,'w',newline='') as f:
  w=csv.writer(f);w.writerow(['interval','roll0','pitch0','yaw0','roll1','pitch1','yaw1','range0','ax','ay','bx','by'])
  for k,(i,j,pa,pb,r0,r1,rg) in enumerate(accepted):
   for P,Q in zip(pa,pb):w.writerow([k,*r0,*r1,rg,P[0],P[1],Q[0],Q[1]])
 print(f'exported intervals={len(accepted)} points={sum(len(z[2]) for z in accepted)} A/B={A}/{B} -> {x.output}')
if __name__=='__main__':main()
