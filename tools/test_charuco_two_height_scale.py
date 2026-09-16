#!/usr/bin/env python3
"""Two-height ChArUco metric-scale test: LOW -> HIGH -> LOW -> HIGH."""
import argparse, time
import cv2
import numpy as np

SQUARE_M=0.027315
MARKER_M=0.020031
FX,FY,CX,CY=558.982,561.532,318.225,249.733
D=np.array([-0.15313,0.87674,-0.00029407,-0.0048752,-1.6608],np.float64)
K=np.array([[FX,0,CX],[0,FY,CY],[0,0,1]],np.float64)
DICT=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
BOARD=cv2.aruco.CharucoBoard((5,7),SQUARE_M,MARKER_M,DICT)
DET=cv2.aruco.CharucoDetector(BOARD)

def get_z(frame,min_corners):
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    cc,ci,mc,mi=DET.detectBoard(gray)
    if cc is None or ci is None or len(cc)<min_corners:return None
    ids=ci.reshape(-1).astype(int)
    obj=BOARD.getChessboardCorners()[ids].astype(np.float32)
    img=cc.reshape(-1,2).astype(np.float32)
    ok,rvec,tvec=cv2.solvePnP(obj,img,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:return None
    R,_=cv2.Rodrigues(rvec)
    cam=-R.T@tvec
    return abs(float(cam[2,0]))*1000.0,len(ids)

def collect(cap,label,n,min_corners):
    vals=[]
    print(f"\n{label}: НЕ ДВИГАЙ ДРОН. Собираю {n} хороших кадров...")
    while len(vals)<n:
        ok,fr=cap.read()
        if not ok:continue
        r=get_z(fr,min_corners)
        if r:
            vals.append(r[0])
            if len(vals)%10==0 or len(vals)==n:
                print(f"  {len(vals)}/{n}  Zmedian={np.median(vals):.2f} mm")
        cv2.imshow("JT-Zero ChArUco two-height test",fr)
        if (cv2.waitKey(1)&0xff) in (27,ord('q')):raise KeyboardInterrupt
    return np.asarray(vals)

def wait_space(cap,text):
    print("\n"+text)
    print("Когда положение готово и дрон НЕПОДВИЖЕН — нажми SPACE в окне камеры.")
    while True:
        ok,fr=cap.read()
        if not ok:continue
        cv2.putText(fr,text,(15,35),cv2.FONT_HERSHEY_SIMPLEX,.62,(255,255,255),2,cv2.LINE_AA)
        cv2.imshow("JT-Zero ChArUco two-height test",fr)
        k=cv2.waitKey(1)&0xff
        if k==32:return
        if k in (27,ord('q')):raise KeyboardInterrupt

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--camera",default="/dev/video0")
    ap.add_argument("--step-min-mm",type=float,default=49.0)
    ap.add_argument("--step-max-mm",type=float,default=50.0)
    ap.add_argument("--samples",type=int,default=60)
    ap.add_argument("--min-corners",type=int,default=8)
    a=ap.parse_args()
    cap=cv2.VideoCapture(a.camera)
    if not cap.isOpened():raise SystemExit("Cannot open camera "+a.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*"MJPG"))
    try:
        wait_space(cap,"1/4 НИЖНЕЕ положение")
        low1=collect(cap,"LOW-1",a.samples,a.min_corners)
        wait_space(cap,"2/4 ПОДНИМИ РОВНО НА 49-50 мм")
        high1=collect(cap,"HIGH-1",a.samples,a.min_corners)
        wait_space(cap,"3/4 ВЕРНИ В НИЖНЕЕ положение")
        low2=collect(cap,"LOW-2",a.samples,a.min_corners)
        wait_space(cap,"4/4 СНОВА ПОДНИМИ НА 49-50 мм")
        high2=collect(cap,"HIGH-2",a.samples,a.min_corners)
    except KeyboardInterrupt:
        print("\nABORTED");return
    finally:
        cap.release();cv2.destroyAllWindows()
    m=[float(np.median(x)) for x in (low1,high1,low2,high2)]
    d1=m[1]-m[0];d2=m[3]-m[2];dm=(d1+d2)/2
    physical_mid=(a.step_min_mm+a.step_max_mm)/2
    print("\n===== RESULT =====")
    print("LOW-1  median Z: %.2f mm"%m[0]);print("HIGH-1 median Z: %.2f mm"%m[1])
    print("LOW-2  median Z: %.2f mm"%m[2]);print("HIGH-2 median Z: %.2f mm"%m[3])
    print("delta #1: %.2f mm"%d1);print("delta #2: %.2f mm"%d2)
    print("delta mean: %.2f mm"%dm)
    print("physical step: %.1f .. %.1f mm"%(a.step_min_mm,a.step_max_mm))
    print("repeatability |d1-d2|: %.2f mm"%abs(d1-d2))
    print("scale vs 49.5-mm midpoint: %.4f  error=%+.2f%%"%(dm/physical_mid,(dm/physical_mid-1)*100))
    if a.step_min_mm<=dm<=a.step_max_mm:
        print("VERDICT: measured PnP step lies inside the stated physical 49-50 mm interval")
    else:
        print("VERDICT: measured PnP step lies outside the stated physical 49-50 mm interval")

if __name__=="__main__":main()
