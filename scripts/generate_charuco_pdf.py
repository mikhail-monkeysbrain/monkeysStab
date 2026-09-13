#!/usr/bin/env python3
from pathlib import Path
import sys, zlib
import cv2
import numpy as np

OUT=Path(sys.argv[1] if len(sys.argv)>1 else "charuco.pdf")
SX,SY=5,7
SQUARE_MM=30.0
MARKER_MM=22.0
DPI=300
A4_W_MM,A4_H_MM=210.0,297.0
PW=round(A4_W_MM/25.4*DPI)
PH=round(A4_H_MM/25.4*DPI)
BW=round(SX*SQUARE_MM/25.4*DPI)
BH=round(SY*SQUARE_MM/25.4*DPI)

D=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board=cv2.aruco.CharucoBoard((SX,SY),SQUARE_MM/1000.0,MARKER_MM/1000.0,D)
b=board.generateImage((BW,BH),marginSize=0,borderBits=1)
page=np.full((PH,PW),255,np.uint8)
x=(PW-BW)//2
y=(PH-BH)//2+35
page[y:y+BH,x:x+BW]=b

def put(txt, yy, scale=.55, thick=1):
    font=cv2.FONT_HERSHEY_SIMPLEX
    (tw,_),_=cv2.getTextSize(txt,font,scale,thick)
    cv2.putText(page,txt,((PW-tw)//2,yy),font,scale,0,thick,cv2.LINE_AA)

put("monkeysStab ChArUco calibration board",65,.7,2)
put("5x7 squares | square 30.0 mm | marker 22.0 mm | DICT_4X4_50",95,.48,1)
put("PRINT AT 100% / ACTUAL SIZE - disable Fit to page",120,.48,1)

ry=min(PH-70,y+BH+45)
rx=(PW-round(100/25.4*DPI))//2
rlen=round(100/25.4*DPI)
cv2.line(page,(rx,ry),(rx+rlen,ry),0,2)
for i in range(11):
    xx=rx+round(i*10/25.4*DPI)
    cv2.line(page,(xx,ry-12),(xx,ry+12),0,2)
put("100 mm reference - verify after printing",ry+38,.42,1)

raw=page.tobytes()
comp=zlib.compress(raw,9)
W_PT=A4_W_MM/25.4*72.0
H_PT=A4_H_MM/25.4*72.0
objs=[]

def obj(data):
    if isinstance(data,str):
        data=data.encode("ascii")
    objs.append(data)
    return len(objs)

obj("<< /Type /Catalog /Pages 2 0 R >>")
obj("<< /Type /Pages /Kids [3 0 R] /Count 1 >>")
obj("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.4f %.4f] /Resources << /XObject << /Im0 4 0 R >> >> /Contents 5 0 R >>"%(W_PT,H_PT))
img_hdr=("<< /Type /XObject /Subtype /Image /Width %d /Height %d /ColorSpace /DeviceGray /BitsPerComponent 8 /Filter /FlateDecode /Length %d >>\nstream\n"%(PW,PH,len(comp))).encode("ascii")
obj(img_hdr+comp+b"\nendstream")
content=("q\n%.4f 0 0 %.4f 0 0 cm\n/Im0 Do\nQ\n"%(W_PT,H_PT)).encode("ascii")
obj(("<< /Length %d >>\nstream\n"%len(content)).encode("ascii")+content+b"endstream")

pdf=bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
off=[0]
for i,data in enumerate(objs,1):
    off.append(len(pdf))
    pdf+=("%d 0 obj\n"%i).encode("ascii")+data+b"\nendobj\n"
xref=len(pdf)
pdf+=("xref\n0 %d\n0000000000 65535 f \n"%(len(objs)+1)).encode("ascii")
for o in off[1:]:
    pdf+=("%010d 00000 n \n"%o).encode("ascii")
pdf+=("trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n"%(len(objs)+1,xref)).encode("ascii")
OUT.parent.mkdir(parents=True,exist_ok=True)
OUT.write_bytes(pdf)
print(OUT)
