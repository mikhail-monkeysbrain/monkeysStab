#!/usr/bin/env python3
"""Автономный DataFlash-аудит IMU/XKF1/XKF2 и AHRS_TRIM.

Без pymavlink. Декодирует FMT из BIN, печатает статистику IMU,
XKF1/XKF2 и сравнивает BASE с точным inverse AHRS_TRIM (R^T).
"""
import argparse, math, statistics, struct
from pathlib import Path

HEAD=b"\xA3\x95"; FMT_TYPE=0x80; FMT_LEN=89
DF={"b":("b",1),"B":("B",1),"h":("h",2),"H":("H",2),"i":("i",4),"I":("I",4),
    "q":("q",8),"Q":("Q",8),"f":("f",4),"d":("d",8),"c":("h",2),"C":("H",2),
    "e":("i",4),"E":("I",4),"L":("i",4),"M":("B",1),"n":("4s",4),
    "N":("16s",16),"Z":("64s",64),"a":("32s",32)}

def cstr(b): return b.split(b"\0",1)[0].decode("ascii","replace")
def parse_fmt(r):
    return {"type":r[3],"length":r[4],"name":cstr(r[5:9]),
            "format":cstr(r[9:25]),"columns":cstr(r[25:89]).split(",")}
def unpack(fmt,p): return struct.unpack("<"+"".join(DF[x][0] for x in fmt),p)
def scale(row,fmt,cols):
    for k,ch in zip(cols,fmt):
        if ch=="c": row[k]/=100.0
        elif ch=="C": row[k]/=100.0
        elif ch=="e": row[k]/=100.0
        elif ch=="E": row[k]/=100.0
    return row
def stat(xs):
    return (statistics.fmean(xs),statistics.median(xs),
            statistics.pstdev(xs) if len(xs)>1 else 0.0,min(xs),max(xs))
def show(label,xs):
    a=stat(xs); print(f"{label}: n={len(xs)} mean={a[0]:+.6f} median={a[1]:+.6f} sd={a[2]:.6f} min={a[3]:+.6f} max={a[4]:+.6f}")
def mat321(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
            [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
            [-sp,cp*sr,cp*cr]]
def mv(M,v): return [sum(M[i][j]*v[j] for j in range(3)) for i in range(3)]
def tr(M): return [list(x) for x in zip(*M)]
def median3(rows,keys): return [statistics.median(float(r[k]) for r in rows) for k in keys]

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("bin"); ap.add_argument("--core",type=int,default=0)
    ap.add_argument("--trim-x",type=float,default=-0.03428453207)
    ap.add_argument("--trim-y",type=float,default=-0.0220823437)
    args=ap.parse_args(); data=Path(args.bin).read_bytes()

    fmts={}; i=0
    while i+3<=len(data):
        if data[i:i+2]!=HEAD: i+=1; continue
        typ=data[i+2]
        if typ==FMT_TYPE and i+FMT_LEN<=len(data):
            f=parse_fmt(data[i:i+FMT_LEN]); fmts[f["type"]]=f; i+=FMT_LEN; continue
        f=fmts.get(typ); i += f["length"] if f and f["length"]>=3 and i+f["length"]<=len(data) else 1

    wanted={"IMU":[],"XKF1":[],"XKF2":[]}
    i=0
    while i+3<=len(data):
        if data[i:i+2]!=HEAD: i+=1; continue
        typ=data[i+2]
        if typ==FMT_TYPE: i+=FMT_LEN if i+FMT_LEN<=len(data) else 1; continue
        f=fmts.get(typ)
        if not f or f["length"]<3 or i+f["length"]>len(data): i+=1; continue
        if f["name"] in wanted:
            vals=unpack(f["format"],data[i+3:i+f["length"]])
            row=scale(dict(zip(f["columns"],vals)),f["format"],f["columns"])
            wanted[f["name"]].append(row)
        i+=f["length"]

    for name in wanted:
        f=next((z for z in fmts.values() if z["name"]==name),None)
        print(name,"FMT:",f)
    x1=[r for r in wanted["XKF1"] if int(r.get("C",-1))==args.core]
    x2=[r for r in wanted["XKF2"] if int(r.get("C",-1))==args.core]
    print(f"XKF1 core={args.core}: {len(x1)}  XKF2: {len(x2)}")
    if x1:
        for k in ("Roll","Pitch","Yaw"): show("XKF1 "+k,[float(r[k]) for r in x1])
    if x2:
        for k in ("AX","AY","AZ"): show("XKF2 "+k,[float(r[k]) for r in x2])

    instances=sorted({int(r.get("I",-1)) for r in wanted["IMU"]})
    for inst in instances:
        rr=[r for r in wanted["IMU"] if int(r.get("I",-1))==inst]
        print(f"IMU{inst}: {len(rr)}")
        for k in ("AccX","AccY","AccZ"): show(f"  {k}",[float(r[k]) for r in rr])

    if not x1: return
    att=median3(x1,("Roll","Pitch","Yaw"))
    # XKF1 angles use DataFlash c scaling and are degrees.
    ar,ap_,ay=[math.radians(v) for v in att]
    Ratt=mat321(ar,ap_,ay)
    Rtrim=mat321(args.trim_x,args.trim_y,0.0)
    Rinv=tr(Rtrim)  # exact inverse
    print(f"ATT median deg={att}")
    print(f"trim rad=[{args.trim_x:+.9f},{args.trim_y:+.9f}] exact inverse=transpose")

    for inst in instances:
        rr=[r for r in wanted["IMU"] if int(r.get("I",-1))==inst]
        if not rr: continue
        a=median3(rr,("AccX","AccY","AccZ"))
        base=mv(Ratt,a); base[2]+=9.80665
        ai=mv(Rinv,a); inv=mv(Ratt,ai); inv[2]+=9.80665
        print(f"IMU{inst} median raw=[{a[0]:+.6f},{a[1]:+.6f},{a[2]:+.6f}]")
        print(f"  BASE N/E/D=[{base[0]:+.6f},{base[1]:+.6f},{base[2]:+.6f}] |NE|={math.hypot(base[0],base[1]):.6f}")
        print(f"  EXACT_INVTRIM_TO_ATT N/E/D=[{inv[0]:+.6f},{inv[1]:+.6f},{inv[2]:+.6f}] |NE|={math.hypot(inv[0],inv[1]):.6f}")

if __name__=="__main__": main()
