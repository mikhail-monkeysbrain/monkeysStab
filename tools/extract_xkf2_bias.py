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
def mm(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def tr(M): return [list(x) for x in zip(*M)]
def median3(rows,keys): return [statistics.median(float(r[k]) for r in rows) for k in keys]

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("bin"); ap.add_argument("--core",type=int,default=0)
    ap.add_argument("--trim-x",type=float,default=-0.03428453207)
    ap.add_argument("--trim-y",type=float,default=-0.0220823437)
    ap.add_argument("--aligned",action="store_true",help="time-align IMU/XKF1/XKF2 and audit XKF2 accel-bias frame/sign")
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

    if args.aligned:
        # Time-aligned audit: nearest XKF1/XKF2 to each IMU sample.
        # XKF2 AX/AY/AZ semantics are deliberately tested, not assumed.
        def nearest(rows,t):
            if not rows: return None
            return min(rows,key=lambda r:abs(int(r["TimeUS"])-t))
        Rtrim=mat321(args.trim_x,args.trim_y,0.0)
        Rinv=tr(Rtrim)
        g=9.80665
        print("\n===== TIME-ALIGNED XKF2 BIAS AUDIT =====")
        for inst in instances:
            rr=[r for r in wanted["IMU"] if int(r.get("I",-1))==inst]
            scores={k:[] for k in (
                "NO_BIAS",
                "RAW_MINUS_BIAS","RAW_PLUS_BIAS",
                "TRIMMED_MINUS_BIAS","TRIMMED_PLUS_BIAS",
                "NED_MINUS_BIAS","NED_PLUS_BIAS")}
            pairs=0; d1s=[]; d2s=[]
            for im in rr:
                t=int(im["TimeUS"]); a=[float(im[k]) for k in ("AccX","AccY","AccZ")]
                q1=nearest(x1,t); q2=nearest(x2,t)
                if q1 is None or q2 is None: continue
                d1=abs(int(q1["TimeUS"])-t); d2=abs(int(q2["TimeUS"])-t)
                if d1>100000 or d2>100000: continue
                d1s.append(d1); d2s.append(d2); pairs+=1
                ar,ap_,ay=[math.radians(float(q1[k])) for k in ("Roll","Pitch","Yaw")]
                Ratt=mat321(ar,ap_,ay)
                bias=[float(q2[k]) for k in ("AX","AY","AZ")]
                def resid(v):
                    n=mv(Ratt,mv(Rinv,v)); n[2]+=g
                    return math.hypot(n[0],n[1])
                scores["NO_BIAS"].append(resid(a))
                scores["RAW_MINUS_BIAS"].append(resid([a[i]-bias[i] for i in range(3)]))
                scores["RAW_PLUS_BIAS"].append(resid([a[i]+bias[i] for i in range(3)]))
                # Bias interpreted in trim-corrected body frame.
                at=mv(Rinv,a)
                for sign,label in ((-1,"TRIMMED_MINUS_BIAS"),(1,"TRIMMED_PLUS_BIAS")):
                    v=[at[i]+sign*bias[i] for i in range(3)]
                    n=mv(Ratt,v); n[2]+=g
                    scores[label].append(math.hypot(n[0],n[1]))
                # Bias interpreted directly in NED.
                n0=mv(Ratt,at); n0[2]+=g
                scores["NED_MINUS_BIAS"].append(math.hypot(n0[0]-bias[0],n0[1]-bias[1]))
                scores["NED_PLUS_BIAS"].append(math.hypot(n0[0]+bias[0],n0[1]+bias[1]))
            print(f"IMU{inst}: aligned={pairs}")
            if pairs:
                print(f"  nearest XKF1 dt_us median={statistics.median(d1s):.0f} max={max(d1s)}")
                print(f"  nearest XKF2 dt_us median={statistics.median(d2s):.0f} max={max(d2s)}")
                ranked=[]
                for name,xs in scores.items():
                    med=statistics.median(xs); rms=math.sqrt(statistics.fmean(v*v for v in xs))
                    ranked.append((rms,name,med))
                    print(f"  {name:20s} median|NE|={med:.6f} RMS|NE|={rms:.6f}")
                best=min(ranked)
                print(f"  BEST_BIAS_HYPOTHESIS={best[1]} RMS|NE|={best[0]:.6f}")

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
        variants = [
            ("BASE", Ratt),
            ("RATT_RTRIM", mm(Ratt, Rtrim)),
            ("RATT_RTRIM_T", mm(Ratt, Rinv)),
            ("RTRIM_RATT", mm(Rtrim, Ratt)),
            ("RTRIM_T_RATT", mm(Rinv, Ratt)),
        ]
        print(f"IMU{inst} median raw=[{a[0]:+.6f},{a[1]:+.6f},{a[2]:+.6f}]")
        scored=[]
        for name,R in variants:
            ned=mv(R,a); ned[2]+=9.80665
            ne=math.hypot(ned[0],ned[1])
            scored.append((ne,name,ned))
            print(f"  {name:16s} N/E/D=[{ned[0]:+.6f},{ned[1]:+.6f},{ned[2]:+.6f}] |NE|={ne:.6f}")
        best=min(scored,key=lambda z:z[0])
        print(f"  BEST={best[1]} |NE|={best[0]:.6f}")

if __name__=="__main__": main()
