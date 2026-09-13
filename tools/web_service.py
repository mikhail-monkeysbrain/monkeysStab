#!/usr/bin/env python3
import csv
import json
import math
import os
import signal
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/"config"/"runtime.json"
GEOMETRY=ROOT/"config"/"mount_geometry.json"
FC_PROFILE=ROOT/"config"/"fc_profile.json"
RUN_ROOT=Path(os.environ.get("MONKEYS_RUN_ROOT", str(Path.home()/"monkeysStab_runs")))
WEB_LOG=RUN_ROOT/"web_runtime.log"
DEFAULTS={
    "focal_scale":0.931,
    "feature_roi":[0.20,0.32,0.80,0.90],
    "max_features":500,
    "local_gui":True,
}

_lock=threading.RLock()
_proc=None
_log_handle=None
_router_proc=None
_router_log_handle=None
_router_started_here=False
_zero={"x":None,"y":None,"z":None}
FC_ENDPOINT="tcp://127.0.0.1:5760"

def load_json(path, fallback):
    try:
        with open(path,"r",encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return fallback

def load_config():
    d=load_json(CONFIG, dict(DEFAULTS))
    out=dict(DEFAULTS)
    out.update({k:v for k,v in d.items() if k in out})
    return out

def validate_config(d):
    fs=float(d.get("focal_scale"))
    if not (0.5 < fs < 2.0):
        raise ValueError("focal_scale должен быть в диапазоне 0.5..2.0")
    roi=d.get("feature_roi")
    if not isinstance(roi,list) or len(roi)!=4:
        raise ValueError("feature_roi должен содержать 4 числа")
    roi=[float(x) for x in roi]
    x0,y0,x1,y1=roi
    if not (0<=x0<x1<=1 and 0<=y0<y1<=1 and x1-x0>=0.20 and y1-y0>=0.20):
        raise ValueError("feature_roi: 0..1, ширина/высота не менее 0.20")
    mf=int(d.get("max_features"))
    if not (100 <= mf <= 1000):
        raise ValueError("max_features должен быть 100..1000")
    return {
        "focal_scale":fs,
        "feature_roi":roi,
        "max_features":mf,
        "local_gui":bool(d.get("local_gui",True)),
    }

def save_config(d):
    d=validate_config(d)
    tmp=CONFIG.with_suffix(".json.tmp")
    with open(tmp,"w",encoding="utf-8") as f:
        json.dump(d,f,ensure_ascii=False,indent=2)
        f.write("\n")
    os.replace(tmp,CONFIG)
    return d

def running():
    global _proc
    with _lock:
        return _proc is not None and _proc.poll() is None

def tcp_ready(host="127.0.0.1",port=5760):
    try:
        with socket.create_connection((host,port),timeout=0.35):
            return True
    except OSError:
        return False

def ensure_router():
    global _router_proc,_router_log_handle,_router_started_here
    with _lock:
        if tcp_ready():
            return
        RUN_ROOT.mkdir(parents=True,exist_ok=True)
        router_log=RUN_ROOT/"mavlink_router_web.log"
        _router_log_handle=open(router_log,"a",encoding="utf-8",buffering=1)
        _router_log_handle.write("\n===== WEB ROUTER START %s =====\n"%time.strftime("%Y-%m-%d %H:%M:%S"))
        env=os.environ.copy()
        env["MONKEYS_FC_TCP_PORT"]="5760"
        _router_proc=subprocess.Popen(
            ["bash",str(ROOT/"scripts"/"run_mavlink_wifi.sh")],
            cwd=str(ROOT),env=env,
            stdout=_router_log_handle,stderr=subprocess.STDOUT,
            start_new_session=True,text=True
        )
        _router_started_here=True
    deadline=time.time()+6.0
    while time.time()<deadline:
        if tcp_ready(): return
        if _router_proc.poll() is not None:
            raise RuntimeError("MAVLink router завершился при запуске")
        time.sleep(0.1)
    raise RuntimeError("MAVLink router не открыл tcp://127.0.0.1:5760")

def stop_router():
    global _router_proc,_router_log_handle,_router_started_here
    with _lock:
        if not _router_started_here or _router_proc is None:
            return
        p=_router_proc
        _router_proc=None
        _router_started_here=False
    if p.poll() is None:
        try: os.killpg(p.pid,signal.SIGTERM)
        except ProcessLookupError: pass
        try: p.wait(timeout=4)
        except subprocess.TimeoutExpired:
            try: os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError: pass
    if _router_log_handle:
        try:_router_log_handle.close()
        except Exception:pass
        _router_log_handle=None

def fc_control(*args):
    ensure_router()
    env=os.environ.copy()
    env["MONKEYS_FC"]=FC_ENDPOINT
    cp=subprocess.run(
        ["bash",str(ROOT/"scripts"/"fc_control.sh"),*args],
        cwd=str(ROOT),env=env,text=True,capture_output=True,timeout=12
    )
    if cp.returncode!=0:
        raise RuntimeError((cp.stderr or cp.stdout or "команда FC завершилась с ошибкой").strip())
    text=cp.stdout.strip().splitlines()
    if not text: raise RuntimeError("FC не вернул состояние")
    try: return json.loads(text[-1])
    except Exception: raise RuntimeError("Некорректный ответ FC: "+text[-1])

def start_runtime():
    global _proc,_log_handle
    with _lock:
        if running():
            return {"ok":True,"already_running":True,"pid":_proc.pid}
        RUN_ROOT.mkdir(parents=True,exist_ok=True)
        _log_handle=open(WEB_LOG,"a",encoding="utf-8",buffering=1)
        _log_handle.write("\n===== WEB START %s =====\n"%time.strftime("%Y-%m-%d %H:%M:%S"))
        ensure_router()
        env=os.environ.copy()
        env["MONKEYS_LOCAL_GUI"]="0"
        env["MONKEYS_FC"]=FC_ENDPOINT
        _proc=subprocess.Popen(
            ["bash",str(ROOT/"scripts"/"run_system.sh")],
            cwd=str(ROOT), env=env,
            stdout=_log_handle, stderr=subprocess.STDOUT,
            start_new_session=True, text=True
        )
        return {"ok":True,"pid":_proc.pid}

def stop_runtime():
    global _proc,_log_handle
    with _lock:
        if not running():
            return {"ok":True,"already_stopped":True}
        pid=_proc.pid
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            _proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            try: os.killpg(pid, signal.SIGKILL)
            except ProcessLookupError: pass
            _proc.wait(timeout=2)
        if _log_handle:
            try: _log_handle.close()
            except Exception: pass
            _log_handle=None
        return {"ok":True}

def latest_run_csv():
    try:
        candidates=sorted(RUN_ROOT.glob("*_OPTICAL_FLOW/optical_flow_mavlink.csv"),
                          key=lambda p:p.stat().st_mtime, reverse=True)
        return candidates[0] if candidates else None
    except Exception:
        return None

def tail_rows(path, max_rows=180):
    if not path or not path.exists():
        return []
    try:
        with open(path,"rb") as f:
            header_line=f.readline().decode("utf-8","replace").strip()
            if not header_line: return []
            header=next(csv.reader([header_line]))
            f.seek(0,os.SEEK_END)
            size=f.tell()
            back=min(size, 512*1024)
            f.seek(size-back,os.SEEK_SET)
            chunk=f.read().decode("utf-8","replace")
        lines=chunk.splitlines()
        if back < size and lines: lines=lines[1:]
        data=[]
        for line in lines[-max_rows*2:]:
            if not line or line.startswith("mono_ns,"): continue
            try:
                vals=next(csv.reader([line]))
                if len(vals)!=len(header): continue
                data.append(dict(zip(header,vals)))
            except Exception:
                pass
        return data[-max_rows:]
    except Exception:
        return []

def fnum(row,key,default=None):
    try:
        v=float(row.get(key,""))
        return v if math.isfinite(v) else default
    except Exception:
        return default

def telemetry():
    rows=tail_rows(latest_run_csv(),180)
    if not rows:
        return {"available":False,"running":running(),"trail":[]}
    last=rows[-1]
    x=fnum(last,"ekf_x_ned")
    y=fnum(last,"ekf_y_ned")
    z=fnum(last,"ekf_z_ned")
    with _lock:
        zx,zy,zz=_zero["x"],_zero["y"],_zero["z"]
    if zx is None and x is not None:
        zx,zy,zz=x,y,z
    trail=[]
    if zx is not None:
        for r in rows:
            px=fnum(r,"ekf_x_ned"); py=fnum(r,"ekf_y_ned"); pz=fnum(r,"ekf_z_ned")
            if px is not None and py is not None:
                trail.append({
                    "x_mm":(px-zx)*1000.0,
                    "y_mm":(py-zy)*1000.0,
                    "z_mm":(pz-zz)*1000.0 if pz is not None and zz is not None else None,
                })
    history=[]
    if rows:
        t0=fnum(rows[0],"mono_ns",0) or 0
        for rr in rows:
            tt=(fnum(rr,"mono_ns",t0)-t0)/1e9 if t0 else 0.0
            hx=fnum(rr,"ekf_x_ned"); hy=fnum(rr,"ekf_y_ned"); hz=fnum(rr,"ekf_z_ned")
            hvx=fnum(rr,"ekf_vx_ned",0) or 0; hvy=fnum(rr,"ekf_vy_ned",0) or 0; hvz=fnum(rr,"ekf_vz_ned",0) or 0
            history.append({
                "t":tt,
                "x":(hx-zx) if hx is not None and zx is not None else None,
                "y":(hy-zy) if hy is not None and zy is not None else None,
                "z":(hz-zz) if hz is not None and zz is not None else None,
                "speed":math.sqrt(hvx*hvx+hvy*hvy+hvz*hvz),
                "range":fnum(rr,"luna_m"),
            })

    return {
        "available":True,
        "running":running(),
        "frame":int(fnum(last,"frame",0) or 0),
        "valid":int(fnum(last,"valid",0) or 0),
        "quality":int(fnum(last,"quality",0) or 0),
        "features":int(fnum(last,"features",0) or 0),
        "tracked":int(fnum(last,"tracked",0) or 0),
        "inliers":int(fnum(last,"inliers",0) or 0),
        "range_m":fnum(last,"luna_m"),
        "range_age_ms":fnum(last,"luna_age_ms"),
        "armed":bool(int(fnum(last,"fc_armed",0) or 0)),
        "ekf_valid":bool(int(fnum(last,"ekf_local_valid",0) or 0)),
        "x_mm":(x-zx)*1000.0 if x is not None and zx is not None else None,
        "y_mm":(y-zy)*1000.0 if y is not None and zy is not None else None,
        "z_mm":(z-zz)*1000.0 if z is not None and zz is not None else None,
        "vx":fnum(last,"ekf_vx_ned"),
        "vy":fnum(last,"ekf_vy_ned"),
        "vz":fnum(last,"ekf_vz_ned"),
        "roll_deg":math.degrees(fnum(last,"fc_roll",0) or 0),
        "pitch_deg":math.degrees(fnum(last,"fc_pitch",0) or 0),
        "yaw_deg":math.degrees(fnum(last,"fc_yaw",0) or 0),
        "trail":trail,
        "history":history,
    }

def set_zero():
    t=telemetry()
    if not t.get("available"):
        raise RuntimeError("Нет телеметрии")
    rows=tail_rows(latest_run_csv(),2)
    if not rows: raise RuntimeError("Нет телеметрии")
    r=rows[-1]
    with _lock:
        _zero["x"]=fnum(r,"ekf_x_ned")
        _zero["y"]=fnum(r,"ekf_y_ned")
        _zero["z"]=fnum(r,"ekf_z_ned")

def log_tail(max_lines=120):
    try:
        with open(WEB_LOG,"r",encoding="utf-8",errors="replace") as f:
            return "\n".join(f.readlines()[-max_lines:])
    except Exception:
        return ""

HTML=r'''<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>monkeysStab</title>
<style>
:root{font-family:system-ui,-apple-system,sans-serif;color:#e8edf2;background:#11161b}
body{margin:0;background:#11161b}.wrap{max-width:1450px;margin:auto;padding:18px}
h1{margin:0 0 4px;font-size:28px} .sub{color:#9fb0bd;margin-bottom:16px}
.grid{display:grid;grid-template-columns:390px 1fr;gap:16px}
.card{background:#192129;border:1px solid #2c3943;border-radius:12px;padding:16px}
label{display:block;font-size:13px;color:#aebbc5;margin:10px 0 4px}
input{box-sizing:border-box;width:100%;background:#0f151a;color:#eef4f8;border:1px solid #40515d;border-radius:7px;padding:9px}
.row{display:grid;grid-template-columns:repeat(4,1fr);gap:7px}
button{border:0;border-radius:8px;padding:10px 14px;font-weight:700;cursor:pointer;margin:5px 5px 5px 0}
.primary{background:#4da3ff;color:#06111b}.danger{background:#e05b65;color:white}.soft{background:#34434e;color:#eef4f8}
.status{display:flex;gap:10px;align-items:center;margin-bottom:12px}.dot{width:12px;height:12px;border-radius:50%;background:#777}.on{background:#45c878}.off{background:#e05b65}
.metrics{display:grid;grid-template-columns:repeat(5,1fr);gap:8px;margin-bottom:10px}.metric{background:#10171d;border-radius:8px;padding:10px}.metric b{display:block;font-size:20px}.metric span{color:#8fa1ae;font-size:12px}
canvas{width:100%;height:560px;background:#0c1115;border-radius:10px}
.visualRow{display:grid;grid-template-columns:1.45fr .75fr;gap:12px;margin-top:12px}
#drone3d{height:360px}
.inclinometers{display:grid;grid-template-columns:1fr;gap:10px}
.gaugeWrap{background:#10171d;border-radius:10px;padding:8px}
.gaugeWrap b{display:block;text-align:center;margin-bottom:4px}
.gauge{width:100%;height:102px;background:#0c1115;border-radius:8px}
pre{height:190px;overflow:auto;background:#0c1115;border-radius:8px;padding:10px;white-space:pre-wrap;font-size:12px}
.small{font-size:12px;color:#93a5b2}.good{color:#56d88b}.bad{color:#f06b75}
@media(max-width:900px){.grid{grid-template-columns:1fr}.metrics{grid-template-columns:repeat(2,1fr)}canvas{height:420px}.visualRow{grid-template-columns:1fr}#drone3d{height:320px}.inclinometers{grid-template-columns:repeat(3,1fr)}}
</style>
</head>
<body><div class="wrap">
<h1>monkeysStab</h1><div class="sub">Optical Flow + TF-Luna + ArduPilot EKF3</div>
<div class="grid">
<div>
<div class="card">
<h3>Запуск</h3>
<div class="status"><span id="dot" class="dot off"></span><b id="runState">Остановлено</b></div>
<button class="primary" onclick="start()">ЗАПУСТИТЬ ДЛЯ ПОЛЁТА</button>
<button class="danger" onclick="stop()">ОСТАНОВИТЬ</button>
<button class="soft" onclick="zero()">НОВАЯ ТОЧКА 0</button>
<div class="small">Запуск runtime сам по себе не ARM-ит FC и не переключает режим.</div>
</div>
<div class="card" style="margin-top:16px">
<h3>Полётный контроллер</h3>
<div class="status"><span id="fcDot" class="dot off"></span><b id="fcState">FC: нет связи</b></div>
<div style="margin-bottom:8px"><b>Режим: <span id="fcMode">—</span></b></div>
<button class="primary" onclick="armFc()">ARM</button>
<button class="danger" onclick="disarmFc()">DISARM</button>
<div style="margin-top:8px">
<button class="soft" onclick="setMode('stabilize','Stabilize')">STABILIZE</button>
<button class="soft" onclick="setMode('poshold','PosHold')">POSHOLD</button>
<button class="soft" onclick="setMode('loiter','Loiter')">LOITER</button>
</div>
<div id="fcMsg" class="small">Команды подтверждаются по HEARTBEAT FC.</div>
</div>
<div class="card" style="margin-top:16px">
<h3>Стартовые параметры</h3>
<label>focal_scale</label><input id="focal" type="number" min=".5" max="2" step=".001">
<label>Feature ROI (x0 / y0 / x1 / y1)</label>
<div class="row"><input id="r0" type="number" step=".01"><input id="r1" type="number" step=".01"><input id="r2" type="number" step=".01"><input id="r3" type="number" step=".01"></div>
<label>Максимум точек</label><input id="features" type="number" min="100" max="1000" step="10">
<button class="soft" onclick="saveConfig()">СОХРАНИТЬ</button>
<div id="saveMsg" class="small"></div>
</div>
<div class="card" style="margin-top:16px">
<h3>Текущая геометрия</h3><pre id="geometry" style="height:120px"></pre>
<h3>Профиль FC</h3><pre id="profile" style="height:150px"></pre>
</div>
</div>
<div>
<div class="card">
<div class="metrics">
<div class="metric"><span>X</span><b id="mx">—</b></div>
<div class="metric"><span>Y</span><b id="my">—</b></div>
<div class="metric"><span>Z</span><b id="mz">—</b></div>
<div class="metric"><span>TF-Luna</span><b id="mr">—</b></div>
<div class="metric"><span>Flow quality</span><b id="mq">—</b></div>
</div>
<canvas id="plot" width="1000" height="650"></canvas>
<div class="visualRow">
  <div>
    <h3>3D ориентация БПЛА</h3>
    <canvas id="drone3d" width="900" height="500"></canvas>
  </div>
  <div>
    <h3>Инклинометр</h3>
    <div class="inclinometers">
      <div class="gaugeWrap"><b>ROLL</b><canvas id="gRoll" class="gauge" width="320" height="120"></canvas></div>
      <div class="gaugeWrap"><b>PITCH</b><canvas id="gPitch" class="gauge" width="320" height="120"></canvas></div>
      <div class="gaugeWrap"><b>YAW</b><canvas id="gYaw" class="gauge" width="320" height="120"></canvas></div>
    </div>
  </div>
</div>
<div class="metrics" style="margin-top:10px">
<div class="metric"><span>Roll</span><b id="roll">—</b></div>
<div class="metric"><span>Pitch</span><b id="pitch">—</b></div>
<div class="metric"><span>Yaw</span><b id="yaw">—</b></div>
<div class="metric"><span>Inliers</span><b id="inl">—</b></div>
<div class="metric"><span>FC</span><b id="arm">—</b></div>
</div>
</div>
<div class="card" style="margin-top:16px"><h3>Журнал</h3><pre id="log"></pre></div>
</div>
</div>
</div>
<script>
async function api(path,opt){let r=await fetch(path,opt);let j=await r.json();if(!r.ok)throw new Error(j.error||r.statusText);return j}
function fmt(v,d=0){return v==null?'—':Number(v).toFixed(d)}
async function loadConfig(){let j=await api('/api/config'); let c=j.runtime;
focal.value=c.focal_scale; [r0.value,r1.value,r2.value,r3.value]=c.feature_roi; features.value=c.max_features;
geometry.textContent=JSON.stringify(j.geometry,null,2); profile.textContent=JSON.stringify(j.fc_profile.params,null,2)}
async function saveConfig(){try{let body={focal_scale:+focal.value,feature_roi:[+r0.value,+r1.value,+r2.value,+r3.value],max_features:+features.value,local_gui:true};
await api('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});saveMsg.textContent='Сохранено';}catch(e){saveMsg.textContent='Ошибка: '+e.message}}
async function start(){try{await api('/api/start',{method:'POST'});}catch(e){alert(e.message)}}
async function stop(){try{await api('/api/stop',{method:'POST'});}catch(e){alert(e.message)}}
async function zero(){try{await api('/api/zero',{method:'POST'});}catch(e){alert(e.message)}}
async function armFc(){
 if(!confirm('ARM: разрешить запуск моторов? Аппарат должен быть подготовлен к безопасному запуску.'))return;
 try{let j=await api('/api/fc/arm',{method:'POST'});fcMsg.textContent='ARM подтверждён FC';showFc(j)}catch(e){fcMsg.textContent='ARM отклонён: '+e.message;alert(e.message)}
}
async function disarmFc(){
 if(!confirm('DISARM: отключить моторы? В полёте обычный DISARM может быть запрещён ArduPilot.'))return;
 try{let j=await api('/api/fc/disarm',{method:'POST'});fcMsg.textContent='DISARM подтверждён FC';showFc(j)}catch(e){fcMsg.textContent='DISARM отклонён: '+e.message;alert(e.message)}
}
async function setMode(id,name){
 if(!confirm('Переключить режим на '+name+'?'))return;
 try{let j=await api('/api/fc/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:id})});fcMsg.textContent='Режим '+name+' подтверждён FC';showFc(j)}catch(e){fcMsg.textContent='Смена режима отклонена: '+e.message;alert(e.message)}
}
function showFc(j){
 fcDot.className='dot on';
 fcState.textContent=j.armed?'ARMED':'DISARMED';
 fcState.className=j.armed?'bad':'good';
 fcMode.textContent=j.mode+(j.mode==='Other'?' ('+j.custom_mode+')':'');
}
async function refreshFc(){
 try{let j=await api('/api/fc');showFc(j)}
 catch(e){fcDot.className='dot off';fcState.textContent='FC: нет связи';fcState.className='bad';fcMode.textContent='—'}
}
function draw(t){
 let c=plot,ctx=c.getContext('2d'),w=c.width,h=c.height;ctx.clearRect(0,0,w,h);
 ctx.fillStyle='#0c1115';ctx.fillRect(0,0,w,h);
 const cx=w/2,cy=h/2,scale=Math.min(w,h)/(2*550);
 ctx.strokeStyle='#26343d';ctx.lineWidth=1;
 for(let mm=-500;mm<=500;mm+=100){let x=cx+mm*scale,y=cy-mm*scale;ctx.beginPath();ctx.moveTo(x,40);ctx.lineTo(x,h-40);ctx.stroke();ctx.beginPath();ctx.moveTo(40,y);ctx.lineTo(w-40,y);ctx.stroke()}
 ctx.strokeStyle='#607582';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(cx,35);ctx.lineTo(cx,h-35);ctx.stroke();ctx.beginPath();ctx.moveTo(35,cy);ctx.lineTo(w-35,cy);ctx.stroke();
 ctx.fillStyle='#9fb0bd';ctx.font='16px system-ui';ctx.fillText('N +X',cx+8,55);ctx.fillText('E +Y',w-85,cy-10);ctx.fillText('фиксированный масштаб ±500 мм',45,h-18);
 if(t.trail&&t.trail.length){ctx.strokeStyle='#4da3ff';ctx.lineWidth=3;ctx.beginPath();t.trail.forEach((p,i)=>{let x=cx+p.y_mm*scale,y=cy-p.x_mm*scale;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});ctx.stroke();
 let p=t.trail[t.trail.length-1],x=cx+p.y_mm*scale,y=cy-p.x_mm*scale;ctx.fillStyle='#56d88b';ctx.beginPath();ctx.arc(x,y,8,0,Math.PI*2);ctx.fill();}
}

function rot3(p,roll,pitch,yaw){
 const cr=Math.cos(roll),sr=Math.sin(roll),cp=Math.cos(pitch),sp=Math.sin(pitch),cy=Math.cos(yaw),sy=Math.sin(yaw);
 let x=p[0],y=p[1],z=p[2];
 let x1=x, y1=cr*y-sr*z, z1=sr*y+cr*z;
 let x2=cp*x1+sp*z1, y2=y1, z2=-sp*x1+cp*z1;
 return [cy*x2-sy*y2, sy*x2+cy*y2, z2];
}
function project3(p,w,h){
 const d=5.5, s=Math.min(w,h)*0.23, z=d-p[2];
 return [w/2 + p[1]*s/z, h/2 - p[0]*s/z];
}
function drawDrone(t){
 let c=drone3d,ctx=c.getContext('2d'),w=c.width,h=c.height;
 ctx.clearRect(0,0,w,h);ctx.fillStyle='#0c1115';ctx.fillRect(0,0,w,h);
 let r=(t.roll_deg||0)*Math.PI/180,p=(t.pitch_deg||0)*Math.PI/180,y=(t.yaw_deg||0)*Math.PI/180;
 ctx.strokeStyle='#26343d';ctx.lineWidth=1;
 for(let i=-5;i<=5;i++){let a=project3([i*.35,-1.8,-1.1],w,h),b=project3([i*.35,1.8,-1.1],w,h);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
 let c1=project3([-1.8,i*.35,-1.1],w,h),d1=project3([1.8,i*.35,-1.1],w,h);ctx.beginPath();ctx.moveTo(...c1);ctx.lineTo(...d1);ctx.stroke();}
 const arm=1.25,z=0;
 const pts={f:[arm,0,z],b:[-arm,0,z],l:[0,-arm,z],rr:[0,arm,z],c:[0,0,z]};
 function rp(v){return project3(rot3(v,r,p,y),w,h)}
 function line(a,b,color,width){let A=rp(a),B=rp(b);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(...A);ctx.lineTo(...B);ctx.stroke()}
 line([arm,0,0],[-arm,0,0],'#4da3ff',9);line([0,-arm,0],[0,arm,0],'#8ea1ad',9);
 for(const q of [[arm,0,0],[-arm,0,0],[0,-arm,0],[0,arm,0]]){let P=rp(q);ctx.fillStyle='#56d88b';ctx.beginPath();ctx.arc(P[0],P[1],18,0,Math.PI*2);ctx.fill();ctx.strokeStyle='#dbe7ee';ctx.lineWidth=3;ctx.beginPath();ctx.arc(P[0],P[1],28,0,Math.PI*2);ctx.stroke();}
 let C=rp([0,0,0]);ctx.fillStyle='#dbe7ee';ctx.beginPath();ctx.arc(C[0],C[1],16,0,Math.PI*2);ctx.fill();
 let F=rp([1.55,0,0]);ctx.fillStyle='#ffb14d';ctx.beginPath();ctx.moveTo(F[0],F[1]);let f1=rp([1.15,-.18,0]),f2=rp([1.15,.18,0]);ctx.lineTo(f1[0],f1[1]);ctx.lineTo(f2[0],f2[1]);ctx.closePath();ctx.fill();
 ctx.fillStyle='#9fb0bd';ctx.font='17px system-ui';ctx.fillText('нос',Math.min(w-55,F[0]+10),Math.max(24,F[1]-6));
 ctx.fillText('roll '+fmt(t.roll_deg,1)+'°   pitch '+fmt(t.pitch_deg,1)+'°   yaw '+fmt(t.yaw_deg,1)+'°',20,28);
}
function drawGauge(id,value,range,mode){
 let c=document.getElementById(id),ctx=c.getContext('2d'),w=c.width,h=c.height;
 ctx.clearRect(0,0,w,h);ctx.fillStyle='#0c1115';ctx.fillRect(0,0,w,h);
 let cx=w/2,cy=h*.70,R=Math.min(w*.42,h*.58);
 ctx.strokeStyle='#33434e';ctx.lineWidth=10;ctx.beginPath();ctx.arc(cx,cy,R,Math.PI,2*Math.PI);ctx.stroke();
 for(let i=0;i<=10;i++){let a=Math.PI+i*Math.PI/10,x1=cx+Math.cos(a)*(R-8),y1=cy+Math.sin(a)*(R-8),x2=cx+Math.cos(a)*(R+7),y2=cy+Math.sin(a)*(R+7);ctx.strokeStyle='#6c7f8c';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke();}
 let v=Number(value||0),norm;
 if(mode==='yaw'){v=((v+180)%360+360)%360-180;norm=(v+180)/360;} else {v=Math.max(-range,Math.min(range,v));norm=(v+range)/(2*range);}
 let a=Math.PI+norm*Math.PI;
 ctx.strokeStyle='#56d88b';ctx.lineWidth=5;ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx+Math.cos(a)*(R-10),cy+Math.sin(a)*(R-10));ctx.stroke();
 ctx.fillStyle='#eef4f8';ctx.beginPath();ctx.arc(cx,cy,6,0,Math.PI*2);ctx.fill();
 ctx.font='bold 20px system-ui';ctx.textAlign='center';ctx.fillText(fmt(value,1)+'°',cx,cy+30);
 ctx.font='12px system-ui';ctx.fillStyle='#8fa1ae';ctx.fillText(mode==='yaw'?'−180°                             +180°':'−'+range+'°                               +'+range+'°',cx,18);
}
async function refresh(){try{
 let t=await api('/api/telemetry'); dot.className='dot '+(t.running?'on':'off');runState.textContent=t.running?'Работает':'Остановлено';
 mx.textContent=fmt(t.x_mm,0)+' мм';my.textContent=fmt(t.y_mm,0)+' мм';mz.textContent=fmt(t.z_mm,0)+' мм';mr.textContent=t.range_m==null?'—':fmt(t.range_m*1000,0)+' мм';mq.textContent=t.quality==null?'—':t.quality;
 roll.textContent=fmt(t.roll_deg,1)+'°';pitch.textContent=fmt(t.pitch_deg,1)+'°';yaw.textContent=fmt(t.yaw_deg,1)+'°';inl.textContent=(t.inliers??'—')+'/'+(t.tracked??'—');arm.textContent=t.armed?'ARMED':'DISARMED';draw(t);drawDrone(t);drawGauge('gRoll',t.roll_deg,45,'angle');drawGauge('gPitch',t.pitch_deg,45,'angle');drawGauge('gYaw',t.yaw_deg,180,'yaw');
 let l=await api('/api/log');log.textContent=l.text||'';log.scrollTop=log.scrollHeight;
 }catch(e){}}
loadConfig();refresh();refreshFc();setInterval(refresh,700);setInterval(refreshFc,1800);
</script></body></html>'''

class H(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass
    def send_json(self,obj,status=200):
        b=json.dumps(obj,ensure_ascii=False).encode()
        self.send_response(status);self.send_header("Content-Type","application/json; charset=utf-8")
        self.send_header("Content-Length",str(len(b)));self.end_headers();self.wfile.write(b)
    def body_json(self):
        n=int(self.headers.get("Content-Length","0") or 0)
        return json.loads(self.rfile.read(n).decode("utf-8")) if n else {}
    def do_GET(self):
        p=urlparse(self.path).path
        try:
            if p=="/":
                b=HTML.encode();self.send_response(200);self.send_header("Content-Type","text/html; charset=utf-8");self.send_header("Content-Length",str(len(b)));self.end_headers();self.wfile.write(b)
            elif p=="/api/config":
                self.send_json({"runtime":load_config(),"geometry":load_json(GEOMETRY,{}),"fc_profile":load_json(FC_PROFILE,{})})
            elif p=="/api/status":
                self.send_json({"running":running(),"pid":_proc.pid if running() else None})
            elif p=="/api/telemetry":
                self.send_json(telemetry())
            elif p=="/api/log":
                self.send_json({"text":log_tail()})
            elif p=="/api/fc":
                self.send_json(fc_control("status"))
            else:self.send_json({"error":"not found"},404)
        except Exception as e:self.send_json({"error":str(e)},500)
    def do_POST(self):
        p=urlparse(self.path).path
        try:
            if p=="/api/config":
                if running(): raise RuntimeError("Остановите flight runtime перед изменением стартовых параметров")
                self.send_json({"ok":True,"runtime":save_config(self.body_json())})
            elif p=="/api/start": self.send_json(start_runtime())
            elif p=="/api/stop": self.send_json(stop_runtime())
            elif p=="/api/zero": set_zero();self.send_json({"ok":True})
            elif p=="/api/fc/arm": self.send_json(fc_control("arm"))
            elif p=="/api/fc/disarm": self.send_json(fc_control("disarm"))
            elif p=="/api/fc/mode":
                mode=str(self.body_json().get("mode","")).lower()
                if mode not in ("stabilize","poshold","loiter"):
                    raise ValueError("Разрешены только Stabilize, PosHold и Loiter")
                self.send_json(fc_control("mode",mode))
            else:self.send_json({"error":"not found"},404)
        except Exception as e:self.send_json({"error":str(e)},400)

def ips():
    out=[]
    try:
        for info in socket.getaddrinfo(socket.gethostname(),None,socket.AF_INET):
            ip=info[4][0]
            if not ip.startswith("127.") and ip not in out: out.append(ip)
    except Exception: pass
    return out

if __name__=="__main__":
    import argparse
    ap=argparse.ArgumentParser()
    ap.add_argument("--host",default="0.0.0.0")
    ap.add_argument("--port",type=int,default=8080)
    a=ap.parse_args()
    RUN_ROOT.mkdir(parents=True,exist_ok=True)
    print("="*70)
    print("monkeysStab WEB")
    print(f"Локально: http://127.0.0.1:{a.port}")
    for ip in ips(): print(f"С компьютера в той же сети: http://{ip}:{a.port}")
    print("Web-интерфейс: runtime + FC ARM/DISARM + Stabilize/PosHold/Loiter.")
    print("="*70,flush=True)
    try:
        ensure_router()
        print("MAVLink router: ГОТОВ, Mission Planner UDP 14550",flush=True)
        ThreadingHTTPServer((a.host,a.port),H).serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if running(): stop_runtime()
        stop_router()
