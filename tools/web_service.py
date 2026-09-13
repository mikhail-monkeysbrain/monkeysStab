#!/usr/bin/env python3
import csv
import json
import math
import re
import os
import signal
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from collections import deque
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
    "visualization_mode":"simple",
}

_lock=threading.RLock()
_proc=None
_log_handle=None
_router_proc=None
_router_log_handle=None
_router_started_here=False
_statustext_proc=None
_statustext_thread=None
_zero={"x":None,"y":None,"z":None}
_journal=deque(maxlen=500)
_messages=deque(maxlen=500)
FC_ENDPOINT="tcp://127.0.0.1:5760"
GEOMETRY_PARAMS=["FLOW_POS_X","FLOW_POS_Y","FLOW_POS_Z","RNGFND1_POS_X","RNGFND1_POS_Y","RNGFND1_POS_Z"]

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
        "visualization_mode":str(d.get("visualization_mode","simple")) if str(d.get("visualization_mode","simple")) in ("advanced","simple","light") else "simple",
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

def log_event(level,text):
    with _lock:
        _journal.append({
            "ts":time.strftime("%Y-%m-%d %H:%M:%S"),
            "level":str(level).upper(),
            "text":str(text),
        })

def fc_param_cli(*args,timeout=35):
    ensure_router()
    env=os.environ.copy()
    env["MONKEYS_FC"]=FC_ENDPOINT
    cp=subprocess.run(
        ["bash",str(ROOT/"scripts"/"fc_params_cli.sh"),*map(str,args)],
        cwd=str(ROOT),env=env,text=True,capture_output=True,timeout=timeout
    )
    if cp.returncode!=0:
        raise RuntimeError((cp.stderr or cp.stdout or "ошибка PARAM").strip())
    return cp.stdout

def parse_param_values(text):
    out={}
    for line in text.splitlines():
        if "=" not in line or line.startswith("TARGET "): continue
        k,v=line.split("=",1)
        try: out[k.strip()]=float(v.strip().split()[0])
        except Exception: pass
    return out

def read_fc_params(names):
    if not names:return {}
    return parse_param_values(fc_param_cli("read",*names))

def profile_param_names():
    p=load_json(FC_PROFILE,{})
    return list((p.get("params") or {}).keys())

def set_profile_params(values):
    if running(): raise RuntimeError("Остановите flight runtime перед изменением параметров FC")
    st=fc_control("status")
    if st.get("armed"): raise RuntimeError("FC должен быть DISARMED")
    allowed=set(profile_param_names())
    clean={}
    for k,v in values.items():
        if k not in allowed: raise ValueError("Параметр не разрешён: "+str(k))
        fv=float(v)
        if not math.isfinite(fv): raise ValueError("Некорректное значение "+k)
        clean[k]=fv
    if not clean: raise ValueError("Нет параметров для записи")
    geom_before=read_fc_params(GEOMETRY_PARAMS)
    args=["set"]
    for k,v in clean.items(): args += [k,f"{v:.6f}"]
    fc_param_cli(*args,timeout=50)
    got=read_fc_params(list(clean.keys()))
    bad=[k for k,v in clean.items() if k not in got or not math.isclose(got[k],v,rel_tol=0,abs_tol=max(1e-6,abs(v)*1e-5))]
    if bad: raise RuntimeError("Не подтверждены: "+", ".join(bad))
    geom_after=read_fc_params(GEOMETRY_PARAMS)
    changed=[k for k in GEOMETRY_PARAMS if k in geom_before and k in geom_after and not math.isclose(geom_before[k],geom_after[k],rel_tol=0,abs_tol=1e-6)]
    if changed: raise RuntimeError("ОШИБКА БЕЗОПАСНОСТИ: изменилась геометрия: "+", ".join(changed))
    prof=load_json(FC_PROFILE,{})
    prof.setdefault("params",{}).update(got)
    tmp=FC_PROFILE.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(prof,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    os.replace(tmp,FC_PROFILE)
    log_event("INFO","Параметры FC записаны: "+", ".join(clean.keys()))
    return got

def save_local_geometry(vals):
    cfg=load_json(GEOMETRY,{"frame":"FRD","units":"m","reference":"FC_IMU","camera":{"name":"OV9281"},"rangefinder":{"name":"TF-Luna"}})
    cfg.setdefault("camera",{})["name"]="OV9281";cfg.setdefault("rangefinder",{})["name"]="TF-Luna"
    cfg["camera"].update({"x":vals["FLOW_POS_X"],"y":vals["FLOW_POS_Y"],"z":vals["FLOW_POS_Z"]})
    cfg["rangefinder"].update({"x":vals["RNGFND1_POS_X"],"y":vals["RNGFND1_POS_Y"],"z":vals["RNGFND1_POS_Z"]})
    tmp=GEOMETRY.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+"\n",encoding="utf-8");os.replace(tmp,GEOMETRY)
    yp=ROOT/"config"/"ov9281_current_mount.yaml"
    text=yp.read_text(encoding="utf-8")
    x,y,z=vals["FLOW_POS_X"],vals["FLOW_POS_Y"],vals["FLOW_POS_Z"]
    block=("data: [ 0.000000000, -1.000000000,  0.000000000,  {x:.6f},\n"
           "       -1.000000000,  0.000000000,  0.000000000,  {yflu:.6f},\n"
           "        0.000000000,  0.000000000, -1.000000000,  {zflu:.6f},\n"
           "        0.000000000,  0.000000000,  0.000000000,  1.0000 ]").format(x=x,yflu=-y,zflu=-z)
    text2,n=re.subn(r"data:\s*\[.*?1\.0000\s*\]",block,text,count=1,flags=re.S)
    if n!=1: raise RuntimeError("Не удалось обновить T_BS.data в camera YAML")
    yp.write_text(text2,encoding="utf-8")

def set_geometry(values):
    if running(): raise RuntimeError("Остановите flight runtime перед изменением геометрии")
    st=fc_control("status")
    if st.get("armed"): raise RuntimeError("FC должен быть DISARMED")
    clean={}
    for k in GEOMETRY_PARAMS:
        if k not in values: raise ValueError("Не заполнено "+k)
        v=float(values[k])
        if not math.isfinite(v) or abs(v)>2.0: raise ValueError(k+": допустимо ±2 м")
        clean[k]=v
    args=["set"]
    for k in GEOMETRY_PARAMS: args += [k,f"{clean[k]:.6f}"]
    fc_param_cli(*args,timeout=50)
    got=read_fc_params(GEOMETRY_PARAMS)
    bad=[k for k in GEOMETRY_PARAMS if k not in got or not math.isclose(got[k],clean[k],rel_tol=0,abs_tol=max(1e-6,abs(clean[k])*1e-5))]
    if bad: raise RuntimeError("Геометрия не подтверждена: "+", ".join(bad))
    save_local_geometry(got)
    log_event("INFO","Геометрия датчиков записана и синхронизирована")
    return got

def start_statustext_monitor():
    global _statustext_proc,_statustext_thread
    if _statustext_proc is not None and _statustext_proc.poll() is None:return
    ensure_router()
    env=os.environ.copy();env["MONKEYS_FC"]=FC_ENDPOINT
    _statustext_proc=subprocess.Popen(
        ["bash",str(ROOT/"scripts"/"fc_statustext_monitor.sh")],
        cwd=str(ROOT),env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,
        bufsize=1
    )
    def reader():
        if not _statustext_proc.stdout:return
        for line in _statustext_proc.stdout:
            try:
                j=json.loads(line)
                txt=str(j.get("text","")).strip()
                if not txt:continue
                with _lock:_messages.append({
                    "ts":time.strftime("%H:%M:%S"),
                    "severity":int(j.get("severity",6)),
                    "text":txt,
                })
            except Exception:
                pass
    _statustext_thread=threading.Thread(target=reader,daemon=True)
    _statustext_thread.start()

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
        log_event("INFO","MAVLink router запущен")
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
        log_event("INFO","Flight runtime запущен")
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
        log_event("INFO","Flight runtime остановлен")
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
<title>monkeysStab — UAV Control & Visualizer</title>
<style>
*{box-sizing:border-box}
:root{
 --bg:#07111b;--panel:#0c1a27;--panel2:#0a1621;--line:#1d3950;--line2:#14293a;
 --text:#e9f3fb;--muted:#87a5bd;--blue:#1299ff;--green:#0bd777;--red:#ff4654;
 --yellow:#ffc928;--cyan:#15d2ff;--purple:#c98bff;
 font-family:Inter,ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif;
 color:var(--text);background:var(--bg)
}
html,body{margin:0;min-height:100%;background:
 radial-gradient(circle at 75% -10%,#0e2638 0,#07111b 35%),
 linear-gradient(#07111b,#061018)}
body{overflow-x:hidden}
button,input,select{font:inherit}
button{cursor:pointer}
.topbar{height:64px;border-bottom:1px solid var(--line);display:flex;align-items:center;padding:0 18px;background:#071522dd;backdrop-filter:blur(12px);position:sticky;top:0;z-index:20}
.brand{display:flex;align-items:center;gap:12px;min-width:280px}
.logo{width:34px;height:34px;border:3px solid var(--blue);transform:rotate(30deg);border-radius:8px;position:relative;box-shadow:0 0 18px #1299ff55}
.logo:after{content:"";position:absolute;inset:7px;border:2px solid #42b6ff;border-radius:4px}
.brand b{font-size:22px;letter-spacing:.3px}.brand small{display:block;color:#9ab3c8;margin-top:1px}
.nav{display:flex;height:100%;align-items:center;gap:8px;flex:1}
.nav button{height:100%;padding:0 22px;background:none;color:#a4bdd1;border:0;border-bottom:3px solid transparent;font-weight:700}
.nav button.active{color:#20aaff;border-bottom-color:#20aaff;background:#0f2d44}
.topStatus{display:flex;gap:22px;align-items:center;font-size:13px;color:#9fb5c6}
.okdot{width:10px;height:10px;border-radius:50%;display:inline-block;background:#17d768;box-shadow:0 0 10px #17d76888;margin-right:7px}
.baddot{background:#ff4b59;box-shadow:0 0 10px #ff4b5988}
.main{display:grid;grid-template-columns:275px minmax(580px,1fr) 270px;gap:12px;padding:12px;max-width:1900px;margin:auto}
.col{display:flex;flex-direction:column;gap:12px}
.card{background:linear-gradient(180deg,#0c1a27,#091621);border:1px solid #1c3b53;border-radius:9px;box-shadow:inset 0 1px 0 #ffffff08;padding:13px}
.card h3{font-size:15px;margin:0 0 12px}.card h4{font-size:13px;color:#a8bfd0;margin:8px 0}
.fcstate{font-size:25px;font-weight:800;display:flex;align-items:center;gap:10px}.modeLine{margin:7px 0 10px;color:#a7bfd1}.modeLine b{color:#27aaff}
.btnrow{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px}
.btnrow3{display:grid;grid-template-columns:1fr;gap:7px;margin-top:9px}
.btnrow3 .btn{width:100%;min-width:0;padding:9px 6px;font-size:12px}
.btn{border:1px solid #31526a;border-radius:6px;padding:10px 8px;background:#102438;color:#eaf5fc;font-weight:800;min-width:0;max-width:100%;overflow:hidden;text-overflow:ellipsis}
.btn:hover{filter:brightness(1.12)}.btn.green{background:#0aad5b;border-color:#0de479}.btn.red{background:#a92431;border-color:#ff4050}.btn.blue{background:#087aca;border-color:#17a9ff}.btn.stop{color:#ff5361;background:#24111a;border-color:#dc3444}
.field{display:grid;grid-template-columns:1fr 112px;align-items:center;gap:8px;margin:9px 0;color:#9fb7ca;font-size:13px}
.field input,.field select{width:100%;background:#0b1823;color:#eaf3f9;border:1px solid #26465d;border-radius:5px;padding:7px}
.startBig{width:100%;margin-top:10px;border:1px solid #12df77;background:#0b9f56;color:white;border-radius:6px;padding:11px;font-weight:900}
.stopBig{width:100%;margin-top:8px;border:1px solid #ff4352;background:#201018;color:#ff5a67;border-radius:6px;padding:10px;font-weight:900}
.kv{display:grid;grid-template-columns:1fr auto;gap:5px 9px;font-size:12px}.kv span:nth-child(odd){color:#86a7bf}.kv span:nth-child(even){color:#dfeaf2}
.sceneCard{padding:0;overflow:hidden;position:relative;min-height:625px}
.sceneTitle{position:absolute;left:14px;top:10px;z-index:4;font-weight:800}
#glCanvas{display:block;width:100%;height:625px;background:
 radial-gradient(circle at 50% 15%,#11304a55,#07121c 52%),#07121c}
.sceneControls{position:absolute;right:12px;top:10px;background:#081521dd;border:1px solid #26475e;border-radius:7px;padding:8px 10px;font-size:12px;z-index:4}
.sceneControls label{display:block;margin:5px 0;color:#b3c9d7}
.sceneLegend{position:absolute;left:14px;bottom:12px;display:flex;gap:8px;z-index:4}
.miniBtn{border:1px solid #294c65;background:#0c1c29;color:#dbe8f0;padding:8px 11px;border-radius:5px}
.telemetryStrip{position:absolute;right:14px;bottom:12px;background:#081521cc;border:1px solid #24445a;border-radius:6px;padding:8px 11px;font-size:12px;color:#a9c0d0;z-index:4}
.metrics{display:grid;grid-template-columns:repeat(5,1fr);gap:8px;margin-top:10px}
.metric{background:#09151f;border:1px solid #163147;border-radius:7px;padding:9px}.metric span{font-size:11px;color:#7fa1ba}.metric b{display:block;font-size:18px;margin-top:2px}
.gaugeBox{padding:10px 12px}.gLine{display:grid;grid-template-columns:58px 1fr 52px;gap:7px;align-items:center;margin:12px 0;font-size:12px}.gLine strong{text-align:right}
.bar{height:7px;background:#19364a;border-radius:10px;position:relative}.bar:after{content:"";position:absolute;left:50%;top:-5px;height:17px;width:2px;background:#5f7f95}.needle{position:absolute;top:-4px;width:5px;height:15px;border-radius:2px;background:#18e278;box-shadow:0 0 8px currentColor;transform:translateX(-50%)}
#compass{width:100%;height:205px;display:block}
.viewGrid{display:grid;grid-template-columns:1fr 1fr;gap:7px}.viewItem{height:80px;border:1px solid #294b63;border-radius:5px;background:#08141e;display:flex;align-items:end;justify-content:center;padding:6px;color:#9fb9cb;font-size:11px}.viewItem.active{border-color:#18a8ff;box-shadow:inset 0 0 0 1px #18a8ff55}
.bottomCharts{display:grid;grid-template-columns:1.3fr 1fr 1fr;gap:10px;margin-top:10px}.chartCard{padding:10px}.chartCard h3{margin-bottom:4px}.chart{width:100%;height:170px;display:block;background:#08131d;border-radius:5px}
.logCard{grid-column:1/-1}.logHead{display:flex;justify-content:space-between;align-items:center}.log{height:120px;background:#07121a;border:1px solid #122a3c;border-radius:5px;padding:8px;overflow:auto;white-space:pre-wrap;font:12px/1.35 ui-monospace,SFMono-Regular,Consolas,monospace;color:#9cb5c8}
.footer{height:38px;border-top:1px solid #16364d;display:flex;align-items:center;justify-content:space-between;padding:0 16px;color:#7798ae;font-size:12px}
.badge{display:inline-flex;align-items:center;gap:5px}
@media(max-width:1250px){.main{grid-template-columns:255px 1fr}.right{grid-column:1/-1;display:grid;grid-template-columns:1fr 1fr 1fr}.sceneCard{min-height:520px}#glCanvas{height:520px}.bottomCharts{grid-template-columns:1fr}.topStatus{display:none}}
@media(max-width:850px){.main{grid-template-columns:1fr}.left,.right{grid-column:auto}.right{display:flex}.nav{display:none}.brand{min-width:0;flex:1}.sceneCard{min-height:430px}#glCanvas{height:430px}.metrics{grid-template-columns:repeat(2,1fr)}}
</style>
</head>
<body>
<div class="topbar">
 <div class="brand"><div class="logo"></div><div><b>monkeysStab</b><small>UAV Control & Visualizer</small></div></div>
 <div class="nav">
  <button class="active" onclick="goSection('flight',this)">▲ ПОЛЁТ</button><button onclick="goSection('settings',this)">⚙ НАСТРОЙКИ</button><button onclick="goSection('telemetry',this)">∿ ТЕЛЕМЕТРИЯ</button><button onclick="goSection('journal',this)">▤ ЖУРНАЛ</button><button onclick="goSection('system',this)">⚙ СИСТЕМА</button>
 </div>
 <div class="topStatus"><span><i id="linkDot" class="okdot baddot"></i>СВЯЗЬ: <b id="linkText">НЕТ</b></span><span id="clock">--:--:--</span></div>
</div>

<div class="main" id="flight">
 <div class="col left">
  <div class="card">
   <h3>Полётный контроллер</h3>
   <div class="fcstate"><i id="fcDotBig" class="okdot baddot"></i><span id="fcState">НЕТ СВЯЗИ</span></div>
   <div class="modeLine">Режим: <b id="fcMode">—</b></div>
   <div class="btnrow"><button class="btn green" onclick="armFc()">🔒 ARM</button><button class="btn red" onclick="disarmFc()">🔒 DISARM</button></div>
   <div class="btnrow3">
    <button id="mStab" class="btn" onclick="setMode('stabilize','Stabilize')">STABILIZE</button>
    <button id="mPos" class="btn" onclick="setMode('poshold','PosHold')">POSHOLD</button>
    <button id="mLoi" class="btn" onclick="setMode('loiter','Loiter')">LOITER</button>
   </div>
   <div id="fcMsg" style="margin-top:8px;color:#7798ae;font-size:11px">Команды подтверждаются FC.</div>
  </div>

  <div class="card" id="settings">
   <h3>Стартовые параметры</h3>
   <div class="field"><span>Focal scale</span><input id="focal" type="number" min=".5" max="2" step=".001"></div>
   <div class="field"><span>ROI x0</span><input id="r0" type="number" step=".01"></div>
   <div class="field"><span>ROI y0</span><input id="r1" type="number" step=".01"></div>
   <div class="field"><span>ROI x1</span><input id="r2" type="number" step=".01"></div>
   <div class="field"><span>ROI y1</span><input id="r3" type="number" step=".01"></div>
   <div class="field"><span>Feature points</span><input id="features" type="number" min="100" max="1000" step="10"></div>
   <button class="btn" style="width:100%" onclick="saveConfig()">СОХРАНИТЬ ПАРАМЕТРЫ</button>
   <div id="saveMsg" style="font-size:11px;color:#7798ae;margin-top:5px"></div>
   <button class="startBig" onclick="start()">▶ ЗАПУСТИТЬ СИСТЕМУ</button>
   <button class="stopBig" onclick="stop()">■ ОСТАНОВИТЬ</button>
  </div>

  <div class="card">
   <h3>Текущая геометрия (мм)</h3>
   <div id="geomKv" class="kv"></div>
  </div>
  <div class="card">
   <h3>Профиль FC (EKF3)</h3>
   <div id="profileKv" class="kv"></div>
  </div>
 </div>

 <div class="col center">
  <div class="card sceneCard">
   <div class="sceneTitle">3D — Траектория и ориентация</div>
   <canvas id="glCanvas"></canvas>
   <div class="sceneControls">
    <label><input id="showTrail" type="checkbox" checked> Траектория</label>
    <label><input id="showGrid" type="checkbox" checked> Сетка</label>
    <label><input id="showAxes" type="checkbox" checked> Оси X/Y/Z</label>
    <label><input id="followCam" type="checkbox"> След камеры</label>
   </div>
   <div class="sceneLegend"><button class="miniBtn" onclick="zero()">⟳ HOME = текущая точка</button><button class="miniBtn" onclick="resetView()">⌂ Сброс вида</button></div>
   <div class="telemetryStrip"><span id="sceneXYZ">X 0.000 · Y 0.000 · Z 0.000 m</span></div>
  </div>

  <div class="metrics">
   <div class="metric"><span>X</span><b id="mx">—</b></div>
   <div class="metric"><span>Y</span><b id="my">—</b></div>
   <div class="metric"><span>Z</span><b id="mz">—</b></div>
   <div class="metric"><span>TF-Luna</span><b id="mr">—</b></div>
   <div class="metric"><span>Flow quality</span><b id="mq">—</b></div>
  </div>

  <div class="bottomCharts" id="telemetry">
   <div class="card chartCard"><h3>X / Y / Z (м)</h3><canvas id="xyzChart" class="chart"></canvas></div>
   <div class="card chartCard"><h3>Скорость (м/с)</h3><canvas id="speedChart" class="chart"></canvas></div>
   <div class="card chartCard"><h3>TF-Luna (м)</h3><canvas id="rangeChart" class="chart"></canvas></div>
   <div class="card logCard" id="journal">
    <div class="logHead"><h3>Журнал</h3><button class="miniBtn" onclick="document.getElementById('log').textContent=''">Очистить окно</button></div>
    <div id="log" class="log"></div>
   </div>
  </div>
 </div>

 <div class="col right">
  <div class="card gaugeBox">
   <h3>Инклинометр</h3>
   <div class="gLine"><span>ROLL</span><div class="bar"><i id="rollNeedle" class="needle"></i></div><strong id="roll">—</strong></div>
   <div class="gLine"><span>PITCH</span><div class="bar"><i id="pitchNeedle" class="needle"></i></div><strong id="pitch">—</strong></div>
   <div class="gLine"><span>YAW</span><div class="bar"><i id="yawNeedle" class="needle" style="background:#1ca6ff"></i></div><strong id="yaw">—</strong></div>
  </div>
  <div class="card"><h3>Компас (Yaw)</h3><canvas id="compass" width="240" height="205"></canvas></div>
  <div class="card">
   <h3>Виды модели</h3>
   <div class="viewGrid">
    <div class="viewItem active" onclick="setView('iso',this)">Изометрия</div>
    <div class="viewItem" onclick="setView('side',this)">Сбоку</div>
    <div class="viewItem" onclick="setView('front',this)">Спереди</div>
    <div class="viewItem" onclick="setView('top',this)">Сверху</div>
   </div>
  </div>
  <div class="card">
   <h3>Состояние</h3>
   <div class="kv">
    <span>Runtime</span><span id="runState">Остановлен</span>
    <span>FC</span><span id="arm">—</span>
    <span>Inliers</span><span id="inl">—</span>
    <span>Frame</span><span id="frame">—</span>
    <span>EKF</span><span id="ekf">—</span>
   </div>
  </div>
 </div>
</div>

<div class="footer" id="system">
 <div>● &nbsp; monkeysStab Web UI &nbsp; | &nbsp; Raspberry Pi 5</div>
 <div>MAVLink router: <b id="routerStatus" style="color:#16d979">OK</b> &nbsp; | &nbsp; Mission Planner: UDP 14550 &nbsp; | &nbsp; Runtime: <b id="footerRuntime">остановлен</b></div>
</div>

<script>
const $=id=>document.getElementById(id);
let latest=null,fcLatest=null;
let viewMode='iso',viewYaw=.75,viewPitch=.65,viewDist=6.4;
let drag=false,lastX=0,lastY=0;

async function api(path,opt){let r=await fetch(path,opt);let j=await r.json();if(!r.ok)throw new Error(j.error||r.statusText);return j}
function goSection(id,btn){
 document.querySelectorAll('.nav button').forEach(x=>x.classList.remove('active'));btn.classList.add('active');
 let el=$(id);if(el)el.scrollIntoView({behavior:'smooth',block:id==='system'?'end':'start'});
}
function fmt(v,d=1){return v==null||!Number.isFinite(Number(v))?'—':Number(v).toFixed(d)}
function clamp(v,a,b){return Math.max(a,Math.min(b,v))}
function setActiveMode(mode){
 ['mStab','mPos','mLoi'].forEach(id=>$(id).classList.remove('blue'));
 if(mode==='Stabilize')$('mStab').classList.add('blue');
 if(mode==='PosHold')$('mPos').classList.add('blue');
 if(mode==='Loiter')$('mLoi').classList.add('blue');
}
async function loadConfig(){
 let j=await api('/api/config'),c=j.runtime;
 $('focal').value=c.focal_scale;[$('r0').value,$('r1').value,$('r2').value,$('r3').value]=c.feature_roi;$('features').value=c.max_features;
 let g=j.geometry||{},gv=[];
 if(g.camera){gv.push(['FLOW_POS_X',g.camera.x*1000],['FLOW_POS_Y',g.camera.y*1000],['FLOW_POS_Z',g.camera.z*1000])}
 if(g.rangefinder){gv.push(['RNGFND1_POS_X',g.rangefinder.x*1000],['RNGFND1_POS_Y',g.rangefinder.y*1000],['RNGFND1_POS_Z',g.rangefinder.z*1000])}
 $('geomKv').innerHTML=gv.map(x=>'<span>'+x[0]+'</span><span>'+fmt(x[1],1)+'</span>').join('');
 let p=(j.fc_profile||{}).params||{},keys=['EK3_SRC1_POSXY','EK3_SRC1_VELXY','EK3_SRC1_POSZ','EK3_SRC1_YAW','FLOW_TYPE','FLOW_FXSCALER','FLOW_FYSCALER','EK3_FLOW_DELAY'];
 $('profileKv').innerHTML=keys.map(k=>'<span>'+k+'</span><span>'+(p[k]??'—')+'</span>').join('');
}
async function saveConfig(){
 try{
  let body={focal_scale:+$('focal').value,feature_roi:[+$('r0').value,+$('r1').value,+$('r2').value,+$('r3').value],max_features:+$('features').value,local_gui:true};
  await api('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  $('saveMsg').textContent='Сохранено';
 }catch(e){$('saveMsg').textContent='Ошибка: '+e.message}
}
async function start(){try{await api('/api/start',{method:'POST'});}catch(e){alert(e.message)}}
async function stop(){try{await api('/api/stop',{method:'POST'});}catch(e){alert(e.message)}}
async function zero(){try{await api('/api/zero',{method:'POST'});}catch(e){alert(e.message)}}
async function armFc(){if(!confirm('ARM: разрешить запуск моторов?'))return;try{showFc(await api('/api/fc/arm',{method:'POST'}))}catch(e){alert(e.message)}}
async function disarmFc(){if(!confirm('DISARM: отключить моторы?'))return;try{showFc(await api('/api/fc/disarm',{method:'POST'}))}catch(e){alert(e.message)}}
async function setMode(id,name){if(!confirm('Переключить режим на '+name+'?'))return;try{showFc(await api('/api/fc/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:id})}))}catch(e){alert(e.message)}}
function showFc(j){
 fcLatest=j;$('linkDot').classList.remove('baddot');$('linkText').textContent='OK';$('fcDotBig').classList.remove('baddot');
 $('fcState').textContent=j.armed?'ARMED':'DISARMED';$('fcState').style.color=j.armed?'#ff5967':'#e9f3fb';$('fcMode').textContent=j.mode;$('arm').textContent=j.armed?'ARMED':'DISARMED';setActiveMode(j.mode);
}
async function refreshFc(){try{showFc(await api('/api/fc'))}catch(e){$('linkDot').classList.add('baddot');$('linkText').textContent='НЕТ';$('fcDotBig').classList.add('baddot');$('fcState').textContent='НЕТ СВЯЗИ';$('fcMode').textContent='—'}}

function updateHud(t){
 latest=t;$('runState').textContent=t.running?'Работает':'Остановлен';$('footerRuntime').textContent=t.running?'работает':'остановлен';$('footerRuntime').style.color=t.running?'#15d876':'#8aa5b8';
 $('mx').textContent=fmt(t.x_mm,0)+' мм';$('my').textContent=fmt(t.y_mm,0)+' мм';$('mz').textContent=fmt(t.z_mm,0)+' мм';$('mr').textContent=t.range_m==null?'—':fmt(t.range_m*1000,0)+' мм';$('mq').textContent=t.quality??'—';
 $('roll').textContent=fmt(t.roll_deg,1)+'°';$('pitch').textContent=fmt(t.pitch_deg,1)+'°';$('yaw').textContent=fmt(t.yaw_deg,1)+'°';
 $('inl').textContent=(t.inliers??'—')+'/'+(t.tracked??'—');$('frame').textContent=t.frame??'—';$('ekf').textContent=t.ekf_valid?'VALID':'NO DATA';
 $('sceneXYZ').textContent='X '+fmt((t.x_mm||0)/1000,3)+' · Y '+fmt((t.y_mm||0)/1000,3)+' · Z '+fmt((t.z_mm||0)/1000,3)+' m';
 $('rollNeedle').style.left=(50+clamp(t.roll_deg||0,-45,45)/45*50)+'%';
 $('pitchNeedle').style.left=(50+clamp(t.pitch_deg||0,-45,45)/45*50)+'%';
 let y=((t.yaw_deg||0)+180)%360-180;$('yawNeedle').style.left=(50+y/180*50)+'%';
 drawCompass(t.yaw_deg||0);drawHistory(t.history||[]);renderScene();
}

function drawCompass(deg){
 let c=$('compass'),ctx=c.getContext('2d'),w=c.width,h=c.height,cx=w/2,cy=h/2,R=82;ctx.clearRect(0,0,w,h);
 ctx.strokeStyle='#294a62';ctx.lineWidth=3;ctx.beginPath();ctx.arc(cx,cy,R,0,Math.PI*2);ctx.stroke();
 ctx.font='12px system-ui';ctx.fillStyle='#cad9e4';ctx.textAlign='center';ctx.textBaseline='middle';
 [['N',0],['E',90],['S',180],['W',270]].forEach(([q,d])=>{let a=(d-90)*Math.PI/180;ctx.fillText(q,cx+Math.cos(a)*(R-13),cy+Math.sin(a)*(R-13))});
 for(let d=0;d<360;d+=10){let a=(d-90)*Math.PI/180,r1=R-4,r2=d%30===0?R-13:R-9;ctx.strokeStyle='#557188';ctx.lineWidth=d%30===0?2:1;ctx.beginPath();ctx.moveTo(cx+Math.cos(a)*r1,cy+Math.sin(a)*r1);ctx.lineTo(cx+Math.cos(a)*r2,cy+Math.sin(a)*r2);ctx.stroke()}
 let a=(deg-90)*Math.PI/180;ctx.fillStyle='#1ca8ff';ctx.beginPath();ctx.moveTo(cx+Math.cos(a)*(R-24),cy+Math.sin(a)*(R-24));ctx.lineTo(cx+Math.cos(a+2.55)*20,cy+Math.sin(a+2.55)*20);ctx.lineTo(cx+Math.cos(a-2.55)*20,cy+Math.sin(a-2.55)*20);ctx.closePath();ctx.fill();
 ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(cx,cy,5,0,Math.PI*2);ctx.fill();ctx.font='bold 22px system-ui';ctx.fillText(fmt(deg,0)+'°',cx,cy+42);
}
function chartBase(c,ctx){
 let w=c.width=c.clientWidth*devicePixelRatio,h=c.height=c.clientHeight*devicePixelRatio;ctx.scale(devicePixelRatio,devicePixelRatio);w=c.clientWidth;h=c.clientHeight;
 ctx.fillStyle='#08131d';ctx.fillRect(0,0,w,h);ctx.strokeStyle='#183246';ctx.lineWidth=1;
 for(let i=1;i<5;i++){let y=i*h/5;ctx.beginPath();ctx.moveTo(32,y);ctx.lineTo(w-8,y);ctx.stroke()}
 for(let i=1;i<6;i++){let x=32+i*(w-40)/6;ctx.beginPath();ctx.moveTo(x,10);ctx.lineTo(x,h-22);ctx.stroke()}
 return [w,h];
}
function plotSeries(ctx,data,key,min,max,color,w,h){
 let vals=data.map(d=>d[key]).filter(v=>v!=null&&Number.isFinite(v));if(!vals.length)return;
 if(min==null){min=Math.min(...vals);max=Math.max(...vals);if(Math.abs(max-min)<1e-6){min-=1;max+=1}else{let p=(max-min)*.15;min-=p;max+=p}}
 ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();let started=false;
 data.forEach((d,i)=>{let v=d[key];if(v==null||!Number.isFinite(v))return;let x=32+i*(w-44)/Math.max(1,data.length-1),y=10+(max-v)/(max-min)*(h-34);if(!started){ctx.moveTo(x,y);started=true}else ctx.lineTo(x,y)});ctx.stroke();
}
function drawHistory(h){
 let c=$('xyzChart'),ctx=c.getContext('2d'),[w,hh]=chartBase(c,ctx);let vals=[];h.forEach(d=>['x','y','z'].forEach(k=>{if(d[k]!=null)vals.push(d[k])}));let m=Math.max(.05,...vals.map(Math.abs));plotSeries(ctx,h,'x',-m,m,'#ff4352',w,hh);plotSeries(ctx,h,'y',-m,m,'#16d878',w,hh);plotSeries(ctx,h,'z',-m,m,'#218cff',w,hh);
 c=$('speedChart');ctx=c.getContext('2d');[w,hh]=chartBase(c,ctx);plotSeries(ctx,h,'speed',0,Math.max(.2,...h.map(d=>d.speed||0))*1.15,'#ffd11f',w,hh);
 c=$('rangeChart');ctx=c.getContext('2d');[w,hh]=chartBase(c,ctx);let rv=h.map(d=>d.range).filter(v=>v!=null),rmax=Math.max(.5,...rv)*1.2;plotSeries(ctx,h,'range',0,rmax,'#c98cff',w,hh);
}

let gl,prog,bufPos,bufCol,locMvp,locPos,locCol;
function m4mul(a,b){let o=new Float32Array(16);for(let c=0;c<4;c++)for(let r=0;r<4;r++){let v=0;for(let k=0;k<4;k++)v+=a[k*4+r]*b[c*4+k];o[c*4+r]=v}return o}
function perspective(fov,asp,n,f){let t=1/Math.tan(fov/2);return new Float32Array([t/asp,0,0,0,0,t,0,0,0,0,(f+n)/(n-f),-1,0,0,2*f*n/(n-f),0])}
function lookAt(e,t,u){let z=norm(sub(e,t)),x=norm(cross(u,z)),y=cross(z,x);return new Float32Array([x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,-dot(x,e),-dot(y,e),-dot(z,e),1])}
const sub=(a,b)=>a.map((v,i)=>v-b[i]),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],norm=a=>{let l=Math.hypot(...a)||1;return a.map(v=>v/l)}
function initGL(){
 let c=$('glCanvas');gl=c.getContext('webgl',{antialias:true,alpha:false});if(!gl)return;
 let vs=gl.createShader(gl.VERTEX_SHADER);gl.shaderSource(vs,'attribute vec3 p;attribute vec3 c;uniform mat4 m;varying vec3 v;void main(){gl_Position=m*vec4(p,1.0);v=c;gl_PointSize=8.0;}');gl.compileShader(vs);
 let fs=gl.createShader(gl.FRAGMENT_SHADER);gl.shaderSource(fs,'precision mediump float;varying vec3 v;void main(){gl_FragColor=vec4(v,1.0);}');gl.compileShader(fs);
 prog=gl.createProgram();gl.attachShader(prog,vs);gl.attachShader(prog,fs);gl.linkProgram(prog);gl.useProgram(prog);
 locPos=gl.getAttribLocation(prog,'p');locCol=gl.getAttribLocation(prog,'c');locMvp=gl.getUniformLocation(prog,'m');bufPos=gl.createBuffer();bufCol=gl.createBuffer();
 c.onmousedown=e=>{drag=true;lastX=e.clientX;lastY=e.clientY};window.onmouseup=()=>drag=false;window.onmousemove=e=>{if(!drag)return;viewYaw+=(e.clientX-lastX)*.008;viewPitch=clamp(viewPitch+(e.clientY-lastY)*.008,.1,1.45);lastX=e.clientX;lastY=e.clientY;renderScene()};
 c.onwheel=e=>{e.preventDefault();viewDist=clamp(viewDist+e.deltaY*.005,3.5,11);renderScene()};
}
function addLine(P,C,a,b,col){P.push(...a,...b);C.push(...col,...col)}
function addCircle(P,C,center,r,col,plane='xy'){let n=28;for(let i=0;i<n;i++){let a=i/n*Math.PI*2,b=(i+1)/n*Math.PI*2,A=[...center],B=[...center];if(plane==='xy'){A[0]+=Math.cos(a)*r;A[1]+=Math.sin(a)*r;B[0]+=Math.cos(b)*r;B[1]+=Math.sin(b)*r}else{A[0]+=Math.cos(a)*r;A[2]+=Math.sin(a)*r;B[0]+=Math.cos(b)*r;B[2]+=Math.sin(b)*r}addLine(P,C,A,B,col)}}
function rotLocal(p,r,pit,y){let cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(pit),sp=Math.sin(pit),cy=Math.cos(y),sy=Math.sin(y);let [x,Y,z]=p;let y1=cr*Y-sr*z,z1=sr*Y+cr*z,x1=x;let x2=cp*x1+sp*z1,y2=y1,z2=-sp*x1+cp*z1;return [cy*x2-sy*y2,sy*x2+cy*y2,z2]}
function renderScene(){
 if(!gl)return;let c=$('glCanvas'),dpr=devicePixelRatio,w=Math.floor(c.clientWidth*dpr),h=Math.floor(c.clientHeight*dpr);if(c.width!==w||c.height!==h){c.width=w;c.height=h}gl.viewport(0,0,w,h);gl.clearColor(.025,.065,.095,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.enable(gl.DEPTH_TEST);
 let P=[],C=[];
 if($('showGrid').checked){for(let i=-10;i<=10;i++){let q=i*.25;addLine(P,C,[-2.5,q,0],[2.5,q,0],[.08,.23,.34]);addLine(P,C,[q,-2.5,0],[q,2.5,0],[.08,.23,.34])}}
 if($('showAxes').checked){addLine(P,C,[0,0,0],[1.15,0,0],[1,.15,.15]);addLine(P,C,[0,0,0],[0,1.15,0],[.1,1,.25]);addLine(P,C,[0,0,0],[0,0,1.15],[.1,.45,1])}
 addCircle(P,C,[0,0,.01],.08,[.1,1,.35]);
 if(latest&&$('showTrail').checked&&(latest.trail||[]).length>1){let tr=latest.trail;for(let i=1;i<tr.length;i++){let a=tr[i-1],b=tr[i];addLine(P,C,[a.x_mm/1000,a.y_mm/1000,-(a.z_mm||0)/1000],[b.x_mm/1000,b.y_mm/1000,-(b.z_mm||0)/1000],[.05,.75,1])}}
 let pos=latest?[(latest.x_mm||0)/1000,(latest.y_mm||0)/1000,-(latest.z_mm||0)/1000]:[0,0,.2],rr=(latest?.roll_deg||0)*Math.PI/180,pp=(latest?.pitch_deg||0)*Math.PI/180,yy=(latest?.yaw_deg||0)*Math.PI/180;
 function wp(v){let q=rotLocal(v,rr,pp,yy);return[q[0]+pos[0],q[1]+pos[1],q[2]+pos[2]]}
 let arm=.32;addLine(P,C,wp([arm,arm,0]),wp([-arm,-arm,0]),[.7,.78,.84]);addLine(P,C,wp([arm,-arm,0]),wp([-arm,arm,0]),[.7,.78,.84]);
 [[arm,arm],[-arm,-arm],[arm,-arm],[-arm,arm]].forEach((xy,i)=>{let ctr=wp([xy[0],xy[1],.03]),n=30;for(let k=0;k<n;k++){let a=k/n*Math.PI*2,b=(k+1)/n*Math.PI*2,A=wp([xy[0]+Math.cos(a)*.17,xy[1]+Math.sin(a)*.17,.03]),B=wp([xy[0]+Math.cos(b)*.17,xy[1]+Math.sin(b)*.17,.03]);addLine(P,C,A,B,i<2?[.1,.9,.55]:[.25,.55,1])}});
 let body=[[-.12,-.08,-.05],[.12,-.08,-.05],[.12,.08,-.05],[-.12,.08,-.05],[-.12,-.08,.07],[.12,-.08,.07],[.12,.08,.07],[-.12,.08,.07]],edges=[[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];edges.forEach(e=>addLine(P,C,wp(body[e[0]]),wp(body[e[1]]),[1,.45,.08]));addLine(P,C,wp([.1,0,.02]),wp([.48,0,.02]),[1,.1,.1]);
 let a=viewYaw,p=viewPitch;if(viewMode==='top'){a=0;p=.05}else if(viewMode==='front'){a=Math.PI/2;p=.4}else if(viewMode==='side'){a=0;p=.4}
 let eye=[Math.cos(a)*Math.cos(p)*viewDist,Math.sin(a)*Math.cos(p)*viewDist,Math.sin(p)*viewDist],target=$('followCam').checked?pos:[0,0,.25],V=lookAt(eye,target,[0,0,1]),Pr=perspective(.8,w/h,.05,40),M=m4mul(Pr,V);gl.uniformMatrix4fv(locMvp,false,M);
 gl.bindBuffer(gl.ARRAY_BUFFER,bufPos);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(P),gl.DYNAMIC_DRAW);gl.enableVertexAttribArray(locPos);gl.vertexAttribPointer(locPos,3,gl.FLOAT,false,0,0);
 gl.bindBuffer(gl.ARRAY_BUFFER,bufCol);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(C),gl.DYNAMIC_DRAW);gl.enableVertexAttribArray(locCol);gl.vertexAttribPointer(locCol,3,gl.FLOAT,false,0,0);gl.drawArrays(gl.LINES,0,P.length/3);
}
function setView(v,el){viewMode=v;document.querySelectorAll('.viewItem').forEach(x=>x.classList.remove('active'));el.classList.add('active');renderScene()}
function resetView(){viewMode='iso';viewYaw=.75;viewPitch=.65;viewDist=6.4;renderScene()}

async function refresh(){
 try{let t=await api('/api/telemetry');updateHud(t);let l=await api('/api/log');$('log').textContent=l.text||'';$('log').scrollTop=$('log').scrollHeight}catch(e){}
}
setInterval(()=>{$('clock').textContent=new Date().toLocaleTimeString('ru-RU')},1000);
loadConfig();initGL();refresh();refreshFc();setInterval(refresh,700);setInterval(refreshFc,1800);window.addEventListener('resize',()=>{renderScene();if(latest)drawHistory(latest.history||[])});
</script>
</body>
</html>'''

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
