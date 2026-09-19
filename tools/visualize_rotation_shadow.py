#!/usr/bin/env python3
"""Self-contained interactive visualizer for deltar_rotation_shadow.csv."""
import argparse,csv,json,math,webbrowser
from pathlib import Path

def f(row,key,default=0.0):
    try:
        v=float(row.get(key,default)); return v if math.isfinite(v) else default
    except Exception:return default

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv",nargs="?",help="deltar_rotation_shadow.csv; default = newest run")
    ap.add_argument("--out",default="/tmp/jtzero_rotation_shadow.html")
    ap.add_argument("--open",action="store_true")
    a=ap.parse_args()
    if a.csv:
        p=Path(a.csv)
    else:
        root=Path.home()/"monkeysStab_runs"
        q=sorted(root.glob("*/deltar_rotation_shadow.csv"),key=lambda x:x.stat().st_mtime,reverse=True)
        if not q: raise SystemExit("deltar_rotation_shadow.csv not found")
        p=q[0]
    rows=list(csv.DictReader(p.open(newline="")))
    arms=[
      ("WORKED5","w5_valid","w5_dN_m","w5_dE_m"),
      ("ATT ΔR + lever","deltar_valid","imu_dN_m","imu_dE_m"),
      ("ATT-rate ΔR + lever","gyro_valid","gyro_imu_dN_m","gyro_imu_dE_m"),
      ("HIGHRES ΔR + lever","highres_valid","highres_imu_dN_m","highres_imu_dE_m"),
    ]
    paths={}
    for name,v,n,e in arms:
        N=E=0.0; pts=[[0,0,0]]
        for r in rows:
            if int(f(r,v,0))!=1: continue
            N+=f(r,n)*1000; E+=f(r,e)*1000
            pts.append([E,N,0])
        paths[name]=pts
    hcN=hcE=hlN=hlE=0.0
    for r in rows:
        if int(f(r,"highres_valid",0))!=1: continue
        hcN+=f(r,"highres_camera_dN_m")*1000; hcE+=f(r,"highres_camera_dE_m")*1000
        hlN+=f(r,"highres_lever_dN_m")*1000; hlE+=f(r,"highres_lever_dE_m")*1000
    data={"source":str(p),"paths":paths,"camera":[hcE,hcN,0],"lever":[hlE,hlN,0],
          "imu":[hcE-hlE,hcN-hlN,0],"cameraLeverMm":[62.5,0,50.0]}
    html='''<!doctype html><meta charset="utf-8"><title>JT-Zero ΔR 3D</title>
<style>
body{margin:0;background:#0b0f14;color:#e8eef7;font:14px system-ui;overflow:hidden}
#c{position:absolute;inset:0;width:100%;height:100%}.panel{position:absolute;left:16px;top:16px;background:#111a24e8;border:1px solid #33465c;border-radius:10px;padding:14px;max-width:430px}
h2{margin:0 0 8px;font-size:17px}.muted{color:#9fb0c3}.row{margin:5px 0}.sw{display:inline-block;width:12px;height:3px;margin-right:7px;vertical-align:middle}
button{background:#203044;color:#fff;border:1px solid #45617e;border-radius:6px;padding:6px 9px;margin:5px 4px 0 0}
</style><canvas id="c"></canvas><div class="panel"><h2>JT-Zero — ΔR / lever-arm</h2>
<div class="muted">Мышь: вращение · колесо: масштаб. Оси: E=X, N=Y, Up=Z.</div>
<div id="legend"></div><hr><div><b>Что считается:</b> CAMERA — движение центра камеры из лучей; LEVER — движение камеры только из-за вращения вокруг IMU; IMU = CAMERA − LEVER.</div>
<div id="nums"></div><button onclick="view='top';draw()">Вид сверху</button><button onclick="view='iso';draw()">3D</button></div>
<script>
const D=__DATA__,C=document.getElementById('c'),g=C.getContext('2d');
let yaw=-.65,pitch=.72,zoom=3.0,view='iso',drag=false,lx=0,ly=0;
const colors={'WORKED5':'#ff5f57','ATT ΔR + lever':'#ffd166','ATT-rate ΔR + lever':'#5ac8fa','HIGHRES ΔR + lever':'#66e38a'};
function resize(){C.width=innerWidth*devicePixelRatio;C.height=innerHeight*devicePixelRatio;draw()} addEventListener('resize',resize);
function rot(p){let [x,y,z]=p;if(view==='top')return [x,y,0];let cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);let X=cy*x-sy*y,Y=sy*x+cy*y;return [X,cp*Y-sp*z,sp*Y+cp*z]}
function P(p){let q=rot(p),s=zoom*devicePixelRatio;return [C.width*.56+q[0]*s,C.height*.55-q[1]*s]}
function line(a,b,col,w=1){let A=P(a),B=P(b);g.strokeStyle=col;g.lineWidth=w*devicePixelRatio;g.beginPath();g.moveTo(...A);g.lineTo(...B);g.stroke()}
function poly(ps,col,w=2){if(ps.length<2)return;g.strokeStyle=col;g.lineWidth=w*devicePixelRatio;g.beginPath();let q=P(ps[0]);g.moveTo(...q);for(let i=1;i<ps.length;i++){q=P(ps[i]);g.lineTo(...q)}g.stroke()}
function dot(p,col,r=4){let q=P(p);g.fillStyle=col;g.beginPath();g.arc(q[0],q[1],r*devicePixelRatio,0,7);g.fill()}
function draw(){
 g.clearRect(0,0,C.width,C.height);g.fillStyle='#0b0f14';g.fillRect(0,0,C.width,C.height);
 for(let x=-150;x<=150;x+=25)line([x,-150,0],[x,150,0],'#172330');
 for(let y=-150;y<=150;y+=25)line([-150,y,0],[150,y,0],'#172330');
 line([-150,0,0],[150,0,0],'#49657f',1.5);line([0,-150,0],[0,150,0],'#49657f',1.5);
 // schematic drone centered at IMU
 line([-70,0,25],[70,0,25],'#8b9bad',4);line([0,-70,25],[0,70,25],'#8b9bad',4);
 dot([0,0,25],'#ffffff',6); // IMU
 const cam=[D.cameraLeverMm[0],D.cameraLeverMm[1],-D.cameraLeverMm[2]+25];
 line([0,0,25],cam,'#b388ff',3);dot(cam,'#b388ff',6);
 for(const [n,p] of Object.entries(D.paths))poly(p,colors[n],2.3);
 dot([0,0,0],'#fff',3);
 // endpoint decomposition vectors, lifted slightly for readability
 line([0,0,8],D.camera.map((v,i)=>i===2?8:v),'#ff8a65',3);
 line([0,0,14],D.lever.map((v,i)=>i===2?14:v),'#b388ff',3);
 line([0,0,20],D.imu.map((v,i)=>i===2?20:v),'#66e38a',3);
}
C.onmousedown=e=>{drag=true;lx=e.clientX;ly=e.clientY};onmouseup=()=>drag=false;
onmousemove=e=>{if(!drag||view==='top')return;yaw+=(e.clientX-lx)*.008;pitch+=(e.clientY-ly)*.008;pitch=Math.max(-1.4,Math.min(1.4,pitch));lx=e.clientX;ly=e.clientY;draw()};
C.onwheel=e=>{zoom*=Math.exp(-e.deltaY*.001);zoom=Math.max(.5,Math.min(12,zoom));draw();e.preventDefault()};
let L='';for(const n of Object.keys(D.paths))L+=`<div class=row><span class=sw style="background:${colors[n]}"></span>${n}: <b>${Math.hypot(...D.paths[n].at(-1).slice(0,2)).toFixed(1)} mm</b></div>`;
document.getElementById('legend').innerHTML=L;
const mag=v=>Math.hypot(v[0],v[1]).toFixed(1);
document.getElementById('nums').innerHTML=`<div class=row>HIGHRES CAMERA: <b>${mag(D.camera)} mm</b></div><div class=row>HIGHRES LEVER: <b>${mag(D.lever)} mm</b></div><div class=row>остаток IMU: <b>${mag(D.imu)} mm</b></div><div class=muted>${D.source}</div>`;
resize();
</script>'''.replace("__DATA__",json.dumps(data,ensure_ascii=False))
    out=Path(a.out); out.write_text(html,encoding="utf-8")
    print(out)
    print("Открой в браузере: file://"+str(out))
    if a.open:webbrowser.open(out.as_uri())
if __name__=="__main__":main()
