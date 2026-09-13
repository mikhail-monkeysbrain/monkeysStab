import * as THREE from '/assets/three.module.js';
import {GLTFLoader} from '/assets/GLTFLoader.js';

const statusEl=document.getElementById('advanced3dStatus');
function set3dStatus(text,bad=false){
 if(!statusEl)return;
 statusEl.textContent=text;
 statusEl.style.display=text?'block':'none';
 statusEl.style.color=bad?'#ff6672':'#9fb8ca';
 statusEl.style.borderColor=bad?'#a5323c':'#31516a';
}

const host=document.getElementById('advancedScene');
const scene=new THREE.Scene();
scene.background=new THREE.Color(0x07121c);
scene.fog=new THREE.FogExp2(0x07121c,0.055);

const camera=new THREE.PerspectiveCamera(48,1,0.03,60);
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});
renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));
renderer.outputColorSpace=THREE.SRGBColorSpace;
renderer.shadowMap.enabled=true;
renderer.shadowMap.type=THREE.PCFSoftShadowMap;
host.appendChild(renderer.domElement);
set3dStatus('Three.js запущен, загружаю модель…');

scene.add(new THREE.HemisphereLight(0xb9ddff,0x102030,1.65));
const sun=new THREE.DirectionalLight(0xffffff,2.0);
sun.position.set(3,6,2);
sun.castShadow=true;
scene.add(sun);

const grid=new THREE.GridHelper(5,20,0x1979a8,0x103b55);
grid.position.y=0;
grid.material.transparent=true;
grid.material.opacity=.78;
scene.add(grid);

const axes=new THREE.Group();
function axisLine(a,b,color){
 const g=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...a),new THREE.Vector3(...b)]);
 const m=new THREE.LineBasicMaterial({color});
 axes.add(new THREE.Line(g,m));
}
axisLine([0,0,0],[1.2,0,0],0xff4050);   // X / North
axisLine([0,0,0],[0,1.2,0],0x2f8cff);   // Up
axisLine([0,0,0],[0,0,1.2],0x28ef5e);   // Y / East
scene.add(axes);

const homeMat=new THREE.MeshBasicMaterial({color:0x12e96f,side:THREE.DoubleSide});
const home=new THREE.Mesh(new THREE.RingGeometry(.065,.085,48),homeMat);
home.rotation.x=-Math.PI/2;
home.position.set(0,0,0);
scene.add(home);

const trailMat=new THREE.LineBasicMaterial({color:0x1ab7ff});
let trailGeom=new THREE.BufferGeometry();
let trailLine=new THREE.Line(trailGeom,trailMat);
scene.add(trailLine);

const droneRoot=new THREE.Group();
scene.add(droneRoot);
let droneModel=null;

// NED -> Three scene:
//   N (FC X / North) -> scene +X
//   E (FC Y / East)  -> scene +Z
//   D (FC Z / Down)  -> scene -Y
// This keeps the ground plane at scene Y=0 and makes yaw=0 point the nose
// exactly along the visible +X axis.
const sceneFromNed=new THREE.Matrix4().set(
 1, 0, 0, 0,
 0, 0,-1, 0,
 0, 1, 0, 0,
 0, 0, 0, 1
);
const qSceneFromNed=new THREE.Quaternion().setFromRotationMatrix(sceneFromNed);

// CesiumDrone/glTF is right-handed and Y-up; its visual forward is -Z.
// Map model axes to ArduPilot body FRD with a proper rotation (det=+1):
// model +X (right) -> body +Y (right)
// model +Y (up)    -> body -Z (up)
// model -Z (front) -> body +X (forward)
// The previous +Z->+X mapping had det=-1 (reflection), so the zero-attitude
// quaternion was invalid and the aircraft appeared on its side.
const bodyFromModel=new THREE.Matrix4().set(
 0, 0,-1, 0,
 1, 0, 0, 0,
 0,-1, 0, 0,
 0, 0, 0, 1
);
const qBodyFromModel=new THREE.Quaternion().setFromRotationMatrix(bodyFromModel);

set3dStatus('Загрузка 3D-модели…');
new GLTFLoader().load('/assets/CesiumDrone.glb',gltf=>{
 droneModel=gltf.scene;
 droneModel.traverse(o=>{if(o.isMesh){o.castShadow=true;o.receiveShadow=true}});
 const box=new THREE.Box3().setFromObject(droneModel);
 const size=new THREE.Vector3(); box.getSize(size);
 const center=new THREE.Vector3(); box.getCenter(center);
 droneModel.position.sub(center);
 const span=Math.max(size.x,size.y,size.z)||1;
 const targetSpan=.48; // physical display span ~48 cm on the metre grid
 const scale=targetSpan/span;
 droneModel.scale.setScalar(scale);

 // The FC position is the vehicle reference point, not the landing gear.
 // Keep the centered GLB origin exactly at the telemetry X/Y/Z point.
 droneRoot.add(droneModel);
 set3dStatus('');
},undefined,e=>{console.error('GLB load failed',e);set3dStatus('Ошибка загрузки GLB: '+(e?.message||e),true)});

let enabled=false;
let az=Math.PI*.25, el=Math.PI*.28, dist=4.8;
let target=new THREE.Vector3(0,0,0);
let currentPos=new THREE.Vector3();
let dragging=false,lastX=0,lastY=0;

function updateCamera(){
 const follow=document.getElementById('followCam')?.checked;
 const t=follow?currentPos:target;
 const ce=Math.cos(el);
 camera.position.set(
   t.x + dist*ce*Math.sin(az),
   t.y + dist*Math.sin(el),
   t.z + dist*ce*Math.cos(az)
 );
 camera.lookAt(t);
}
function resize(){
 const w=Math.max(2,host.clientWidth),h=Math.max(2,host.clientHeight);
 renderer.setSize(w,h,false);
 camera.aspect=w/h;
 camera.updateProjectionMatrix();
}
function setView(v){
 if(v==='top'){az=0;el=Math.PI/2-.02;dist=4.1}
 else if(v==='front'){az=0;el=.20;dist=4.7}
 else if(v==='side'){az=Math.PI/2;el=.20;dist=4.7}
 else {az=Math.PI*.25;el=Math.PI*.28;dist=4.8}
 updateCamera();
}
function resetView(){setView('iso')}

renderer.domElement.addEventListener('pointerdown',e=>{
 if(!enabled)return;
 dragging=true;lastX=e.clientX;lastY=e.clientY;
 renderer.domElement.setPointerCapture(e.pointerId);
});
renderer.domElement.addEventListener('pointermove',e=>{
 if(!dragging||!enabled)return;
 az-=(e.clientX-lastX)*.008;
 el=THREE.MathUtils.clamp(el+(e.clientY-lastY)*.008,.05,Math.PI/2-.03);
 lastX=e.clientX;lastY=e.clientY;
 updateCamera();
});
renderer.domElement.addEventListener('pointerup',e=>{
 dragging=false;
 try{renderer.domElement.releasePointerCapture(e.pointerId)}catch(_){}
});
renderer.domElement.addEventListener('wheel',e=>{
 if(!enabled)return;
 e.preventDefault();
 dist=THREE.MathUtils.clamp(dist+e.deltaY*.004,1.8,12);
 updateCamera();
},{passive:false});

function update(t){
 const n=(Number(t.x_mm)||0)/1000;
 const e=(Number(t.y_mm)||0)/1000;
 const d=(Number(t.z_mm)||0)/1000;
 currentPos.set(n,-d,e);
 droneRoot.position.copy(currentPos);

 const roll=THREE.MathUtils.degToRad(Number(t.roll_deg)||0);
 const pitch=THREE.MathUtils.degToRad(Number(t.pitch_deg)||0);
 const yaw=THREE.MathUtils.degToRad(Number(t.yaw_deg)||0);
 const qNed=new THREE.Quaternion().setFromEuler(new THREE.Euler(roll,pitch,yaw,'ZYX'));
 droneRoot.quaternion.copy(qSceneFromNed).multiply(qNed).multiply(qBodyFromModel);

 const pts=(t.trail||[]).map(p=>new THREE.Vector3(
   (Number(p.x_mm)||0)/1000,
   -(Number(p.z_mm)||0)/1000,
   (Number(p.y_mm)||0)/1000
 ));
 trailGeom.dispose();
 trailGeom=new THREE.BufferGeometry().setFromPoints(pts.length?pts:[new THREE.Vector3()]);
 trailLine.geometry=trailGeom;

 trailLine.visible=document.getElementById('showTrail')?.checked!==false;
 grid.visible=document.getElementById('showGrid')?.checked!==false;
 axes.visible=document.getElementById('showAxes')?.checked!==false;
 updateCamera();
}
function setEnabled(v){
 enabled=!!v;
 resize();
 updateCamera();
}
function frame(){
 if(enabled){
   trailLine.visible=document.getElementById('showTrail')?.checked!==false;
   grid.visible=document.getElementById('showGrid')?.checked!==false;
   axes.visible=document.getElementById('showAxes')?.checked!==false;
   renderer.render(scene,camera);
 }
 requestAnimationFrame(frame);
}
window.ThreeAdvanced={update,setView,resetView,resize,setEnabled};
resize();
updateCamera();

// The classic dashboard script can finish loading config before this ES module.
// Synchronize the renderer with the already-selected visualization mode here.
const initialAdvanced=(window.visualizationMode||'simple')==='advanced';
setEnabled(initialAdvanced);
if(initialAdvanced){
  host.style.display='block';
  if(window.latest) update(window.latest);
}
frame();
