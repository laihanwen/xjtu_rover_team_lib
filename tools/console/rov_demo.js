/* Pure browser simulation: no fetch, control sockets or camera connections. */
let armed=false,recording=false,t=0;
function put(id,value){const e=document.getElementById(id);if(e)e.textContent=value}
function act(a){armed=a==='arm';put('mode',armed?'模拟 ARM':'模拟 DISARM');put('gate','虚拟状态 · 不发送命令')}
function speedMode(m){put('speed-state',m==='low'?'模拟低速':'模拟提速')}
function recordAction(a){recording=a==='start';put('record-state',recording?'模拟双摄录制 · 不保存视频':'模拟录制停止');document.getElementById('record-start').disabled=recording;document.getElementById('record-stop').disabled=!recording}
function togglePreview(){put('camera-down-state','合成下视画面');put('camera-front-state','合成前视画面')}
addEventListener('DOMContentLoaded',()=>{
 for(const id of ['arm','level','speed-low','speed-higher'])document.getElementById(id).disabled=false;
 for(const role of ['down','front']){const v=document.getElementById('camera-'+role).parentElement;v.innerHTML='<canvas width="640" height="480" aria-label="模拟'+role+'摄像头"></canvas>';const c=v.firstChild,x=c.getContext('2d');x.fillStyle='#12465a';x.fillRect(0,0,640,480);x.strokeStyle='#407589';for(let i=0;i<10;i++){x.beginPath();x.moveTo(i*80,0);x.lineTo(i*80,480);x.moveTo(0,i*60);x.lineTo(640,i*60);x.stroke()}x.fillStyle='#e8eeee';x.fillRect(260,185,120,110);x.fillStyle='#102332';for(let j=0;j<5;j++)for(let i=0;i<5;i++)if((i+j)%2)x.fillRect(270+i*20,190+j*20,20,20);x.fillStyle='#fff';x.font='18px sans-serif';x.fillText('合成画面 · '+role,20,35);put('camera-'+role+'-state','模拟视频 · 无相机连接')}
 put('mode','模拟 DISARM');put('level-state','模拟水平基准');put('detail','ROV 独立遥控界面 · 虚拟检视');put('device','无硬件连接');
 setInterval(()=>{t+=.2;put('depth-value',(0.7+Math.sin(t)*.01).toFixed(3)+' m');put('pitch-value',(Math.sin(t)*.6).toFixed(1)+'°');put('roll-value',(Math.cos(t)*.4).toFixed(1)+'°');put('yaw-value','12.0°');put('age-value','模拟 20 ms');put('hold-state',armed?'模拟姿态保持':'模拟待命')},200);
});
addEventListener('DOMContentLoaded',()=>{put('record-state','模拟录制已停止 · 不保存实际视频');put('log-state','模拟日志 · 无设备遥测');put('preview-toggle','查看合成双摄')});
