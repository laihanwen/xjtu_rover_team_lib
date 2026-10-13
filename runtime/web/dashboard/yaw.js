'use strict';
const el=id=>document.getElementById(id);
let status=null,trial=null,results=[],lastAt=0;
const wrap=x=>((x+180)%360+360)%360-180;
// STATUS has a combined sensor fault, not an independent IMU validity bit.
// Depth commissioning can set that fault; record it without claiming IMU validation.
const usable=s=>s.status_fresh===true&&s.serial===true&&s.armed===false&&Number.isFinite(s.yaw_rad)&&Number.isFinite(s.status_age_sec)&&s.status_age_sec>=0&&s.status_age_sec<=0.5;
function invalidate(reason){if(trial){trial.invalid=reason;el('result').textContent='本次无效：'+reason;}el('end').disabled=true;}
el('begin').onclick=()=>{
 if(!status||!usable(status)||Date.now()-lastAt>1500)return;
 trial={run_id:status.run_id,direction:Number(el('direction').value),started_at:new Date().toISOString(),last:status.yaw_rad*180/Math.PI,delta:0,samples:[],invalid:null};
 el('end').disabled=false;el('result').textContent='正在记录，请缓慢转动机体。';
};
el('end').onclick=()=>{
 if(!trial||trial.invalid||!usable(status)||Date.now()-lastAt>1500)return;
 const sign=Math.abs(trial.delta)>=20?Math.sign(trial.delta)*trial.direction:null;
 results.push({...trial,finished_at:new Date().toISOString(),suggested_imu_yaw_sign:sign});
 el('result').textContent=sign?'累计变化 '+trial.delta.toFixed(1)+'°；建议 imu yaw 符号 '+(sign>0?'+1':'-1')+'。请反向复测；本页不自动应用标定。':'转角不足 20°，无符号结论。';
 trial=null;el('end').disabled=true;el('export').disabled=false;
};
el('export').onclick=()=>{const blob=new Blob([JSON.stringify({endpoint:location.host,exported_at:new Date().toISOString(),convention:'X forward, Y left, Z up; CCW viewed from above positive',results},null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='yaw-calibration-'+Date.now()+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
async function poll(){try{
 const response=await fetch('/api/status',{cache:'no-store',signal:AbortSignal.timeout(1500)});if(!response.ok)throw Error('HTTP '+response.status);
 const s=await response.json(),now=Date.now();
 if(trial&&(now-lastAt>1500||s.run_id!==trial.run_id||!usable(s)||(Number.isFinite(s.imu_tick_ms)&&Number.isFinite(status?.imu_tick_ms)&&s.imu_tick_ms<status.imu_tick_ms)))invalidate('遥测过期、ARM、MCU重启或运行会话变化');
 status=s;lastAt=now;el('begin').disabled=!usable(s);
 const angle=s.yaw_rad*180/Math.PI;el('yaw').textContent=usable(s)?angle.toFixed(2)+'°':'无有效 DISARM yaw';
 el('health').textContent=usable(s)?(s.yaw_source||'Pi Runtime')+' / 遥测新鲜 / DISARM；综合故障位 '+s.error_flags:'等待新鲜 yaw 遥测和 DISARM';
 el('depth').textContent='深度：'+(s.depth_sample_fresh&&Number.isFinite(s.depth_m)?s.depth_m.toFixed(3)+' m':'无有效绝对深度；不影响独立 yaw 观察');
 if(trial&&!trial.invalid){const step=wrap(angle-trial.last);if(Math.abs(step)>45)invalidate('单次角度跳变超过 45°，请重新缓慢测试');else{trial.delta+=step;trial.last=angle;if(trial.samples.length>=1200)invalidate('记录已达上限，请重新测试');else trial.samples.push({received_at:new Date().toISOString(),yaw_deg:angle,status_age_sec:s.status_age_sec,error_flags:s.error_flags,delta_deg:trial.delta});}}
 }catch(error){invalidate('连接中断');el('begin').disabled=true;el('health').textContent=error.message;el('yaw').textContent='—';}finally{setTimeout(poll,200);}}
poll();
