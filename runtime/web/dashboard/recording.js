'use strict';
// Reuse the existing recorder; never open cameras or alter vehicle control.
const recordingHead=document.createElement('div');
recordingHead.className='camera-recording';
document.querySelector('.cameras .camera-pair').before(recordingHead);
const recordingLabel=document.createElement('b');recordingLabel.textContent='双摄录制';
recordingHead.append(recordingLabel,document.getElementById('record-status'));
const recordingActions=document.createElement('div');
const recordStart=document.createElement('button'),recordStop=document.createElement('button');
recordStart.textContent='开始双摄录制';recordStop.textContent='停止双摄录制';
recordStart.className=recordStop.className='secondary';
recordStart.disabled=recordStop.disabled=true;
recordingActions.append(recordStart,recordStop);recordingHead.append(recordingActions);
const recordingDetail=document.createElement('p');recordingDetail.className='footnote';
recordingHead.append(recordingDetail);
let demoRecording=true,recordBusy=false;
async function refreshRecording(){
  if(!config)return;
  if(config.demo){
    recordStart.disabled=demoRecording;recordStop.disabled=!demoRecording;
    recordStart.textContent='模拟开始双摄录制';recordStop.textContent='模拟停止双摄录制';
    text('record-status',demoRecording?'模拟录制中':'模拟录制已停止');
    text('record-value',demoRecording?'前视 + 下视 · 模拟':'双摄录制已停止 · 模拟');
    recordingDetail.textContent='虚拟录像状态，不生成视频文件、不连接设备。';return;
  }
  if(config.source==='auv'){
    recordStart.hidden=recordStop.hidden=true;
    const enabled=state?.recordingEnabled===true;
    text('record-status',enabled?(state.recording?'机载双摄记录就绪':'机载记录异常'):'机载记录未启用');
    recordingDetail.textContent='AUV 由机载 Runtime 自动保存双摄原图；本页只读，任务期间不提供停录。'+(state?.recordingDetail||'');return;
  }
  try{
    const s=await get('/recording');
    recordStart.disabled=recordBusy||s.active;recordStop.disabled=recordBusy||!s.active;
    text('record-status',s.error?'录制异常':s.active?'双摄录制中':'录制已停止');
    text('record-value',s.active?'前视 + 下视 · PC 录像':'PC 双摄录制已停止');
    recordingDetail.textContent=(s.error||'')+' 下视 '+(s.cameras?.down?.frames||0)+' 帧 / '+(s.cameras?.down?.received_fps||0)+' FPS；前视 '+(s.cameras?.front?.frames||0)+' 帧 / '+(s.cameras?.front?.received_fps||0)+' FPS。';
  }catch(e){recordStart.disabled=recordStop.disabled=true;text('record-status','录制状态不可用');recordingDetail.textContent=e.message;}
}
async function recordingAction(action){
  if(!config||recordBusy)return;
  if(config.demo){demoRecording=action==='start';addEvent('模拟 · 双摄录制'+(demoRecording?'开始':'停止'));await refreshRecording();return;}
  if(config.source!=='rov')return;
  recordBusy=true;recordStart.disabled=recordStop.disabled=true;
  try{const r=await fetch('/recording',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})});if(!r.ok)throw Error(await r.text());addEvent('PC 双摄录制'+(action==='start'?'开始':'停止'));}
  catch(e){dialog('录制请求未确认',e.message);}
  finally{recordBusy=false;await refreshRecording();}
}
recordStart.onclick=()=>recordingAction('start');recordStop.onclick=()=>recordingAction('stop');
setInterval(refreshRecording,700);refreshRecording();
