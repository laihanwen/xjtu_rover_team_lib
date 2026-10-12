'use strict';
// Read-only: one same-frame JSON/JPEG snapshot, no command endpoints.
(() => {
  const panel=document.createElement('section');panel.id='perception';panel.className='panel perception';
  panel.innerHTML=`<div class="panel-head"><div><h2>感知检查</h2><span class="muted">先看识别证据，再判断是否具备定位条件</span></div><span id="pv-source" class="mini-badge">等待数据源</span></div>
    <div class="pv-tools"><label>模拟场景 <select id="pv-demo"><option value="partial">标签与局部地图</option><option value="undecoded">看见标签但未解码</option><option value="ambiguous">网格关联不唯一</option><option value="stale">感知断流</option></select></label><button id="pv-export" class="secondary" disabled>保存感知快照</button></div>
    <div class="pv-layout"><div><div class="pv-image"><img id="pv-image" hidden alt="同一感知帧的标签和颜色候选叠加图"><p id="pv-empty">等待感知接口；普通视频能播放不代表算法已识别。</p></div><p id="pv-frame" class="footnote">暂无帧号</p><div class="pv-legend"><span>绿色框：已解码标签</span><span>橙色轮廓：颜色候选，尚非锥类别</span><span>红点：局部关联节点</span></div></div>
    <div class="pv-evidence"><article><h3>标签解码 <b id="pv-tag">等待</b></h3><p id="pv-tag-detail">只有当前帧解码出 ID 才算识别。</p></article><article><h3>局部网格 <b id="pv-grid-state">等待</b></h3><p id="pv-grid-detail">候选节点不等于唯一格位。</p></article><article><h3>定位证据 <b id="pv-pose">等待</b></h3><p id="pv-pose-detail">过期位姿不能用于运动。</p></article><article><h3>锥体识别 <b id="pv-cones">等待</b></h3><p id="pv-cone-detail">原图中的颜色轮廓不能直接判断圆锥或方锥。</p></article></div></div>
    <div class="pv-map-head"><h3>累计九宫格</h3><span id="pv-map-state">未知格不当空格</span></div><div class="pv-map-layout"><div><div id="pv-map" class="pv-map"></div><div class="pv-yellow">规范黄边朝下；不代表比赛绝对方向</div></div><div class="pv-help"><p id="pv-map-detail">等待观测证据。</p><b>如何确认“观察到了”</b><p>标签必须出现实际 ID；网格必须有唯一关联；锥必须有类别和格位。只有候选轮廓时，仍不能认为任务已完成。</p><p>可以保持 DISARM、人工搬动观察。此页只读，不启动任务、不解除运动门控。</p></div></div>`;
  document.querySelector('.cameras').after(panel);
  const q=id=>document.getElementById(id);let last=null,received=0;
  const set=(id,value,kind='')=>{q(id).textContent=value;q(id).className=kind;};
  const finite=v=>typeof v==='number'&&Number.isFinite(v);
  const names={unknown:'未知',empty:'已确认空格',circle:'圆锥',square:'方锥'};
  const reasons={
    'waiting for perception frame':'尚未收到任务一感知帧；确认服务版本与任务配置',
    'sensor input stale or missing':'传感器输入缺失或过期，局部定位未执行',
    'local calibration unverified':'局部几何参数未验证',
    'metric calibration unverified':'度量参数未验证',
    'grid not processed':'尚未处理网格',
    'topology or yellow-edge conflict':'管线拓扑或黄边方向冲突',
    'temporal association ambiguous or discontinuous':'连续帧关联仍有歧义或不连续'
  };
  const explain=s=>reasons[s]||s||'未提供原因';
  function render(p,demo=false){
    last=p;received=Date.now();q('pv-export').disabled=false;
    set('pv-source',demo?'模拟感知 · 非实测':p.frame_fresh?'实时识别证据':'感知未就绪',demo?'pv-warn':p.frame_fresh?'pv-ok':'pv-bad');
    q('pv-image').hidden=!p.frame_fresh||!p.image_url;q('pv-empty').hidden=!q('pv-image').hidden;
    if(!q('pv-image').hidden)q('pv-image').src=p.image_url;else {q('pv-image').removeAttribute('src');set('pv-empty',p.sequence?'感知帧已过期；旧图已隐藏。':explain(p.reason));}
    set('pv-frame',`${demo?'模拟 · ':''}${p.camera||'相机未知'} / ${p.model||'模型未知'} / ${p.phase||'阶段未知'} / 帧 ${p.sequence||'—'} / ${finite(p.age_sec)?p.age_sec.toFixed(2)+' s':'年龄未知'}`);
    const tags=p.tag_fresh?Array.from(p.tags||[]):[];
    const target=tags.some(t=>Array.isArray(p.target_ids)&&(p.target_ids.length===0||p.target_ids.includes(t.id)));
    set('pv-tag',tags.length?'ID '+tags.map(t=>t.id).join(', ')+(target?'':' · 非目标'):'未解码 / 已过期',target?'pv-ok':'pv-warn');
    set('pv-tag-detail',`${p.tag_camera||'来源未知'}，源帧 ${p.tag_sequence||'—'}。${tags.length?'当前单帧解码结果；'+(p.tag_confirmed?'任务曾满足稳定触发。':'尚不等于稳定触发。'):'人眼看见标签图案不等于算法已读出编码。'}`);
    const grid=p.frame_fresh&&p.geometry_valid&&p.association_unique;
    set('pv-grid-state',grid?'唯一关联':!p.local_enabled?'局部管线关闭':p.hypotheses>1?'关联有歧义':'未建立',grid?'pv-ok':'pv-warn');
    set('pv-grid-detail',`节点 ${p.nodes??0}，关联假设 ${p.hypotheses??0}。${explain(p.reason)}`);
    const pose=p.frame_fresh&&p.pose_valid;
    set('pv-pose',pose?(demo?'模拟解算有效':'定位证据有效'):'不可用于定位',pose?'pv-ok':'pv-bad');
    set('pv-pose-detail',pose?`位姿源帧 ${p.pose_sequence||p.sequence}，${p.pose_model||p.model} / ${p.pose_source==='short_odometry'?'短时里程计':'地标'}。行 ${p.row?.toFixed(2)}、列 ${p.col?.toFixed(2)}（0 基准）；重投影 ${finite(p.reprojection_px)?p.reprojection_px.toFixed(2):'—'} px。仍须实际安全门许可。`:`${explain(p.pose_reason)}；位姿年龄 ${finite(p.pose_age_sec)?p.pose_age_sec.toFixed(2)+' s':'未知'}。`);
    set('pv-cones',p.frame_fresh?`颜色候选 ${p.red_components??0}`:'证据过期','pv-warn');
    const classifiers=p.colour_cones>=0?`方案 ${p.cone_mode||'未知'}：颜色 ${p.colour_cones}、形状 ${p.shape_cones} 个。`:'当前帧未完成格内分类。';
    set('pv-cone-detail',classifiers+explain(p.cone_reason));
    const cells=Array.isArray(p.cells)?p.cells:[];let known=0,cones=0,conflicts=0;q('pv-map').replaceChildren();
    for(let i=0;i<9;i++){
      const c=cells.find(c=>c.row===Math.floor(i/3)&&c.col===i%3)||{type:'unknown'};
      const type=c.conflict?'unknown':names[c.type]?c.type:'unknown';
      if(type!=='unknown')known++;if(type==='circle'||type==='square')cones++;if(c.conflict)conflicts++;
      const cell=document.createElement('div');cell.className='pv-cell pv-'+type+(c.visible_now&&p.frame_fresh?' pv-visible':'')+(c.conflict?' pv-conflict':'');
      const coord=document.createElement('small'),label=document.createElement('b'),detail=document.createElement('span');
      coord.textContent=`(${Math.floor(i/3)+1}, ${i%3+1})`;label.textContent=c.conflict?'证据冲突':names[type];
      const votes=Array.isArray(c.votes)?c.votes.reduce((a,b)=>a+b,0):0;
      detail.textContent=c.visible_now&&p.frame_fresh?'当前整格可见':c.observed?`累计证据 ${votes} 票`:'未确认';cell.append(coord,label,detail);q('pv-map').append(cell);
    }
    set('pv-map-state',p.map_complete?'地图已冻结':`已确认 ${known}/9 格，锥 ${cones}/4，冲突 ${conflicts}`,p.map_complete?'pv-ok':'pv-warn');
    set('pv-map-detail',`${p.frame_fresh?'累计地图':'历史地图：当前感知已过期'}。描边只表示当前整格可见；空格须观测确认，不由目标未检出推断。地图冻结不代表遍历已经完成。`);
  }
  function demoSnapshot(){
    const mode=q('pv-demo').value,stale=mode==='stale',ambiguous=mode==='ambiguous',undecoded=mode==='undecoded';
    const canvas=document.createElement('canvas');canvas.width=640;canvas.height=480;const c=canvas.getContext('2d');
    c.fillStyle='#215964';c.fillRect(0,0,640,480);c.strokeStyle='#d2dad3';c.lineWidth=9;
    for(const x of [80,330,580]){c.beginPath();c.moveTo(x,0);c.lineTo(x,480);c.stroke();}for(const y of [110,360]){c.beginPath();c.moveTo(0,y);c.lineTo(640,y);c.stroke();}
    c.strokeStyle='#e8c45a';c.beginPath();c.moveTo(0,360);c.lineTo(640,360);c.stroke();
    c.fillStyle='#e5e3d2';c.fillRect(360,135,104,104);c.fillStyle='#101a22';c.fillRect(370,145,84,84);c.fillStyle='#e5e3d2';for(const [x,y] of [[382,157],[418,157],[394,181],[430,193]])c.fillRect(x,y,12,12);
    if(!undecoded){c.strokeStyle='#78d89c';c.lineWidth=3;c.strokeRect(370,145,84,84);c.fillStyle='#78d89c';c.font='18px sans-serif';c.fillText('ID 18 / DEMO',370,129);}
    c.strokeStyle='#efb552';c.strokeRect(130,185,62,115);c.fillStyle='#efb552';c.fillText('candidate',124,174);
    c.fillStyle='#fff';c.font='16px sans-serif';c.fillText('DEMO / PARTIAL GRID / NOT A REAL CAMERA',12,465);
    const cells=Array.from({length:9},(_,i)=>({row:Math.floor(i/3),col:i%3,type:ambiguous?'unknown':i===6?'circle':i===7?'square':i===4?'empty':'unknown',observed:!ambiguous&&[4,6,7].includes(i),visible_now:!stale&&[4,6,7].includes(i),votes:[i===4?3:0,i===6?3:0,i===7?3:0]}));
    return {sequence:128,epoch:2,age_sec:stale?4:.12,stamp:100,frame_fresh:!stale,camera:'front',model:'underwater',phase:'BUILD_MAP',local_enabled:true,sensor_ready:!stale,geometry_valid:!ambiguous&&!stale,association_unique:!ambiguous&&!stale,nodes:4,hypotheses:ambiguous?3:1,reason:ambiguous?'temporal association ambiguous or discontinuous':'模拟：局部节点关联',pose_valid:!ambiguous&&!stale,pose_age_sec:stale?4:.12,pose_reason:ambiguous?'关联不唯一':stale?'感知断流':'模拟解算',row:1.42,col:1.65,reprojection_px:.8,red_components:2,colour_cones:1,shape_cones:0,cone_mode:'colour',cone_reason:'模拟：主体分类仍需证据',target_ids:[18],tag_confirmed:true,tag_camera:'front',tag_sequence:128,tag_fresh:!undecoded&&!stale,tags:undecoded?[]:[{id:18}],cells,map_complete:false,image_url:canvas.toDataURL('image/jpeg')};
  }
  q('pv-demo').onchange=()=>{if(config?.demo)render(demoSnapshot(),true);};
  q('pv-export').onclick=()=>{if(!last)return;const data={source:config?.demo?'VIRTUAL_DEMO':'RUNTIME',received_at:new Date(received).toISOString(),perception:last};const a=document.createElement('a'),url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));a.href=url;a.download='auv-perception-snapshot.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
  async function tick(){
    if(!config){setTimeout(tick,250);return;}
    q('pv-demo').parentElement.hidden=!config.demo;
    if(config.demo){render(demoSnapshot(),true);return;}
    try{
      const r=await fetch('/api/perception',{cache:'no-store',signal:AbortSignal.timeout(1500)});
      if(!r.ok)throw Error(r.status===404?'当前服务未提供感知接口，需要更新 Runtime':'感知接口 HTTP '+r.status);
      const p=await r.json();if(p.schema!=='auv.perception.v1')throw Error('感知接口版本不匹配');render(p);
    }catch(e){
      if(last){const priorReceived=received;render({...last,frame_fresh:false,tag_fresh:false,pose_valid:false,image_url:''});received=priorReceived;}
      set('pv-source','感知连接中断','pv-bad');set('pv-empty',e.message);q('pv-image').hidden=true;q('pv-empty').hidden=false;
    }setTimeout(tick,1000);
  }tick();
})();
