'use strict';
const $=id=>document.getElementById(id),types={unknown:'未知',empty:'空格',circle:'圆锥',square:'方锥',obstacle:'障碍'};
let snapshot=null,rawLog='',logSource='',logRunId='';
for(let i=0;i<9;i++){
 const cell=document.createElement('div');cell.className='cell';
 const label=document.createElement('label');label.textContent=`第 ${Math.floor(i/3)+1} 行 · 第 ${i%3+1} 列`;
 const select=document.createElement('select');select.dataset.index=String(i);
 for(const [value,name] of Object.entries(types)){const option=document.createElement('option');option.value=value;option.textContent=name;select.append(option)}
 select.addEventListener('change',checkMap);label.append(select);cell.append(label);$('grid').append(cell);
}
function cells(){return [...$('grid').querySelectorAll('select')].map((select,i)=>({row:Math.floor(i/3),col:i%3,type:select.value,source:'operator_manual'}))}
function checkMap(){const a=cells(),cones=a.filter(c=>c.type==='circle'||c.type==='square').length,unknown=a.filter(c=>c.type==='unknown').length;
 $('map-check').textContent=`已填写交通锥 ${cones} 个；未知格 ${unknown} 个。${cones!==4?'若赛场确有四锥，请核对填写。':''}`;
}
checkMap();
function annotation(){const manual=cells(),measured=Array.isArray(snapshot?.cells)?snapshot.cells:[];
 const disagreements=manual.filter(c=>c.type!=='unknown').flatMap(c=>{const prior=measured.find(p=>p.row===c.row&&p.col===c.col);return prior&&prior.type&&prior.type!==c.type?[{row:c.row,col:c.col,observed_type:prior.type,manual_type:c.type}]:[]});
 return {schema:'auv.manual_grid_annotation.v1',created_at:new Date().toISOString(),source:'operator_manual',run_id:logRunId||snapshot?.run_id||null,yellow_edge_field_direction:$('yellow').value,grid_convention:'canonical_yellow_edge_down',submission_grid:manual,machine_cells:measured,disagreements,note:$('note').value.trim(),runtime_snapshot:snapshot&&(!logRunId||snapshot.run_id===logRunId)?{phase:snapshot.phase,armed:snapshot.armed,motion_enabled:snapshot.motion_enabled,run_id:snapshot.run_id,recording_directory:snapshot.recording_directory}:null,log_source:logSource||null,raw_log_bytes:new TextEncoder().encode(rawLog).length};}
function download(name,content,type){const url=URL.createObjectURL(new Blob([content],{type})),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)}
function ingest(text,source){const lines=text.split(/\r?\n/).filter(Boolean);let run='';for(const line of lines){try{const item=JSON.parse(line);if(item.run_id){if(run&&run!==item.run_id)throw Error('multiple run IDs');run=item.run_id}}catch(e){if(e.message==='multiple run IDs')throw e;throw Error('日志包含无效 JSON 行')}}
 if(snapshot?.run_id&&run&&snapshot.run_id!==run)throw Error(`日志 run_id ${run} 与当前运行 ${snapshot.run_id} 不一致`);
 rawLog=lines.join('\n')+(lines.length?'\n':'');logSource=source;logRunId=run;
 $('log-info').textContent=`${source} · ${lines.length} 条 · run_id ${run||'未提供'}`;$('export-log').disabled=!lines.length;
}
$('fetch-log').onclick=async()=>{try{const response=await fetch('/api/run/events.ndjson',{cache:'no-store',signal:AbortSignal.timeout(5000)});if(!response.ok)throw Error(`HTTP ${response.status}`);ingest(await response.text(),'当前机载 events.ndjson（当前段）')}catch(e){$('log-info').textContent=`读取失败：${e.message}`}};
$('file').onchange=async e=>{const file=e.target.files?.[0];if(!file)return;try{if(file.size>25_000_000)throw Error('单文件上限 25 MB');ingest(await file.text(),file.name)}catch(error){$('log-info').textContent=`导入失败：${error.message}`}};
$('export-map').onclick=()=>download(`manual-grid-${snapshot?.run_id||Date.now()}.json`,JSON.stringify(annotation(),null,2),'application/json');
$('export-log').onclick=()=>{if(!rawLog)return;const a=annotation();const record={run_id:a.run_id,timestamp_unix_ms:Date.now(),event:'MANUAL_GRID_ANNOTATION',source:'operator_manual',detail:JSON.stringify({yellow_edge_field_direction:a.yellow_edge_field_direction,grid_convention:a.grid_convention,submission_grid:a.submission_grid,disagreements:a.disagreements,note:a.note,log_source:a.log_source,raw_log_bytes:a.raw_log_bytes})};download(`annotated-events-${a.run_id||Date.now()}.ndjson`,rawLog+JSON.stringify(record)+'\n','application/x-ndjson')};
async function poll(){try{const r=await fetch('/api/status',{cache:'no-store',signal:AbortSignal.timeout(2000)});if(!r.ok)throw Error(`HTTP ${r.status}`);const s=await r.json();snapshot=s;
 $('live').textContent=`run_id ${s.run_id}\n阶段 ${s.phase} · DISARM ${s.armed===false?'是':'否'} · 运动许可 ${s.motion_enabled?'开':'关'}\n搜索航点 ${s.tag_docking?.search_enabled?'已配置':'未配置'} · 终点航点 ${s.tag_docking?.finish_waypoints??0} 个\n前视 ID18 ${s.tag_docking?.front_detected?'检测到':'未检测'} · 下视 ID18 ${s.tag_docking?.down_detected?'检测到':'未检测'}\n黄边线索 ${s.tag_docking?.yellow_edge_candidate?'已连续观察到候选（方位待人工确认）':'未确认'}\n记录目录 ${s.recording_directory||'未知'}`;
 $('gate').textContent=(s.tag_docking?.startup_blockers||[]).length?`启动阻断：${s.tag_docking.startup_blockers.join('、')}`:'启动门状态以机载配置为准';
 }catch(e){$('live').textContent=`状态读取失败：${e.message}`}}
poll();setInterval(poll,1000);
