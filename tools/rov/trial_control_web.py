"""Local manual ROV console: physical deadman plus explicit operator ARM."""
import argparse
import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from Re_control import RemoteControl, load_mapping
from trial_protocol import manual_frame, centered
from video_recorder import DatasetRecorder
from telemetry_log import TelemetryLog

HTML = '''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>ROV 遥控驾驶台</title>
<style>*{box-sizing:border-box}body{margin:0;padding:36px clamp(18px,4vw,64px);max-width:1440px;margin-inline:auto;background:#0b1421;color:#e6eef8;font:16px/1.6 "Segoe UI","Microsoft YaHei",sans-serif}h1{font-size:clamp(26px,4vw,42px);letter-spacing:-1px;margin:6px 0 12px}h2{font-size:18px;margin:0 0 14px}p{color:#a8bacd}a{color:#61d9c4;text-underline-offset:4px}section,.card{background:#121f30;border:1px solid #26364a;padding:24px;border-radius:18px;margin:20px 0;box-shadow:0 8px 28px #0002}button{padding:13px 22px;font-family:inherit;font-size:15px;font-weight:600;border:1px solid #35516b;border-radius:10px;background:#20354a;color:#e6eef8;cursor:pointer;min-height:48px}button:hover{filter:brightness(1.15)}button:focus-visible,a:focus-visible{outline:3px solid #61d9c4;outline-offset:4px}button:disabled{opacity:.4;cursor:not-allowed}#arm{background:#217869;border-color:#40bba3}#stop{background:#b73749;border-color:#d96673;margin-left:8px}.warn,.bad{color:#ffc594}.axis{display:grid;grid-template-columns:180px 1fr 75px;gap:14px;align-items:center;margin:18px 0}meter{width:100%;height:24px;accent-color:#61d9c4}pre{white-space:pre-wrap;overflow-wrap:anywhere;font:13px/1.8 Consolas,monospace;color:#a8bacd}.eyebrow{font-size:12px;letter-spacing:3px;color:#61d9c4;font-weight:700}.subtitle{margin-bottom:26px}.stats{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}.stat{background:#152438;border:1px solid #2a3e54;border-radius:14px;padding:18px}.stat span{display:block;color:#91a8bd;font-size:12px}.stat strong{display:block;font-size:26px;line-height:1.4;color:#f0f7ff;margin-top:8px}.layout{display:grid;grid-template-columns:1.7fr 1fr;gap:22px;align-items:start}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.cell{min-height:86px;padding:12px;background:#182b40;border:1px solid #35516b;border-radius:10px;font-size:13px}.cell.current{border:2px solid #61d9c4}.cell.next{background:#254764}.small{font-size:13px;color:#a8bacd}video{width:100%;aspect-ratio:16/9;object-fit:contain;background:#050b12;border-radius:12px}details summary{cursor:pointer;color:#c7d8e9;padding:8px 0}#mode{margin-top:20px}#detail{min-height:24px}@media(max-width:760px){body{padding:20px 16px}.stats{grid-template-columns:repeat(2,1fr)}.layout{grid-template-columns:1fr}.axis{grid-template-columns:1fr 60px}.axis span:first-child{grid-column:1/-1}section,.card{padding:18px}#stop{margin:8px 0}}</style>
<div class="eyebrow">UNDERWATER CONTROL / ROV</div><h1>遥控驾驶台</h1><p class="subtitle">手柄控制 · 实时姿态 · 双摄回传</p><div class="stats"><div class="stat"><span>俯仰 PITCH</span><strong id="pitch-value">—</strong></div><div class="stat"><span>横滚 ROLL</span><strong id="roll-value">—</strong></div><div class="stat"><span>航向 YAW</span><strong id="yaw-value">—</strong></div><div class="stat"><span>遥测延迟</span><strong id="age-value">—</strong></div></div><section><button id="arm" onclick="act('arm')" disabled>ARM · 低速试用</button>
<button id="stop" onclick="act('stop')">停止 / DISARM（Esc）</button>

<p><label><input id="shore" type="checkbox"> 已在岸上可靠放平，保持未 ARM</label> <button id="level" onclick="act('level')">手动校准水平</button></p><p id="level-state">水平基准待确认</p><details><summary>控制参数与工作模式</summary><p>连续水下模式：横滚P=7.5，俯仰PD=7.5/1.5、I=0；右杆左右回中自动锁航向，转向后回中锁新航向，2°开始纠正、低于1°停止；输出渐变200 μs/秒，STOP和失联立即回中。定深和两路舵机关闭。中位1492 μs、限幅±100；无固定运行时限，左肩许可及停机保护仍有效。</p></details>
<p>将四根摇杆回中，左肩开关拨到允许位，再点击 ARM。左肩拨回其余位置、关闭页面、手柄掉线或网络失联都会停机，恢复连接不会自动 ARM。</p>
<p class="warn">实际急停以你已确认的硬件急停装置为准；空转只作短时核对。</p>
<h2 id="mode" aria-live="polite">正在连接</h2><p id="detail"></p><p id="device"></p></section>
<section><h2>YOLO 数据采集</h2><p>CSI 下视与 USB 前视分别录制到 PC；视频不叠加识别标注，每分钟分段保存。录制独立于 ARM 和遥控许可。</p><button id="record-start" onclick="recordAction('start')">开始双摄录制</button> <button id="record-stop" onclick="recordAction('stop')">结束并保存</button><p id="record-state" aria-live="polite">录制未开始</p><details><summary>保存目录与采集详情</summary><pre id="record-detail"></pre></details></section><section><h2>双摄像头回传</h2><a href="http://192.168.137.150:8080/" target="_blank" rel="noopener">打开双摄像头实时画面（左侧 CSI 下视，右侧 USB 前视）</a></section>
<section><p>八路起转补偿±48 μs（岸上实测），实际总限幅±100 μs；水中需复核。标定脉冲已禁用；混控优先保留调平纠正，升降/侧移方向已反转。</p></section><section><h2>手柄输入</h2><div id="axes"></div><p id="gate"></p></section>
<section><h2>IMU / PID 日志</h2><p id="log-state">日志启动中</p><pre id="log-detail"></pre></section><section><h2>通信与设备状态</h2><details><summary>查看完整 STM32 遥测</summary><pre id="telemetry"></pre></details></section>
<script>async function pulse(){const r=await fetch('/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'pulse',motor:Number(document.querySelector('#motor').value),offset:Number(document.querySelector('#offset').value)})});if(!r.ok)alert(await r.text())}async function act(action){const r=await fetch('/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,...(action==='level'?{shore_confirmed:document.querySelector('#shore').checked}:{})})});if(!r.ok)alert(await r.text())}
async function recordAction(action){try{const r=await fetch('/recording',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})});const data=await r.json();if(!r.ok)document.getElementById('record-state').textContent=data.error||'录制操作失败';else await recordingState()}catch(e){document.getElementById('record-state').textContent='录制连接失败'}}
async function recordingState(){try{const r=await fetch('/recording',{cache:'no-store'});if(!r.ok)throw Error('状态不可用');const s=await r.json();document.getElementById('record-start').disabled=s.active;document.getElementById('record-stop').disabled=!s.active;document.getElementById('record-state').textContent=s.error||((s.active?'正在录制':'录制已停止')+' · 下视 '+(s.cameras.down?.frames||0)+' 帧 · 前视 '+(s.cameras.front?.frames||0)+' 帧 · 实收 下视 '+(s.cameras.down?.received_fps||0)+' / 前视 '+(s.cameras.front?.received_fps||0)+' FPS');document.getElementById('record-detail').textContent=JSON.stringify(s,null,2)}catch(e){document.getElementById('record-state').textContent='录制状态不可用'}}setInterval(recordingState,1000);recordingState();
async function logState(){try{const s=await(await fetch('/logs',{cache:'no-store'})).json();document.getElementById('log-state').textContent=s.error?'日志错误：'+s.error:'每 '+s.interval_s+' 秒记录一次 · 已保存 '+s.records+' 条';document.getElementById('log-detail').textContent=s.directory}catch(e){document.getElementById('log-state').textContent='日志状态不可用'}}setInterval(logState,1000);logState();
document.addEventListener('keydown' ,e=>{if(e.key==='Escape')act('stop')});
const labels=['右杆左右 · 偏航','右杆前后 · 前后','左杆前后 · 升沉','左杆左右 · 横移'];
labels.forEach((l,i)=>{const d=document.createElement('div');d.className='axis';const label=document.createElement('span');label.textContent=l;const m=document.createElement('meter');m.min=-1;m.max=1;m.value=0;m.id='a'+i;const v=document.createElement('span');v.id='v'+i;d.append(label,m,v);document.querySelector('#axes').append(d)});
async function update(){try{const s=await(await fetch('/state',{cache:'no-store'})).json();document.querySelector('#arm').disabled=!s.can_arm;document.querySelector('#level').disabled=!s.connected||s.telemetry.armed||s.level_pending;document.querySelector('#level-state').textContent=s.level_pending?'水平校准中，请保持静止':s.telemetry.level_calibrated?'水平已校准（本次开机）':'水平未校准：岸上放平后手动校准，重启需重做';document.querySelector('#mode').textContent=s.error||(!s.connected?'未连接树莓派':s.telemetry.armed?'已ARM · 低速手动':'未ARM · 输出中位');document.querySelector('#detail').textContent=s.detail||'';document.querySelector('#device').textContent=s.device||'';document.querySelector('#gate').textContent='左肩许可：'+(s.deadman?'允许':'停止')+' / 四轴回中：'+(s.centered?'是':'否');(s.axes||[]).slice(0,4).forEach((x,i)=>{document.querySelector('#a'+i).value=x;document.querySelector('#v'+i).textContent=x.toFixed(3)});for(const [id,key] of [['pitch-value','pitch_deg'],['roll-value','roll_deg'],['yaw-value','yaw_deg']])document.getElementById(id).textContent=Number.isFinite(s.telemetry[key])?s.telemetry[key].toFixed(1)+'°':'—';document.getElementById('age-value').textContent=Number.isFinite(s.telemetry_age)?Math.round(s.telemetry_age*1000)+' ms':'—';document.querySelector('#telemetry').textContent=JSON.stringify({遥测年龄秒:s.telemetry_age,俯仰度:s.telemetry.pitch_deg,横滚度:s.telemetry.roll_deg,偏航度:s.telemetry.yaw_deg,深度米:s.telemetry.depth_m,姿态调平:s.attitude_hold,航向保持功能:s.heading_hold_enabled,深度保持:false,状态错误:s.telemetry.error_flags,八推限幅内比例:s.telemetry.outputs,串口CRC错误:s.uart_crc_errors},null,2)}catch(e){document.querySelector('#arm').disabled=true;document.querySelector('#level').disabled=true;document.querySelector('#level-state').textContent='水平状态不可用';document.querySelector('#mode').textContent='界面连接中断';for(const id of ['pitch-value','roll-value','yaw-value','age-value'])document.getElementById(id).textContent='—';for(let i=0;i<4;i++){document.getElementById('a'+i).value=0;document.getElementById('v'+i).textContent='—'}}}setInterval(update,150);update();</script></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.137.150')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument('--web-port', type=int, default=8767)
    parser.add_argument('--mapping-config', default=str(Path(__file__).with_name('radiomaster-pocket.json')))
    parser.add_argument('--camera-port', type=int, default=8080)
    parser.add_argument('--record-dir', default=str(Path(__file__).resolve().parents[2] / 'data' / 'rov-recordings'))
    parser.add_argument('--log-dir', default=str(Path(__file__).resolve().parents[2] / 'logs' / 'rov'))
    args = parser.parse_args()
    recorder = DatasetRecorder(f'http://{args.host}:{args.camera_port}',args.record_dir)
    os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
    mapping = load_mapping(args.mapping_config)
    controller = RemoteControl(-.5,-.1,mapping)
    lock = threading.Lock()
    state = {'connected':False,'telemetry':{},'error':'','detail':'等待手柄',
             'axes':[],'deadman':False,'centered':False,'can_arm':False}
    operator = {'last_poll':0.0,'action':None}
    def log_snapshot():
        with lock:
            result = dict(state)
            # Reply ages are relative to arrival; account for a stalled TCP loop.
            elapsed = time.monotonic()-state.get('_received_at',time.monotonic())
            for name in ('telemetry_age','pid_age'):
                if result.get(name) is not None: result[name] += elapsed
            return result
    telemetry_log = TelemetryLog(args.log_dir,log_snapshot)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):
            pass
        def reply(self,data,code=200,html=False):
            body = data.encode() if html else json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(code);self.send_header('Content-Type','text/html; charset=utf-8' if html else 'application/json; charset=utf-8')
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def do_GET(self):
            if self.path=='/':return self.reply(HTML,html=True)
            if self.path=='/recording':return self.reply(recorder.status())
            if self.path=='/logs':return self.reply(telemetry_log.status())
            if self.path!='/state':return self.reply({'error':'not found'},404)
            with lock:
                operator['last_poll']=time.monotonic()
                snapshot=dict(state)
            self.reply(snapshot)
        def do_POST(self):
            if self.path not in ('/action','/recording') or self.headers.get('Origin')!=f'http://127.0.0.1:{args.web_port}':
                return self.reply({'error':'invalid origin'},403)
            try:
                size=int(self.headers.get('Content-Length',0))
                if not 0<size<=64:raise ValueError('size')
                request=json.loads(self.rfile.read(size)); action=request['action']
                if self.path=='/recording':
                    if action not in ('start','stop'):raise ValueError('action')
                    return self.reply(recorder.start() if action=='start' else recorder.stop())
                if action not in ('arm','stop','level'):raise ValueError('action')
                with lock:
                    if action=='arm' and not state['can_arm']:
                        return self.reply({'error':'回中、左肩许可及新鲜IMU遥测后才能ARM'},409)
                    if action=='level':
                        if request.get('shore_confirmed') is not True or not state['connected'] or not state.get('centered') or state['telemetry'].get('armed') or state.get('telemetry_age',999)>.3:
                            return self.reply({'error':'请在岸上放平、未ARM、摇杆回中并勾选确认'},409)
                        operator['action']={'action':'level','shore_confirmed':True}
                    elif action=='pulse':
                        motor,offset=request.get('motor'),request.get('offset')
                        if type(motor) is not int or type(offset) is not int or not 0<=motor<8 or not -75<=offset<=75:raise ValueError('pulse')
                        if not state.get('deadman') or not state.get('centered') or not state['telemetry'].get('armed'):return self.reply({'error':'先回中、左肩许可并显式ARM'},409)
                        operator['action']={'action':'pulse','motor':motor,'offset':offset}
                    else:operator['action']=action
                self.reply({'queued':action})
            except (ValueError,KeyError,TypeError,OSError) as error:self.reply({'error':str(error)},400)
    server=ThreadingHTTPServer(('127.0.0.1',args.web_port),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    print(f'ROV console http://127.0.0.1:{args.web_port}/ ; no automatic ARM',flush=True)
    conn=None;buffer=bytearray();lease=None;last_reply=0.0;sequence=0
    try:
        while True:
            now=time.monotonic()
            try:
                frame=controller.read()
                if frame is None:raise ValueError('手柄未连接')
                frame=manual_frame(frame)
                axes=[controller.device.get_axis(i) for i in mapping['motion_axes']]
                physical_deadman=controller.device.get_axis(mapping['servo_select_axis'])>=.5
                with lock:
                    live=now-operator['last_poll']<=.8
                    action=operator['action'];operator['action']=None
                deadman=physical_deadman and live
                if conn is None:
                    conn=socket.create_connection((args.host,args.port),timeout=.2)
                    conn.settimeout(.005);conn.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                    buffer.clear();lease=None;last_reply=0.0
                try:data=conn.recv(4096)
                except socket.timeout:data=None
                if data==b'':raise OSError('树莓派断开连接')
                if data:
                    buffer.extend(data)
                    if len(buffer)>8192:raise ValueError('reply overflow')
                    while b'\n' in buffer:
                        line,_,remaining=buffer.partition(b'\n');buffer=bytearray(remaining)
                        reply=json.loads(line);lease=reply['lease'];last_reply=now
                        with lock:
                            state.update({k:v for k,v in reply.items() if k!='lease'})
                            state['_received_at']=time.monotonic()
                if lease is not None and now-last_reply<=.2:
                    sequence+=1
                    message={'lease':lease,'sequence':sequence,'frame':list(frame),'deadman':deadman}
                    if isinstance(action,dict):message.update(action)
                    elif action is not None:message['action']=action
                    conn.sendall((json.dumps(message)+'\n').encode())
                elif last_reply and now-last_reply>.5:raise OSError('遥测网络超时')
                with lock:
                    telemetry=state['telemetry']
                    state.update(connected=True,error='',axes=axes,deadman=deadman,centered=centered(frame),device=controller.device.get_name())
                    state['can_arm']=bool(deadman and centered(frame) and now-last_reply<.2 and state.get('telemetry_age',999)<.3 and telemetry.get('roll_deg') is not None and telemetry.get('level_calibrated') and not state.get('level_pending') and not telemetry.get('armed'))
                time.sleep(.04)
            except (OSError,ValueError,KeyError,TypeError) as error:
                if conn:conn.close()
                conn=None;lease=None
                with lock:
                    state.update(connected=False,can_arm=False,deadman=False,error=str(error),
                                 telemetry={},telemetry_age=None,axes=[],centered=False)
                time.sleep(.2)
    finally:
        if conn:conn.close()
        recorder.close()
        telemetry_log.close()
        server.shutdown()


if __name__=='__main__':
    main()
