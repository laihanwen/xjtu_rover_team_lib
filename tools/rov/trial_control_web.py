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

HTML = '''<!doctype html><html lang="zh"><meta charset="utf-8"><title>ROV 低速手柄试用</title>
<style>body{font:18px system-ui;max-width:960px;margin:30px auto;background:#eef2f6;color:#172b3c}section{background:white;padding:22px;margin:15px 0;border-radius:12px}button{padding:15px 24px;font:inherit;border:0;border-radius:8px;cursor:pointer}#stop{background:#b82435;color:white}#arm{background:#12665d;color:white}button:disabled{opacity:.4;cursor:default}pre{white-space:pre-wrap;font:15px monospace}.axis{display:grid;grid-template-columns:160px 1fr 70px;gap:12px;margin:14px 0}meter{width:100%;height:25px}.warn{color:#8d4210}</style>
<h1>ROV 低速手柄试用</h1><section>
<p>连续水下模式：横滚P=0.75，俯仰PD=0.75/0.15、I=0；右杆左右回中自动锁航向，转向后回中锁新航向，2°开始纠正、低于1°停止；输出渐变200 μs/秒，STOP和失联立即回中。定深和两路舵机关闭。中位1492 μs、限幅±100；无固定运行时限，左肩许可及停机保护仍有效。</p>
<p>将四根摇杆回中，左肩开关拨到允许位（原始 axis5 ≥ +0.5），再点击 ARM。左肩拨回其余位置、关闭页面、手柄掉线或网络失联都会停机，恢复连接不会自动 ARM。</p>
<p class="warn">目前没有漏水传感器。实际急停以你已确认的硬件急停装置为准；空转只作短时核对。</p>
<button id="arm" onclick="act('arm')" disabled>ARM · 低速试用</button>
<button id="stop" onclick="act('stop')">停止 / DISARM（Esc）</button>
<h2 id="mode">正在连接</h2><p id="detail"></p><p id="device"></p></section>
<section><h2>CSI 摄像头回传</h2><a href="http://192.168.137.150:8080/" target="_blank" rel="noopener">打开双摄像头实时画面（CSI 为前视画面）</a></section>
<section><p>八路起转补偿±48 μs（岸上实测），实际总限幅±100 μs；水中需复核。标定脉冲已禁用；混控优先保留调平纠正，升降/侧移方向已反转。</p></section><section><h2>手柄输入</h2><div id="axes"></div><p id="gate"></p></section>
<section><h2>STM32 实际遥测</h2><pre id="telemetry"></pre></section>
<script>async function pulse(){const r=await fetch('/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'pulse',motor:Number(document.querySelector('#motor').value),offset:Number(document.querySelector('#offset').value)})});if(!r.ok)alert(await r.text())}async function act(action){const r=await fetch('/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})});if(!r.ok)alert(await r.text())}
document.addEventListener('keydown',e=>{if(e.key==='Escape')act('stop')});
const labels=['右杆左右 · 偏航','右杆前后 · 前后','左杆前后 · 升沉','左杆左右 · 横移'];
labels.forEach((l,i)=>{const d=document.createElement('div');d.className='axis';const label=document.createElement('span');label.textContent=l;const m=document.createElement('meter');m.min=-1;m.max=1;m.value=0;m.id='a'+i;const v=document.createElement('span');v.id='v'+i;d.append(label,m,v);document.querySelector('#axes').append(d)});
async function update(){try{const s=await(await fetch('/state',{cache:'no-store'})).json();document.querySelector('#arm').disabled=!s.can_arm;document.querySelector('#mode').textContent=s.error||(!s.connected?'未连接树莓派':s.telemetry.armed?'已ARM · 低速手动':'未ARM · 输出中位');document.querySelector('#detail').textContent=s.detail||'';document.querySelector('#device').textContent=s.device||'';document.querySelector('#gate').textContent='左肩许可：'+(s.deadman?'允许':'停止')+' / 四轴回中：'+(s.centered?'是':'否');(s.axes||[]).slice(0,4).forEach((x,i)=>{document.querySelector('#a'+i).value=x;document.querySelector('#v'+i).textContent=x.toFixed(3)});document.querySelector('#telemetry').textContent=JSON.stringify({遥测年龄秒:s.telemetry_age,俯仰度:s.telemetry.pitch_deg,横滚度:s.telemetry.roll_deg,偏航度:s.telemetry.yaw_deg,深度米:s.telemetry.depth_m,姿态调平:s.attitude_hold,航向保持功能:s.heading_hold_enabled,深度保持:false,状态错误:s.telemetry.error_flags,八推限幅内比例:s.telemetry.outputs,串口CRC错误:s.uart_crc_errors},null,2)}catch(e){document.querySelector('#arm').disabled=true;document.querySelector('#mode').textContent='界面连接中断'}}setInterval(update,150);update();</script></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.137.150')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument('--web-port', type=int, default=8767)
    parser.add_argument('--mapping-config', default=str(Path(__file__).with_name('radiomaster-pocket.json')))
    args = parser.parse_args()
    os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
    mapping = load_mapping(args.mapping_config)
    controller = RemoteControl(-.5,-.1,mapping)
    lock = threading.Lock()
    state = {'connected':False,'telemetry':{},'error':'','detail':'等待手柄',
             'axes':[],'deadman':False,'centered':False,'can_arm':False}
    operator = {'last_poll':0.0,'action':None}
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):
            pass
        def reply(self,data,code=200,html=False):
            body = data.encode() if html else json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(code);self.send_header('Content-Type','text/html; charset=utf-8' if html else 'application/json; charset=utf-8')
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def do_GET(self):
            if self.path=='/':return self.reply(HTML,html=True)
            if self.path!='/state':return self.reply({'error':'not found'},404)
            with lock:
                operator['last_poll']=time.monotonic()
                snapshot=dict(state)
            self.reply(snapshot)
        def do_POST(self):
            if self.path!='/action' or self.headers.get('Origin')!=f'http://127.0.0.1:{args.web_port}':
                return self.reply({'error':'invalid origin'},403)
            try:
                size=int(self.headers.get('Content-Length',0))
                if not 0<size<=64:raise ValueError('size')
                request=json.loads(self.rfile.read(size)); action=request['action']
                if action not in ('arm','stop'):raise ValueError('action')
                with lock:
                    if action=='arm' and not state['can_arm']:
                        return self.reply({'error':'回中、左肩许可及新鲜IMU遥测后才能ARM'},409)
                    if action=='pulse':
                        motor,offset=request.get('motor'),request.get('offset')
                        if type(motor) is not int or type(offset) is not int or not 0<=motor<8 or not -75<=offset<=75:raise ValueError('pulse')
                        if not state.get('deadman') or not state.get('centered') or not state['telemetry'].get('armed'):return self.reply({'error':'先回中、左肩许可并显式ARM'},409)
                        operator['action']={'action':'pulse','motor':motor,'offset':offset}
                    else:operator['action']=action
                self.reply({'queued':action})
            except (ValueError,KeyError,TypeError):self.reply({'error':'bad request'},400)
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
                    state['can_arm']=bool(deadman and centered(frame) and now-last_reply<.2 and state.get('telemetry_age',999)<.3 and telemetry.get('roll_deg') is not None and not telemetry.get('armed'))
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
        server.shutdown()


if __name__=='__main__':
    main()
