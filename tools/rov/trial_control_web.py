"""Local manual ROV console: physical deadman plus explicit operator ARM."""
import argparse
import json
import os
import socket
import threading
import time
from urllib.request import urlopen, Request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from Re_control import RemoteControl, load_mapping
from trial_protocol import manual_frame, centered
from video_recorder import DatasetRecorder
from telemetry_log import TelemetryLog

WEB_DIR = Path(__file__).with_name('web')
DASHBOARD_DIR = Path(__file__).resolve().parents[2] / 'runtime/web/dashboard'
HTML = (WEB_DIR / 'console.html').read_text(encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.137.150')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument('--web-port', type=int, default=8767)
    parser.add_argument('--mapping-config', default=str(Path(__file__).with_name('radiomaster-pocket.json')))
    parser.add_argument('--camera-port', type=int, default=8080)
    parser.add_argument('--record-dir', default=str(Path(__file__).resolve().parents[2] / 'data' / 'rov-recordings'))
    parser.add_argument('--log-dir', default=str(Path(__file__).resolve().parents[2] / 'logs' / 'rov'))
    parser.add_argument('--diagnostic-log', action='store_true', help='Enable separate 10 Hz rotating diagnostic snapshots')
    parser.add_argument('--dial-min', type=float, help='Measured SI lower endpoint')
    parser.add_argument('--dial-max', type=float, help='Measured SI upper endpoint')
    args = parser.parse_args()
    recorder = DatasetRecorder(f'http://{args.host}:{args.camera_port}',args.record_dir)
    os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
    mapping = load_mapping(args.mapping_config)
    if mapping['dial_axis'] is not None and (args.dial_min is None and mapping.get('dial_min') is None or args.dial_max is None and mapping.get('dial_max') is None):
        parser.error('SI requires measured --dial-min and --dial-max; keep dial_axis null until measured')
    controller = RemoteControl(args.dial_min if args.dial_min is not None else mapping.get('dial_min',-.5),
                               args.dial_max if args.dial_max is not None else mapping.get('dial_max',-.1),mapping)
    lock = threading.Lock()
    state = {'connected':False,'telemetry':{},'error':'','detail':'等待手柄',
             'axes':[],'deadman':False,'centered':False,'can_arm':False,
             'speed_control_available':True,'selected_speed':'low'}
    localization_state={'valid':False,'reason':'service_unavailable'}
    localization_stop=threading.Event()
    camera_base=f'http://{args.host}:{args.camera_port}'
    def localization_poll():
        while not localization_stop.is_set():
            try:
                with urlopen(camera_base+'/api/localization',timeout=.4) as response:
                    value=json.loads(response.read(8192))
                if not isinstance(value,dict):raise ValueError('invalid localization status')
                value['received_monotonic']=time.monotonic()
            except (OSError,ValueError):value={'valid':False,'reason':'service_unavailable'}
            with lock:localization_state.clear();localization_state.update(value)
            localization_stop.wait(.2)
    threading.Thread(target=localization_poll,daemon=True).start()
    operator = {'last_poll':0.0,'action':None,'speed':0}
    def log_snapshot():
        with lock:
            result = dict(state)
            result["localization"]=dict(localization_state)
            # Reply ages are relative to arrival; account for a stalled TCP loop.
            elapsed = time.monotonic()-state.get('_received_at',time.monotonic())
            for name in ('telemetry_age','pid_age'):
                if result.get(name) is not None: result[name] += elapsed
            return result
    telemetry_log = TelemetryLog(args.log_dir,log_snapshot)
    diagnostic_log = TelemetryLog(Path(args.log_dir)/'diagnostic',log_snapshot,interval=.1) if args.diagnostic_log else None
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):
            pass
        def reply(self,data,code=200,html=False):
            body = data.encode() if html else json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(code);self.send_header('Content-Type','text/html; charset=utf-8' if html else 'application/json; charset=utf-8')
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def do_GET(self):
            if self.path == '/dashboard':
                self.send_response(302);self.send_header('Location','/dashboard/');self.end_headers();return
            if self.path == '/dashboard/config':
                return self.reply({'demo':False,'source':'rov','camera_base':camera_base})
            dashboard_assets={'/dashboard/':('index.html','text/html'),
                '/dashboard/dashboard.css':('dashboard.css','text/css'),
                '/dashboard/dashboard.js':('dashboard.js','text/javascript'),
                '/dashboard/recording.js':('recording.js','text/javascript')}
            if self.path in dashboard_assets:
                name,mime=dashboard_assets[self.path];body=(DASHBOARD_DIR/name).read_bytes()
                self.send_response(200);self.send_header('Content-Type',mime+'; charset=utf-8')
                self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body);return
            if self.path=='/':return self.reply(HTML,html=True)
            if self.path=='/console-config':return self.reply({'camera_base':f'http://{args.host}:{args.camera_port}'})
            assets={'/console.css':('console.css','text/css'),'/console.js':('console.js','text/javascript')}
            if self.path in assets:
                name,mime=assets[self.path];body=(WEB_DIR/name).read_bytes()
                self.send_response(200);self.send_header('Content-Type',mime+'; charset=utf-8')
                self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body);return
            if self.path=='/localization':
                with lock:value=dict(localization_state)
                return self.reply(value)
            if self.path=='/recording':return self.reply(recorder.status())
            if self.path=='/logs':return self.reply(dict(telemetry_log.status(),diagnostic=diagnostic_log.status() if diagnostic_log else None))
            if self.path!='/state':return self.reply({'error':'not found'},404)
            with lock:
                operator['last_poll']=time.monotonic()
                snapshot=dict(state)
            self.reply(snapshot)
        def do_POST(self):
            if self.path not in ('/action','/recording','/localization') or self.headers.get('Origin')!=f'http://127.0.0.1:{args.web_port}':
                return self.reply({'error':'invalid origin'},403)
            try:
                size=int(self.headers.get('Content-Length',0))
                if not 0<size<=64:raise ValueError('size')
                request=json.loads(self.rfile.read(size)); action=request['action']
                if self.path=='/localization':
                    if action!='reset':raise ValueError('action')
                    with lock:
                        if not state.get('connected') or state.get('telemetry',{}).get('armed',True):return self.reply({'error':'Require connected DISARM state'},409)
                    try:
                        with urlopen(Request(camera_base+'/api/localization/reset',data=b'{}',method='POST'),timeout=.8) as response:value=json.loads(response.read(8192))
                        return self.reply(value)
                    except (OSError,ValueError):return self.reply({'error':'Reset rejected: require calibrated fresh input and 2 seconds stable attitude/depth'},409)
                if self.path=='/recording':
                    if action not in ('start','stop'):raise ValueError('action')
                    return self.reply(recorder.start() if action=='start' else recorder.stop())
                if action not in ('arm','stop','level','speed'):raise ValueError('action')
                with lock:
                    if action=='speed':
                        if state.get('telemetry',{}).get('armed'):
                            return self.reply({'error':'DISARM before changing speed'},409)
                        if request.get('mode') not in ('low','higher'):raise ValueError('speed mode')
                        operator['speed']=2 if request['mode']=='higher' else 0
                        state['selected_speed']=request['mode']
                        return self.reply({'selected_speed':request['mode']})
                    if action=='arm' and not state['can_arm']:
                        return self.reply({'error':'回中、独立安全许可及新鲜IMU遥测后才能ARM'},409)
                    if action=='level':
                        if request.get('shore_confirmed') is not True or not state['connected'] or not state.get('centered') or state['telemetry'].get('armed') or state.get('telemetry_age',999)>.3:
                            return self.reply({'error':'请在岸上放平、未ARM、摇杆回中并勾选确认'},409)
                        operator['action']={'action':'level','shore_confirmed':True}
                    elif action=='pulse':
                        motor,offset=request.get('motor'),request.get('offset')
                        if type(motor) is not int or type(offset) is not int or not 0<=motor<8 or not -75<=offset<=75:raise ValueError('pulse')
                        if not state.get('deadman') or not state.get('centered') or not state['telemetry'].get('armed'):return self.reply({'error':'先回中、独立安全许可并显式ARM'},409)
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
            controller_ok=False
            try:
                frame=controller.read()
                if frame is None:raise ValueError('手柄未连接')
                with lock: selected_speed=operator['speed']
                frame=bytearray(frame);frame[6]=selected_speed
                frame=manual_frame(frame)
                with lock:
                    state['camera_control'] = {
                        'selector': int(frame[7]), 'dial_byte': int(frame[5]),
                        'dial_verified': mapping['dial_axis'] is not None,
                        'requested_angle_deg': -45.0 + frame[5]*55.0/255.0 if frame[7]==1 and frame[10]==1 else None,
                        'selector_axis': mapping['servo_select_axis'], 'dial_axis': mapping['dial_axis'],
                        'main_placeholder': True, 'position_feedback': False}
                hold_switches={'SA':bool(frame[8]),'SD':bool(frame[9])}
                axes=[controller.device.get_axis(i) for i in mapping['motion_axes']]
                physical_deadman=controller.device.get_axis(mapping.get('deadman_axis',mapping['servo_select_axis']))>=.5
                controller_ok=True
                with lock:
                    state.update(controller_connected=True,axes=axes,centered=centered(frame),
                                 device=controller.device.get_name(),hold_switches=hold_switches,
                                 detail='手柄已识别；等待树莓派遥测')
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
                    state.update(connected=True,error='',axes=axes,deadman=deadman,centered=centered(frame),device=controller.device.get_name(),hold_switches=hold_switches,
                                 detail='手柄与树莓派已连接；'+('已使能' if telemetry.get('armed') else '未 ARM'))
                    state['can_arm']=bool(telemetry.get('operating_mode')!='auv' and deadman and centered(frame) and now-last_reply<.2 and state.get('telemetry_age',999)<.3 and telemetry.get('roll_deg') is not None and telemetry.get('level_calibrated') and not state.get('level_pending') and not telemetry.get('armed'))
                time.sleep(.04)
            except (OSError,ValueError,KeyError,TypeError) as error:
                if conn:conn.close()
                conn=None;lease=None
                with lock:
                    state.update(connected=False,can_arm=False,deadman=False,error=str(error),
                                 telemetry={},telemetry_age=None,controller_connected=controller_ok)
                    if controller_ok:
                        state['detail']='手柄已识别；树莓派链路未连接：'+str(error)
                    else:
                        state.update(axes=[],centered=False,device='',hold_switches={},detail=str(error))
                time.sleep(.2)
    finally:
        if conn:conn.close()
        localization_stop.set()
        recorder.close()
        telemetry_log.close()
        if diagnostic_log: diagnostic_log.close()
        server.shutdown()


if __name__=='__main__':
    main()
