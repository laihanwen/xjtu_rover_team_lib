"""Local, operator-labelled joystick check. Never opens the ROV connection."""
import argparse
import json
import os
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from Re_control import joystick_attached

STEPS = [
    ('右摇杆左右', '先向左到底，再向右到底并保持，然后点完成。'),
    ('右摇杆前后', '先向后到底，再向前到底并保持，然后点完成。'),
    ('左摇杆前后', '先向后到底，再向前到底并保持，然后点完成。'),
    ('左摇杆左右', '先向左到底，再向右到底并保持，然后点完成。'),
    ('滚轮', '先向左转到底，再向右转到底并保持；如果没有模拟滚轮，点跳过。'),
    ('左肩开关', '依次拨到前、中、后的位置，最后保持后位，然后点完成。'),
    ('右肩开关', '依次拨到前、中、后的位置，最后保持后位，然后点完成。'),
    ('SA', '按下再松开至少两次，然后点完成。'),
    ('SD', '按下再松开至少两次，然后点完成。'),
    ('SE', '按下再松开至少两次，然后点完成。'),
]
HTML = '''<!doctype html><meta charset="utf-8"><title>遥控器逐项检查</title>
<style>body{font:18px system-ui;max-width:900px;margin:35px auto;background:#f5f7fa;color:#243146}button{font:inherit;padding:12px 22px;margin:8px;border-radius:8px;cursor:pointer}pre{font:15px monospace;background:white;padding:18px;white-space:pre-wrap}h2{color:#12656b}</style>
<h1>遥控器逐项检查</h1><p>仅读取 PC 手柄。不会连接树莓派、串口或发送推进器指令。</p>
<p>每项操作前先回中／松开；点“开始该项”，按提示操作，再点“完成该项”。</p>
<h2 id="step"></h2><p id="hint"></p><p id="device"></p>
<button id="start" onclick="act('start')">开始该项</button>
<button id="finish" onclick="act('finish')">完成该项</button>
<button id="skip" onclick="act('skip')">跳过该项</button>
<pre id="live"></pre><h3>已记录结果</h3><pre id="results"></pre><a href="/report">查看完整报告</a>
<script>async function act(a){await fetch('/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:a})});await refresh()}
async function refresh(){let s=await(await fetch('/state',{cache:'no-store'})).json();document.getElementById('step').textContent=s.label;document.getElementById('hint').textContent=s.hint;document.getElementById('device').textContent=s.error||s.device;document.getElementById('live').textContent=JSON.stringify(s.live,null,2);document.getElementById('results').textContent=JSON.stringify(s.results,null,2);document.getElementById('start').disabled=s.active||s.done||!!s.error;document.getElementById('finish').disabled=!s.active||!!s.error;document.getElementById('skip').disabled=s.done}setInterval(refresh,200);refresh()</script>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    global STEPS
    parser.add_argument('--servo-only', action='store_true', help='Read-only SI / SC endpoint check')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--output', default='build/controller-guided.json')
    args = parser.parse_args()
    if args.servo_only:
        STEPS = [
            ('SI：向下端点', '仅转动 SI：先来回转动覆盖完整行程，最后保持你定义的向下极限，再点完成。不要动其他开关。'),
            ('SI：向上端点', '仅转动 SI 到你定义的向上极限，保持后点完成。'),
            ('SC：下档', '仅拨 SC：先走过下、中、上三个档位，最后保持下档（不控制舵机），点完成。'),
            ('SC：中档', '仅将 SC 拨到中档（摄像头），保持后点完成。'),
            ('SC：上档', '仅将 SC 拨到上档（机械爪占位），保持后点完成。'),
        ]
    os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
    import pygame
    pygame.display.init()
    pygame.joystick.init()
    lock = threading.Lock()
    state = {'index': 0, 'active': False, 'live': {}, 'device': '', 'error': '等待手柄', 'results': []}
    samples = []
    report = {'read_only': True, 'results': []}

    def save():
        report['results'] = state['results']
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, value, html=False, code=200):
            data = value.encode('utf-8') if html else json.dumps(value, ensure_ascii=False).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'text/html; charset=utf-8' if html else 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/':
                return self.reply(HTML, html=True)
            with lock:
                if self.path == '/report':
                    return self.reply(report)
                if self.path != '/state':
                    return self.reply({'error': 'not found'}, code=404)
                done = state['index'] >= len(STEPS)
                summary = [{k: v for k, v in row.items() if k != 'samples'} for row in state['results']]
                return self.reply(dict(state, results=summary, done=done,
                    label='检查完成' if done else STEPS[state['index']][0],
                    hint='结果已保存，可以回到聊天。' if done else STEPS[state['index']][1]))

        def do_POST(self):
            origin = self.headers.get('Origin')
            if self.path != '/action' or origin != f'http://127.0.0.1:{args.port}':
                return self.reply({'error': 'invalid origin'}, code=403)
            try:
                size = int(self.headers.get('Content-Length', 0))
                if not 0 < size <= 128:
                    raise ValueError('body size')
                action = json.loads(self.rfile.read(size))['action']
            except (ValueError, KeyError):
                return self.reply({'error': 'invalid request'}, code=400)
            with lock:
                if state['index'] >= len(STEPS):
                    return self.reply({'error': 'already complete'}, code=409)
                if action == 'start' and not state['active'] and not state['error']:
                    samples.clear()
                    state['active'] = True
                elif action == 'skip' or (action == 'finish' and state['active'] and not state['error']):
                    result = {'label': STEPS[state['index']][0], 'skipped': action == 'skip'}
                    if action == 'finish':
                        if len(samples) < 25:
                            return self.reply({'error': '请至少采样半秒'}, code=409)
                        result['axis_ranges'] = [{'axis': i, 'min': min(s['axes'][i] for s in samples),
                            'max': max(s['axes'][i] for s in samples), 'end': samples[-1]['axes'][i]}
                            for i in range(len(samples[0]['axes']))]
                        result['changed_axes'] = [r['axis'] for r in result['axis_ranges'] if r['max']-r['min'] > .2]
                        result['changed_buttons'] = [i for i in range(len(samples[0]['buttons']))
                            if min(s['buttons'][i] for s in samples) != max(s['buttons'][i] for s in samples)]
                        result['samples'] = list(samples)
                    state['results'].append(result)
                    state['active'] = False
                    state['index'] += 1
                    save()
                else:
                    return self.reply({'error': 'invalid state'}, code=409)
            self.reply({'ok': True})

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'READ ONLY: http://127.0.0.1:{args.port}/', flush=True)
    device = None
    try:
        while True:
            pygame.event.pump()
            if device is not None and not joystick_attached(pygame, device):
                device = None
            if device is None and pygame.joystick.get_count():
                device = pygame.joystick.Joystick(0)
            with lock:
                if device is None:
                    state['error'] = '手柄未连接；请重新连接，当前项需重新采样'
                    state['active'] = False
                else:
                    state['error'] = ''
                    state['device'] = device.get_name()
                    report['device'] = {'name': device.get_name(), 'guid': device.get_guid()}
                    state['live'] = {'axes': [round(device.get_axis(i), 5) for i in range(device.get_numaxes())],
                        'buttons': [device.get_button(i) for i in range(device.get_numbuttons())]}
                    if state['active']:
                        samples.append(dict(state['live'], time=time.time()))
                        if len(samples) >= 3000:
                            state['active'] = False
                            state['error'] = '本项超过60秒，请重新开始该项'
            time.sleep(.02)
    finally:
        server.shutdown()
        with lock:
            save()
        pygame.quit()


if __name__ == '__main__':
    main()
