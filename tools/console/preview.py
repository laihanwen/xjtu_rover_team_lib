"""Loopback-only UI demonstration. No robot connections or command endpoints."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ASSETS = Path(__file__).resolve().parents[2]/'runtime/web/dashboard'
ROV = Path(__file__).resolve().parents[2]/'tools/rov/web'


class Preview(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass
    def do_GET(self):
        if self.path == '/':
            self.send_response(302)
            self.send_header('Location', '/dashboard/')
            self.end_headers()
            return
        path = self.path.split('?', 1)[0]
        if path == '/rov/':
            html=(ROV/'console.html').read_text(encoding='utf-8')
            html=html.replace('<script defer src="/console.js"></script>','<script defer src="/rov/demo.js"></script>')
            html=html.replace('/console.css','/rov/console.css')
            html=html.replace('<main>','<main><section><b class="demo-banner">虚拟 ROV 检视 · 不连接机器人</b><p>所有遥测和画面为模拟。<a href="/dashboard/">查看 AUV 模拟 →</a></p></section>')
            body=html.encode();mime='text/html'
        elif path in ('/rov/console.css','/rov/demo.js'):
            file=ROV/'console.css' if path.endswith('.css') else Path(__file__).with_name('rov_demo.js')
            body=file.read_bytes();mime='text/css' if path.endswith('.css') else 'text/javascript'
        elif path == '/dashboard/config':
            body = json.dumps({'demo': True, 'source': 'virtual', 'camera_base': ''}).encode()
            mime = 'application/json'
        else:
            files = {'/dashboard/': ('index.html', 'text/html'),
                     '/dashboard/dashboard.css': ('dashboard.css', 'text/css'),
                     '/dashboard/dashboard.js': ('dashboard.js', 'text/javascript'),
                     '/dashboard/recording.js': ('recording.js', 'text/javascript')}
            files.update({'/dashboard/logs.html':('logs.html','text/html'),'/dashboard/logs.js':('logs.js','text/javascript')})
            if path not in files:
                self.send_error(404)
                return
            name, mime = files[path]
            body = (ASSETS/name).read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', mime+'; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)
    def do_POST(self):
        self.send_error(405, 'Virtual UI has no hardware command endpoints')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8769)
    args = parser.parse_args()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Preview)
    print(f'Virtual console http://127.0.0.1:{args.port}/dashboard/ ; no hardware access', flush=True)
    server.serve_forever()
