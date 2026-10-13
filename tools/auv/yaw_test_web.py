"""Local read-only yaw page. Proxy only GET /api/status; no control endpoints."""
import argparse
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import urllib.request
import urllib.error

ROOT = Path(__file__).resolve().parents[2]


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/api/status':
            try:
                with urllib.request.urlopen('http://192.168.137.150:8080/api/status', timeout=1) as response:
                    data = response.read(4 * 1024 * 1024 + 1)
                if len(data) > 4 * 1024 * 1024:
                    raise ValueError('status response too large')
                state = json.loads(data)
                state['yaw_source'] = 'Pi AUV Runtime'
                if state.get('serial') is not True or state.get('status_fresh') is not True:
                    # Reuse the existing PC bridge. Never open the UART or TCP controller.
                    try:
                        with urllib.request.urlopen('http://127.0.0.1:8767/state', timeout=1) as response:
                            bridge = json.load(response)
                        t, pid = bridge.get('telemetry', {}), bridge.get('pid', {})
                        age, pid_age = bridge.get('telemetry_age'), bridge.get('pid_age')
                        fresh = (bridge.get('connected') is True and isinstance(age, (int, float))
                                 and 0 <= age <= 0.5 and isinstance(pid_age, (int, float))
                                 and 0 <= pid_age <= 0.5 and pid.get('imu_fresh') is True)
                        yaw = t.get('yaw_deg')
                        state.update(yaw_source='PC ROV bridge /state',
                                     run_id='rov-bridge:' + str(state.get('run_id')),
                                     serial=bridge.get('connected') is True,
                                     status_fresh=fresh, status_age_sec=age,
                                     armed=t.get('armed'), error_flags=t.get('error_flags'),
                                     yaw_rad=math.radians(yaw) if isinstance(yaw, (int, float)) else None,
                                     telemetry_sequence=t.get('sequence'), imu_tick_ms=pid.get('tick_ms'))
                    except (OSError, ValueError, TypeError):
                        pass
                data = json.dumps(state).encode('utf-8')
                mime = 'application/json'
            except (OSError, ValueError):
                self.send_error(502, 'Pi status unavailable')
                return
        elif self.path in ('/', '/yaw.html', '/yaw.js'):
            name = 'yaw.js' if self.path == '/yaw.js' else 'yaw.html'
            data = (ROOT / 'runtime/web/dashboard' / name).read_bytes()
            mime = 'text/javascript' if name.endswith('.js') else 'text/html'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', mime + '; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8771)
    args = parser.parse_args()
    print(f'http://127.0.0.1:{args.port}/', flush=True)
    ThreadingHTTPServer(('127.0.0.1', args.port), Handler).serve_forever()
