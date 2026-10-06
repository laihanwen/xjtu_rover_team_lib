"""Five-second snapshots, no polling leases, UART, or control commands."""
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time


class TelemetryLog:
    def __init__(self, directory, snapshot, interval=5, max_bytes=10*1024*1024):
        self.directory = Path(directory) / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        self.snapshot = snapshot
        self.interval = interval
        self.max_bytes = max_bytes
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.info = {'directory':str(self.directory),'interval_s':interval,'records':0,'error':None}
        self.thread = threading.Thread(target=self.run, name='imu-pid-log', daemon=True)
        self.thread.start()

    def status(self):
        with self.lock:
            return dict(self.info)

    @staticmethod
    def record(state, now):
        state = copy.deepcopy(state)
        age = state.get('telemetry_age')
        fresh = bool(state.get('connected') and age is not None and age <= .3)
        pid_age = state.get('pid_age')
        pid_fresh = bool(fresh and state.get('pid') and pid_age is not None and pid_age <= .3)
        return {'schema':1,'utc':datetime.now(timezone.utc).isoformat(),
                'pc_monotonic_s':now,'connected':bool(state.get('connected')),
                'telemetry_age_s':age,'imu_valid':bool(fresh and all(
                    state.get('telemetry',{}).get(k) is not None
                    for k in ('roll_deg','pitch_deg','yaw_deg'))),
                'imu':state.get('telemetry',{}) if fresh else None,
                'pid_valid':pid_fresh,'pid_age_s':pid_age,
                'pid':state.get('pid') if pid_fresh else None,
                'operator':{k:state.get(k) for k in ('axes','deadman','centered','device')},
                'pwm_limit_us':state.get('pwm_limit'),'selected_speed':state.get('selected_speed'),
                'uart_crc_errors':state.get('uart_crc_errors'),'detail':state.get('detail'),
                'error':state.get('error')}

    def run(self):
        stream = None
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            segment = 0
            deadline = time.monotonic() + self.interval
            while not self.stop_event.wait(max(0,deadline-time.monotonic())):
                now = time.monotonic()
                line = json.dumps(self.record(self.snapshot(),now),ensure_ascii=False,allow_nan=False)+'\n'
                if stream is None or stream.tell()+len(line.encode('utf-8')) > self.max_bytes:
                    if stream: stream.close()
                    segment += 1
                    stream = (self.directory/f'telemetry_{segment:04d}.jsonl').open('w',encoding='utf-8',newline='\n')
                stream.write(line)
                stream.flush()
                with self.lock: self.info['records'] += 1
                deadline += self.interval
                if deadline <= time.monotonic():
                    deadline = time.monotonic()+self.interval
        except Exception as error:
            with self.lock: self.info['error'] = str(error)
        finally:
            if stream: stream.close()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)
