"""Five-second snapshots, no polling leases, UART, or control commands."""
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time
import uuid


class TelemetryLog:
    def __init__(self, directory, snapshot, interval=1, max_bytes=10*1024*1024):
        if interval <= 0 or max_bytes <= 0:
            raise ValueError('invalid log limits')
        self.directory = Path(directory) / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'_'+uuid.uuid4().hex[:8])
        self.snapshot = snapshot
        self.interval = interval
        self.max_bytes = max_bytes
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.info = {'directory':str(self.directory),'interval_s':interval,'records':0,'error':None}
        self.recent = []
        self.thread = threading.Thread(target=self.run, name='imu-pid-log', daemon=True)
        self.thread.start()

    def status(self):
        with self.lock:
            return dict(self.info, recent=copy.deepcopy(self.recent))

    @staticmethod
    def record(state, now):
        state = copy.deepcopy(state)
        age = state.get('telemetry_age')
        fresh = bool(state.get('connected') and isinstance(age,(int,float)) and 0 <= age <= .3)
        pid_age = state.get('pid_age')
        pid_fresh = bool(fresh and state.get('pid') and isinstance(pid_age,(int,float)) and 0 <= pid_age <= .3)
        return {'schema':1,'schema_name':'rov.telemetry','schema_revision':2,
                'source':'pc_received_rov_bridge','record_type':'SNAPSHOT',
                'units':{'imu_angles':'deg','depth':'m','outputs':'firmware_native','age':'s'},
                'validity':{'telemetry_fresh':fresh,'reason':'fresh' if fresh else 'disconnected_or_stale'},
                'received_cache':state,
                'utc':datetime.now(timezone.utc).isoformat(),
                'pc_monotonic_s':now,'connected':bool(state.get('connected')),
                'localization':state.get('localization'),
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
        previous = None
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            manifest={'schema':'rov.log_session.v2','session_id':self.directory.name,
                      'source':'PC cache snapshots; no additional UART requests',
                      'interval_s':self.interval,'clock':'pc_monotonic_s; utc is wall clock',
                      'limitations':['Transitions observed at sampling cadence, not exact MCU event time',
                                      'Stale IMU/PID omitted; absent values are unknown, never zero',
                                      'PC and Pi monotonic clocks must not be subtracted'],
                      'files':'telemetry_*.jsonl; chronological sequence across rotated segments'}
            (self.directory/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
            segment = 0
            deadline = time.monotonic() + self.interval
            while not self.stop_event.wait(max(0,deadline-time.monotonic())):
                now = time.monotonic()
                record=self.record(self.snapshot(),now)
                record['session_id']=self.directory.name
                record['sequence']=self.info['records']+1
                observed={'connected':record['connected'],'telemetry_fresh':record['validity']['telemetry_fresh'],
                          'armed':(record['imu'] or {}).get('armed'),
                          'deadman':record['operator']['deadman'],'error':record['error'],
                          'heading_state':(record['pid'] or {}).get('heading_state'),
                          'depth_state':(record['pid'] or {}).get('depth_state')}
                record['changes']=[] if previous is None else [dict(field=k,before=previous[k],after=v) for k,v in observed.items() if previous[k]!=v]
                previous=observed
                line = json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n'
                if stream is None or stream.tell()+len(line.encode('utf-8')) > self.max_bytes:
                    if stream: stream.close()
                    segment += 1
                    stream = (self.directory/f'telemetry_{segment:04d}.jsonl').open('w',encoding='utf-8',newline='\n')
                stream.write(line)
                stream.flush()
                with self.lock:
                    self.info['records'] += 1
                    self.recent.append(record)
                    self.recent=self.recent[-60:]
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
