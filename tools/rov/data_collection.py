"""Timed operator-labelled data capture; no control commands or camera capture."""
import json
import math
from pathlib import Path
import threading
import time
import uuid
from datetime import datetime, timezone

KINDS = {'depth', 'front_calibration', 'camera_handover', 'installation', 'motion_response'}


def metadata(value):
    if value.get('kind') not in KINDS:
        raise ValueError('请选择采集项目')
    if value.get('safe_confirmed') is not True:
        raise ValueError('请确认无动力或可靠固定；采集入口不发送运动命令')
    result = {'kind': value['kind'], 'note': str(value.get('note', ''))[:1000]}
    for key in ('reference_depth_m', 'pool_depth_m', 'camera_pitch_deg'):
        x = value.get(key)
        if x not in (None, ''):
            x = float(x)
            if not math.isfinite(x) or (key != 'camera_pitch_deg' and not 0 <= x <= 100):
                raise ValueError('实测参数不合法：' + key)
            result[key] = x
    return result


class DataCollection:
    def __init__(self, directory, snapshot, recording):
        self.directory = Path(directory)
        self.snapshot = snapshot
        self.recording = recording
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.file = None
        self.current = {'active': False, 'samples': 0, 'error': '', 'directory': ''}

    def status(self):
        with self.lock:
            return dict(self.current)

    def write(self, value):
        self.file.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')
        self.file.flush()

    def start(self, value):
        info = metadata(value)
        duration = float(value.get('duration_sec', 30))
        if not math.isfinite(duration) or not 10 <= duration <= 600:
            raise ValueError('采集时长应为 10–600 秒')
        with self.lock:
            if self.current['active']:
                raise ValueError('已有采集正在进行，请先结束')
            self.directory.mkdir(parents=True, exist_ok=True)
            folder = self.directory / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'_'+uuid.uuid4().hex[:12])
            folder.mkdir()
            self.file = (folder/'samples.jsonl').open('w', encoding='utf-8')
            self.current = dict(active=True, samples=0, error='', directory=str(folder), duration_sec=duration,
                                metadata=info, latest=None, verified=False)
            try:
                self.write(dict(event='BEGIN', utc=datetime.now(timezone.utc).isoformat(), monotonic=time.monotonic(),
                                metadata=info, recording=self.recording(), calibration_verified=False, nominal_interval_sec=.5))
                self.stop_event.clear()
                self.thread = threading.Thread(target=self.collect, daemon=True)
                self.thread.start()
            except Exception as error:
                self.file.close()
                self.current.update(active=False, error=str(error))
                self.thread = None
                raise
            return self.status()

    def collect(self):
        began = time.monotonic()
        try:
            while not self.stop_event.is_set() and time.monotonic()-began < self.current['duration_sec']:
                value = dict(event='SAMPLE', utc=datetime.now(timezone.utc).isoformat(),
                             monotonic=time.monotonic(), elapsed_sec=time.monotonic()-began,
                             observation=self.snapshot(), recording=self.recording())
                with self.lock:
                    self.write(value)
                    self.current['samples'] += 1
                    self.current['latest'] = value['observation']
                self.stop_event.wait(.5)
        except Exception as error:
            with self.lock:
                self.current['error'] = str(error)
        finally:
            with self.lock:
                try:
                    self.write(dict(event='END', monotonic=time.monotonic(), samples=self.current['samples'],
                                    error=self.current['error'], recording=self.recording()))
                except Exception as error:
                    self.current['error'] = str(error)
                finally:
                    self.file.close()
                    self.current['active'] = False

    def mark(self, value):
        info = metadata(value)
        with self.lock:
            if not self.current['active']:
                raise ValueError('请先开始采集')
            self.write(dict(event='MARK', utc=datetime.now(timezone.utc).isoformat(), monotonic=time.monotonic(), metadata=info))
        return self.status()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=4)
        return self.status()

    def export(self):
        with self.lock:
            if self.current['active']:
                raise ValueError('先结束采集再导出')
            if not self.current['directory']:
                raise ValueError('尚无采集记录')
            return (Path(self.current['directory'])/'samples.jsonl').read_bytes()
