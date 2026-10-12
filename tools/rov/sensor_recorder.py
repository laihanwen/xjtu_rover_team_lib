"""Bounded, optional Pi-clock sensor evidence writer; never controls hardware."""
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid
import hashlib
import shutil
from datetime import datetime, timezone


class SensorRecorder:
    def __init__(self, directory, model=None, max_bytes=64*1024*1024, queue_size=512,min_free_bytes=256*1024*1024):
        if type(queue_size) is not int or queue_size<1 or max_bytes<1 or min_free_bytes<0:
            raise ValueError('invalid sensor recording resource budget')
        self.min_free_bytes=min_free_bytes;self.free_bytes=None
        self.directory = Path(directory) / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
        self.model = {key: (model or {}).get(key) for key in
                      ['source_commit', 'imu_signs', 'imu_offsets_deg', 'depth_zero_m',
                       'depth_reference', 'sensor_model_verified']}
        self.model['depth_absolute_offset_m']=(model or {}).get('depth_absolute_offset_m')
        self.max_bytes = max_bytes
        self.queue = queue.Queue(maxsize=queue_size)
        self.closed = threading.Event()
        self.error = ''
        self.dropped = 0
        self.written = 0
        self.bytes_written = 0
        self.previous = None
        self.mcu_epoch=0;self.last_tick=None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def offer(self, telemetry, stamp, kind='status'):
        if self.error or self.closed.is_set():
            self.dropped+=1;return
        key = (kind, telemetry.get('sequence'), telemetry.get('tick_ms'), stamp)
        if key == self.previous:
            return
        self.previous = key
        tick=telemetry.get('tick_ms') if kind=='pid' else None
        if type(tick) is int:
            if self.last_tick is not None and tick<self.last_tick:self.mcu_epoch+=1
            self.last_tick=tick
        fields = ['roll_deg', 'pitch_deg', 'yaw_deg', 'depth_m']
        finite = math.isfinite(stamp) and all(type(telemetry.get(k)) in (int, float)
                                            and math.isfinite(telemetry[k]) for k in fields)
        row = {k: telemetry.get(k) if type(telemetry.get(k)) in (int,float)
               and math.isfinite(telemetry[k]) else None for k in fields}
        row.update(schema='pi.sensor.v1', kind=kind, stamp=stamp if math.isfinite(stamp) else None,
                   clock='Pi CLOCK_MONOTONIC', stamp_kind='UART receipt; not MCU sensor sample',
                   sequence=telemetry.get('sequence'), numeric_valid=finite,
                   armed=bool(telemetry.get('armed', True)), error_flags=telemetry.get('error_flags'),
                   depth_sample_fresh=(type(telemetry.get('depth_m')) in (int,float) and math.isfinite(telemetry['depth_m'])) if kind=='pid' and 'depth_m' in telemetry else None,
                   imu_sample_fresh=telemetry.get('imu_fresh') if kind=='pid' else None,
                   mcu_tick_ms=tick,mcu_epoch=self.mcu_epoch,
                   utc=datetime.now(timezone.utc).isoformat())
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            self.dropped += 1

    def status(self):
        return dict(enabled=True, directory=str(self.directory), written=self.written,
                    dropped=self.dropped, error=self.error, bytes=self.bytes_written,free_bytes=self.free_bytes)

    def _run(self):
        try:
            self.directory.mkdir(parents=True, exist_ok=False)
            boot = Path('/proc/sys/kernel/random/boot_id')
            metadata = dict(schema='pi.sensor.session.v1', boot_id=boot.read_text().strip() if boot.exists() else None,
                            recorder_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                            clock='Pi CLOCK_MONOTONIC', model=self.model,
                            freshness='Status freshness unknown; PID imu_fresh bit and finite depth (firmware NAN on stale)',
                            max_bytes=self.max_bytes)
            (self.directory/'session.json').write_text(json.dumps(metadata, allow_nan=False, indent=2), encoding='utf-8')
            with (self.directory/'sensors.jsonl').open('w', encoding='utf-8') as stream:
                last_flush = time.monotonic()
                last_space_check=-1
                while not self.closed.is_set() or not self.queue.empty():
                    try:
                        row = self.queue.get(timeout=.1)
                    except queue.Empty:
                        if time.monotonic()-last_flush >= 1:
                            stream.flush(); last_flush = time.monotonic()
                        continue
                    row['dropped_before'] = self.dropped
                    if self.error:self.dropped+=1;continue
                    if time.monotonic()-last_space_check>=1:
                        self.free_bytes=shutil.disk_usage(self.directory).free;last_space_check=time.monotonic()
                        if self.free_bytes<self.min_free_bytes:
                            self.error='sensor recording disk reserve reached';self.dropped+=1;continue
                    data = json.dumps(row, allow_nan=False)+'\n'
                    size = len(data.encode('utf-8'))
                    if self.bytes_written+size > self.max_bytes:
                        self.error = 'sensor recording byte budget exhausted'
                        self.dropped += 1
                        continue
                    stream.write(data); self.bytes_written += size; self.written += 1
                    if time.monotonic()-last_flush >= 1:
                        stream.flush(); last_flush = time.monotonic()
            result=self.status();digest=hashlib.sha256()
            with (self.directory/'sensors.jsonl').open('rb') as stream:
                for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
            result['sensors_sha256']=digest.hexdigest()
            (self.directory/'result.json').write_text(json.dumps(result), encoding='utf-8')
        except (OSError, ValueError) as error:
            self.error = str(error)

    def close(self):
        self.closed.set()
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            self.error = 'sensor writer shutdown timeout'
