"""Read-only latest telemetry export; disk I/O never runs in the UART loop."""
import json
import math
import os
from pathlib import Path
import threading
import time

class LocalizationTelemetry:
    def __init__(self,path):
        self.path=Path(path);self.latest=None;self.lock=threading.Lock();self.closed=threading.Event();self.error=''
        self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()
    def offer(self,telemetry,received,now):
        fields=('roll_deg','pitch_deg','yaw_deg','depth_m')
        valid=0<=now-received<=.3 and all(type(telemetry.get(k)) in (int,float) and math.isfinite(telemetry[k]) for k in fields)
        value={k:telemetry.get(k) if valid else None for k in fields}
        value.update(stamp=received,valid=valid,armed=bool(telemetry.get('armed',True)),sequence=telemetry.get('sequence'))
        with self.lock:self.latest=value
    def _run(self):
        previous=None
        while not self.closed.wait(.1):
            with self.lock:value=self.latest
            if value is None or value is previous:continue
            try:
                self.path.parent.mkdir(parents=True,exist_ok=True)
                temp=self.path.with_suffix('.tmp')
                temp.write_text(json.dumps(value,allow_nan=False),encoding='utf-8')
                os.replace(temp,self.path);previous=value;self.error=''
            except OSError as e:self.error=str(e)
    def close(self):
        self.closed.set();self.thread.join(timeout=1)
