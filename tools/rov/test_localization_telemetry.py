import json
from pathlib import Path
import tempfile,time,unittest
from localization_telemetry import LocalizationTelemetry
class ExportTests(unittest.TestCase):
 def test_latest_atomic_and_stale(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'input.json';export=LocalizationTelemetry(p)
   try:
    export.offer(dict(roll_deg=0,pitch_deg=1,yaw_deg=2,depth_m=.5,armed=False,sequence=7),10,10.1)
    limit=time.monotonic()+2
    while not p.exists() and time.monotonic()<limit:time.sleep(.02)
    value=json.loads(p.read_text());self.assertTrue(value['valid']);self.assertEqual(value['stamp'],10)
    export.offer(dict(roll_deg=0,pitch_deg=1,yaw_deg=2,depth_m=.5,armed=False),10,11)
    while time.monotonic()<limit:
     value=json.loads(p.read_text())
     if not value['valid']:break
     time.sleep(.02)
    self.assertFalse(value['valid']);self.assertIsNone(value['depth_m'])
   finally:export.close()
 def test_missing_and_nan_rejected_without_blocking(self):
  with tempfile.TemporaryDirectory() as d:
   export=LocalizationTelemetry(Path(d)/'i.json')
   try:
    export.offer({'depth_m':float('nan')},1,1)
    with export.lock:self.assertFalse(export.latest['valid']);json.dumps(export.latest,allow_nan=False)
   finally:export.close()
if __name__=='__main__':unittest.main()
