import json
from pathlib import Path
from datetime import datetime
import tempfile,unittest
from review_manual_task import telemetry
class EvidenceTests(unittest.TestCase):
 def test_stale_sensor_is_not_motion_evidence(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'telemetry_1.jsonl';p.write_text(json.dumps(dict(utc='2026-10-09T14:50:03+00:00',pc_monotonic_s=1,imu_valid=False,imu={'depth_m':-2,'outputs':[1]},localization={'valid':False,'reason':'calibration_required'}))+'\n{broken\n',encoding='utf-8')
   stats,rows=telemetry([p],datetime.fromisoformat('2026-10-09T14:50:02+00:00'),datetime.fromisoformat('2026-10-09T14:50:04+00:00'))
   self.assertEqual(stats['records'],1);self.assertIsNone(stats['depth_range_m']);self.assertEqual(stats['output_near_limit_records'],0);self.assertEqual(len(stats['parse_errors']),1)
if __name__=='__main__':unittest.main()
