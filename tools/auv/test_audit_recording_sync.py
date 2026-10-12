import json
import tempfile
import unittest
from pathlib import Path
from audit_recording_sync import audit, utc


class SyncEvidenceTests(unittest.TestCase):
    def test_near_receipt_is_never_metric_sync(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            logs = root / 'logs'
            logs.mkdir()
            folder = root / 'videos' / '20261011T163151_example'
            folder.mkdir(parents=True)
            (logs / 'telemetry_0001.jsonl').write_text(json.dumps(dict(
                utc='2026-10-11T16:31:51+00:00', imu_valid=True,
                telemetry_age_s=.02, imu=dict(sequence=1, depth_m=.4))) + '\n', encoding='utf-8')
            (folder / 'down_0001.frames.jsonl').write_text(json.dumps(dict(
                frame=0, capture_monotonic_sec=25,
                received_utc='2026-10-11T16:31:51.010+00:00')) + '\n', encoding='utf-8')
            report = audit(root / 'videos', logs, root / 'out')
            self.assertEqual(report['totals']['pc_receipt_candidates_fresh'], 1)
            self.assertEqual(report['totals']['metric_ready'], 0)
            self.assertFalse(report['metric_replay_authorized'])

    def test_utc_without_timezone_rejected(self):
        with self.assertRaises(ValueError):
            utc('2026-10-11T16:31:51')


if __name__ == '__main__':
    unittest.main()
