import json
import tempfile
import unittest
from pathlib import Path
from sensor_recorder import SensorRecorder


class EvidenceWriterTests(unittest.TestCase):
    def test_distinct_status_pid_and_missing_metadata(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder=SensorRecorder(temp)
            value=dict(sequence=2,roll_deg=1,pitch_deg=2,yaw_deg=3,depth_m=.4,armed=False)
            recorder.offer(value,10)
            recorder.offer(value,10)
            recorder.offer(dict(value,imu_fresh=True,tick_ms=20),10,'pid')
            recorder.close()
            rows=[json.loads(line) for line in (recorder.directory/'sensors.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows),2)
            self.assertEqual(rows[0]['stamp'],10)
            self.assertIsNone(rows[0]['depth_sample_fresh'])
            self.assertTrue(rows[1]['depth_sample_fresh'])
            self.assertTrue(rows[1]['imu_sample_fresh'])
            self.assertFalse(rows[0]['armed'])
            model=json.loads((recorder.directory/'session.json').read_text())['model']
            self.assertIsNone(model['imu_signs'])
            self.assertIsNone(model['depth_absolute_offset_m'])

    def test_budget_exhaustion_is_visible(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder=SensorRecorder(temp,max_bytes=1)
            recorder.offer(dict(roll_deg=0,pitch_deg=0,yaw_deg=0,depth_m=float('nan'),imu_fresh=True),1,'pid')
            recorder.close()
            self.assertEqual(recorder.written,0)
            self.assertEqual(recorder.dropped,1)
            self.assertIn('budget',recorder.error)


if __name__=='__main__':unittest.main()
