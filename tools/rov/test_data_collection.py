import json
from pathlib import Path
import tempfile
import time
import unittest
from data_collection import DataCollection, metadata


class CollectionTests(unittest.TestCase):
    def test_start_failure_can_retry_without_leaking_file(self):
        with tempfile.TemporaryDirectory() as root:
            def failed_recording():
                raise OSError('recording status unavailable')
            c=DataCollection(root,lambda:{},failed_recording)
            with self.assertRaises(OSError):c.start(dict(kind='depth',safe_confirmed=True))
            self.assertFalse(c.status()['active'])
            self.assertTrue(c.file.closed)
            self.assertTrue(c.status()['error'])
            c.recording=lambda:{}
            c.start(dict(kind='depth',safe_confirmed=True))
            c.stop()
            self.assertFalse(c.status()['active'])

    def test_metadata_requires_physical_confirmation(self):
        with self.assertRaises(ValueError):metadata({'kind':'depth'})
        for x in ('NaN', '-1', '101'):
            with self.assertRaises(ValueError):metadata(dict(kind='depth',safe_confirmed=True,reference_depth_m=x))

    def test_persists_raw_invalid_depth_and_video_reference(self):
        with tempfile.TemporaryDirectory() as root:
            c=DataCollection(root,lambda:{'source':'runtime','status':{'depth_m':None,'depth_sample_fresh':False}},lambda:{'active':True,'directory':'video-run'})
            c.start(dict(kind='depth',safe_confirmed=True,reference_depth_m=.2,duration_sec=20))
            deadline=time.monotonic()+2
            while c.status()['samples']<1 and time.monotonic()<deadline:time.sleep(.01)
            c.mark(dict(kind='depth',safe_confirmed=True,note='known depth'))
            with self.assertRaises(ValueError):c.export()
            c.stop()
            rows=[json.loads(x) for x in c.export().decode().splitlines()]
            self.assertEqual([x['event'] for x in rows],['BEGIN','SAMPLE','MARK','END'])
            self.assertIsNone(rows[1]['observation']['status']['depth_m'])
            self.assertEqual(rows[1]['recording']['directory'],'video-run')
            self.assertFalse(rows[0]['calibration_verified'])

    def test_duplicate_start_and_invalid_mark(self):
        with tempfile.TemporaryDirectory() as root:
            c=DataCollection(root,lambda:{},lambda:{})
            with self.assertRaises(ValueError):c.mark(dict(kind='depth',safe_confirmed=True))
            c.start(dict(kind='depth',safe_confirmed=True))
            try:
                with self.assertRaises(ValueError):c.start(dict(kind='depth',safe_confirmed=True))
            finally:c.stop()

    def test_write_failure_reported(self):
        with tempfile.TemporaryDirectory() as root:
            c=DataCollection(root,lambda:{'invalid':float('nan')},lambda:{})
            c.start(dict(kind='depth',safe_confirmed=True))
            deadline=time.monotonic()+2
            while c.status()['active'] and time.monotonic()<deadline:time.sleep(.01)
            self.assertFalse(c.status()['active'])
            self.assertTrue(c.status()['error'])


if __name__=='__main__':unittest.main()
