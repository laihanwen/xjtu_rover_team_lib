import json
from pathlib import Path
import struct
import tempfile
import time
import unittest
from telemetry_log import TelemetryLog
from trial_protocol import decode_pid, encode, Parser


class LogTests(unittest.TestCase):
    def test_pid_wire_and_wrapped_error(self):
        payload = struct.pack('<IB11f',1234,31,1,2,179,-1,-2,2,7,14,20,3,4)
        frame = Parser().push(encode(0x84,payload))[0]
        decoded = decode_pid(frame[1])
        self.assertEqual(decoded['tick_ms'],1234)
        self.assertEqual(decoded['yaw_error_deg'],2)
        self.assertTrue(decoded['heading_hold_active'])
        self.assertEqual(decoded['pitch_correction'],14)
        with self.assertRaises(ValueError): decode_pid(payload[:-1])

    def test_missing_and_stale_not_claimed_running(self):
        state = {'connected':True,'telemetry_age':.1,'telemetry':{'roll_deg':0,'pitch_deg':0,'yaw_deg':0}}
        result = TelemetryLog.record(state,12)
        self.assertTrue(result['imu_valid'])
        self.assertFalse(result['pid_valid'])
        self.assertIsNone(result['pid'])
        state.update(pid={'armed':True},pid_age=1)
        self.assertFalse(TelemetryLog.record(state,13)['pid_valid'])
        state['telemetry_age']=1
        self.assertFalse(TelemetryLog.record(state,14)['imu_valid'])

    def test_hold_diagnostic_extension(self):
        payload=struct.pack('<IB14f',1234,255,*([0]*11+[1.2,1.1,30]))
        decoded=decode_pid(payload)
        self.assertTrue(decoded['heading_requested'])
        self.assertTrue(decoded['depth_requested'])
        self.assertTrue(decoded['depth_hold_active'])
        self.assertAlmostEqual(decoded['depth_target_m'],1.1,places=5)

    def test_periodic_flush_rotation_no_source_mutation(self):
        source={'connected':False,'axes':[0,0,0,0]}
        with tempfile.TemporaryDirectory() as directory:
            logger=TelemetryLog(directory,lambda:source,interval=.025,max_bytes=1)
            deadline=time.monotonic()+1
            while logger.status()['records']<3 and time.monotonic()<deadline: time.sleep(.01)
            logger.close()
            self.assertIsNone(logger.status()['error'])
            paths=list(Path(directory).rglob('*.jsonl'))
            self.assertGreaterEqual(len(paths),3)
            self.assertTrue(all(json.loads(p.read_text())['schema']==1 for p in paths))
            self.assertEqual(source,{'connected':False,'axes':[0,0,0,0]})

    def test_io_error_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            file=Path(directory)/'file';file.write_text('x')
            logger=TelemetryLog(file,lambda:{},interval=.01)
            logger.thread.join(1)
            self.assertIsNotNone(logger.status()['error'])
            logger.close()


if __name__=='__main__':unittest.main()
