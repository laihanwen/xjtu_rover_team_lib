import struct
import sys
import types
import unittest
from unittest.mock import patch
from one_click_pi import PREPARE
from trial_protocol import encode, Parser

class PreparationTests(unittest.TestCase):
    def run_prepare(self,armed=False,repeated=False):
        writes=[];sequence=0;ticks=0
        class Port:
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def write(self,data):writes.append(data)
            def read(self,_):
                nonlocal sequence
                sequence+=1
                payload=bytearray(46);struct.pack_into('<I',payload,0,1 if repeated else sequence)
                payload[4]=int(armed);payload[29]=8
                return encode(0x80,payload)
        def clock():
            nonlocal ticks
            ticks+=1;return ticks*.2
        fake_serial=types.SimpleNamespace(Serial=lambda *a,**k:Port())
        fake_yaml=types.SimpleNamespace(safe_load=lambda _:dict(serial=dict(device=''),motion=dict(motion_commands_enabled=False),operation={}))
        result=types.SimpleNamespace(returncode=0,stdout='/etc/auv-runtime/runtime.yaml')
        with patch.dict(sys.modules,serial=fake_serial,yaml=fake_yaml),patch('pathlib.Path.read_text',return_value=''),patch('subprocess.run',return_value=result),patch('time.monotonic',side_effect=clock):
            try:exec(PREPARE,{'DEVICE':'fake'});failure=None
            except RuntimeError as e:failure=str(e)
        frames=Parser().push(b''.join(writes))
        self.assertTrue(frames)
        self.assertTrue(all(kind in (1,2) for kind,payload in frames))
        self.assertTrue(all(payload[-1]==0 for kind,payload in frames if kind==2))
        return failure
    def test_neutral_confirmation(self):self.assertIsNone(self.run_prepare())
    def test_armed_rejected(self):self.assertIn('confirmation failed',self.run_prepare(armed=True))
    def test_repeated_status_rejected(self):self.assertIn('confirmation failed',self.run_prepare(repeated=True))

if __name__=='__main__':unittest.main()
