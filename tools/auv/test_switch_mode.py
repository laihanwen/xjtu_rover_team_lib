import struct
import unittest
from switch_mode import select_mcu
from trial_protocol import encode, Parser


class FakeMcu:
    def __init__(self, capable=True, reject=False, neutral=True, wrong_mode=False):
        self.capable=capable;self.reject=reject;self.neutral=neutral;self.wrong_mode=wrong_mode
        self.mode=0;self.seq=0;self.sent=[];self.queue=b'';self.now=0
    def write(self, data):
        for kind,payload in Parser().push(data):
            self.sent.append((kind,payload))
            if kind==9:
                self.mode=payload[4]
                self.queue+=encode(0x7f,bytes([9,3 if self.reject else 0])+payload[:4])
    def read(self, _):
        self.now+=.05;self.seq+=1
        payload=bytearray(46);struct.pack_into('<I',payload,0,self.seq)
        payload[4]=(16 if self.capable else 0)|(32 if self.mode and not self.wrong_mode else 0)
        payload[29]=8
        if not self.neutral:struct.pack_into('<h',payload,30,100)
        result=self.queue+encode(0x80,payload);self.queue=b'';return result


class SwitchTests(unittest.TestCase):
    def test_switch_and_never_arm(self):
        mcu=FakeMcu()
        self.assertEqual(select_mcu(mcu,1,clock=lambda:mcu.now),'auv')
        self.assertEqual(sum(k==9 for k,p in mcu.sent),1)
        self.assertTrue(all(p[4]==0 for k,p in mcu.sent if k==2))
        self.assertTrue(all(k in (1,2,9) for k,p in mcu.sent))
    def test_old_firmware_not_selected(self):
        mcu=FakeMcu(capable=False)
        with self.assertRaises(RuntimeError):select_mcu(mcu,1,clock=lambda:mcu.now)
        self.assertFalse(any(k==9 for k,p in mcu.sent))
    def test_non_neutral_not_selected(self):
        mcu=FakeMcu(neutral=False)
        with self.assertRaises(RuntimeError):select_mcu(mcu,1,timeout=.5,clock=lambda:mcu.now)
        self.assertFalse(any(k==9 for k,p in mcu.sent))
    def test_rejection_or_wrong_status_blocks_handoff(self):
        for mcu in (FakeMcu(reject=True),FakeMcu(wrong_mode=True)):
            with self.assertRaises(RuntimeError):select_mcu(mcu,1,timeout=1,clock=lambda:mcu.now)


if __name__=='__main__':unittest.main()
