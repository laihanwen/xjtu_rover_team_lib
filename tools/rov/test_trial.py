import json
import socket
import struct
import unittest
from trial_protocol import encode, Parser, LeaseGate, manual_frame, centered
from trial_server import run


class TrialTests(unittest.TestCase):
    def test_crc_fragmentation_corruption_resync(self):
        packet = encode(5, b'\x01\0\0\0'+bytes([0xa5,127,127,127,127,127,0,2,0,0,0,1]))
        parser = Parser()
        self.assertEqual(parser.push(b'noise'+packet[:8]), [])
        self.assertEqual(parser.push(packet[8:]), [(5,packet[5:-2])])
        broken = bytearray(packet);broken[-1] ^= 1
        self.assertEqual(parser.push(broken+packet), [(5,packet[5:-2])])
        self.assertEqual(parser.errors, 1)

    def test_manual_excludes_actuators_and_holds(self):
        frame = manual_frame(bytes([0xa5,0,255,127,130,255,2,1,1,1,1]))
        self.assertEqual(frame[1:5], bytes([0,255,127,130]))
        self.assertEqual(frame[5:], bytes([127,0,2,0,0,0]))
        self.assertFalse(centered(frame))

    def test_network_lease_rejects_stale_replay(self):
        gate = LeaseGate();lease = gate.issue(10)
        self.assertTrue(gate.accept({'sequence':1,'lease':lease},10.1))
        self.assertFalse(gate.accept({'sequence':1,'lease':lease},10.11))
        self.assertFalse(gate.accept({'sequence':2,'lease':lease},10.3))
        self.assertFalse(gate.accept({'sequence':2,'lease':999},10.1))

    def test_live_deadman_never_arms_without_operator(self):
        self.check_unarmed_input(False)

    def test_calibration_pulse_rejected_when_unarmed(self):
        self.check_unarmed_input(True)

    def check_unarmed_input(self, pulse):
        payload = bytearray(46);payload[29] = 8
        class Uart:
            in_waiting = 53
            def __init__(self):self.packets=[]
            def read(self,n):return encode(0x80,payload)
            def write(self,data):self.packets.append(data);return len(data)
        class Conn:
            def __init__(self):self.reply=None;self.count=0
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def settimeout(self,*args):pass
            def setsockopt(self,*args):pass
            def sendall(self,data):self.reply=json.loads(data)
            def recv(self,n):
                self.count+=1
                if self.count>20:return b''
                if self.reply is None:raise socket.timeout()
                return (json.dumps({'sequence':self.count,'lease':self.reply['lease'],
                    'action':'pulse' if pulse else None,'motor':0,'offset':2,
                    'deadman':True,'frame':[0xa5,127,127,127,127,127,0,2,0,0,0]})+'\n').encode()
        class Server:
            def __init__(self):self.seen=False
            def accept(self):
                if self.seen:raise StopIteration()
                self.seen=True;return Conn(),('test',1)
        uart=Uart()
        with self.assertRaises(StopIteration):run(uart,Server())
        messages=[Parser().push(p)[0] for p in uart.packets]
        self.assertTrue(messages)
        self.assertTrue(all(kind != 7 for kind, _ in messages))
        self.assertTrue(all(kind!=2 or payload[4]==0 for kind,payload in messages))


if __name__=='__main__':unittest.main()
