import json
import socket
import struct
import unittest
from unittest.mock import patch
from trial_protocol import encode, Parser, LeaseGate, manual_frame, centered
from trial_server import run, LevelCheck


class TrialTests(unittest.TestCase):
    def test_dual_mode_status_compatibility(self):
        from trial_protocol import decode_status
        payload=bytearray(46);payload[29]=8
        self.assertEqual(decode_status(payload)['operating_mode'],'fixed_profile')
        payload[4]=16
        self.assertEqual(decode_status(payload)['operating_mode'],'rov')
        payload[4]=48
        self.assertEqual(decode_status(payload)['operating_mode'],'auv')

    def test_manual_calibration_bridge_waits_for_ack_and_never_arms(self):
        self.check_level_action(True)

    def test_calibration_without_shore_confirmation_sends_no_command(self):
        self.check_level_action(False)

    def check_level_action(self, confirm):
        payload=bytearray(46);payload[29]=8
        struct.pack_into('<f',payload,17,.1);struct.pack_into('<f',payload,21,.05)
        class Uart:
            in_waiting=4096
            def __init__(self):self.packets=[];self.pending=[];self.count=0
            def read(self,n):
                self.count+=1;struct.pack_into('<I',payload,0,self.count)
                data=encode(0x80,payload)+b''.join(self.pending);self.pending=[]
                return data
            def write(self,data):
                self.packets.append(data);kind,body=Parser().push(data)[0]
                if kind==8:
                    payload[4]=8
                    struct.pack_into('<f',payload,17,0);struct.pack_into('<f',payload,21,0)
                    self.pending.append(encode(0x7f,bytes([8,0])+body[:4]))
                elif kind==2:self.pending.append(encode(0x7f,bytes([2,0])+body[:4]))
                return len(data)
        class Conn:
            def __init__(self):self.reply=None;self.sent=False;self.count=0;self.replies=[]
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def settimeout(self,*args):pass
            def setsockopt(self,*args):pass
            def sendall(self,data):self.reply=json.loads(data);self.replies.append(self.reply)
            def recv(self,n):
                self.count+=1
                if self.count>100:return b''
                if self.reply is None:raise socket.timeout()
                message={'sequence':self.count,'lease':self.reply['lease'],'deadman':False,
                    'frame':[0xa5,127,127,127,127,127,0,2,0,0,0]}
                if not self.sent:message.update(action='level',shore_confirmed=confirm);self.sent=True
                return (json.dumps(message)+'\n').encode()
        conn=Conn()
        class Server:
            seen=False
            def accept(self):
                if self.seen:raise StopIteration()
                self.seen=True;return conn,('test',1)
        clock=[0]
        def advance():clock[0]+=.05;return clock[0]
        uart=Uart()
        with patch('trial_server.time.monotonic',side_effect=advance):
            with self.assertRaises(StopIteration):run(uart,Server())
        messages=[Parser().push(p)[0] for p in uart.packets]
        self.assertEqual(sum(kind==8 for kind,_ in messages),int(confirm))
        self.assertTrue(all(kind!=2 or body[4]==0 for kind,body in messages))
        if confirm:
            self.assertTrue(any('校准完成' in r['detail'] and not r['level_pending'] for r in conn.replies))
            self.assertTrue(conn.replies[-1]['telemetry']['level_calibrated'])

    def test_level_requires_stable_distinct_disarm_samples(self):
        check=LevelCheck(0)
        for i in range(20):
            self.assertFalse(check.add({'sequence':i,'armed':False,'pitch_deg':3,'roll_deg':-4},i*.1))
        self.assertTrue(check.add({'sequence':20,'armed':False,'pitch_deg':3.1,'roll_deg':-4},2))
        with self.assertRaises(ValueError):
            LevelCheck(0).add({'armed':True,'pitch_deg':0,'roll_deg':0},0)
        check=LevelCheck(0)
        for i in range(20):check.add({'sequence':i,'pitch_deg':i*.1,'roll_deg':0},i*.1)
        with self.assertRaises(ValueError):check.add({'sequence':20,'pitch_deg':2,'roll_deg':0},2)
        check=LevelCheck(0)
        for i in range(30):self.assertFalse(check.add({'sequence':1,'pitch_deg':0,'roll_deg':0},i*.1))
        with self.assertRaises(ValueError):check.add({'sequence':1,'pitch_deg':0,'roll_deg':0},4.1)

    def test_crc_fragmentation_corruption_resync(self):
        packet = encode(5, b'\x01\0\0\0'+bytes([0xa5,127,127,127,127,127,0,2,0,0,0,1]))
        parser = Parser()
        self.assertEqual(parser.push(b'noise'+packet[:8]), [])
        self.assertEqual(parser.push(packet[8:]), [(5,packet[5:-2])])
        broken = bytearray(packet);broken[-1] ^= 1
        self.assertEqual(parser.push(broken+packet), [(5,packet[5:-2])])
        self.assertEqual(parser.errors, 1)

    def test_manual_camera_only_preserves_hold_switches(self):
        frame = manual_frame(bytes([0xa5,0,255,127,130,255,2,1,1,1,1]))
        self.assertEqual(frame[1:5], bytes([0,255,127,130]))
        self.assertEqual(frame[5:], bytes([255,2,1,1,1,1]))
        self.assertFalse(centered(frame))

    def test_camera_requires_middle_and_verified_dial(self):
        for selector in (0, 1, 2):
            for verified in (0, 1):
                frame = manual_frame(bytes([0xa5,127,127,127,127,255,0,selector,0,0,verified]))
                camera = selector == 1 and verified == 1
                self.assertEqual(frame[5], 255 if camera else 127)
                self.assertEqual(frame[7], selector)
                self.assertEqual(frame[10], int(camera))

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
