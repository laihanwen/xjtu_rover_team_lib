"""Exclusive Pi TCP/CRC UART bridge. Never ARM without an operator action."""
import argparse
import json
import socket
import struct
import time
from trial_protocol import Parser, LeaseGate, encode, manual_frame, centered, decode_status, decode_pid


class LevelCheck:
    """Require two seconds of fresh, stable DISARM telemetry before zeroing."""
    def __init__(self, now):
        self.started = now
        self.samples = []
        self.sequence = None

    def add(self, telemetry, now):
        if telemetry.get('armed'):
            raise ValueError('水平校准只允许未ARM')
        angles = (telemetry.get('pitch_deg'), telemetry.get('roll_deg'))
        if any(x is None for x in angles):
            raise ValueError('校准IMU无效')
        if telemetry.get('sequence') != self.sequence:
            self.sequence = telemetry.get('sequence')
            if self.samples:
                angles = tuple(a + (x-a+180)%360-180 for a,x in zip(self.samples[0],angles))
            self.samples.append(angles)
        if now-self.started > 4:
            raise ValueError('校准采样超时')
        if now-self.started < 2 or len(self.samples) < 18:
            return False
        if any(max(s[i] for s in self.samples)-min(s[i] for s in self.samples) > .5 for i in (0,1)):
            raise ValueError('姿态不稳定，请在岸上放平后重新校准')
        return True


def run(uart, server):
    protocol = Parser()
    sequence = int(time.monotonic()*1000) & 0xffffffff
    telemetry = {}
    telemetry_ms = 0.0
    pid = {}
    pid_ms = 0.0
    def write(kind, payload):
        packet = encode(kind, payload)
        if uart.write(packet) != len(packet):
            raise OSError('incomplete UART write')
    def stop():
        write(6, b'\x01')
        write(2, struct.pack('<IB', sequence, 0))
    stop()
    while True:
        conn, peer = server.accept()
        print('Connected', peer, flush=True)
        gate = LeaseGate()
        frame = bytes([0xa5,127,127,127,127,127,0,2,0,0,0])
        deadman = False
        last_input = 0.0
        last_send = last_status = 0.0
        arm_pending = None
        level_check = None
        level_ack = None
        buffer = bytearray()
        detail = '等待手柄；未ARM'
        with conn:
            conn.settimeout(.01)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                stop()
                while True:
                    now = time.monotonic()
                    for kind, payload in protocol.push(uart.read(min(4096, uart.in_waiting))):
                        if kind == 0x80:
                            telemetry = decode_status(payload)
                            telemetry_ms = now
                            if level_check:
                                try:
                                    if level_check.add(telemetry,now):
                                        sequence = (sequence+1) & 0xffffffff
                                        write(8,struct.pack('<IB',sequence,1))
                                        level_ack = (sequence,now)
                                        level_check = None
                                        detail = '水平稳定，等待STM32校准响应'
                                except ValueError as e:
                                    level_check = None
                                    detail = str(e)
                        elif kind == 0x84:
                            pid = decode_pid(payload)
                            pid_ms = now
                        elif kind == 0x7f and len(payload) == 6 and payload[0] == 8:
                            if level_ack and struct.unpack_from('<I',payload,2)[0] == level_ack[0]:
                                detail = '岸上水平校准完成；本次开机有效' if payload[1] == 0 else 'STM32拒绝校准'
                                level_ack = None
                        elif kind == 0x7f and len(payload) == 6 and payload[0] == 2:
                            if payload[1] != 0:
                                detail = 'ARM/DISARM拒绝码 '+str(payload[1])
                    try:
                        data = conn.recv(4096)
                    except socket.timeout:
                        data = None
                    if data == b'':
                        break
                    if data:
                        buffer.extend(data)
                        if len(buffer) > 8192:
                            raise ValueError('input overflow')
                        while b'\n' in buffer:
                            line, _, remainder = buffer.partition(b'\n')
                            buffer = bytearray(remainder)
                            if len(line) > 512:
                                raise ValueError('oversized message')
                            message = json.loads(line)
                            if not gate.accept(message, now):
                                stop(); deadman = False; arm_pending = None
                                detail = '过期或重复指令已停止'
                                continue
                            values = message.get('frame')
                            if not isinstance(values, list) or len(values) != 11 or any(type(x) is not int or not 0<=x<=255 for x in values):
                                raise ValueError('bad frame')
                            frame = manual_frame(bytes(values))
                            if type(message.get('deadman')) is not bool:
                                raise ValueError('bad deadman')
                            deadman = message['deadman']
                            last_input = now
                            action = message.get('action')
                            if (level_check or level_ack) and not centered(frame):
                                level_check = level_ack = None
                                detail = '校准取消：摇杆未回中'
                            if action == 'level':
                                stop(); arm_pending = None; deadman = False
                                if (message.get('shore_confirmed') is not True or
                                    not centered(frame) or now-telemetry_ms > .3 or telemetry.get('armed')):
                                    detail = '请确认岸上放平、未ARM、摇杆回中及新鲜遥测'
                                elif level_check is None and level_ack is None:
                                    level_check = LevelCheck(now)
                                    detail = '岸上水平采样中，请保持静止2秒'
                            elif action == 'stop' or not deadman:
                                stop(); arm_pending = None
                                if action == 'stop':
                                    level_check = level_ack = None
                                if not level_check and not level_ack and action is not None:
                                    detail = '停止；需重新点击ARM'
                            elif action == 'arm':
                                if level_check or level_ack or not telemetry.get('level_calibrated') or not centered(frame) or now-telemetry_ms > .3 or telemetry.get('roll_deg') is None:
                                    detail = '拒绝ARM：摇杆未回中或遥测/IMU无效'
                                elif not telemetry.get('armed'):
                                    arm_pending = now
                                    detail = 'ARM准备中'
                            elif action == 'pulse_disabled':
                                motor, offset = message.get('motor'), message.get('offset')
                                if type(motor) is not int or type(offset) is not int or not 0 <= motor < 8 or not -75 <= offset <= 75:
                                    raise ValueError('bad pulse')
                                if telemetry.get('armed') and centered(frame) and now-telemetry_ms < .3:
                                    sequence = (sequence+1) & 0xffffffff
                                    write(7, struct.pack('<IBh', sequence, motor, offset))
                                    detail = '单路1秒测试已发送；随后自动回中'
                            elif action is not None:
                                raise ValueError('unknown action')
                    if now-last_input > .2 or now-telemetry_ms > .5:
                        if deadman or arm_pending is not None:
                            stop()
                        deadman = False; arm_pending = None
                        if level_check or level_ack:
                            level_check = level_ack = None
                            detail = '校准取消：输入或遥测失联'
                    if level_ack and now-level_ack[1] > 1:
                        level_ack = None
                        detail = '校准响应超时，请确认新固件已烧录'
                    if level_check or level_ack:
                        deadman = False; arm_pending = None
                    if now-last_send >= .05 and now-last_input <= .2:
                        last_send = now
                        sequence = (sequence+1) & 0xffffffff
                        write(5, struct.pack('<I', sequence)+frame+bytes([int(deadman)]))
                        write(1, struct.pack('<II', sequence, int(now*1000)&0xffffffff))
                        if arm_pending is not None:
                            if not deadman or not centered(frame):
                                stop(); arm_pending = None
                            elif now-arm_pending < .2:
                                write(6, b'\0')
                            else:
                                write(2, struct.pack('<IB', sequence, 1))
                                print('Operator ARM requested', flush=True)
                                arm_pending = None
                    if now-last_status >= .1:
                        last_status = now
                        reply = {'lease': gate.issue(now), 'telemetry': telemetry,
                                 'telemetry_age': now-telemetry_ms, 'detail': detail,
                                 'pid': pid, 'pid_age': now-pid_ms if pid else None,
                                 'level_pending': bool(level_check or level_ack),
                                 'uart_crc_errors': protocol.errors, 'pwm_limit': 125 if frame[6]==2 else 100,
                                 'servos_enabled': False, 'attitude_hold': True, 'heading_hold_enabled': True, 'pitch_pd': [7.5, 1.5], 'calibration_mode': False, 'start_offset_us': 48,
                                 'depth_hold': bool(pid.get('depth_hold_active')),
                                 'heading_hold': bool(pid.get('heading_hold_active')),
                                 'heading_requested': bool(pid.get('heading_requested')),
                                 'depth_requested': bool(pid.get('depth_requested'))}
                        conn.sendall((json.dumps(reply, allow_nan=False)+'\n').encode())
            except (OSError, ValueError, KeyError, TypeError) as error:
                print('Client stopped:', error, flush=True)
            finally:
                stop()
                print('Disconnected; DISARM', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument('--device', default='/dev/serial0')
    args = parser.parse_args()
    import serial
    with serial.Serial(args.device,115200,timeout=0,write_timeout=.1,exclusive=True) as uart:
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            server.bind((args.bind,args.port)); server.listen(1)
            run(uart, server)


if __name__ == '__main__':
    main()
