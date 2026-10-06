"""CRC UART framing and short-lived Pi challenges for manual commissioning."""
import binascii
import math
import struct


def encode(kind, payload):
    if len(payload) > 64:
        raise ValueError("payload too long")
    body = bytes([1, kind, len(payload)]) + payload
    return b"\xaa\x55" + body + struct.pack('<H', binascii.crc_hqx(body, 0xffff))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def push(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            if self.buffer[:2] != b'\xaa\x55':
                if len(self.buffer) == 1 and self.buffer[0] == 0xaa:
                    break
                del self.buffer[0]
                continue
            if len(self.buffer) < 5:
                break
            length = self.buffer[4]
            if self.buffer[2] != 1 or length > 64:
                self.errors += 1
                del self.buffer[0]
                continue
            size = length + 7
            if len(self.buffer) < size:
                break
            body = bytes(self.buffer[2:size-2])
            if binascii.crc_hqx(body, 0xffff) != struct.unpack('<H', self.buffer[size-2:size])[0]:
                self.errors += 1
                del self.buffer[0]
                continue
            frames.append((self.buffer[3], bytes(self.buffer[5:size-2])))
            del self.buffer[:size]
        return frames


def manual_frame(frame):
    if len(frame) != 11 or frame[0] != 0xa5:
        raise ValueError('invalid RC frame')
    frame = bytearray(frame)
    frame[5], frame[6], frame[7] = 127, 0, 2
    frame[8:11] = b'\0\0\0'
    return bytes(frame)


def centered(frame):
    return all(115 <= x <= 139 for x in frame[1:5])


class LeaseGate:
    def __init__(self):
        self.leases = {}
        self.counter = 0
        self.sequence = -1

    def issue(self, now):
        self.counter += 1
        self.leases = {k: t for k, t in self.leases.items() if now-t <= .25}
        self.leases[self.counter] = now
        return self.counter

    def accept(self, message, now):
        sequence = message.get('sequence')
        lease = message.get('lease')
        if type(sequence) is not int or type(lease) is not int:
            return False
        if sequence <= self.sequence or lease not in self.leases or now-self.leases[lease] > .25:
            return False
        self.sequence = sequence
        return True


def decode_status(payload):
    if len(payload) != 46 or payload[29] != 8:
        raise ValueError('bad STATUS')
    def number(offset, degrees=False):
        value = struct.unpack_from('<f', payload, offset)[0]
        return value * (180/math.pi if degrees else 1) if math.isfinite(value) else None
    return {'sequence': struct.unpack_from('<I',payload)[0],
            'level_calibrated': bool(payload[4] & 8),
            'armed': bool(payload[4] & 1), 'failsafe': bool(payload[4] & 4),
            'error_flags': struct.unpack_from('<I', payload, 5)[0],
            'depth_m': number(13), 'roll_deg': number(17, True),
            'pitch_deg': number(21, True), 'yaw_deg': number(25, True),
            'outputs': [x/1000 for x in struct.unpack_from('<8h', payload, 30)]}
