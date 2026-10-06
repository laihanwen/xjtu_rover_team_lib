#!/usr/bin/env python3
"""Dry serial integration test. Never sends an ARM command to the emulator."""
import os
import pathlib
import pty
import select
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
from startup import wait_for_socket


def crc(data):
    value = 0xffff
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xffff
    return value


def frame(kind, payload):
    body = bytes([1, kind, len(payload)]) + payload
    return b'\xaa\x55' + body + struct.pack('<H', crc(body))


def command(path, text):
    with socket.socket(socket.AF_UNIX) as peer:
        peer.connect(path)
        peer.sendall(text.encode())
        return peer.recv(4096).decode()


def main(binary):
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        config = pathlib.Path('runtime/config/runtime.yaml').read_text()
        config = config.replace('/dev/v4l/by-id/REPLACE_WITH_REAL_CAMERA', '/dev/v4l/by-id/NO_CAMERA')
        config = config.replace('device: ""', f'device: "{device}"')
        config = config.replace('/run/auv-runtime/control.sock', str(root / 'control.sock'))
        config = config.replace('/run/auv-runtime/hls', str(root / 'hls'))
        config = config.replace('/var/log/auv-runtime/events.ndjson', str(root / 'events.ndjson'))
        config = config.replace('/var/log/auv-runtime/debug', str(root / 'debug'))
        config = config.replace('/usr/local/share/auv-runtime/web', str(pathlib.Path('runtime/web').resolve()))
        config = config.replace('enabled: true', 'enabled: false')
        path = root / 'runtime.yaml'
        path.write_text(config)
        proc = subprocess.Popen([binary, str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            wait_for_socket(proc, root / 'control.sock')
            received = bytearray()
            status = struct.pack('<IBIfffffB', 1, 0, 0, 12., 1., 0., 0., 0., 0)
            assert len(status) == 30
            os.write(master, frame(0x80, status)[:-1] + b'\x00')  # bad CRC
            time.sleep(.1)
            assert '"serial":false' in command(str(root / 'control.sock'), 'status')
            invalid_depth = struct.pack('<IBIfffffB', 2, 0, 0, 12., float('nan'), 0., 0., 0., 0)
            os.write(master, frame(0x80, invalid_depth))
            time.sleep(.1)
            reported = command(str(root / 'control.sock'), 'status')
            assert '"serial":true' in reported and '"telemetry_valid":false' in reported
            for _ in range(12):
                os.write(master, frame(0x80, status))
                ready, _, _ = select.select([master], [], [], .05)
                if ready:
                    received.extend(os.read(master, 4096))
            assert '"serial":true' in command(str(root / 'control.sock'), 'status')
            assert received.count(b'\xaa\x55\x01\x01\x08') >= 5, 'heartbeat below expected rate'
            assert b'\xaa\x55\x01\x02\x05' in received, 'startup DISARM missing'
            assert b'\xaa\x55\x01\x03\x14' not in received, 'unexpected motion target'
            assert command(str(root / 'control.sock'), 'arm SAFE_TO_ARM').startswith('ERR')
            armed_status = bytearray(status)
            armed_status[4] = 1
            received.clear()
            for _ in range(5):
                os.write(master, frame(0x80, armed_status))
                ready, _, _ = select.select([master], [], [], .05)
                if ready:
                    received.extend(os.read(master, 4096))
            assert b'\xaa\x55\x01\x02\x05' in received, 'unsolicited ARM was not withdrawn'
            marker = received.find(b'\xaa\x55\x01\x03\x14')
            assert marker >= 0, 'neutral target missing before DISARM'
            assert received[marker + 9:marker + 17] == b'\x00' * 8, 'nonzero velocity emitted'
            assert command(str(root / 'control.sock'), 'disarm').startswith('OK')
            deadline = time.monotonic() + .5
            while time.monotonic() < deadline:
                ready, _, _ = select.select([master], [], [], .05)
                if ready:
                    received.extend(os.read(master, 4096))
            actuator = received.find(b'\xaa\x55\x01\x04\x09')
            assert actuator >= 0, 'gripper STOP missing on DISARM'
            assert received[actuator + 9] == 1
            assert received[actuator + 10:actuator + 14] == b'\x00' * 4
            time.sleep(1.2)
            state = command(str(root / 'control.sock'), 'status')
            assert 'STATUS timeout' in state and '"phase":"FAULT"' in state
        finally:
            proc.send_signal(signal.SIGTERM)
            proc.wait(timeout=5)
            os.close(master)
            os.close(slave)
    print('runtime PTY safety test passed')


if __name__ == '__main__':
    main(sys.argv[1])
