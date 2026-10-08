#!/usr/bin/env python3
"""Synthetic mission-one camera/serial replay with all motion disabled."""
import json
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
import threading
import time
from startup import wait_for_socket

import cv2
import numpy as np


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


def request(path, value):
    with socket.socket(socket.AF_UNIX) as peer:
        peer.connect(path)
        peer.sendall(value.encode())
        return peer.recv(8192).decode()


def make_video(path):
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 480))
    if not writer.isOpened():
        raise RuntimeError('MJPG VideoWriter unavailable')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, 7, 90)
    for _ in range(150):
        image = np.full((480, 640, 3), 210, np.uint8)
        cv2.rectangle(image, (80, 20), (560, 460), (0, 255, 255), 16)
        for x in (240, 400):
            cv2.line(image, (x, 20), (x, 460), (20, 20, 20), 7)
        for y in (167, 313):
            cv2.line(image, (80, y), (560, y), (20, 20, 20), 7)
        for x, y in [(160, 93), (480, 93)]:
            cv2.circle(image, (x, y), 36, (0, 80, 255), -1)
        for x, y in [(160, 386), (480, 386)]:
            cv2.rectangle(image, (x - 35, y - 35), (x + 35, y + 35), (0, 80, 255), -1)
        image[194:284, 275:365] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        writer.write(image)
    writer.release()


def main(binary):
    master, slave = pty.openpty()
    stop = threading.Event()
    emitted = bytearray()
    with tempfile.TemporaryDirectory() as temp:
        root = pathlib.Path(temp)
        video = root / 'scene.avi'
        make_video(video)
        config = pathlib.Path('runtime/test/runtime_fixture.yaml').read_text()
        config = config.replace('csi:0', f'file:{video}')
        config = config.replace('device: ""', f'device: "{os.ttyname(slave)}"')
        config = config.replace('/run/auv-runtime/control.sock', str(root / 'control.sock'))
        config = config.replace('/run/auv-runtime/hls', str(root / 'hls'))
        config = config.replace('/var/log/auv-runtime/events.ndjson', str(root / 'events.ndjson'))
        config = config.replace('/var/log/auv-runtime/debug', str(root / 'debug'))
        config = config.replace('/usr/local/share/auv-runtime/web', str(pathlib.Path('runtime/web').resolve()))
        config = config.replace('enabled: true', 'enabled: false')
        # Safety regression only; real Pi throughput is checked separately.
        config = config.replace('pose_timeout_sec: 0.5', 'pose_timeout_sec: 2.0')
        path = root / 'runtime.yaml'
        path.write_text(config)
        proc = subprocess.Popen([binary, str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        status = struct.pack('<IBIfffffB', 1, 0, 0, 12., 1., 0., 0., 0., 0)

        def stm32():
            while not stop.is_set():
                try:
                    os.write(master, frame(0x80, status))
                    readable, _, _ = select.select([master], [], [], .05)
                    if readable:
                        emitted.extend(os.read(master, 8192))
                except OSError:
                    return

        worker = threading.Thread(target=stm32, daemon=True)
        worker.start()
        try:
            wait_for_socket(proc, root / 'control.sock')
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                ready = json.loads(request(str(root / 'control.sock'), 'status'))
                if ready['serial'] and ready['telemetry_valid'] and ready['camera_age_sec'] >= 0:
                    break
                time.sleep(.05)
            else:
                raise AssertionError(f'camera or STM32 not ready: {ready}')
            assert request(str(root / 'control.sock'), 'start').startswith('OK')
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                state = json.loads(request(str(root / 'control.sock'), 'status'))
                if state['phase'] == 'VISIT_CONES':
                    break
                if state['phase'] == 'FAULT':
                    raise AssertionError(f"unexpected fault: {state['fault']}")
                time.sleep(.1)
            else:
                raise AssertionError(f"route not reached: {state}")
            assert state['apriltag_found'] and state['grid_complete']
            assert state['cone_count'] == 4 and state['route_waypoints'] > 0
            assert sorted(cell['object'] for cell in state['cells']).count('circle_cone') == 2
            assert sorted(cell['object'] for cell in state['cells']).count('square_cone') == 2
            assert not state['armed'] and not state['motion_enabled']
            assert b'\xaa\x55\x01\x03\x14' not in emitted
            assert list((root / 'debug').glob('map_*.jpg')), 'map debug image missing'
        finally:
            stop.set()
            proc.send_signal(signal.SIGTERM)
            proc.wait(timeout=5)
            worker.join(timeout=1)
            os.close(master)
            os.close(slave)
        events = [json.loads(line) for line in (root / 'events.ndjson').read_text().splitlines()]
        assert all(event['timestamp_unix_ms'] > 0 for event in events)
        assert any(event['event'] == 'PLAN' and 'path=' in event['detail'] for event in events)
    print('mission-one offline replay passed')


if __name__ == '__main__':
    main(sys.argv[1])
