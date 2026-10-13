#!/usr/bin/env python3
"""One-shot boot route against virtual cameras and STM32 PTY; no physical I/O."""
import json
import os
from pathlib import Path
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

import cv2
import numpy as np
import yaml

from offline_replay import crc, frame
from startup import wait_for_socket


def request(path, command):
    with socket.socket(socket.AF_UNIX) as peer:
        peer.connect(path)
        peer.sendall(command.encode())
        chunks = []
        while chunk := peer.recv(4096):
            chunks.append(chunk)
        return b''.join(chunks).decode()


def video(path, tag_after=None):
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 15, (320, 240))
    assert writer.isOpened()
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5), 18, 72)
    image = np.full((240, 320, 3), 180, dtype=np.uint8)
    for row in range(12, 240, 28):
        for col in range(12, 320, 28):
            cv2.circle(image, (col, row), 3, (110, 110, 110), -1)
    for index in range(360):
        frame_image = image.copy()
        if tag_after is not None and index >= tag_after:
            frame_image[75:163, 116:204] = 255
            frame_image[83:155, 124:196] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        writer.write(frame_image)
    writer.release()


def wait_status(path, predicate, timeout):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = json.loads(request(path, 'status'))
        if predicate(last):
            return last
        time.sleep(.08)
    raise AssertionError(f'status timeout: {last}')


def main(binary):
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    assert device.startswith('/dev/pts/')
    stop = threading.Event()
    state = dict(armed=False, depth=0.0, status_sequence=0, pressure_sequence=0)
    arm_requests = []
    targets = []
    with tempfile.TemporaryDirectory(prefix='auv-boot-route-') as directory:
        root = Path(directory)
        down, front = root/'down.avi', root/'front.avi'
        video(down)
        video(front, tag_after=250)
        cfg = yaml.safe_load(Path('runtime/config/pi-auv-boot-tag-route.yaml').read_text())
        for camera, path in (('camera', down), ('camera_front', front)):
            cfg[camera].update(source=f'file:{path}', fps=15,
                camera_matrix=[250, 0, 160, 0, 250, 120, 0, 0, 1],
                distortion_coefficients=[0]*5,
                calibration_quality='synthetic_test_only')
        cfg['serial']['device'] = device
        cfg['control']['socket'] = str(root/'control.sock')
        cfg['operation'].update(startup_delay_sec=.1, startup_timeout_sec=35)
        cfg['localization']['enabled'] = False  # Exercise the documented open-loop fallback.
        cfg['recording'].update(directory=str(root/'runs'), minimum_free_bytes=1048576)
        cfg['logging'].update(events=str(root/'events.ndjson'), debug_dir=str(root/'evidence'))
        cfg['video']['enabled'] = False
        cfg['video']['directory'] = str(root/'hls')
        cfg['web']['enabled'] = False
        path = root/'route.yaml'
        path.write_text(yaml.safe_dump(cfg))
        subprocess.run([binary, '--check-config', str(path)], check=True,
                       stdout=subprocess.DEVNULL)

        def emulator():
            pending = bytearray()
            last_pressure = 0.0
            while not stop.is_set():
                try:
                    state['status_sequence'] += 1
                    sequence = state['status_sequence']
                    os.write(master, frame(0x80, struct.pack('<IBIfffffB',
                        sequence, int(state['armed']), 0, 12., state['depth'], 0., 0., 0., 0)))
                    now = time.monotonic()
                    if now-last_pressure >= .5:
                        last_pressure = now
                        state['pressure_sequence'] += 1
                        os.write(master, frame(0x82, struct.pack('<IfBII', sequence,
                            state['depth'], 1, state['pressure_sequence'], 0)))
                    ready, _, _ = select.select([master], [], [], .04)
                    if ready:
                        pending.extend(os.read(master, 8192))
                    while len(pending) >= 7:
                        if pending[:2] != b'\xaa\x55':
                            del pending[0]
                            continue
                        length = 7 + pending[4]
                        if len(pending) < length:
                            break
                        packet = bytes(pending[:length]); del pending[:length]
                        if crc(packet[2:-2]) != struct.unpack('<H', packet[-2:])[0]:
                            continue
                        kind, data = packet[3], packet[5:-2]
                        if kind == 2:
                            state['armed'] = bool(data[4])
                            arm_requests.append(state['armed'])
                            os.write(master, frame(0x7f, bytes([2, 0])+data[:4]))
                        elif kind == 3:
                            target = struct.unpack('<Iffff', data)
                            targets.append(target)
                            if state['armed']:
                                difference = target[3]-state['depth']
                                state['depth'] += max(-.02, min(.02, difference))
                        elif kind == 4:
                            os.write(master, frame(0x7f, bytes([4, 0])+data[:4]))
                except OSError:
                    return

        worker = threading.Thread(target=emulator, daemon=True)
        worker.start()
        proc = None
        try:
            proc = subprocess.Popen([binary, str(path)], stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL)
            socket_path = str(root/'control.sock')
            wait_for_socket(proc, root/'control.sock')
            initial = wait_status(socket_path, lambda s: s['serial'] and
                                  s['down_capture_frames'] > 0 and s['front_frames'] > 0, 5)
            assert initial['phase'] == 'DEPTH_STABLE_WAIT' and True not in arm_requests, (
                initial['phase'], initial['fault'], arm_requests)
            for command in ('start', 'arm SAFE_TO_ARM', 'pause', 'resume', 'abort', 'reset'):
                assert request(socket_path, command).startswith('ERR'), command
            state['depth'] = .5
            wet_since = time.monotonic()
            wait_status(socket_path, lambda s: s['phase'] in ('WAIT_ARM', 'ASCEND_0.2M',
                        'FORWARD', 'RETURN', 'ARRIVED_HOLD', 'DONE'), 16)
            assert time.monotonic()-wet_since >= 10, 'started before ten stable seconds'
            finished = wait_status(socket_path, lambda s: s['phase'] in ('DONE', 'FAULT'), 24)
            assert finished['phase'] == 'DONE', finished['fault']
            assert arm_requests.count(True) == 1, arm_requests
            wait_status(socket_path, lambda s: not s['armed'], 2)
            assert arm_requests[-1] is False
            assert any(target[1] > 0 for target in targets), 'no forward command'
            assert any(target[1] < 0 for target in targets), 'no reverse command'
            assert all(abs(target[1]) <= .04001 and abs(target[2]) < 1e-6 for target in targets)
            run = Path(finished['recording_directory'])
            events = [json.loads(line) for line in (run/'events.ndjson').read_text().splitlines()]
            names = [event['event'] for event in events]
            ordered = ('DEPTH_STABLE', 'ARM_REQUEST', 'ARM_CONFIRMED', 'ASCEND_START',
                       'ASCEND_DONE', 'FORWARD_START', 'FORWARD_END', 'TAG_SEEN',
                       'RETURN_START', 'RETURN_OPEN_LOOP', 'RETURN_END', 'HOLD_STABLE', 'RUN_END')
            positions = [names.index(name) for name in ordered]
            assert positions == sorted(positions), names
            tag_events = [e['detail'] for e in events if e['event'] == 'TAG_SEEN']
            assert any('id=18' in detail and 'pose_valid=' in detail and
                       ('tag_body_x_m=' in detail or 'pose_unavailable=true' in detail)
                       for detail in tag_events), tag_events
            assert any('tag' in e['detail'] for e in events if e['event'] == 'FORWARD_END')
            assert any('open_loop' in e['detail'] for e in events if e['event'] == 'RETURN_START')
            assert any('task_result=complete' in e['detail'] for e in events if e['event'] == 'RUN_END')
            assert list(run.glob('down-*.mjpg')) and list(run.glob('front-*.mjpg'))
            assert list(run.glob('down-*.frames.jsonl')) and list(run.glob('front-*.frames.jsonl'))
            before = finished['down_capture_frames']
            time.sleep(.3)
            after = json.loads(request(socket_path, 'status'))
            assert proc.poll() is None and after['down_capture_frames'] > before
            assert after['recording_ready'] and not (run/'recording-complete').exists()
            assert (root/'control.sock.autonomous-started').exists()
            proc.send_signal(signal.SIGTERM)
            proc.wait(timeout=5)
            state['depth'] = .5  # Present the same stable in-water condition after service restart.
            proc = subprocess.Popen([binary, str(path)], stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL)
            wait_for_socket(proc, root/'control.sock')
            blocked = wait_status(socket_path, lambda s: s['phase'] == 'FAULT', 16)
            assert 'already started since boot' in blocked['fault'], blocked
            assert arm_requests.count(True) == 1, arm_requests
        finally:
            if proc and proc.poll() is None:
                proc.send_signal(signal.SIGTERM)
                proc.wait(timeout=5)
            stop.set()
            worker.join(timeout=2)
            os.close(master)
            os.close(slave)
    print('Boot tag route virtual cameras/STM32, recording and restart latch passed')


if __name__ == '__main__':
    main(sys.argv[1])
