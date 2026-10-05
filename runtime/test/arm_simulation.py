#!/usr/bin/env python3
"""Virtual STM32 ARM/ACK and motion gate test; PTY only, no hardware device."""
import json
import os
import pathlib
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
from offline_replay import crc, frame, make_video, request


def main(binary):
    master, slave = pty.openpty()
    stop = threading.Event()
    targets = []
    arm_messages = []
    with tempfile.TemporaryDirectory() as temp:
        root = pathlib.Path(temp)
        video = root / 'scene.avi'
        make_video(video)
        config = pathlib.Path('runtime/config/runtime.yaml').read_text()
        replacements = {
            '/dev/v4l/by-id/REPLACE_WITH_REAL_CAMERA': f'file:{video}',
            'device: ""': f'device: "{os.ttyname(slave)}"',
            '/run/auv-runtime/control.sock': str(root / 'control.sock'),
            '/run/auv-runtime/hls': str(root / 'hls'),
            '/var/log/auv-runtime/events.ndjson': str(root / 'events.ndjson'),
            '/var/log/auv-runtime/debug': str(root / 'debug'),
            '/usr/local/share/auv-runtime/web': str(pathlib.Path('runtime/web').resolve()),
            'enabled: true': 'enabled: false',
            'motion_commands_enabled: false': 'motion_commands_enabled: true',
            'directions_calibrated: false': 'directions_calibrated: true',
            'limits_calibrated: false': 'limits_calibrated: true',
            'camera_matrix: []': 'camera_matrix: [600, 0, 320, 0, 600, 240, 0, 0, 1]',
            'distortion_coefficients: []': 'distortion_coefficients: [0, 0, 0, 0, 0]',
            'surge_from_row: 0.0': 'surge_from_row: 0.1',
            'sway_from_col: 0.0': 'sway_from_col: 0.1',
            # This test checks serial/ARM safety, not the Pi vision throughput gate.
            'pose_timeout_sec: 0.5': 'pose_timeout_sec: 2.0',
        }
        for old, new in replacements.items():
            config = config.replace(old, new)
        path = root / 'runtime.yaml'
        path.write_text(config)
        proc = subprocess.Popen([binary, str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        state = {'armed': False, 'leak': False, 'sequence': 0}

        def emulator():
            incoming = bytearray()
            while not stop.is_set():
                try:
                    state['sequence'] += 1
                    flags = int(state['armed']) | (2 if state['leak'] else 0)
                    payload = struct.pack('<IBIfffffB', state['sequence'], flags, 0,
                                          12., 1., 0., 0., 0., 0)
                    os.write(master, frame(0x80, payload))
                    ready, _, _ = select.select([master], [], [], .05)
                    if ready:
                        incoming.extend(os.read(master, 8192))
                    while len(incoming) >= 7:
                        if incoming[:2] != b'\xaa\x55':
                            del incoming[0]
                            continue
                        length = 7 + incoming[4]
                        if len(incoming) < length:
                            break
                        packet = bytes(incoming[:length])
                        del incoming[:length]
                        if crc(packet[2:-2]) != struct.unpack('<H', packet[-2:])[0]:
                            continue
                        kind, data = packet[3], packet[5:-2]
                        if kind == 2:
                            state['armed'] = bool(data[4])
                            arm_messages.append(state['armed'])
                            os.write(master, frame(0x7f, bytes([2, 0]) + data[:4]))
                        elif kind == 3:
                            targets.append(struct.unpack('<Iffff', data))
                except OSError:
                    return

        worker = threading.Thread(target=emulator, daemon=True)
        worker.start()
        try:
            for _ in range(100):
                if (root / 'control.sock').exists():
                    break
                time.sleep(.01)
            for _ in range(100):
                preflight = json.loads(request(str(root / 'control.sock'), 'status'))
                if preflight['serial'] and preflight['camera_age_sec'] >= 0:
                    break
                time.sleep(.02)
            assert request(str(root / 'control.sock'), 'start').startswith('OK')
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                snapshot = json.loads(request(str(root / 'control.sock'), 'status'))
                if snapshot['phase'] == 'VISIT_CONES':
                    break
                if snapshot['phase'] == 'FAULT':
                    raise AssertionError(snapshot['fault'])
                time.sleep(.1)
            else:
                raise AssertionError(f'no route: {snapshot}')
            assert request(str(root / 'control.sock'), 'arm SAFE_TO_ARM').startswith('OK')
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline and not any(abs(t[1]) > .001 or abs(t[2]) > .001 for t in targets):
                time.sleep(.05)
            assert any(abs(t[1]) > .001 or abs(t[2]) > .001 for t in targets), (
                'no bounded motion target', targets, request(str(root / 'control.sock'), 'status'))
            assert all(abs(t[1]) <= .10001 and abs(t[2]) <= .10001 for t in targets)
            moving = [t for t in targets if abs(t[1]) > .001 or abs(t[2]) > .001]
            assert all(t[3] == 1.0 and t[4] == 0.0 for t in moving)
            assert request(str(root / 'control.sock'), 'pause').startswith('OK')
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline and state['armed']:
                time.sleep(.02)
            assert not state['armed']
            assert json.loads(request(str(root / 'control.sock'), 'status'))['phase'] == 'PAUSED'
            assert request(str(root / 'control.sock'), 'resume').startswith('OK')
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                snapshot = json.loads(request(str(root / 'control.sock'), 'status'))
                if not snapshot['armed'] and snapshot['phase'] == 'VISIT_CONES':
                    break
                time.sleep(.02)
            assert request(str(root / 'control.sock'), 'arm SAFE_TO_ARM').startswith('OK')
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline and not state['armed']:
                time.sleep(.02)
            assert state['armed']
            assert request(str(root / 'control.sock'), 'disarm').startswith('OK')
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline and arm_messages[-1] is not False:
                time.sleep(.02)
            assert arm_messages[-1] is False, 'DISARM not transmitted'
            count = len(targets)
            time.sleep(.2)
            assert all(t[1] == 0. and t[2] == 0. for t in targets[count:]), 'motion persisted after DISARM'
            state['leak'] = True
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                snapshot = json.loads(request(str(root / 'control.sock'), 'status'))
                if snapshot['phase'] == 'FAULT':
                    break
                time.sleep(.02)
            assert snapshot['phase'] == 'FAULT' and snapshot['leak'], f'leak did not fault mission: {snapshot}'
            state['leak'] = False
            time.sleep(.1)
            assert request(str(root / 'control.sock'), 'reset').startswith('OK')
            assert request(str(root / 'control.sock'), 'start').startswith('OK')
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                snapshot = json.loads(request(str(root / 'control.sock'), 'status'))
                if snapshot['phase'] == 'VISIT_CONES':
                    break
                if snapshot['phase'] == 'FAULT':
                    raise AssertionError(snapshot['fault'])
                time.sleep(.05)
            else:
                raise AssertionError('route did not recover after explicit reset/start')
            assert request(str(root / 'control.sock'), 'arm SAFE_TO_ARM').startswith('OK')
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline and not state['armed']:
                time.sleep(.02)
            assert state['armed'], (arm_messages, request(str(root / 'control.sock'), 'status'))
            proc.send_signal(signal.SIGTERM)
            proc.wait(timeout=5)
            time.sleep(.1)
            assert arm_messages[-1] is False, 'process exit did not request DISARM'
        finally:
            stop.set()
            if proc.poll() is None:
                proc.send_signal(signal.SIGTERM)
                proc.wait(timeout=5)
            worker.join(timeout=1)
            os.close(master)
            os.close(slave)
    print('virtual ARM simulation passed')


if __name__ == '__main__':
    main(sys.argv[1])
