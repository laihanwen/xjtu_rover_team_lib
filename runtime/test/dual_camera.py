"""Two concurrent camera streams, CSI subprocess failure/recovery; no real devices."""
import json
import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time
import cv2
import numpy as np
from offline_replay import make_video, request
from startup import wait_for_socket


def main(binary, csi_down=False):
    with tempfile.TemporaryDirectory() as directory:
        root = pathlib.Path(directory)
        make_video(root / 'down.avi')
        cv2.imwrite(str(root / 'front.jpg'), np.full((240, 320, 3), (30, 220, 30), np.uint8))
        fake = root / 'rpicam-vid'
        fake.write_text('''#!/usr/bin/python3
import os, pathlib, sys, time
root=pathlib.Path(__file__).parent
(root/'child.pid').write_text(str(os.getpid()))
frame=(root/'front.jpg').read_bytes()
if (root/'stall').exists():
    time.sleep(20)
else:
    while True:
        sys.stdout.buffer.write(frame[:5]); sys.stdout.buffer.flush()
        sys.stdout.buffer.write(frame[5:]); sys.stdout.buffer.flush()
        time.sleep(0.067)
''')
        fake.chmod(0o755)
        config = pathlib.Path('runtime/test/runtime_fixture.yaml').read_text()
        replacements = {
            'csi:0': f'file:{root / "down.avi"}',
            'enabled: true': 'enabled: false',
            '/run/auv-runtime/control.sock': str(root / 'control.sock'),
            '/run/auv-runtime/hls': str(root / 'hls'),
            '/var/log/auv-runtime/events.ndjson': str(root / 'events.ndjson'),
            '/var/log/auv-runtime/debug': str(root / 'debug'),
        }
        for old, new in replacements.items():
            config = config.replace(old, new)
        config = config.replace('/dev/v4l/by-id/REPLACE_WITH_FRONT_USB_CAMERA', 'csi:1')
        config = config.replace('camera_front:\n  enabled: false', 'camera_front:\n  enabled: true')
        if csi_down:
            config=config.replace(f'file:{root / "down.avi"}', 'csi:0').replace('source: csi:1', f'source: file:{root / "down.avi"}')
        (root / 'runtime.yaml').write_text(config)
        environment = dict(os.environ, PATH=str(root) + ':' + os.environ['PATH'])
        with (root / 'stderr.log').open('w') as errors:
            process = subprocess.Popen([binary, str(root / 'runtime.yaml')], env=environment,
                                       stdout=subprocess.DEVNULL, stderr=errors)
            try:
                wait_for_socket(process, root / 'control.sock')
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    state = json.loads(request(str(root / 'control.sock'), 'status'))
                    if state['vision_frames'] >= 3 and state['front_frames'] >= 5:
                        break
                    time.sleep(.1)
                else:
                    raise AssertionError((state, (root / 'stderr.log').read_text()))
                assert not state['armed'] and not state['motion_enabled'] and not state['front_degraded']
                # Stall the replacement CSI process; USB vision and control stay alive.
                (root / 'stall').touch()
                os.kill(int((root / 'child.pid').read_text()), signal.SIGTERM)
                time.sleep(1.5)
                stalled = json.loads(request(str(root / 'control.sock'), 'status'))
                if csi_down:
                    assert stalled['camera_age_sec'] > .5
                    assert stalled['front_frames'] > state['front_frames']
                    assert not stalled['front_degraded']
                else:
                    assert stalled['front_degraded'] and stalled['front_camera_age_sec'] > .5
                assert stalled['control_ticks'] > state['control_ticks']
                if not csi_down:
                    assert stalled['vision_frames'] > state['vision_frames']
                assert not stalled['armed'] and not stalled['motion_enabled']
                child = int((root / 'child.pid').read_text())
                deadline = time.monotonic() + 12
                while time.monotonic() < deadline:
                    replacement = int((root / 'child.pid').read_text())
                    if replacement != child:
                        break
                    time.sleep(.1)
                else:
                    raise AssertionError('stalled CSI process was not restarted')
                (root / 'stall').unlink()
                os.kill(replacement, signal.SIGTERM)
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    recovered = json.loads(request(str(root / 'control.sock'), 'status'))
                    ready=(recovered['camera_age_sec'] < .5 and recovered['down_capture_frames'] > stalled['down_capture_frames']) if csi_down else (not recovered['front_degraded'] and recovered['front_frames'] > stalled['front_frames'])
                    if ready:
                        break
                    time.sleep(.1)
                else:
                    raise AssertionError(('CSI did not recover', recovered))
                child = int((root / 'child.pid').read_text())
            finally:
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=5)
            assert not pathlib.Path(f'/proc/{child}').exists(), 'CSI child left running'
    print('dual camera capture, stall isolation and child cleanup passed')


if __name__ == '__main__':
    main(sys.argv[1])
    main(sys.argv[1], csi_down=True)
