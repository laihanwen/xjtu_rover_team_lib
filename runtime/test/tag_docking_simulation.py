"""Virtual cameras and STM32 PTY only: never opens a physical control device."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time

import cv2
import numpy as np
import yaml
from offline_replay import crc, frame, request
from startup import wait_for_socket


def video(path, center):
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 15, (320, 240))
    assert writer.isOpened()
    rng = np.random.default_rng(18)
    # Trackable floor texture without pixel noise producing thousands of false
    # tag contours. Keep more than the odometry's 30 required feature points.
    image = np.full((240, 320, 3), 180, dtype=np.uint8)
    for y0 in range(12, 240, 24):
        for x0 in range(12, 320, 24):
            shade = int(rng.integers(80, 140))
            cv2.circle(image, (x0, y0), 3, (shade, shade, shade), -1)
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5), 18, 70)
    x, y = center[0]-35, center[1]-35
    image[y-8:y+78, x-8:x+78] = 255
    image[y:y+70, x:x+70] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    for _ in range(150):
        writer.write(image)
    writer.release()


def main(binary):
    master, slave = pty.openpty()
    stop = threading.Event()
    state = {'armed': False, 'depth': .60, 'sequence': 0}
    arm_requests, targets = [], []
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        down, front = root/'down.avi', root/'front.avi'
        video(down, (160, 120))
        video(front, (160, 100))
        cfg = yaml.safe_load(Path('runtime/config/pi-auv-tag-docking.yaml').read_text())
        k = [250, 0, 160, 0, 250, 120, 0, 0, 1]
        rotation = [0, -1, 0, -1, 0, 0, 0, 0, -1]
        for camera, path in [('camera', down), ('camera_front', front)]:
            cfg[camera].update(source=f'file:{path}', camera_matrix=k,
                distortion_coefficients=[0]*5, calibration_quality='independently_verified', fps=15)
        cfg['serial']['device'] = os.ttyname(slave)
        cfg['control']['socket'] = str(root/'control.sock')
        cfg['recording'].update(directory=str(root/'runs'), minimum_free_bytes=1048576)
        cfg['logging'].update(events=str(root/'events.ndjson'), debug_dir=str(root/'evidence'))
        cfg['video']['directory'] = str(root/'hls')
        cfg['motion'].update(motion_commands_enabled=True, directions_calibrated=True, limits_calibrated=True)
        cfg['localization'].update(camera_matrix=k, distortion=[0]*5, camera_to_body=rotation,
            camera_offset_m=[0, 0, 0], depth_offset_m=[0, 0, 0], imu_signs=[1, 1, 1],
            imu_offsets_deg=[0, 0, 0], pool_depth_m=2, calibration_verified=True)
        # Synthetic rig has two parallel downward cameras; real rig calibration is never altered.
        cfg['tag_docking'].update(geometry_verified=True, corridor_verified=True, depth_verified=True,
            front_camera_to_body=rotation, down_camera_to_body=rotation,
            front_camera_offset_m=[0, 0, 0], down_camera_offset_m=[0, 0, 0],
            depth_targets_m=[.61, .60, .605], depth_rate_mps=.1, depth_hold_sec=.2,
            maximum_radius_m=2, center_hold_sec=.3)
        path = root/'config.yaml'
        path.write_text(yaml.safe_dump(cfg))
        subprocess.run([binary, '--check-config', str(path)], check=True)

        def emulator():
            pending = bytearray()
            while not stop.is_set():
                try:
                    state['sequence'] += 1
                    seq = state['sequence']
                    os.write(master, frame(0x80, struct.pack('<IBIfffffB', seq, int(state['armed']),
                        0, 12., state['depth'], 0., 0., 0., 0)))
                    os.write(master, frame(0x82, struct.pack('<IfBII', seq, state['depth'], 1, seq, 0)))
                    readable, _, _ = select.select([master], [], [], .03)
                    if readable:
                        pending.extend(os.read(master, 8192))
                    while len(pending) >= 7:
                        if pending[:2] != b'\xaa\x55':
                            del pending[0]
                            continue
                        length = 7+pending[4]
                        if len(pending) < length:
                            break
                        packet = bytes(pending[:length]); del pending[:length]
                        if crc(packet[2:-2]) != struct.unpack('<H', packet[-2:])[0]:
                            continue
                        kind, data = packet[3], packet[5:-2]
                        if kind == 2:
                            state['armed'] = bool(data[4]); arm_requests.append(state['armed'])
                            os.write(master, frame(0x7f, bytes([2, 0])+data[:4]))
                        elif kind == 3:
                            value = struct.unpack('<Iffff', data); targets.append(value)
                            if state['armed']:
                                state['depth'] = value[3]
                        elif kind == 4:
                            os.write(master, frame(0x7f, bytes([4, 0])+data[:4]))
                except OSError:
                    return

        proc = subprocess.Popen([binary, str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        worker = threading.Thread(target=emulator, daemon=True); worker.start()
        try:
            socket = str(root/'control.sock')
            wait_for_socket(proc, Path(socket))
            deadline = time.monotonic()+12
            while time.monotonic() < deadline:
                status = json.loads(request(socket, 'status'))
                if status['origin_ready'] and status['recording_ready'] and status['tag_docking']['front_metric_valid']:
                    break
                assert not status['fault'], status
                time.sleep(.1)
            else:
                raise AssertionError(f'preflight failed: {status}')
            assert True not in arm_requests
            response=''
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                response=request(socket,'start')
                if response.startswith('OK'):break
                time.sleep(.05)
            assert response.startswith('OK'), (response,json.loads(request(socket,'status')))
            response=request(socket, 'arm SAFE_TO_ARM')
            assert response.startswith('OK'), (response,json.loads(request(socket,'status')))
            phases = set()
            deadline = time.monotonic()+15
            while time.monotonic() < deadline:
                status = json.loads(request(socket, 'status')); phases.add(status['phase'])
                assert not status['fault'], status
                if status['phase'] == 'HOVER' and status['tag_docking']['completion_evidence_saved'] and state['armed']:
                    break
                time.sleep(.04)
            else:
                raise AssertionError(f'task failed: {status}')
            assert 'DEPTH_TEST' in phases and 'DOWN_CENTER' in phases, phases
            run = Path(status['recording_directory'])
            events = [json.loads(s) for s in (run/'events.ndjson').read_text().splitlines()]
            assert sum(e['event'] == 'DEPTH_TEST_REPORT' for e in events) == 3
            assert (run/'evidence/tag18-down-centered.jpg').exists()
            # Continue beyond acquisition: the robot remains armed and receives
            # depth/yaw hold targets until an explicit operator stop.
            before=len(targets)
            time.sleep(.5)
            status=json.loads(request(socket,'status'))
            assert status['phase']=='HOVER' and state['armed'] and len(targets)>before, status
            assert arm_requests[-1] is True and targets
            assert all(abs(t[1]) <= .08001 and abs(t[2]) <= .08001 and t[3] <= 1.2 for t in targets)
            assert request(socket, 'reset').startswith('ERR')
            assert request(socket, 'start').startswith('ERR')
            assert request(socket,'abort').startswith('OK')
            deadline=time.monotonic()+2
            while state['armed'] and time.monotonic()<deadline:
                time.sleep(.02)
            assert not state['armed'] and arm_requests[-1] is False
        finally:
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait()
            stop.set(); worker.join(timeout=2)
            os.close(master); os.close(slave)
        print('Tag docking virtual runtime passed; no physical devices used')


if __name__ == '__main__':
    main(sys.argv[1])
