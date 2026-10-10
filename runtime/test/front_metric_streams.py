"""Front metric scheduling with mismatched rates and independent stalls; PTY only."""
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np
import yaml
from offline_replay import request
from startup import wait_for_socket


def scenario(binary, down_fps, front_fps):
    master, slave = pty.openpty()
    try:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cv2.imwrite(str(root / 'frame.jpg'), np.full((240, 320, 3), 210, np.uint8))
            fake = root / 'rpicam-vid'
            fake.write_text('''#!/usr/bin/python3
import pathlib,sys,time
root=pathlib.Path(__file__).parent
camera=sys.argv[sys.argv.index('--camera')+1]
fps=int(sys.argv[sys.argv.index('--framerate')+1])
frame=(root/'frame.jpg').read_bytes()
while True:
    if not (root/('stall-'+camera)).exists():
        sys.stdout.buffer.write(frame);sys.stdout.buffer.flush()
    time.sleep(1/fps)
''')
            fake.chmod(0o755)
            cfg = yaml.safe_load(Path('runtime/config/pi-auv-task-one.yaml').read_text())
            k = [250, 0, 160, 0, 250, 120, 0, 0, 1]
            d = [0, 0, 0, 0, 0]
            rotation = [0, 1, 0, 1, 0, 0, 0, 0, -1]
            for name, source, fps in [('camera', 'csi:0', down_fps), ('camera_front', 'csi:1', front_fps)]:
                cfg[name].update(source=source, enabled=True, width=320, height=240, fps=fps,
                                 camera_matrix=k, distortion_coefficients=d)
            cfg['serial']['device'] = os.ttyname(slave)
            cfg['control']['socket'] = str(root / 'control.sock')
            cfg['logging'].update(events=str(root / 'events.ndjson'), debug_dir=str(root / 'debug'))
            cfg['recording'].update(directory=str(root / 'recordings'), minimum_free_bytes=1024)
            cfg['video']['enabled'] = False
            cfg['web']['enabled'] = False
            cfg['operation'].update(mode='debug', auto_start=False, auto_arm=False)
            cfg['motion'].update(motion_commands_enabled=True, directions_calibrated=True, limits_calibrated=True)
            cfg['localization'].update(enabled=True, camera_matrix=k, distortion=d, camera_to_body=rotation,
                camera_offset_m=[0, 0, 0], depth_offset_m=[0, 0, 0], imu_signs=[1, 1, 1],
                imu_offsets_deg=[0, 0, 0], pool_depth_m=2, calibration_verified=True)
            cfg['observation_search'].update(enabled=True, corridor_calibrated=True,
                maximum_radius_m=2, depth_target_m=.3, waypoints_m=[[1, 0]])
            cfg['surface_traversal'].update(enabled=True, ascent_clearance_verified=True,
                center_approach_verified=True, center_approach_radius_cells=.3,
                surface_localization_verified=True, ascent_cell=[1, 1], cell_size_m=.5,
                boundary_clearance_m=.1, camera_matrix=[])
            cfg['front_metric'].update(enabled=True, camera_to_body=rotation,
                camera_offset_m=[0, 0, 0], depth_offset_m=[0, 0, 0])
            for phase in ['underwater', 'surface']:
                cfg['front_metric'][phase].update(verified=True, camera_matrix=k, distortion_coefficients=d)
            path = root / 'config.yaml'
            path.write_text(yaml.safe_dump(cfg))
            environment = dict(os.environ, PATH=str(root) + ':' + os.environ['PATH'])
            with (root / 'stderr.log').open('w') as errors:
                process = subprocess.Popen([binary, str(path)], env=environment,
                                           stdout=subprocess.DEVNULL, stderr=errors)
                try:
                    try:
                        wait_for_socket(process, root / 'control.sock')
                    except AssertionError as error:
                        raise AssertionError((root / 'stderr.log').read_text()) from error

                    def status():
                        value = json.loads(request(str(root / 'control.sock'), 'status'))
                        assert value['phase'] == 'INIT' and not value['armed']
                        assert value['metric_camera'] == 'front'
                        return value

                    deadline = time.monotonic() + 5
                    while status()['vision_frames'] < 5:
                        assert time.monotonic() < deadline, (root / 'stderr.log').read_text()
                        time.sleep(.1)
                    # Pause down less than the CSI restart threshold. Front
                    # metric processing must continue without any new down frame.
                    (root / 'stall-0').touch()
                    time.sleep(.25)
                    before = status()
                    time.sleep(.85)
                    after = status()
                    assert after['vision_frames'] > before['vision_frames'], (before, after)
                    assert after['down_capture_frames'] == before['down_capture_frames']
                    (root / 'stall-0').unlink()
                    (root / 'stall-1').touch()
                    time.sleep(.25)
                    before = status()
                    time.sleep(.85)
                    after = status()
                    assert after['down_capture_frames'] > before['down_capture_frames']
                    assert after['vision_frames'] == before['vision_frames'], (before, after)
                    assert after['front_camera_age_sec'] > cfg['safety']['frame_timeout_sec']
                    (root / 'stall-1').unlink()
                    deadline = time.monotonic() + 3
                    while status()['vision_frames'] <= after['vision_frames']:
                        assert time.monotonic() < deadline
                        time.sleep(.1)
                finally:
                    process.send_signal(signal.SIGTERM)
                    process.wait(timeout=5)
    finally:
        os.close(master)
        os.close(slave)


if __name__ == '__main__':
    scenario(sys.argv[1], 5, 20)
    scenario(sys.argv[1], 20, 5)
    print('front metric streams remain independent at 5/20 and 20/5 fps; stalls recover in DISARM')
