"""Read-only candidate geometry never certifies production or enables motion."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import yaml
from tag_docking_simulation import video
from offline_replay import request
from startup import wait_for_socket


def main(binary):
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        for role in ['front', 'down']:
            video(root/(role+'.avi'), (160, 120))
        cfg = yaml.safe_load(Path('runtime/config/pi-auv-tag-docking.yaml').read_text())
        k = [400, 0, 160, 0, 400, 120, 0, 0, 1]
        rotation = [0, -1, 0, -1, 0, 0, 0, 0, -1]
        for role, key in [('down', 'camera'), ('front', 'camera_front')]:
            cfg[key].update(source='file:'+str(root/(role+'.avi')), camera_matrix=k,
                            distortion_coefficients=[0]*5, fps=15)
        cfg['serial']['device'] = ''
        cfg['web']['enabled'] = False
        cfg['control']['socket'] = str(root/'control.sock')
        cfg['recording'].update(directory=str(root/'runs'), minimum_free_bytes=1048576)
        cfg['logging'].update(events=str(root/'events.ndjson'), debug_dir=str(root/'debug'))
        cfg['tag_docking'].update(geometry_verified=False, front_camera_to_body=rotation,
                                 down_camera_to_body=rotation, front_camera_offset_m=[0, 0, 0],
                                 down_camera_offset_m=[0, 0, 0])
        path = root/'config.yaml'
        path.write_text(yaml.safe_dump(cfg))
        process = subprocess.Popen([binary, str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        try:
            socket = root/'control.sock'
            wait_for_socket(process, socket)
            deadline = time.monotonic()+10
            while time.monotonic()<deadline:
                status = json.loads(request(str(socket), 'status'))
                d = status['tag_docking']
                if all(d['geometry_probe'][role]['valid'] for role in ['front', 'down']):
                    break
                time.sleep(.1)
            else:
                raise AssertionError(status)
            assert d['geometry_verified'] is False
            assert d['front_metric_valid'] is False and d['down_metric_valid'] is False
            assert status['motion_enabled'] is False and status['armed'] is False
            for role in ['front', 'down']:
                p = d['geometry_probe'][role]
                assert p['tag_body_m'][2] < 0 and p['reprojection_px'] <= 2
            assert request(str(socket), 'start').startswith('ERR')
            assert request(str(socket), 'arm SAFE_TO_ARM').startswith('ERR')
        finally:
            process.send_signal(signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
    print('read-only geometry probe isolation passed')


if __name__ == '__main__':
    main(sys.argv[1])
