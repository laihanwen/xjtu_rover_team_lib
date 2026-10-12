"""Read-only observation in INIT/DISARM, using a synthetic file camera only."""
import base64
import json
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
from urllib.request import urlopen
import cv2
import numpy as np
import yaml


def main(binary):
    with tempfile.TemporaryDirectory() as directory:
        folder=Path(directory)
        image=np.full((240,320,3),(50,80,60),dtype=np.uint8)
        image[40:200,80:240]=255
        marker=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5),18,140)
        image[50:190,90:230]=cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
        video=folder/'tag.avi';writer=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'MJPG'),10,(320,240))
        assert writer.isOpened()
        for _ in range(100):writer.write(image)
        writer.release()
        with socket.socket() as sock:
            sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
        cfg=yaml.safe_load(Path('runtime/test/runtime_fixture.yaml').read_text())
        cfg['camera'].update(source='file:'+str(video),width=320,height=240,fps=10,camera_matrix=[],distortion_coefficients=[])
        cfg['camera_front']['enabled']=False;cfg['serial']['device']=''
        cfg['vision'].update(apriltag_family='tag16h5',apriltag_camera='down',apriltag_ids=[18])
        cfg['control']['socket']=str(folder/'control.sock')
        cfg['operation'].update(mode='debug',auto_start=False,auto_arm=False)
        cfg['motion']['motion_commands_enabled']=False
        cfg['logging'].update(events=str(folder/'events.ndjson'),debug_dir=str(folder/'debug'))
        cfg['video'].update(enabled=False,directory=str(folder/'hls'))
        cfg['web'].update(enabled=True,bind='127.0.0.1',port=port,assets=str(Path('runtime/web').resolve()))
        path=folder/'config.yaml';path.write_text(yaml.safe_dump(cfg))
        errors=(folder/'stderr').open('w')
        process=subprocess.Popen([binary,str(path)],stdout=subprocess.DEVNULL,stderr=errors)
        try:
            deadline=time.monotonic()+10
            def get(route):
                with urlopen(f'http://127.0.0.1:{port}'+route,timeout=1) as response:return json.load(response)
            while True:
                assert process.poll() is None,(folder/'stderr').read_text()
                try:perception=get('/api/perception')
                except OSError:perception={}
                if perception.get('tag_fresh') and any(t['id']==18 for t in perception.get('tags',[])) and perception.get('image_url'):break
                assert time.monotonic()<deadline,perception
                time.sleep(.1)
            assert perception['schema']=='auv.perception.v1' and perception['frame_fresh']
            assert not perception['pose_valid'] and not perception['tag_confirmed']
            assert all(c['type']=='unknown' for c in perception['cells'])
            jpeg=base64.b64decode(perception['image_url'].split(',',1)[1])
            assert cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR) is not None
            status=get('/api/status')
            assert not status['armed'] and not status['apriltag_found'] and status['phase']=='INIT',status
            print('INIT/DISARM: actual ID18 decoded by C++, JPEG and unknown map returned; no navigation or task trigger')
        finally:
            process.send_signal(signal.SIGTERM);process.wait(timeout=5);errors.close()


if __name__=='__main__':main(sys.argv[1])
