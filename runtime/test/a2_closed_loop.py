"""Full Runtime closed loop with synthetic cameras and a virtual MCU PTY only.

Images are projected from the simulated body position; motion is integrated
only from decoded Runtime UART targets, never from the planned waypoint list.
The synthetic rig is nadir for deterministic grid coverage, not real-rig FOV.
"""
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


def run(binary, missing_surface=False, local=False):
    cv2.setNumThreads(1)
    master, slave = pty.openpty()
    stop = threading.Event()
    state = dict(armed=False, depth=.4, row=2.15 if local else 1.5, col=1.5, vx=0., vy=0., target=.4)
    lock = threading.Lock()
    phases, targets, snapshots = [], [], []
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        texture = np.full((900,900,3),210,np.uint8)
        cv2.rectangle(texture,(3,3),(896,896),(0,0,0),30)
        for p in (300,600):
            cv2.line(texture,(p,0),(p,899),(0,0,0),30)
            cv2.line(texture,(0,p),(899,p),(0,0,0),30)
        # Keep the synthetic direction strip >=5 pixels at the farthest camera
        # view. A 2-pixel strip aliased away under the unchanged 5x5 kernel.
        cv2.line(texture,(0,880),(899,880),(0,255,255),48)
        for x,y in ((150,150),(750,150)):
            cv2.circle(texture,(x,y),60,(0,80,255),-1)
        for x,y in ((150,750),(750,750)):
            cv2.rectangle(texture,(x-60,y-60),(x+60,y+60),(0,80,255),-1)
        # Tag is observed by the same front stream that later maps the grid.
        texture[315:585,315:585]=255
        marker=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5),18,240)
        texture[330:570,330:570]=cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
        map_texture=None;mapping=threading.Event();reacquire_hold=threading.Event()
        if local:
            noise=np.random.default_rng(18).integers(64,83,(900,900),dtype=np.uint8)
            texture=cv2.cvtColor(noise,cv2.COLOR_GRAY2BGR)
            for coordinate in [0,300,600,899]:
                cv2.line(texture,(coordinate,0),(coordinate,899),(255,255,255),32)
                cv2.line(texture,(0,coordinate),(899,coordinate),(0,255,255) if coordinate==899 else (255,255,255),32)
            for x,y in [(150,150),(750,150)]:cv2.circle(texture,(x,y),60,(0,80,255),-1)
            for x,y in [(150,750),(750,750)]:cv2.rectangle(texture,(x-60,y-60),(x+60,y+60),(0,80,255),-1)
            map_texture=texture.copy()
            # Synthetic tag leaves view after acquisition; never use tag ink
            # as empty-floor training evidence. No real-rig geometry implied.
            texture[718:813,403:498]=255
            local_marker=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5),18,83)
            texture[724:807,409:492]=cv2.cvtColor(local_marker,cv2.COLOR_GRAY2BGR)
        fake = root/'rpicam-vid'
        fake.write_text('''#!/usr/bin/python3
import pathlib,sys,time
root=pathlib.Path(__file__).parent
camera=sys.argv[sys.argv.index('--camera')+1]
name='down.jpg' if camera=='0' else 'front.jpg'
while True:
 try:
  sys.stdout.buffer.write((root/name).read_bytes());sys.stdout.buffer.flush()
 except FileNotFoundError:pass
 time.sleep(.05)
''')
        fake.chmod(0o755)
        cfg = yaml.safe_load(Path('runtime/config/pi-auv-task-one.yaml').read_text())
        kd = [250,0,160,0,250,120,0,0,1]
        rotation = [0,1,0,1,0,0,0,0,-1]
        cfg['camera'].update(source='csi:0',fps=20,camera_matrix=kd,distortion_coefficients=[0]*5)
        cfg['camera_front'].update(source='csi:1',width=320,height=240,fps=20,
            calibration_width=320,calibration_height=240,camera_matrix=[130,0,160,0,130,120,0,0,1],distortion_coefficients=[0]*5)
        cfg['serial']['device'] = os.ttyname(slave)
        cfg['control']['socket'] = str(root/'control.sock')
        cfg['logging'].update(events=str(root/'events.ndjson'),debug_dir=str(root/'evidence'))
        cfg['recording'].update(directory=str(root/'runs'),minimum_free_bytes=1024)
        cfg['web']['enabled'] = cfg['video']['enabled'] = False
        cfg['vision'].update(white_grid_edges=False,apriltag_family='tag16h5',apriltag_ids=[18])
        cfg['motion'].update(motion_commands_enabled=True,directions_calibrated=True,limits_calibrated=True,maximum_speed=.2)
        cfg['localization'].update(camera_matrix=kd,distortion=[0]*5,camera_to_body=rotation,
            camera_offset_m=[0]*3,depth_offset_m=[0]*3,imu_signs=[1]*3,imu_offsets_deg=[0]*3,
            pool_depth_m=1.4,calibration_verified=True)
        cfg['observation_search'].update(enabled=True,corridor_calibrated=True,maximum_radius_m=2,
            waypoints_m=[[1.,0]],depth_target_m=.4,waypoint_timeout_sec=30,maximum_speed=.02)
        cfg['surface_traversal'].update(enabled=True,ascent_clearance_verified=True,center_approach_verified=True,
            center_approach_radius_cells=.3,surface_localization_verified=True,ascent_cell=[1,1],cell_size_m=.5,
            boundary_clearance_m=.05,depth_target_m=.3,ascent_rate_mps=.1,surface_stable_sec=.2,
            center_stable_sec=.15,reacquire_frames=3,reacquire_timeout_sec=3,maximum_speed=.2,
            position_gain=1.,arrival_stable_sec=.15,pose_timeout_sec=.8,height_tolerance_m=.2,
            minimum_grid_confidence=.65)
        cfg['front_metric'].update(enabled=True,camera_to_body=rotation,camera_offset_m=[0]*3,depth_offset_m=[0]*3)
        for phase,fx in [('underwater',130),('surface',90)]:
            cfg['front_metric'][phase].update(verified=True,width=320,height=240,
                camera_matrix=[fx,0,160,0,fx,120,0,0,1],distortion_coefficients=[0]*5)
        if local:
            cfg['vision'].update(white_grid_edges=True)
            cfg['camera_front']['camera_matrix']=kd
            for phase in ['underwater','surface']:cfg['front_metric'][phase]['camera_matrix']=kd
            cfg['front_metric']['camera_offset_m']=[.2,0,0]
            cfg['local_mapping'].update(enabled=True,navigation_verified=True,
                empty_floor=dict(verified=True,hsv_lower=[0,0,50],hsv_upper=[179,15,110],minimum_support=.95))
            cfg['observation_search'].update(maximum_speed=.1,waypoint_timeout_sec=40,
                waypoints_m=[[-.775,-.45],[-.775,.45],[.125,.45],[.125,-.45],[0,0]])
            cfg['mission'].update(map_timeout_sec=120,surface_timeout_sec=90,cone_visit_timeout_sec=180)
            cfg['grid_exit'].update(enabled=True,verified=True,allowed_sides_mask=1,
                body_radius_m=.05,clearance_m=.02,timeout_sec=90)
        path = root/'config.yaml';path.write_text(yaml.safe_dump(cfg))
        surface = threading.Event()
        def render():
            while not stop.is_set():
                with lock:depth,row,col=state['depth'],state['row'],state['col']
                fx=250 if local else (90 if surface.is_set() else 130)
                scale=fx/(1.4-depth)
                offset=.2 if local else 0
                points=np.float32([[160+scale*(x-col*.5),120+scale*(y-row*.5-offset)] for x,y in [(0,0),(1.5,0),(1.5,1.5),(0,1.5)]])
                if local:assert not all(0<=p[0]<320 and 0<=p[1]<240 for p in points),'local camera must never show full outer grid'
                h=cv2.getPerspectiveTransform(np.float32([[0,0],[899,0],[899,899],[0,899]]),points)
                active_texture=map_texture if local and mapping.is_set() else texture
                image=cv2.warpPerspective(active_texture,h,(320,240),borderValue=(70,70,70) if local else (110,110,110))
                if surface.is_set() and missing_surface:image[:]=110
                if local and reacquire_hold.is_set():image[:]=70
                tmp=root/'front.tmp';tmp.write_bytes(cv2.imencode('.jpg',image)[1].tobytes());tmp.replace(root/'front.jpg')
                # Both synthetic cameras observe the SAME integrated motion.
                # A static down video previously hid actual traversal motion.
                down_scale=250/(1.4-depth)
                down_points=np.float32([[160+down_scale*(x-col*.5),120+down_scale*(y-row*.5)] for x,y in [(0,0),(1.5,0),(1.5,1.5),(0,1.5)]])
                down_h=cv2.getPerspectiveTransform(np.float32([[0,0],[899,0],[899,899],[0,899]]),down_points)
                down_image=cv2.warpPerspective(active_texture,down_h,(320,240),borderValue=(70,70,70) if local else (110,110,110))
                tmp=root/'down.tmp';tmp.write_bytes(cv2.imencode('.jpg',down_image)[1].tobytes());tmp.replace(root/'down.jpg')
                time.sleep(.05)
        def mcu():
            pending=bytearray();sequence=0;last=time.monotonic()
            while not stop.is_set():
                try:
                    now=time.monotonic();dt=min(now-last,.1);last=now;sequence+=1
                    with lock:
                        if state['armed']:
                            state['row']+=state['vx']*dt/.5;state['col']+=state['vy']*dt/.5
                            state['depth']+=np.clip(state['target']-state['depth'],-dt*.15,dt*.15)
                        armed,depth=state['armed'],state['depth']
                    os.write(master,frame(0x80,struct.pack('<IBIfffffB',sequence,int(armed),0,12.,depth,0.,0.,0.,0)))
                    os.write(master,frame(0x82,struct.pack('<IfBII',sequence,depth,1,sequence,0)))
                    if select.select([master],[],[],.03)[0]:pending.extend(os.read(master,8192))
                    while len(pending)>=7:
                        if pending[:2]!=b'\xaa\x55':del pending[0];continue
                        size=7+pending[4]
                        if len(pending)<size:break
                        packet=bytes(pending[:size]);del pending[:size]
                        if crc(packet[2:-2])!=struct.unpack('<H',packet[-2:])[0]:continue
                        kind,data=packet[3],packet[5:-2]
                        if kind==2:
                            with lock:state['armed']=bool(data[4])
                            os.write(master,frame(0x7f,bytes([2,0])+data[:4]))
                        elif kind==3:
                            target=struct.unpack('<Iffff',data);targets.append(target)
                            with lock:state.update(vx=target[1],vy=target[2],target=target[3])
                except OSError:return
        workers=[threading.Thread(target=f,daemon=True) for f in (render,mcu)]
        for worker in workers:worker.start()
        env=dict(os.environ,PATH=str(root)+':'+os.environ['PATH'])
        errors=(root/'stderr').open('w')
        proc=subprocess.Popen([binary,str(path)],env=env,stdout=subprocess.DEVNULL,stderr=errors)
        try:
            try:
                wait_for_socket(proc,root/'control.sock',timeout=15)
            except AssertionError as error:
                raise AssertionError((root/'stderr').read_text()) from error
            def status():
                value=json.loads(request(str(root/'control.sock'),'status'));snapshots.append(value)
                if local and value['phase']=='BUILD_MAP':mapping.set()
                if not phases or phases[-1]!=value['phase']:phases.append(value['phase'])
                return value
            deadline=time.monotonic()+8
            while True:
                s=status()
                if s['origin_ready'] and s['recording_ready'] and s['safe_status']:break
                assert time.monotonic()<deadline,s
                time.sleep(.05)
            assert request(str(root/'control.sock'),'start').startswith('OK')
            armed=False;deadline=time.monotonic()+(300 if local else 75);blocked_until=None
            while time.monotonic()<deadline:
                s=status()
                if s['arm_gate_ready'] and not armed:
                    # The status snapshot does not reserve the safety gate;
                    # fresh telemetry/pose must still pass at request time.
                    response=request(str(root/'control.sock'),'arm SAFE_TO_ARM')
                    assert response.startswith(('OK','ERR')),response
                    armed=response.startswith('OK')
                if s['phase']=='RELOCALIZE_SURFACE':
                    if blocked_until is None:
                        blocked_until=time.monotonic()+.35
                        if local:reacquire_hold.set()
                    # For .35 s after ascent the stream still emits underwater
                    # geometry. No old pose may authorize surface traversal.
                    if time.monotonic()<blocked_until:
                        assert not s['surface_pose_ready'] and not s['surface_grid_valid'],s
                        assert s['cmd_vx']==s['cmd_vy']==0,s
                    else:surface.set();reacquire_hold.clear()
                if s['surface_grid_valid']:
                    assert s['metric_pose_surface_frame'],s
                    assert s['metric_pose_stamp_sec']>s['surface_reacquire_start_sec'],s
                if s['phase'] in ('COMPLETE','FAULT'):break
                time.sleep(.04)
            expected='FAULT' if missing_surface else 'COMPLETE'
            assert s['phase']==expected,{'status':s,'phases':phases,'stderr':(root/'stderr').read_text()}
            # DISARM and event writes run on separate threads. Wait for the
            # actual virtual MCU acknowledgement and recorder flush, rather
            # than equating a state-machine transition with completed I/O.
            deadline=time.monotonic()+2
            while s['armed']:
                assert time.monotonic()<deadline,s
                time.sleep(.03);s=status()
            event_path=Path(s['recording_directory'])/'events.ndjson'
            while True:
                events=[json.loads(line) for line in event_path.read_text().splitlines()]
                if missing_surface or any(e.get('event')=='A2_COMPLETE' for e in events):break
                assert time.monotonic()<deadline,events
                time.sleep(.03)
            assert any(e.get('event')=='APRILTAG_FOUND' and 'camera=front' in e.get('detail','') for e in events),events
            visits=[e for e in events if e.get('event')=='CONE_VISITED']
            if not missing_surface:
                assert all(p in phases for p in ['BUILD_MAP','SURFACE_FOR_CONES','RELOCALIZE_SURFACE','VISIT_CONES','COMPLETE']),phases
                assert len(visits)==4 and len({e['detail'].split(' frame=')[0] for e in visits})==4,visits
                assert 'A2_COMPLETE' in {e.get('event') for e in events}
                if local:
                    assert 'EXIT_GRID' in phases and any(e.get('event')=='EXIT_CONFIRMED' for e in events),phases
                    assert any(e.get('event')=='LOCAL_GRID' for e in events),events
            else:
                assert 'VISIT_CONES' not in phases and not visits,phases
            assert all(abs(t[1])<=.20001 and abs(t[2])<=.20001 for t in targets)
            output=os.environ.get('AUV_SIM_EVIDENCE_DIR')
            if output:
                out=Path(output);out.mkdir(parents=True,exist_ok=True)
                (out/(('local-' if local else '')+('missing-surface.json' if missing_surface else 'closed-loop.json'))).write_text(json.dumps(
                    dict(phases=phases,final=s,visits=visits,snapshots=snapshots,events=events,targets=targets),indent=2))
            print(json.dumps(dict(local=local,scenario='missing_surface' if missing_surface else 'closed_loop',phases=phases,visits=len(visits))))
        finally:
            # Preserve diagnostics even when an assertion interrupts the run.
            # Temporary videos and virtual serial devices are not evidence of
            # real-rig navigation; keep phase/status data explicitly synthetic.
            output=os.environ.get('AUV_SIM_EVIDENCE_DIR')
            if output:
                out=Path(output);out.mkdir(parents=True,exist_ok=True)
                prefix=('local-' if local else '')+('missing-surface' if missing_surface else 'closed-loop')
                (out/(prefix+'-diagnostics.json')).write_text(json.dumps(
                    dict(synthetic=True,phases=phases,snapshots=snapshots,targets=targets),indent=2))
                errors.flush()
                (out/(prefix+'-stderr.log')).write_text((root/'stderr').read_text())
                with lock:final_body=dict(state)
                (out/(prefix+'-body.json')).write_text(json.dumps(final_body,indent=2))
                (out/(prefix+'-config.yaml')).write_bytes(path.read_bytes())
                for name in ['front.jpg','down.jpg']:
                    if (root/name).exists():(out/(prefix+'-'+name)).write_bytes((root/name).read_bytes())
                for event_file in (root/'runs').glob('*/events.ndjson'):
                    (out/(prefix+'-events.ndjson')).write_bytes(event_file.read_bytes())
            proc.send_signal(signal.SIGTERM);proc.wait(timeout=5);stop.set()
            for worker in workers:worker.join(timeout=2)
            errors.close();os.close(master);os.close(slave)


if __name__=='__main__':
    local='--local' in sys.argv[2:]
    run(sys.argv[1],local=local);run(sys.argv[1],missing_surface=True,local=local)
