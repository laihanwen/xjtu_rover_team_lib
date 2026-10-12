"""Align same-boot Pi frame clocks with bounded, independently fresh PID receipts.

Receipt times are not hardware sensor sample times. A measured model and explicit
phase are required; missing values never become calibration defaults.
"""
import argparse
import bisect
import collections
import json
import math
from pathlib import Path


def measured_model(model):
    signs = model.get('imu_signs')
    offsets = model.get('imu_offsets_deg')
    depth = model.get('depth_absolute_offset_m')
    return (model.get('sensor_model_verified') is True and isinstance(signs,list) and
            len(signs)==3 and all(type(x) is int and x in (-1,1) for x in signs) and
            isinstance(offsets,list) and len(offsets)==3 and
            all(type(x) in (int,float) and math.isfinite(x) for x in offsets) and
            type(depth) in (int,float) and math.isfinite(depth))


def align(frames, sensors, session, phase):
    if phase not in ('underwater','surface'):
        raise ValueError('explicit supported phase required')
    if not measured_model(session.get('model') or {}):
        raise ValueError('measured IMU mapping and absolute-depth offset required')
    boot = session.get('boot_id')
    if not boot:
        raise ValueError('sensor boot identity required')
    samples = [r for r in sensors if r.get('kind')=='pid']
    if len(set(r.get('mcu_epoch',0) for r in samples))>1:
        raise ValueError('MCU reset/wrap in sensor data; split the recording before replay')
    # Re-receiving the same MCU snapshot cannot refresh sensor evidence.
    unique=[];last_tick=None
    for row in samples:
        tick=row.get('mcu_tick_ms')
        if tick is not None and tick==last_tick:continue
        unique.append(row);last_tick=tick
    samples=unique
    stamps = [r['stamp'] for r in samples]
    if any(type(t) not in (int,float) or not math.isfinite(t) for t in stamps) or any(b<a for a,b in zip(stamps,stamps[1:])):
        raise ValueError('invalid or reordered sensor times')
    previous = -math.inf
    for frame in frames:
        stamp = frame['capture_monotonic_sec']
        if not math.isfinite(stamp) or stamp<=previous:
            raise ValueError('non-increasing frame clock')
        previous=stamp;index=bisect.bisect_right(stamps,stamp)-1
        row=samples[index] if index>=0 else None
        reason='ready'
        if frame.get('pi_boot_id')!=boot:reason='Pi boot mismatch or missing'
        elif row is None or stamp-row['stamp']>.15:reason='missing or stale PID receipt'
        elif row.get('imu_sample_fresh') is not True or row.get('depth_sample_fresh') is not True:reason='independent sensor invalid/stale'
        elif not row.get('numeric_valid'):reason='invalid numeric sensors'
        elif row.get('dropped_before',0)>0:reason='recording dropped sensor evidence'
        value=dict(frame=frame['frame'],stamp=stamp,phase=phase,valid=reason=='ready',reason=reason)
        if reason=='ready':
            value.update(telemetry_stamp=row['stamp'],roll_deg=row['roll_deg'],pitch_deg=row['pitch_deg'],
                         yaw_deg=row['yaw_deg'],depth_m=row['depth_m']+session['model']['depth_absolute_offset_m'])
        yield value


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('sensor_session',type=Path);p.add_argument('frame_sidecar',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--phase',choices=['underwater','surface'],required=True)
    args=p.parse_args()
    def rows(path):
        with path.open(encoding='utf-8-sig') as stream:return [json.loads(line) for line in stream if line.strip()]
    session=json.loads((args.sensor_session/'session.json').read_text(encoding='utf-8'))
    values=list(align(rows(args.frame_sidecar),rows(args.sensor_session/'sensors.jsonl'),session,args.phase))
    camera=args.frame_sidecar.name.split('_')[0]
    if camera not in ('front','down'):raise ValueError('native front/down sidecar required')
    video_session=json.loads((args.frame_sidecar.parent/'session.json').read_text(encoding='utf-8'))
    space='rectified' if video_session['camera_calibration'][camera]['preview_rectified'] else 'raw'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.output.exists():raise ValueError('output already exists')
    args.output.write_text(json.dumps(dict(camera=camera,image_space=space,
                          sensor_model=session['model'],frames=[v for v in values if v['valid']]),indent=2),encoding='utf-8')
    args.output.with_suffix('.audit.json').write_text(json.dumps(dict(records=values,
            reasons=dict(collections.Counter(v['reason'] for v in values)),
            timestamp_kind='Pi UART receipt, not hardware sampling'),indent=2),encoding='utf-8')
    print('Ready frames:',sum(v['valid'] for v in values),'/',len(values))
