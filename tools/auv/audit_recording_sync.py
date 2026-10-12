"""Audit native frame/ROV telemetry associations without inventing clock sync."""
import argparse
import bisect
import collections
import json
import math
from datetime import datetime
from pathlib import Path


def utc(value):
    time = datetime.fromisoformat(value)
    if time.tzinfo is None:
        raise ValueError('timezone-aware UTC required')
    return time.timestamp()


def json_rows(path):
    with path.open(encoding='utf-8-sig') as stream:
        for line in stream:
            yield json.loads(line)


def audit(recordings, logs, output):
    output.mkdir(parents=True, exist_ok=False)
    samples = []
    common_inputs = 0
    for file in sorted(logs.glob('telemetry_*.jsonl')):
        for row in json_rows(file):
            value = (row.get('localization') or {}).get('input')
            common_inputs += bool(value)
            imu = row.get('imu') or {}
            samples.append(dict(utc=utc(row['utc']), valid=bool(row.get('imu_valid')),
                                age=row.get('telemetry_age_s'), sequence=imu.get('sequence'),
                                depth=imu.get('depth_m'), roll=imu.get('roll_deg'),
                                pitch=imu.get('pitch_deg'), yaw=imu.get('yaw_deg')))
    samples.sort(key=lambda x:x['utc'])
    times = [x['utc'] for x in samples]
    totals = collections.Counter()
    sessions = []
    for folder in sorted(recordings.glob('20261011T*')):
        if folder.name < '20261011T152258':
            continue
        counts = collections.Counter()
        with (output / (folder.name + '-associations.jsonl')).open('w', encoding='utf-8') as result:
            for sidecar in sorted(folder.glob('*.frames.jsonl')):
                previous = -math.inf
                for frame in json_rows(sidecar):
                    stamp = frame['capture_monotonic_sec']
                    if not math.isfinite(stamp) or stamp <= previous:
                        raise ValueError('non-monotonic camera timestamps: ' + str(sidecar))
                    previous = stamp
                    received = utc(frame['received_utc'])
                    # Causal PC-receipt candidate only. This is not the Pi
                    # sensor sample time and cannot authorize metric replay.
                    index = bisect.bisect_right(times, received) - 1
                    candidate = samples[index] if index >= 0 else None
                    gap = received - candidate['utc'] if candidate else None
                    fresh = bool(candidate and candidate['valid'] and gap <= .15 and
                                 isinstance(candidate['age'], (int, float)) and
                                 0 <= candidate['age'] <= .15)
                    counts['frames'] += 1
                    counts['pc_receipt_candidates_fresh'] += fresh
                    counts['metric_ready'] += 0
                    result.write(json.dumps(dict(file=sidecar.name.replace('.frames.jsonl', '.avi'),
                        frame=frame['frame'], stamp_pi=stamp, received_utc=frame['received_utc'],
                        candidate_utc=datetime.fromtimestamp(candidate['utc'], datetime.fromisoformat(frame['received_utc']).tzinfo).isoformat() if candidate else None,
                        pc_receipt_gap_sec=gap, telemetry=candidate,
                        pc_receipt_candidate_fresh=fresh, metric_ready=False,
                        rejection='missing same-clock sensor input and verified absolute-depth/IMU mapping'),
                        allow_nan=False) + '\n')
        totals.update(counts)
        sessions.append(dict(session=folder.name, **counts))
    report = dict(schema='recording_sync_audit.v1', telemetry_records=len(samples),
                  localization_input_records=common_inputs, totals=dict(totals), sessions=sessions,
                  metric_replay_authorized=False,
                  limitations=['PC UTC receive matching is not sensor/camera synchronization',
                               'No guessed clock offset, yaw sign or absolute-depth zero',
                               'Repeated cache snapshots are not independent sensor events'])
    (output / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings', type=Path)
    parser.add_argument('logs', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.recordings, args.logs, args.output), ensure_ascii=False))
