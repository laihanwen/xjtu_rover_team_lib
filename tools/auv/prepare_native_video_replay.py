"""Prepare independent, rectified C++ replay samples from native ROV recordings.

Never estimates telemetry or changes navigation verification flags. Output frames
are sparse observations, not a continuous trajectory or a new camera recording.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def sample_indices(rows, interval):
    if not math.isfinite(interval) or interval <= 0:
        raise ValueError('interval must be positive')
    previous = -math.inf
    selected = []
    next_stamp = -math.inf
    for index, row in enumerate(rows):
        stamp = row['capture_monotonic_sec']
        if row['frame'] != index or not math.isfinite(stamp) or stamp <= previous:
            raise ValueError('frame sidecar is incomplete or non-monotonic')
        previous = stamp
        if stamp >= next_stamp:
            selected.append(index)
            next_stamp = stamp + interval
    return selected


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def prepare(session, camera, output, interval):
    import cv2
    import numpy as np
    cv2.setNumThreads(1)
    metadata = json.loads((session / 'session.json').read_text(encoding='utf-8-sig'))
    calibration = metadata['camera_calibration'][camera]
    width, height = calibration['width'], calibration['height']
    k = np.asarray(calibration['camera_matrix'], dtype=float).reshape(3, 3)
    d = np.asarray(calibration['raw_distortion_coefficients'], dtype=float)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('invalid recorded camera model')
    # Keep the original K and native dimensions; rectification happens once.
    maps = cv2.initUndistortRectifyMap(k, d, None, k, (width, height), cv2.CV_32FC1)
    output.mkdir(parents=True, exist_ok=False)
    records = []
    for source in sorted(session.glob(camera + '_*.avi')):
        sidecar = source.with_suffix('.frames.jsonl')
        rows = [json.loads(line) for line in sidecar.read_text(encoding='utf-8-sig').splitlines() if line.strip()]
        indices = sample_indices(rows, interval)
        capture = cv2.VideoCapture(str(source))
        if not capture.isOpened() or int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) != len(rows):
            raise ValueError('video/sidecar frame count mismatch: ' + str(source))
        destination = output / source.name
        writer = cv2.VideoWriter(str(destination), cv2.VideoWriter_fourcc(*'FFV1'), 1, (width, height))
        if not writer.isOpened():
            raise RuntimeError('cannot create replay video')
        mapping = []
        try:
            for index in indices:
                capture.set(cv2.CAP_PROP_POS_FRAMES, index)
                ok, image = capture.read()
                if not ok or image.shape[:2] != (height, width):
                    raise ValueError('decode failure or calibration dimension mismatch')
                if not calibration['preview_rectified']:
                    image = cv2.remap(image, *maps, cv2.INTER_LINEAR)
                writer.write(image)
                mapping.append(dict(replay_frame=len(mapping), source_file=source.name,
                                    source_frame=index, **{key: rows[index][key] for key in
                                    ('capture_monotonic_sec', 'received_utc')}))
        finally:
            capture.release()
            writer.release()
        destination.with_suffix('.mapping.jsonl').write_text(
            ''.join(json.dumps(row, allow_nan=False) + '\n' for row in mapping), encoding='utf-8')
        records.append(dict(source=str(source.resolve()), source_sha256=sha256(source),
                            sidecar_sha256=sha256(sidecar), output=destination.name,
                            output_sha256=sha256(destination), sampled_frames=len(mapping)))
    if not records:
        raise ValueError('no camera recordings found')
    report = dict(schema='native_semantic_replay.v1', session=metadata['session'], camera=camera,
                  image_space='rectified', camera_matrix=k.reshape(-1).tolist(), distortion=[0]*len(d),
                  calibration_id=calibration['calibration_id'], interval_sec=interval,
                  independent_observations=True, codec='FFV1 lossless', metric_navigation_verified=False,
                  session_sha256=sha256(session / 'session.json'), files=records,
                  cpp_arguments='OUTPUT_VIDEO OUTPUT_DIRECTORY 1 --independent',
                  limitations=['AVI playback time is artificial; use source frame mapping',
                               'PC receive UTC does not prove sensor synchronization',
                               'Full-grid rejection is not a local mapping result'])
    (output / 'manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('camera', choices=['front', 'down'])
    parser.add_argument('output', type=Path)
    parser.add_argument('--interval', type=float, default=2)
    args = parser.parse_args()
    print(json.dumps(prepare(args.session, args.camera, args.output, args.interval), ensure_ascii=False))
