"""Review screen-recorded camera previews; no UART, motion or camera access.

ROIs must describe the image rectangles in the screen recording. Resampling is
only for approximate pixel statistics, never calibration or metric navigation.
Already corrected previews are not undistorted again.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def roi(text):
    values = tuple(int(v) for v in text.split(','))
    if len(values) != 4 or min(values[:2]) < 0 or min(values[2:]) <= 0:
        raise argparse.ArgumentTypeError('ROI requires x,y,width,height')
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video', type=Path)
    parser.add_argument('--down-roi', type=roi, required=True)
    parser.add_argument('--front-roi', type=roi, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sample-hz', type=float, default=1)
    args = parser.parse_args()
    if not 0 < args.sample_hz <= 30:
        parser.error('sample-hz must be in (0,30]')
    cv2.setNumThreads(1)
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise RuntimeError('Cannot open supplied video')
    fps = capture.get(cv2.CAP_PROP_FPS)
    total = capture.get(cv2.CAP_PROP_FRAME_COUNT)
    if fps <= 0 or total <= 0:
        raise RuntimeError('Missing video timing metadata')
    params = cv2.aruco.DetectorParameters()
    params.aprilTagQuadDecimate = 1.0
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5), params)
    samples = []
    try:
        for time_sec in np.arange(0, total/fps, 1/args.sample_hz):
            capture.set(cv2.CAP_PROP_POS_MSEC, float(time_sec)*1000)
            ok, image = capture.read()
            if not ok:
                break
            sample = {'video_time_sec': float(time_sec)}
            for role, bounds in [('down', args.down_roi), ('front', args.front_roi)]:
                x, y, width, height = bounds
                if x+width > image.shape[1] or y+height > image.shape[0]:
                    raise RuntimeError(f'{role} ROI outside screen recording')
                preview = cv2.resize(image[y:y+height, x:x+width], (320, 240))
                corners, ids, _ = detector.detectMarkers(preview)
                tags = []
                if ids is not None:
                    for square, tag_id in zip(corners, ids.flatten()):
                        points = square[0]
                        center = points.mean(axis=0)
                        edge = min(float(np.linalg.norm(points[i]-points[(i+1)%4])) for i in range(4))
                        border = float(min(points[:, 0].min(), points[:, 1].min(),
                                           320-points[:, 0].max(), 240-points[:, 1].max()))
                        tags.append({'id': int(tag_id), 'center_px': center.tolist(),
                            'minimum_edge_px': edge, 'border_margin_px': border,
                            'center_error_px': float(np.linalg.norm(center-[160, 120])),
                            'pixel_quality_usable': bool(edge >= 8 and border >= 3)})
                sample[role] = tags
            samples.append(sample)
    finally:
        capture.release()
    summary = {}
    for role in ['down', 'front']:
        found = [(s['video_time_sec'], t) for s in samples for t in s[role] if t['id'] == 18]
        summary[role] = {'detected_samples': len(found),
            'detected_times_sec': [time for time, _ in found],
            'minimum_edge_range_px': [min(t['minimum_edge_px'] for _, t in found),
                                      max(t['minimum_edge_px'] for _, t in found)] if found else None}
    summary['both_detected_samples'] = sum(
        any(t['id'] == 18 for t in s['down']) and any(t['id'] == 18 for t in s['front']) for s in samples)
    result = {'video': str(args.video), 'duration_sec': total/fps,
        'sample_hz': args.sample_hz, 'opencv_version': cv2.__version__,
        'down_roi': args.down_roi, 'front_roi': args.front_roi,
        'coordinate_space': 'approximate screen ROI resampled to 320x240; no metric pose',
        'summary': summary, 'samples': samples}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    main()
