"""Offline chessboard video calibration; outputs never change live ROS settings."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def save_image(path, frame):
    ok, encoded = cv2.imencode('.jpg', frame)
    if not ok:
        raise RuntimeError(f'Cannot encode {path}')
    encoded.tofile(str(path))


def fit(views, obj, size, flags=0):
    return cv2.calibrateCameraExtended(
        [obj] * len(views), [v['corners'] for v in views], size, None, None, flags=flags)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--cols', type=int, default=11, help='Inner corners, not squares')
    parser.add_argument('--rows', type=int, default=8)
    parser.add_argument('--square-mm', type=float, default=15)
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--classic-first', action='store_true', help='Fast classical detection first, useful for compressed videos')
    parser.add_argument('--classic-only', action='store_true', help='Use classical detector without SB fallback')
    parser.add_argument('--min-pose-distance', type=float, default=3.0, help='RMS corner displacement in pixels for duplicate rejection')
    parser.add_argument('--fix-k3', action='store_true', help='Constrain weakly observed high order radial distortion to zero')
    parser.add_argument('--enable-ros-calibration', action='store_true', help='Enable ROS use only after independent quality review')
    args = parser.parse_args()
    if args.interval <= 0 or args.cols < 2 or args.rows < 2 or args.square_mm <= 0 or args.min_pose_distance <= 0:
        parser.error('Interval and square size must be positive; corner dimensions must be >= 2')
    cv2.setNumThreads(2)
    args.output.mkdir(parents=True, exist_ok=True)
    pattern = (args.cols, args.rows)
    obj = np.zeros((args.cols * args.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[:args.cols, :args.rows].T.reshape(-1, 2) * args.square_mm / 1000
    views, records, size = [], [], None
    paths = sorted(p for p in ([args.input] if args.input.is_file() else args.input.iterdir())
                   if p.suffix.lower() in ('.avi', '.jpg', '.jpeg', '.png', '.bmp'))
    for path in paths:
        is_image = path.suffix.lower() != '.avi'
        cap = None if is_image else cv2.VideoCapture(str(path))
        if cap is not None and not cap.isOpened():
            raise RuntimeError(f'Cannot open {path}')
        fps, count = (1.0, 1) if is_image else (cap.get(cv2.CAP_PROP_FPS), int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
        if fps <= 0:
            raise RuntimeError(f'Invalid FPS: {path}')
        step = max(1, round(fps * args.interval))
        found = 0
        for index in range(0, count, step):
            if is_image:
                frame = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
                ok = frame is not None
            else:
                cap.set(cv2.CAP_PROP_POS_FRAMES, index)
                ok, frame = cap.read()
            if not ok:
                continue
            current_size = (frame.shape[1], frame.shape[0])
            if size is not None and current_size != size:
                raise RuntimeError('Mixed image dimensions: calibrate each mode separately')
            size = current_size
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if args.classic_first or args.classic_only:
                enlarged = cv2.resize(gray, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
                success, corners = cv2.findChessboardCorners(enlarged, pattern,
                    cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
                if success:
                    corners = cv2.cornerSubPix(enlarged, corners, (5, 5), (-1, -1),
                        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 0.001)) / 3 * 2
            else:
                success = False
            # Upsampling aids detection of the small board in these recordings.
            if not success and not args.classic_only:
                enlarged = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
                success, corners = cv2.findChessboardCornersSB(
                    enlarged, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
            if not success and not args.classic_only:
                contrast = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
                enlarged = cv2.resize(contrast, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC)
                success, corners = cv2.findChessboardCornersSB(
                    enlarged, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
                if not success:
                    success, corners = cv2.findChessboardCorners(
                        enlarged, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
                    if success:
                        corners = cv2.cornerSubPix(enlarged, corners, (5, 5), (-1, -1),
                            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 0.001))
                if success:
                    corners = corners / 2  # Common scale below is two.
            record = {'video': path.name, 'frame': index, 'seconds': index / fps,
                      'detected': bool(success)}
            records.append(record)
            if len(records) % 100 == 0:
                print(f'{path.name}: scanned {len(records)} frames, selected {len(views)}', flush=True)
            if not success:
                continue
            corners = (corners / 2).astype(np.float32).reshape(-1, 1, 2)
            found += 1
            # Avoid overweighting consecutive nearly identical board poses.
            if any(np.sqrt(np.mean((corners - v['corners']) ** 2)) < args.min_pose_distance for v in views):
                record['selection'] = 'similar_pose'
                continue
            record['selection'] = 'selected'
            views.append({'corners': corners, 'frame_image': frame, 'record': record})
        if cap is not None:
            cap.release()
        if not is_image or len(records) % 25 == 0:
            print(f'Processed {len(records)} frames; {len(views)} selected total', flush=True)
    (args.output / 'detections.json').write_text(
        json.dumps(records, ensure_ascii=False, indent=2), encoding='utf-8')
    selected_dir = args.output / 'selected_frames'
    selected_dir.mkdir(exist_ok=True)
    for i, v in enumerate(views):
        save_image(selected_dir / f'{i:04d}.jpg', v['frame_image'])
    (args.output / 'selected_sources.json').write_text(
        json.dumps([v['record'] for v in views], ensure_ascii=False, indent=2), encoding='utf-8')
    if len(views) < 12:
        raise RuntimeError(f'Only {len(views)} diverse views; collect more material')
    # Deterministic holdout; report it independently of training outlier removal.
    held = views[::5]
    train = [v for i, v in enumerate(views) if i % 5]
    flags = cv2.CALIB_FIX_K3 if args.fix_k3 else 0
    initial = fit(train, obj, size, flags)
    errors = initial[-1].ravel()
    threshold = max(0.6, float(np.median(errors) + 3 * 1.4826 * np.median(abs(errors - np.median(errors)))))
    kept = [v for v, e in zip(train, errors) if e <= threshold]
    if len(kept) < 12:
        raise RuntimeError('Too few views after outlier rejection')
    rms, k, d, rv, tv, std, _, per_view = fit(kept, obj, size, flags)
    hold_errors = []
    for v in held:
        ok, r, t = cv2.solvePnP(obj, v['corners'], k, d, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            raise RuntimeError('Held-out pose estimation failed')
        r, t = cv2.solvePnPRefineLM(obj, v['corners'], k, d, r, t)
        projected, _ = cv2.projectPoints(obj, r, t, k, d)
        hold_errors.append(float(np.sqrt(np.mean(np.sum((projected - v['corners']) ** 2, axis=2)))))
    new_k, roi = cv2.getOptimalNewCameraMatrix(k, d, size, 0, size)
    report = {'image_size': size, 'inner_corners': pattern, 'square_size_m': args.square_mm / 1000,
              'calibration_flags': int(flags), 'fix_k3': args.fix_k3,
              'sample_interval_seconds': args.interval, 'min_pose_distance_px': args.min_pose_distance,
              'model': 'plumb_bob', 'sampled_frames': len(records),
              'detections': sum(r['detected'] for r in records), 'diverse_views': len(views),
              'training_views': len(kept), 'rejected_training_views': len(train)-len(kept),
              'holdout_views': len(held), 'rms_px': rms,
              'holdout_rms_px': float(np.sqrt(np.mean(np.square(hold_errors)))),
              'holdout_errors_px': hold_errors, 'camera_matrix': k.tolist(),
              'distortion_coefficients': d.ravel().tolist(),
              'intrinsic_std_deviations': std.ravel().tolist(),
              'per_view_errors_px': per_view.ravel().tolist(),
              'training_sources': [v['record'] for v in kept],
              'holdout_sources': [v['record'] for v in held],
              'rectified_camera_matrix': new_k.tolist(), 'rectified_roi': list(roi)}
    (args.output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    camera = {'image_width': size[0], 'image_height': size[1], 'camera_name': args.output.name,
              'camera_matrix': {'rows': 3, 'cols': 3, 'data': k.ravel().tolist()},
              'distortion_model': 'plumb_bob',
              'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': d.ravel().tolist()},
              'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).ravel().tolist()},
              'projection_matrix': {'rows': 3, 'cols': 4, 'data': np.column_stack((new_k, np.zeros(3))).ravel().tolist()}}
    (args.output / 'camera_info.yaml').write_text(yaml.safe_dump(camera, sort_keys=False), encoding='utf-8')
    params = {'/**': {'ros__parameters': {'calibration_configured': args.enable_ros_calibration,
              'camera_matrix': k.ravel().tolist(), 'distortion_coefficients': d.ravel().tolist()}}}
    (args.output / 'ros_parameters.yaml').write_text(yaml.safe_dump(params, sort_keys=False), encoding='utf-8')
    np.savez(args.output / 'calibration.npz', camera_matrix=k, distortion_coefficients=d,
             image_size=size, rectified_camera_matrix=new_k)
    for i, v in enumerate(held[:6]):
        raw = v['frame_image']
        annotated = raw.copy()
        cv2.drawChessboardCorners(annotated, pattern, v['corners'], True)
        corrected = cv2.undistort(raw, k, d, None, new_k)
        save_image(args.output / f'preview_{i:02d}.jpg', np.hstack((annotated, corrected)))
    print(json.dumps({key: report[key] for key in ('image_size', 'training_views', 'rms_px', 'holdout_rms_px')}, indent=2))


if __name__ == '__main__':
    main()
