"""Read-only tag screening and annotation pack for existing task videos.

No rectification, geometric pose, grid classification or actuator control.
Sample statistics are not framewise recall; human visibility labels are needed.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2


def screen(source, output, interval):
    cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise ValueError(f'Cannot decode {source}')
    fps = cap.get(cv2.CAP_PROP_FPS)
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if fps <= 0 or count <= 0:
        cap.release()
        raise ValueError('Video has no usable frame count/FPS')
    output.mkdir(parents=True, exist_ok=True)
    images = output/'samples'; images.mkdir(exist_ok=True)
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5))
    rows = []
    try:
        for index in range(0, count, max(1, round(fps*interval))):
            cap.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, image = cap.read()
            if not ok:
                rows.append(dict(frame=index, time_sec=index/fps, decode_ok=False))
                continue
            corners, ids, _ = detector.detectMarkers(image)
            detections = []
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                detections = [dict(id=int(value), corners=points.reshape(4, 2).tolist())
                              for points, value in zip(corners, ids.reshape(-1))]
            filename=f'samples/{index:07d}.jpg'
            preview=cv2.resize(image,(480, round(image.shape[0]*480/image.shape[1])))
            if not cv2.imwrite(str(output/filename),preview):
                raise OSError('Cannot save sample')
            rows.append(dict(frame=index,time_sec=index/fps,decode_ok=True,
                             detections=detections,preview=filename))
    finally:
        cap.release()
    with source.open('rb') as stream:
        digest=hashlib.file_digest(stream,'sha256').hexdigest()
    metadata=dict(source=str(source.resolve()),sha256=digest,fps=fps,frames=count,
                  interval_sec=interval,image_space='already_rectified_display_video',
                  family='tag16h5',expected_id=18,rectification_applied=False,
                  metric_pose_validated=False,sampled_frames=len(rows),
                  sampled_frames_with_id18=sum(any(t['id']==18 for t in r.get('detections',[])) for r in rows))
    (output/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
    (output/'detections.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows),encoding='utf-8')
    # Blank fields mean not reviewed; no unknown cell is silently empty.
    fields=['frame','time_sec','preview','tag_visible','expected_tag_id','grid_four_corners_visible',
            'yellow_edge_image_side','cone_count','notes']+[f'cell_{r}_{c}' for r in range(3) for c in range(3)]
    with (output/'ground-truth.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for row in rows:writer.writerow({k:row.get(k,'') for k in fields})
    (output/'README.txt').write_text(
        '人工检查 samples 图像并填写 ground-truth.csv。tag_visible 填 yes/no/uncertain；'
        '格位填 empty/circle_cone/square_cone/unknown。黄边填 top/right/bottom/left/unknown。'
        '样本用于筛查，不是完整逐帧验收；预览图缩小仅用于人工检查，检测使用原尺寸。'
        '当前脚本只检测标签，未运行 C++ GridMapper/ConeDetector，也未验证位姿。',encoding='utf-8')
    print(json.dumps(metadata,ensure_ascii=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--interval',type=float,default=2)
    args=parser.parse_args()
    if args.interval<=0:parser.error('--interval must be positive')
    screen(args.video,args.output,args.interval)
