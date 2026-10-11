"""加密复核可见但未解码区间，不改写人工标注或矫正视频。"""
import argparse
import csv
import json
from pathlib import Path
import cv2

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--review',type=Path,required=True)
    parser.add_argument('--materials',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5))
    summary={}
    for camera,name in [('front','前.mp4'),('down','下.mp4')]:
        with (args.review/f'{camera}-annotations.csv').open(encoding='utf-8-sig') as stream:
            labels=list(csv.DictReader(stream))
        cap=cv2.VideoCapture(str(args.materials/name));fps=cap.get(cv2.CAP_PROP_FPS);count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if not cap.isOpened() or fps<=0:raise ValueError(name)
        misses=[int(row['frame']) for row in labels if row['tag_visible']=='yes' and not row['automatic_tag_ids']]
        indices=sorted({i for center in misses for i in range(max(0,center-round(fps)),min(count,center+round(fps)+1),max(1,round(fps*.1)))})
        output=args.output/camera;output.mkdir(exist_ok=True);rows=[]
        try:
            for index in indices:
                cap.set(cv2.CAP_PROP_POS_FRAMES,index);ok,image=cap.read()
                if not ok:rows.append({'frame':index,'decode_ok':False});continue
                gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);corners,ids,rejected=detector.detectMarkers(gray)
                detections=[]
                if ids is not None:
                    for points,value in zip(corners,ids.reshape(-1)):
                        points=points.reshape(4,2)
                        detections.append({'id':int(value),'corners':points.tolist(),
                                           'minimum_edge_px':min(float(cv2.norm(points[k]-points[(k+1)%4])) for k in range(4))})
                    cv2.aruco.drawDetectedMarkers(image,corners,ids)
                rows.append({'frame':index,'time_sec':index/fps,'decode_ok':True,'detections':detections,
                             'rejected_quads':len(rejected),'image_laplacian_variance':float(cv2.Laplacian(gray,cv2.CV_64F).var()),
                             'label_visibility':'not_reviewed','crop_occlusion':'not_reviewed','tilt':'not_reviewed'})
                if index in misses:
                    cv2.imwrite(str(output/f'{index:07d}.jpg'),image)
        finally:cap.release()
        (output/'frames.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')
        summary[camera]={'original_missed_visible_samples':len(misses),'dense_samples':len(rows),
                         'dense_id18_hits':sum(any(d['id']==18 for d in row.get('detections',[])) for row in rows),
                         'note':'加密帧尚无人工作真值；全图清晰度不能替代标签ROI清晰度；拒绝四边形不能当标签'}
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False))

if __name__=='__main__':main()
