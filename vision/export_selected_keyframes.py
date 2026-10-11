"""从已选择原始帧导出高清图与空轮廓标注，不伪造观测。"""
import argparse
import json
from pathlib import Path
import cv2

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selection',type=Path,required=True)
    parser.add_argument('--materials',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    manifest=[]
    for camera,name in [('front','前.mp4'),('down','下.mp4')]:
        selected=json.loads((args.selection/f'{camera}-selected.json').read_text(encoding='utf-8'))
        cap=cv2.VideoCapture(str(args.materials/name))
        if not cap.isOpened():raise ValueError(name)
        directory=args.output/camera;directory.mkdir(exist_ok=True)
        try:
            for record in selected:
                frame=record['frame'];filename=f'{frame:07d}.png';target=directory/filename
                cap.set(cv2.CAP_PROP_POS_FRAMES,frame);ok,image=cap.read()
                if not ok:raise ValueError(f'{name}: {frame}')
                if not cv2.imwrite(str(target),image):raise OSError(target)
                annotation=directory/f'{frame:07d}.json'
                # Never overwrite later manual polygon work.
                if not annotation.exists():
                    annotation.write_text(json.dumps({'version':'5.0.1','flags':{},'shapes':[],
                        'imagePath':filename,'imageData':None,'imageHeight':image.shape[0],'imageWidth':image.shape[1]},ensure_ascii=False,indent=2),encoding='utf-8')
                manifest.append(dict(record,camera=camera,image=f'{camera}/{filename}',
                    image_space='already_rectified_display_video',pixel_annotation_status='not_annotated'))
        finally:cap.release()
    (args.output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'exported':len(manifest),'pixel_annotations_created':'empty templates only'}))

if __name__=='__main__':main()
