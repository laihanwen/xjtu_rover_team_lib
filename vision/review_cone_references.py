"""在人工确认的实例ROI分析条纹；结果不是自动整锥检测验收。"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from review_selected_geometry import analyze

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image',type=Path);parser.add_argument('--references',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    image=cv2.imread(str(args.image));reference=json.loads(args.references.read_text(encoding='utf-8'))
    if image is None or image.shape[:2]!=(reference['image_height'],reference['image_width']):raise ValueError('reference image size mismatch')
    records=[]
    for instance in reference['instances']:
        x,y,w,h=instance['region_xywh'];roi=image[y:y+h,x:x+w];pieces=analyze(roi)
        # Experimental gray-world correction of the full frame, not ROI-specific
        # tuning; colour balance is not camera geometric rectification.
        means=image.reshape(-1,3).mean(axis=0)
        gains=means.mean()/np.maximum(means,1)
        balanced=np.clip(roi.astype(np.float32)*gains,0,255).astype(np.uint8)
        corrected_pieces=analyze(balanced)
        overlay=roi.copy()
        for index,piece in enumerate(pieces):
            cv2.drawContours(overlay,[np.array(piece['contour'],np.int32).reshape(-1,1,2)],-1,(0,0,255),2)
            bx,by,bw,bh=piece['bounds'];cv2.putText(overlay,f'band{index}',(bx,max(20,by)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),1)
        cv2.imwrite(str(args.output/(instance['id']+'-original.png')),roi)
        cv2.imwrite(str(args.output/(instance['id']+'-bands.png')),overlay)
        cv2.imwrite(str(args.output/(instance['id']+'-balanced.png')),balanced)
        records.append(dict(instance,colour_components=len(pieces),pieces=pieces,
            experimental_gray_world_components=len(corrected_pieces),gray_world_gains_bgr=gains.tolist(),
            automatic_instance_detected=False,body_contour_valid=False,base_contour_valid=False,
            reason='人工ROI只用于实例参考；条纹轮廓不等于锥体轮廓'))
    (args.output/'reference-review.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps([{'id':r['id'],'class':r['class'],'colour_components':r['colour_components'],
                      'balanced_components':r['experimental_gray_world_components']} for r in records]))

if __name__=='__main__':main()
