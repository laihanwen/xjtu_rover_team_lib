"""局部白管/黄边候选诊断；无全局格号、位姿或运动输出。"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

def candidates(image):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    white=cv2.inRange(hsv,(0,0,120),(179,90,255))
    yellow=cv2.inRange(hsv,(15,60,60),(40,255,255))
    mask=cv2.bitwise_or(white,yellow)
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    edges=cv2.Canny(mask,60,160)
    found=cv2.HoughLinesP(edges,1,np.pi/180,40,minLineLength=40,maxLineGap=8)
    lines=[] if found is None else found.reshape(-1,4).tolist()
    # Bound pairwise work; excess candidates are ambiguous, not success.
    overflow=len(lines)>80;lines=lines[:80];nodes=[]
    for i,a in enumerate(lines):
        p=np.array(a[:2],float);u=np.array(a[2:],float)-p
        for b in lines[i+1:]:
            q=np.array(b[:2],float);v=np.array(b[2:],float)-q
            cross=u[0]*v[1]-u[1]*v[0]
            if abs(cross)<.5*np.linalg.norm(u)*np.linalg.norm(v):continue
            delta=q-p;t=(delta[0]*v[1]-delta[1]*v[0])/cross
            s=(delta[0]*u[1]-delta[1]*u[0])/cross
            if not(-.08<=t<=1.08 and -.08<=s<=1.08):continue
            point=p+t*u
            if not(0<=point[0]<image.shape[1] and 0<=point[1]<image.shape[0]):continue
            if all(np.linalg.norm(point-np.array(existing))>8 for existing in nodes):nodes.append(point.tolist())
    return {'lines':lines,'intersection_candidates':nodes,'candidate_overflow':overflow,
            'global_association':'unverified','pose_valid':False}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('samples',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True);results=[]
    for index,path in enumerate(sorted(args.samples.glob('*.jpg'))):
        image=cv2.imread(str(path))
        if image is None:raise ValueError(str(path))
        result=candidates(image);result.update(sample=path.name,image_space='reduced_annotation_preview',width=image.shape[1],height=image.shape[0]);results.append(result)
        if index%10==0:
            for a in result['lines']:cv2.line(image,tuple(a[:2]),tuple(a[2:]),(255,0,255),1)
            for p in result['intersection_candidates']:cv2.circle(image,tuple(round(v) for v in p),4,(0,0,255),1)
            cv2.imwrite(str(args.output/path.name),image)
    (args.output/'candidates.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in results),encoding='utf-8')
    print(json.dumps({'samples':len(results),'with_candidates':sum(bool(x['intersection_candidates']) for x in results),'global_pose_valid':False}))

if __name__=='__main__':main()
