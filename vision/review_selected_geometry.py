"""高清关键帧的轮廓与管线候选证据；分类/全局关联保持未知。"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from local_grid_candidates import candidates

def analyze(image):
    lab=cv2.cvtColor(image,cv2.COLOR_BGR2Lab);channels=cv2.split(lab)
    channels=list(channels);channels[0]=cv2.createCLAHE(2.,(8,8)).apply(channels[0])
    enhanced=cv2.cvtColor(cv2.merge(channels),cv2.COLOR_Lab2BGR)
    hsv=cv2.cvtColor(enhanced,cv2.COLOR_BGR2HSV)
    mask=cv2.bitwise_or(cv2.inRange(hsv,(0,80,50),(20,255,255)),cv2.inRange(hsv,(165,80,50),(179,255,255)))
    kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel);mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    output=[]
    for contour in contours:
        area=cv2.contourArea(contour)
        if area<image.shape[0]*image.shape[1]*.0003:continue
        x,y,w,h=cv2.boundingRect(contour);perimeter=cv2.arcLength(contour,True)
        output.append({'bounds':[x,y,w,h],'area_px':area,'circularity':float(4*np.pi*area/(perimeter*perimeter)) if perimeter else 0,
                       'contour':contour.reshape(-1,2).tolist(),'shape':'unknown','reason':'colour candidate; body/base/global cell unassociated'})
    return sorted(output,key=lambda x:x['area_px'],reverse=True)[:12]

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('selected',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--colour-balance',action='store_true',help='实验性整帧灰世界，不是畸变矫正')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    source=json.loads((args.selected/'manifest.json').read_text(encoding='utf-8'));reports=[];tiles=[]
    for record in source:
        image=cv2.imread(str(args.selected/record['image']))
        if image is None:raise ValueError(record['image'])
        gains=np.ones(3)
        if args.colour_balance:
            means=image.reshape(-1,3).mean(axis=0);gains=means.mean()/np.maximum(means,1)
        detection_image=np.clip(image.astype(np.float32)*gains,0,255).astype(np.uint8)
        cones=analyze(detection_image);grid=candidates(image)
        report=dict(record,cone_colour_candidates=cones,local_grid=grid,semantic_map_valid=False,
                    experimental_colour_balance=args.colour_balance,gains_bgr=gains.tolist())
        reports.append(report);debug=image.copy()
        for i,cone in enumerate(cones):
            points=np.array(cone['contour'],np.int32).reshape(-1,1,2);cv2.drawContours(debug,[points],-1,(0,0,255),2)
            x,y,w,h=cone['bounds'];cv2.rectangle(debug,(x,y),(x+w,y+h),(0,255,255),2)
            cv2.putText(debug,f'C{i}: unknown',(x,max(25,y)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,255),2)
        camera=record['camera'];frame=record['frame'];directory=args.output/camera;directory.mkdir(exist_ok=True)
        cv2.imwrite(str(directory/f'{frame:07d}-contours.jpg'),debug)
        for a in grid['lines']:cv2.line(debug,tuple(a[:2]),tuple(a[2:]),(255,0,255),1)
        for p in grid['intersection_candidates']:cv2.circle(debug,tuple(round(v) for v in p),6,(0,255,0),2)
        cv2.imwrite(str(directory/f'{frame:07d}-geometry.jpg'),debug)
        if record['cone_body_clear']=='是':
            tile=cv2.resize(debug,(480,360));cv2.rectangle(tile,(0,0),(480,35),(0,0,0),-1)
            cv2.putText(tile,f'{camera} f{frame} cones:{len(cones)} nodes:{len(grid["intersection_candidates"])}',(5,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
            tiles.append(tile)
    for start in range(0,len(tiles),6):
        batch=tiles[start:start+6];batch+=[np.zeros((360,480,3),np.uint8)]*(6-len(batch))
        cv2.imwrite(str(args.output/f'contact-{start//6}.jpg'),np.vstack([np.hstack(batch[:3]),np.hstack(batch[3:])]))
    (args.output/'evidence.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'selected_frames':len(reports),'frames_with_colour_candidates':sum(bool(x['cone_colour_candidates']) for x in reports),'global_pose_valid':False}))

if __name__=='__main__':main()
