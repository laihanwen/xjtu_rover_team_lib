"""Offline video/tag visibility and telemetry evidence audit. Never sends control."""
import argparse,json,math
from pathlib import Path
from datetime import datetime

def telemetry(files,start,end):
    rows=[];errors=[]
    for file in files:
        with file.open(encoding='utf-8-sig') as stream:
            for number,line in enumerate(stream,1):
                try:
                    r=json.loads(line);t=datetime.fromisoformat(r['utc'])
                    if start<=t<=end:rows.append(r)
                except (ValueError,KeyError,TypeError) as e:errors.append(dict(file=str(file),line=number,error=str(e)))
    rows.sort(key=lambda r:r['pc_monotonic_s'])
    valid=[r['imu'] for r in rows if r.get('imu_valid') and isinstance(r.get('imu'),dict)]
    depths=[v['depth_m'] for v in valid if isinstance(v.get('depth_m'),(int,float)) and math.isfinite(v['depth_m'])]
    return dict(records=len(rows),imu_valid_records=len(valid),depth_range_m=[min(depths),max(depths)] if depths else None,
        negative_depth_records=sum(d<0 for d in depths),localization_valid_records=sum(bool((r.get('localization')or{}).get('valid'))for r in rows),
        output_near_limit_records=sum(any(isinstance(x,(int,float))and abs(x)>=.99 for x in v.get('outputs',[]))for v in valid),
        localization_reasons=sorted(set((r.get('localization')or{}).get('reason','unknown')for r in rows)),parse_errors=errors),rows

def video(file,output,interval):
    import cv2
    cv2.setNumThreads(2)
    c=cv2.VideoCapture(str(file))
    if not c.isOpened():raise RuntimeError('Cannot open '+str(file))
    fps=c.get(cv2.CAP_PROP_FPS);count=int(c.get(cv2.CAP_PROP_FRAME_COUNT))
    if fps<=0:raise ValueError('invalid video fps')
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5))
    rows=[]
    for index in range(0,count,max(1,round(interval*fps))):
        c.set(cv2.CAP_PROP_POS_FRAMES,index);ok,f=c.read()
        if not ok:rows.append(dict(frame=index,video_seconds=index/fps,error='decode_failed'));continue
        # Exported footage is scaled; analyse in its own image space, not with runtime K/D.
        scale=min(1,640/f.shape[1]);small=cv2.resize(f,None,fx=scale,fy=scale)
        corners,ids,_=detector.detectMarkers(small);found=[]
        if ids is not None:
            for points,id in zip(corners,ids.ravel()):
                p=points.reshape(4,2);edges=[float(cv2.norm(p[i]-p[(i+1)%4]))for i in range(4)]
                found.append(dict(id=int(id),center=p.mean(axis=0).tolist(),minimum_edge_px=min(edges),
                    border_clear=bool((p[:,0]>=3).all()and(p[:,1]>=3).all()and(p[:,0]<small.shape[1]-3).all()and(p[:,1]<small.shape[0]-3).all())))
        rows.append(dict(frame=index,video_seconds=index/fps,analysis_size=[small.shape[1],small.shape[0]],tags=found,
            expected_id_visible=any(t['id']==18 for t in found),image_space='exported_unknown',metric_valid=False))
    c.release();(output/(file.stem+'-observations.jsonl')).write_text('\n'.join(json.dumps(r,ensure_ascii=False)for r in rows),encoding='utf-8')
    return dict(file=str(file),fps=fps,frames=count,duration_sec=count/fps,sampled=len(rows),id18_visible_samples=sum(r.get('expected_id_visible',False)for r in rows),
        note='Visibility is detector-dependent; not proof of metric navigation or camera synchronisation')

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--videos',nargs='+',type=Path,required=True);p.add_argument('--logs',type=Path,required=True);p.add_argument('--start',required=True);p.add_argument('--end',required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--interval',type=float,default=1)
    a=p.parse_args()
    if not math.isfinite(a.interval)or a.interval<=0:raise ValueError('interval must be positive')
    start,end=datetime.fromisoformat(a.start),datetime.fromisoformat(a.end)
    if start.tzinfo is None or end.tzinfo is None or end<start:raise ValueError('timezone-aware ordered dates required')
    a.output.mkdir(parents=True,exist_ok=True)
    stats,rows=telemetry(sorted(a.logs.glob('telemetry_*.jsonl')),start,end)
    if not rows:raise RuntimeError('No telemetry in requested interval')
    (a.output/'telemetry.jsonl').write_text('\n'.join(json.dumps(r,ensure_ascii=False)for r in rows),encoding='utf-8')
    report=dict(schema='manual_task_evidence.v1',telemetry=stats,videos=[video(f,a.output,a.interval)for f in a.videos],
        limitations=['Edited video time is not UTC; align through original frame sidecars','Output normalisation is not thrust or speed calibration','No automatic calibration promotion','Repeated telemetry is not independent sensor sampling'])
    (a.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
