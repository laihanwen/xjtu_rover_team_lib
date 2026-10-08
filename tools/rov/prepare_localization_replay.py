"""Align recorded CSI frame timestamps with Pi LOCALIZATION event inputs."""
import argparse,bisect,json,math
from pathlib import Path

def align(frames,events):
    inputs=[]
    for event in events:
        if event.get('event')!='LOCALIZATION':continue
        detail=event.get('detail',{})
        if isinstance(detail,str):detail=json.loads(detail)
        value=detail.get('input')
        if value and value.get('valid') and math.isfinite(value['stamp']):inputs.append(value)
    inputs.sort(key=lambda v:v['stamp']);times=[v['stamp'] for v in inputs]
    previous=-1
    for frame in frames:
        stamp=frame['capture_monotonic_sec']
        if not math.isfinite(stamp) or stamp<=previous:raise ValueError('non-increasing frame timestamps: do not mix Pi boots')
        previous=stamp;index=bisect.bisect_right(times,stamp)-1
        value=inputs[index] if index>=0 and stamp-times[index]<=.15 else {'stamp':stamp,'valid':False,'armed':True}
        yield dict(value,frame_stamp=stamp)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frames',type=Path);parser.add_argument('events',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args()
    def read(p):return [json.loads(line) for line in p.read_text(encoding='utf-8-sig').splitlines() if line.strip()]
    values=list(align(read(args.frames),read(args.events)))
    args.output.write_text(''.join(json.dumps(v,allow_nan=False)+'\n' for v in values),encoding='utf-8')
    print(f'{len(values)} frames; {sum(not v.get("valid") for v in values)} unmatched inputs')
if __name__=='__main__':main()
