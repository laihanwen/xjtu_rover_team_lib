"""Independent PC dataset recorder: JPEG snapshots -> separate MJPEG AVI + timestamps.
No joystick, ARM, UART or mission control operations are performed here.
"""
import json
import struct
import threading
import time
import shutil
import math
from datetime import datetime, timezone
from pathlib import Path
from urllib.request import urlopen
from urllib.error import HTTPError


def multipart_frame(response, boundary):
    """Read one bounded MJPEG part; HTTPResponse removes transfer chunk framing."""
    marker=b'--'+boundary
    line=response.readline(4097)
    if line.strip()!=marker:
        raise ValueError('invalid MJPEG boundary or disconnected stream')
    headers={};size=0
    while True:
        line=response.readline(4097);size+=len(line)
        if not line or len(line)>4096 or size>16384:
            raise ValueError('invalid MJPEG headers')
        if line==b'\r\n':break
        key,sep,value=line.partition(b':')
        if not sep:raise ValueError('invalid MJPEG header')
        headers[key.decode('ascii').lower()]=value.strip().decode('ascii')
    length=int(headers.get('content-length',0))
    if not 0<length<=8*1024*1024 or headers.get('content-type')!='image/jpeg':
        raise ValueError('invalid MJPEG frame length/type')
    jpeg=response.read(length)
    if len(jpeg)!=length or response.read(2)!=b'\r\n':
        raise ValueError('truncated MJPEG frame')
    return jpeg,headers.get('x-frame-time-monotonic'),headers.get('x-camera-source')


def jpeg_size(data):
    if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
        raise ValueError('invalid JPEG')
    i=2
    while i+4<=len(data):
        if data[i]!=255:raise ValueError('invalid JPEG marker')
        while data[i]==255:i+=1
        marker=data[i];i+=1
        if marker in (0xd8,0xd9,0x01) or 0xd0<=marker<=0xd7:continue
        size=int.from_bytes(data[i:i+2],'big')
        if size<2 or i+size>len(data):raise ValueError('invalid JPEG segment')
        if marker in (0xc0,0xc1,0xc2):
            h=int.from_bytes(data[i+3:i+5],'big');w=int.from_bytes(data[i+5:i+7],'big')
            if not 0<w<=4096 or not 0<h<=4096:raise ValueError('invalid dimensions')
            return w,h
        i+=size
    raise ValueError('missing JPEG dimensions')


def chunk(tag,data):
    return tag+struct.pack('<I',len(data))+data+(b'\0' if len(data)%2 else b'')


class MjpegAvi:
    """Small standard AVI muxer; keeps JPEG bytes unchanged (no re-encoding)."""
    def __init__(self,path,width,height,fps=15):
        self.path=Path(path);self.width=width;self.height=height;self.fps=fps
        self.index=[];self.maximum=0;self.stream=self.path.open('w+b')
        self._header(0,0)
        self.movi=self.stream.tell()-4
    def _header(self,frames,rate):
        avih=struct.pack('<14I',round(1e6/(rate/1000 if rate else self.fps)),0,0,0x10,frames,0,1,self.maximum,self.width,self.height,0,0,0,0)
        strh=struct.pack('<4s4sIHH8I4h',b'vids',b'MJPG',0,0,0,0,1000,rate or round(self.fps*1000),0,frames,self.maximum,0xffffffff,0,0,0,self.width,self.height)
        strf=struct.pack('<IiiHH4sIiiII',40,self.width,self.height,1,24,b'MJPG',self.width*self.height*3,0,0,0,0)
        hdrl=chunk(b'LIST',b'hdrl'+chunk(b'avih',avih)+chunk(b'LIST',b'strl'+chunk(b'strh',strh)+chunk(b'strf',strf)))
        self.stream.seek(0);self.stream.write(b'RIFF'+b'\0'*4+b'AVI '+hdrl+b'LIST'+b'\0'*4+b'movi')
    def write(self,jpeg):
        if jpeg_size(jpeg)!=(self.width,self.height):raise ValueError('camera resolution changed')
        offset=self.stream.tell()-self.movi
        self.stream.write(chunk(b'00dc',jpeg));self.index.append((offset,len(jpeg)))
        self.maximum=max(self.maximum,len(jpeg));self.stream.flush()
    def close(self,elapsed=None):
        if self.stream.closed:return
        end=self.stream.tell()
        self.stream.write(chunk(b'idx1',b''.join(struct.pack('<4sIII',b'00dc',0x10,o,n) for o,n in self.index)))
        final=self.stream.tell();rate=round(len(self.index)*1000/elapsed) if elapsed and elapsed>0 else 0
        self._header(len(self.index),rate)
        self.stream.seek(4);self.stream.write(struct.pack('<I',final-8))
        self.stream.seek(self.movi-4);self.stream.write(struct.pack('<I',end-self.movi))
        self.stream.close()


class DatasetRecorder:
    def __init__(self,base_url,directory,fps=30,segment_seconds=60,min_free_bytes=512*1024*1024):
        self.base_url=base_url.rstrip('/');self.directory=Path(directory).resolve()
        self.fps=fps;self.segment_seconds=segment_seconds;self.min_free_bytes=min_free_bytes
        self.control_lock=threading.Lock();self.lock=threading.Lock();self.stop_event=threading.Event();self.threads=[]
        self.info={'active':False,'session':'','directory':str(self.directory),'error':'','cameras':{}}
    def status(self):
        with self.lock:return json.loads(json.dumps(self.info))
    def start(self):
        with self.control_lock:return self._start()
    def _start(self):
        if any(t.is_alive() for t in self.threads):raise ValueError('录制仍在运行或正在停止')
        with urlopen(self.base_url+'/api/status',timeout=2) as r:state=json.load(r)
        sources={'down':state.get('down_source',''),'front':state.get('front_source','')}
        if not sources['down'].startswith('csi:') or not sources['front'].startswith('/dev/v4l/by-id/') or not state.get('front_enabled'):
            raise ValueError('请先部署CSI下视、USB前视双摄配置；相机来源尚未确认')
        self.directory.mkdir(parents=True,exist_ok=True)
        if shutil.disk_usage(self.directory).free<self.min_free_bytes:raise ValueError('磁盘剩余空间不足')
        session=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.root=self.directory/session;self.root.mkdir()
        manifest={'session':session,'started_utc':datetime.now(timezone.utc).isoformat(),'sources':sources,'format':'MJPEG AVI','timestamp_clock':'Pi monotonic seconds; PC UTC receive time','nominal_fps':self.fps,'segment_seconds':self.segment_seconds,'overlay':False}
        (self.root/'session.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
        self.stop_event.clear()
        with self.lock:self.info.update(active=True,session=session,error='',directory=str(self.root),cameras={k:{'frames':0,'error':''} for k in sources})
        self.threads=[threading.Thread(target=self._capture,args=(role,source),daemon=True) for role,source in sources.items()]
        for t in self.threads:t.start()
        return self.status()
    def stop(self):
        with self.control_lock:
            self.stop_event.set()
            return self.status()
    def close(self):
        self.stop()
        for t in self.threads:t.join(timeout=3)
    def _capture(self,role,source):
        writer=None;sidecar=None;segment=0;last_stamp=None;opened=0;total=0;failed=0
        response=None;poll_fallback=False;boundary=None;started=time.monotonic()
        try:
            while not self.stop_event.is_set():
                before=time.monotonic()
                try:
                    if not poll_fallback and response is None:
                        try:
                            response=urlopen(self.base_url+'/api/camera/'+role+'.mjpeg',timeout=1)
                            if response.headers.get_content_type()!='multipart/x-mixed-replace':
                                raise ValueError('invalid MJPEG stream type')
                            boundary=response.headers.get_param('boundary')
                            if not boundary or len(boundary)>100:raise ValueError('invalid MJPEG boundary')
                            boundary=boundary.encode('ascii')
                        except HTTPError as e:
                            e.close()
                            if e.code not in (404,501):raise
                            poll_fallback=True
                    if poll_fallback:
                        with urlopen(self.base_url+'/api/camera/'+role+'.jpg',timeout=1) as snapshot:
                            jpeg=snapshot.read(8*1024*1024+1)
                            stamp=snapshot.headers.get('X-Frame-Time-Monotonic')
                            actual=snapshot.headers.get('X-Camera-Source')
                    else:
                        jpeg,stamp,actual=multipart_frame(response,boundary)
                    if len(jpeg)>8*1024*1024:raise ValueError('JPEG exceeds size limit')
                    if actual!=source:raise ValueError('相机来源变更或缺少来源标记')
                    if stamp is None:raise ValueError('缺少帧时间标记，请升级runtime')
                    stamp=float(stamp)
                    if not math.isfinite(stamp) or stamp<0:raise ValueError("invalid frame timestamp")
                    if last_stamp is not None and stamp<last_stamp:
                        raise RuntimeError('相机时间倒退/runtime重启，请开启新采集会话')
                    if stamp!=last_stamp:
                        if shutil.disk_usage(self.root).free<self.min_free_bytes:raise OSError('disk reserve reached')
                        now=time.monotonic()
                        if writer and (now-opened>=self.segment_seconds or writer.stream.tell()>=512*1024*1024):
                            writer.close(now-opened);sidecar.close();writer=None
                        if not writer:
                            segment+=1;opened=now
                            prefix=self.root/f'{role}_{segment:04d}'
                            writer=MjpegAvi(prefix.with_suffix('.avi'),*jpeg_size(jpeg),self.fps)
                            sidecar=prefix.with_suffix('.frames.jsonl').open('w',encoding='utf-8')
                        writer.write(jpeg)
                        sidecar.write(json.dumps({'frame':len(writer.index)-1,'capture_monotonic_sec':stamp,'received_utc':datetime.now(timezone.utc).isoformat(),'source':source,'width':writer.width,'height':writer.height})+'\n');sidecar.flush()
                        total+=1;last_stamp=stamp
                    failed=0
                    with self.lock:self.info['cameras'][role].update(frames=total,error='',
                        received_fps=round(total/max(.001,time.monotonic()-started),2),
                        transport='JPEG polling fallback' if poll_fallback else 'MJPEG stream')
                except Exception as e:
                    if response:response.close();response=None
                    failed+=1
                    with self.lock:self.info['cameras'][role]['error']=str(e)
                    if failed>=15 or isinstance(e,OSError) and 'disk reserve' in str(e):
                        raise RuntimeError(role+': '+str(e)) from e
                if poll_fallback or failed:
                    self.stop_event.wait(max(.01 if failed else 0,1/self.fps-(time.monotonic()-before)))
        except Exception as e:
            self.stop_event.set()
            with self.lock:self.info['error']=str(e)
        finally:
            if response:response.close()
            try:
                if writer:writer.close(time.monotonic()-opened)
                if sidecar:sidecar.close()
            except Exception as e:
                with self.lock:self.info['error']='录像收尾失败：'+str(e)
            with self.lock:
                self.info['cameras'][role]['finished']=True
                if all(c.get('finished') for c in self.info['cameras'].values()):
                    self.info['active']=False
                    (self.root/'result.json').write_text(json.dumps(self.info,indent=2,ensure_ascii=False),encoding='utf-8')
