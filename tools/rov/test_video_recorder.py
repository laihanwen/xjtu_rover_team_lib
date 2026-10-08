import base64,json,struct,tempfile,threading,time,unittest,io
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from video_recorder import DatasetRecorder,MjpegAvi,jpeg_size,multipart_frame
JPEG=base64.b64decode("/9j/4AAQSkZJRgABAgAAAQABAAD//gAQTGF2YzYxLjE5LjEwMAD/2wBDAAgEBAQEBAUFBQUFBQYGBgYGBgYGBgYGBgYHBwcICAgHBwcGBgcHCAgICAkJCQgICAgJCQoKCgwMCwsODg4RERT/xABNAAEBAAAAAAAAAAAAAAAAAAAABwEBAQEAAAAAAAAAAAAAAAAAAAUHEAEAAAAAAAAAAAAAAAAAAAAAEQEAAAAAAAAAAAAAAAAAAAAA/8AAEQgAGAAgAwEiAAIRAAMRAP/aAAwDAQACEQMRAD8AjgDf0sAAAAAB/9k=")
class RecorderTests(unittest.TestCase):
 def test_start_failure_remains_in_status(self):
  with tempfile.TemporaryDirectory() as d:
   r=DatasetRecorder('http://127.0.0.1:1',d)
   with self.assertRaises(OSError):r.start()
   self.assertFalse(r.status()['active'])
   self.assertIn('录制启动失败',r.status()['error'])
 def test_avi_index_and_header(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'clip.avi';w=MjpegAvi(p,32,24)
   w.write(JPEG);w.write(JPEG);w.close(.2)
   b=p.read_bytes();self.assertEqual(b[:4],b'RIFF');self.assertEqual(struct.unpack_from('<I',b,4)[0],len(b)-8)
   self.assertIn(b'idx1',b);self.assertEqual(b.count(b'00dc'),4)
   self.assertEqual(jpeg_size(JPEG),(32,24))
 def test_invalid_jpeg_is_rejected(self):
  with self.assertRaises(ValueError):jpeg_size(b'invalid')
 def serve(self,duplicate=False,wrong=False,stream=False):
  visits=[]
  class Handler(BaseHTTPRequestHandler):
   def do_GET(self):
    visits.append(self.path)
    if self.path=='/api/status':
     b=json.dumps({'down_source':'csi:0' if not wrong else '/dev/v4l/by-id/usb','front_source':'/dev/v4l/by-id/usb','front_enabled':True,'camera_calibration':{'down':{'preview_rectified':True,'preview_distortion_coefficients':[0,0,0,0,0]}}}).encode();self.send_response(200);self.end_headers();self.wfile.write(b)
    elif self.path in ('/api/camera/down.jpg','/api/camera/front.jpg'):
     self.send_response(200);self.send_header('X-Camera-Source','csi:0' if '/down.' in self.path else '/dev/v4l/by-id/usb');self.send_header('X-Frame-Time-Monotonic',str(1 if duplicate else time.monotonic()));self.end_headers();self.wfile.write(JPEG)
    elif stream and self.path in ('/api/camera/down.mjpeg','/api/camera/front.mjpeg'):
     self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=auvframe');self.end_headers()
     try:
      for _ in range(90):
       source='csi:0' if '/down.' in self.path else '/dev/v4l/by-id/usb'
       body=(f'--auvframe\r\nContent-Type: image/jpeg\r\nContent-Length: {len(JPEG)}\r\nX-Camera-Source: {source}\r\nX-Frame-Time-Monotonic: {time.monotonic()}\r\n\r\n').encode()+JPEG+b'\r\n'
       # Exercise packet fragmentation instead of assuming a frame == a TCP read.
       for i in range(0,len(body),71):self.wfile.write(body[i:i+71])
       self.wfile.flush();time.sleep(1/30)
     except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError):pass
    else:self.send_error(404)
   def log_message(self,*args):pass
  server=ThreadingHTTPServer(('127.0.0.1',0),Handler);threading.Thread(target=server.serve_forever,daemon=True).start()
  self.addCleanup(server.server_close);self.addCleanup(server.shutdown)
  return 'http://127.0.0.1:'+str(server.server_port),visits
 def test_dual_capture_rotation_and_finalize(self):
  url,visits=self.serve()
  with tempfile.TemporaryDirectory() as d:
   r=DatasetRecorder(url,d,fps=30,segment_seconds=.15,min_free_bytes=0);r.start()
   try:
    with self.assertRaises(ValueError):r.start()
    time.sleep(.45)
   finally:r.close()
   state=r.status();self.assertFalse(state['active']);self.assertEqual(state['error'],'')
   self.assertGreater(state['cameras']['down']['frames'],3);self.assertGreater(state['cameras']['front']['frames'],3)
   files=list(Path(state['directory']).glob('*.avi'));self.assertGreaterEqual(len(files),4)
   self.assertTrue((Path(state['directory'])/'result.json').exists())
   metadata=json.loads((Path(state['directory'])/'session.json').read_text())
   self.assertTrue(metadata['camera_calibration']['down']['preview_rectified'])
   self.assertEqual(metadata['camera_calibration']['down']['preview_distortion_coefficients'],[0,0,0,0,0])
   self.assertTrue(all(p in ('/api/status','/api/camera/down.jpg','/api/camera/front.jpg','/api/camera/down.mjpeg','/api/camera/front.mjpeg') for p in visits))
   for p in files:self.assertIn(b'idx1',p.read_bytes())
 def test_stream_uses_one_connection_per_camera_and_measures_fps(self):
  url,visits=self.serve(stream=True)
  with tempfile.TemporaryDirectory() as d:
   r=DatasetRecorder(url,d,min_free_bytes=0);r.start();time.sleep(.55);r.close()
   s=r.status();self.assertEqual(s['error'],'');self.assertFalse(s['active'])
   for role in ('down','front'):
    self.assertEqual(visits.count('/api/camera/'+role+'.mjpeg'),1)
    self.assertNotIn('/api/camera/'+role+'.jpg',visits)
    self.assertGreater(s['cameras'][role]['frames'],10)
    self.assertGreater(s['cameras'][role]['received_fps'],20)
    self.assertEqual(s['cameras'][role]['transport'],'MJPEG stream')
 def test_multipart_bounds_and_truncation(self):
  header=b'--auvframe\r\nContent-Type: image/jpeg\r\nContent-Length: '
  for body in (header+b'9000000\r\n\r\n',header+b'5\r\n\r\nxx',b'--wrong\r\n',b'--auvframe\r\n'+b'x'*5000):
   with self.assertRaises(ValueError):multipart_frame(io.BytesIO(body),b'auvframe')
 def test_duplicate_source_frames_not_repeated(self):
  url,_=self.serve(duplicate=True)
  with tempfile.TemporaryDirectory() as d:
   r=DatasetRecorder(url,d,fps=30,min_free_bytes=0);r.start();time.sleep(.2);r.close()
   self.assertEqual(r.status()['cameras']['down']['frames'],1)
   self.assertEqual(r.status()['cameras']['front']['frames'],1)
 def test_unconfirmed_camera_roles_rejected(self):
  url,_=self.serve(wrong=True)
  with tempfile.TemporaryDirectory() as d:
   r=DatasetRecorder(url,d,min_free_bytes=0)
   with self.assertRaises(ValueError):r.start()
   self.assertFalse(r.status()['active'])
 def test_independent_ffmpeg_decoder(self):
  try:import imageio_ffmpeg
  except ImportError:self.skipTest('optional independent decoder unavailable')
  import subprocess
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'clip.avi';w=MjpegAvi(p,32,24)
   for _ in range(3):w.write(JPEG)
   w.close(.3)
   result=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-i',str(p),'-f','rawvideo','-pix_fmt','gray','-'],capture_output=True,check=True)
   self.assertEqual(result.stderr,b'');self.assertEqual(len(result.stdout),32*24*3)
if __name__=='__main__':unittest.main()
