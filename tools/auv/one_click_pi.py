"""SSH preparation/finalization for the explicit one-click firmware workflow."""
import argparse
import os
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools/rov'))
from maintenance import remote

PREPARE = '''import time,serial,subprocess,yaml
from pathlib import Path
from trial_protocol import encode,Parser,decode_status
config=yaml.safe_load(Path('/etc/auv-runtime/runtime.yaml').read_text())
if config['serial'].get('device') or config['motion']['motion_commands_enabled'] or config['operation'].get('auto_start') or config['operation'].get('auto_arm'):
 raise RuntimeError('Existing ROV camera-only configuration required; no configuration overwritten')
for u in ['auv-runtime','auv-rov']:subprocess.run(['systemctl','cat',u],check=True,capture_output=True)
service=subprocess.run(['systemctl','show','auv-runtime','-p','ExecStart'],check=True,capture_output=True,text=True).stdout
if '/etc/auv-runtime/runtime.yaml' not in service:raise RuntimeError('Runtime service configuration mismatch')
units=['auv-rov','auv-runtime','auv-observation','auv-task-one','auv-tag-docking']
for u in units:
 if subprocess.run(['systemctl','cat',u],capture_output=True).returncode==0:
  subprocess.run(['systemctl','stop',u],check=True)
p=Parser();neutral=set();last_sequence=None;deadline=time.monotonic()+5
with serial.Serial(DEVICE,115200,timeout=.05,exclusive=True) as port:
 while time.monotonic()<deadline:
  seq=int(time.monotonic()*1000)&0xffffffff
  port.write(encode(1,seq.to_bytes(4,'little')+b'\\x00'*4))
  port.write(encode(2,seq.to_bytes(4,'little')+b'\\x00'))
  for kind,payload in p.push(port.read(4096)):
   if kind==0x80:
    s=decode_status(payload)
    if last_sequence is not None and ((s['sequence']-last_sequence)&0xffffffff) not in range(1,0x80000000):continue
    last_sequence=s['sequence']
    if not s['armed'] and len(s.get('outputs',[]))==8 and all(abs(v)<=.001 for v in s['outputs']):neutral.add(payload)
    else:neutral.clear()
  if len(neutral)>=3:break
 else:raise RuntimeError('MCU DISARM/neutral confirmation failed; services remain stopped')
print('MCU DISARM and eight neutral outputs confirmed; services stopped')
'''

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('stage',choices=['prepare','finish'])
    p.add_argument('--host',default='192.168.137.150')
    p.add_argument('--user',default='pi')
    p.add_argument('--device',default='/dev/serial0')
    p.add_argument('--log-dir',required=True)
    a=p.parse_args()
    import paramiko
    logs=Path(a.log_dir);logs.mkdir(parents=True,exist_ok=True)
    known=ROOT/'build/maintenance/known_hosts';known.parent.mkdir(parents=True,exist_ok=True)
    password=os.environ.get('AUV_DEPLOY_PASSWORD')
    client=paramiko.SSHClient();client.load_system_host_keys()
    if known.exists():client.load_host_keys(str(known))
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(a.host,username=a.user,password=password,timeout=10,auth_timeout=10,banner_timeout=10)
        client.save_host_keys(str(known))
        # Only these source helpers are uploaded; measured Pi configurations stay in place.
        remote_dir='/tmp/auv-one-click-tools'
        remote(client,'mkdir -p '+remote_dir+'/auv '+remote_dir+'/rov',logs/'ssh.log')
        with client.open_sftp() as ftp:
            for source,target in [('tools/auv/switch_mode.py','auv/switch_mode.py'),('tools/rov/trial_protocol.py','rov/trial_protocol.py')]:
                ftp.put(str(ROOT/source),remote_dir+'/'+target)
        if a.stage=='prepare':
            script='DEVICE='+repr(a.device)+'\n'+PREPARE
            command='PYTHONPATH='+remote_dir+'/rov python3 -c '+shlex.quote(script)
        else:
            command='python3 '+remote_dir+'/auv/switch_mode.py rov --device '+shlex.quote(a.device)
        remote(client,command,logs/(a.stage+'.log'),password,elevated=True)
    finally:client.close()

if __name__=='__main__':main()
