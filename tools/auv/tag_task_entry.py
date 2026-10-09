"""Windows SSH launcher for the independent tag test; never ARM."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/rov'))
from maintenance import remote


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('check', 'start', 'task'))
    parser.add_argument('--log-dir', required=True)
    args = parser.parse_args()
    image = ROOT / 'firmware/stm32/MDK-ARM/AUV_TAG_DOCK/AUV_TAG_DOCK.hex'
    expected = hashlib.sha256(image.read_bytes()).hexdigest()
    records = sorted((ROOT / 'build/maintenance').glob('*/flash.json'))
    if not records:
        raise RuntimeError('缺少本地烧录校验记录；此入口不执行烧录')
    latest = json.loads(records[-1].read_text())
    if latest.get('verified') is not True or latest.get('hex_sha256') != expected:
        raise RuntimeError('最近烧录记录与当前 AUV_TAG_DOCK 不匹配；不能仅切换 Pi 服务')
    import paramiko
    logs = Path(args.log_dir)
    logs.mkdir(parents=True, exist_ok=True)
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    known = ROOT / 'build/maintenance/known_hosts'
    if known.exists():
        client.load_host_keys(str(known))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    password = os.environ.get('AUV_DEPLOY_PASSWORD')
    try:
        client.connect('192.168.137.150', username='pi', password=password, timeout=10, auth_timeout=10)
        script = (ROOT / 'tools/auv/tag_task_remote.py').read_text(encoding='utf-8')
        command = 'python3 -c ' + shlex.quote(script) + ' ' + shlex.quote(args.action) + ' --expected-firmware ' + expected
        remote(client, command, logs / (args.action + '.log'), password, elevated=True)
    finally:
        client.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('入口失败：' + str(error))
        raise SystemExit(1)
