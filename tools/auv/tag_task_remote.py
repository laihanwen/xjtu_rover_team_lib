"""Pi-side tag test preflight. No ARM, firmware write or configuration changes."""
import argparse
import json
from pathlib import Path
import shlex
import socket
import subprocess
import time

UNIT = 'auv-tag-docking.service'
CONFIG = '/etc/auv-runtime/pi-auv-tag-docking.yaml'
CONFLICTS = ('auv-rov', 'auv-runtime', 'auv-observation', 'auv-task-one')


def run(*args):
    return subprocess.run(args, capture_output=True, text=True, timeout=20)


def validate_config(config):
    if config.get('mission', {}).get('profile') != 'tag_docking':
        raise RuntimeError('配置不是标签测试任务')
    operation = config.get('operation', {})
    if operation.get('mode') != 'debug' or operation.get('auto_start') is not False or operation.get('auto_arm') is not False:
        raise RuntimeError('入口要求 debug、auto_start=false、auto_arm=false')
    if not config.get('serial', {}).get('device'):
        raise RuntimeError('配置没有 UART，不能用于实机检查')


def control(command):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
        conn.settimeout(3)
        conn.connect('/run/auv-runtime/control.sock')
        conn.sendall(command.encode())
        chunks = []
        size = 0
        while True:
            data = conn.recv(65536)
            if not data:
                break
            chunks.append(data)
            size += len(data)
            if size > 4 * 1024 * 1024:
                raise RuntimeError('状态响应超出限制')
        response = b''.join(chunks).decode()
        if response.startswith('ERR'):
            raise RuntimeError(response.strip())
        return response


def task_ready(status):
    if status.get('mission_profile') != 'tag_docking' or status.get('operation_mode') != 'debug':
        raise RuntimeError('运行中的任务或操作模式不匹配')
    if status.get('armed') is not False:
        raise RuntimeError('MCU 未确认 DISARM')
    required = ('serial', 'status_fresh', 'safe_status', 'depth_sample_fresh', 'recording_ready', 'origin_ready')
    missing = [key for key in required if status.get(key) is not True]
    if status.get('tag_docking', {}).get('startup_ready') is not True:
        missing.append('tag_docking.startup_ready')
    if missing:
        raise RuntimeError('任务未就绪：' + ', '.join(missing))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('check', 'start', 'task'))
    parser.add_argument('--expected-firmware', required=True)
    args = parser.parse_args()
    import yaml
    if not Path(CONFIG).is_file():
        raise RuntimeError('尚未部署标签任务配置 ' + CONFIG + '；请先完成 Runtime 构建、测试和部署')
    config = yaml.safe_load(Path(CONFIG).read_text())
    validate_config(config)
    if config.get('release', {}).get('firmware_hex_sha256', '').lower() != args.expected_firmware.lower():
        raise RuntimeError('部署记录中的固件与本地已校验 AUV_TAG_DOCK 固件不匹配')
    fragment = run('systemctl', 'show', UNIT, '-p', 'FragmentPath', '--value')
    if fragment.returncode or not fragment.stdout.strip():
        raise RuntimeError('尚未部署 auv-tag-docking 服务，请先完成 Runtime 构建、测试和部署')
    if run('systemctl', 'show', UNIT, '-p', 'DropInPaths', '--value').stdout.strip():
        raise RuntimeError('服务存在覆盖配置，需核对后再使用入口')
    unit = Path(fragment.stdout.strip()).read_text()
    commands = [line[len('ExecStart='):] for line in unit.splitlines() if line.startswith('ExecStart=')]
    if len(commands) != 1:
        raise RuntimeError('无法确认唯一 Runtime 启动命令')
    command = shlex.split(commands[0])
    if len(command) != 2 or command[1] != CONFIG or Path(command[0]).name != 'auv_runtime':
        raise RuntimeError('服务启动程序或配置路径不匹配')
    checked = run(command[0], '--check-config', CONFIG)
    if checked.returncode:
        raise RuntimeError('原生配置校验失败：' + checked.stdout + checked.stderr)
    print(checked.stdout.strip())
    conflicts = [u for u in CONFLICTS if run('systemctl', 'is-active', '--quiet', u).returncode == 0]
    if conflicts:
        raise RuntimeError('存在冲突服务，入口不会强行停止：' + ', '.join(conflicts))
    active = run('systemctl', 'is-active', '--quiet', UNIT).returncode == 0
    if args.action == 'start' and not active:
        started = run('systemctl', 'start', UNIT)
        if started.returncode:
            raise RuntimeError('服务启动失败：' + started.stderr)
        active = True
        deadline = time.monotonic() + 15
        while not Path('/run/auv-runtime/control.sock').exists():
            if time.monotonic() >= deadline or run('systemctl', 'is-active', '--quiet', UNIT).returncode:
                raise RuntimeError('Runtime 未建立控制接口，请检查 journalctl -u auv-tag-docking')
            time.sleep(.25)
    if not active:
        raise RuntimeError('配置校验通过，但服务未启动；使用启动入口进入待命')
    status = json.loads(control('status'))
    if status.get('mission_profile') != 'tag_docking' or status.get('operation_mode') != 'debug' or status.get('auto_start') is not False or status.get('auto_arm') is not False:
        raise RuntimeError('当前运行进程与配置不匹配')
    print(json.dumps(status, ensure_ascii=False, indent=2))
    if args.action == 'task':
        task_ready(status)
        print(control('start').strip())
        print('仅请求进入 WAIT_ARM，未 ARM。当前任务约 5 秒等待 ARM，超时会故障。')
    else:
        try:
            task_ready(status)
            print('任务启动检查通过；尚未请求 start 或 ARM。')
        except RuntimeError as error:
            print('待命/观测可用，但 ' + str(error))
            if args.action == 'check':
                raise


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('检查/启动失败：' + str(error))
        raise SystemExit(1)
