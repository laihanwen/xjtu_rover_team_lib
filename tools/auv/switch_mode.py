"""Pi local, explicit DISARM-only session mode switch. Never requests ARM.

Run from this checkout after installing the dual firmware and both services.
No mode or service changes occur merely by importing this module.
"""
import argparse
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'rov'))
from trial_protocol import encode, Parser


def select_mcu(port, mode, timeout=5, clock=time.monotonic):
    """Prove neutral output, then require matching ACK and post-command STATUS."""
    parser = Parser()
    deadline = clock()+timeout
    sequence = time.monotonic_ns()//1000000 & 0xffffffff
    next_send = 0
    neutral = set()
    last_status_sequence = None
    command_sent = False
    mode_ack = False
    selected_at = 0
    while clock() < deadline:
        now = clock()
        if now >= next_send:
            port.write(encode(1, struct.pack('<II', sequence, 0)))
            port.write(encode(2, struct.pack('<IB', sequence, 0)))
            next_send = now+.1
        for kind, payload in parser.push(port.read(4096)):
            if kind == 0x7f and len(payload) == 6 and payload[0] == 9:
                if struct.unpack_from('<I', payload, 2)[0] != sequence:
                    continue
                if payload[1] != 0:
                    raise RuntimeError(f'MCU rejected mode request: {payload[1]}')
                mode_ack = True
            if kind != 0x80 or len(payload) != 46 or payload[29] != 8:
                continue
            flags = payload[4]
            if not flags & 16:
                raise RuntimeError('MCU has no dual-mode capability; retain DISARM')
            frame_sequence = struct.unpack_from('<I', payload)[0]
            if last_status_sequence is not None and ((frame_sequence-last_status_sequence)&0xffffffff) not in range(1, 0x80000000):
                continue
            last_status_sequence = frame_sequence
            outputs = struct.unpack_from('<8h', payload, 30)
            safe = not flags & 1 and all(abs(v) <= 1 for v in outputs)
            if not safe:
                neutral.clear()
                if command_sent:
                    raise RuntimeError('Non-neutral/armed state after mode selection')
                continue
            if not command_sent:
                neutral.add(frame_sequence)
                if len(neutral) >= 3:
                    port.write(encode(9, struct.pack('<IB', sequence, mode)))
                    command_sent = True
                    selected_at = clock()
            elif mode_ack and bool(flags & 32) == bool(mode) and clock()-selected_at >= .2:
                return 'auv' if mode else 'rov'
    raise RuntimeError('Mode switch timeout; services remain stopped and ARM was not requested')


def systemctl(*args):
    return subprocess.run(['systemctl', *args], check=True, capture_output=True, text=True).stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['rov', 'auv'])
    parser.add_argument('--device', default='/dev/serial0')
    parser.add_argument('--auv-unit', choices=['auv-tag-docking', 'auv-observation', 'auv-task-one'], default='auv-tag-docking')
    parser.add_argument('--auv-config', default='/etc/auv-runtime/pi-auv-tag-docking.yaml')
    args = parser.parse_args()
    if sys.platform != 'linux' or os.geteuid() != 0:
        parser.error('Run locally on the Pi with sudo; no services changed')
    import fcntl
    import serial
    import yaml
    with open('/run/auv-mode-switch.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        units = ['auv-rov', 'auv-runtime', 'auv-observation', 'auv-task-one', 'auv-tag-docking']
        targets = ['auv-rov', 'auv-runtime'] if args.mode == 'rov' else [args.auv_unit]
        config_path = '/etc/auv-runtime/runtime.yaml' if args.mode == 'rov' else args.auv_config
        config = yaml.safe_load(Path(config_path).read_text())
        if config['operation'].get('auto_start') or config['operation'].get('auto_arm'):
            raise RuntimeError('Switch entry requires auto_start/auto_arm disabled')
        device = config['serial'].get('device', '')
        if args.mode == 'rov' and (device or config['motion']['motion_commands_enabled']):
            raise RuntimeError('ROV Runtime must remain cameras-only')
        if args.mode == 'auv' and device != args.device:
            raise RuntimeError('AUV Runtime serial device differs from selected MCU')
        for unit in targets:
            systemctl('cat', unit)
        if config_path not in systemctl('show', 'auv-runtime' if args.mode == 'rov' else args.auv_unit, '-p', 'ExecStart'):
            raise RuntimeError('Selected service does not use the checked config')
        snapshot = {'mode': args.mode, 'monotonic_sec': time.monotonic(), 'previous': {}}
        for unit in units:
            snapshot['previous'][unit] = {}
            for action in ['is-active', 'is-enabled']:
                result = subprocess.run(['systemctl', action, unit], capture_output=True, text=True)
                snapshot['previous'][unit][action] = result.stdout.strip()
        directory = Path('/var/log/auv-runtime/mode-switches')
        directory.mkdir(parents=True, exist_ok=True)
        record = directory/f'{time.time_ns()}.json'
        record.write_text(json.dumps(snapshot, indent=2))
        # Disable competing autostarts too; reboot requires an explicit switch
        # again, because the MCU always boots ROV/DISARM.
        existing = [u for u in units if subprocess.run(
            ['systemctl', 'cat', u], capture_output=True).returncode == 0]
        try:
            for unit in existing:
                systemctl('stop', unit)
                systemctl('disable', unit)
            with serial.Serial(args.device, 115200, timeout=.05, exclusive=True) as port:
                selected = select_mcu(port, int(args.mode == 'auv'))
            for unit in targets:
                systemctl('start', unit)
                if systemctl('is-active', unit).strip() != 'active':
                    raise RuntimeError(f'{unit} did not start')
            snapshot['result'] = f'{selected}: DISARM waiting for explicit operator request'
            record.write_text(json.dumps(snapshot, indent=2))
            print(snapshot['result'])
        except Exception as error:
            for unit in targets:
                subprocess.run(['systemctl', 'stop', unit], check=False, capture_output=True)
            snapshot['error'] = str(error)
            record.write_text(json.dumps(snapshot, indent=2))
            raise


if __name__ == '__main__':
    main()
