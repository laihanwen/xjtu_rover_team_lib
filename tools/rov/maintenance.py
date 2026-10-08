"""Repeatable hardware maintenance. Never sends ARM or motion commands."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import tarfile
import time
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[2]


def flash(args, output):
    print('Loading flash dependencies...', flush=True)
    from intelhex import IntelHex
    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.memory_map import RamRegion
    from pyocd.flash.file_programmer import FileProgrammer
    from pyocd.flash.flash import Flash
    from pyocd.target.pack.flash_algo import PackFlashAlgo
    image_path = ROOT / 'firmware/stm32/MDK-ARM/Copy_cup/Copy_cup.hex'
    image = IntelHex(str(image_path))
    segments = image.segments()
    if not segments or not all(0x08000000 <= a < b <= 0x08008000 for a, b in segments):
        raise ValueError('HEX must fit the verified first 32 KiB firmware region')
    pack = Path(args.pack)
    print('Preparing SRAM flash algorithm...', flush=True)
    algo = PackFlashAlgo(str(pack / 'Flash/STM32F4xx_1024.FLM')).get_pyocd_flash_algo(
        1024, RamRegion(start=0x20000000, length=0x20000))
    print(f'Connecting probe: {args.probe}', flush=True)
    session = ConnectHelper.session_with_chosen_probe(
        unique_id=args.probe, target_override='stm32f405rg', connect_mode='halt',
        frequency=25000, resume_on_disconnect=False, auto_unlock=False, no_config=True,
        options={'pack': str(pack), 'cmsis_dap.limit_packets': True,
                 'hide_programming_progress': True, 'reset_type': 'sw_sysresetreq'})
    if session is None:
        raise RuntimeError('Requested debug probe unavailable')
    with session:
        print('Connected; resetting and reading fresh backup...', flush=True)
        target = session.target
        target.reset_and_halt()
        def read_firmware():
            # PTD02HW is reliable with small transfers; a single 32 KiB read
            # can fail after programming even when the flash contents are valid.
            data = bytearray()
            for address in range(0x08000000, 0x08008000, 32):
                data.extend(target.read_memory_block8(address, 32))
            return bytes(data)
        original = read_firmware()
        (output / 'preflash-32k.bin').write_bytes(original)
        print('Backup saved; programming...', flush=True)
        merged = bytearray(original)
        for a, b in segments:
            merged[a-0x08000000:b-0x08000000] = image.tobinarray(start=a, end=b-1).tobytes()
        binary = output / 'program-32k.bin'
        binary.write_bytes(merged)
        def replace(memory_map):
            for region in memory_map:
                if region.is_flash and region.flash is not None:
                    region.algo = algo
                    region.flash = Flash(target, algo)
                    region.flash.region = region
                    region.flash.double_buffer_supported = False
                if region.has_subregions:
                    replace(region.submap)
        replace(target.memory_map)
        target.write32(0xE0042008, target.read32(0xE0042008) | (1 << 12))
        FileProgrammer(session, chip_erase='sector', smart_flash=False,
                       trust_crc=False, keep_unwritten=False).program(str(binary), base_address=0x08000000)
        target.halt()
        print('Programming finished; verifying full readback...', flush=True)
        if read_firmware() != bytes(merged):
            raise RuntimeError('Full 32 KiB readback mismatch; target remains halted')
        report = {'hex_sha256': hashlib.sha256(image_path.read_bytes()).hexdigest(),
                  'backup_sha256': hashlib.sha256(original).hexdigest(),
                  'verified': True, 'arm_sent': False}
        (output / 'flash.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        target.reset()
    print(json.dumps(report))


def remote(client, command, log, password=None, elevated=False):
    if elevated:
        command = ("sudo -S -p '' sh -c " if password is not None else
                   'sudo -n sh -c ') + shlex.quote(command)
    stdin, stdout, stderr = client.exec_command(command, timeout=600)
    if password is not None:
        stdin.write(password + '\n')
        stdin.flush()
        stdin.channel.shutdown_write()
    # Drain both streams while the command runs to avoid SSH window deadlocks.
    channel = stdout.channel
    deadline = time.monotonic() + 900
    with log.open('a', encoding='utf-8') as stream:
        while True:
            if time.monotonic() > deadline:
                channel.close()
                raise TimeoutError('Remote step exceeded 15 minutes; inspect Pi before retrying')
            for ready, read in ((channel.recv_ready, channel.recv),
                                (channel.recv_stderr_ready, channel.recv_stderr)):
                if ready():
                    text = read(65536).decode('utf-8', 'replace')
                    print(text, end='', flush=True)
                    stream.write(text)
            if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
                break
            time.sleep(.05)
    code = channel.recv_exit_status()
    if code:
        raise RuntimeError(f'Remote command failed: exit {code}; see {log}')


def pi(args, output, deploy=False):
    import paramiko
    password = os.environ.get('AUV_DEPLOY_PASSWORD')
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    known = ROOT / 'build/maintenance/known_hosts'
    if known.exists():
        client.load_host_keys(str(known))
    # First connection pins the key; later key changes are rejected.
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, username=args.user, password=password, timeout=10,
                   auth_timeout=10, banner_timeout=10)
    client.save_host_keys(str(known))
    try:
        if deploy:
            archive = output / 'source.tar.gz'
            with tarfile.open(archive, 'w:gz') as tar:
                paths = ['CMakeLists.txt', 'core', 'runtime', 'tools/rov']
                def exclude(info):
                    return None if '__pycache__' in Path(info.name).parts or info.name.endswith('.pyc') else info
                for path in paths:
                    tar.add(ROOT / path, arcname=path, filter=exclude)
            directory = '/tmp/auv-maintenance-' + output.name
            remote(client, 'mkdir -p ' + shlex.quote(directory), output / 'deploy.log')
            with client.open_sftp() as sftp:
                sftp.put(str(archive), directory + '/source.tar.gz')
            remote(client, f'cd {shlex.quote(directory)} && tar -xzf source.tar.gz && '
                   'cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON && '
                   'cmake --build build -j 3 && ctest --test-dir build --output-on-failure && '
                   'python3 -c "import serial"', output / 'deploy.log')
            # Existing service identities and measured configuration are prerequisites.
            command = f'cd {shlex.quote(directory)} && sh tools/rov/deploy/maintenance_pi.sh'
            if args.apply_pi_profile:
                command += ' --apply-pi-profile'
            remote(client, command, output / 'deploy.log', password, elevated=True)
        remote(client, "systemctl is-active auv-runtime auv-rov && "
               "systemctl show auv-runtime auv-rov -p ExecStart -p ActiveEnterTimestamp && "
               "python3 -c \"import json,urllib.request; "
               "d=json.load(urllib.request.urlopen('http://192.168.137.150:8080/api/status',timeout=5)); "
               "print(json.dumps(d))\"", output / 'check.log')
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['flash','deploy','check','all'])
    parser.add_argument('--probe', default='ATK 20190528')
    parser.add_argument('--pack', default='C:/Keil_v5/ARM/PACK/Keil/STM32F4xx_DFP/1.0.8')
    parser.add_argument('--host', default='192.168.137.150')
    parser.add_argument('--user', default='pi')
    parser.add_argument('--apply-pi-profile', action='store_true')
    args = parser.parse_args()
    output = ROOT / 'build/maintenance' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    output.mkdir(parents=True)
    print(f'Logs: {output}', flush=True)
    try:
        if args.action in ('flash','all'):
            flash(args, output)
        if args.action in ('deploy','check','all'):
            pi(args, output, deploy=args.action != 'check')
    except Exception as error:
        (output / 'failure.txt').write_text(str(error), encoding='utf-8')
        parser.exit(1, f'FAILED: {error}\n')


if __name__ == '__main__':
    main()
