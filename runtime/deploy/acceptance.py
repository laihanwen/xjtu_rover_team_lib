#!/usr/bin/env python3
"""Read-only 30-minute Pi resource and runtime timing acceptance sampler."""
import argparse
import json
import os
import pathlib
import socket
import subprocess
import time


def status(path):
    with socket.socket(socket.AF_UNIX) as peer:
        peer.settimeout(2)
        peer.connect(path)
        peer.sendall(b'status')
        return json.loads(peer.recv(16384))


def cpu_ticks(pid):
    fields = pathlib.Path(f'/proc/{pid}/stat').read_text().split()
    return int(fields[13]) + int(fields[14])


def process_tree(pid):
    result = [pid]
    for current in result:
        try:
            children = pathlib.Path(f'/proc/{current}/task/{current}/children').read_text().split()
            result.extend(int(child) for child in children)
        except OSError:
            pass
    return result


def resource_totals(pid):
    pids = process_tree(pid)
    ticks = memory = 0
    for current in pids:
        try:
            ticks += cpu_ticks(current)
            memory += rss_mib(current)
        except OSError:
            pass
    return ticks, memory


def rss_mib(pid):
    for line in pathlib.Path(f'/proc/{pid}/status').read_text().splitlines():
        if line.startswith('VmRSS:'):
            return int(line.split()[1]) / 1024
    return 0.0


def temperature():
    values = []
    for path in pathlib.Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        try:
            value = float(path.read_text()) / 1000
            if 0 < value < 150:
                values.append(value)
        except (OSError, ValueError):
            pass
    return max(values) if values else None


def throttled():
    try:
        output = subprocess.check_output(['vcgencmd', 'get_throttled'], timeout=2, text=True)
        return int(output.split('=')[1].strip(), 16)
    except (FileNotFoundError, subprocess.SubprocessError, ValueError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=int, default=1800)
    parser.add_argument('--socket', default='/run/auv-runtime/control.sock')
    parser.add_argument('--pid', type=int)
    parser.add_argument('--output', type=pathlib.Path, help='also write the JSON result to this file')
    args = parser.parse_args()
    if args.duration < 10:
        parser.error('--duration must be at least 10 seconds')
    if args.pid is None:
        args.pid = int(subprocess.check_output(
            ['systemctl', 'show', '-p', 'MainPID', '--value', 'auv-runtime'], text=True).strip())
    if args.pid <= 0:
        parser.error('auv-runtime is not running')
    interval = os.sysconf('SC_CLK_TCK')
    started = time.monotonic()
    previous = (started, resource_totals(args.pid)[0], status(args.socket))
    cpu_samples, memory_samples, temp_samples, latency_samples = [], [], [], []
    control_rates, heartbeat_rates, vision_rates = [], [], []
    faults, throttling_seen = [], []
    throttling_available = True
    while time.monotonic() - started < args.duration:
        time.sleep(1)
        now = time.monotonic()
        ticks, memory = resource_totals(args.pid)
        state = status(args.socket)
        elapsed = now - previous[0]
        cpu_samples.append((ticks - previous[1]) / interval / elapsed * 100)
        memory_samples.append(memory)
        control_rates.append((state['control_ticks'] - previous[2]['control_ticks']) / elapsed)
        heartbeat_rates.append((state['heartbeats'] - previous[2]['heartbeats']) / elapsed)
        vision_rates.append((state['vision_frames'] - previous[2]['vision_frames']) / elapsed)
        if state['vision_latency_p99_ms'] >= 0:
            latency_samples.append(state['vision_latency_p99_ms'])
        temp = temperature()
        if temp is not None:
            temp_samples.append(temp)
        throttle = throttled()
        if throttle is None:
            throttling_available = False
        if throttle:
            throttling_seen.append(throttle)
        if state['fault']:
            faults.append(state['fault'])
        if state['armed']:
            faults.append('unexpected ARM during endurance check')
        if not state['serial'] or not state['telemetry_valid']:
            faults.append('STM32 STATUS unavailable or invalid')
        if state['camera_age_sec'] < 0 or state['camera_age_sec'] > .5:
            faults.append('camera frame stale')
        if state['leak'] or state['error_flags']:
            faults.append('STM32 unsafe status')
        if state['log_degraded']:
            faults.append('event logging degraded')
        if state['video_degraded']:
            faults.append('video: ' + state['video_detail'])
        if state['web_detail']:
            faults.append('web: ' + state['web_detail'])
        previous = now, ticks, state
    metrics = {
        'duration_sec': round(time.monotonic() - started, 1),
        'mean_vision_hz': sum(vision_rates) / len(vision_rates),
        'mean_control_hz': sum(control_rates) / len(control_rates),
        'mean_heartbeat_hz': sum(heartbeat_rates) / len(heartbeat_rates),
        'max_rolling_latency_p99_ms': max(latency_samples) if latency_samples else None,
        'max_rss_mib': max(memory_samples),
        'mean_cpu_percent_one_core_100': sum(cpu_samples) / len(cpu_samples),
        'max_temp_c': max(temp_samples) if temp_samples else None,
        'throttling_flags': throttling_seen,
        'throttling_checked': throttling_available,
        'faults': sorted(set(faults)),
    }
    metrics['passed'] = (
        metrics['mean_vision_hz'] >= 10 and metrics['mean_control_hz'] >= 19 and
        metrics['mean_heartbeat_hz'] >= 19 and
        metrics['max_rolling_latency_p99_ms'] is not None and
        metrics['max_rolling_latency_p99_ms'] < 250 and
        metrics['max_rss_mib'] < 700 and metrics['mean_cpu_percent_one_core_100'] < 300 and
        metrics['max_temp_c'] is not None and metrics['max_temp_c'] < 75 and
        throttling_available and not throttling_seen and not faults
    )
    output = json.dumps(metrics, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output)
    print(output, end='')
    raise SystemExit(0 if metrics['passed'] else 1)


if __name__ == '__main__':
    main()
