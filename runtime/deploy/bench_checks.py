#!/usr/bin/env python3
"""Read-only, staged checks for the deployed Raspberry Pi runtime.

Only Python's standard library is required. This tool never sends a control
command, including START, ARM or DISARM.
"""
import argparse
import datetime as dt
import json
import os
from pathlib import Path
import socket
import stat
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.parse import urljoin, urlparse
from urllib.request import urlopen


def runtime_status(path):
    with socket.socket(socket.AF_UNIX) as peer:
        peer.settimeout(3)
        peer.connect(path)
        peer.sendall(b"status")
        chunks = []
        while True:
            chunk = peer.recv(16384)
            if not chunk:
                break
            chunks.append(chunk)
            if b"\n" in chunk:
                break
    return json.loads(b"".join(chunks))


def config_values(path):
    """Read the simple scalar fields needed here without a YAML dependency."""
    values = {}
    section = None
    for raw in Path(path).read_text().splitlines():
        line = raw.split(" #", 1)[0].rstrip()
        if not line or line.lstrip().startswith("#"):
            continue
        if not raw.startswith(" ") and line.endswith(":"):
            section = line[:-1]
        elif section and line.startswith("  ") and not line.startswith("    ") and ":" in line:
            key, value = line.strip().split(":", 1)
            values[f"{section}.{key}"] = value.strip().strip("\"'")
    return values


def http_get(url):
    with urlopen(url, timeout=4) as response:
        return response.status, response.read(), dict(response.headers)


def check(name, passed, detail, required=True):
    return {"name": name, "result": "PASS" if passed else ("FAIL" if required else "PENDING"),
            "detail": str(detail)}


def preflight(args, config):
    checks = []
    service = subprocess.run(["systemctl", "is-active", args.service], capture_output=True, text=True)
    checks.append(check("service", service.returncode == 0, service.stdout.strip() or service.stderr.strip()))
    try:
        status = runtime_status(args.socket)
        checks.append(check("control_socket", True, args.socket))
    except (OSError, ValueError) as exc:
        status = None
        checks.append(check("control_socket", False, exc))
    try:
        mode = stat.S_IMODE(os.stat(args.socket).st_mode)
        checks.append(check("socket_mode", mode == 0o660, oct(mode)))
    except OSError as exc:
        checks.append(check("socket_mode", False, exc))
    checks.append(check("motion_disabled", config.get("motion.motion_commands_enabled") == "false",
                        config.get("motion.motion_commands_enabled", "missing")))
    if status:
        checks.append(check("disarmed", status.get("armed") is False, status.get("armed")))
        checks.append(check("mission_idle", status.get("phase") == "INIT", status.get("phase")))
        checks.append(check("no_fault", not status.get("fault"), status.get("fault", "")))
    try:
        code, _, _ = http_get(args.web_url.rstrip("/") + "/api/status")
        checks.append(check("web_status", code == 200, f"HTTP {code}"))
    except (OSError, URLError, ValueError) as exc:
        checks.append(check("web_status", False, exc))
    camera = config.get("camera.source", "")
    serial = config.get("serial.device", "")
    checks.append(check("camera_device", camera.startswith("/dev/v4l/by-id/") and Path(camera).exists(),
                        camera or "not configured", required=args.strict_hardware))
    checks.append(check("serial_device", bool(serial) and Path(serial).exists(),
                        serial or "not configured", required=args.strict_hardware))
    return checks, {"status": status, "camera_device": camera, "serial_device": serial}


def sample_status(path, duration, period=1.0):
    first = runtime_status(path)
    samples = [first]
    start = time.monotonic()
    while time.monotonic() - start < duration:
        time.sleep(min(period, max(0.0, duration - (time.monotonic() - start))))
        samples.append(runtime_status(path))
    return samples, time.monotonic() - start


def camera_check(args, config):
    device = config.get("camera.source", "")
    checks = [check("stable_camera_device", device.startswith("/dev/v4l/by-id/") and Path(device).exists(), device)]
    samples, elapsed = sample_status(args.socket, args.duration)
    first, last = samples[0], samples[-1]
    hz = (last["vision_frames"] - first["vision_frames"]) / elapsed
    checks.extend([
        check("disarmed", all(s.get("armed") is False for s in samples), "all samples"),
        check("vision_rate", hz >= args.min_vision_hz, f"{hz:.2f} Hz"),
        check("fresh_frames", all(0 <= s.get("camera_age_sec", -1) < args.max_frame_age
                                  for s in samples[1:]), f"limit {args.max_frame_age}s"),
        check("video_pipeline", not last.get("video_degraded") and not last.get("web_detail"),
              last.get("video_detail") or last.get("web_detail") or "healthy"),
    ])
    url = args.web_url.rstrip("/") + "/hls/index.m3u8"
    try:
        code, body, _ = http_get(url)
        playlist = body.decode("utf-8")
        entries = [line.strip() for line in playlist.splitlines() if line.strip() and not line.startswith("#")]
        checks.append(check("hls_playlist", code == 200 and "#EXTM3U" in playlist and bool(entries),
                            f"HTTP {code}, {len(entries)} segments"))
        if entries:
            segment_url = urljoin(url, entries[-1])
            if urlparse(segment_url).netloc != urlparse(url).netloc:
                raise ValueError("HLS segment points outside the Pi")
            segment_code, segment, _ = http_get(segment_url)
            checks.append(check("hls_segment", segment_code == 200 and len(segment) > 0,
                                f"HTTP {segment_code}, {len(segment)} bytes"))
        else:
            checks.append(check("hls_segment", False, "playlist has no segment"))
    except (OSError, URLError, ValueError, UnicodeError) as exc:
        checks.append(check("hls_playlist", False, exc))
    return checks, {"vision_hz": round(hz, 2), "start_status": first, "end_status": last}


def serial_check(args, config):
    device = config.get("serial.device", "")
    checks = [check("serial_device", bool(device) and Path(device).exists(), device or "not configured")]
    checks.append(check("motion_disabled", config.get("motion.motion_commands_enabled") == "false",
                        config.get("motion.motion_commands_enabled", "missing")))
    samples, elapsed = sample_status(args.socket, args.duration)
    first, last = samples[0], samples[-1]
    hz = (last["heartbeats"] - first["heartbeats"]) / elapsed
    checks.extend([
        check("disarmed", all(s.get("armed") is False for s in samples), "all samples"),
        check("serial_connected", all(s.get("serial") is True for s in samples[1:]), "all later samples"),
        check("telemetry_valid", all(s.get("telemetry_valid") is True for s in samples[1:]), "all later samples"),
        check("safe_status", all(not s.get("leak") and s.get("error_flags") == 0
                                 for s in samples), "no leak or error flags"),
        check("heartbeat_rate", hz >= args.min_heartbeat_hz, f"{hz:.2f} Hz"),
        check("no_fault", not last.get("fault"), last.get("fault", "")),
    ])
    return checks, {"heartbeat_hz": round(hz, 2), "start_status": first, "end_status": last}


def fault_watch(args, _config):
    timeline = []
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        sample = runtime_status(args.socket)
        timeline.append({"time_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
                         "phase": sample.get("phase"), "armed": sample.get("armed"),
                         "serial": sample.get("serial"), "camera_age_sec": sample.get("camera_age_sec"),
                         "leak": sample.get("leak"), "error_flags": sample.get("error_flags"),
                         "fault": sample.get("fault")})
        if sample.get("phase") == "FAULT":
            break
        time.sleep(0.1)
    last = timeline[-1]
    fault_text = str(last["fault"]).lower()
    expected = {"camera": "camera" in fault_text,
                "serial": "stm32" in fault_text or "serial" in fault_text,
                "leak": last["leak"] is True,
                "error": isinstance(last["error_flags"], int) and last["error_flags"] != 0}[args.expect]
    checks = [check("expected_fault", last["phase"] == "FAULT" and expected,
                    last["fault"] or "timeout without fault"),
              check("disarmed", last["armed"] is False, last["armed"])]
    return checks, {"timeline": timeline}


def collect(args, config):
    result = {"config_fields": {name: config.get(name, "") for name in
              ("camera.source", "serial.device", "motion.motion_commands_enabled",
               "video.encoder", "web.bind", "web.port")}}
    try:
        result["status"] = runtime_status(args.socket)
    except (OSError, ValueError) as exc:
        result["status_error"] = str(exc)
    for name, command in {
        "service": ["systemctl", "status", args.service, "--no-pager", "-l"],
        "journal": ["journalctl", "-u", args.service, "-b", "--no-pager", "-n", "100"],
        "devices": ["sh", "-c", "ls -l /dev/v4l/by-id /dev/serial/by-id 2>&1"],
        "temperature": ["vcgencmd", "measure_temp"],
        "throttling": ["vcgencmd", "get_throttled"],
    }.items():
        try:
            completed = subprocess.run(command, capture_output=True, text=True, timeout=5)
            result[name] = {"exit_code": completed.returncode,
                            "output": completed.stdout + completed.stderr}
        except (OSError, subprocess.TimeoutExpired) as exc:
            result[name] = str(exc)
    return [], result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("preflight", "camera", "serial", "fault-watch", "collect"))
    parser.add_argument("--config", default="/etc/auv-runtime/runtime.yaml")
    parser.add_argument("--socket", default="/run/auv-runtime/control.sock")
    parser.add_argument("--web-url", default="http://192.168.137.201:8080")
    parser.add_argument("--service", default="auv-runtime.service")
    parser.add_argument("--output-dir", type=Path, default=Path.home() / "auv-test-reports")
    parser.add_argument("--duration", type=float, default=15)
    parser.add_argument("--min-vision-hz", type=float, default=10)
    parser.add_argument("--min-heartbeat-hz", type=float, default=19)
    parser.add_argument("--max-frame-age", type=float, default=0.5)
    parser.add_argument("--strict-hardware", action="store_true")
    parser.add_argument("--expect", choices=("camera", "serial", "leak", "error"))
    parser.add_argument("--timeout", type=float, default=15)
    args = parser.parse_args(argv)
    if args.duration < 2 or args.timeout < 1:
        parser.error("duration must be >= 2 seconds and timeout >= 1 second")
    if args.stage == "fault-watch" and not args.expect:
        parser.error("fault-watch requires --expect")
    try:
        config = config_values(args.config)
        function = {"preflight": preflight, "camera": camera_check, "serial": serial_check,
                    "fault-watch": fault_watch, "collect": collect}[args.stage]
        checks, details = function(args, config)
    except (OSError, ValueError, KeyError) as exc:
        checks, details = [check("execution", False, exc)], {}
    report = {"timestamp_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
              "stage": args.stage, "checks": checks, "details": details,
              "passed": all(item["result"] != "FAIL" for item in checks)}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    name = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + args.stage
    path = args.output_dir / (name + ".json")
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    for item in checks:
        print(f"{item['result']:7} {item['name']}: {item['detail']}")
    print(f"Report: {path}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
