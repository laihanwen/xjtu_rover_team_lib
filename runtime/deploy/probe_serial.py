#!/usr/bin/env python3
"""Read-only probe that distinguishes the CRC AUV link from legacy A6/A7 telemetry."""

import argparse
import json
import os
from pathlib import Path
import select
import subprocess
import termios
import time


def config_serial_device(path):
    section = None
    for raw in Path(path).read_text().splitlines():
        line = raw.split(" #", 1)[0].rstrip()
        if not line or line.lstrip().startswith("#"):
            continue
        if not raw.startswith(" ") and line.endswith(":"):
            section = line[:-1]
        elif section == "serial" and line.startswith("  ") and ":" in line:
            key, value = line.strip().split(":", 1)
            if key == "device":
                return value.strip().strip("\"'")
    return ""


def crc16_ccitt(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xFFFF
    return value


def count_crc_frames(data):
    count = 0
    offset = 0
    while offset + 7 <= len(data):
        start = data.find(b"\xaa\x55", offset)
        if start < 0 or start + 7 > len(data):
            break
        payload_length = data[start + 4]
        size = 7 + payload_length
        if payload_length <= 64 and start + size <= len(data):
            frame = data[start:start + size]
            expected = int.from_bytes(frame[-2:], "little")
            if frame[2] == 1 and crc16_ccitt(frame[2:-2]) == expected:
                count += 1
                offset = start + size
                continue
        offset = start + 1
    return count


def count_legacy_frames(data):
    counts = {"a6_imu": 0, "a7_depth": 0}
    offset = 0
    while offset < len(data):
        header = data[offset]
        size = 8 if header == 0xA6 else (6 if header == 0xA7 else 0)
        if size and offset + size <= len(data):
            frame = data[offset:offset + size]
            if sum(frame[1:-1]) & 0xFF == frame[-1]:
                counts["a6_imu" if header == 0xA6 else "a7_depth"] += 1
                offset += size
                continue
        offset += 1
    return counts


def service_active(name):
    try:
        result = subprocess.run(
            ["systemctl", "is-active", "--quiet", name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        return result.returncode == 0
    except OSError:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="/etc/auv-runtime/runtime.yaml")
    parser.add_argument("--device", help="overrides serial.device from the config")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--service", default="auv-runtime.service")
    args = parser.parse_args()
    if args.duration < 1 or args.duration > 60:
        parser.error("--duration must be between 1 and 60 seconds")
    if service_active(args.service):
        parser.error(f"stop {args.service} before probing so two readers do not consume the same UART")
    device = args.device or config_serial_device(args.config)
    if not device:
        parser.error("serial device is empty; pass --device /dev/serial0")

    fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
    original = termios.tcgetattr(fd)
    attributes = termios.tcgetattr(fd)
    attributes[0] = 0
    attributes[1] = 0
    attributes[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attributes[3] = 0
    attributes[4] = termios.B115200
    attributes[5] = termios.B115200
    attributes[6][termios.VMIN] = 0
    attributes[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attributes)
    received = bytearray()
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline-time.monotonic())
            ready, _, _ = select.select([fd], [], [], min(0.2, remaining))
            if ready:
                chunk = os.read(fd, 4096)
                if chunk:
                    received.extend(chunk)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, original)
        os.close(fd)

    crc_frames = count_crc_frames(received)
    legacy = count_legacy_frames(received)
    legacy_frames = legacy["a6_imu"] + legacy["a7_depth"]
    if crc_frames and legacy_frames:
        protocol = "mixed"
    elif crc_frames:
        protocol = "auv_crc"
    elif legacy_frames:
        protocol = "legacy_a6_a7"
    else:
        protocol = "unknown"
    result = {
        "device": device,
        "duration_sec": args.duration,
        "bytes_received": len(received),
        "protocol": protocol,
        "auv_crc_frames": crc_frames,
        **legacy,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if protocol in ("auv_crc", "legacy_a6_a7") else 1)


if __name__ == "__main__":
    main()
