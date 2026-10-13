#!/usr/bin/env python3
"""Switch dedicated boot recording/route services without changing runtime.yaml.

The route still needs MCU safety readiness and a fresh shore-side level
calibration after every STM32 restart. Depth stability alone cannot prove water.
"""
import argparse
import copy
import os
from pathlib import Path
import subprocess
import sys

import yaml


SOURCE = Path("/etc/auv-runtime/pi-auv-tag-docking.yaml")
TARGET = Path("/etc/auv-runtime/pi-auv-tag-docking-autoboot.yaml")
UNIT = Path("/etc/systemd/system/auv-tag-docking-autoboot.service")
ACTIVE = "auv-tag-docking-autoboot.service"
ROUTE_SOURCE = Path("/etc/auv-runtime/pi-auv-boot-tag-route.yaml")
ROUTE_TEMPLATE = Path("/usr/local/etc/auv-runtime/pi-auv-boot-tag-route.yaml")
ROUTE_UNIT = Path("/etc/systemd/system/auv-boot-tag-route.service")
ROUTE_ACTIVE = "auv-boot-tag-route.service"
OTHER = ("auv-rov.service", "auv-runtime.service", "auv-observation.service",
         "auv-task-one.service", "auv-tag-docking.service", ACTIVE, ROUTE_ACTIVE)
LATCH_OWNERS = ("auv-runtime.service", "auv-observation.service",
                "auv-task-one.service", "auv-tag-docking.service", ACTIVE, ROUTE_ACTIVE)


def command(*args):
    subprocess.run(args, check=True)


def read_yaml(path):
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or data.get("mission", {}).get("profile") != "tag_docking":
        raise ValueError(f"{path} must be a tag_docking configuration")
    return data


def recording_config(source):
    data = copy.deepcopy(source)
    data["operation"].update(mode="debug", auto_start=False, auto_arm=False)
    data["motion"].update(motion_commands_enabled=False)
    data["recording"].update(enabled=True, required_for_motion=True)
    data["web"]["enabled"] = True
    # 'video.enabled' controls HLS only. The onboard raw MJPEG recorder above
    # starts with the process and writes both camera indexes.
    return data


def write_config(source, target):
    data = recording_config(read_yaml(source))
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)


def render_unit(template, runtime_bin):
    if not runtime_bin.is_file() or not os.access(runtime_bin, os.X_OK):
        raise FileNotFoundError(f"runtime binary unavailable: {runtime_bin}")
    text = template.read_text(encoding="utf-8")
    old = "ExecStart=/usr/local/bin/auv_runtime "
    if text.count(old) != 1:
        raise ValueError("unexpected autoboot unit template")
    return text.replace(old, f"ExecStart={runtime_bin} ")


def require_root():
    if os.geteuid() != 0:
        raise PermissionError("run enable-record/disable with sudo on the Pi")


def ensure_disarmed():
    ctl = Path("/usr/local/bin/auvctl")
    if not ctl.exists():
        raise RuntimeError("auvctl unavailable; verify DISARM before changing services")
    state = subprocess.run((str(ctl), "status"), capture_output=True, text=True)
    if state.returncode:
        raise RuntimeError("cannot confirm DISARM from auvctl; do not switch live service")
    import json
    if json.loads(state.stdout).get("armed") is not False:
        raise RuntimeError("STM32 reports ARM; use emergency auvctl disarm first")


def enable_record(args):
    import grp
    require_root()
    ensure_disarmed()
    group = grp.getgrnam("auv").gr_gid
    staged = TARGET.with_name(TARGET.name + ".candidate")
    write_config(args.source, staged)
    os.chown(staged, 0, group)
    staged.chmod(0o640)
    try:
        command("runuser", "-u", "auv", "--", str(args.runtime_bin), "--check-config", str(staged))
        os.replace(staged, TARGET)
    finally:
        staged.unlink(missing_ok=True)
    UNIT.write_text(render_unit(args.unit_template, args.runtime_bin), encoding="utf-8")
    UNIT.chmod(0o644)
    # Existing /etc units may still use Preserve=restart. A manual service
    # switch must not delete the once-per-boot autonomous-started latch.
    for service in LATCH_OWNERS:
        dropin = Path("/etc/systemd/system") / (service + ".d") / "10-preserve-autonomous-latch.conf"
        dropin.parent.mkdir(parents=True, exist_ok=True)
        dropin.write_text("[Service]\nRuntimeDirectoryPreserve=yes\n", encoding="utf-8")
        dropin.chmod(0o644)
    command("systemctl", "daemon-reload")
    for service in OTHER:
        if service == ACTIVE:
            continue
        found = subprocess.run(("systemctl", "cat", service), capture_output=True)
        if found.returncode == 0:
            command("systemctl", "disable", "--now", service)
    command("systemctl", "enable", ACTIVE)
    command("systemctl", "restart", ACTIVE)
    print(f"Recording-only autoboot enabled: {TARGET}; automatic START/ARM disabled")


def enable_route(args):
    import grp
    require_root()
    ensure_disarmed()
    chosen = ROUTE_SOURCE if ROUTE_SOURCE.exists() and args.route_source == ROUTE_TEMPLATE else args.route_source
    config = yaml.safe_load(chosen.read_text(encoding="utf-8-sig"))
    if chosen == ROUTE_TEMPLATE:
        # Carry device-specific paths from the currently commissioned manual
        # service; keep the dedicated trial's motion limits and authorization.
        manual = read_yaml(SOURCE)
        for section in ("camera", "camera_front", "serial", "web", "logging", "recording", "localization"):
            config[section] = copy.deepcopy(manual[section])
    config["web"]["enabled"] = True
    if config.get("mission", {}).get("profile") != "boot_tag_route" or not config.get("boot_tag_route", {}).get("explicit_motion_authorization"):
        raise ValueError("route configuration lacks dedicated one-shot trial authorization")
    device = Path(config.get("serial", {}).get("device", ""))
    if "REPLACE" in str(device) or not device.exists() or not device.is_char_device():
        raise ValueError("replace serial.device with the real STM32 character device first")
    if not config.get("operation", {}).get("auto_start") or not config.get("operation", {}).get("auto_arm"):
        raise ValueError("route configuration must request one-shot automatic START and ARM")
    candidate = ROUTE_SOURCE.with_name(ROUTE_SOURCE.name + ".candidate")
    candidate.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")
    try:
        os.chown(candidate, 0, grp.getgrnam("auv").gr_gid)
        candidate.chmod(0o640)
        command("runuser", "-u", "auv", "--", str(args.runtime_bin), "--check-config", str(candidate))
        os.replace(candidate, ROUTE_SOURCE)
    finally:
        candidate.unlink(missing_ok=True)
    ROUTE_UNIT.write_text(render_unit(args.route_unit_template, args.runtime_bin), encoding="utf-8")
    ROUTE_UNIT.chmod(0o644)
    for service in LATCH_OWNERS:
        dropin = Path("/etc/systemd/system") / (service + ".d") / "10-preserve-autonomous-latch.conf"
        dropin.parent.mkdir(parents=True, exist_ok=True)
        dropin.write_text("[Service]\nRuntimeDirectoryPreserve=yes\n", encoding="utf-8")
        dropin.chmod(0o644)
    command("systemctl", "daemon-reload")
    for service in OTHER:
        if service == ROUTE_ACTIVE:
            continue
        if subprocess.run(("systemctl", "cat", service), capture_output=True).returncode == 0:
            command("systemctl", "disable", "--now", service)
    command("systemctl", "enable", "--now", ROUTE_ACTIVE)
    print(f"One-shot route service enabled: {ROUTE_SOURCE}; MCU gates still apply")


def disable():
    require_root()
    ensure_disarmed()
    manual = read_yaml(SOURCE)
    if manual["operation"]["mode"] != "debug" or manual["operation"]["auto_start"] or manual["operation"]["auto_arm"]:
        raise RuntimeError("manual tag-docking configuration is not debug/explicit-start")
    for service in (ACTIVE, ROUTE_ACTIVE):
        if subprocess.run(("systemctl", "cat", service), capture_output=True).returncode == 0:
            command("systemctl", "disable", "--now", service)
    command("systemctl", "enable", "--now", "auv-tag-docking.service")
    print("Manual tag-docking service restored; existing runtime.yaml untouched")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("prepare-record", "enable-record", "enable-route", "disable"))
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=TARGET,
                        help="prepare-record only: output YAML path")
    parser.add_argument("--runtime-bin", type=Path, default=Path("/usr/local/bin/auv_runtime"))
    parser.add_argument("--route-source", type=Path, default=ROUTE_TEMPLATE)
    parser.add_argument("--route-unit-template", type=Path,
                        default=Path("/usr/local/lib/systemd/system/auv-boot-tag-route.service"))
    parser.add_argument("--unit-template", type=Path,
                        default=Path("/usr/local/lib/systemd/system/auv-tag-docking-autoboot.service"))
    args = parser.parse_args()
    try:
        if args.action == "prepare-record":
            write_config(args.source, args.output)
            print(args.output)
        elif args.action == "enable-record":
            enable_record(args)
        elif args.action == "enable-route":
            enable_route(args)
        else:
            disable()
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"autoboot: {error}\n")


if __name__ == "__main__":
    main()
