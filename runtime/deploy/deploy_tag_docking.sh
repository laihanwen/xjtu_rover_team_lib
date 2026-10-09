#!/bin/sh
set -eu
# Run as root from a source checkout after native build/tests. Never ARM/start a mission.
test "$(id -u)" -eq 0
id auv >/dev/null
test -x build/runtime/auv_runtime
test -f /etc/auv-runtime/runtime.yaml
test -e /dev/serial0
stamp=$(date -u +%Y%m%dT%H%M%SZ)
identity=$(sha256sum build/runtime/auv_runtime | cut -c 1-12)
release=/opt/auv-tag-docking/20261008-$identity
backup=/var/backups/auv-tag-docking/$stamp
mkdir -p "$backup" "$release"
cp /etc/auv-runtime/runtime.yaml "$backup/runtime.yaml"
for unit in auv-runtime auv-rov auv-observation auv-task-one auv-tag-docking; do
    systemctl is-enabled "$unit" > "$backup/$unit.enabled" 2>&1 || true
    systemctl is-active "$unit" > "$backup/$unit.active" 2>&1 || true
done
if test -f /etc/auv-runtime/pi-auv-tag-docking.yaml; then
    cp /etc/auv-runtime/pi-auv-tag-docking.yaml "$backup/pi-auv-tag-docking.yaml"
fi
if test -f /etc/systemd/system/auv-tag-docking.service; then
    cp /etc/systemd/system/auv-tag-docking.service "$backup/auv-tag-docking.service"
fi
cmake --install build --prefix "$release"
python3 - "$release" "$backup" <<'PY'
from pathlib import Path
import sys, yaml
release, backup = map(Path, sys.argv[1:])
config=yaml.safe_load(Path('runtime/config/pi-auv-tag-docking.yaml').read_text())
actual=yaml.safe_load(Path('/etc/auv-runtime/runtime.yaml').read_text())
for role in ['camera','camera_front']:
    observed=actual[role]
    if (observed['width'],observed['height'])!=(config[role]['width'],config[role]['height']):
        raise RuntimeError('Measured camera resolution differs from tag docking calibration')
    config[role]=observed
config['serial']['device']='/dev/serial0'
config['web'].update(enabled=True,bind='192.168.137.150',assets=str(release/'share/auv-runtime/web'))
config['operation'].update(mode='debug',auto_start=False,auto_arm=False)
config['motion']['motion_commands_enabled']=False
config['release']['firmware_hex_sha256']=Path('firmware-tag-docking.sha256').read_text().strip()
config['release']['git_commit']=Path('release-commit.txt').read_text().strip()
config['release']['source_state']='local changes; see native validation and deployment records'
config['release']['calibration_id']=config['camera']['calibration_id']
candidate=backup/'candidate.yaml'
candidate.write_text(yaml.safe_dump(config,allow_unicode=True))
unit=Path('runtime/deploy/auv-tag-docking.service').read_text().replace(
    '/usr/local/bin/auv_runtime',str(release/'bin/auv_runtime'))
(backup/'candidate.service').write_text(unit)
PY
"$release/bin/auv_runtime" --check-config "$backup/candidate.yaml"
systemctl stop auv-rov auv-runtime
for unit in auv-observation auv-task-one auv-tag-docking; do
    if systemctl cat "$unit" >/dev/null 2>&1; then systemctl stop "$unit"; fi
done
# Keep startup ownership on the selected AUV service across reboots, still DISARM.
systemctl disable auv-rov auv-runtime
for unit in auv-observation auv-task-one; do
    if systemctl cat "$unit" >/dev/null 2>&1; then systemctl disable "$unit"; fi
done
install -o root -g auv -m 0640 "$backup/candidate.yaml" /etc/auv-runtime/pi-auv-tag-docking.yaml
install -m 0644 "$backup/candidate.service" /etc/systemd/system/auv-tag-docking.service
systemctl daemon-reload
systemctl enable auv-tag-docking
systemctl start auv-tag-docking
printf 'AUV waiting DISARM. Release: %s. Previous service/config state: %s\n' "$release" "$backup"
