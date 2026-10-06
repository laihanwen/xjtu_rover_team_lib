#!/bin/sh
set -eu
# Update an already provisioned Pi. No apt, ARM, or motion commands.
id auv >/dev/null
test -f /etc/auv-runtime/runtime.yaml
test -f /etc/systemd/system/auv-rov.service
test -f /etc/systemd/system/auv-rov.service.d/30-manual-trial.conf
test -d /usr/local/lib/auv-rov
backup=/var/backups/auv-maintenance/$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p "$backup"
cp /etc/auv-runtime/runtime.yaml "$backup/runtime.yaml"
cp -a /usr/local/lib/auv-rov "$backup/rov"
cp /usr/local/bin/auv_runtime "$backup/auv_runtime"
systemctl stop auv-rov auv-runtime
cmake --install build --prefix /usr/local
cp tools/rov/*.py /usr/local/lib/auv-rov/
if [ "${1:-}" = --apply-pi-profile ]; then
    cp runtime/config/pi-rov.yaml /etc/auv-runtime/runtime.yaml
fi
chown root:auv /etc/auv-runtime/runtime.yaml
chmod 0640 /etc/auv-runtime/runtime.yaml
systemctl start auv-runtime auv-rov
echo "Installed; configuration backup: $backup. Manual shore calibration and explicit ARM still required."
