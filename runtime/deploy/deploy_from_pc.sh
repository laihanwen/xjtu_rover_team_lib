#!/bin/sh
set -eu
# Usage: deploy_from_pc.sh pi-user@192.168.137.201 /home/pi/auv
# SSH uses an existing key and refuses password login. sudo may prompt on the Pi.
if [ "$#" -ne 2 ]; then
  echo 'usage: deploy_from_pc.sh SSH_HOST REMOTE_CHECKOUT_DIR' >&2
  exit 2
fi
host=$1
remote_dir=$2
case "$host" in *[!a-zA-Z0-9_.@:-]*|'') echo 'invalid SSH host' >&2; exit 2;; esac
case "$remote_dir" in *[!a-zA-Z0-9_./-]*|'') echo 'invalid remote path' >&2; exit 2;; esac
case "$remote_dir" in /*) ;; *) echo 'remote path must be absolute' >&2; exit 2;; esac
ssh -o BatchMode=yes "$host" "mkdir -p '$remote_dir'"
tar -czf - CMakeLists.txt runtime core docs/calibration |
  ssh -o BatchMode=yes "$host" "tar -xzf - -C '$remote_dir'"
ssh -o BatchMode=yes -tt "$host" "cd '$remote_dir' && runtime/deploy/install_pi.sh"
