#!/bin/sh
set -eu
# Run on the Raspberry Pi from the repository root. Uses configured SSH keys only.
if [ ! -f src/auv_core/CMakeLists.txt ]; then
  echo 'Run from the AUV repository root' >&2
  exit 2
fi
sudo apt-get update
sudo apt-get install -y cmake ninja-build g++ pkg-config libopencv-dev libyaml-cpp-dev libcpp-httplib-dev ffmpeg
if ! getent group auv >/dev/null 2>&1; then
  sudo groupadd --system auv
fi
if ! id auv >/dev/null 2>&1; then
  sudo useradd --system --create-home --gid auv --groups video,dialout auv
fi
operator_user=$(id -un)
if [ "$operator_user" != root ]; then
  sudo usermod -aG auv "$operator_user"
fi
cmake -S . -B build-lightweight -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build-lightweight -j 3
ctest --test-dir build-lightweight --output-on-failure
if [ -f /etc/auv-runtime/runtime.yaml ]; then
  backup_config=$(mktemp)
  sudo cp /etc/auv-runtime/runtime.yaml "$backup_config"
  trap 'rm -f "$backup_config"' EXIT
  restore_config=1
else
  restore_config=0
fi
sudo cmake --install build-lightweight --prefix /usr/local
sudo install -d -m 0750 -o auv -g auv /etc/auv-runtime
if [ "$restore_config" -eq 1 ]; then
  sudo cp "$backup_config" /etc/auv-runtime/runtime.yaml
else
  sudo cp /usr/local/etc/auv-runtime/runtime.yaml /etc/auv-runtime/runtime.yaml
fi
sudo chown root:auv /etc/auv-runtime/runtime.yaml
sudo chmod 0640 /etc/auv-runtime/runtime.yaml
sudo cp runtime/deploy/auv-runtime.service /etc/systemd/system/auv-runtime.service
sudo systemctl daemon-reload
sudo systemctl enable --now auv-runtime.service
printf '%s\n' 'Service installed. It starts DISARMED. Edit /etc/auv-runtime/runtime.yaml with measured hardware values.'
printf '%s\n' 'Reconnect SSH to activate auv group membership before using auvctl.'
