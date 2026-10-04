# Lightweight mission-one runtime

This C++ process shares `auv_core` with the ROS nodes. It implements the first task only: AprilTag, 3×3 grid, cone classification, route planning and grid traversal. STM32 retains attitude/depth PID, mixer and the hardware heartbeat failsafe.

## Native build

On Debian 13 / Raspberry Pi OS with `cmake`, `ninja-build`, `g++`, `libopencv-dev`, `libyaml-cpp-dev`, `libcpp-httplib-dev` and `ffmpeg` installed:

```sh
cmake -S . -B build-lightweight -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-lightweight -j 3
ctest --test-dir build-lightweight --output-on-failure
```

Native CTest covers core mapping/planning/status validation and PTY serial faults. If the system Python also has OpenCV and NumPy, it adds a synthetic AprilTag/grid/cone video replay and a virtual STM32 ARM/ACK/limit/leak/process-exit test. These tests use only a pseudoterminal, never a real serial port.

Copy and edit `runtime/config/runtime.yaml`. The sample refuses motion: `motion_commands_enabled: false`, empty serial device, uncalibrated control directions and camera intrinsics. A production camera source must be a stable `/dev/v4l/by-id/...` symlink. `file:/absolute/path/video.mp4` can be used for offline replay, but motion remains disabled without calibration. The process starts in INIT and DISARM, and requires `auvctl start`; it never auto arms or resumes movement after a fault or restart.

```sh
./build-lightweight/runtime/auv_runtime runtime/config/runtime.yaml
./build-lightweight/runtime/auvctl status
./build-lightweight/runtime/auvctl start
./build-lightweight/runtime/auvctl disarm
```

`auvctl` uses `/run/auv-runtime/control.sock` (mode `0660`) and is intended over SSH. The SSH user must belong to the `auv` group. ARM additionally requires `auvctl arm --confirm SAFE_TO_ARM`, a fresh safe STM32 STATUS, VISIT_CONES, calibrated motion config and explicit motion enablement. `pause`, `abort`, `disarm` and faults withdraw motion and request DISARM. STM32 heartbeat loss remains the final safety barrier if Linux exits abruptly.

The web page at `http://192.168.137.201:8080/` is read only. `hls.js` is packaged locally. Video uses FFmpeg `h264_v4l2m2m`, 640×480 at 20 fps, 2 Mbit/s and 0.5 s HLS segments. If encoding fails, status reports video degradation. Software `libx264` requires `video.software_fallback_enabled: true` or an explicit `video.encoder: libx264` change. HTTP is also degraded if cpp-httplib is absent at build time or the wired address is unavailable. Neither failure stops mission control. NDJSON records timestamped events, route and periodic state; it rotates by size or day. The completed grid debug frame is saved under `logging.debug_dir`.

## Deployment

On the Pi checkout, run `runtime/deploy/install_pi.sh`. It installs dependencies, builds, tests, installs the service and starts it DISARMED. Existing `/etc/auv-runtime/runtime.yaml` is preserved. Check `systemctl status auv-runtime`, `journalctl -u auv-runtime`, and `/var/log/auv-runtime/events.ndjson`. Use SSH keys for remote access; this repository contains no password handling.

From the PC, `runtime/deploy/deploy_from_pc.sh pi-user@192.168.137.201 /home/pi/auv` copies the current checkout over SSH and runs the Pi installer. SSH password login is disabled for the script; remote `sudo` may prompt through the terminal.

Use the staged, read-only Pi checks in [deploy/TESTING.md](deploy/TESTING.md). `run_bench.sh preflight` verifies deployment safety and reports missing hardware as pending; `camera`, `serial`, `fault-watch`, `endurance`, and `collect` provide focused checks and saved JSON reports. The endurance script calls `acceptance.py`, measures vision/control/heartbeat rates, rolling frame latency, process-tree CPU and RSS, temperature and throttling, and requires a valid camera and STM32 status. Missing throttling data does not count as a pass.

Do not set `motion_commands_enabled: true` until the camera calibration, row/column to body-frame signs, speed limit and serial device have been measured. First motion tests must have thruster power disconnected, propellers removed or thrusters firmly secured. The Debian 13 native build and two Pi CTests passed on 2026-10-04. The 30-minute thermal/performance run, real camera/HLS test, real STM32 bench run and no-prop closed-loop acceptance remain pending hardware.
