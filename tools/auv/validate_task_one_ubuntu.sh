#!/usr/bin/env bash
# Isolated software validation only: no SSH, flashing or physical UART.
set -uo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$root" || exit 1
build=${1:-build-task-one-ubuntu}
python=${PYTHON:-python3}
jobs=${JOBS:-3}
stamp=$(date -u +%Y%m%dT%H%M%SZ)
evidence="$root/build/ubuntu-task-one-evidence/$stamp"
mkdir -p "$evidence" || exit 1
export AUV_SIM_EVIDENCE_DIR="$evidence/runtime"
mkdir -p "$AUV_SIM_EVIDENCE_DIR"
failed=0
run_check() {
  local name=$1
  shift
  printf '\n[%s]\n' "$name"
  "$@" 2>&1 | tee "$evidence/$name.log"
  local result=${PIPESTATUS[0]}
  printf '%s\t%s\n' "$name" "$result" >> "$evidence/checks.tsv"
  if (( result != 0 )); then failed=1; fi
  return "$result"
}
git rev-parse HEAD > "$evidence/source-commit.txt"
git status --porcelain=v1 > "$evidence/source-status.txt"
git diff --binary HEAD > "$evidence/source-working-tree.patch"
uname -a > "$evidence/environment.txt"
cmake --version >> "$evidence/environment.txt"
"$python" --version >> "$evidence/environment.txt" 2>&1
run_check python-dependencies "$python" -c 'import cv2,numpy,yaml; print("OpenCV",cv2.__version__); print("NumPy",numpy.__version__)' || exit 1
run_check configure cmake -S . -B "$build" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DAUV_PERCEPTION_ONLY=OFF "-DPython3_EXECUTABLE=$python" || exit 1
run_check build cmake --build "$build" -j "$jobs" || exit 1
run_check registered-tests ctest --test-dir "$build" -N
# Missing Python dependencies must not silently omit the Runtime fixtures.
if ! grep -q 'local_task_one_closed_loop' "$evidence/registered-tests.log"; then
  printf 'Required local Runtime fixture was not registered.\n' | tee "$evidence/missing-test.log"
  exit 1
fi
run_check ctest ctest --test-dir "$build" --output-on-failure
run_check sensor-recording "$python" -m unittest discover -s tools/rov -p test_sensor_recorder.py
run_check sensor-replay "$python" -m unittest discover -s tools/auv -p test_prepare_sensor_replay.py
run_check training-manifest "$python" -m unittest discover -s vision/training -p test_train_cone_body.py
run_check rov-protocol "$python" -m unittest discover -s tools/rov -p test_trial.py
run_check video-recording "$python" -m unittest discover -s tools/rov -p test_video_recorder.py
if [[ -f "$build/Testing/Temporary/LastTest.log" ]]; then
  cp -- "$build/Testing/Temporary/LastTest.log" "$evidence/LastTest.log"
fi
find "$build" -type f \( -name auv_runtime -o -name auv_local_mapping_replay -o -name auv_task_video_replay \) -exec sha256sum {} + > "$evidence/binaries.sha256"
find "$evidence" -type f ! -name SHA256SUMS -exec sha256sum {} + > "$evidence/SHA256SUMS"
printf '\nEvidence: %s\n' "$evidence"
printf 'Overall failed: %s (0 = passed; see skipped tests separately)\n' "$failed"
exit "$failed"
