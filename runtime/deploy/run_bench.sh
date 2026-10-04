#!/bin/sh
set -eu

root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
reports=${AUV_REPORT_DIR:-"$HOME/auv-test-reports"}
mkdir -p "$reports"

usage() {
  echo 'usage: run_bench.sh preflight|camera|serial|endurance|collect|fault-watch [seconds|fault-kind]' >&2
  echo 'All checks are read only. START, ARM and hardware fault injection remain manual.' >&2
  exit 2
}

[ "$#" -ge 1 ] || usage
stage=$1
shift
case "$stage" in
  preflight|collect)
    [ "$#" -eq 0 ] || usage
    exec python3 "$root/runtime/deploy/bench_checks.py" "$stage" --output-dir "$reports"
    ;;
  camera|serial)
    duration=${1:-15}
    [ "$#" -le 1 ] || usage
    exec python3 "$root/runtime/deploy/bench_checks.py" "$stage" --duration "$duration" --output-dir "$reports"
    ;;
  fault-watch)
    [ "$#" -ge 1 ] && [ "$#" -le 2 ] || usage
    kind=$1
    timeout=${2:-15}
    exec python3 "$root/runtime/deploy/bench_checks.py" fault-watch --expect "$kind" --timeout "$timeout" --output-dir "$reports"
    ;;
  endurance)
    duration=${1:-1800}
    [ "$#" -le 1 ] || usage
    report="$reports/$(date -u +%Y%m%dT%H%M%SZ)_endurance.json"
    python3 "$root/runtime/deploy/acceptance.py" --duration "$duration" --output "$report"
    ;;
  *) usage ;;
esac
