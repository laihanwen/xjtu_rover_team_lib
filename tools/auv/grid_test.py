"""Quick entry for existing grid mapping/traversal tests; never controls hardware."""
import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
PATTERNS = {
    'mapping': r'^(local_task_one_closed_loop)$',
    'traversal': r'^local_stage_',
    'all': r'^(local_task_one_closed_loop|local_stage_.*|a2_closed_loop|grid_exit_closed_loop)$',
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', required=True, type=Path,
                        help='Existing full Linux Runtime build with BUILD_TESTING=ON')
    parser.add_argument('--scope', choices=PATTERNS, default='all')
    parser.add_argument('--list', action='store_true', help='List tests only')
    args = parser.parse_args()
    build = args.build_dir.resolve()
    if not (build / 'CMakeCache.txt').is_file():
        parser.error('Missing CMakeCache.txt; configure the full Linux build first')
    command = ['ctest', '--test-dir', str(build), '-R', PATTERNS[args.scope]]
    # Verify registration before execution: CTest otherwise succeeds with zero tests.
    inventory = subprocess.run(command + ['--show-only=json-v1'], check=True,
                               capture_output=True, text=True)
    tests = json.loads(inventory.stdout)['tests']
    required = {'mapping': 'local_task_one_closed_loop',
                'traversal': 'local_stage_cones_current',
                'all': 'local_task_one_closed_loop'}[args.scope]
    names = {test['name'] for test in tests}
    if required not in names or (args.scope == 'all' and 'local_stage_cones_current' not in names):
        raise RuntimeError('Required tests absent; rebuild with Python cv2/numpy/yaml and BUILD_TESTING=ON')
    print('\n'.join(sorted(names)), flush=True)
    if args.list:
        return 0
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    evidence = ROOT / 'build' / 'grid-test' / stamp
    evidence.mkdir(parents=True)
    env = dict(os.environ, AUV_SIM_EVIDENCE_DIR=str(evidence))
    print('Simulation only; evidence: ' + str(evidence), flush=True)
    # Serial execution avoids shared fixture contention and preserves each failure.
    result = subprocess.run(command + ['--output-on-failure', '-j', '1',
                                      '--output-log', str(evidence / 'ctest.log')], env=env)
    (evidence / 'result.json').write_text(json.dumps({
        'scope': args.scope, 'tests': sorted(names), 'returncode': result.returncode,
        'hardware_test': False,
    }, indent=2), encoding='utf-8')
    return result.returncode


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, OSError, subprocess.CalledProcessError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(1)
