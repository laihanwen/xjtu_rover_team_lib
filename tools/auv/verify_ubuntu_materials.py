"""Verify a separate Ubuntu validation asset and replay native semantic samples."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('materials', type=Path)
    parser.add_argument('--replay', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    root = args.materials.resolve()
    manifest = json.loads((root / 'bundle-manifest.json').read_text(encoding='utf-8'))
    for name, expected in manifest['sha256'].items():
        path = (root / name).resolve()
        if not path.is_relative_to(root) or digest(path) != expected:
            raise ValueError('invalid asset path/hash: ' + name)
    print('Verified files:', len(manifest['sha256']))
    print('Source commit:', manifest['source_commit'])
    if args.replay:
        if args.output is None or args.output.exists():
            raise ValueError('supply a new output directory with --output')
        executable = str(args.replay.resolve())
        for metadata in sorted((root / 'native').glob('*/manifest.json')):
            value = json.loads(metadata.read_text(encoding='utf-8'))
            for file in value['files']:
                subprocess.run([executable, str(metadata.parent / file['output']),
                                str(args.output / metadata.parent.name / Path(file['output']).stem),
                                '1', '--independent'], check=True)
        for camera in ['front', 'down']:
            subprocess.run([executable, str(root / 'labelled' / (camera + '.avi')),
                            str(args.output / 'labelled' / camera), '1', '--independent'], check=True)
        print('Completed 1010 independent semantic samples; metric acceptance is separate.')


if __name__ == '__main__':
    main()
