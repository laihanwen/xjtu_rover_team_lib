"""Verify the filled-review handoff and optionally replay independent samples."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import subprocess


def digest(path):
    value=hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda:source.read(1024*1024),b''):value.update(block)
    return value.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('materials',type=Path);p.add_argument('--replay',type=Path);p.add_argument('--output',type=Path)
    a=p.parse_args();root=a.materials.resolve()
    manifest=json.loads((root/'bundle-manifest.json').read_text(encoding='utf-8'))
    if manifest.get('schema')!='auv.perception.review.bundle.v1':raise ValueError('Wrong bundle schema')
    for name,expected in manifest['sha256'].items():
        file=(root/name).resolve()
        if not file.is_relative_to(root) or digest(file)!=expected:raise ValueError('Invalid path/hash: '+name)
    print('Verified files:',len(manifest['sha256']));print('Handoff source:',manifest['source_commit'])
    print('Existing analysis source:',manifest['analysis_source_commit'])
    if a.replay:
        if not a.output or a.output.exists():raise ValueError('Supply a new output directory')
        for group in manifest['semantic_groups']:
            name=group['name'];video=(root/group['video']).resolve()
            if not re.fullmatch(r'[a-z0-9_-]+',name) or not video.is_relative_to(root):raise ValueError('Invalid group')
            destination=a.output/name
            subprocess.run([str(a.replay.resolve()),str(video),str(destination),'1','--independent'],check=True)
            with (destination/'frames.csv').open() as source:rows=list(csv.DictReader(source))
            if len(rows)!=group['frames']:raise ValueError('Replay frame count mismatch: '+name)
        print('Replayed 39 independent review samples; pixel/metric truth remains incomplete.')


if __name__=='__main__':main()
