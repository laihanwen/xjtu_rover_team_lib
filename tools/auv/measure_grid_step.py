"""Inspect connector placement pitch in the supplied SolidWorks AP203 grid.

This is a placement inspector, not a general STEP solid/bounding-box reader.
It never writes navigation configuration or treats pipe length as grid pitch.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np


def measure(path):
    raw = path.read_bytes()
    text = raw.decode('latin1')
    entities = {int(i): v for i, v in re.findall(r'#(\d+)\s*=\s*(.*?);', text, re.S)}
    if 'SI_UNIT ( .MILLI., .METRE. )' not in text:
        raise ValueError('Expected millimetre STEP length units')
    refs = lambda value: [int(x) for x in re.findall(r'#(\d+)', value)]
    def vector(entity):
        value = re.search(r'\(\s*([^()]+)\)\s*\)', entities[entity]).group(1)
        return np.array([float(x) for x in value.split(',')])
    names = {}
    for i, value in entities.items():
        if value.startswith('SHAPE_REPRESENTATION'):
            name = re.search(r"'([^']*)'", value).group(1)
            names[i] = re.sub(r'\\X2\\(.*?)\\X0\\',
                             lambda m: bytes.fromhex(m[1]).decode('utf-16-be'), name)
    assembly = next(i for i, name in names.items() if '九宫格装配体' in name)
    nodes = []
    for value in entities.values():
        if 'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION' not in value:
            continue
        parent, child, transform = refs(value)
        if parent != assembly or not any(s in names.get(child, '') for s in ['弯头', '三通', '四通']):
            continue
        placement, _ = refs(entities[transform])
        point, _, _ = refs(entities[placement])
        nodes.append(dict(transform=transform, placement=placement,
                          part=names[child], position_mm=vector(point).tolist()))
    if len(nodes) != 16:
        raise ValueError(f'Expected 16 grid connectors, found {len(nodes)}')
    points = np.array([n['position_mm'] for n in nodes])
    # This supplied assembly's plane spans global Z and a horizontal direction.
    low = points[points[:, 2] < points[:, 2].min()+.01]
    direction = low[np.argmax(np.linalg.norm(low-low[0], axis=1))]-low[0]
    direction /= np.linalg.norm(direction)
    coordinates = np.column_stack([(points-points[0])@direction, points[:, 2]])
    levels = [np.unique(np.round(coordinates[:, axis]-coordinates[:, axis].min(), 4)).tolist()
              for axis in (0, 1)]
    if any(len(level) != 4 for level in levels):
        raise ValueError('Connector placements do not form four lines on each axis')
    pitches = [np.diff(level).tolist() for level in levels]
    if not np.allclose(pitches, 820, atol=.01):
        raise ValueError(f'Unexpected pitch: {pitches}')
    return dict(source=str(path.resolve()), sha256=hashlib.sha256(raw).hexdigest(),
                units='mm', connector_nodes=nodes, line_coordinates_mm=levels,
                adjacent_line_pitch_mm=pitches, nominal_cell_pitch_mm=820,
                three_cell_centerline_span_mm=2460,
                limits='Connector placement pitch only; not clear opening, outer silhouette or field measurement')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('step', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = measure(args.step)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('16 connector placements: cell pitch 820 mm; 3-cell centerline span 2460 mm')
