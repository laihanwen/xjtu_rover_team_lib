"""Create an auditable model manifest from real artifacts and metrics."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import yaml


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def create_manifest(model: Path, metrics: Path, classes: Path, output: Path, name: str) -> None:
    if not model.is_file():
        raise FileNotFoundError(model)
    if not metrics.is_file():
        raise FileNotFoundError(metrics)
    metric_data = json.loads(metrics.read_text(encoding="utf-8"))
    if not isinstance(metric_data, dict) or not metric_data:
        raise ValueError("metrics must be a non-empty JSON object")
    class_data = yaml.safe_load(classes.read_text(encoding="utf-8"))
    names = class_data.get("names") if isinstance(class_data, dict) else None
    if not isinstance(names, dict) or not names:
        raise ValueError("classes file must contain a non-empty names mapping")
    manifest = {
        "name": name,
        "task": "object_detection",
        "classes": [str(names[index]) for index in sorted(names)],
        "artifact": {"file": model.name, "sha256": sha256(model)},
        "metrics": metric_data,
        "source_metrics": metrics.name,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("metrics", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--classes", type=Path, default=Path("configs/classes.yaml"))
    parser.add_argument("--name", default="sea_cucumber_yolo11n-v1")
    args = parser.parse_args()
    create_manifest(args.model, args.metrics, args.classes, args.output, args.name)
    print(f"manifest written: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
