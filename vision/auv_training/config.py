"""Configuration helpers for repeatable AUV training runs."""

from __future__ import annotations

from pathlib import Path

import yaml

from auv_training.dataset import load_class_names


def load_config(path: Path) -> dict[str, object]:
    resolved = path.resolve()
    content = yaml.safe_load(resolved.read_text(encoding="utf-8"))
    if not isinstance(content, dict):
        raise TypeError(f"{path}: expected a YAML mapping")
    content["_config_dir"] = resolved.parent
    return content


def resolve_path(config: dict[str, object], value: str) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    vision_root = Path(__file__).resolve().parents[1]
    return (vision_root / path).resolve()


def write_dataset_yaml(config: dict[str, object], destination: Path) -> Path:
    dataset_root = resolve_path(config, str(config["dataset_root"]))
    classes_path = resolve_path(config, str(config["classes"]))
    names = load_class_names(classes_path)
    payload = {
        "path": str(dataset_root),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "names": {index: name for index, name in enumerate(names)},
    }
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return destination
