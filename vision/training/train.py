"""Validate data, then train the configured YOLO11n detector."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from ultralytics import YOLO

from auv_training.config import load_config, resolve_path, write_dataset_yaml
from auv_training.dataset import validate_dataset


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/sea_cucumber_yolo11n.yaml"))
    arguments = parser.parse_args()
    config = load_config(arguments.config)
    dataset_root = resolve_path(config, str(config["dataset_root"]))
    classes_path = resolve_path(config, str(config["classes"]))
    report = validate_dataset(dataset_root, classes_path)
    if not report.valid:
        print(json.dumps(report.to_dict(), indent=2, ensure_ascii=False))
        raise SystemExit("dataset validation failed; training was not started")
    device = config.get("device", 0)
    if str(device) != "cpu" and not torch.cuda.is_available():
        raise SystemExit("CUDA training requested but torch.cuda.is_available() is false")
    output_dir = resolve_path(config, str(config["output_dir"]))
    dataset_yaml = write_dataset_yaml(config, output_dir / "dataset.generated.yaml")
    model = YOLO(str(config["model"]))
    result = model.train(
        data=str(dataset_yaml),
        imgsz=int(config["image_size"]),
        epochs=int(config["epochs"]),
        batch=int(config["batch"]),
        patience=int(config["patience"]),
        workers=int(config["workers"]),
        seed=int(config["seed"]),
        device=device,
        cache=bool(config["cache"]),
        project=str(output_dir),
        name=str(config["experiment_name"]),
        exist_ok=False,
    )
    print(f"training output: {result.save_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
