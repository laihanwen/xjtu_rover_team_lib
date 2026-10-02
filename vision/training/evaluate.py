"""Evaluate a trained checkpoint on the immutable test split."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from ultralytics import YOLO

from auv_training.config import load_config, write_dataset_yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=Path)
    parser.add_argument("--config", type=Path, default=Path("configs/sea_cucumber_yolo11n.yaml"))
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    if not arguments.model.is_file():
        raise SystemExit(f"model does not exist: {arguments.model}")
    config = load_config(arguments.config)
    dataset_yaml = write_dataset_yaml(config, arguments.output.parent / "dataset.generated.yaml")
    model = YOLO(str(arguments.model.resolve()))
    metrics = model.val(
        data=str(dataset_yaml),
        split="test",
        imgsz=int(config["image_size"]),
        batch=int(config["batch"]),
        device=config.get("device", 0),
        plots=True,
    )
    payload = {key: float(value) for key, value in metrics.results_dict.items()}
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
