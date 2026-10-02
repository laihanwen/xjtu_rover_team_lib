"""Run auditable PyTorch/ONNX evaluation on the same test split."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import onnx
import onnxruntime as ort
from ultralytics import YOLO

from auv_training.config import load_config, write_dataset_yaml


def evaluate(model_path: Path, dataset_yaml: Path, image_size: int) -> dict[str, float]:
    metrics = YOLO(str(model_path.resolve())).val(
        data=str(dataset_yaml), split="test", imgsz=image_size, device="cpu", plots=False
    )
    return {key: float(value) for key, value in metrics.results_dict.items()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("onnx_model", type=Path)
    parser.add_argument("--config", type=Path, default=Path("configs/sea_cucumber_yolo11n.yaml"))
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args()
    for model in (arguments.checkpoint, arguments.onnx_model):
        if not model.is_file():
            raise SystemExit(f"model does not exist: {model}")
    onnx.checker.check_model(onnx.load(str(arguments.onnx_model)))
    providers = ort.get_available_providers()
    config = load_config(arguments.config)
    dataset_yaml = write_dataset_yaml(config, arguments.report.parent / "dataset.generated.yaml")
    native = evaluate(arguments.checkpoint, dataset_yaml, int(config["image_size"]))
    exported = evaluate(arguments.onnx_model, dataset_yaml, int(config["image_size"]))
    differences = {key: abs(native[key] - exported.get(key, 0.0)) for key in native}
    payload = {
        "onnxruntime_providers": providers,
        "pytorch": native,
        "onnx": exported,
        "absolute_metric_differences": differences,
    }
    arguments.report.parent.mkdir(parents=True, exist_ok=True)
    arguments.report.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
