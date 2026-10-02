"""Export a real trained checkpoint to static ONNX and emit its SHA-256."""

from __future__ import annotations

import argparse
import hashlib
import shutil
from pathlib import Path

import onnx
from ultralytics import YOLO

from auv_training.config import load_config


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--config", type=Path, default=Path("configs/sea_cucumber_yolo11n.yaml"))
    arguments = parser.parse_args()
    if not arguments.checkpoint.is_file():
        raise SystemExit(f"checkpoint does not exist: {arguments.checkpoint}")
    config = load_config(arguments.config)
    export = config["export"]
    model = YOLO(str(arguments.checkpoint.resolve()))
    exported = Path(
        model.export(
            format="onnx",
            imgsz=int(config["image_size"]),
            opset=int(export["opset"]),
            dynamic=bool(export["dynamic"]),
            simplify=bool(export["simplify"]),
            batch=int(export["batch"]),
            device="cpu",
        )
    )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(exported, arguments.output)
    onnx.checker.check_model(onnx.load(str(arguments.output)))
    print(f"onnx={arguments.output}")
    print(f"sha256={sha256(arguments.output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
