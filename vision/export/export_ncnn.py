"""Export a trained YOLO checkpoint to NCNN for Raspberry Pi 4B deployment."""

from __future__ import annotations

import argparse
import hashlib
import shutil
from pathlib import Path

DEFAULT_IMAGE_SIZE = 416
NCNN_SUFFIXES = (".param", ".bin")
ARTIFACT_DIRECTORY = "models/artifacts"
MANIFEST_DIRECTORY = "models/manifests"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_image_size(image_size: int) -> int:
    if image_size <= 0:
        raise ValueError(f"imgsz must be positive: {image_size}")
    if image_size % 32 != 0:
        raise ValueError(f"imgsz must be a multiple of 32: {image_size}")
    return image_size


def resolve_export_directory(exported: Path) -> Path:
    """Ultralytics returns the ncnn directory, but tolerate a returned file path."""
    if exported.is_dir():
        return exported
    return exported.parent


def collect_ncnn_files(source: Path) -> list[Path]:
    files = sorted(
        path for path in source.iterdir() if path.is_file() and path.suffix in NCNN_SUFFIXES
    )
    if len(files) != len(NCNN_SUFFIXES):
        raise FileNotFoundError(f"expected one .param and one .bin in: {source}")
    return files


def stage_artifacts(exported: Path, output_dir: Path) -> list[Path]:
    """Copy model.ncnn.param and model.ncnn.bin into the requested output directory."""
    source = resolve_export_directory(exported)
    files = collect_ncnn_files(source)
    output_dir.mkdir(parents=True, exist_ok=True)
    staged: list[Path] = []
    for path in files:
        destination = output_dir / path.name
        shutil.copy2(path, destination)
        staged.append(destination)
    return staged


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path, help="trained YOLO weights file (.pt)")
    parser.add_argument("output_dir", type=Path, help="directory that receives the ncnn files")
    parser.add_argument(
        "--imgsz",
        type=int,
        default=DEFAULT_IMAGE_SIZE,
        help="square input size; 416 is the size measured on the Pi 4B",
    )
    parser.add_argument(
        "--half",
        action="store_true",
        help="export fp16 weights; ARM fp16 gains are limited, keep it off by default",
    )
    return parser


def export_model(checkpoint: Path, image_size: int, half: bool) -> Path:
    from ultralytics import YOLO

    model = YOLO(str(checkpoint.resolve()))
    # ARM NEON has no native fp16 compute path, so fp16 mostly halves the file
    # size and gives the Pi 4B little speed gain. Keep it disabled unless measured.
    return Path(
        model.export(
            format="ncnn",
            imgsz=image_size,
            batch=1,
            half=half,
            device="cpu",
        )
    )


def main() -> int:
    arguments = build_parser().parse_args()
    if not arguments.checkpoint.is_file():
        raise SystemExit(f"checkpoint does not exist: {arguments.checkpoint}")
    try:
        image_size = validate_image_size(arguments.imgsz)
    except ValueError as error:
        raise SystemExit(str(error)) from error

    exported = export_model(arguments.checkpoint, image_size, bool(arguments.half))
    source = resolve_export_directory(exported)
    staged = stage_artifacts(exported, arguments.output_dir)
    for path in staged:
        print(f"{path.name}={path}")
        print(f"{path.name}_bytes={path.stat().st_size}")
        print(f"{path.name}_sha256={sha256(path)}")

    metadata = source / "metadata.yaml"
    if metadata.is_file():
        print(f"metadata={metadata}")
        print(metadata.read_text(encoding="utf-8").rstrip("\n"))
    else:
        print(f"metadata missing: {metadata}")
    print("keep metadata.yaml archived with the ncnn artifacts, otherwise the input")
    print("size and the class order can no longer be proven later")
    print(
        f"weights and ncnn artifacts belong to {ARTIFACT_DIRECTORY}/ (git-ignored); "
        "do not commit them"
    )
    print(f"the model manifest belongs to {MANIFEST_DIRECTORY}/")
    print(
        "next: uv run python export/create_manifest.py <artifact> <metrics> "
        f"{MANIFEST_DIRECTORY}/<name>.yaml"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
