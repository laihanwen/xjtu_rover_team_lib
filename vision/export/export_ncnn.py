"""Export a trained YOLO checkpoint to NCNN for Raspberry Pi 4B deployment."""

from __future__ import annotations

import argparse
import hashlib
import shutil
from pathlib import Path

DEFAULT_IMAGE_SIZE = 416
NCNN_SUFFIXES = (".param", ".bin")
METADATA_FILE = "metadata.yaml"
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
    files_by_suffix = {
        suffix: sorted(
            path for path in source.iterdir() if path.is_file() and path.suffix == suffix
        )
        for suffix in NCNN_SUFFIXES
    }
    if any(len(files) != 1 for files in files_by_suffix.values()):
        raise FileNotFoundError(f"expected exactly one .param and one .bin in: {source}")
    param = files_by_suffix[".param"][0]
    binary = files_by_suffix[".bin"][0]
    if param.stem != binary.stem:
        raise FileNotFoundError(f"NCNN .param and .bin must have the same basename in: {source}")
    return sorted((param, binary))


def stage_artifacts(exported: Path, output_dir: Path) -> list[Path]:
    """Copy the NCNN pair and its required metadata into the output directory."""
    source = resolve_export_directory(exported)
    files = collect_ncnn_files(source)
    metadata = source / METADATA_FILE
    if not metadata.is_file():
        raise FileNotFoundError(f"expected {METADATA_FILE} beside NCNN artifacts: {source}")
    output_dir.mkdir(parents=True, exist_ok=True)
    staged: list[Path] = []
    for path in (*files, metadata):
        destination = output_dir / path.name
        if path.resolve() != destination.resolve():
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
    staged = stage_artifacts(exported, arguments.output_dir)
    for path in staged:
        print(f"{path.name}={path}")
        print(f"{path.name}_bytes={path.stat().st_size}")
        print(f"{path.name}_sha256={sha256(path)}")

    metadata = arguments.output_dir / METADATA_FILE
    if metadata.is_file():
        print(f"metadata={metadata}")
        print(metadata.read_text(encoding="utf-8").rstrip("\n"))
    else:
        raise SystemExit(f"metadata was not staged: {metadata}")
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
