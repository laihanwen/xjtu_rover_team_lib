"""YOLO detection dataset validation shared by training and command-line tools."""

from __future__ import annotations

import hashlib
from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml
from PIL import Image, UnidentifiedImageError

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
SPLITS = ("train", "val", "test")


@dataclass
class DatasetReport:
    root: str
    class_names: list[str]
    images_by_split: dict[str, int] = field(default_factory=dict)
    boxes_by_split: dict[str, int] = field(default_factory=dict)
    boxes_by_class: dict[str, int] = field(default_factory=dict)
    negative_images: int = 0
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def valid(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["valid"] = self.valid
        return result


def load_class_names(path: Path) -> list[str]:
    content = yaml.safe_load(path.read_text(encoding="utf-8"))
    names = content.get("names") if isinstance(content, dict) else None
    if not isinstance(names, dict) or not names:
        raise ValueError(f"{path}: expected a non-empty 'names' mapping")
    indexes = sorted(names)
    if indexes != list(range(len(indexes))):
        raise ValueError(f"{path}: class indexes must be contiguous from zero")
    return [str(names[index]) for index in indexes]


def _image_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_label(
    path: Path, class_names: list[str], report: DatasetReport
) -> int:
    if not path.exists():
        report.errors.append(f"missing label: {path}")
        return 0
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return 0
    count = 0
    for line_number, line in enumerate(text.splitlines(), start=1):
        fields = line.split()
        location = f"{path}:{line_number}"
        if len(fields) != 5:
            report.errors.append(f"{location}: expected 5 fields, got {len(fields)}")
            continue
        try:
            class_id = int(fields[0])
            coordinates = [float(value) for value in fields[1:]]
        except ValueError:
            report.errors.append(f"{location}: label fields are not numeric")
            continue
        if class_id < 0 or class_id >= len(class_names):
            report.errors.append(f"{location}: class id {class_id} is out of range")
            continue
        center_x, center_y, width, height = coordinates
        if not all(0.0 <= value <= 1.0 for value in coordinates):
            report.errors.append(f"{location}: normalized coordinates must be within [0, 1]")
            continue
        if width <= 0.0 or height <= 0.0:
            report.errors.append(f"{location}: width and height must be positive")
            continue
        if center_x - width / 2.0 < 0.0 or center_x + width / 2.0 > 1.0:
            report.errors.append(f"{location}: box exceeds horizontal image bounds")
            continue
        if center_y - height / 2.0 < 0.0 or center_y + height / 2.0 > 1.0:
            report.errors.append(f"{location}: box exceeds vertical image bounds")
            continue
        report.boxes_by_class[class_names[class_id]] += 1
        count += 1
    return count


def validate_dataset(root: Path, classes_path: Path) -> DatasetReport:
    root = root.resolve()
    class_names = load_class_names(classes_path.resolve())
    report = DatasetReport(
        root=str(root),
        class_names=class_names,
        boxes_by_class={name: 0 for name in class_names},
    )
    digest_split: dict[str, str] = {}
    for split in SPLITS:
        image_dir = root / "images" / split
        label_dir = root / "labels" / split
        if not image_dir.is_dir():
            report.errors.append(f"missing image split directory: {image_dir}")
            report.images_by_split[split] = 0
            report.boxes_by_split[split] = 0
            continue
        if not label_dir.is_dir():
            report.errors.append(f"missing label split directory: {label_dir}")
        images = sorted(
            path for path in image_dir.rglob("*") if path.suffix.lower() in IMAGE_SUFFIXES
        )
        report.images_by_split[split] = len(images)
        report.boxes_by_split[split] = 0
        if not images:
            report.warnings.append(f"{split} split contains no images")
        image_stems: set[Path] = set()
        for image_path in images:
            relative = image_path.relative_to(image_dir).with_suffix("")
            image_stems.add(relative)
            try:
                with Image.open(image_path) as image:
                    image.verify()
            except (OSError, UnidentifiedImageError) as error:
                report.errors.append(f"invalid image {image_path}: {error}")
                continue
            digest = _image_digest(image_path)
            previous_split = digest_split.get(digest)
            if previous_split is not None and previous_split != split:
                report.errors.append(
                    f"duplicate image content across {previous_split}/{split}: {image_path}"
                )
            else:
                digest_split[digest] = split
            label_path = label_dir / relative.with_suffix(".txt")
            box_count = _validate_label(label_path, class_names, report)
            report.boxes_by_split[split] += box_count
            if box_count == 0:
                report.negative_images += 1
        if label_dir.is_dir():
            for label_path in label_dir.rglob("*.txt"):
                relative = label_path.relative_to(label_dir).with_suffix("")
                if relative not in image_stems:
                    report.errors.append(f"label has no matching image: {label_path}")
    return report
