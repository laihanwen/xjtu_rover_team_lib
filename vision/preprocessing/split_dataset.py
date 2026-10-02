"""Create deterministic train/val/test YOLO splits from a labeled flat directory."""

from __future__ import annotations

import argparse
import random
import shutil
from pathlib import Path

from auv_training.dataset import IMAGE_SUFFIXES, SPLITS


def split_dataset(source: Path, destination: Path, seed: int = 42,
                  val_fraction: float = 0.2, test_fraction: float = 0.1) -> dict[str, int]:
    """Copy images and matching labels from ``source/images`` and ``source/labels``.

    The source is intentionally not deleted. Existing destination files are rejected
    unless ``--force`` is handled by the CLI, preventing accidental dataset loss.
    """
    if not 0 <= val_fraction < 1 or not 0 <= test_fraction < 1:
        raise ValueError("split fractions must be in [0, 1)")
    if val_fraction + test_fraction >= 1:
        raise ValueError("val_fraction + test_fraction must be less than 1")
    image_root = source / "images"
    label_root = source / "labels"
    if not image_root.is_dir() or not label_root.is_dir():
        raise ValueError("source must contain images/ and labels/ directories")
    images = sorted(p for p in image_root.rglob("*") if p.suffix.lower() in IMAGE_SUFFIXES)
    if not images:
        raise ValueError(f"no images found under {image_root}")
    pairs: list[tuple[Path, Path]] = []
    for image in images:
        relative = image.relative_to(image_root)
        label = (label_root / relative).with_suffix(".txt")
        if not label.is_file():
            raise ValueError(f"missing label for {image}: {label}")
        pairs.append((image, label))
    random.Random(seed).shuffle(pairs)
    test_count = round(len(pairs) * test_fraction)
    val_count = round(len(pairs) * val_fraction)
    assignments = {
        "test": pairs[:test_count],
        "val": pairs[test_count:test_count + val_count],
        "train": pairs[test_count + val_count:],
    }
    for split in SPLITS:
        for image, label in assignments[split]:
            relative = image.relative_to(image_root)
            target_image = destination / "images" / split / relative
            target_label = (destination / "labels" / split / relative).with_suffix(".txt")
            target_image.parent.mkdir(parents=True, exist_ok=True)
            target_label.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(image, target_image)
            shutil.copy2(label, target_label)
    return {split: len(assignments[split]) for split in SPLITS}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="flat directory containing images/ and labels/")
    parser.add_argument("destination", type=Path, help="processed YOLO dataset directory")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--val-fraction", type=float, default=0.2)
    parser.add_argument("--test-fraction", type=float, default=0.1)
    parser.add_argument(
        "--force", action="store_true", help="allow writing into a non-empty destination"
    )
    args = parser.parse_args()
    if args.destination.exists() and any(args.destination.iterdir()) and not args.force:
        parser.error(f"destination is not empty: {args.destination}; use --force explicitly")
    counts = split_dataset(args.source, args.destination, args.seed,
                           args.val_fraction, args.test_fraction)
    print("split complete:", ", ".join(f"{k}={v}" for k, v in counts.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
