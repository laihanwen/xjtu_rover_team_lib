"""Generate a tiny synthetic YOLO dataset for pipeline smoke tests only."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw


def generate(output: Path, count: int = 12) -> None:
    if count < 3:
        raise ValueError("count must be at least 3")
    image_dir = output / "images"
    label_dir = output / "labels"
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    for index in range(count):
        image = Image.new("RGB", (320, 240), (12 + index, 24, 36))
        draw = ImageDraw.Draw(image)
        left, top, right, bottom = 80 + index % 3 * 8, 55, 240, 185
        draw.ellipse((left, top, right, bottom), fill=(220, 150, 40))
        image.save(image_dir / f"sample_{index:04d}.png")
        cx = (left + right) / 2 / 320
        cy = (top + bottom) / 2 / 240
        width = (right - left) / 320
        height = (bottom - top) / 240
        (label_dir / f"sample_{index:04d}.txt").write_text(
            f"0 {cx:.6f} {cy:.6f} {width:.6f} {height:.6f}\n", encoding="utf-8"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--count", type=int, default=12)
    args = parser.parse_args()
    generate(args.output, args.count)
    print(f"generated {args.count} smoke samples at {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
