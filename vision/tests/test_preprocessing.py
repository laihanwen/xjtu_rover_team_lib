from pathlib import Path

from PIL import Image

from auv_training.dataset import validate_dataset
from preprocessing.generate_smoke_dataset import generate
from preprocessing.split_dataset import split_dataset


def test_smoke_generation_and_split(tmp_path: Path) -> None:
    raw = tmp_path / "raw"
    processed = tmp_path / "processed"
    generate(raw, count=10)
    counts = split_dataset(raw, processed, seed=42, val_fraction=0.2, test_fraction=0.2)
    assert counts == {"train": 6, "val": 2, "test": 2}
    report = validate_dataset(processed, Path(__file__).parents[1] / "configs/classes.yaml")
    assert report.valid
    assert report.images_by_split == counts
    assert report.boxes_by_split == counts


def test_generated_images_are_readable(tmp_path: Path) -> None:
    raw = tmp_path / "raw"
    generate(raw, count=3)
    with Image.open(raw / "images" / "sample_0000.png") as image:
        assert image.size == (320, 240)
