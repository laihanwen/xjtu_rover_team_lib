from pathlib import Path

from PIL import Image

from auv_training.dataset import validate_dataset


def write_classes(path: Path) -> None:
    path.write_text("names:\n  0: sea_cucumber\n  1: turtle\n  2: starfish\n", encoding="utf-8")


def make_dataset(root: Path) -> None:
    for split in ("train", "val", "test"):
        (root / "images" / split).mkdir(parents=True)
        (root / "labels" / split).mkdir(parents=True)
        image = Image.new("RGB", (32, 24), (20, 30 + len(split), 40))
        image.save(root / "images" / split / f"{split}.png")
        (root / "labels" / split / f"{split}.txt").write_text(
            "0 0.5 0.5 0.25 0.25\n", encoding="utf-8"
        )


def test_valid_dataset(tmp_path: Path) -> None:
    classes = tmp_path / "classes.yaml"
    write_classes(classes)
    root = tmp_path / "dataset"
    make_dataset(root)
    report = validate_dataset(root, classes)
    assert report.valid
    assert report.images_by_split == {"train": 1, "val": 1, "test": 1}
    assert report.boxes_by_class["sea_cucumber"] == 3


def test_rejects_invalid_class_and_box(tmp_path: Path) -> None:
    classes = tmp_path / "classes.yaml"
    write_classes(classes)
    root = tmp_path / "dataset"
    make_dataset(root)
    (root / "labels" / "train" / "train.txt").write_text(
        "3 0.5 0.5 0.2 0.2\n0 0.95 0.5 0.2 0.2\n", encoding="utf-8"
    )
    report = validate_dataset(root, classes)
    assert not report.valid
    assert any("class id 3" in error for error in report.errors)
    assert any("horizontal image bounds" in error for error in report.errors)


def test_rejects_cross_split_duplicates(tmp_path: Path) -> None:
    classes = tmp_path / "classes.yaml"
    write_classes(classes)
    root = tmp_path / "dataset"
    make_dataset(root)
    duplicate = (root / "images" / "train" / "train.png").read_bytes()
    (root / "images" / "val" / "val.png").write_bytes(duplicate)
    report = validate_dataset(root, classes)
    assert not report.valid
    assert any("duplicate image content" in error for error in report.errors)


def test_accepts_empty_label_as_negative_image(tmp_path: Path) -> None:
    classes = tmp_path / "classes.yaml"
    write_classes(classes)
    root = tmp_path / "dataset"
    make_dataset(root)
    (root / "labels" / "test" / "test.txt").write_text("", encoding="utf-8")
    report = validate_dataset(root, classes)
    assert report.valid
    assert report.negative_images == 1
