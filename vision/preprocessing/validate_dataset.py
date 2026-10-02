"""Validate an AUV YOLO dataset and write an auditable JSON report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from auv_training.dataset import validate_dataset


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_root", type=Path)
    parser.add_argument("--classes", type=Path, default=Path("configs/classes.yaml"))
    parser.add_argument("--report", type=Path)
    arguments = parser.parse_args()
    report = validate_dataset(arguments.dataset_root, arguments.classes)
    output = json.dumps(report.to_dict(), indent=2, ensure_ascii=False)
    print(output)
    if arguments.report:
        arguments.report.parent.mkdir(parents=True, exist_ok=True)
        arguments.report.write_text(output + "\n", encoding="utf-8")
    return 0 if report.valid else 2


if __name__ == "__main__":
    raise SystemExit(main())
