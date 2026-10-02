import hashlib
import json
from pathlib import Path

import yaml
import pytest

from export.create_manifest import create_manifest


def test_manifest_contains_artifact_hash_and_metrics(tmp_path: Path) -> None:
    model = tmp_path / "model.onnx"
    model.write_bytes(b"synthetic-model")
    metrics = tmp_path / "metrics.json"
    metrics.write_text(json.dumps({"map50": 0.75}), encoding="utf-8")
    classes = tmp_path / "classes.yaml"
    classes.write_text("names:\n  0: sea_cucumber\n", encoding="utf-8")
    output = tmp_path / "manifest.yaml"
    create_manifest(model, metrics, classes, output, "test-model")
    manifest = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert manifest["name"] == "test-model"
    assert manifest["artifact"]["sha256"] == hashlib.sha256(b"synthetic-model").hexdigest()
    assert manifest["metrics"] == {"map50": 0.75}


def test_manifest_rejects_empty_metrics(tmp_path: Path) -> None:
    model = tmp_path / "model.onnx"
    model.write_bytes(b"model")
    metrics = tmp_path / "metrics.json"
    metrics.write_text("{}", encoding="utf-8")
    classes = tmp_path / "classes.yaml"
    classes.write_text("names:\n  0: sea_cucumber\n", encoding="utf-8")
    with pytest.raises(ValueError, match="non-empty"):
        create_manifest(model, metrics, classes, tmp_path / "manifest.yaml", "test")
