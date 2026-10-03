import hashlib
from pathlib import Path

import pytest

from export.export_ncnn import (
    DEFAULT_IMAGE_SIZE,
    build_parser,
    collect_ncnn_files,
    resolve_export_directory,
    sha256,
    stage_artifacts,
    validate_image_size,
)


def test_sha256_matches_hashlib(tmp_path: Path) -> None:
    artifact = tmp_path / "model.ncnn.bin"
    artifact.write_bytes(b"ncnn-weights")
    assert sha256(artifact) == hashlib.sha256(b"ncnn-weights").hexdigest()


def test_parser_uses_pi_image_size_and_no_half_by_default() -> None:
    arguments = build_parser().parse_args(["best.pt", "out"])
    assert arguments.imgsz == DEFAULT_IMAGE_SIZE == 416
    assert arguments.half is False
    assert arguments.checkpoint == Path("best.pt")
    assert arguments.output_dir == Path("out")


def test_parser_accepts_imgsz_and_half_overrides() -> None:
    arguments = build_parser().parse_args(["best.pt", "out", "--imgsz", "640", "--half"])
    assert arguments.imgsz == 640
    assert arguments.half is True


def test_parser_rejects_missing_positional_arguments() -> None:
    with pytest.raises(SystemExit):
        build_parser().parse_args(["best.pt"])


def test_image_size_rejects_non_positive() -> None:
    with pytest.raises(ValueError, match="positive"):
        validate_image_size(0)


def test_image_size_rejects_non_multiple_of_32() -> None:
    with pytest.raises(ValueError, match="multiple of 32"):
        validate_image_size(400)


def test_resolve_export_directory_handles_directory_and_file(tmp_path: Path) -> None:
    directory = tmp_path / "best_ncnn_model"
    directory.mkdir()
    param = directory / "model.ncnn.param"
    param.write_text("7767517\n", encoding="utf-8")
    assert resolve_export_directory(directory) == directory
    assert resolve_export_directory(param) == directory


def test_collect_ncnn_files_returns_param_and_bin(tmp_path: Path) -> None:
    (tmp_path / "model.ncnn.param").write_text("7767517\n", encoding="utf-8")
    (tmp_path / "model.ncnn.bin").write_bytes(b"weights")
    files = collect_ncnn_files(tmp_path)
    assert [path.suffix for path in files] == [".bin", ".param"]


def test_collect_ncnn_files_rejects_missing_bin(tmp_path: Path) -> None:
    (tmp_path / "model.ncnn.param").write_text("7767517\n", encoding="utf-8")
    with pytest.raises(FileNotFoundError, match="one .param and one .bin"):
        collect_ncnn_files(tmp_path)


def test_collect_ncnn_files_rejects_empty_directory(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="one .param and one .bin"):
        collect_ncnn_files(tmp_path)


def test_stage_artifacts_copies_into_output_directory(tmp_path: Path) -> None:
    export_dir = tmp_path / "best_ncnn_model"
    export_dir.mkdir()
    (export_dir / "model.ncnn.param").write_text("7767517\n", encoding="utf-8")
    (export_dir / "model.ncnn.bin").write_bytes(b"weights")
    output_dir = tmp_path / "artifacts"
    staged = stage_artifacts(export_dir, output_dir)
    assert {path.name for path in staged} == {"model.ncnn.param", "model.ncnn.bin"}
    for path in staged:
        assert path.parent == output_dir
        assert path.is_file()
    assert (output_dir / "model.ncnn.bin").read_bytes() == b"weights"


def test_stage_artifacts_accepts_returned_param_file(tmp_path: Path) -> None:
    export_dir = tmp_path / "best_ncnn_model"
    export_dir.mkdir()
    param = export_dir / "model.ncnn.param"
    param.write_text("7767517\n", encoding="utf-8")
    (export_dir / "model.ncnn.bin").write_bytes(b"weights")
    output_dir = tmp_path / "artifacts"
    staged = stage_artifacts(param, output_dir)
    assert {path.name for path in staged} == {"model.ncnn.param", "model.ncnn.bin"}
