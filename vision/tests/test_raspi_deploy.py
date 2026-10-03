import http.client
import queue
import threading
from pathlib import Path

import numpy as np
import pytest
import yaml

from raspi_deploy.dual_cam_ncnn import (
    _put_latest,
    letterbox,
    validate_model_metadata,
)
from raspi_deploy.mjpeg_stream import WebStreamer


def _config() -> dict:
    return {
        "class_names": ["sea_cucumber", "starfish", "turtle"],
        "input_size": 416,
    }


def _write_metadata(directory: Path, **overrides) -> None:
    metadata = {
        "names": {0: "sea_cucumber", 1: "starfish", 2: "turtle"},
        "imgsz": [416, 416],
    }
    metadata.update(overrides)
    (directory / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")


def test_letterbox_preserves_aspect_ratio_and_uses_padding() -> None:
    frame = np.full((100, 200, 3), 50, dtype=np.uint8)
    output, transform = letterbox(frame, 416)
    assert output.shape == (416, 416, 3)
    assert transform.scale == pytest.approx(2.08)
    assert transform.pad_x == 0
    assert transform.pad_y == 104
    assert np.all(output[:104] == 114)
    assert np.all(output[104:312] == 50)


def test_metadata_accepts_matching_model(tmp_path: Path) -> None:
    _write_metadata(tmp_path)
    metadata = validate_model_metadata(tmp_path, _config())
    assert metadata["imgsz"] == [416, 416]


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"names": ["wrong"]}, "class_names mismatch"),
        ({"imgsz": [640, 640]}, "input_size mismatch"),
    ],
)
def test_metadata_rejects_runtime_mismatch(tmp_path: Path, override: dict, message: str) -> None:
    _write_metadata(tmp_path, **override)
    with pytest.raises(ValueError, match=message):
        validate_model_metadata(tmp_path, _config())


def test_put_latest_replaces_stale_value() -> None:
    values = queue.Queue(maxsize=1)
    values.put_nowait("old")
    _put_latest(values, "new")
    assert values.get_nowait() == "new"


def test_mjpeg_stream_responds_and_stops_with_connected_client() -> None:
    streamer = WebStreamer(port=0, frame_interval=0.001)
    connection = http.client.HTTPConnection("127.0.0.1", streamer.port, timeout=2)
    connection.request("GET", "/stream/0")
    response = connection.getresponse()
    assert response.status == 200
    assert response.read(64).startswith(b"--frame\r\nContent-Type: image/jpeg")

    stopper = threading.Thread(target=streamer.stop)
    stopper.start()
    stopper.join(timeout=2)
    connection.close()
    assert not stopper.is_alive()
