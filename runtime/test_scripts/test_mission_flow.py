"""Mission-flow runtime assertions for task one.

Covers the AprilTag gate: a scene without the tag must stay in SEARCH_APRILTAG
and never advance to BUILD_MAP. Reads the selected no_tag JPG from input/read/.
"""
from __future__ import annotations

import time

import pytest

import scenes
import visualize
from harness import RuntimeHarness


def _gate_scene():
    index = scenes.load_ground_truth()
    for f in scenes.list_read_scenes():
        if index.get(f.name, {}).get("with_apriltag") is False:
            return f.stem, f
    return None


def test_apriltag_gate_blocks_mapping(runtime_binary, tmp_path):
    gate = _gate_scene()
    if not gate:
        pytest.skip("no no_tag scene in input/read; run generate_scenes.py")
    stem, f = gate

    video = tmp_path / f"{stem}.avi"
    scenes.build_video_from_jpg(scenes.READ_DIR / f.name, video)
    with RuntimeHarness(runtime_binary, video) as h:
        h.wait_ready()
        h.start()

        h.wait_for(lambda s: s["phase"] == "SEARCH_APRILTAG")
        deadline = time.monotonic() + 3.0
        state = None
        while time.monotonic() < deadline:
            state = h.status()
            assert state["phase"] == "SEARCH_APRILTAG", (
                f"left SEARCH_APRILTAG without AprilTag: {state['phase']}"
            )
            time.sleep(0.05)
        assert not state["apriltag_found"]

        visualize.write_scene_artifacts(stem, h, state, {"SEARCH_APRILTAG": state})
