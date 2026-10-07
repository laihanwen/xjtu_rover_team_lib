"""Vision/mapping runtime assertions for task one.

Reads the selected input JPGs from input/read/ (see generate_scenes.py), runs
each with_apriltag scene to VISIT_CONES, and asserts the fused semantic map
matches ground truth, the cone classification is complete, the phase order is
correct, and the planned path covers every cone. Outputs are persisted under
output/<timestamp>/.
"""
from __future__ import annotations

import pytest

import scenes
import visualize
from harness import RuntimeHarness, plan_path

EXPECTED_ORDER = ["SEARCH_APRILTAG", "BUILD_MAP", "PLAN_CONES", "VISIT_CONES"]


def _full_scenes():
    index = scenes.load_ground_truth()
    out = []
    for f in scenes.list_read_scenes():
        meta = index.get(f.name, {})
        if meta.get("with_apriltag"):
            out.append((f.stem, f, meta))
    return out


def _assert_subsequence(actual, expected, name):
    seen = 0
    for phase in actual:
        if seen < len(expected) and phase == expected[seen]:
            seen += 1
    assert seen == len(expected), f"{name}: expected {expected} in {actual}"


def test_semantic_map_matches_ground_truth(runtime_binary, tmp_path):
    full = _full_scenes()
    if not full:
        pytest.skip("no with_apriltag scenes in input/read; run generate_scenes.py")

    for stem, f, meta in full:
        video = tmp_path / f"{stem}.avi"
        scenes.build_video_from_jpg(scenes.READ_DIR / f.name, video)
        with RuntimeHarness(runtime_binary, video) as h:
            h.wait_ready()
            h.start()
            state, snapshots = h.run_to_visit()

            assert state["apriltag_found"], f"{stem}: AprilTag not detected"
            assert state["grid_complete"], f"{stem}: grid not complete"
            assert state["cone_count"] == 4, f"{stem}: expected 4 cones"

            cells = {(c["row"], c["col"]): c["object"] for c in state["cells"]}
            assert len(cells) == 9, f"{stem}: expected 9 cells"
            assert cells == meta["cells"], f"{stem}: cell map mismatch"

            phases = [e["detail"] for e in h.events() if e["event"] == "MISSION"]
            _assert_subsequence(phases, EXPECTED_ORDER, stem)

            path = plan_path(h.events())
            cone_cells = {rc for rc, obj in meta["cells"].items() if obj != "unknown"}
            assert cone_cells <= set(path), f"{stem}: route missing cones {cone_cells - set(path)}"

            visualize.write_scene_artifacts(stem, h, state, snapshots)
