#!/usr/bin/env python3
"""Generate the pool of possible task-one input scenes and select them all.

Run once (or whenever the scene set changes):

    python3 runtime/test_scripts/generate_scenes.py

It writes every scene as a JPG under input/, emits input/ground_truth.json, and
copies the whole set into input/read/ (the default selection). To test only a
subset, delete the unwanted JPGs from input/read/.
"""
from __future__ import annotations

import json

import cv2

import scenes

# (filename stem, layout name, with_apriltag)
SCENES = [
    (f"scene_{name}", name, True) for name in sorted(scenes.LAYOUTS)
] + [
    ("scene_corners_no_tag", "corners", False),
]


def main() -> None:
    scenes.INPUT_DIR.mkdir(parents=True, exist_ok=True)
    scenes.READ_DIR.mkdir(parents=True, exist_ok=True)

    truth: dict[str, dict] = {}
    for stem, layout_name, with_tag in SCENES:
        frame = scenes.render_frame(
            scenes.LAYOUTS[layout_name],
            with_apriltag=with_tag,
            camouflage=True,
            seed=scenes.DEFAULT_SEED,
        )
        filename = f"{stem}.jpg"
        pool_path = scenes.INPUT_DIR / filename
        cv2.imwrite(str(pool_path), frame)

        layout_truth = scenes.ground_truth(scenes.LAYOUTS[layout_name])
        truth[filename] = {
            "layout": layout_name,
            "with_apriltag": with_tag,
            "cells": {f"{r},{c}": obj for (r, c), obj in layout_truth.items()},
        }

        # Default selection: everything.
        (scenes.READ_DIR / filename).write_bytes(pool_path.read_bytes())

    (scenes.INPUT_DIR / scenes.GROUND_TRUTH_FILE).write_text(
        json.dumps(truth, indent=2, sort_keys=True) + "\n"
    )
    print(
        f"generated {len(SCENES)} scenes into {scenes.INPUT_DIR} "
        f"(all selected in {scenes.READ_DIR})"
    )


if __name__ == "__main__":
    main()
