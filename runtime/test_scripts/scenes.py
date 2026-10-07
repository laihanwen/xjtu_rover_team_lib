"""Deterministic synthetic scenes for task-one (mission-one) runtime tests.

Every frame is generated from a fixed RNG seed, so identical inputs always
produce byte-identical videos (see the project determinism rule). A scene is a
3x3 grid with exactly one yellow edge (the bottom edge; the other three edges
are white) and a tag36h11 AprilTag placed outside the grid; four traffic cones
are placed inside the grid according to a "lot-drawn" layout and carry a
deterministic same-hue camouflage texture.

The geometry keeps the validated pipeline topology (grid border -> 3x3 cells ->
cone detector); the single yellow bottom edge and outside AprilTag mirror the
corrected competition spec, where the generated map must use the yellow edge as
its bottom edge.
"""
from __future__ import annotations

import json
import os
import pathlib

import cv2
import numpy as np

WIDTH, HEIGHT = 640, 480
FPS = 30
FRAMES = 150  # 5 s; the runtime re-opens the file on EOF, so this loops.

# Observation directories: input pool + per-run selection + timestamped output.
OBSERVE_ROOT = pathlib.Path(os.environ.get("AUV_OBSERVE_DIR", "/tmp/auv-observe"))
INPUT_DIR = OBSERVE_ROOT / "input"
READ_DIR = INPUT_DIR / "read"
OUTPUT_DIR = OBSERVE_ROOT / "output"
GROUND_TRUTH_FILE = "ground_truth.json"

DEFAULT_SEED = 20261007

# Grid geometry. The outer frame is white on three sides and yellow on the
# bottom edge (the edge the task requires the generated map to place as its
# bottom), over a dark-blue pool floor that is neither white nor yellow. The
# grid is shifted down so a top strip is free for the AprilTag, which sits
# outside the grid.
BORDER = ((80, 130), (560, 460))
BORDER_THICKNESS = 16
POOL_FLOOR_BGR = (120, 60, 20)   # dark blue pool floor (V < white_v_min)
YELLOW_EDGE_BGR = (0, 255, 255)  # the one yellow grid edge (bottom)
LINE_BGR = (255, 255, 255)       # white: other 3 edges + internal grid lines
LINE_THICKNESS = 7
VERTICAL_LINES = (240, 400)     # divides columns 0|1|2
HORIZONTAL_LINES = (240, 350)   # divides rows 0|1|2

COL_CENTERS = (160, 320, 480)
ROW_CENTERS = (185, 295, 405)

# grid_mapper rectifies the source grid rectangle (BORDER) to a square
# GRID_SIZE x GRID_SIZE top-down image, stretching x and y by different factors.
GRID_SIZE = 600
GRID_WIDTH = BORDER[1][0] - BORDER[0][0]    # 480
GRID_HEIGHT = BORDER[1][1] - BORDER[0][1]   # 330

# Cone geometry is expressed in the rectified (top-down) space, where a
# circle-cross-section cone is a circle and a square one is a square. render_frame
# draws each cone scaled back by the inverse stretch (see _source_axes) so it is
# round in the rectified image; a source-space circle would instead become an
# ellipse with aspect ~0.69 after rectification, below the detector's 0.70 floor.
CONE_RADIUS = 45.0   # circle radius in the rectified image (px)
CONE_HALF = 45.0     # square half-side in the rectified image (px)
CONE_BGR = (0, 80, 255)   # orange, hue ~9 deg -> inside the red HSV mask range
CAMO_DARK = (0, 40, 180)   # darker red-orange, still in-range
CAMO_LIGHT = (0, 130, 255)  # lighter orange, still in-range

APRILTAG_ID = 7
APRILTAG_PX = 90
APRILTAG_PLACEMENT = (slice(15, 105), slice(275, 365))  # outside the grid, above its top edge

# Representative "lot-drawn" arrangements: exactly two circles + two squares.
# These arrangements keep the centre cell empty (all four cones sit in the ring
# cells), matching a valid draw; the AprilTag is no longer inside the grid.
LAYOUTS = {
    "corners": {(0, 0): "circle", (0, 2): "square", (2, 0): "circle", (2, 2): "square"},
    "top_row": {(0, 0): "circle", (0, 1): "square", (0, 2): "circle", (1, 0): "square"},
    "right_column": {(0, 2): "circle", (1, 2): "square", (2, 2): "circle", (2, 1): "square"},
    "bottom_row": {(2, 0): "square", (2, 1): "circle", (2, 2): "square", (1, 2): "circle"},
    "cross": {(0, 1): "circle", (1, 0): "square", (1, 2): "square", (2, 1): "circle"},
}


def ground_truth(layout: dict) -> dict:
    """Return the full 3x3 semantic-map expectation for a cone layout."""
    truth = {}
    for row in range(3):
        for col in range(3):
            shape = layout.get((row, col))
            if shape == "circle":
                truth[(row, col)] = "circle_cone"
            elif shape == "square":
                truth[(row, col)] = "square_cone"
            else:
                truth[(row, col)] = "unknown"
    return truth


def _april_tag() -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.generateImageMarker(dictionary, APRILTAG_ID, APRILTAG_PX)


def _source_axes(rectified_r: float) -> tuple[float, float]:
    """Map a rectified-space radius to source-space ellipse semi-axes.

    The rectification stretches the 480x330 source grid into a 600x600 square, so
    a circle of radius `rectified_r` in the top-down view must be drawn here as an
    ellipse whose semi-axes are the inverse of that stretch.
    """
    return (rectified_r * GRID_WIDTH / GRID_SIZE, rectified_r * GRID_HEIGHT / GRID_SIZE)


def _camouflage(image: np.ndarray, center, shape: str, seed: int) -> None:
    """Overlay a deterministic static same-hue texture inside a cone silhouette.

    The seed is fixed per cone, so the pattern is identical on every frame and
    across runs. Texture points are placed inside the source-space (foreshortened)
    silhouette so they stay within the cone after rectification.
    """
    rng = np.random.default_rng(seed)
    if shape == "circle":
        rx, ry = _source_axes(CONE_RADIUS)
    else:
        rx, ry = _source_axes(CONE_HALF)
    for _ in range(12):
        if shape == "circle":
            angle = rng.uniform(0.0, 2.0 * np.pi)
            distance = rng.uniform(0.0, 0.65)
            x = int(center[0] + distance * rx * np.cos(angle))
            y = int(center[1] + distance * ry * np.sin(angle))
        else:
            x = int(center[0] + rng.uniform(-0.6, 0.6) * rx)
            y = int(center[1] + rng.uniform(-0.6, 0.6) * ry)
        radius = int(rng.integers(2, 6))
        colour = CAMO_DARK if rng.random() < 0.5 else CAMO_LIGHT
        cv2.circle(image, (x, y), radius, colour, -1)


def render_frame(
    layout: dict,
    *,
    with_apriltag: bool,
    camouflage: bool,
    seed: int,
) -> np.ndarray:
    """Render one deterministic 640x480 BGR frame for a cone layout."""
    image = np.full((HEIGHT, WIDTH, 3), POOL_FLOOR_BGR, np.uint8)
    (x0, y0), (x1, y1) = BORDER
    # Outer frame: three white edges plus one yellow bottom edge.
    cv2.rectangle(image, BORDER[0], BORDER[1], LINE_BGR, BORDER_THICKNESS)
    cv2.line(image, (x0, y1), (x1, y1), YELLOW_EDGE_BGR, BORDER_THICKNESS)
    for x in VERTICAL_LINES:
        cv2.line(image, (x, y0), (x, y1), LINE_BGR, LINE_THICKNESS)
    for y in HORIZONTAL_LINES:
        cv2.line(image, (x0, y), (x1, y), LINE_BGR, LINE_THICKNESS)
    if with_apriltag:
        rows, cols = APRILTAG_PLACEMENT
        image[rows, cols] = cv2.cvtColor(_april_tag(), cv2.COLOR_GRAY2BGR)
    for (row, col), shape in layout.items():
        center = (COL_CENTERS[col], ROW_CENTERS[row])
        if shape == "circle":
            rx, ry = _source_axes(CONE_RADIUS)
            cv2.ellipse(
                image, center, (int(round(rx)), int(round(ry))), 0.0, 0.0, 360.0,
                CONE_BGR, -1)
        else:
            rx, ry = _source_axes(CONE_HALF)
            x, y = center
            cv2.rectangle(
                image,
                (int(round(x - rx)), int(round(y - ry))),
                (int(round(x + rx)), int(round(y + ry))),
                CONE_BGR,
                -1,
            )
        if camouflage:
            _camouflage(image, center, shape, seed=seed + row * 3 + col)
    return image


def write_scene(
    path: pathlib.Path,
    layout: dict,
    *,
    with_apriltag: bool = True,
    camouflage: bool = True,
    seed: int = DEFAULT_SEED,
) -> dict:
    """Write a deterministic MJPG AVI and return the ground-truth cell map."""
    writer = cv2.VideoWriter(
        str(path), cv2.VideoWriter_fourcc(*"FFV1"), FPS, (WIDTH, HEIGHT)
    )
    if not writer.isOpened():
        raise RuntimeError("FFV1 VideoWriter unavailable")
    try:
        for _ in range(FRAMES):
            writer.write(
                render_frame(layout, with_apriltag=with_apriltag, camouflage=camouflage, seed=seed)
            )
    finally:
        writer.release()
    return ground_truth(layout)


def list_read_scenes(read_dir=None) -> list[pathlib.Path]:
    """Return the selected input JPGs, sorted by filename."""
    read_dir = pathlib.Path(read_dir) if read_dir else READ_DIR
    return sorted(read_dir.glob("*.jpg"))


def load_ground_truth(path=None) -> dict:
    """Load {filename: {layout, with_apriltag, cells: {(row, col): object}}}."""
    path = pathlib.Path(path) if path else INPUT_DIR / GROUND_TRUTH_FILE
    if not path.exists():
        return {}
    raw = json.loads(path.read_text())
    index = {}
    for filename, meta in raw.items():
        cells = {
            tuple(int(part) for part in key.split(",")): value
            for key, value in meta["cells"].items()
        }
        index[filename] = {
            "layout": meta["layout"],
            "with_apriltag": meta["with_apriltag"],
            "cells": cells,
        }
    return index


def build_video_from_jpg(jpg_path, video_path, frames: int = FRAMES) -> None:
    """Assemble a deterministic MJPG video from a single scene JPG."""
    image = cv2.imread(str(jpg_path))
    if image is None:
        raise RuntimeError(f"cannot read input image: {jpg_path}")
    # FFV1 (lossless) rather than MJPG: MJPG writes warm-up frames that are a few
    # pixels off, which leaks sub-pixel noise into the runtime's corner overlay and
    # breaks byte-determinism of the debug image. FFV1 keeps all 150 frames identical.
    writer = cv2.VideoWriter(
        str(video_path), cv2.VideoWriter_fourcc(*"FFV1"), FPS, (WIDTH, HEIGHT)
    )
    if not writer.isOpened():
        raise RuntimeError("FFV1 VideoWriter unavailable")
    try:
        for _ in range(frames):
            writer.write(image)
    finally:
        writer.release()
