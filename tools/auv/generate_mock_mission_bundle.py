#!/usr/bin/env python3
"""Generate a wholly synthetic, self-contained example of a mission review bundle.

This does not connect to a Pi. Pillow is used only by this demonstration generator;
the production fetch/review tool needs Python's standard library alone.
"""
import argparse
import datetime as dt
import io
import json
import math
import shutil
import zipfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from collect_mission_run import make_report, sha256

BASE_STEADY = 42000.0
FPS = 5
FRAMES = 100


def font(size):
    try:
        return ImageFont.truetype("arial.ttf", size)
    except OSError:
        return ImageFont.load_default(size=size)


def scene(role, relative_sec, sequence):
    image = Image.new("RGB", (640, 480), "#123e53" if role == "front" else "#205061")
    draw = ImageDraw.Draw(image)
    if role == "front":
        draw.rectangle((0, 0, 640, 270), fill="#1a5268")
        draw.rectangle((0, 270, 640, 480), fill="#577273")
        for y in (310, 370, 430):
            draw.line((0, y, 640, y), fill="#859891", width=2)
        size = 14 + int(min(relative_sec / 5, 1) * 62)
        x, y = 320, 285
        draw.rectangle((x-size, y-size, x+size, y+size), fill="white", outline="black", width=7)
        draw.rectangle((x-size//2, y-size//2, x+size//2, y+size//2), fill="black")
        draw.text((x+size+12, y-12), "TAG 18", fill="white", font=font(18))
        for x0, shape in ((135, "CIRCLE"), (505, "SQUARE")):
            draw.polygon(((x0, 295), (x0-26, 365), (x0+26, 365)), fill="#ee8e40")
            if shape == "CIRCLE":
                draw.ellipse((x0-26, 350, x0+26, 375), outline="white", width=3)
            else:
                draw.rectangle((x0-26, 350, x0+26, 375), outline="white", width=3)
    else:
        draw.rectangle((45, 48, 595, 470), fill="#697e78")
        if relative_sec >= 3.5:
            for x in (110, 250, 390, 530):
                draw.line((x, 56, x, 460), fill="white", width=6)
            for y in (60, 195, 330, 465):
                draw.line((110, y, 530, y), fill="#f9cd3f" if y == 465 else "white", width=7)
            for x, y, shape in ((180, 125, "circle"), (460, 125, "square"),
                                (320, 260, "circle"), (320, 395, "square")):
                if shape == "circle":
                    draw.ellipse((x-23, y-23, x+23, y+23), fill="#f28b3e", outline="black", width=3)
                else:
                    draw.rectangle((x-23, y-23, x+23, y+23), fill="#f28b3e", outline="black", width=3)
            draw.rectangle((156, 237, 204, 285), fill="white", outline="black", width=5)
            draw.rectangle((169, 250, 191, 272), fill="black")
            draw.text((210, 252), "ID18", fill="white", font=font(17))
    draw.rectangle((0, 0, 640, 46), fill="#a01220")
    draw.text((10, 8), "SIMULATED DATA - NOT PI VIDEO", fill="white", font=font(24))
    draw.rectangle((0, 444, 640, 480), fill="#172a35")
    draw.text((10, 449), f"{role.upper()}  seq={sequence:03d}  Pi steady={BASE_STEADY+relative_sec:.3f}s", fill="white", font=font(17))
    output = io.BytesIO()
    image.save(output, "JPEG", quality=83)
    return output.getvalue()


def state_at(t):
    if t < 5:
        x = .42 * t / 5
        phase = "FRONT_APPROACH" if t < 4 else "DOWN_CENTER"
    elif t < 13:
        x = .42
        phase = "HOVER"
    else:
        x = max(0, .42 * (18.7-t) / 5.7)
        phase = "RETURN_TO_FINISH" if t < 18.7 else "APPROACH_COMPLETE"
    return round(x, 4), round(.015 * math.sin(t*.65), 4), phase


def generate(output_root):
    now = dt.datetime.now().astimezone().replace(microsecond=0)
    bundle = output_root / ("run_" + now.strftime("%Y%m%d_%H%M%S"))
    if bundle.exists():
        raise FileExistsError(f"Refusing to overwrite {bundle}")
    anchor_ms = int(now.timestamp() * 1000)
    run_id = f"{anchor_ms}-{int(BASE_STEADY*1_000_000_000)}"
    raw = bundle / "raw" / run_id
    evidence = raw / "evidence"
    evidence.mkdir(parents=True)
    onboard = f"/var/log/auv-runtime/runs/{run_id}"
    (raw / "manifest.yaml").write_text(
        f"run_id: {run_id}\nprofile: tag_docking\nmode: autonomous\nsimulation: true\n"
        "time_basis: simulated Pi receive steady seconds; not exposure timestamps\n"
        "video_format: synthetic JPEG frames in MJPG segments with JSONL offsets/timestamps\n"
        "release:\n  git_commit: SIMULATION_ONLY\n  firmware_hex_sha256: SIMULATION_ONLY\n  calibration_id: SIMULATION_ONLY\n", encoding="utf-8")
    (raw / "config.yaml").write_text(
        "# SIMULATION ONLY; not a deployable motion configuration\n"
        "simulation: true\nmission:\n  profile: tag_docking\n"
        "tag_docking:\n  id: 18\n  approach_only: true\n  observation_sec: 8\n  return_to_origin: true\n"
        "motion:\n  motion_commands_enabled: false\noperation:\n  auto_arm: false\n",
        encoding="utf-8")
    for role in ("front", "down"):
        for segment in (1, 2):
            video = raw / f"{role}-{segment}.mjpg"
            index = raw / f"{role}-{segment}.frames.jsonl"
            with video.open("wb") as body, index.open("w", encoding="utf-8") as rows:
                for i in range((segment-1)*50, segment*50):
                    relative = .06 + i / FPS + (.018 if role == "down" else 0) + (i % 3 - 1)*.006
                    stamp = BASE_STEADY + relative
                    sequence = i+1
                    image = scene(role, relative, sequence)
                    offset = body.tell()
                    body.write(image)
                    x, y, phase = state_at(relative)
                    row = {"sequence": sequence, "steady_sec": round(stamp, 6),
                           "source": f"SIMULATED_{role}", "image_space": "raw",
                           "offset": offset, "bytes": len(image), "dropped_since_previous": 0,
                           "telemetry": {"simulation": True, "armed": 1 <= relative < 18.8,
                                         "depth_m": .7, "x_m": x, "y_m": y, "phase": phase}}
                    rows.write(json.dumps(row, separators=(",", ":")) + "\n")
    cells = [{"row": row, "col": col, "object": "unknown", "source": "simulation"}
             for row in range(3) for col in range(3)]
    placements = [(0, 0, "circle_cone"), (0, 2, "square_cone"),
                  (1, 1, "circle_cone"), (2, 1, "square_cone")]
    cones = []
    for row, col, kind in placements:
        cells[row*3+col]["object"] = kind
        cells[row*3+col]["confidence"] = .93
        cones.append({"row": row, "col": col, "shape": kind.replace("_cone", ""),
                      "relative_xy_m": [round((col-1)*.82, 2), round((1-row)*.82, 2)],
                      "confidence": .93, "contour_frame": "rectified_down_pixels",
                      "contour": [[150+col*105, 80+row*105], [178+col*105, 80+row*105],
                                  [178+col*105, 108+row*105], [150+col*105, 108+row*105]],
                      "source": "simulation"})
    map_name = "map_42007000.jpg"
    (evidence / f"{map_name}.json").write_text(json.dumps({
        "run_id": run_id, "simulation": True, "source": "simulation_proposed_machine_map",
        "steady_sec": BASE_STEADY+7, "camera": "down", "frame_sequence": 36,
        "rows": 3, "cols": 3, "bottom_edge": "yellow", "orientation_valid": True,
        "confidence": .93, "corners": [[110, 60], [530, 60], [530, 465], [110, 465]],
        "geometry_source": "synthetic_complete_outer_corners", "local_nodes": [],
        "cells": cells, "cones": cones}, indent=2), encoding="utf-8")
    for name, role, relative in ((map_name, "down", 7),
                                 (map_name+".rectified.jpg", "down", 7),
                                 (map_name+".cones.jpg", "down", 7),
                                 ("tag18-down-centered.jpg", "down", 5),
                                 ("apriltag_18_42001800.jpg", "front", 1.8)):
        (evidence / name).write_bytes(scene(role, relative, round(relative*FPS)+1))
    (evidence / "tag18-down-centered.yaml").write_text(
        "simulation: true\nfamily: tag16h5\nid: 18\nsize_m: 0.138\n"
        "image_space: rectified\nframe_sequence: 26\nsteady_sec: 42005.0\n"
        "center_error_px: 3.2\ncalibration_id: SIMULATION_ONLY\n", encoding="utf-8")
    (raw / "tag-grid-binding.json").write_text(json.dumps({
        "simulation": True, "run_id": run_id, "frame": "tag18",
        "grid_frame": "yellow_bottom_pipe_centres", "stamp": BASE_STEADY+7,
        "quality": "synthetic_only"}, indent=2), encoding="utf-8")
    (raw / "planned_route.json").write_text(json.dumps({
        "simulation": True, "run_id": run_id, "kind": "illustrative_return_route",
        "coordinate_frame": "relative_run_origin_X_forward_Y_left_m",
        "path": [[0, 0], [.14, 0], [.28, .01], [.42, .01], [.28, .01], [.14, 0], [0, 0]]}, indent=2), encoding="utf-8")
    with (raw / "trajectory.jsonl").open("w", encoding="utf-8") as stream:
        for i in range(39):
            relative = i*.5
            x, y, phase = state_at(relative)
            stream.write(json.dumps({"run_id": run_id, "simulation": True,
                "steady_sec": BASE_STEADY+relative, "x_m": x, "y_m": y,
                "phase": phase, "source": "synthetic_localization"}) + "\n")
    def event(relative, name, detail):
        return {"run_id": run_id, "timestamp_unix_ms": anchor_ms + round(relative*1000),
                "steady_sec": round(BASE_STEADY+relative, 6), "event": name,
                "detail": "SIMULATED " + detail, "source": "simulation"}
    events = [
        event(0, "BOOT", "mode=autonomous DISARM"),
        event(.01, "TIME_ANCHOR", "wall/steady clock pair; synthetic Pi receive time"),
        event(.4, "TAG_DOCK_START", "one-shot simulated start"),
        event(.6, "ARM_REQUEST", "simulated safety gate accepted"),
        event(.9, "ARM_CONFIRMED", "simulated STM32 ACK and armed STATUS"),
        event(1.8, "TAG_IDENTIFIED", "id=18 camera=front frame=10 frame_steady_sec=42001.8 tag_body_x_m=0.35 tag_body_y_m=0.02 tag_body_z_m=-0.7 reprojection_px=0.8"),
        event(1.81, "TAG_DOCK_PHASE", "FRONT_APPROACH"),
        event(4.0, "TAG_IDENTIFIED", "id=18 camera=down frame=21 frame_steady_sec=42004.0 tag_body_x_m=0.05 tag_body_y_m=0.01 tag_body_z_m=-0.4 reprojection_px=0.6"),
        event(4.01, "TAG_DOCK_PHASE", "DOWN_CENTER"),
        event(5.0, "TAG_ABOVE_CONFIRMED", "id=18 down_frame=26 center_error_px=3.2"),
        event(5.01, "OBSERVATION_START", "onboard front/down raw recording active; timed hold=8s"),
        event(5.02, "TAG_DOCK_PHASE", "HOVER"),
        event(5.8, "YELLOW_EDGE_CANDIDATE", "camera=down cue_only=true"),
        event(7.0, "MAP_IMAGE", f"{onboard}/evidence/{map_name}"),
        event(7.02, "MAP_COMPLETE", f"{onboard}/evidence/{map_name}.json"),
        event(13.01, "OBSERVATION_END", "timed observation complete"),
        event(13.02, "RETURN_START", "measured route to run origin"),
        event(13.03, "TAG_DOCK_PHASE", "RETURN_TO_FINISH"),
        event(18.70, "RETURN_COMPLETE", "measured finish route complete"),
        event(18.71, "TASK_COMPLETE", "minimal tag route complete"),
        event(18.80, "DISARM", "simulated stop"),
        event(19.20, "SHUTDOWN", "DISARM requested"),
    ]
    for i in range(38):
        relative = .5 + i*.5
        x, y, phase = state_at(relative)
        events.append(event(relative, "TAG_DOCK_TRACE",
            f"phase={phase} x_m={x} y_m={y} session=1 depth_m=0.7 depth_target_m=0.7 sensor_sequence={i+1} goal_x_m=0.42 goal_y_m=0.01 down_error_px=3.2 front_frame={round(relative*FPS)+1} down_frame={round(relative*FPS)+1}"))
    events.sort(key=lambda row: row["steady_sec"])
    with (raw / "events.ndjson.1").open("w", encoding="utf-8") as first, \
         (raw / "events.ndjson").open("w", encoding="utf-8") as second:
        for item in events:
            (first if item["steady_sec"] < BASE_STEADY+10 else second).write(
                json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n")
    result = make_report(bundle, raw, None, .5)
    if result:
        raise RuntimeError("Synthetic bundle failed its own review; inspect review.md")
    map_image = Image.open(bundle / "top_down_map.png").convert("RGB")
    draw = ImageDraw.Draw(map_image)
    draw.rectangle((0, 0, map_image.width, 34), fill="#a01220")
    draw.text((9, 4), "SIMULATED DATA - NOT A WATER TEST", fill="white", font=font(21))
    draw.rectangle((0, 392, map_image.width, 420), fill="#a01220")
    draw.text((9, 397), "O=circle cone  []=square cone  S=start  T=tag vicinity  E=end", fill="white", font=font(14))
    map_image.save(bundle / "top_down_map.png")
    (bundle / "SIMULATION_NOTICE.md").write_text(
        "# SIMULATED DATA — NOT A PI RUN\n\n"
        "Every source frame, timestamp, event, pose, trajectory, cone and map cell in this package is synthetic. "
        "No Pi connection, deployment, ARM, motor action, or water test occurred.\n\n"
        "The event/frame offsets and hashes are internally consistent examples. "
        "The machine map artifact illustrates the proposed review format; current tag_docking does not generate this map. "
        "Grasp count 0 means zero confirmed grasp events in this mock route, not a real-world negative observation.\n",
        encoding="utf-8")
    archive = bundle.with_suffix(".zip")
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as output:
        for path in sorted(bundle.rglob("*")):
            if path.is_file():
                output.write(path, path.relative_to(bundle))
    print(f"SIMULATED bundle: {bundle}")
    print(f"ZIP: {archive} ({archive.stat().st_size} bytes, SHA256 {sha256(archive)})")
    return bundle, archive


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=Path("logs/mission-runs"))
    args = parser.parse_args()
    generate(args.output_root)


if __name__ == "__main__":
    main()
