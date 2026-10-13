#!/usr/bin/env python3
"""Fetch one Pi run by SSH key and build a read-only referee evidence bundle.

No camera/serial/control access. The source MJPG and JSONL files remain untouched.
"""
import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import re
import shutil
import struct
import subprocess
import sys
import tarfile
import zlib
from bisect import bisect_left
from pathlib import Path

RUN_ROOT = "/var/log/auv-runtime/runs"
RUN_ID = re.compile(r"[A-Za-z0-9_-]+\Z")
KEY_EVENTS = {"BOOT", "RUN_START", "RUN_END", "TIME_ANCHOR", "TAG_DOCK_START", "ARM_REQUEST", "ARM_CONFIRMED",
              "TAG_IDENTIFIED", "TAG_ABOVE_CONFIRMED", "OBSERVATION_START",
              "MAP_COMPLETE", "OBSERVATION_END", "RETURN_START",
              "RETURN_COMPLETE", "TASK_COMPLETE", "FAULT", "SHUTDOWN",
              "DEPTH_STABLE", "AUTO_START", "ASCEND_START", "ASCEND_DONE",
              "FORWARD_START", "TAG_SEEN", "FORWARD_END", "RETURN_OPEN_LOOP",
              "RETURN_END", "HOLD_STABLE"}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def ssh(host, remote, key, timeout=30, output=None):
    command = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8"]
    if key:
        command += ["-i", str(key)]
    command += [host, remote]
    result = subprocess.run(command, stdout=output or subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=timeout, check=False)
    if result.returncode:
        raise RuntimeError(f"SSH failed ({result.returncode}): {result.stderr.decode(errors='replace').strip()}")
    return result.stdout.decode(errors="replace") if output is None else ""


def fetch(args):
    if args.run_id:
        run_id = args.run_id
    else:
        names = ssh(args.host, f"find {RUN_ROOT} -mindepth 1 -maxdepth 1 -type d -printf '%f\\n'",
                    args.identity, timeout=20).splitlines()
        candidates = sorted(name for name in names if RUN_ID.fullmatch(name))
        if not candidates:
            raise RuntimeError("No onboard run directories found")
        run_id = candidates[-1]
    if not RUN_ID.fullmatch(run_id):
        raise ValueError("run_id contains unsupported characters")
    output = args.output_root / ("run_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    output.mkdir(parents=True)
    archive = output / "source.tar"
    try:
        with archive.open("wb") as stream:
            ssh(args.host, f"tar -C {RUN_ROOT} -cf - -- {run_id}", args.identity,
                timeout=args.timeout, output=stream)
        with tarfile.open(archive) as tar:
            members = tar.getmembers()
            for member in members:
                parts = Path(member.name).parts
                if not parts or parts[0] != run_id or member.issym() or member.islnk() or not (member.isfile() or member.isdir()):
                    raise RuntimeError(f"Unsafe archive entry: {member.name}")
                if any(part in ("..", ".") for part in parts):
                    raise RuntimeError(f"Unsafe archive path: {member.name}")
            for member in members:
                target = output / "raw" / member.name
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                else:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    source = tar.extractfile(member)
                    if source is None:
                        raise RuntimeError(f"Cannot read archive entry: {member.name}")
                    with source, target.open("wb") as sink:
                        shutil.copyfileobj(source, sink)
        archive.unlink()
    except Exception:
        # Keep a failed transfer for diagnosis, but never call it a successful backup.
        raise
    return output, output / "raw" / run_id


def records(path, errors):
    result = []
    try:
        with path.open(encoding="utf-8") as stream:
            for number, line in enumerate(stream, 1):
                try:
                    result.append(json.loads(line))
                except (ValueError, UnicodeError) as exc:
                    errors.append(f"{path.name}:{number}: invalid JSON: {exc}")
    except OSError as exc:
        errors.append(f"{path.name}: {exc}")
    return result


def load_frames(raw, role, errors):
    frames = []
    indexes = sorted(raw.glob(f"{role}-*.frames.jsonl"), key=lambda p: int(p.name.split("-")[1].split(".")[0]))
    for video in raw.glob(f"{role}-*.mjpg"):
        if not (raw / video.name.replace(".mjpg", ".frames.jsonl")).exists():
            errors.append(f"Missing frame index for {video.name}")
    for index in indexes:
        video = raw / index.name.replace(".frames.jsonl", ".mjpg")
        if not video.exists():
            errors.append(f"Missing video segment for {index.name}")
            continue
        size = video.stat().st_size
        last_stamp = -math.inf
        with index.open(encoding="utf-8") as stream:
            for line_no, line in enumerate(stream, 1):
                try:
                    item = json.loads(line)
                    stamp = float(item["steady_sec"])
                    offset, length = int(item["offset"]), int(item["bytes"])
                    if not math.isfinite(stamp) or stamp <= last_stamp or offset < 0 or length < 4 or offset + length > size:
                        raise ValueError("nonmonotonic stamp or out-of-bounds JPEG")
                    with video.open("rb") as source:
                        source.seek(offset)
                        data = source.read(length)
                    if not (data.startswith(b"\xff\xd8") and data.endswith(b"\xff\xd9")):
                        raise ValueError("invalid JPEG boundaries")
                    frames.append({"stamp": stamp, "sequence": int(item["sequence"]),
                                   "video": video, "offset": offset, "bytes": length})
                    last_stamp = stamp
                except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
                    errors.append(f"{index.name}:{line_no}: {exc}")
        if not any(frame["video"] == video for frame in frames):
            errors.append(f"No valid indexed frames in {video.name}")
    frames.sort(key=lambda frame: frame["stamp"])
    if not frames:
        errors.append(f"Missing or empty {role} onboard recording/index")
    return frames


def nearest(frames, stamp, tolerance):
    if not frames or not math.isfinite(stamp):
        return None
    stamps = [frame["stamp"] for frame in frames]
    position = bisect_left(stamps, stamp)
    choices = [frames[i] for i in (position - 1, position) if 0 <= i < len(frames)]
    result = min(choices, key=lambda frame: abs(frame["stamp"] - stamp))
    return result if abs(result["stamp"] - stamp) <= tolerance else None


def png(path, pixels):
    height, width = len(pixels), len(pixels[0])
    def chunk(kind, body):
        return struct.pack("!I", len(body)) + kind + body + struct.pack("!I", zlib.crc32(kind + body) & 0xffffffff)
    scan = b"".join(b"\0" + b"".join(bytes(color) for color in row) for row in pixels)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack("!2I5B", width, height, 8, 2, 0, 0, 0)) +
                     chunk(b"IDAT", zlib.compress(scan, 6)) + chunk(b"IEND", b""))


def draw_map(path, cells, traces, tag_trace_index=None):
    width, height = 720, 420
    pixels = [[(250, 251, 252) for _ in range(width)] for _ in range(height)]
    def dot(x, y, color, radius=3):
        for yy in range(max(0, y-radius), min(height, y+radius+1)):
            for xx in range(max(0, x-radius), min(width, x+radius+1)):
                if (xx-x)**2 + (yy-y)**2 <= radius**2:
                    pixels[yy][xx] = color
    def line(x1, y1, x2, y2, color):
        steps = max(abs(x2-x1), abs(y2-y1), 1)
        for i in range(steps+1):
            dot(round(x1+(x2-x1)*i/steps), round(y1+(y2-y1)*i/steps), color, 1)
    def box(x0, y0, x1, y1, color):
        for y in range(y0, y1):
            for x in range(x0, x1):
                pixels[y][x] = color
    glyphs = {"S": ["01110","10000","10000","01110","00001","00001","11110"],
              "T": ["11111","00100","00100","00100","00100","00100","00100"],
              "E": ["11111","10000","10000","11110","10000","10000","11111"]}
    def label(x, y, letter, color):
        for dy, row in enumerate(glyphs[letter]):
            for dx, bit in enumerate(row):
                if bit == "1":
                    for oy in range(2):
                        for ox in range(2):
                            if 0 <= x+dx*2+ox < width and 0 <= y+dy*2+oy < height:
                                pixels[y+dy*2+oy][x+dx*2+ox] = color
    for i in range(4):
        line(30+i*105, 45, 30+i*105, 360, (70, 78, 87))
        line(30, 45+i*105, 345, 45+i*105, (70, 78, 87))
    for cell in cells:
        row, col = cell.get("row"), cell.get("col")
        kind = cell.get("object", cell.get("type", "unknown"))
        if not isinstance(row, int) or not isinstance(col, int) or not (0 <= row < 3 and 0 <= col < 3):
            continue
        x, y = 82 + col*105, 97 + row*105
        if "circle" in kind:
            dot(x, y, (41, 119, 189), 20)
        elif "square" in kind:
            box(x-18, y-18, x+18, y+18, (226, 112, 46))
        elif kind == "empty":
            dot(x, y, (177, 194, 171), 3)
        else:
            dot(x, y, (170, 174, 180), 4)
    line(30, 363, 345, 363, (220, 181, 43))  # canonical yellow edge down
    coords = [(float(t[0]), float(t[1])) for t in traces if all(math.isfinite(float(v)) for v in t)]
    if coords:
        xs, ys = zip(*coords)
        span = max(max(xs)-min(xs), max(ys)-min(ys), .25)
        scale = 280/span
        mapped = [(round(520+(x-(min(xs)+max(xs))/2)*scale),
                   round(205-(y-(min(ys)+max(ys))/2)*scale)) for x, y in coords]
        for first, second in zip(mapped, mapped[1:]):
            line(*first, *second, (42, 122, 105))
        dot(*mapped[0], (24, 145, 70), 8)  # run origin/start
        label(mapped[0][0]+10, mapped[0][1]-10, "S", (24, 145, 70))
        if tag_trace_index is not None and 0 <= tag_trace_index < len(mapped):
            dot(*mapped[tag_trace_index], (41, 119, 189), 8)  # measured hover point, tag vicinity
            label(mapped[tag_trace_index][0]+10, mapped[tag_trace_index][1]-10, "T", (41, 119, 189))
        dot(*mapped[-1], (188, 54, 63), 7)  # last measured position
        label(mapped[-1][0]+10, mapped[-1][1]-10, "E", (188, 54, 63))
    png(path, pixels)


def parse_detail(detail):
    return dict(re.findall(r"([A-Za-z_]+)=([^ ]+)", str(detail)))


def make_report(bundle, raw, manual_grid, tolerance, evidence_only=False):
    errors = []
    run_id = raw.name
    manifest_text = (raw / "manifest.yaml").read_text(encoding="utf-8") if (raw / "manifest.yaml").exists() else ""
    simulated = bool(re.search(r"(?m)^simulation:\s*true\s*$", manifest_text))
    for required in ("manifest.yaml", "config.yaml"):
        if not (raw / required).exists():
            errors.append(f"Missing {required} run provenance")
    events = []
    for path in sorted(raw.glob("events.ndjson*")):
        events += records(path, errors)
    if not events:
        errors.append("Missing or empty events.ndjson (including rotated segments)")
    valid_events = []
    for event in events:
        if event.get("run_id") != run_id:
            errors.append(f"Event run_id mismatch: {event.get('run_id')}")
            continue
        try:
            stamp = float(event["steady_sec"])
            if not math.isfinite(stamp):
                raise ValueError("nonfinite")
            valid_events.append(event)
        except (KeyError, TypeError, ValueError):
            errors.append(f"Event missing valid steady_sec: {event.get('event')}")
    events = valid_events
    events.sort(key=lambda item: float(item.get("steady_sec", -1)))
    frames = {role: load_frames(raw, role, errors) for role in ("front", "down")}
    stats = {}
    for role, items in frames.items():
        duration = max(0., items[-1]["stamp"]-items[0]["stamp"]) if items else 0.
        stats[role] = {"frames": len(items), "duration_sec": duration,
                       "average_fps": (len(items)-1)/duration if duration > 0 else 0.,
                       "segments": len(list(raw.glob(f"{role}-*.mjpg")))}
    keydir = bundle / "keyframes"
    keydir.mkdir(exist_ok=True)
    table = bundle / "event_video_alignment.csv"
    columns = ["event", "steady_sec", "wall_utc", "detail"]
    for role in ("front", "down"):
        columns += [f"{role}_video_sec", f"{role}_sequence", f"{role}_delta_sec", f"{role}_source"]
    with table.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, columns)
        writer.writeheader()
        for number, event in enumerate(events):
            stamp = float(event.get("steady_sec", "nan"))
            wall = event.get("timestamp_unix_ms")
            wall_text = dt.datetime.fromtimestamp(wall/1000, dt.timezone.utc).isoformat() if isinstance(wall, (int, float)) else ""
            row = {"event": event.get("event", ""), "steady_sec": stamp,
                   "wall_utc": wall_text, "detail": event.get("detail", "")}
            for role in ("front", "down"):
                item = nearest(frames[role], stamp, tolerance)
                if item:
                    row[f"{role}_video_sec"] = round(item["stamp"] - frames[role][0]["stamp"], 6)
                    row[f"{role}_sequence"] = item["sequence"]
                    row[f"{role}_delta_sec"] = round(item["stamp"] - stamp, 6)
                    row[f"{role}_source"] = f"{item['video'].name}@{item['offset']}+{item['bytes']}"
                    if event.get("event") in KEY_EVENTS:
                        with item["video"].open("rb") as video:
                            video.seek(item["offset"])
                            image = video.read(item["bytes"])
                        (keydir / f"{number:05d}_{event['event']}_{role}_{item['sequence']}.jpg").write_bytes(image)
                else:
                    row[f"{role}_source"] = "NO_FRAME_WITHIN_TOLERANCE"
                    if event.get("event") in KEY_EVENTS - {"BOOT", "RUN_START", "TIME_ANCHOR", "SHUTDOWN"}:
                        errors.append(f"No {role} frame within {tolerance:.3f}s of {event.get('event')} at {stamp}")
            writer.writerow(row)
    map_files = sorted((raw / "evidence").glob("map_*.jpg.json")) if (raw / "evidence").exists() else []
    map_source = "none"
    cells = []
    if map_files:
        result = json.loads(map_files[-1].read_text(encoding="utf-8"))
        if result.get("run_id") == run_id:
            cells, map_source = result.get("cells", []), str(map_files[-1].relative_to(raw))
        else:
            errors.append("Machine map run_id mismatch")
    if manual_grid:
        annotation = json.loads(manual_grid.read_text(encoding="utf-8"))
        if annotation.get("run_id") not in (run_id, None):
            errors.append("Manual grid run_id mismatch; not applied")
        elif not cells:
            cells = annotation.get("submission_grid", [])
            map_source = f"operator_manual:{manual_grid.name}"
            shutil.copy2(manual_grid, bundle / "manual-grid.json")
    if not cells:
        cells = [{"row": row, "col": col, "type": "unknown"} for row in range(3) for col in range(3)]
        if not evidence_only:
            errors.append("No verified machine grid map; map cells remain unknown")
    cone_cells = [cell for cell in cells if any(shape in str(cell.get("object", cell.get("type", "")))
                                            for shape in ("circle", "square"))]
    review_events = list(events)
    if map_source != "none" and map_source != "unknown":
        review_events.append({"run_id": run_id, "event": "GRID_MAP_REVIEW",
                              "source": "operator_manual" if map_source.startswith("operator_manual:") else "machine_map_artifact",
                              "source_file": map_source, "cells": cells,
                              "note": "Derived review entry; original events.ndjson is unchanged"})
    with (bundle / "review_events.ndjson").open("w", encoding="utf-8") as stream:
        for item in review_events:
            stream.write(json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n")
    traces = []
    tag_trace_index = None
    for event in events:
        if event.get("event") in ("TAG_DOCK_TRACE", "BOOT_ROUTE_TRACE"):
            values = parse_detail(event.get("detail", ""))
            try:
                traces.append((float(values["x_m"]), float(values["y_m"])))
                if values.get("phase") == "HOVER" and tag_trace_index is None:
                    tag_trace_index = len(traces)-1
            except (KeyError, ValueError):
                pass
    if not traces and (raw / "trajectory.jsonl").exists():
        for point in records(raw / "trajectory.jsonl", errors):
            try:
                traces.append((float(point["x_m"]), float(point["y_m"])))
            except (KeyError, ValueError):
                pass
    if not traces and not evidence_only:
        errors.append("No measured XY trajectory; route panel is empty")
    draw_map(bundle / "top_down_map.png", cells, traces, tag_trace_index)
    all_files = [path for path in raw.rglob("*") if path.is_file()]
    manifest = bundle / "source_sha256.csv"
    with manifest.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(["source", "bytes", "sha256"])
        for path in sorted(all_files):
            writer.writerow([str(path.relative_to(raw)), path.stat().st_size, sha256(path)])
    names = {event.get("event") for event in events}
    missing_events = sorted({"TAG_DOCK_START", "TAG_IDENTIFIED", "TAG_ABOVE_CONFIRMED", "OBSERVATION_START",
                             "OBSERVATION_END", "RETURN_START", "RETURN_COMPLETE", "TASK_COMPLETE"} - names)
    if missing_events and not evidence_only:
        errors.append("Missing full-route events: " + ", ".join(missing_events))
    if not evidence_only and not (raw / "evidence").exists():
        errors.append("Missing evidence/debug_dir")
    if not evidence_only and not map_files:
        errors.append("Missing machine map artifact; manual annotation is not sensor evidence")
    summary = {"run_id": run_id, "review_scope": "video_and_logs" if evidence_only else "full_route",
               "map_source": map_source, "cone_cells": cone_cells,
               "simulation": simulated,
               "apriltag_identified": any(event.get("event") in ("TAG_IDENTIFIED", "APRILTAG_FOUND", "TAG_SEEN") for event in events),
               "return_confirmed": "RETURN_COMPLETE" in names or "RETURN_END" in names,
               "return_open_loop": "RETURN_OPEN_LOOP" in names or any(
                   event.get("event") == "RETURN_START" and "open_loop" in event.get("detail", "") for event in events),
               "grasp_confirmed_count": sum(event.get("event") == "GRASP_CONFIRMED" for event in events),
               "collision_status": "detected" if any(event.get("event") in ("COLLISION", "IMPACT") for event in events) else "unconfirmed",
               "valve_status": "confirmed" if "VALVE_COMPLETE" in names else "unconfirmed",
               "video": stats, "errors": errors}
    (bundle / "review_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    report = bundle / "review.md"
    lines = [f"# {'SIMULATED DATA — ' if simulated else ''}Mission run {run_id}", "",
             "This is generated data, not Pi evidence or a water test." if simulated else "Onboard source files are retained below.",
             "", f"Source: {raw}", f"Map source: {map_source}",
             f"Events: {len(events)}; measured trace samples: {len(traces)}; cone cells: {len(cone_cells)}", "",
             "| Camera | Segments | Frames | Duration s | Mean received FPS |", "|---|---:|---:|---:|---:|" ]
    for role, data in stats.items():
        lines.append(f"| {role} | {data['segments']} | {data['frames']} | {data['duration_sec']:.3f} | {data['average_fps']:.2f} |")
    lines += ["", "## Evidence limits", "", "- Frame times are Pi receive/decode monotonic times, not exposure times. Cameras are not hardware synchronized.",
              f"- Event/frame matches require |delta| <= {tolerance:.3f} s; see signed deltas in CSV. Variable-rate MJPG seconds derive from frame indexes, not a nominal FPS.",
              "- Grid cells are canonical yellow-edge-down. Manual cells, if used, are operator evidence. Unknown is never treated as empty.",
              "- Grid panel uses circle/square glyphs; S=start, T=measured hover point near tag, E=last measured position on the route panel. T is not a surveyed tag coordinate.",
              "- The trace panel shows measured relative positions only; it does not infer tag/world position from an unverified yaw transform.",
              "- No gripper count, collision-free claim, valve success, or referee task completion is inferred from missing/command-only events.",
              "", "## Missing or unverified", ""]
    if summary["return_open_loop"]:
        lines.insert(lines.index("## Missing or unverified"),
                     "- Return used a time-symmetric open-loop estimate; this does not prove arrival at the start point.")
    lines += [f"- ERROR: {item}" for item in errors] or ["- None in the checked scope"]
    lines += ["", "## Source files", "", "See source_sha256.csv for original file size and hash. Originals are under raw/."]
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Bundle: {bundle}\nRun: {run_id}\nEvents: {len(events)}")
    for role, data in stats.items():
        print(f"{role}: {data['segments']} segments, {data['frames']} frames, {data['duration_sec']:.3f}s, {data['average_fps']:.2f} FPS")
    print("Source files:")
    for path in sorted(all_files):
        print(f"  {path.relative_to(raw)}  {path.stat().st_size} bytes")
    for item in errors:
        print("ERROR:", item, file=sys.stderr)
    return 1 if errors else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="pi@192.168.137.150")
    parser.add_argument("--identity", type=Path, help="optional SSH private key; otherwise use ssh-agent/default key")
    parser.add_argument("--run-id", help="onboard run directory; default latest")
    parser.add_argument("--output-root", type=Path, default=Path("logs/mission-runs"))
    parser.add_argument("--local-run", type=Path, help="analyze existing downloaded run without SSH")
    parser.add_argument("--evidence-only", action="store_true",
                        help="validate dual video and event timing without requiring route/map events")
    parser.add_argument("--manual-grid", type=Path, help="optional manual-grid JSON exported by minimal-task page")
    parser.add_argument("--max-frame-delta", type=float, default=0.5)
    parser.add_argument("--timeout", type=int, default=600)
    args = parser.parse_args()
    if not 0 < args.max_frame_delta <= 5:
        parser.error("--max-frame-delta must be within (0,5] seconds")
    try:
        if args.local_run:
            raw = args.local_run.resolve()
            bundle = args.output_root / ("run_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
            if bundle.exists():
                raise FileExistsError(f"Refusing to overwrite {bundle}")
            bundle.mkdir(parents=True)
        else:
            bundle, raw = fetch(args)
        return make_report(bundle, raw, args.manual_grid, args.max_frame_delta, args.evidence_only)
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired, tarfile.TarError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
