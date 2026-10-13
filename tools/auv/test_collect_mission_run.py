import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).with_name("collect_mission_run.py")
spec = importlib.util.spec_from_file_location("collect_mission_run", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MissionReviewTests(unittest.TestCase):
    def test_complete_bundle_with_machine_map(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = root / "run-2"
            evidence = raw / "evidence"
            evidence.mkdir(parents=True)
            (raw / "manifest.yaml").write_text("run_id: run-2\n")
            (raw / "config.yaml").write_text("mission: tag_docking\n")
            cells = [dict(row=i//3, col=i%3, object="circle_cone" if i < 2 else
                          "square_cone" if i < 4 else "unknown") for i in range(9)]
            (evidence / "map_1.jpg.json").write_text(json.dumps(dict(run_id="run-2", cells=cells)))
            names = ["TAG_DOCK_START", "TAG_IDENTIFIED", "TAG_ABOVE_CONFIRMED", "OBSERVATION_START",
                     "OBSERVATION_END", "RETURN_START", "RETURN_COMPLETE", "TASK_COMPLETE"]
            events = [dict(run_id="run-2", event=name, steady_sec=10.1,
                           timestamp_unix_ms=1760000000000, detail="") for name in names]
            events += [dict(run_id="run-2", event="TAG_DOCK_TRACE", steady_sec=10.1,
                            timestamp_unix_ms=1760000000000, detail=f"phase=HOVER x_m={x} y_m=0") for x in (0, .1)]
            (raw / "events.ndjson").write_text("\n".join(map(json.dumps, events)) + "\n")
            jpeg = b"\xff\xd8frame\xff\xd9"
            for role in ("front", "down"):
                (raw / f"{role}-1.mjpg").write_bytes(jpeg)
                (raw / f"{role}-1.frames.jsonl").write_text(json.dumps(
                    dict(sequence=1, steady_sec=10.0, offset=0, bytes=len(jpeg))) + "\n")
            bundle = root / "report"
            bundle.mkdir()
            self.assertEqual(module.make_report(bundle, raw, None, .5), 0)
            summary = json.loads((bundle / "review_summary.json").read_text())
            self.assertEqual(len(summary["cone_cells"]), 4)
            self.assertEqual(summary["collision_status"], "unconfirmed")
            self.assertEqual(summary["grasp_confirmed_count"], 0)

    def test_frame_match_and_source_integrity(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = root / "run-1"
            raw.mkdir()
            (raw / "evidence").mkdir()
            events = [dict(run_id="run-1", event="TIME_ANCHOR", steady_sec=10.1,
                           timestamp_unix_ms=1760000000000, detail="anchor")]
            (raw / "events.ndjson").write_text("\n".join(map(json.dumps, events)) + "\n")
            jpeg = b"\xff\xd8payload\xff\xd9"
            for role in ("front", "down"):
                (raw / f"{role}-1.mjpg").write_bytes(jpeg)
                (raw / f"{role}-1.frames.jsonl").write_text(json.dumps(
                    dict(sequence=3, steady_sec=10.0, offset=0, bytes=len(jpeg))) + "\n")
            bundle = root / "report"
            bundle.mkdir()
            self.assertEqual(module.make_report(bundle, raw, None, .5), 1)  # route/map absent are explicit errors
            with (bundle / "event_video_alignment.csv").open(encoding="utf-8-sig", newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["front_sequence"], "3")
            self.assertEqual(row["down_sequence"], "3")
            self.assertEqual(row["front_delta_sec"], "-0.1")
            self.assertEqual((bundle / "keyframes/00000_TIME_ANCHOR_front_3.jpg").read_bytes(), jpeg)
            self.assertEqual((bundle / "top_down_map.png").read_bytes()[:8], b"\x89PNG\r\n\x1a\n")

    def test_out_of_bounds_frame_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            raw = Path(temp)
            (raw / "front-1.mjpg").write_bytes(b"\xff\xd8\xff\xd9")
            (raw / "front-1.frames.jsonl").write_text(json.dumps(
                dict(sequence=1, steady_sec=1.0, offset=2, bytes=4)) + "\n")
            errors = []
            self.assertEqual(module.load_frames(raw, "front", errors), [])
            self.assertTrue(any("out-of-bounds" in error for error in errors))

    def test_boot_route_video_and_logs_without_map(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = root / "run-boot"
            raw.mkdir()
            (raw / "manifest.yaml").write_text("run_id: run-boot\n")
            (raw / "config.yaml").write_text("mission:\n  profile: boot_tag_route\n")
            events = [dict(run_id="run-boot", event=name, steady_sec=10.0,
                           timestamp_unix_ms=1760000000000, detail=detail)
                      for name, detail in (("RUN_START", ""), ("TAG_SEEN", "id=18 camera=down"),
                                           ("RETURN_START", "open_loop_duration_estimate"),
                                           ("RETURN_END", "open-loop reverse duration"),
                                           ("RUN_END", "task_result=complete"))]
            (raw / "events.ndjson").write_text("\n".join(map(json.dumps, events)) + "\n")
            jpeg = b"\xff\xd8frame\xff\xd9"
            for role in ("front", "down"):
                (raw / f"{role}-1.mjpg").write_bytes(jpeg)
                (raw / f"{role}-1.frames.jsonl").write_text(json.dumps(
                    dict(sequence=7, steady_sec=10.0, offset=0, bytes=len(jpeg))) + "\n")
            bundle = root / "report"
            bundle.mkdir()
            self.assertEqual(module.make_report(bundle, raw, None, .5, evidence_only=True), 0)
            summary = json.loads((bundle / "review_summary.json").read_text())
            self.assertTrue(summary["apriltag_identified"])
            self.assertTrue(summary["return_confirmed"])
            self.assertTrue(summary["return_open_loop"])
            with (bundle / "event_video_alignment.csv").open(encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(next(row for row in rows if row["event"] == "TAG_SEEN")["down_sequence"], "7")
            self.assertIn("open-loop estimate", (bundle / "review.md").read_text())


if __name__ == "__main__":
    unittest.main()
