"""复核已标注可见标签；实验参数不修改 Runtime，也不输出导航位姿。"""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--review", type=Path, required=True)
    parser.add_argument("--materials", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    default = cv2.aruco.ArucoDetector(dictionary)
    parameters = cv2.aruco.DetectorParameters()
    parameters.adaptiveThreshWinSizeMax = 63
    parameters.adaptiveThreshWinSizeStep = 4
    parameters.minMarkerPerimeterRate = 0.01
    tuned = cv2.aruco.ArucoDetector(dictionary, parameters)
    report = {"scope": "人工标签可见采样帧；未核实人工ID；非正式召回率与导航验收", "cameras": {}}
    for camera, name in [("front", "前.mp4"), ("down", "下.mp4")]:
        with (args.review / f"{camera}-annotations.csv").open(encoding="utf-8-sig") as stream:
            labels = list(csv.DictReader(stream))
        cap = cv2.VideoCapture(str(args.materials / name))
        if not cap.isOpened():
            raise ValueError(f"无法打开 {name}")
        records, tiles = [], []
        try:
            for label in labels:
                index = int(label["frame"])
                cap.set(cv2.CAP_PROP_POS_FRAMES, index)
                ok, frame = cap.read()
                if not ok:
                    raise ValueError(f"无法解码 {name} 帧 {index}")
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                enhanced = cv2.createCLAHE(2.0, (8, 8)).apply(gray)
                variants = [("default", gray, default), ("clahe", enhanced, default),
                            ("wide_threshold", gray, tuned), ("clahe_wide", enhanced, tuned)]
                result = {"frame": index, "time_sec": float(label["time_sec"]),
                          "tag_visible": label["tag_visible"], "variants": {}}
                for key, image, detector in variants:
                    corners, ids, rejected = detector.detectMarkers(image)
                    result["variants"][key] = {"ids": [] if ids is None else ids.reshape(-1).tolist(),
                                               "rejected_quads": len(rejected)}
                records.append(result)
                tile = cv2.resize(frame, (480, 360))
                cv2.rectangle(tile, (0, 0), (480, 40), (0, 0, 0), -1)
                hits = [key for key, value in result["variants"].items() if 18 in value["ids"]]
                cv2.putText(tile, f'{camera} f{index} {result["time_sec"]:.0f}s ID18: {len(hits)}/4',
                            (5, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
                if label["tag_visible"] == "yes":
                    tiles.append(tile)
        finally:
            cap.release()
        for start in range(0, len(tiles), 12):
            batch = tiles[start:start + 12]
            batch += [np.zeros((360, 480, 3), np.uint8)] * (12 - len(batch))
            mosaic = np.vstack([np.hstack(batch[k:k+3]) for k in range(0, 12, 3)])
            if not cv2.imwrite(str(args.output / f"{camera}-visible-{start//12}.jpg"), mosaic):
                raise OSError("样图保存失败")
        report["cameras"][camera] = {
            "total_samples": len(records),
            "visible_samples": sum(item["tag_visible"] == "yes" for item in records),
            "id18_hits_by_variant": {key: sum(item["tag_visible"] == "yes" and 18 in item["variants"][key]["ids"] for item in records)
                                     for key in ["default", "clahe", "wide_threshold", "clahe_wide"]},
            "id18_on_labelled_no": {key: sum(item["tag_visible"] == "no" and 18 in item["variants"][key]["ids"] for item in records)
                                   for key in ["default", "clahe", "wide_threshold", "clahe_wide"]},
            "records": records}
    (args.output / "tag-analysis.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({key: {k: v for k, v in value.items() if k != "records"}
                      for key, value in report["cameras"].items()}, ensure_ascii=False))


if __name__ == "__main__":
    main()
