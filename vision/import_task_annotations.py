"""只读导入图片标注表；保留空白，不用场地模板填补逐帧观测。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

NS = {"s": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
FIELDS = ["frame", "time_sec", "automatic_tag_ids", "tag_visible", "expected_tag_id",
          "grid_four_corners_visible", "yellow_edge_image_side", "cone_count", "notes"]
FIELDS += [f"cell_{r}_{c}" for r in range(3) for c in range(3)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workbook", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summary = {"workbook_sha256": hashlib.sha256(args.workbook.read_bytes()).hexdigest(),
               "coordinate_convention": "黄边朝下；行列从左上开始，CSV 为零基索引", "sheets": {}}
    with zipfile.ZipFile(args.workbook) as archive:
        strings = []
        if "xl/sharedStrings.xml" in archive.namelist():
            for item in ET.fromstring(archive.read("xl/sharedStrings.xml")).findall("s:si", NS):
                strings.append("".join(item.itertext()))
        for source, camera in [("xl/worksheets/sheet1.xml", "front"),
                               ("xl/worksheets/sheet2.xml", "down")]:
            records = []
            for row in ET.fromstring(archive.read(source)).findall("s:sheetData/s:row", NS):
                if int(row.attrib["r"]) < 5:
                    continue
                values = {}
                for cell in row.findall("s:c", NS):
                    column = "".join(ch for ch in cell.attrib["r"] if ch.isalpha())
                    value = cell.find("s:v", NS)
                    text = value.text if value is not None else ""
                    if cell.attrib.get("t") == "s" and text:
                        text = strings[int(text)]
                    elif cell.attrib.get("t") == "inlineStr":
                        text = "".join(cell.find("s:is", NS).itertext())
                    values[column] = text or ""
                if not values.get("B"):
                    continue
                records.append(dict(zip(FIELDS, [values.get(chr(c), "") for c in range(66, 84)])))
            with (args.output / f"{camera}-annotations.csv").open("w", encoding="utf-8-sig", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=FIELDS)
                writer.writeheader()
                writer.writerows(records)
            summary["sheets"][camera] = {
                "rows": len(records),
                "filled_by_field": {field: sum(bool(record[field]) for record in records) for field in FIELDS[3:]},
                "tag_visible_values": sorted(set(record["tag_visible"] for record in records)),
                "tag_visibility_counts": {value: sum(record["tag_visible"] == value for record in records)
                                          for value in ["yes", "no", "uncertain", ""]},
                "grid_four_corners_counts": {value: sum(record["grid_four_corners_visible"] == value for record in records)
                                            for value in ["yes", "no", "uncertain", ""]},
                "automatic_tag_positive_rows": sum(bool(record["automatic_tag_ids"]) for record in records),
                "visible_with_automatic_tag": sum(record["tag_visible"] == "yes" and bool(record["automatic_tag_ids"]) for record in records),
            }
    (args.output / "import-summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
