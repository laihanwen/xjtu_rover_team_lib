#!/usr/bin/env python3
"""Convert CI build and review output into a short, actionable TODO file."""

import argparse
from datetime import datetime, timezone
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-status", required=True)
    parser.add_argument("--build-log", required=True)
    parser.add_argument("--review", required=True)
    parser.add_argument("--commit", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def review_findings(review: str) -> list[str]:
    if "No actionable findings" in review:
        return []
    try:
        findings = review.split("## Findings", 1)[1].split("## Test Gaps", 1)[0]
    except IndexError:
        return []
    return [line.strip() for line in findings.splitlines() if line.strip().startswith("-")]


def build_findings(status: str, log: str) -> list[str]:
    if status == "0":
        return []
    lines = [line.strip() for line in log.splitlines() if line.strip()]
    errors = [
        line for line in lines
        if "error:" in line.lower() or "failed" in line.lower() or "failure" in line.lower()
    ]
    details = errors[-5:] or lines[-5:]
    return [
        f"- [ ] **P1** Firmware build/test failed (status `{status}`). `{detail[:240]}`"
        for detail in details
    ]


def render(args: argparse.Namespace) -> str:
    review = Path(args.review).read_text(encoding="utf-8")
    build_log = Path(args.build_log).read_text(encoding="utf-8", errors="replace")
    findings = build_findings(args.build_status, build_log) + review_findings(review)
    commit = args.commit[:12]
    timestamp = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")

    if not findings:
        findings = ["- [x] 本次 CI 构建、测试和远程代码审查未发现待处理问题。"]

    lines = [
        "# CI TODO",
        "",
        "> 此文件由 GitHub Actions 自动生成。优先处理未勾选项目；不要手工删除历史依据。",
        "",
        f"## 最近一次检查（{timestamp}，`{commit}`）",
        "",
        *findings,
        "",
        "## 处理规则",
        "",
        "- 修复问题后，在对应提交或 PR 中补充测试，并等待下一次 CI 更新此文件。",
        "- P0/P1 涉及推进器、ARM/DISARM、漏水、急停或串口 failsafe 时，禁止直接带桨实机验证。",
        "- 远程模型的意见需要人工确认，构建失败和测试失败优先于模型判断。",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(render(args), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())