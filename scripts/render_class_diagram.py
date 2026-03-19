#!/usr/bin/env python3
"""
Render or verify the repo UML class diagram.

Usage:
  python3 scripts/render_class_diagram.py
    Render docs/CLASS_DIAGRAM.md to docs/CLASS_DIAGRAM.svg.

  python3 scripts/render_class_diagram.py --check
    Re-render the diagram to a temporary file and verify that the source and
    tracked SVG expose the same class titles.

Skill:
  Use $mpm-siggraph-2017-class-diagram to regenerate docs/CLASS_DIAGRAM.md and
  docs/CLASS_DIAGRAM.svg.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
import re
import xml.etree.ElementTree as ET
from pathlib import Path


DEFAULT_INPUT = Path("docs/CLASS_DIAGRAM.md")
DEFAULT_OUTPUT = Path("docs/CLASS_DIAGRAM.svg")


def repo_root_from_cwd() -> Path:
    try:
        output = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"],
            text=True,
            stderr=subprocess.STDOUT,
        )
    except subprocess.CalledProcessError as exc:
        raise SystemExit("Run this script from inside the repository.") from exc
    return Path(output.strip())


def resolve_repo_path(repo_root: Path, raw_path: str | Path) -> Path:
    path = Path(raw_path)
    resolved = path if path.is_absolute() else (repo_root / path).resolve()
    try:
        resolved.relative_to(repo_root)
    except ValueError as exc:
        raise SystemExit(
            f"Path must stay inside the repository: {resolved}"
        ) from exc
    return resolved


def rendered_svg_candidates(output_path: Path) -> list[Path]:
    pattern = f"{output_path.stem}-*{output_path.suffix}"
    return sorted(output_path.parent.glob(pattern))


def extract_mermaid_class_names(markdown_text: str) -> list[str]:
    pattern = re.compile(r"^\s*class\s+([A-Za-z_][A-Za-z0-9_]*)\s*\{", re.MULTILINE)
    return pattern.findall(markdown_text)


def extract_svg_class_titles(svg_text: str) -> list[str]:
    root = ET.fromstring(svg_text)
    titles: list[str] = []
    for element in root.iter():
        if not element.tag.endswith("g"):
            continue
        if element.attrib.get("class") != "label":
            continue
        if "font-weight: bolder" not in element.attrib.get("style", ""):
            continue
        text = "".join(element.itertext()).strip()
        if text:
            titles.append(text)
    return titles


def render_mermaid(input_path: Path, output_path: Path, cwd: Path) -> Path:
    if shutil.which("mmdc") is None:
        raise SystemExit(
            "mmdc was not found. Install @mermaid-js/mermaid-cli before rendering."
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    relative_output = output_path.relative_to(cwd)

    subprocess.run(
        ["mmdc", "-i", str(input_path), "-o", relative_output.as_posix()],
        cwd=cwd,
        check=True,
    )

    candidates = rendered_svg_candidates(output_path)
    if len(candidates) != 1:
        raise SystemExit(
            "Expected exactly one rendered SVG artifact.\n"
            f"  output root: {output_path.parent}\n"
            f"  candidates: {[candidate.name for candidate in candidates]}"
        )
    return candidates[0]


def check_render(input_path: Path, output_path: Path, cwd: Path) -> int:
    if not output_path.exists():
        raise SystemExit(f"Missing tracked diagram output: {output_path}")

    source_class_names = sorted(set(extract_mermaid_class_names(input_path.read_text(encoding="utf-8"))))
    tracked_class_titles = sorted(set(extract_svg_class_titles(output_path.read_text(encoding="utf-8"))))

    if source_class_names != tracked_class_titles:
        print("Tracked SVG does not match the Mermaid source class titles.", file=sys.stderr)
        print(f"  source : {source_class_names}", file=sys.stderr)
        print(f"  tracked: {tracked_class_titles}", file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory(dir=cwd, prefix="class_diagram_render_") as temp_dir:
        temp_output = Path(temp_dir) / output_path.name
        rendered_output = render_mermaid(input_path, temp_output, cwd)

        rendered_class_titles = sorted(set(extract_svg_class_titles(rendered_output.read_text(encoding="utf-8"))))
        if source_class_names != rendered_class_titles:
            print("Fresh render does not match the Mermaid source class titles.", file=sys.stderr)
            print(f"  source  : {source_class_names}", file=sys.stderr)
            print(f"  rendered: {rendered_class_titles}", file=sys.stderr)
            return 1

    print(f"Class diagram is up to date: {output_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Render or check the repo UML class diagram."
    )
    parser.add_argument(
        "--input",
        default=str(DEFAULT_INPUT),
        help="Path to the Mermaid markdown source (default: docs/CLASS_DIAGRAM.md).",
    )
    parser.add_argument(
        "--output",
        default=str(DEFAULT_OUTPUT),
        help="Path to the SVG output (default: docs/CLASS_DIAGRAM.svg).",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Re-render to a temporary file and verify the tracked SVG class titles.",
    )
    args = parser.parse_args()

    repo_root = repo_root_from_cwd()
    input_path = resolve_repo_path(repo_root, args.input)
    output_path = resolve_repo_path(repo_root, args.output)

    if not input_path.exists():
        raise SystemExit(f"Missing input diagram source: {input_path}")

    if args.check:
        return check_render(input_path, output_path, repo_root)

    with tempfile.TemporaryDirectory(dir=repo_root, prefix="class_diagram_render_") as temp_dir:
        temp_output = Path(temp_dir) / output_path.name
        rendered_output = render_mermaid(input_path, temp_output, repo_root)
        rendered_output.replace(output_path)

    print(f"Rendered class diagram to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
