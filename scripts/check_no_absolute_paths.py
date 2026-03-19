#!/usr/bin/env python3
"""
Check staged or tracked files for machine-specific absolute paths.

Usage:
  python3 scripts/check_no_absolute_paths.py
    Scan the staged content of changed files. This is the mode used by the
    pre-commit hook.

  python3 scripts/check_no_absolute_paths.py --all-tracked
    Scan every tracked file in the working tree.

The checker is intentionally focused on repository-host specific absolute paths,
such as /home/<user>, /Users/<user>, and the current clone's absolute root.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Violation:
    path: str
    line_no: int
    match: str
    line: str


def repo_root_from_cwd() -> Path:
    try:
        output = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"],
            text=True,
            stderr=subprocess.STDOUT,
        )
    except subprocess.CalledProcessError as exc:
        raise SystemExit("Run this checker from inside a git repository.") from exc
    return Path(output.strip())


def list_staged_paths(repo_root: Path) -> list[str]:
    output = subprocess.check_output(
        ["git", "diff", "--cached", "--name-only", "--diff-filter=ACMR", "-z"],
        cwd=repo_root,
    )
    return [path for path in output.decode("utf-8", errors="strict").split("\0") if path]


def list_tracked_paths(repo_root: Path) -> list[str]:
    output = subprocess.check_output(["git", "ls-files", "-z"], cwd=repo_root)
    return [path for path in output.decode("utf-8", errors="strict").split("\0") if path]


def load_staged_blob(repo_root: Path, rel_path: str) -> bytes:
    return subprocess.check_output(["git", "show", f":{rel_path}"], cwd=repo_root)


def load_working_tree_blob(repo_root: Path, rel_path: str) -> bytes:
    return (repo_root / rel_path).read_bytes()


def compile_forbidden_patterns(repo_root: Path) -> list[re.Pattern[str]]:
    repo_root_abs = repo_root.resolve().as_posix()
    home_abs = Path.home().resolve().as_posix()

    return [
        re.compile(re.escape(repo_root_abs)),
        re.compile(re.escape(home_abs)),
        re.compile(r"/home/[A-Za-z0-9._-]+(?:/[^\s\"'`<>]*)?"),
        re.compile(r"/Users/[A-Za-z0-9._-]+(?:/[^\s\"'`<>]*)?"),
    ]


def scan_text(rel_path: str, text: str, patterns: list[re.Pattern[str]]) -> list[Violation]:
    violations: list[Violation] = []
    for line_no, line in enumerate(text.splitlines(), start=1):
        for pattern in patterns:
            match = pattern.search(line)
            if match is not None:
                violations.append(
                    Violation(
                        path=rel_path,
                        line_no=line_no,
                        match=match.group(0),
                        line=line.rstrip(),
                    )
                )
    return violations


def print_report(violations: list[Violation]) -> None:
    print("Forbidden absolute paths found:")
    for violation in violations:
        print(
            f"  {violation.path}:{violation.line_no}: "
            f"{violation.match} | {violation.line}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check for machine-specific absolute paths in repository files."
    )
    parser.add_argument(
        "--all-tracked",
        action="store_true",
        help="Scan all tracked files in the working tree instead of staged content.",
    )
    args = parser.parse_args()

    repo_root = repo_root_from_cwd()
    patterns = compile_forbidden_patterns(repo_root)

    if args.all_tracked:
        rel_paths = list_tracked_paths(repo_root)
        loader = lambda rel_path: load_working_tree_blob(repo_root, rel_path)
        source_label = "tracked files"
    else:
        rel_paths = list_staged_paths(repo_root)
        loader = lambda rel_path: load_staged_blob(repo_root, rel_path)
        source_label = "staged files"

    violations: list[Violation] = []
    for rel_path in rel_paths:
        try:
            blob = loader(rel_path)
        except (FileNotFoundError, subprocess.CalledProcessError):
            continue

        if b"\0" in blob:
            continue

        text = blob.decode("utf-8", errors="ignore")
        violations.extend(scan_text(rel_path, text, patterns))

    if violations:
        print_report(violations)
        return 1

    print(f"No forbidden absolute paths found in {len(rel_paths)} {source_label}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
