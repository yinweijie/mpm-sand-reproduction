#!/usr/bin/env python3
"""
Regression test for the pile_from_spout baseline.

Usage:
  python3 scripts/regress_pile_from_spout_baseline.py
    Build the release binary, run a scratch export, and compare the result against
    outputs/baselines/pile_from_spout_16/manifest.json.

  python3 scripts/regress_pile_from_spout_baseline.py --check-only
    Verify an existing scratch export in outputs/.test/baseline_pile_from_spout_16
    without rebuilding or rerunning.

  python3 scripts/regress_pile_from_spout_baseline.py --update-baseline
    Rerun the scratch export, verify it, then replace the tracked baseline under
    outputs/baselines/pile_from_spout_16.

Skill:
  Use $mpm-siggraph-2017-baseline to rerun and verify the tracked baseline from Codex.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path


SCENE_PATH = Path("scenes/pile_from_spout.json")
STEPS = 16
TRACKED_BASELINE_ROOT = Path("outputs/baselines/pile_from_spout_16")
SCRATCH_ROOT = Path("outputs/.test/baseline_pile_from_spout_16")


def repo_root_from_cwd() -> Path:
    try:
        output = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"],
            text=True,
            stderr=subprocess.STDOUT,
        )
    except subprocess.CalledProcessError as exc:
        raise SystemExit(
            "Run this script from inside the mpm_SIGGRAPH_2017 repository."
        ) from exc
    return Path(output.strip())


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stream_command(command: list[str], cwd: Path, log_path: Path | None = None) -> None:
    log_handle = None
    try:
        if log_path is not None:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_handle = log_path.open("w", encoding="utf-8")

        process = subprocess.Popen(
            command,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        assert process.stdout is not None
        for line in process.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            if log_handle is not None:
                log_handle.write(line)
                log_handle.flush()

        return_code = process.wait()
        if return_code != 0:
            raise subprocess.CalledProcessError(return_code, command)
    finally:
        if log_handle is not None:
            log_handle.close()


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def ensure_under_outputs(repo_root: Path, path: Path) -> Path:
    outputs_root = (repo_root / "outputs").resolve()
    resolved = path.resolve()
    try:
        resolved.relative_to(outputs_root)
    except ValueError as exc:
        raise SystemExit(f"Refusing to touch path outside {outputs_root}: {resolved}") from exc
    return resolved


def build_release_binary(repo_root: Path, scene_path: Path) -> None:
    build_script = repo_root / "build.sh"
    print(f"Building release binary with {build_script} ...")
    stream_command(["bash", str(build_script), "release", str(scene_path)], cwd=repo_root)


def simulation_command_string(
    binary_path: Path,
    scene_path: Path,
    output_dir: Path,
    repo_root: Path,
) -> str:
    binary_rel = binary_path.relative_to(repo_root).as_posix()
    scene_rel = scene_path.relative_to(repo_root).as_posix()
    output_rel = output_dir.relative_to(repo_root).as_posix()
    return (
        f"./{binary_rel} "
        f"--scene {scene_rel} "
        f"--steps {STEPS} "
        f"--export "
        f"--output-dir {output_rel}"
    )


def run_export(
    repo_root: Path,
    binary_path: Path,
    scene_path: Path,
    output_root: Path,
    steps: int,
) -> Path:
    if output_root.exists():
        shutil.rmtree(output_root)
    frames_dir = output_root / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    run_log_path = output_root / "run.log"
    command = [
        str(binary_path),
        "--scene",
        str(scene_path),
        "--steps",
        str(steps),
        "--export",
        "--output-dir",
        str(frames_dir),
    ]
    print(f"Running baseline export into {output_root} ...")
    stream_command(command, cwd=repo_root, log_path=run_log_path)
    return run_log_path


def parse_summary_line(run_log: str) -> str | None:
    for line in run_log.splitlines():
        if line.startswith("KLAR2016 development step:"):
            return line.strip()
    return None


def verify_output(output_root: Path, reference: dict) -> None:
    run_log_path = output_root / "run.log"
    if not run_log_path.exists():
        raise SystemExit(f"Missing run log: {run_log_path}")

    run_log = run_log_path.read_text(encoding="utf-8")
    actual_summary = parse_summary_line(run_log)
    expected_summary = reference["summary_line"]
    if actual_summary != expected_summary:
        raise SystemExit(
            "Summary line mismatch.\n"
            f"Expected: {expected_summary}\n"
            f"Actual:   {actual_summary}"
        )

    frames_dir = output_root / "frames"
    expected_frames = reference["frames"]
    actual_frames = sorted(frames_dir.glob("*.ply"))
    if len(actual_frames) != len(expected_frames):
        raise SystemExit(
            f"Frame count mismatch: expected {len(expected_frames)}, got {len(actual_frames)}"
        )

    for expected, actual in zip(expected_frames, actual_frames, strict=True):
        if actual.name != Path(expected["file"]).name:
            raise SystemExit(
                f"Frame name mismatch: expected {expected['file']}, got {actual.name}"
            )
        actual_hash = sha256_file(actual)
        if actual_hash != expected["sha256"]:
            raise SystemExit(
                f"Hash mismatch for {actual.name}: expected {expected['sha256']}, got {actual_hash}"
            )


def write_manifest(
    output_root: Path,
    reference: dict,
    repo_root: Path,
    binary_path: Path,
    scene_path: Path,
) -> None:
    output_dir = output_root / "frames"
    manifest = {
        "name": reference["name"],
        "generated_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "git_commit": subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=repo_root,
            text=True,
        ).strip(),
        "output_root": output_root.relative_to(repo_root).as_posix(),
        "command": simulation_command_string(binary_path, scene_path, output_dir, repo_root),
        "scene": reference["scene"],
        "steps": reference["steps"],
        "output_dir": output_dir.relative_to(repo_root).as_posix(),
        "summary_line": reference["summary_line"],
        "stats": reference["stats"],
        "frames": reference["frames"],
    }
    (output_root / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n",
        encoding="utf-8",
    )


def update_tracked_baseline(scratch_root: Path, tracked_root: Path) -> None:
    if tracked_root.exists():
        shutil.rmtree(tracked_root)
    shutil.copytree(scratch_root, tracked_root)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Rebuild and verify the tracked pile_from_spout baseline."
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Verify an existing scratch export without rebuilding or rerunning.",
    )
    parser.add_argument(
        "--update-baseline",
        action="store_true",
        help="Replace outputs/baselines/pile_from_spout_16 after a successful verification.",
    )
    args = parser.parse_args()

    repo_root = repo_root_from_cwd()
    scene_path = repo_root / SCENE_PATH
    tracked_root = ensure_under_outputs(repo_root, repo_root / TRACKED_BASELINE_ROOT)
    scratch_root = ensure_under_outputs(repo_root, repo_root / SCRATCH_ROOT)
    reference_path = tracked_root / "manifest.json"

    if not reference_path.exists():
        raise SystemExit(f"Tracked baseline manifest not found: {reference_path}")

    reference = load_json(reference_path)
    binary_path = repo_root / "build" / "Release" / "klar2016_sand"

    if not args.check_only:
        build_release_binary(repo_root, scene_path)
        run_export(repo_root, binary_path, scene_path, scratch_root, STEPS)

    verify_output(scratch_root, reference)
    write_manifest(scratch_root, reference, repo_root, binary_path, scene_path)

    if args.update_baseline:
        print(f"Updating tracked baseline at {tracked_root} ...")
        update_tracked_baseline(scratch_root, tracked_root)
        write_manifest(tracked_root, reference, repo_root, binary_path, scene_path)

    print(f"Regression check passed: {scratch_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
