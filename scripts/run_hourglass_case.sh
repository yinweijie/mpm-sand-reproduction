#!/usr/bin/env bash

set -euo pipefail

mode="${1:-release}"
steps="${2:-3000}"
output_root="${3:-outputs/hourglass_case}"
scene="scenes/hourglass.json"

if [[ "${mode}" == "release" ]]; then
    build_dir="build/Release"
    binary="${build_dir}/klar2016_sand"
else
    build_dir="build/Debug"
    binary="${build_dir}/klar2016_sand"
fi

frames_dir="${output_root}/frames"
video_path="${output_root}/hourglass.mp4"

echo "Building ${mode} binary..."
bash ./build.sh "${mode}" "${scene}"

echo "Running hourglass simulation with frame export..."
"./${binary}" \
    --scene "${scene}" \
    --steps "${steps}" \
    --export \
    --output-dir "${frames_dir}"

echo "Rendering MP4 animation..."
python3 tools/render_ply_sequence.py \
    --input-dir "${frames_dir}" \
    --scene "${scene}" \
    --fit-mode hybrid \
    --fps 20 \
    --point-size 1.2 \
    --output "${video_path}" \
    --title "klar2016 hourglass"

echo "Frames: ${frames_dir}"
echo "Video:  ${video_path}"
