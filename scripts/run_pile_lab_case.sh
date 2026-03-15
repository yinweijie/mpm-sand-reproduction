#!/usr/bin/env bash

set -euo pipefail

mode="${1:-release}"
steps="${2:-2400}"
output_root="${3:-outputs/pile_lab_case}"
friction_angle="${4:-50}"
scene="scenes/pile_lab.json"

if [[ "${mode}" == "release" ]]; then
    build_dir="build/Release"
    binary="${build_dir}/klar2016_sand"
else
    build_dir="build/Debug"
    binary="${build_dir}/klar2016_sand"
fi

frames_dir="${output_root}/frames"
video_path="${output_root}/pile_lab.mp4"

echo "Building ${mode} binary..."
bash ./build.sh "${mode}" "${scene}"

echo "Running pile lab simulation with frame export..."
"./${binary}" \
    --scene "${scene}" \
    --friction-angle "${friction_angle}" \
    --steps "${steps}" \
    --export \
    --output-dir "${frames_dir}"

echo "Rendering preview MP4 with matplotlib..."
python3 tools/render_ply_sequence.py \
    --input-dir "${frames_dir}" \
    --scene "${scene}" \
    --fit-mode hybrid \
    --camera-preset pile_lab \
    --fps 20 \
    --point-size 4.0 \
    --output "${video_path}" \
    --title "klar2016 pile lab phi${friction_angle}"

echo "Frames: ${frames_dir}"
echo "Preview: ${video_path}"
echo "For a reliable final render, use:"
echo "  scripts/render_pile_lab_blender.sh ${frames_dir} ${output_root}/pile_lab_blender.mp4 ${scene}"
