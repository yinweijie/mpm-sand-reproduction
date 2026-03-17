#!/usr/bin/env bash

set -euo pipefail

frames_dir="${1:-outputs/.test/pile_lab_case/frames}"
output_path="${2:-outputs/.test/pile_lab_case/pile_lab_blender.mp4}"
scene_path="${3:-scenes/pile_lab.json}"
shading_mode="${4:-studio}"
render_mode="${5:-points}"

default_blender="/home/ywj22/blender-5.0.1-linux-x64/blender"
if [[ -n "${BLENDER_BIN:-}" ]]; then
    blender_bin="${BLENDER_BIN}"
elif [[ -x "${default_blender}" ]]; then
    blender_bin="${default_blender}"
else
    blender_bin="blender"
fi

if ! command -v "${blender_bin}" >/dev/null 2>&1; then
    echo "Blender not found. Set BLENDER_BIN or install blender first." >&2
    exit 1
fi

mkdir -p "$(dirname "${output_path}")"

"${blender_bin}" -b -P tools/render_ply_sequence_blender.py -- \
    --input-dir "${frames_dir}" \
    --scene "${scene_path}" \
    --output "${output_path}" \
    --engine CYCLES \
    --samples 64 \
    --resolution 960 \
    --frame-repeat 1 \
    --shading-mode "${shading_mode}" \
    --render-mode "${render_mode}"

echo "Blender render written to ${output_path}"
