#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render klar2016 PLY particle frames to an MP4 animation."
    )
    parser.add_argument("--input-dir", required=True, help="Directory containing frame_*.ply files.")
    parser.add_argument("--output", required=True, help="Output .mp4 path.")
    parser.add_argument("--scene", help="Optional scene JSON for fixed domain bounds.")
    parser.add_argument(
        "--fit-mode",
        choices=("scene", "particle", "hybrid"),
        default="scene",
        help="How to choose view bounds: full scene, particle bounds, or a cropped hybrid view.",
    )
    parser.add_argument("--fps", type=int, default=24, help="Video frames per second.")
    parser.add_argument(
        "--min-duration-seconds",
        type=float,
        default=2.0,
        help="Minimum output duration for short frame sequences.",
    )
    parser.add_argument("--dpi", type=int, default=140, help="Figure DPI for MP4 output.")
    parser.add_argument("--point-size", type=float, default=2.0, help="Scatter point size.")
    parser.add_argument("--elev", type=float, default=18.0, help="Camera elevation angle.")
    parser.add_argument("--azim", type=float, default=-62.0, help="Camera azimuth angle.")
    parser.add_argument(
        "--camera-preset",
        choices=("default", "pile_lab"),
        default="default",
        help="Optional view and styling preset.",
    )
    parser.add_argument(
        "--hide-domain-box",
        action="store_true",
        help="Hide the scene bounding box wireframe.",
    )
    parser.add_argument(
        "--padding-ratio",
        type=float,
        default=0.10,
        help="Relative padding applied around particle bounds when fitting the camera.",
    )
    parser.add_argument(
        "--hybrid-min-fraction",
        type=float,
        default=0.45,
        help="Minimum visible extent relative to the scene box for hybrid framing.",
    )
    parser.add_argument(
        "--title",
        default="klar2016 box case",
        help="Figure title shown above the animation.",
    )
    parser.add_argument(
        "--show-axes",
        action="store_true",
        help="Show matplotlib 3D axes, ticks, and labels.",
    )
    return parser.parse_args()


def expand_hourglass_shell(scene: dict) -> list[dict]:
    shell = scene.get("hourglass_shell")
    if not isinstance(shell, dict) or not shell.get("enabled", True):
        return []

    center = np.array(shell["waist_center"], dtype=float)
    top_y = float(shell["top_y"])
    bottom_y = float(shell["bottom_y"])
    neck_half_width = float(shell["neck_half_width"])
    top_half_width = float(shell["top_half_width"])
    bottom_half_width = float(shell["bottom_half_width"])
    upper_mu = float(shell.get("upper_friction_coefficient", 0.55))
    lower_mu = float(shell.get("lower_friction_coefficient", 0.60))
    add_bottom_cap = bool(shell.get("add_bottom_cap", False))
    bottom_cap_mu = float(shell.get("bottom_cap_friction_coefficient", 0.85))

    if not (bottom_y < center[1] < top_y):
        raise RuntimeError("hourglass_shell requires bottom_y < waist_center.y < top_y")

    upper_slope = (top_half_width - neck_half_width) / (top_y - center[1])
    lower_slope = (bottom_half_width - neck_half_width) / (center[1] - bottom_y)
    pad = 1.0e-3

    def collider(point: np.ndarray, normal: np.ndarray, mu: float, bounds_min: np.ndarray, bounds_max: np.ndarray) -> dict:
        return {
            "point": point.tolist(),
            "normal": (normal / np.linalg.norm(normal)).tolist(),
            "friction_coefficient": mu,
            "bounds_min": bounds_min.tolist(),
            "bounds_max": bounds_max.tolist(),
        }

    colliders = [
        collider(
            np.array([center[0] - neck_half_width, center[1], center[2]]),
            np.array([1.0, upper_slope, 0.0]),
            upper_mu,
            np.array([center[0] - top_half_width - pad, center[1] - pad, center[2] - top_half_width - pad]),
            np.array([center[0] - neck_half_width + pad, top_y + pad, center[2] + top_half_width + pad]),
        ),
        collider(
            np.array([center[0] + neck_half_width, center[1], center[2]]),
            np.array([-1.0, upper_slope, 0.0]),
            upper_mu,
            np.array([center[0] + neck_half_width - pad, center[1] - pad, center[2] - top_half_width - pad]),
            np.array([center[0] + top_half_width + pad, top_y + pad, center[2] + top_half_width + pad]),
        ),
        collider(
            np.array([center[0], center[1], center[2] - neck_half_width]),
            np.array([0.0, upper_slope, 1.0]),
            upper_mu,
            np.array([center[0] - top_half_width - pad, center[1] - pad, center[2] - top_half_width - pad]),
            np.array([center[0] + top_half_width + pad, top_y + pad, center[2] - neck_half_width + pad]),
        ),
        collider(
            np.array([center[0], center[1], center[2] + neck_half_width]),
            np.array([0.0, upper_slope, -1.0]),
            upper_mu,
            np.array([center[0] - top_half_width - pad, center[1] - pad, center[2] + neck_half_width - pad]),
            np.array([center[0] + top_half_width + pad, top_y + pad, center[2] + top_half_width + pad]),
        ),
        collider(
            np.array([center[0] - neck_half_width, center[1], center[2]]),
            np.array([1.0, -lower_slope, 0.0]),
            lower_mu,
            np.array([center[0] - bottom_half_width - pad, bottom_y - pad, center[2] - bottom_half_width - pad]),
            np.array([center[0] - neck_half_width + pad, center[1] + pad, center[2] + bottom_half_width + pad]),
        ),
        collider(
            np.array([center[0] + neck_half_width, center[1], center[2]]),
            np.array([-1.0, -lower_slope, 0.0]),
            lower_mu,
            np.array([center[0] + neck_half_width - pad, bottom_y - pad, center[2] - bottom_half_width - pad]),
            np.array([center[0] + bottom_half_width + pad, center[1] + pad, center[2] + bottom_half_width + pad]),
        ),
        collider(
            np.array([center[0], center[1], center[2] - neck_half_width]),
            np.array([0.0, -lower_slope, 1.0]),
            lower_mu,
            np.array([center[0] - bottom_half_width - pad, bottom_y - pad, center[2] - bottom_half_width - pad]),
            np.array([center[0] + bottom_half_width + pad, center[1] + pad, center[2] - neck_half_width + pad]),
        ),
        collider(
            np.array([center[0], center[1], center[2] + neck_half_width]),
            np.array([0.0, -lower_slope, -1.0]),
            lower_mu,
            np.array([center[0] - bottom_half_width - pad, bottom_y - pad, center[2] + neck_half_width - pad]),
            np.array([center[0] + bottom_half_width + pad, center[1] + pad, center[2] + bottom_half_width + pad]),
        ),
    ]

    if add_bottom_cap:
        colliders.append(
            collider(
                np.array([center[0], bottom_y, center[2]]),
                np.array([0.0, 1.0, 0.0]),
                bottom_cap_mu,
                np.array([center[0] - bottom_half_width - pad, bottom_y - pad, center[2] - bottom_half_width - pad]),
                np.array([center[0] + bottom_half_width + pad, bottom_y + pad, center[2] + bottom_half_width + pad]),
            )
        )

    return colliders


def expand_cylinder_shells(scene: dict) -> list[dict]:
    return list(scene.get("cylinder_shells", []))


def load_scene_data(scene_path: Path) -> tuple[np.ndarray, np.ndarray, list[dict], list[dict]]:
    root = json.loads(scene_path.read_text(encoding="utf-8"))
    scene = root["scene"]
    colliders = list(scene.get("colliders", []))
    colliders.extend(expand_hourglass_shell(scene))
    cylinder_shells = expand_cylinder_shells(scene)
    return (
        np.array(scene["domain_min"], dtype=float),
        np.array(scene["domain_max"], dtype=float),
        colliders,
        cylinder_shells,
    )


def list_frames(input_dir: Path) -> list[Path]:
    frames = sorted(input_dir.glob("frame_*.ply"))
    if not frames:
        raise FileNotFoundError(f"No PLY frames found under '{input_dir}'.")
    return frames


def read_ply_points(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8") as handle:
        vertex_count = None
        while True:
            line = handle.readline()
            if line == "":
                raise RuntimeError(f"Unexpected EOF while parsing header in '{path}'.")
            stripped = line.strip()
            if stripped.startswith("element vertex "):
                vertex_count = int(stripped.split()[-1])
            if stripped == "end_header":
                break

        if vertex_count is None:
            raise RuntimeError(f"PLY header in '{path}' is missing 'element vertex'.")

        if vertex_count == 0:
            return np.zeros((0, 8), dtype=float)

        data = np.loadtxt(handle, dtype=float)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        return data


def infer_bounds(frames: Iterable[Path]) -> tuple[np.ndarray, np.ndarray]:
    bbox_min = np.array([np.inf, np.inf, np.inf], dtype=float)
    bbox_max = np.array([-np.inf, -np.inf, -np.inf], dtype=float)

    for frame in frames:
        data = read_ply_points(frame)
        if data.size == 0:
            continue
        positions = data[:, :3]
        bbox_min = np.minimum(bbox_min, positions.min(axis=0))
        bbox_max = np.maximum(bbox_max, positions.max(axis=0))

    if not np.all(np.isfinite(bbox_min)) or not np.all(np.isfinite(bbox_max)):
        raise RuntimeError("Unable to infer bounds from an empty frame sequence.")

    padding = np.maximum(0.02 * (bbox_max - bbox_min), 1.0e-3)
    return bbox_min - padding, bbox_max + padding


def particle_bounds(frame: Path) -> tuple[np.ndarray, np.ndarray]:
    data = read_ply_points(frame)
    if data.size == 0:
        raise RuntimeError(f"Frame '{frame}' contains no particles.")
    positions = data[:, :3]
    return positions.min(axis=0), positions.max(axis=0)


def fit_particle_bounds(
    bbox_min: np.ndarray,
    bbox_max: np.ndarray,
    padding_ratio: float,
) -> tuple[np.ndarray, np.ndarray]:
    extent = np.maximum(bbox_max - bbox_min, 1.0e-4)
    padding = np.maximum(padding_ratio * extent, 1.0e-3)
    return bbox_min - padding, bbox_max + padding


def hybrid_bounds(
    scene_min: np.ndarray,
    scene_max: np.ndarray,
    particle_min: np.ndarray,
    particle_max: np.ndarray,
    padding_ratio: float,
    min_fraction: float,
) -> tuple[np.ndarray, np.ndarray]:
    fit_min, fit_max = fit_particle_bounds(particle_min, particle_max, padding_ratio)
    scene_extent = scene_max - scene_min
    fit_extent = fit_max - fit_min
    min_extent = np.maximum(scene_extent * min_fraction, 1.0e-3)
    target_extent = np.maximum(fit_extent, min_extent)
    center = 0.5 * (fit_min + fit_max)
    result_min = center - 0.5 * target_extent
    result_max = center + 0.5 * target_extent

    for axis in range(3):
        if target_extent[axis] >= scene_extent[axis]:
            result_min[axis] = scene_min[axis]
            result_max[axis] = scene_max[axis]
            continue

        if result_min[axis] < scene_min[axis]:
            shift = scene_min[axis] - result_min[axis]
            result_min[axis] += shift
            result_max[axis] += shift
        if result_max[axis] > scene_max[axis]:
            shift = result_max[axis] - scene_max[axis]
            result_min[axis] -= shift
            result_max[axis] -= shift

    return result_min, result_max


def draw_box(ax: plt.Axes, bbox_min: np.ndarray, bbox_max: np.ndarray) -> None:
    x0, y0, z0 = bbox_min
    x1, y1, z1 = bbox_max
    corners = np.array(
        [
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1],
        ],
        dtype=float,
    )
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    for start, end in edges:
        ax.plot(
            [corners[start, 0], corners[end, 0]],
            [corners[start, 2], corners[end, 2]],
            [corners[start, 1], corners[end, 1]],
            color="#3a3025",
            linewidth=0.8,
            alpha=0.55,
        )


def compute_plane_patch(collider: dict) -> np.ndarray | None:
    if "bounds_min" not in collider or "bounds_max" not in collider:
        return None

    point = np.array(collider["point"], dtype=float)
    normal = np.array(collider["normal"], dtype=float)
    normal_norm = np.linalg.norm(normal)
    if normal_norm <= 1.0e-12:
        return None
    normal /= normal_norm

    bounds_min = np.array(collider["bounds_min"], dtype=float)
    bounds_max = np.array(collider["bounds_max"], dtype=float)
    x0, y0, z0 = bounds_min
    x1, y1, z1 = bounds_max
    corners = np.array(
        [
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1],
        ],
        dtype=float,
    )
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]

    points: list[np.ndarray] = []
    epsilon = 1.0e-9
    for start, end in edges:
        p0 = corners[start]
        p1 = corners[end]
        d0 = float(np.dot(p0 - point, normal))
        d1 = float(np.dot(p1 - point, normal))
        if abs(d0) <= epsilon:
            points.append(p0)
        if abs(d1) <= epsilon:
            points.append(p1)
        if d0 * d1 < 0.0:
            t = d0 / (d0 - d1)
            points.append(p0 + t * (p1 - p0))

    unique: list[np.ndarray] = []
    for candidate in points:
        if not any(np.linalg.norm(candidate - existing) <= 1.0e-7 for existing in unique):
            unique.append(candidate)

    if len(unique) < 3:
        return None

    centroid = sum(unique) / len(unique)
    reference = np.array([1.0, 0.0, 0.0], dtype=float)
    if abs(float(np.dot(reference, normal))) > 0.9:
        reference = np.array([0.0, 1.0, 0.0], dtype=float)
    tangent_u = np.cross(normal, reference)
    tangent_u /= np.linalg.norm(tangent_u)
    tangent_v = np.cross(normal, tangent_u)

    def angle_of(vertex: np.ndarray) -> float:
        offset = vertex - centroid
        return math.atan2(float(np.dot(offset, tangent_v)), float(np.dot(offset, tangent_u)))

    ordered = sorted(unique, key=angle_of)
    return np.array(ordered, dtype=float)


def draw_plane_patch(ax: plt.Axes, polygon: np.ndarray) -> None:
    vertices = [[(float(p[0]), float(p[2]), float(p[1])) for p in polygon]]
    patch = Poly3DCollection(
        vertices,
        facecolor="#c7a67f",
        edgecolor="#6a4f3a",
        linewidth=0.9,
        alpha=0.18,
    )
    ax.add_collection3d(patch)


def draw_cylinder_shell(
    ax: plt.Axes,
    shell: dict,
    color: str = "#8d7156",
    linewidth: float = 1.2,
    alpha: float = 0.85,
) -> None:
    center = np.array(shell["center"], dtype=float)
    radius = float(shell["radius"])
    y_min = center[1] + float(shell["y_min"])
    y_max = center[1] + float(shell["y_max"])
    theta = np.linspace(0.0, 2.0 * math.pi, 96)
    xs = center[0] + radius * np.cos(theta)
    zs = center[2] + radius * np.sin(theta)
    ax.plot(xs, zs, np.full_like(xs, y_min), color=color, linewidth=linewidth, alpha=alpha)
    ax.plot(xs, zs, np.full_like(xs, y_max), color=color, linewidth=linewidth, alpha=alpha)
    for angle in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi):
        x = center[0] + radius * math.cos(angle)
        z = center[2] + radius * math.sin(angle)
        ax.plot([x, x], [z, z], [y_min, y_max], color=color, linewidth=linewidth, alpha=alpha)


def draw_plane_outline(
    ax: plt.Axes,
    polygon: np.ndarray,
    color: str = "#6a4f3a",
    linewidth: float = 1.0,
    alpha: float = 0.8,
) -> None:
    closed = np.vstack([polygon, polygon[0]])
    ax.plot(
        closed[:, 0],
        closed[:, 2],
        closed[:, 1],
        color=color,
        linewidth=linewidth,
        alpha=alpha,
    )


def blur_image(image: np.ndarray) -> np.ndarray:
    kernel = np.array([1.0, 4.0, 6.0, 4.0, 1.0], dtype=float)
    kernel /= kernel.sum()
    blurred = np.apply_along_axis(lambda row: np.convolve(row, kernel, mode="same"), 1, image)
    blurred = np.apply_along_axis(lambda col: np.convolve(col, kernel, mode="same"), 0, blurred)
    return blurred


def smooth_signal(values: np.ndarray, passes: int = 3) -> np.ndarray:
    if len(values) == 0:
        return values
    kernel = np.array([1.0, 4.0, 6.0, 4.0, 1.0], dtype=float)
    kernel /= kernel.sum()
    result = values.copy()
    for _ in range(passes):
        padded = np.pad(result, (2, 2), mode="edge")
        result = np.convolve(padded, kernel, mode="valid")
    return result


def build_radial_pile_profile(
    positions: np.ndarray,
    center_xz: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    radii = np.linalg.norm(positions[:, [0, 2]] - center_xz[None, :], axis=1)
    if len(radii) == 0:
        return np.array([0.0, 0.01], dtype=float), np.array([0.0, 0.0], dtype=float)

    outer_radius = max(float(np.quantile(radii, 0.995)) * 1.05, 0.008)
    edges = np.linspace(0.0, outer_radius, 36)
    centers = 0.5 * (edges[:-1] + edges[1:])
    heights = np.full(len(centers), np.nan, dtype=float)

    for index in range(len(centers)):
        mask = (radii >= edges[index]) & (radii < edges[index + 1])
        if np.count_nonzero(mask) < 16:
            continue
        heights[index] = float(np.quantile(positions[mask, 1], 0.97))

    valid = np.flatnonzero(np.isfinite(heights))
    if len(valid) == 0:
        peak_height = float(np.quantile(positions[:, 1], 0.98))
        return np.array([0.0, outer_radius], dtype=float), np.array([peak_height, 0.0], dtype=float)

    heights[: valid[0]] = heights[valid[0]]
    heights[valid[-1] + 1 :] = heights[valid[-1]]
    missing = ~np.isfinite(heights)
    if np.any(missing):
        heights[missing] = np.interp(
            centers[missing],
            centers[valid],
            heights[valid],
        )

    heights = smooth_signal(heights, passes=3)
    peak_index = int(np.argmax(heights))
    for index in range(peak_index + 1, len(heights)):
        heights[index] = min(heights[index], heights[index - 1])

    heights = np.maximum(heights, 0.0)
    radii_profile = np.concatenate(([0.0], centers, [outer_radius * 1.06]))
    heights_profile = np.concatenate(([heights[0]], heights, [0.0]))
    return radii_profile, heights_profile


def build_revolved_surface(
    center_xz: np.ndarray,
    radii: np.ndarray,
    heights: np.ndarray,
    theta_samples: int = 120,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    theta = np.linspace(0.0, 2.0 * math.pi, theta_samples)
    radius_grid, theta_grid = np.meshgrid(radii, theta, indexing="xy")
    height_grid, _ = np.meshgrid(heights, theta, indexing="xy")
    x_grid = center_xz[0] + radius_grid * np.cos(theta_grid)
    z_grid = center_xz[1] + radius_grid * np.sin(theta_grid)
    return x_grid, z_grid, height_grid


def build_floor_patch(center_xz: np.ndarray, half_extent: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    xs = np.linspace(center_xz[0] - half_extent, center_xz[0] + half_extent, 2)
    zs = np.linspace(center_xz[1] - half_extent, center_xz[1] + half_extent, 2)
    x_grid, z_grid = np.meshgrid(xs, zs)
    y_grid = np.zeros_like(x_grid)
    return x_grid, z_grid, y_grid


def rotation_y(angle_radians: float) -> np.ndarray:
    c = math.cos(angle_radians)
    s = math.sin(angle_radians)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=float,
    )


def rotation_x(angle_radians: float) -> np.ndarray:
    c = math.cos(angle_radians)
    s = math.sin(angle_radians)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=float,
    )


def project_pile_lab_points(
    points: np.ndarray,
    look_at: np.ndarray,
    yaw_degrees: float,
    pitch_degrees: float,
    vertical_scale: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rotation = rotation_x(math.radians(pitch_degrees)) @ rotation_y(math.radians(yaw_degrees))
    camera = (points - look_at[None, :]) @ rotation.T
    return camera[:, 0], camera[:, 1] * vertical_scale, camera[:, 2]


def build_pile_lab_render_setup(
    frame_paths: list[Path],
    cylinder_shells: list[dict],
) -> dict:
    final_positions = None
    for frame_path in reversed(frame_paths):
        data = read_ply_points(frame_path)
        if data.size > 0:
            final_positions = data[:, :3]
            break

    if final_positions is None:
        raise RuntimeError("pile_lab rendering requires at least one non-empty frame.")

    if cylinder_shells:
        center_xz = np.array(cylinder_shells[0]["center"], dtype=float)[[0, 2]]
    else:
        center_xz = np.median(final_positions[:, [0, 2]], axis=0)

    look_at = np.array([center_xz[0], 0.0045, center_xz[1]], dtype=float)
    yaw_degrees = 38.0
    pitch_degrees = 20.0
    vertical_scale = 1.22

    projected_u, projected_v, _ = project_pile_lab_points(
        final_positions,
        look_at,
        yaw_degrees,
        pitch_degrees,
        vertical_scale,
    )
    u_low, u_high = np.quantile(projected_u, [0.01, 0.99])
    v_low, v_high = np.quantile(projected_v, [0.01, 0.99])
    u_span = max(float(u_high - u_low), 1.0e-4)
    v_span = max(float(v_high - v_low), 1.0e-4)

    radial_distance = np.linalg.norm(final_positions[:, [0, 2]] - center_xz[None, :], axis=1)
    floor_half_extent = max(float(np.quantile(radial_distance, 0.995)) * 1.02, 0.011)
    floor_polygon = np.array(
        [
            [center_xz[0] - floor_half_extent, 0.0, center_xz[1] - floor_half_extent],
            [center_xz[0] + floor_half_extent, 0.0, center_xz[1] - floor_half_extent],
            [center_xz[0] + floor_half_extent, 0.0, center_xz[1] + floor_half_extent],
            [center_xz[0] - floor_half_extent, 0.0, center_xz[1] + floor_half_extent],
        ],
        dtype=float,
    )
    floor_u, floor_v, _ = project_pile_lab_points(
        floor_polygon,
        look_at,
        yaw_degrees,
        pitch_degrees,
        vertical_scale,
    )

    return {
        "look_at": look_at,
        "yaw_degrees": yaw_degrees,
        "pitch_degrees": pitch_degrees,
        "vertical_scale": vertical_scale,
        "u_min": min(float(floor_u.min()), float(u_low - 0.14 * u_span)),
        "u_max": max(float(floor_u.max()), float(u_high + 0.14 * u_span)),
        "v_min": min(float(floor_v.min()) - 0.02 * v_span, float(v_low - 0.16 * v_span)),
        "v_max": max(float(v_high + 0.12 * v_span), float(floor_v.max()) + 0.04 * v_span),
        "floor_uv": np.column_stack([floor_u, floor_v]),
    }


def rasterize_pile_lab_particles(
    positions: np.ndarray,
    setup: dict,
    width: int,
    height: int,
    sand_cmap: LinearSegmentedColormap,
) -> np.ndarray:
    projected_u, projected_v, _ = project_pile_lab_points(
        positions,
        setup["look_at"],
        setup["yaw_degrees"],
        setup["pitch_degrees"],
        setup["vertical_scale"],
    )
    valid = (
        (projected_u >= setup["u_min"]) &
        (projected_u <= setup["u_max"]) &
        (projected_v >= setup["v_min"]) &
        (projected_v <= setup["v_max"])
    )
    if not np.any(valid):
        return np.zeros((height, width, 4), dtype=float)

    projected_u = projected_u[valid]
    projected_v = projected_v[valid]
    visible_positions = positions[valid]

    x_idx = np.clip(
        ((projected_u - setup["u_min"]) / max(setup["u_max"] - setup["u_min"], 1.0e-9) * (width - 1)).astype(int),
        0,
        width - 1,
    )
    y_idx = np.clip(
        ((projected_v - setup["v_min"]) / max(setup["v_max"] - setup["v_min"], 1.0e-9) * (height - 1)).astype(int),
        0,
        height - 1,
    )

    density = np.zeros((height, width), dtype=float)
    color_sum = np.zeros((height, width, 3), dtype=float)
    if len(visible_positions) == 0:
        return np.zeros((height, width, 4), dtype=float)

    y_min = float(np.quantile(visible_positions[:, 1], 0.02))
    y_max = float(np.quantile(visible_positions[:, 1], 0.995))
    height_norm = np.clip(
        (visible_positions[:, 1] - y_min) / max(y_max - y_min, 1.0e-6),
        0.0,
        1.0,
    )
    colors = sand_cmap(0.18 + 0.78 * height_norm)[:, :3]

    np.add.at(density, (y_idx, x_idx), 1.0)
    for channel in range(3):
        np.add.at(color_sum[..., channel], (y_idx, x_idx), colors[:, channel])

    density = blur_image(blur_image(blur_image(density)))
    for channel in range(3):
        color_sum[..., channel] = blur_image(blur_image(blur_image(color_sum[..., channel])))

    positive = density > 1.0e-8
    image = np.zeros((height, width, 4), dtype=float)
    if not np.any(positive):
        return image

    image[..., :3] = color_sum / np.maximum(density[..., None], 1.0e-8)
    density_scale = max(float(np.quantile(density[positive], 0.985)), 1.0e-8)
    image[..., 3] = np.clip(density / density_scale, 0.0, 1.0) ** 0.72
    return image


def render_pile_lab_video(
    args: argparse.Namespace,
    frame_paths: list[Path],
    scene_cylinder_shells: list[dict],
    output_path: Path,
) -> None:
    if not FFMpegWriter.isAvailable():
        raise RuntimeError("matplotlib FFmpeg writer is not available on this system.")

    setup = build_pile_lab_render_setup(frame_paths, scene_cylinder_shells)
    sand_cmap = LinearSegmentedColormap.from_list(
        "pile_lab_sand",
        ["#2f1a12", "#6c4328", "#b67a43", "#eab66f"],
    )

    figure, axes = plt.subplots(figsize=(6.0, 6.0))
    figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=0.92)
    figure.patch.set_facecolor("#f4f1eb")
    axes.set_facecolor("#fbf8f2")
    figure.suptitle(args.title, y=0.965, fontsize=12)

    writer = FFMpegWriter(
        fps=args.fps,
        metadata={"title": args.title, "artist": "klar2016_sand"},
        bitrate=3200,
    )
    minimum_frame_count = max(1, int(math.ceil(args.fps * args.min_duration_seconds)))
    frame_repeat = max(1, int(math.ceil(minimum_frame_count / len(frame_paths))))
    width = 620
    height = 620

    with writer.saving(figure, str(output_path), dpi=args.dpi):
        for frame_path in frame_paths:
            data = read_ply_points(frame_path)
            positions = data[:, :3] if data.size > 0 else np.zeros((0, 3), dtype=float)
            axes.cla()
            axes.fill(
                setup["floor_uv"][:, 0],
                setup["floor_uv"][:, 1],
                color="#d6d1c9",
                zorder=0,
            )
            axes.plot(
                np.r_[setup["floor_uv"][:, 0], setup["floor_uv"][0, 0]],
                np.r_[setup["floor_uv"][:, 1], setup["floor_uv"][0, 1]],
                color="#6f6a63",
                linewidth=1.0,
                alpha=0.55,
                zorder=1,
            )
            if len(positions) > 0:
                image = rasterize_pile_lab_particles(
                    positions,
                    setup,
                    width,
                    height,
                    sand_cmap,
                )
                axes.imshow(
                    image,
                    extent=[setup["u_min"], setup["u_max"], setup["v_min"], setup["v_max"]],
                    origin="lower",
                    interpolation="bilinear",
                    zorder=2,
                )

            axes.set_xlim(setup["u_min"], setup["u_max"])
            axes.set_ylim(setup["v_min"], setup["v_max"])
            axes.set_aspect("equal", adjustable="box")
            axes.set_axis_off()
            for _ in range(frame_repeat):
                writer.grab_frame()

    plt.close(figure)


def apply_camera_preset(
    args: argparse.Namespace,
    frame_paths: list[Path],
    scene_bounds: tuple[np.ndarray, np.ndarray] | None,
    scene_colliders: list[dict],
    cylinder_shells: list[dict],
    bbox_min: np.ndarray,
    bbox_max: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, bool]:
    hide_domain_box = args.hide_domain_box
    if args.camera_preset != "pile_lab":
        return bbox_min, bbox_max, hide_domain_box

    final_min, final_max = particle_bounds(frame_paths[-1])
    center = 0.5 * (final_min + final_max)
    lateral_extent = max(final_max[0] - final_min[0], final_max[2] - final_min[2], 0.03)
    target_extent = max(lateral_extent * 1.05, 0.024)
    bbox_min = np.array(
        [center[0] - 0.5 * target_extent, 0.0, center[2] - 0.5 * target_extent],
        dtype=float,
    )
    bbox_max = np.array(
        [center[0] + 0.5 * target_extent, final_max[1] + 0.006, center[2] + 0.5 * target_extent],
        dtype=float,
    )
    for shell in cylinder_shells:
        center_y = float(shell["center"][1])
        lower_spout = center_y + float(shell["y_min"])
        bbox_max[1] = max(bbox_max[1], min(lower_spout + 0.010, center_y + float(shell["y_max"])))

    if scene_bounds is not None:
        bbox_min = np.maximum(bbox_min, scene_bounds[0])
        bbox_max = np.minimum(bbox_max, scene_bounds[1])

    args.elev = 12.0
    args.azim = -67.0
    args.padding_ratio = 0.04
    hide_domain_box = True
    return bbox_min, bbox_max, hide_domain_box


def main() -> None:
    args = parse_args()
    input_dir = Path(args.input_dir)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    frame_paths = list_frames(input_dir)
    scene_bounds = None
    scene_colliders: list[dict] = []
    scene_cylinder_shells: list[dict] = []
    if args.scene:
        scene_min, scene_max, scene_colliders, scene_cylinder_shells = load_scene_data(Path(args.scene))
        scene_bounds = (scene_min, scene_max)

    particle_min, particle_max = infer_bounds(frame_paths)
    if args.fit_mode == "scene":
        if scene_bounds is not None:
            bbox_min, bbox_max = scene_bounds
        else:
            bbox_min, bbox_max = particle_min, particle_max
    elif args.fit_mode == "particle":
        bbox_min, bbox_max = fit_particle_bounds(
            particle_min,
            particle_max,
            args.padding_ratio,
        )
    else:
        if scene_bounds is not None:
            bbox_min, bbox_max = hybrid_bounds(
                scene_bounds[0],
                scene_bounds[1],
                particle_min,
                particle_max,
                args.padding_ratio,
                args.hybrid_min_fraction,
            )
        else:
            bbox_min, bbox_max = fit_particle_bounds(
                particle_min,
                particle_max,
                args.padding_ratio,
            )

    bbox_min, bbox_max, hide_domain_box = apply_camera_preset(
        args,
        frame_paths,
        scene_bounds,
        scene_colliders,
        scene_cylinder_shells,
        bbox_min,
        bbox_max,
    )

    if args.camera_preset == "pile_lab":
        render_pile_lab_video(
            args,
            frame_paths,
            scene_cylinder_shells,
            output_path,
        )
        return

    if not FFMpegWriter.isAvailable():
        raise RuntimeError("matplotlib FFmpeg writer is not available on this system.")

    if args.camera_preset == "pile_lab":
        figure = plt.figure(figsize=(6.6, 6.2))
        figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=0.93)
    else:
        figure = plt.figure(figsize=(9.6, 5.4))
        figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=0.94)
    axes = figure.add_subplot(111, projection="3d")
    if args.camera_preset == "pile_lab":
        figure.patch.set_facecolor("#f4f1eb")
        axes.set_facecolor("#fbf8f2")
    else:
        figure.patch.set_facecolor("#f2ede3")
        axes.set_facecolor("#f7f1e7")
    figure.suptitle(args.title, y=0.97, fontsize=14 if args.camera_preset != "pile_lab" else 12)

    plane_patches = []
    for collider in scene_colliders:
        patch = compute_plane_patch(collider)
        if patch is None:
            continue
        plane_patches.append((patch, np.array(collider["normal"], dtype=float)))

    writer = FFMpegWriter(
        fps=args.fps,
        metadata={"title": args.title, "artist": "klar2016_sand"},
        bitrate=3200,
    )
    minimum_frame_count = max(1, int(math.ceil(args.fps * args.min_duration_seconds)))
    frame_repeat = max(1, int(math.ceil(minimum_frame_count / len(frame_paths))))

    with writer.saving(figure, str(output_path), dpi=args.dpi):
        for frame_index, frame_path in enumerate(frame_paths):
            axes.cla()
            floor_polygons: list[np.ndarray] = []
            outline_polygons: list[np.ndarray] = []
            translucent_polygons: list[np.ndarray] = []
            for polygon, normal in plane_patches:
                normal /= np.linalg.norm(normal)
                if args.camera_preset == "pile_lab" and abs(float(normal[1])) > 0.9:
                    floor_polygons.append(polygon)
                    continue
                if args.camera_preset == "pile_lab":
                    outline_polygons.append(polygon)
                else:
                    translucent_polygons.append(polygon)

            for polygon in floor_polygons:
                floor_patch = Poly3DCollection(
                    [[(float(p[0]), float(p[2]), float(p[1] - 1.0e-3)) for p in polygon]],
                    facecolor="#2c2b2c",
                    edgecolor="#1f1f20",
                    linewidth=0.8,
                    alpha=0.12,
                )
                axes.add_collection3d(floor_patch)

            data = read_ply_points(frame_path)
            if data.size > 0:
                positions = data[:, :3]
                colors = positions[:, 1]
                axes.scatter(
                    positions[:, 0],
                    positions[:, 2],
                    positions[:, 1],
                    c=colors,
                    cmap="copper",
                    s=args.point_size,
                    alpha=0.9,
                    linewidths=0.0,
                    depthshade=False,
                )

            for polygon in outline_polygons:
                draw_plane_outline(
                    axes,
                    polygon,
                    color="#8d7156",
                    linewidth=1.2,
                    alpha=0.85,
                )
            for polygon in translucent_polygons:
                draw_plane_patch(axes, polygon)
            if args.camera_preset == "pile_lab":
                for shell in scene_cylinder_shells:
                    draw_cylinder_shell(
                        axes,
                        shell,
                        color="#8d7156",
                        linewidth=1.25,
                        alpha=0.9,
                    )
            if not hide_domain_box:
                draw_box(
                    axes,
                    *(scene_bounds if scene_bounds is not None else (bbox_min, bbox_max)),
                )
            axes.set_xlim(float(bbox_min[0]), float(bbox_max[0]))
            axes.set_ylim(float(bbox_min[2]), float(bbox_max[2]))
            axes.set_zlim(float(bbox_min[1]), float(bbox_max[1]))
            axes.set_box_aspect(
                (
                    float(bbox_max[0] - bbox_min[0]),
                    float(bbox_max[2] - bbox_min[2]),
                    float(bbox_max[1] - bbox_min[1]),
                )
            )
            axes.view_init(elev=args.elev, azim=args.azim)
            if args.show_axes:
                axes.set_xlabel("x")
                axes.set_ylabel("z")
                axes.set_zlabel("y")
                axes.grid(True, alpha=0.25)
            else:
                axes.set_axis_off()
                axes.grid(False)
            for _ in range(frame_repeat):
                writer.grab_frame()

    plt.close(figure)


if __name__ == "__main__":
    main()
