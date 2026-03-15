#!/usr/bin/env python3

"""
Render klar2016 PLY particle frames with Blender.

Usage:
  blender -b -P tools/render_ply_sequence_blender.py -- \
    --input-dir outputs/pile_lab_cyl_phi50/frames \
    --scene scenes/pile_lab.json \
    --output outputs/pile_lab_cyl_phi50/pile_lab_blender.mp4
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import bpy
from mathutils import Vector


def parse_args() -> argparse.Namespace:
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []

    parser = argparse.ArgumentParser(description="Render PLY frame sequences with Blender.")
    parser.add_argument("--input-dir", required=True, help="Directory containing frame_*.ply files.")
    parser.add_argument("--scene", required=True, help="Scene JSON used to place floor and nozzle.")
    parser.add_argument("--output", required=True, help="Output MP4 path.")
    parser.add_argument(
        "--output-format",
        choices=("MP4", "PNG"),
        default="MP4",
        help="Render an MP4 animation or a single PNG frame.",
    )
    parser.add_argument("--fps", type=int, default=20, help="Animation frames per second.")
    parser.add_argument(
        "--engine",
        choices=("CYCLES", "BLENDER_EEVEE"),
        default="CYCLES",
        help="Blender render engine.",
    )
    parser.add_argument("--samples", type=int, default=64, help="Render samples.")
    parser.add_argument("--resolution", type=int, default=960, help="Square output resolution.")
    parser.add_argument(
        "--particle-radius",
        type=float,
        default=0.0,
        help="Rendered sphere radius in scene units. 0 means estimate from particle spacing.",
    )
    parser.add_argument(
        "--particle-radius-scale",
        type=float,
        default=0.34,
        help="Multiplier applied to estimated particle spacing when particle-radius=0.",
    )
    parser.add_argument(
        "--grain-scale-min",
        type=float,
        default=0.82,
        help="Minimum per-grain scale multiplier.",
    )
    parser.add_argument(
        "--grain-scale-max",
        type=float,
        default=0.96,
        help="Maximum per-grain scale multiplier.",
    )
    parser.add_argument(
        "--grain-bump-strength",
        type=float,
        default=0.10,
        help="Normal perturbation strength for sand grains.",
    )
    parser.add_argument(
        "--grain-bump-scale",
        type=float,
        default=220.0,
        help="Noise scale used to break up perfectly smooth grains.",
    )
    parser.add_argument(
        "--camera-preset",
        choices=("pile_lab",),
        default="pile_lab",
        help="Render framing preset.",
    )
    parser.add_argument(
        "--render-mode",
        choices=("points", "surface"),
        default="surface",
        help="Render particles as instanced spheres or as a reconstructed surface.",
    )
    parser.add_argument(
        "--shading-mode",
        choices=("studio", "flat"),
        default="studio",
        help="Studio lighting for presentation or flat colors for geometry diagnosis.",
    )
    parser.add_argument(
        "--frame-repeat",
        type=int,
        default=1,
        help="Repeat each imported frame this many times in the output animation.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Optional limit on the number of imported frames. 0 means all frames.",
    )
    parser.add_argument(
        "--start-frame-index",
        type=int,
        default=0,
        help="Optional starting index within the sorted frame list.",
    )
    parser.add_argument(
        "--floor-size",
        type=float,
        default=0.070,
        help="Rendered floor plane size in scene units.",
    )
    parser.add_argument(
        "--still-frame",
        type=int,
        default=0,
        help="1-based frame number to render when output-format=PNG. 0 means the last frame.",
    )
    return parser.parse_args(argv)


def list_frames(input_dir: Path) -> list[Path]:
    frames = sorted(input_dir.glob("frame_*.ply"))
    if not frames:
        raise FileNotFoundError(f"No PLY frames found in '{input_dir}'.")
    return frames


def read_ascii_ply_positions(path: Path) -> list[tuple[float, float, float]]:
    with path.open("r", encoding="utf-8") as handle:
        vertex_count = None
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError(f"Unexpected EOF while parsing '{path}'.")
            stripped = line.strip()
            if stripped.startswith("element vertex "):
                vertex_count = int(stripped.split()[-1])
            if stripped == "end_header":
                break

        if vertex_count is None:
            raise RuntimeError(f"Missing vertex count in '{path}'.")

        positions: list[tuple[float, float, float]] = []
        for _ in range(vertex_count):
            line = handle.readline()
            if not line:
                raise RuntimeError(f"Unexpected EOF in vertex block of '{path}'.")
            fields = line.strip().split()
            if len(fields) < 3:
                continue
            positions.append((float(fields[0]), float(fields[1]), float(fields[2])))
        return positions


def load_scene_data(scene_path: Path) -> dict:
    root = json.loads(scene_path.read_text(encoding="utf-8"))
    return root


def last_nonempty_positions(frames: list[Path]) -> list[tuple[float, float, float]]:
    for frame_path in reversed(frames):
        positions = read_ascii_ply_positions(frame_path)
        if positions:
            return positions
    raise RuntimeError("No non-empty frame found in input sequence.")


def filter_nonempty_frames(frames: list[Path]) -> list[Path]:
    nonempty_frames: list[Path] = []
    for frame_path in frames:
        if read_ascii_ply_positions(frame_path):
            nonempty_frames.append(frame_path)
    if not nonempty_frames:
        raise RuntimeError("Selected frame window contains no particle data.")
    return nonempty_frames


def bounds_from_positions(positions: list[tuple[float, float, float]]) -> tuple[Vector, Vector]:
    min_corner = Vector((math.inf, math.inf, math.inf))
    max_corner = Vector((-math.inf, -math.inf, -math.inf))
    for x, y, z in positions:
        min_corner.x = min(min_corner.x, x)
        min_corner.y = min(min_corner.y, y)
        min_corner.z = min(min_corner.z, z)
        max_corner.x = max(max_corner.x, x)
        max_corner.y = max(max_corner.y, y)
        max_corner.z = max(max_corner.z, z)
    return min_corner, max_corner


def estimate_particle_spacing(
    positions: list[tuple[float, float, float]],
    sample_limit: int = 2048,
) -> float:
    if len(positions) < 2:
        return 1.0e-4

    bounds_min, bounds_max = bounds_from_positions(positions)
    extent = bounds_max - bounds_min
    bbox_volume = max(extent.x * extent.y * extent.z, 1.0e-12)
    nominal_spacing = (bbox_volume / max(len(positions), 1)) ** (1.0 / 3.0)
    cell_size = max(nominal_spacing * 1.75, 1.0e-6)

    buckets: dict[tuple[int, int, int], list[tuple[int, float, float, float]]] = {}
    for index, (x, y, z) in enumerate(positions):
        key = (
            math.floor(x / cell_size),
            math.floor(y / cell_size),
            math.floor(z / cell_size),
        )
        buckets.setdefault(key, []).append((index, x, y, z))

    stride = max(len(positions) // sample_limit, 1)
    distances: list[float] = []
    for index in range(0, len(positions), stride):
        px, py, pz = positions[index]
        base_key = (
            math.floor(px / cell_size),
            math.floor(py / cell_size),
            math.floor(pz / cell_size),
        )
        best_distance_sq = math.inf
        found_neighbor = False
        for search_radius in (1, 2):
            for dx in range(-search_radius, search_radius + 1):
                for dy in range(-search_radius, search_radius + 1):
                    for dz in range(-search_radius, search_radius + 1):
                        cell_key = (
                            base_key[0] + dx,
                            base_key[1] + dy,
                            base_key[2] + dz,
                        )
                        for other_index, ox, oy, oz in buckets.get(cell_key, []):
                            if other_index == index:
                                continue
                            dxp = px - ox
                            dyp = py - oy
                            dzp = pz - oz
                            distance_sq = dxp * dxp + dyp * dyp + dzp * dzp
                            if distance_sq < best_distance_sq:
                                best_distance_sq = distance_sq
                                found_neighbor = True
            if found_neighbor:
                break
        if found_neighbor:
            distances.append(math.sqrt(best_distance_sq))
        if len(distances) >= sample_limit:
            break

    if not distances:
        return max(nominal_spacing, 1.0e-5)

    distances.sort()
    return max(distances[len(distances) // 2], 1.0e-5)


def sim_to_blender(position: Vector) -> Vector:
    return Vector((position.x, -position.z, position.y))


def clear_scene() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)


def ensure_ply_importer() -> None:
    if hasattr(bpy.ops.wm, "ply_import") or hasattr(bpy.ops.import_mesh, "ply"):
        return
    raise RuntimeError("No Blender PLY import operator is available.")


def create_principled_material(
    name: str,
    base_color: tuple[float, float, float, float],
    roughness: float,
    specular: float,
    emission_strength: float = 0.0,
) -> bpy.types.Material:
    material = bpy.data.materials.new(name=name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    nodes.clear()

    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (240.0, 0.0)

    principled = nodes.new("ShaderNodeBsdfPrincipled")
    principled.location = (-40.0, 0.0)
    principled.inputs["Base Color"].default_value = base_color
    principled.inputs["Roughness"].default_value = min(max(roughness, 0.0), 1.0)
    if "Specular IOR Level" in principled.inputs:
        principled.inputs["Specular IOR Level"].default_value = min(max(specular, 0.0), 1.0)
    elif "Specular" in principled.inputs:
        principled.inputs["Specular"].default_value = min(max(specular, 0.0), 1.0)
    if emission_strength > 1.0e-4:
        if "Emission Color" in principled.inputs:
            principled.inputs["Emission Color"].default_value = base_color
        elif "Emission" in principled.inputs:
            principled.inputs["Emission"].default_value = base_color
        if "Emission Strength" in principled.inputs:
            principled.inputs["Emission Strength"].default_value = emission_strength

    links.new(principled.outputs["BSDF"], output.inputs["Surface"])

    return material


def create_flat_material(
    name: str,
    base_color: tuple[float, float, float, float],
    strength: float = 1.0,
) -> bpy.types.Material:
    material = bpy.data.materials.new(name=name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    nodes.clear()

    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (180.0, 0.0)

    emission = nodes.new("ShaderNodeEmission")
    emission.location = (-40.0, 0.0)
    emission.inputs["Color"].default_value = base_color
    emission.inputs["Strength"].default_value = strength

    links.new(emission.outputs["Emission"], output.inputs["Surface"])
    return material


def create_grain_material(
    name: str,
    bump_strength: float,
    bump_scale: float,
) -> bpy.types.Material:
    material = bpy.data.materials.new(name=name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    nodes.clear()

    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (420.0, 0.0)

    principled = nodes.new("ShaderNodeBsdfPrincipled")
    principled.location = (180.0, 0.0)
    principled.inputs["Roughness"].default_value = 0.96
    if "Specular IOR Level" in principled.inputs:
        principled.inputs["Specular IOR Level"].default_value = 0.03
    elif "Specular" in principled.inputs:
        principled.inputs["Specular"].default_value = 0.03

    attribute = nodes.new("ShaderNodeAttribute")
    attribute.location = (-520.0, 0.0)
    attribute.attribute_name = "grain_random"

    color_ramp = nodes.new("ShaderNodeValToRGB")
    color_ramp.location = (-220.0, 0.0)
    color_ramp.color_ramp.interpolation = "CONSTANT"

    yellow = (225.0 / 255.0, 169.0 / 255.0, 95.0 / 255.0, 1.0)
    brown = (107.0 / 255.0, 84.0 / 255.0, 30.0 / 255.0, 1.0)
    white = (1.0, 1.0, 1.0, 1.0)

    elements = color_ramp.color_ramp.elements
    elements[0].position = 0.0
    elements[0].color = yellow
    elements[1].position = 0.85
    elements[1].color = brown
    elements.new(0.95).color = white

    texture_coordinate = nodes.new("ShaderNodeTexCoord")
    texture_coordinate.location = (-760.0, -220.0)

    noise = nodes.new("ShaderNodeTexNoise")
    noise.location = (-520.0, -220.0)
    noise.inputs["Scale"].default_value = bump_scale
    noise.inputs["Detail"].default_value = 3.2
    noise.inputs["Roughness"].default_value = 0.45

    bump = nodes.new("ShaderNodeBump")
    bump.location = (-220.0, -220.0)
    bump.inputs["Strength"].default_value = bump_strength
    bump.inputs["Distance"].default_value = 0.02

    links.new(attribute.outputs["Fac"], color_ramp.inputs["Fac"])
    links.new(texture_coordinate.outputs["Object"], noise.inputs["Vector"])
    links.new(noise.outputs["Fac"], bump.inputs["Height"])
    links.new(color_ramp.outputs["Color"], principled.inputs["Base Color"])
    links.new(bump.outputs["Normal"], principled.inputs["Normal"])
    links.new(principled.outputs["BSDF"], output.inputs["Surface"])
    return material


def set_object_material(obj: bpy.types.Object, material: bpy.types.Material) -> None:
    if not obj.data.materials:
        obj.data.materials.append(material)
    else:
        obj.data.materials[0] = material


def point_camera_at(camera_obj: bpy.types.Object, target: Vector) -> None:
    direction = target - camera_obj.location
    camera_obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def setup_world(shading_mode: str) -> None:
    world = bpy.data.worlds.new("World")
    bpy.context.scene.world = world
    world.use_nodes = True
    background = world.node_tree.nodes["Background"]
    if shading_mode == "studio":
        background.inputs["Color"].default_value = (0.082, 0.084, 0.090, 1.0)
        background.inputs["Strength"].default_value = 0.028
    else:
        background.inputs["Color"].default_value = (0.968, 0.954, 0.930, 1.0)
        background.inputs["Strength"].default_value = 0.10


def setup_renderer(args: argparse.Namespace) -> tuple[str, Path | None]:
    scene = bpy.context.scene
    scene.render.engine = args.engine
    scene.render.resolution_x = args.resolution
    scene.render.resolution_y = args.resolution
    scene.render.resolution_percentage = 100
    output_path = Path(args.output).resolve()
    sequence_directory: Path | None = None
    render_mode = "still_png"
    if args.output_format == "MP4":
        try:
            scene.render.image_settings.file_format = "FFMPEG"
            scene.render.ffmpeg.format = "MPEG4"
            scene.render.ffmpeg.codec = "H264"
            scene.render.ffmpeg.constant_rate_factor = "HIGH"
            scene.render.ffmpeg.ffmpeg_preset = "GOOD"
            scene.render.filepath = str(output_path)
            render_mode = "direct_mp4"
        except Exception:
            scene.render.image_settings.file_format = "PNG"
            sequence_directory = output_path.parent / f"{output_path.stem}_frames"
            sequence_directory.mkdir(parents=True, exist_ok=True)
            scene.render.filepath = str((sequence_directory / "frame_").resolve())
            render_mode = "png_sequence"
    else:
        scene.render.image_settings.file_format = "PNG"
        scene.render.filepath = str(output_path)
        render_mode = "still_png"
    scene.render.fps = args.fps
    scene.render.film_transparent = False
    scene.render.use_motion_blur = args.shading_mode == "studio" and args.output_format == "MP4"
    if scene.render.use_motion_blur and hasattr(scene.render, "motion_blur_shutter"):
        scene.render.motion_blur_shutter = 0.22

    if hasattr(scene, "display_settings"):
        try:
            scene.display_settings.display_device = "sRGB"
        except Exception:
            pass
    if hasattr(scene, "view_settings"):
        for transform_name in ("AgX", "Filmic", "Standard"):
            try:
                scene.view_settings.view_transform = transform_name
                break
            except Exception:
                continue
        scene.view_settings.exposure = -1.20 if args.shading_mode == "studio" else -0.35
        scene.view_settings.gamma = 1.0

    if args.engine == "CYCLES":
        scene.cycles.samples = args.samples
        scene.cycles.use_adaptive_sampling = True
        scene.cycles.max_bounces = 4
        scene.cycles.preview_samples = min(args.samples, 16)
    else:
        scene.eevee.taa_render_samples = args.samples
        if hasattr(scene.eevee, "use_bloom"):
            scene.eevee.use_bloom = False
        if hasattr(scene.eevee, "use_gtao"):
            scene.eevee.use_gtao = True
        if hasattr(scene.eevee, "gtao_factor"):
            scene.eevee.gtao_factor = 1.25
        if hasattr(scene.eevee, "gtao_quality"):
            scene.eevee.gtao_quality = 0.3

    return render_mode, sequence_directory


def combine_png_sequence_into_mp4(
    sequence_directory: Path,
    output_path: Path,
    fps: int,
) -> None:
    pattern = sequence_directory / "frame_%04d.png"
    command = [
        "ffmpeg",
        "-y",
        "-framerate",
        str(fps),
        "-i",
        str(pattern),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-crf",
        "18",
        str(output_path),
    ]
    try:
        subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError as exc:
        raise RuntimeError("ffmpeg is required to assemble Blender PNG frames into MP4") from exc
    except subprocess.CalledProcessError as exc:
        stderr = exc.stderr.decode("utf-8", errors="replace")
        raise RuntimeError(f"ffmpeg failed while assembling MP4:\n{stderr}") from exc


def setup_lights(center: Vector, radius: float, pile_height: float) -> None:
    center_bl = sim_to_blender(center)
    sun_data = bpy.data.lights.new(name="SunLight", type="SUN")
    sun_data.energy = 0.014
    sun = bpy.data.objects.new("SunLight", sun_data)
    sun.rotation_euler = (math.radians(42.0), 0.0, math.radians(-28.0))
    bpy.context.scene.collection.objects.link(sun)

    key_data = bpy.data.lights.new(name="KeyLight", type="AREA")
    key_data.energy = 0.75
    key_data.shape = "RECTANGLE"
    key_data.size = max(radius * 5.0, 0.07)
    key_data.size_y = max(radius * 3.8, 0.05)
    key = bpy.data.objects.new("KeyLight", key_data)
    key.location = center_bl + Vector((-max(radius * 3.0, 0.040), -max(radius * 3.4, 0.055), max(pile_height * 3.0, 0.055)))
    bpy.context.scene.collection.objects.link(key)
    point_camera_at(key, center_bl + Vector((0.0, 0.0, pile_height * 0.35)))

    fill_data = bpy.data.lights.new(name="FillLight", type="AREA")
    fill_data.energy = 0.18
    fill_data.shape = "RECTANGLE"
    fill_data.size = max(radius * 6.0, 0.08)
    fill_data.size_y = max(radius * 4.0, 0.06)
    fill = bpy.data.objects.new("FillLight", fill_data)
    fill.location = center_bl + Vector((max(radius * 2.8, 0.032), -max(radius * 2.0, 0.030), max(pile_height * 2.2, 0.032)))
    bpy.context.scene.collection.objects.link(fill)
    point_camera_at(fill, center_bl + Vector((0.0, 0.0, pile_height * 0.28)))


def setup_camera(center: Vector, radius: float, pile_height: float) -> bpy.types.Object:
    center_bl = sim_to_blender(center)
    camera_data = bpy.data.cameras.new("Camera")
    camera = bpy.data.objects.new("Camera", camera_data)
    bpy.context.scene.collection.objects.link(camera)

    camera.location = center_bl + Vector(
        (-max(radius * 4.8, 0.055), -max(radius * 6.4, 0.075), max(pile_height * 3.4, 0.040))
    )
    camera_data.lens = 80.0
    camera_data.sensor_width = 36.0
    camera_data.clip_start = 0.001
    camera_data.clip_end = 100.0
    focus_target = center_bl + Vector((0.0, 0.0, max(pile_height * 0.40, 0.0045)))
    camera_data.dof.use_dof = True
    if hasattr(camera_data.dof, "focus_distance"):
        camera_data.dof.focus_distance = (camera.location - focus_target).length
    if hasattr(camera_data.dof, "aperture_fstop"):
        camera_data.dof.aperture_fstop = 16.0
    if hasattr(camera_data.dof, "aperture_blades"):
        camera_data.dof.aperture_blades = 7
    point_camera_at(camera, focus_target)
    bpy.context.scene.camera = camera
    return camera


def create_floor(center: Vector, size: float, material: bpy.types.Material) -> bpy.types.Object:
    center_bl = sim_to_blender(center)
    bpy.ops.mesh.primitive_plane_add(size=size, location=(center_bl.x, center_bl.y, 0.0))
    floor = bpy.context.active_object
    floor.name = "Floor"
    set_object_material(floor, material)
    return floor


def create_cylinder_nozzle(
    shell: dict,
    material: bpy.types.Material,
) -> bpy.types.Object:
    center = Vector(shell["center"])
    center_bl = sim_to_blender(center)
    radius = float(shell["radius"])
    y_min = center.y + float(shell["y_min"])
    y_max = center.y + float(shell["y_max"])
    depth = y_max - y_min

    bpy.ops.mesh.primitive_cylinder_add(
        vertices=max(int(shell.get("segments", 24)), 16),
        radius=radius,
        depth=depth,
        location=(center_bl.x, center_bl.y, y_min + 0.5 * depth),
    )
    nozzle = bpy.context.active_object
    nozzle.name = "Nozzle"
    set_object_material(nozzle, material)
    return nozzle


def create_particle_template(
    radius: float,
    material: bpy.types.Material,
) -> bpy.types.Object:
    bpy.ops.mesh.primitive_uv_sphere_add(
        segments=18,
        ring_count=10,
        radius=radius,
        location=(0.0, 0.0, 0.0),
    )
    template = bpy.context.active_object
    template.name = "ParticleTemplate"
    bpy.ops.object.shade_smooth()
    template.hide_render = True
    template.hide_viewport = True
    set_object_material(template, material)
    return template


def build_particle_node_group(
    template_object: bpy.types.Object,
    material: bpy.types.Material,
    grain_scale_min: float,
    grain_scale_max: float,
) -> bpy.types.GeometryNodeTree:
    node_group = bpy.data.node_groups.new("ParticleRenderNodes", "GeometryNodeTree")
    if hasattr(node_group, "interface"):
        node_group.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
        node_group.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    else:
        node_group.inputs.new("NodeSocketGeometry", "Geometry")
        node_group.outputs.new("NodeSocketGeometry", "Geometry")

    nodes = node_group.nodes
    links = node_group.links
    nodes.clear()

    group_input = nodes.new("NodeGroupInput")
    group_input.location = (-520.0, 0.0)
    group_output = nodes.new("NodeGroupOutput")
    group_output.location = (460.0, 0.0)

    mesh_to_points = nodes.new("GeometryNodeMeshToPoints")
    mesh_to_points.location = (-300.0, 0.0)
    mesh_to_points.mode = "VERTICES"
    mesh_to_points.inputs["Radius"].default_value = 1.0e-5

    object_info = nodes.new("GeometryNodeObjectInfo")
    object_info.location = (-300.0, -200.0)
    object_info.inputs["Object"].default_value = template_object
    object_info.transform_space = "RELATIVE"

    instance_on_points = nodes.new("GeometryNodeInstanceOnPoints")
    instance_on_points.location = (-60.0, 0.0)

    random_value = nodes.new("FunctionNodeRandomValue")
    random_value.location = (-300.0, 180.0)
    random_value.data_type = "FLOAT"
    random_value.inputs["Min"].default_value = 0.0
    random_value.inputs["Max"].default_value = 1.0

    random_scale = nodes.new("FunctionNodeRandomValue")
    random_scale.location = (-300.0, -20.0)
    random_scale.data_type = "FLOAT"
    random_scale.inputs["Min"].default_value = grain_scale_min
    random_scale.inputs["Max"].default_value = grain_scale_max

    combine_scale = nodes.new("ShaderNodeCombineXYZ")
    combine_scale.location = (-150.0, -20.0)

    store_attribute = nodes.new("GeometryNodeStoreNamedAttribute")
    store_attribute.location = (-60.0, 180.0)
    store_attribute.data_type = "FLOAT"
    store_attribute.domain = "POINT"
    store_attribute.inputs["Name"].default_value = "grain_random"

    realize_instances = nodes.new("GeometryNodeRealizeInstances")
    realize_instances.location = (150.0, 0.0)

    set_shade_smooth = nodes.new("GeometryNodeSetShadeSmooth")
    set_shade_smooth.location = (300.0, 0.0)
    set_shade_smooth.inputs["Shade Smooth"].default_value = True

    set_material = nodes.new("GeometryNodeSetMaterial")
    set_material.location = (460.0, 0.0)
    set_material.inputs["Material"].default_value = material

    links.new(group_input.outputs["Geometry"], mesh_to_points.inputs["Mesh"])
    links.new(mesh_to_points.outputs["Points"], store_attribute.inputs["Geometry"])
    links.new(random_value.outputs["Value"], store_attribute.inputs["Value"])
    links.new(store_attribute.outputs["Geometry"], instance_on_points.inputs["Points"])
    links.new(object_info.outputs["Geometry"], instance_on_points.inputs["Instance"])
    links.new(random_scale.outputs["Value"], combine_scale.inputs["X"])
    links.new(random_scale.outputs["Value"], combine_scale.inputs["Y"])
    links.new(random_scale.outputs["Value"], combine_scale.inputs["Z"])
    links.new(combine_scale.outputs["Vector"], instance_on_points.inputs["Scale"])
    links.new(instance_on_points.outputs["Instances"], realize_instances.inputs["Geometry"])
    links.new(realize_instances.outputs["Geometry"], set_shade_smooth.inputs["Geometry"])
    links.new(set_shade_smooth.outputs["Geometry"], set_material.inputs["Geometry"])
    links.new(set_material.outputs["Geometry"], group_output.inputs["Geometry"])
    return node_group


def build_surface_node_group(
    material: bpy.types.Material,
    particle_radius: float,
) -> bpy.types.GeometryNodeTree:
    node_group = bpy.data.node_groups.new("ParticleSurfaceNodes", "GeometryNodeTree")
    if hasattr(node_group, "interface"):
        node_group.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
        node_group.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    else:
        node_group.inputs.new("NodeSocketGeometry", "Geometry")
        node_group.outputs.new("NodeSocketGeometry", "Geometry")

    nodes = node_group.nodes
    links = node_group.links
    nodes.clear()

    group_input = nodes.new("NodeGroupInput")
    group_input.location = (-760.0, 0.0)
    group_output = nodes.new("NodeGroupOutput")
    group_output.location = (360.0, 0.0)

    mesh_to_points = nodes.new("GeometryNodeMeshToPoints")
    mesh_to_points.location = (-560.0, 0.0)
    mesh_to_points.mode = "VERTICES"
    mesh_to_points.inputs["Radius"].default_value = particle_radius * 1.4

    points_to_volume = nodes.new("GeometryNodePointsToVolume")
    points_to_volume.location = (-330.0, 0.0)
    points_to_volume.inputs["Density"].default_value = 1.0
    points_to_volume.inputs["Radius"].default_value = particle_radius * 2.8
    if "Voxel Size" in points_to_volume.inputs:
        points_to_volume.inputs["Voxel Size"].default_value = particle_radius * 0.85

    volume_to_mesh = nodes.new("GeometryNodeVolumeToMesh")
    volume_to_mesh.location = (-80.0, 0.0)
    volume_to_mesh.inputs["Threshold"].default_value = 0.08
    if "Adaptivity" in volume_to_mesh.inputs:
        volume_to_mesh.inputs["Adaptivity"].default_value = 0.0

    set_material = nodes.new("GeometryNodeSetMaterial")
    set_material.location = (140.0, 0.0)
    set_material.inputs["Material"].default_value = material

    links.new(group_input.outputs["Geometry"], mesh_to_points.inputs["Mesh"])
    links.new(mesh_to_points.outputs["Points"], points_to_volume.inputs["Points"])
    links.new(points_to_volume.outputs["Volume"], volume_to_mesh.inputs["Volume"])
    links.new(volume_to_mesh.outputs["Mesh"], set_material.inputs["Geometry"])
    links.new(set_material.outputs["Geometry"], group_output.inputs["Geometry"])
    return node_group


def set_visibility_keyframes(obj: bpy.types.Object, visible_frame_start: int, visible_frame_end: int) -> None:
    scene = bpy.context.scene
    first_frame = scene.frame_start
    last_frame = scene.frame_end

    def insert(frame: int, hidden: bool) -> None:
        frame = max(first_frame, min(last_frame, frame))
        obj.hide_viewport = hidden
        obj.hide_render = hidden
        obj.keyframe_insert(data_path="hide_viewport", frame=frame)
        obj.keyframe_insert(data_path="hide_render", frame=frame)

    insert(first_frame, True)
    if visible_frame_start > first_frame:
        insert(visible_frame_start - 1, True)
    insert(visible_frame_start, False)
    insert(visible_frame_end, False)
    if visible_frame_end < last_frame:
        insert(visible_frame_end + 1, True)


def import_ply_frame(filepath: Path) -> bpy.types.Object:
    resolved_path = str(filepath.resolve())
    last_error: Exception | None = None
    for operator in (
        lambda: bpy.ops.wm.ply_import(filepath=resolved_path),
        lambda: bpy.ops.import_mesh.ply(filepath=resolved_path),
    ):
        try:
            operator()
            imported = bpy.context.selected_objects[0]
            imported.rotation_euler = (math.radians(90.0), 0.0, 0.0)
            return imported
        except Exception as exc:
            last_error = exc

    raise RuntimeError("No Blender PLY import operator is available.") from last_error


def animate_frames(
    frames: list[Path],
    node_group: bpy.types.GeometryNodeTree,
    frame_repeat: int,
) -> None:
    scene = bpy.context.scene
    collection = bpy.data.collections.new("ParticleFrames")
    scene.collection.children.link(collection)

    for index, frame_path in enumerate(frames, start=1):
        imported = import_ply_frame(frame_path)
        collection.objects.link(imported)
        scene.collection.objects.unlink(imported)
        imported.name = frame_path.stem
        modifier = imported.modifiers.new(name="ParticleRender", type="NODES")
        modifier.node_group = node_group

        frame_start = 1 + (index - 1) * frame_repeat
        frame_end = frame_start + frame_repeat - 1
        set_visibility_keyframes(imported, frame_start, frame_end)


def main() -> None:
    args = parse_args()
    input_dir = Path(args.input_dir)
    scene_path = Path(args.scene)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    all_frames = list_frames(input_dir)
    start_frame_index = max(args.start_frame_index, 0)
    if start_frame_index >= len(all_frames):
        raise RuntimeError("start-frame-index exceeds available frame count.")

    frames = all_frames[start_frame_index:]
    if args.max_frames > 0:
        frames = frames[: args.max_frames]
    if not frames:
        raise RuntimeError("Selected frame window is empty.")
    frames = filter_nonempty_frames(frames)

    scene_root = load_scene_data(scene_path)
    shells = scene_root["scene"].get("cylinder_shells", [])
    reference_positions = last_nonempty_positions(all_frames)
    bounds_min, bounds_max = bounds_from_positions(reference_positions)
    center = 0.5 * (bounds_min + bounds_max)
    pile_height = max(bounds_max.y - bounds_min.y, 0.01)
    pile_radius = max(bounds_max.x - bounds_min.x, bounds_max.z - bounds_min.z) * 0.5
    pile_radius = max(pile_radius, 0.010)
    estimated_spacing = estimate_particle_spacing(reference_positions)
    effective_particle_radius = (
        args.particle_radius
        if args.particle_radius > 0.0
        else estimated_spacing * args.particle_radius_scale
    )
    print(
        "Particle render radius:",
        f"{effective_particle_radius:.7f}",
        "(estimated spacing:",
        f"{estimated_spacing:.7f})",
    )

    clear_scene()
    ensure_ply_importer()
    setup_world(args.shading_mode)
    render_output_mode, sequence_directory = setup_renderer(args)

    if args.shading_mode == "flat":
        floor_material = create_flat_material(
            name="FloorMaterial",
            base_color=(0.120, 0.125, 0.138, 1.0),
            strength=0.95,
        )
        nozzle_material = create_flat_material(
            name="NozzleMaterial",
            base_color=(0.470, 0.585, 0.540, 1.0),
            strength=0.90,
        )
        sand_material = create_flat_material(
            name="SandMaterial",
            base_color=(0.698, 0.540, 0.300, 1.0),
            strength=1.05,
        )
    else:
        floor_material = create_principled_material(
            name="FloorMaterial",
            base_color=(0.030, 0.031, 0.035, 1.0),
            roughness=1.0,
            specular=0.01,
            emission_strength=0.0,
        )
        nozzle_material = create_principled_material(
            name="NozzleMaterial",
            base_color=(0.435, 0.525, 0.485, 1.0),
            roughness=0.84,
            specular=0.04,
            emission_strength=0.0,
        )
        sand_material = (
            create_grain_material(
                "SandMaterial",
                bump_strength=args.grain_bump_strength,
                bump_scale=args.grain_bump_scale,
            )
            if args.render_mode == "points"
            else create_principled_material(
                name="SandMaterial",
                base_color=(0.520, 0.402, 0.228, 1.0),
                roughness=0.96,
                specular=0.01,
                emission_strength=0.0,
            )
        )

    if args.floor_size > 0.0:
        create_floor(center, args.floor_size, floor_material)
    if shells:
        create_cylinder_nozzle(shells[0], nozzle_material)
    setup_camera(center, pile_radius, pile_height)
    if args.shading_mode == "studio":
        setup_lights(center, pile_radius, pile_height)

    if args.render_mode == "points":
        particle_template = create_particle_template(effective_particle_radius, sand_material)
        particle_nodes = build_particle_node_group(
            particle_template,
            sand_material,
            args.grain_scale_min,
            args.grain_scale_max,
        )
    else:
        particle_nodes = build_surface_node_group(
            sand_material,
            effective_particle_radius,
        )

    bpy.context.scene.frame_start = 1
    bpy.context.scene.frame_end = len(frames) * max(args.frame_repeat, 1)
    animate_frames(frames, particle_nodes, max(args.frame_repeat, 1))
    if args.output_format == "PNG":
        frame_to_render = args.still_frame if args.still_frame > 0 else bpy.context.scene.frame_end
        frame_to_render = max(bpy.context.scene.frame_start, min(frame_to_render, bpy.context.scene.frame_end))
        bpy.context.scene.frame_set(frame_to_render)
        bpy.ops.render.render(write_still=True)
    else:
        bpy.ops.render.render(animation=True)
        if render_output_mode == "png_sequence":
            if sequence_directory is None:
                raise RuntimeError("PNG sequence directory is missing for MP4 assembly")
            combine_png_sequence_into_mp4(
                sequence_directory,
                Path(args.output).resolve(),
                args.fps,
            )


if __name__ == "__main__":
    main()
