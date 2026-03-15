#include "klar2016/core/simulation_config.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

namespace klar2016 {

namespace {

Vec3d parse_vec3(const nlohmann::json &node, const char *key) {
    const auto &value = node.at(key);
    if (!value.is_array() || value.size() != 3) {
        throw std::runtime_error(fmt::format("Field '{}' must be an array of size 3", key));
    }
    return Vec3d{value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
}

PlaneColliderConfig parse_plane_collider(const nlohmann::json &node) {
    PlaneColliderConfig collider;
    collider.point = parse_vec3(node, "point");
    collider.normal = parse_vec3(node, "normal");
    collider.friction_coefficient = node.value("friction_coefficient", 0.0);
    if (node.contains("bounds_min") || node.contains("bounds_max")) {
        if (!node.contains("bounds_min") || !node.contains("bounds_max")) {
            throw std::runtime_error(
                "Plane collider bounds must provide both 'bounds_min' and 'bounds_max'");
        }
        collider.bounded = true;
        collider.bounds_min = parse_vec3(node, "bounds_min");
        collider.bounds_max = parse_vec3(node, "bounds_max");
    }
    return collider;
}

HourglassShellConfig parse_hourglass_shell(const nlohmann::json &node) {
    HourglassShellConfig shell;
    shell.enabled = node.value("enabled", true);
    shell.waist_center = parse_vec3(node, "waist_center");
    shell.top_y = node.at("top_y").get<double>();
    shell.bottom_y = node.at("bottom_y").get<double>();
    shell.neck_half_width = node.at("neck_half_width").get<double>();
    shell.top_half_width = node.at("top_half_width").get<double>();
    shell.bottom_half_width = node.at("bottom_half_width").get<double>();
    shell.upper_friction_coefficient = node.value(
        "upper_friction_coefficient",
        shell.upper_friction_coefficient);
    shell.lower_friction_coefficient = node.value(
        "lower_friction_coefficient",
        shell.lower_friction_coefficient);
    shell.add_bottom_cap = node.value("add_bottom_cap", shell.add_bottom_cap);
    shell.bottom_cap_friction_coefficient = node.value(
        "bottom_cap_friction_coefficient",
        shell.bottom_cap_friction_coefficient);
    return shell;
}

CylinderShellConfig parse_cylinder_shell(const nlohmann::json &node) {
    CylinderShellConfig shell;
    shell.enabled = node.value("enabled", true);
    shell.center = parse_vec3(node, "center");
    shell.radius = node.at("radius").get<double>();
    shell.y_min = node.at("y_min").get<double>();
    shell.y_max = node.at("y_max").get<double>();
    shell.friction_coefficient = node.value(
        "friction_coefficient",
        shell.friction_coefficient);
    shell.segments = node.value("segments", shell.segments);
    return shell;
}

}  // namespace

SimulationConfig load_config(const std::filesystem::path &path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error(fmt::format("Failed to open scene config '{}'", path.string()));
    }

    nlohmann::json root;
    input >> root;

    SimulationConfig config;

    const auto &scene = root.at("scene");
    config.scene.name = scene.at("name").get<std::string>();
    config.scene.domain_min = parse_vec3(scene, "domain_min");
    config.scene.domain_max = parse_vec3(scene, "domain_max");
    config.scene.gravity = parse_vec3(scene, "gravity");
    config.scene.domain_boundary_friction_coefficient =
        scene.value(
            "domain_boundary_friction_coefficient",
            config.scene.domain_boundary_friction_coefficient);
    config.scene.seed_box_min = parse_vec3(scene, "seed_box_min");
    config.scene.seed_box_max = parse_vec3(scene, "seed_box_max");
    config.scene.seed_from_particles_per_cell = scene.value(
        "seed_from_particles_per_cell",
        config.scene.seed_from_particles_per_cell);
    config.scene.seed_jitter = scene.value("seed_jitter", config.scene.seed_jitter);
    config.scene.seed_particles = scene.at("seed_particles").get<int>();
    config.scene.duration = scene.at("duration").get<double>();
    config.scene.frame_stride = scene.at("frame_stride").get<int>();
    config.scene.preview_steps = scene.at("preview_steps").get<int>();
    if (scene.contains("hourglass_shell")) {
        if (!scene.at("hourglass_shell").is_object()) {
            throw std::runtime_error("Field 'scene.hourglass_shell' must be an object");
        }
        config.scene.hourglass_shell = parse_hourglass_shell(scene.at("hourglass_shell"));
    }
    if (scene.contains("cylinder_shells")) {
        const auto &shells = scene.at("cylinder_shells");
        if (!shells.is_array()) {
            throw std::runtime_error("Field 'scene.cylinder_shells' must be an array");
        }
        config.scene.cylinder_shells.reserve(shells.size());
        for (const auto &shell : shells) {
            config.scene.cylinder_shells.push_back(parse_cylinder_shell(shell));
        }
    }
    if (scene.contains("colliders")) {
        const auto &colliders = scene.at("colliders");
        if (!colliders.is_array()) {
            throw std::runtime_error("Field 'scene.colliders' must be an array");
        }

        config.scene.colliders.reserve(colliders.size());
        for (const auto &collider : colliders) {
            config.scene.colliders.push_back(parse_plane_collider(collider));
        }
    }

    const auto &material = root.at("material");
    config.material.density = material.at("density").get<double>();
    config.material.youngs_modulus = material.at("youngs_modulus").get<double>();
    config.material.poisson_ratio = material.at("poisson_ratio").get<double>();
    config.material.friction_angle_degrees = material.at("friction_angle_degrees").get<double>();
    config.material.hardening_h0 = material.at("hardening_h0").get<double>();
    config.material.hardening_h1 = material.at("hardening_h1").get<double>();
    config.material.hardening_h2 = material.at("hardening_h2").get<double>();
    config.material.hardening_h3 = material.at("hardening_h3").get<double>();

    const auto &solver = root.at("solver");
    config.solver.dx = solver.at("dx").get<double>();
    config.solver.dt_max = solver.at("dt_max").get<double>();
    config.solver.cfl = solver.at("cfl").get<double>();
    config.solver.particles_per_cell = solver.at("particles_per_cell").get<int>();
    config.solver.explicit_only = solver.at("explicit_only").get<bool>();

    if (root.contains("emitter")) {
        const auto &emitter = root.at("emitter");
        config.emitter.enabled = emitter.at("enabled").get<bool>();
        config.emitter.shape = emitter.value("shape", config.emitter.shape);
        config.emitter.sampling_mode = emitter.value(
            "sampling_mode",
            config.emitter.sampling_mode);
        config.emitter.box_min = parse_vec3(emitter, "box_min");
        config.emitter.box_max = parse_vec3(emitter, "box_max");
        config.emitter.radius = emitter.value("radius", config.emitter.radius);
        config.emitter.jitter = emitter.value("jitter", config.emitter.jitter);
        config.emitter.initial_velocity = parse_vec3(emitter, "initial_velocity");
        config.emitter.particles_per_step = emitter.at("particles_per_step").get<int>();
        config.emitter.start_step = emitter.at("start_step").get<int>();
        config.emitter.stop_step = emitter.at("stop_step").get<int>();
        config.emitter.max_particles = emitter.at("max_particles").get<int>();
    }

    if (root.contains("export")) {
        const auto &export_settings = root.at("export");
        config.export_settings.export_ply = export_settings.at("export_ply").get<bool>();
        config.export_settings.output_directory =
            export_settings.at("output_directory").get<std::string>();
        config.export_settings.interval_steps =
            export_settings.at("interval_steps").get<int>();
        config.export_settings.export_initial_frame =
            export_settings.at("export_initial_frame").get<bool>();
    }

    return config;
}

std::string describe(const SimulationConfig &config) {
    std::ostringstream out;
    out << "scene=" << config.scene.name
        << ", dx=" << config.solver.dx
        << ", dt_max=" << config.solver.dt_max
        << ", ppc=" << config.solver.particles_per_cell
        << ", seed_particles=" << config.scene.seed_particles
        << ", seed_mode="
        << (config.scene.seed_from_particles_per_cell ? "ppc" : "count")
        << ", colliders=" << config.scene.colliders.size()
        << ", hourglass_shell=" << (config.scene.hourglass_shell.enabled ? "true" : "false")
        << ", cylinder_shells=" << config.scene.cylinder_shells.size()
        << ", boundary_mu=" << config.scene.domain_boundary_friction_coefficient
        << ", explicit_only=" << (config.solver.explicit_only ? "true" : "false")
        << ", friction_angle=" << config.material.friction_angle_degrees
        << ", emitter=" << (config.emitter.enabled ? "true" : "false")
        << ", emitter_shape=" << config.emitter.shape
        << ", emitter_sampling=" << config.emitter.sampling_mode
        << ", export_ply=" << (config.export_settings.export_ply ? "true" : "false");
    return out.str();
}

std::string describe(std::string_view label, const Vec3d &value) {
    return fmt::format("{}=({}, {}, {})", label, value.x, value.y, value.z);
}

}  // namespace klar2016
