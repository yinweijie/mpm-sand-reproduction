#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace klar2016 {

/// @brief Simple 3D vector used for JSON scene configuration values.
struct Vec3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/// @brief Material parameters for the Drucker-Prager sand constitutive model.
struct MaterialConfig {
    double density = 2200.0;
    double youngs_modulus = 3.537e5;
    double poisson_ratio = 0.3;
    double friction_angle_degrees = 30.0;
    double hardening_h0 = 35.0;
    double hardening_h1 = 9.0;
    double hardening_h2 = 0.3;
    double hardening_h3 = 10.0;
};

/// @brief Global solver controls such as resolution and timestep limits.
struct SolverConfig {
    double dx = 0.0025;
    double dt_max = 1.0e-4;
    double cfl = 1.0;
    int particles_per_cell = 8;
    bool explicit_only = true;
};

/// @brief Plane collider settings, optionally restricted to a bounded region.
struct PlaneColliderConfig {
    Vec3d point{0.0, 0.0, 0.0};
    Vec3d normal{0.0, 1.0, 0.0};
    double friction_coefficient = 0.0;
    bool bounded = false;
    Vec3d bounds_min{0.0, 0.0, 0.0};
    Vec3d bounds_max{0.0, 0.0, 0.0};
};

/// @brief Procedural hourglass shell settings used to synthesize boundary colliders.
struct HourglassShellConfig {
    bool enabled = false;
    Vec3d waist_center{0.5, 0.5, 0.5};
    double top_y = 0.8;
    double bottom_y = 0.1;
    double neck_half_width = 0.03;
    double top_half_width = 0.12;
    double bottom_half_width = 0.10;
    double upper_friction_coefficient = 0.55;
    double lower_friction_coefficient = 0.60;
    bool add_bottom_cap = false;
    double bottom_cap_friction_coefficient = 0.85;
};

/// @brief Procedural cylindrical shell settings used by pile-style scenes.
struct CylinderShellConfig {
    bool enabled = false;
    Vec3d center{0.5, 0.5, 0.5};
    double radius = 0.02;
    double y_min = 0.0;
    double y_max = 1.0;
    double friction_coefficient = 0.0;
    int segments = 24;
};

/// @brief Scene-level settings including domain bounds, seeding, and colliders.
struct SceneConfig {
    std::string name = "hourglass";
    Vec3d domain_min{0.0, 0.0, 0.0};
    Vec3d domain_max{1.0, 1.0, 1.0};
    Vec3d gravity{0.0, -9.81, 0.0};
    double domain_boundary_friction_coefficient = 0.0;
    Vec3d seed_box_min{0.25, 0.5, 0.25};
    Vec3d seed_box_max{0.75, 0.85, 0.75};
    bool seed_from_particles_per_cell = false;
    double seed_jitter = 0.0;
    int seed_particles = 4000;
    double duration = 2.0;
    int frame_stride = 10;
    int preview_steps = 8;
    HourglassShellConfig hourglass_shell;
    std::vector<CylinderShellConfig> cylinder_shells;
    std::vector<PlaneColliderConfig> colliders;
};

/// @brief Runtime emitter settings for scenes that inject particles over time.
struct EmitterConfig {
    bool enabled = false;
    std::string shape = "box";
    std::string sampling_mode = "random";
    Vec3d box_min{0.0, 0.0, 0.0};
    Vec3d box_max{0.0, 0.0, 0.0};
    double radius = 0.0;
    double jitter = 0.0;
    Vec3d initial_velocity{0.0, 0.0, 0.0};
    int particles_per_step = 0;
    int start_step = 0;
    int stop_step = -1;
    int max_particles = 0;
};

/// @brief PLY export controls for preview and regression output.
struct ExportConfig {
    bool export_ply = false;
    std::string output_directory = "outputs/.test";
    int interval_steps = 10;
    bool export_initial_frame = true;
};

/// @brief Root scene configuration loaded from a JSON file.
struct SimulationConfig {
    SceneConfig scene;
    MaterialConfig material;
    SolverConfig solver;
    EmitterConfig emitter;
    ExportConfig export_settings;
};

/// @brief Loads a simulation configuration from a scene JSON file.
SimulationConfig load_config(const std::filesystem::path &path);
/// @brief Produces a compact one-line description of the loaded configuration.
std::string describe(const SimulationConfig &config);
/// @brief Formats a labeled `Vec3d` for console logging.
std::string describe(std::string_view label, const Vec3d &value);

}  // namespace klar2016
