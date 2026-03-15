#include "klar2016/core/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <stdexcept>

#include <fmt/format.h>
#include <Eigen/Dense>

#include "klar2016/core/kernel.hpp"

namespace klar2016 {

namespace {

constexpr double kMinSingularValue = 1.0e-6;
constexpr double kPi = 3.14159265358979323846;
constexpr double kContactSlop = 5.0e-4;

struct LameParameters {
    double mu = 0.0;
    double lambda = 0.0;
};

struct PlasticProjectionResult {
    Eigen::Matrix3d elastic_deformation_gradient = Eigen::Matrix3d::Identity();
    double delta_q = 0.0;
};

struct RuntimePlaneCollider {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitY();
    double friction_coefficient = 0.0;
    bool bounded = false;
    Eigen::Vector3d bounds_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d bounds_max = Eigen::Vector3d::Zero();
};

Eigen::Vector3d to_eigen(const Vec3d &value) {
    return Eigen::Vector3d(value.x, value.y, value.z);
}

Vec3d to_vec3(const Eigen::Vector3d &value) {
    return Vec3d{value.x(), value.y(), value.z()};
}

PlaneColliderConfig make_plane_collider_config(
    const Eigen::Vector3d &point,
    const Eigen::Vector3d &normal,
    double friction_coefficient,
    const Eigen::Vector3d &bounds_min,
    const Eigen::Vector3d &bounds_max) {
    PlaneColliderConfig collider;
    collider.point = to_vec3(point);
    collider.normal = to_vec3(normal);
    collider.friction_coefficient = friction_coefficient;
    collider.bounded = true;
    collider.bounds_min = to_vec3(bounds_min);
    collider.bounds_max = to_vec3(bounds_max);
    return collider;
}

RuntimePlaneCollider make_runtime_plane_collider(const PlaneColliderConfig &collider) {
    RuntimePlaneCollider runtime_collider;
    runtime_collider.point = to_eigen(collider.point);
    runtime_collider.normal = to_eigen(collider.normal);
    const double normal_norm = runtime_collider.normal.norm();
    if (normal_norm <= 1.0e-12) {
        throw std::runtime_error("collider normal must be non-zero");
    }

    runtime_collider.normal /= normal_norm;
    runtime_collider.friction_coefficient = std::max(0.0, collider.friction_coefficient);
    runtime_collider.bounded = collider.bounded;
    if (runtime_collider.bounded) {
        runtime_collider.bounds_min = to_eigen(collider.bounds_min);
        runtime_collider.bounds_max = to_eigen(collider.bounds_max);
        if (
            (runtime_collider.bounds_max.array() <= runtime_collider.bounds_min.array()).any()) {
            throw std::runtime_error("collider bounds must have positive extent");
        }
    }
    return runtime_collider;
}

void append_hourglass_shell_colliders(
    const HourglassShellConfig &shell,
    std::vector<RuntimePlaneCollider> &runtime_colliders) {
    if (!shell.enabled) {
        return;
    }

    const Eigen::Vector3d center = to_eigen(shell.waist_center);
    if (!(shell.top_y > center.y() && center.y() > shell.bottom_y)) {
        throw std::runtime_error("hourglass_shell requires bottom_y < waist_center.y < top_y");
    }
    if (shell.neck_half_width <= 0.0) {
        throw std::runtime_error("hourglass_shell.neck_half_width must be positive");
    }
    if (shell.top_half_width <= shell.neck_half_width) {
        throw std::runtime_error(
            "hourglass_shell.top_half_width must exceed neck_half_width");
    }
    if (shell.bottom_half_width <= shell.neck_half_width) {
        throw std::runtime_error(
            "hourglass_shell.bottom_half_width must exceed neck_half_width");
    }

    constexpr double kBoundsPadding = 1.0e-3;
    const double upper_slope =
        (shell.top_half_width - shell.neck_half_width) /
        (shell.top_y - center.y());
    const double lower_slope =
        (shell.bottom_half_width - shell.neck_half_width) /
        (center.y() - shell.bottom_y);

    const auto push_plane = [&](const PlaneColliderConfig &config) {
        runtime_colliders.push_back(make_runtime_plane_collider(config));
    };

    push_plane(make_plane_collider_config(
        Eigen::Vector3d(center.x() - shell.neck_half_width, center.y(), center.z()),
        Eigen::Vector3d(1.0, upper_slope, 0.0).normalized(),
        shell.upper_friction_coefficient,
        Eigen::Vector3d(
            center.x() - shell.top_half_width - kBoundsPadding,
            center.y() - kBoundsPadding,
            center.z() - shell.top_half_width - kBoundsPadding),
        Eigen::Vector3d(
            center.x() - shell.neck_half_width + kBoundsPadding,
            shell.top_y + kBoundsPadding,
            center.z() + shell.top_half_width + kBoundsPadding)));
    push_plane(make_plane_collider_config(
        Eigen::Vector3d(center.x() + shell.neck_half_width, center.y(), center.z()),
        Eigen::Vector3d(-1.0, upper_slope, 0.0).normalized(),
        shell.upper_friction_coefficient,
        Eigen::Vector3d(
            center.x() + shell.neck_half_width - kBoundsPadding,
            center.y() - kBoundsPadding,
            center.z() - shell.top_half_width - kBoundsPadding),
        Eigen::Vector3d(
            center.x() + shell.top_half_width + kBoundsPadding,
            shell.top_y + kBoundsPadding,
            center.z() + shell.top_half_width + kBoundsPadding)));
    push_plane(make_plane_collider_config(
        Eigen::Vector3d(center.x(), center.y(), center.z() - shell.neck_half_width),
        Eigen::Vector3d(0.0, upper_slope, 1.0).normalized(),
        shell.upper_friction_coefficient,
        Eigen::Vector3d(
            center.x() - shell.top_half_width - kBoundsPadding,
            center.y() - kBoundsPadding,
            center.z() - shell.top_half_width - kBoundsPadding),
        Eigen::Vector3d(
            center.x() + shell.top_half_width + kBoundsPadding,
            shell.top_y + kBoundsPadding,
            center.z() - shell.neck_half_width + kBoundsPadding)));
    push_plane(make_plane_collider_config(
        Eigen::Vector3d(center.x(), center.y(), center.z() + shell.neck_half_width),
        Eigen::Vector3d(0.0, upper_slope, -1.0).normalized(),
        shell.upper_friction_coefficient,
        Eigen::Vector3d(
            center.x() - shell.top_half_width - kBoundsPadding,
            center.y() - kBoundsPadding,
            center.z() + shell.neck_half_width - kBoundsPadding),
        Eigen::Vector3d(
            center.x() + shell.top_half_width + kBoundsPadding,
            shell.top_y + kBoundsPadding,
            center.z() + shell.top_half_width + kBoundsPadding)));

    push_plane(make_plane_collider_config(
        Eigen::Vector3d(center.x() - shell.neck_half_width, center.y(), center.z()),
        Eigen::Vector3d(1.0, -lower_slope, 0.0).normalized(),
        shell.lower_friction_coefficient,
        Eigen::Vector3d(
            center.x() - shell.bottom_half_width - kBoundsPadding,
            shell.bottom_y - kBoundsPadding,
            center.z() - shell.bottom_half_width - kBoundsPadding),
        Eigen::Vector3d(
            center.x() - shell.neck_half_width + kBoundsPadding,
            center.y() + kBoundsPadding,
            center.z() + shell.bottom_half_width + kBoundsPadding)));
    push_plane(make_plane_collider_config(
        Eigen::Vector3d(center.x() + shell.neck_half_width, center.y(), center.z()),
        Eigen::Vector3d(-1.0, -lower_slope, 0.0).normalized(),
        shell.lower_friction_coefficient,
        Eigen::Vector3d(
            center.x() + shell.neck_half_width - kBoundsPadding,
            shell.bottom_y - kBoundsPadding,
            center.z() - shell.bottom_half_width - kBoundsPadding),
        Eigen::Vector3d(
            center.x() + shell.bottom_half_width + kBoundsPadding,
            center.y() + kBoundsPadding,
            center.z() + shell.bottom_half_width + kBoundsPadding)));
    push_plane(make_plane_collider_config(
        Eigen::Vector3d(center.x(), center.y(), center.z() - shell.neck_half_width),
        Eigen::Vector3d(0.0, -lower_slope, 1.0).normalized(),
        shell.lower_friction_coefficient,
        Eigen::Vector3d(
            center.x() - shell.bottom_half_width - kBoundsPadding,
            shell.bottom_y - kBoundsPadding,
            center.z() - shell.bottom_half_width - kBoundsPadding),
        Eigen::Vector3d(
            center.x() + shell.bottom_half_width + kBoundsPadding,
            center.y() + kBoundsPadding,
            center.z() - shell.neck_half_width + kBoundsPadding)));
    push_plane(make_plane_collider_config(
        Eigen::Vector3d(center.x(), center.y(), center.z() + shell.neck_half_width),
        Eigen::Vector3d(0.0, -lower_slope, -1.0).normalized(),
        shell.lower_friction_coefficient,
        Eigen::Vector3d(
            center.x() - shell.bottom_half_width - kBoundsPadding,
            shell.bottom_y - kBoundsPadding,
            center.z() + shell.neck_half_width - kBoundsPadding),
        Eigen::Vector3d(
            center.x() + shell.bottom_half_width + kBoundsPadding,
            center.y() + kBoundsPadding,
            center.z() + shell.bottom_half_width + kBoundsPadding)));

    if (shell.add_bottom_cap) {
        push_plane(make_plane_collider_config(
            Eigen::Vector3d(center.x(), shell.bottom_y, center.z()),
            Eigen::Vector3d::UnitY(),
            shell.bottom_cap_friction_coefficient,
            Eigen::Vector3d(
                center.x() - shell.bottom_half_width - kBoundsPadding,
                shell.bottom_y - kBoundsPadding,
                center.z() - shell.bottom_half_width - kBoundsPadding),
            Eigen::Vector3d(
                center.x() + shell.bottom_half_width + kBoundsPadding,
                shell.bottom_y + kBoundsPadding,
                center.z() + shell.bottom_half_width + kBoundsPadding)));
    }
}

void append_cylinder_shell_colliders(
    const CylinderShellConfig &shell,
    std::vector<RuntimePlaneCollider> &runtime_colliders) {
    if (!shell.enabled) {
        return;
    }

    if (shell.radius <= 0.0) {
        throw std::runtime_error("cylinder_shell.radius must be positive");
    }
    if (!(shell.y_max > shell.y_min)) {
        throw std::runtime_error("cylinder_shell requires y_min < y_max");
    }
    if (shell.segments < 6) {
        throw std::runtime_error("cylinder_shell.segments must be at least 6");
    }

    constexpr double kBoundsPadding = 1.0e-3;
    const Eigen::Vector3d center = to_eigen(shell.center);
    for (int segment = 0; segment < shell.segments; ++segment) {
        const double theta0 =
            2.0 * kPi * static_cast<double>(segment) / static_cast<double>(shell.segments);
        const double theta1 =
            2.0 * kPi * static_cast<double>(segment + 1) / static_cast<double>(shell.segments);
        const double theta_mid = 0.5 * (theta0 + theta1);
        const Eigen::Vector3d radial(std::cos(theta_mid), 0.0, std::sin(theta_mid));
        const Eigen::Vector3d point =
            center + shell.radius * radial + Eigen::Vector3d(0.0, shell.y_min, 0.0);

        const double x0 = center.x() + shell.radius * std::cos(theta0);
        const double z0 = center.z() + shell.radius * std::sin(theta0);
        const double x1 = center.x() + shell.radius * std::cos(theta1);
        const double z1 = center.z() + shell.radius * std::sin(theta1);

        const Eigen::Vector3d bounds_min(
            std::min(x0, x1) - kBoundsPadding,
            center.y() + shell.y_min - kBoundsPadding,
            std::min(z0, z1) - kBoundsPadding);
        const Eigen::Vector3d bounds_max(
            std::max(x0, x1) + kBoundsPadding,
            center.y() + shell.y_max + kBoundsPadding,
            std::max(z0, z1) + kBoundsPadding);

        runtime_colliders.push_back(make_runtime_plane_collider(make_plane_collider_config(
            point,
            -radial,
            shell.friction_coefficient,
            bounds_min,
            bounds_max)));
    }
}

bool plane_collider_contains_projection(
    const RuntimePlaneCollider &collider,
    const Eigen::Vector3d &projection) {
    if (!collider.bounded) {
        return true;
    }

    constexpr double kEpsilon = 1.0e-9;
    return
        (projection.array() >= collider.bounds_min.array() - kEpsilon).all() &&
        (projection.array() <= collider.bounds_max.array() + kEpsilon).all();
}

void apply_unilateral_boundary_response(
    const Eigen::Vector3d &normal,
    double friction_coefficient,
    Eigen::Vector3d &velocity) {
    const double inward_normal_speed = velocity.dot(normal);
    if (inward_normal_speed >= 0.0) {
        return;
    }

    Eigen::Vector3d tangential_velocity = velocity - inward_normal_speed * normal;
    const double tangential_speed = tangential_velocity.norm();
    if (tangential_speed > 1.0e-12) {
        const double friction_drop = friction_coefficient * (-inward_normal_speed);
        const double new_tangential_speed = std::max(0.0, tangential_speed - friction_drop);
        tangential_velocity *= new_tangential_speed / tangential_speed;
    } else {
        tangential_velocity.setZero();
    }

    velocity = tangential_velocity;
}

void apply_plane_collider_velocity(
    const RuntimePlaneCollider &collider,
    const Eigen::Vector3d &position,
    Eigen::Vector3d &velocity) {
    const double signed_distance = (position - collider.point).dot(collider.normal);
    const Eigen::Vector3d projection = position - signed_distance * collider.normal;
    if (!plane_collider_contains_projection(collider, projection)) {
        return;
    }

    if (signed_distance <= kContactSlop) {
        apply_unilateral_boundary_response(
            collider.normal,
            collider.friction_coefficient,
            velocity);
    }
}

void project_particle_against_plane(
    const RuntimePlaneCollider &collider,
    Eigen::Vector3d &position,
    Eigen::Vector3d &velocity) {
    const double signed_distance = (position - collider.point).dot(collider.normal);
    const Eigen::Vector3d projection = position - signed_distance * collider.normal;
    if (!plane_collider_contains_projection(collider, projection)) {
        return;
    }

    if (signed_distance < 0.0) {
        position -= signed_distance * collider.normal;
    }
    if (signed_distance <= kContactSlop) {
        apply_unilateral_boundary_response(
            collider.normal,
            collider.friction_coefficient,
            velocity);
    }
}

LameParameters compute_lame_parameters(const MaterialConfig &material) {
    const double youngs_modulus = material.youngs_modulus;
    const double poisson_ratio = material.poisson_ratio;

    LameParameters result;
    result.mu = youngs_modulus / (2.0 * (1.0 + poisson_ratio));
    result.lambda =
        youngs_modulus * poisson_ratio /
        ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio));
    return result;
}

double friction_angle_from_hardening_state(
    double hardening_state,
    const MaterialConfig &material) {
    if (
        material.hardening_h1 == 0.0 &&
        material.hardening_h2 == 0.0 &&
        material.hardening_h3 == 0.0) {
        return material.friction_angle_degrees;
    }

    return
        material.hardening_h0 +
        (material.hardening_h1 * hardening_state - material.hardening_h3) *
            std::exp(-material.hardening_h2 * hardening_state);
}

double alpha_from_friction_angle_degrees(double friction_angle_degrees) {
    const double clamped_degrees = std::clamp(friction_angle_degrees, 0.0, 89.0);
    const double phi = clamped_degrees * kPi / 180.0;
    return
        std::sqrt(2.0 / 3.0) *
        (2.0 * std::sin(phi)) /
        (3.0 - std::sin(phi));
}

double current_alpha(const Simulation::Particle &particle, const MaterialConfig &material) {
    if (
        material.hardening_h1 == 0.0 &&
        material.hardening_h2 == 0.0 &&
        material.hardening_h3 == 0.0) {
        return alpha_from_friction_angle_degrees(material.friction_angle_degrees);
    }

    return alpha_from_friction_angle_degrees(
        friction_angle_from_hardening_state(particle.hardening_state, material));
}

Eigen::Matrix3d elastic_energy_derivative(
    const Eigen::Matrix3d &elastic_deformation_gradient,
    const MaterialConfig &material) {
    const auto lame = compute_lame_parameters(material);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        elastic_deformation_gradient,
        Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Clamp singular values away from zero to keep log-strain evaluation finite.
    const Eigen::Vector3d singular_values =
        svd.singularValues().cwiseMax(kMinSingularValue);
    const Eigen::Vector3d log_singular_values = singular_values.array().log().matrix();
    const double trace_log = log_singular_values.sum();

    Eigen::Vector3d gradient_diagonal = Eigen::Vector3d::Zero();
    for (int axis = 0; axis < 3; ++axis) {
        gradient_diagonal[axis] =
            (2.0 * lame.mu * log_singular_values[axis] + lame.lambda * trace_log) /
            singular_values[axis];
    }

    return
        svd.matrixU() * gradient_diagonal.asDiagonal() * svd.matrixV().transpose();
}

PlasticProjectionResult project_drucker_prager(
    const Eigen::Matrix3d &candidate_elastic_deformation_gradient,
    double alpha,
    const MaterialConfig &material) {
    const auto lame = compute_lame_parameters(material);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        candidate_elastic_deformation_gradient,
        Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Vector3d singular_values =
        svd.singularValues().cwiseMax(kMinSingularValue);
    const Eigen::Vector3d epsilon = singular_values.array().log().matrix();
    const double trace_epsilon = epsilon.sum();
    const Eigen::Vector3d deviatoric_epsilon =
        epsilon - Eigen::Vector3d::Constant(trace_epsilon / 3.0);
    const double deviatoric_norm = deviatoric_epsilon.norm();
    const double delta_gamma =
        deviatoric_norm +
        ((3.0 * lame.lambda + 2.0 * lame.mu) / (2.0 * lame.mu)) *
            trace_epsilon *
            alpha;

    if (delta_gamma <= 0.0) {
        PlasticProjectionResult result;
        result.elastic_deformation_gradient = candidate_elastic_deformation_gradient;
        return result;
    }

    if (deviatoric_norm <= 1.0e-12 || trace_epsilon > 0.0) {
        PlasticProjectionResult result;
        result.elastic_deformation_gradient = svd.matrixU() * svd.matrixV().transpose();
        result.delta_q = epsilon.norm();
        return result;
    }

    const Eigen::Vector3d projected_log_singular_values =
        epsilon - delta_gamma * deviatoric_epsilon / deviatoric_norm;
    const Eigen::Vector3d projected_singular_values =
        projected_log_singular_values.array().exp().matrix();

    PlasticProjectionResult result;
    result.elastic_deformation_gradient =
        svd.matrixU() *
        projected_singular_values.asDiagonal() *
        svd.matrixV().transpose();
    result.delta_q = std::max(0.0, delta_gamma);
    return result;
}

}  // namespace

Simulation::Simulation(SimulationConfig config) : config_(std::move(config)) {
    const Eigen::Vector3d min_corner(
        config_.scene.domain_min.x,
        config_.scene.domain_min.y,
        config_.scene.domain_min.z);
    const Eigen::Vector3d max_corner(
        config_.scene.domain_max.x,
        config_.scene.domain_max.y,
        config_.scene.domain_max.z);
    const Eigen::Vector3d extent = max_corner - min_corner;
    const Eigen::Array3d cell_counts = (extent.array() / config_.solver.dx).ceil();

    grid_nx_ = static_cast<int>(cell_counts.x()) + 1;
    grid_ny_ = static_cast<int>(cell_counts.y()) + 1;
    grid_nz_ = static_cast<int>(cell_counts.z()) + 1;

    initialize_particles();
}

int Simulation::estimated_grid_cells() const {
    const Eigen::Vector3d min_corner(
        config_.scene.domain_min.x,
        config_.scene.domain_min.y,
        config_.scene.domain_min.z);
    const Eigen::Vector3d max_corner(
        config_.scene.domain_max.x,
        config_.scene.domain_max.y,
        config_.scene.domain_max.z);

    const Eigen::Vector3d extent = max_corner - min_corner;
    const Eigen::Array3d counts = (extent.array() / config_.solver.dx).ceil();

    return static_cast<int>(counts.x() * counts.y() * counts.z());
}

void Simulation::advance(int steps) {
    for (int step = 0; step < steps; ++step) {
        substep();
    }
}

int Simulation::current_step() const {
    return current_step_;
}

void Simulation::write_particle_ply(const std::filesystem::path &path) const {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }

    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error(fmt::format("Failed to open particle export '{}'", path.string()));
    }

    output << "ply\n";
    output << "format ascii 1.0\n";
    output << "element vertex " << particles_.size() << '\n';
    output << "property float x\n";
    output << "property float y\n";
    output << "property float z\n";
    output << "property float vx\n";
    output << "property float vy\n";
    output << "property float vz\n";
    output << "property float alpha\n";
    output << "property float det_fe\n";
    output << "end_header\n";
    output << std::fixed << std::setprecision(9);

    for (const auto &particle : particles_) {
        output
            << particle.position.x() << ' '
            << particle.position.y() << ' '
            << particle.position.z() << ' '
            << particle.velocity.x() << ' '
            << particle.velocity.y() << ' '
            << particle.velocity.z() << ' '
            << particle.alpha << ' '
            << particle.elastic_deformation_gradient.determinant() << '\n';
    }
}

SimulationStats Simulation::stats() const {
    SimulationStats result;
    result.particle_count = static_cast<int>(particles_.size());
    result.active_grid_nodes = static_cast<int>(grid_.size());

    double total_speed = 0.0;
    double total_det_fe = 0.0;
    double total_alpha = 0.0;
    Eigen::Vector3d bbox_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d bbox_max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
    for (const auto &particle : particles_) {
        result.total_mass += particle.mass;
        total_speed += particle.velocity.norm();
        total_det_fe += particle.elastic_deformation_gradient.determinant();
        total_alpha += particle.alpha;
        bbox_min = bbox_min.cwiseMin(particle.position);
        bbox_max = bbox_max.cwiseMax(particle.position);
    }

    if (!particles_.empty()) {
        result.average_speed = total_speed / static_cast<double>(particles_.size());
        result.average_det_fe = total_det_fe / static_cast<double>(particles_.size());
        result.average_alpha = total_alpha / static_cast<double>(particles_.size());
        const Eigen::Vector3d extent = (bbox_max - bbox_min).cwiseMax(Eigen::Vector3d::Zero());
        result.particle_bbox_volume = extent.x() * extent.y() * extent.z();
        result.particle_height_span = extent.y();
    }

    return result;
}

std::string Simulation::build_summary() const {
    const auto current_stats = stats();
    return fmt::format(
        "KLAR2016 development step: scene='{}', estimated_grid_cells={}, active_grid_nodes={}, particles={}, avg_speed={:.6f}, avg_det_fe={:.6f}, avg_alpha={:.6f}, bbox_volume={:.6f}, height_span={:.6f}, explicit_only={}",
        config_.scene.name,
        estimated_grid_cells(),
        current_stats.active_grid_nodes,
        current_stats.particle_count,
        current_stats.average_speed,
        current_stats.average_det_fe,
        current_stats.average_alpha,
        current_stats.particle_bbox_volume,
        current_stats.particle_height_span,
        config_.solver.explicit_only ? "true" : "false");
}

void Simulation::initialize_particles() {
    if (config_.scene.seed_particles < 0) {
        throw std::runtime_error("scene.seed_particles must be non-negative");
    }

    if (
        !config_.scene.seed_from_particles_per_cell &&
        config_.scene.seed_particles == 0 &&
        !config_.emitter.enabled) {
        throw std::runtime_error("scene.seed_particles must be positive when emitter is disabled");
    }

    const Eigen::Vector3d seed_min(
        config_.scene.seed_box_min.x,
        config_.scene.seed_box_min.y,
        config_.scene.seed_box_min.z);
    const Eigen::Vector3d seed_max(
        config_.scene.seed_box_max.x,
        config_.scene.seed_box_max.y,
        config_.scene.seed_box_max.z);

    const Eigen::Vector3d extent = seed_max - seed_min;
    const double volume = extent.x() * extent.y() * extent.z();

    if (config_.scene.seed_from_particles_per_cell) {
        const int samples_per_axis =
            std::max(
                1,
                static_cast<int>(std::lround(
                    std::cbrt(static_cast<double>(config_.solver.particles_per_cell)))));
        if (samples_per_axis * samples_per_axis * samples_per_axis != config_.solver.particles_per_cell) {
            throw std::runtime_error(
                "solver.particles_per_cell must be a perfect cube for regular ppc seeding");
        }

        const double spacing = config_.solver.dx / static_cast<double>(samples_per_axis);
        const double particle_volume =
            std::pow(config_.solver.dx, 3.0) /
            static_cast<double>(config_.solver.particles_per_cell);
        const double particle_mass = config_.material.density * particle_volume;
        const Eigen::Array3d counts =
            ((extent.array() / spacing) + 1.0e-9).floor().max(0.0);
        const int particle_count = static_cast<int>(counts.x() * counts.y() * counts.z());
        particles_.reserve(static_cast<std::size_t>(std::max(0, particle_count)));
        append_regular_seed_particles(
            seed_min,
            seed_max,
            spacing,
            config_.scene.seed_jitter,
            Eigen::Vector3d::Zero(),
            particle_mass,
            particle_volume);
    } else if (config_.scene.seed_particles > 0) {
        const double particle_volume = volume / static_cast<double>(config_.scene.seed_particles);
        const double particle_mass = config_.material.density * particle_volume;
        particles_.reserve(static_cast<std::size_t>(config_.scene.seed_particles));
        append_seed_particles(
            seed_min,
            seed_max,
            config_.scene.seed_particles,
            Eigen::Vector3d::Zero(),
            particle_mass,
            particle_volume);
    }
}

void Simulation::append_regular_seed_particles(
    const Eigen::Vector3d &box_min,
    const Eigen::Vector3d &box_max,
    double spacing,
    double jitter,
    const Eigen::Vector3d &initial_velocity,
    double particle_mass,
    double particle_volume) {
    if (spacing <= 0.0) {
        throw std::runtime_error("regular seed particle spacing must be positive");
    }

    if ((box_max.array() <= box_min.array()).any()) {
        throw std::runtime_error("particle source box must have positive extent");
    }

    const Eigen::Vector3d extent = box_max - box_min;
    const Eigen::Array3i counts =
        ((extent.array() / spacing) + 1.0e-9).floor().cast<int>().max(1);
    const double clamped_jitter = std::clamp(jitter, 0.0, 0.95);
    std::uniform_real_distribution<double> unit_dist(-0.5, 0.5);
    constexpr double kMarginFraction = 0.1;
    const double margin = kMarginFraction * spacing;

    for (int ix = 0; ix < counts.x(); ++ix) {
        for (int iy = 0; iy < counts.y(); ++iy) {
            for (int iz = 0; iz < counts.z(); ++iz) {
                Eigen::Vector3d position =
                    box_min +
                    spacing * Eigen::Vector3d(
                        static_cast<double>(ix) + 0.5,
                        static_cast<double>(iy) + 0.5,
                        static_cast<double>(iz) + 0.5);
                if (clamped_jitter > 0.0) {
                    const Eigen::Vector3d offset(
                        unit_dist(rng_),
                        unit_dist(rng_),
                        unit_dist(rng_));
                    position += clamped_jitter * spacing * offset;
                }
                const Eigen::Vector3d padded_min =
                    (box_min.array() + margin).matrix();
                const Eigen::Vector3d padded_max =
                    (box_max.array() - margin).matrix();
                position = position.cwiseMax(padded_min).cwiseMin(padded_max);

                Particle particle;
                particle.position = position;
                particle.velocity = initial_velocity;
                particle.mass = particle_mass;
                particle.initial_volume = particle_volume;
                particle.alpha = current_alpha(particle, config_.material);
                particles_.push_back(particle);
            }
        }
    }
}

void Simulation::append_seed_particles(
    const Eigen::Vector3d &box_min,
    const Eigen::Vector3d &box_max,
    int particle_count,
    const Eigen::Vector3d &initial_velocity,
    double particle_mass,
    double particle_volume) {
    if (particle_count <= 0) {
        return;
    }

    if ((box_max.array() <= box_min.array()).any()) {
        throw std::runtime_error("particle source box must have positive extent");
    }

    std::uniform_real_distribution<double> dist_x(box_min.x(), box_max.x());
    std::uniform_real_distribution<double> dist_y(box_min.y(), box_max.y());
    std::uniform_real_distribution<double> dist_z(box_min.z(), box_max.z());

    for (int i = 0; i < particle_count; ++i) {
        Particle particle;
        particle.position = Eigen::Vector3d(dist_x(rng_), dist_y(rng_), dist_z(rng_));
        particle.velocity = initial_velocity;
        particle.mass = particle_mass;
        particle.initial_volume = particle_volume;
        particle.alpha = current_alpha(particle, config_.material);
        particles_.push_back(particle);
    }
}

void Simulation::append_cylindrical_particles(
    const Eigen::Vector3d &center,
    double radius,
    double y_min,
    double y_max,
    int particle_count,
    const Eigen::Vector3d &initial_velocity,
    double particle_mass,
    double particle_volume) {
    if (particle_count <= 0) {
        return;
    }

    if (radius <= 0.0) {
        throw std::runtime_error("cylindrical particle source radius must be positive");
    }
    if (!(y_max > y_min)) {
        throw std::runtime_error("cylindrical particle source requires y_min < y_max");
    }

    std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * kPi);
    std::uniform_real_distribution<double> radial_dist(0.0, 1.0);
    std::uniform_real_distribution<double> height_dist(y_min, y_max);

    for (int i = 0; i < particle_count; ++i) {
        const double angle = angle_dist(rng_);
        const double radial = radius * std::sqrt(radial_dist(rng_));
        Particle particle;
        particle.position = Eigen::Vector3d(
            center.x() + radial * std::cos(angle),
            height_dist(rng_),
            center.z() + radial * std::sin(angle));
        particle.velocity = initial_velocity;
        particle.mass = particle_mass;
        particle.initial_volume = particle_volume;
        particle.alpha = current_alpha(particle, config_.material);
        particles_.push_back(particle);
    }
}

void Simulation::append_stratified_cylindrical_particles(
    const Eigen::Vector3d &center,
    double radius,
    double y_min,
    double y_max,
    int particle_count,
    double jitter,
    const Eigen::Vector3d &initial_velocity,
    double particle_mass,
    double particle_volume) {
    if (particle_count <= 0) {
        return;
    }

    if (radius <= 0.0) {
        throw std::runtime_error("cylindrical particle source radius must be positive");
    }
    if (!(y_max > y_min)) {
        throw std::runtime_error("cylindrical particle source requires y_min < y_max");
    }

    const double height = y_max - y_min;
    const double source_volume = kPi * radius * radius * height;
    const double nominal_spacing =
        std::cbrt(source_volume / static_cast<double>(particle_count));
    const int y_layers = std::clamp(
        static_cast<int>(std::round(height / std::max(nominal_spacing, 1.0e-9))),
        1,
        particle_count);
    const double clamped_jitter = std::clamp(jitter, 0.0, 0.95);
    const double golden_angle = kPi * (3.0 - std::sqrt(5.0));
    std::uniform_real_distribution<double> unit_dist(-0.5, 0.5);
    std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * kPi);

    int generated_particles = 0;
    for (int layer = 0; layer < y_layers; ++layer) {
        const int layer_count =
            particle_count / y_layers + ((layer < (particle_count % y_layers)) ? 1 : 0);
        if (layer_count <= 0) {
            continue;
        }

        const double layer_phase = phase_dist(rng_) + golden_angle * static_cast<double>(layer);
        const double layer_fraction =
            (static_cast<double>(layer) + 0.5) / static_cast<double>(y_layers);
        const double base_y = y_min + layer_fraction * height;
        const double layer_spacing = radius / std::sqrt(static_cast<double>(layer_count));
        const double y_spacing = height / static_cast<double>(y_layers);

        for (int sample = 0; sample < layer_count; ++sample) {
            const double sample_fraction =
                (static_cast<double>(sample) + 0.5) / static_cast<double>(layer_count);
            double radial_fraction = std::sqrt(sample_fraction);
            double angle = layer_phase + golden_angle * static_cast<double>(sample);

            if (clamped_jitter > 0.0) {
                radial_fraction +=
                    clamped_jitter * 0.5 * (layer_spacing / std::max(radius, 1.0e-9)) *
                    unit_dist(rng_);
                angle +=
                    clamped_jitter *
                    (2.0 * kPi / static_cast<double>(std::max(layer_count, 1))) *
                    unit_dist(rng_);
            }

            radial_fraction = std::clamp(radial_fraction, 0.0, 1.0);
            const double radial = radius * radial_fraction;
            double y = base_y;
            if (clamped_jitter > 0.0) {
                y += clamped_jitter * 0.35 * y_spacing * unit_dist(rng_);
            }
            y = std::clamp(y, y_min, y_max);

            Particle particle;
            particle.position = Eigen::Vector3d(
                center.x() + radial * std::cos(angle),
                y,
                center.z() + radial * std::sin(angle));
            particle.velocity = initial_velocity;
            particle.mass = particle_mass;
            particle.initial_volume = particle_volume;
            particle.alpha = current_alpha(particle, config_.material);
            particles_.push_back(particle);
            ++generated_particles;
        }
    }

    if (generated_particles != particle_count) {
        throw std::runtime_error("stratified cylindrical emitter generated an unexpected count");
    }
}

void Simulation::emit_particles() {
    if (!config_.emitter.enabled || config_.emitter.particles_per_step <= 0) {
        return;
    }

    if (current_step_ < config_.emitter.start_step) {
        return;
    }

    if (config_.emitter.stop_step >= 0 && current_step_ > config_.emitter.stop_step) {
        return;
    }

    int particle_count = config_.emitter.particles_per_step;
    if (config_.emitter.max_particles > 0) {
        particle_count = std::min(
            particle_count,
            config_.emitter.max_particles - emitted_particles_);
    }

    if (particle_count <= 0) {
        return;
    }

    const Eigen::Vector3d emitter_min(
        config_.emitter.box_min.x,
        config_.emitter.box_min.y,
        config_.emitter.box_min.z);
    const Eigen::Vector3d emitter_max(
        config_.emitter.box_max.x,
        config_.emitter.box_max.y,
        config_.emitter.box_max.z);
    const Eigen::Vector3d initial_velocity(
        config_.emitter.initial_velocity.x,
        config_.emitter.initial_velocity.y,
        config_.emitter.initial_velocity.z);
    const Eigen::Vector3d emitter_extent = emitter_max - emitter_min;
    const double emitter_height = emitter_extent.y();
    double emitter_volume =
        emitter_extent.x() * emitter_extent.y() * emitter_extent.z();
    if (config_.emitter.shape == "cylinder") {
        if (config_.emitter.radius <= 0.0) {
            throw std::runtime_error("emitter.radius must be positive for cylinder emitters");
        }
        emitter_volume = kPi * config_.emitter.radius * config_.emitter.radius * emitter_height;
    } else if (config_.emitter.shape != "box") {
        throw std::runtime_error("emitter.shape must be either 'box' or 'cylinder'");
    }
    const double particle_volume = emitter_volume / static_cast<double>(particle_count);
    const double particle_mass = config_.material.density * particle_volume;

    if (config_.emitter.shape == "cylinder") {
        const Eigen::Vector3d center(
            0.5 * (emitter_min.x() + emitter_max.x()),
            0.0,
            0.5 * (emitter_min.z() + emitter_max.z()));
        if (config_.emitter.sampling_mode == "stratified") {
            append_stratified_cylindrical_particles(
                center,
                config_.emitter.radius,
                emitter_min.y(),
                emitter_max.y(),
                particle_count,
                config_.emitter.jitter,
                initial_velocity,
                particle_mass,
                particle_volume);
        } else if (config_.emitter.sampling_mode == "random") {
            append_cylindrical_particles(
                center,
                config_.emitter.radius,
                emitter_min.y(),
                emitter_max.y(),
                particle_count,
                initial_velocity,
                particle_mass,
                particle_volume);
        } else {
            throw std::runtime_error(
                "emitter.sampling_mode must be either 'random' or 'stratified'");
        }
    } else {
        append_seed_particles(
            emitter_min,
            emitter_max,
            particle_count,
            initial_velocity,
            particle_mass,
            particle_volume);
    }
    emitted_particles_ += particle_count;
}

void Simulation::reset_grid() {
    grid_.clear();
}

void Simulation::substep() {
    emit_particles();
    reset_grid();

    const Eigen::Vector3d domain_min = to_eigen(config_.scene.domain_min);
    const Eigen::Vector3d domain_max = to_eigen(config_.scene.domain_max);
    const Eigen::Vector3d gravity = to_eigen(config_.scene.gravity);
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    const double inv_dx = 1.0 / config_.solver.dx;
    const double dt = config_.solver.dt_max;
    const double cubic_d_inv = 3.0 * inv_dx * inv_dx;
    std::vector<RuntimePlaneCollider> runtime_colliders;
    std::size_t cylinder_segment_count = 0;
    for (const auto &shell : config_.scene.cylinder_shells) {
        cylinder_segment_count += static_cast<std::size_t>(std::max(0, shell.segments));
    }
    runtime_colliders.reserve(6 + config_.scene.colliders.size() + 9 + cylinder_segment_count);

    const double boundary_friction = std::max(0.0, config_.scene.domain_boundary_friction_coefficient);
    runtime_colliders.push_back(RuntimePlaneCollider{
        Eigen::Vector3d(domain_min.x(), domain_min.y(), domain_min.z()),
        Eigen::Vector3d::UnitX(),
        boundary_friction});
    runtime_colliders.push_back(RuntimePlaneCollider{
        Eigen::Vector3d(domain_max.x(), domain_min.y(), domain_min.z()),
        -Eigen::Vector3d::UnitX(),
        boundary_friction});
    runtime_colliders.push_back(RuntimePlaneCollider{
        Eigen::Vector3d(domain_min.x(), domain_min.y(), domain_min.z()),
        Eigen::Vector3d::UnitY(),
        boundary_friction});
    runtime_colliders.push_back(RuntimePlaneCollider{
        Eigen::Vector3d(domain_min.x(), domain_max.y(), domain_min.z()),
        -Eigen::Vector3d::UnitY(),
        boundary_friction});
    runtime_colliders.push_back(RuntimePlaneCollider{
        Eigen::Vector3d(domain_min.x(), domain_min.y(), domain_min.z()),
        Eigen::Vector3d::UnitZ(),
        boundary_friction});
    runtime_colliders.push_back(RuntimePlaneCollider{
        Eigen::Vector3d(domain_min.x(), domain_min.y(), domain_max.z()),
        -Eigen::Vector3d::UnitZ(),
        boundary_friction});

    for (const auto &collider : config_.scene.colliders) {
        runtime_colliders.push_back(make_runtime_plane_collider(collider));
    }
    append_hourglass_shell_colliders(config_.scene.hourglass_shell, runtime_colliders);
    for (const auto &shell : config_.scene.cylinder_shells) {
        append_cylinder_shell_colliders(shell, runtime_colliders);
    }

    for (const auto &particle : particles_) {
        const Eigen::Vector3d position_in_grid = (particle.position - domain_min) * inv_dx;
        const auto sx = build_cubic_axis_stencil(position_in_grid.x(), inv_dx);
        const auto sy = build_cubic_axis_stencil(position_in_grid.y(), inv_dx);
        const auto sz = build_cubic_axis_stencil(position_in_grid.z(), inv_dx);
        const Eigen::Matrix3d affine_c = particle.affine_b * cubic_d_inv;

        for (int ix = 0; ix < 4; ++ix) {
            for (int iy = 0; iy < 4; ++iy) {
                for (int iz = 0; iz < 4; ++iz) {
                    const int i = sx.nodes[static_cast<std::size_t>(ix)];
                    const int j = sy.nodes[static_cast<std::size_t>(iy)];
                    const int k = sz.nodes[static_cast<std::size_t>(iz)];
                    if (!inside_grid(i, j, k)) {
                        continue;
                    }

                    const double weight =
                        sx.weights[static_cast<std::size_t>(ix)] *
                        sy.weights[static_cast<std::size_t>(iy)] *
                        sz.weights[static_cast<std::size_t>(iz)];
                    const Eigen::Vector3d node_position =
                        domain_min +
                        config_.solver.dx * Eigen::Vector3d(
                            static_cast<double>(i),
                            static_cast<double>(j),
                            static_cast<double>(k));
                    const Eigen::Vector3d dpos = node_position - particle.position;
                    const Eigen::Vector3d node_momentum =
                        particle.mass * (particle.velocity + affine_c * dpos);

                    auto &node = grid_[grid_index(i, j, k)];
                    node.mass += weight * particle.mass;
                    node.momentum += weight * node_momentum;
                }
            }
        }
    }

    for (const auto &particle : particles_) {
        const Eigen::Vector3d position_in_grid = (particle.position - domain_min) * inv_dx;
        const auto sx = build_cubic_axis_stencil(position_in_grid.x(), inv_dx);
        const auto sy = build_cubic_axis_stencil(position_in_grid.y(), inv_dx);
        const auto sz = build_cubic_axis_stencil(position_in_grid.z(), inv_dx);
        const Eigen::Matrix3d elastic_gradient =
            elastic_energy_derivative(
                particle.elastic_deformation_gradient,
                config_.material);
        const Eigen::Matrix3d force_matrix =
            -particle.initial_volume *
            elastic_gradient *
            particle.elastic_deformation_gradient.transpose();

        for (int ix = 0; ix < 4; ++ix) {
            for (int iy = 0; iy < 4; ++iy) {
                for (int iz = 0; iz < 4; ++iz) {
                    const int i = sx.nodes[static_cast<std::size_t>(ix)];
                    const int j = sy.nodes[static_cast<std::size_t>(iy)];
                    const int k = sz.nodes[static_cast<std::size_t>(iz)];
                    if (!inside_grid(i, j, k)) {
                        continue;
                    }

                    const Eigen::Vector3d grad_w(
                        sx.gradients[static_cast<std::size_t>(ix)] *
                            sy.weights[static_cast<std::size_t>(iy)] *
                            sz.weights[static_cast<std::size_t>(iz)],
                        sx.weights[static_cast<std::size_t>(ix)] *
                            sy.gradients[static_cast<std::size_t>(iy)] *
                            sz.weights[static_cast<std::size_t>(iz)],
                        sx.weights[static_cast<std::size_t>(ix)] *
                            sy.weights[static_cast<std::size_t>(iy)] *
                            sz.gradients[static_cast<std::size_t>(iz)]);

                    auto &node = grid_[grid_index(i, j, k)];
                    node.momentum += dt * force_matrix * grad_w;
                }
            }
        }
    }

    for (auto &[flat_index, node] : grid_) {
        if (node.mass <= 0.0) {
            continue;
        }

        const Eigen::Vector3i node_index = unpack_grid_index(flat_index);
        const int i = node_index.x();
        const int j = node_index.y();
        const int k = node_index.z();
        const Eigen::Vector3d node_position =
            domain_min +
            config_.solver.dx *
                Eigen::Vector3d(
                    static_cast<double>(i),
                    static_cast<double>(j),
                    static_cast<double>(k));

        node.momentum += dt * node.mass * gravity;
        node.velocity = node.momentum / node.mass;

        for (const auto &collider : runtime_colliders) {
            apply_plane_collider_velocity(collider, node_position, node.velocity);
        }
    }

    for (auto &particle : particles_) {
        const Eigen::Vector3d position_in_grid = (particle.position - domain_min) * inv_dx;
        const auto sx = build_cubic_axis_stencil(position_in_grid.x(), inv_dx);
        const auto sy = build_cubic_axis_stencil(position_in_grid.y(), inv_dx);
        const auto sz = build_cubic_axis_stencil(position_in_grid.z(), inv_dx);

        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        Eigen::Matrix3d affine_b = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d velocity_gradient = Eigen::Matrix3d::Zero();
        for (int ix = 0; ix < 4; ++ix) {
            for (int iy = 0; iy < 4; ++iy) {
                for (int iz = 0; iz < 4; ++iz) {
                    const int i = sx.nodes[static_cast<std::size_t>(ix)];
                    const int j = sy.nodes[static_cast<std::size_t>(iy)];
                    const int k = sz.nodes[static_cast<std::size_t>(iz)];
                    if (!inside_grid(i, j, k)) {
                        continue;
                    }

                    const double weight =
                        sx.weights[static_cast<std::size_t>(ix)] *
                        sy.weights[static_cast<std::size_t>(iy)] *
                        sz.weights[static_cast<std::size_t>(iz)];
                    const auto node_it = grid_.find(grid_index(i, j, k));
                    if (node_it == grid_.end()) {
                        continue;
                    }

                    const auto &node = node_it->second;
                    const Eigen::Vector3d node_position =
                        domain_min +
                        config_.solver.dx * Eigen::Vector3d(
                            static_cast<double>(i),
                            static_cast<double>(j),
                            static_cast<double>(k));
                    const Eigen::Vector3d dpos = node_position - particle.position;
                    const Eigen::Vector3d grad_w(
                        sx.gradients[static_cast<std::size_t>(ix)] *
                            sy.weights[static_cast<std::size_t>(iy)] *
                            sz.weights[static_cast<std::size_t>(iz)],
                        sx.weights[static_cast<std::size_t>(ix)] *
                            sy.gradients[static_cast<std::size_t>(iy)] *
                            sz.weights[static_cast<std::size_t>(iz)],
                        sx.weights[static_cast<std::size_t>(ix)] *
                            sy.weights[static_cast<std::size_t>(iy)] *
                            sz.gradients[static_cast<std::size_t>(iz)]);
                    velocity += weight * node.velocity;
                    affine_b += weight * node.velocity * dpos.transpose();
                    velocity_gradient += node.velocity * grad_w.transpose();
                }
            }
        }

        particle.velocity = velocity;
        particle.affine_b = affine_b;
        const Eigen::Matrix3d candidate_elastic_deformation_gradient =
            (identity + dt * velocity_gradient) * particle.elastic_deformation_gradient;
        const auto plastic_projection =
            project_drucker_prager(
                candidate_elastic_deformation_gradient,
                particle.alpha,
                config_.material);
        particle.elastic_deformation_gradient = plastic_projection.elastic_deformation_gradient;
        particle.hardening_state += plastic_projection.delta_q;
        particle.alpha = current_alpha(particle, config_.material);
        particle.position += dt * particle.velocity;
        for (const auto &collider : runtime_colliders) {
            project_particle_against_plane(collider, particle.position, particle.velocity);
        }
    }

    ++current_step_;
}

int Simulation::grid_index(int i, int j, int k) const {
    return (i * grid_ny_ + j) * grid_nz_ + k;
}

Eigen::Vector3i Simulation::unpack_grid_index(int flat_index) const {
    const int k = flat_index % grid_nz_;
    const int yz_index = flat_index / grid_nz_;
    const int j = yz_index % grid_ny_;
    const int i = yz_index / grid_ny_;
    return Eigen::Vector3i(i, j, k);
}

bool Simulation::inside_grid(int i, int j, int k) const {
    return i >= 0 && i < grid_nx_ && j >= 0 && j < grid_ny_ && k >= 0 && k < grid_nz_;
}

}  // namespace klar2016
