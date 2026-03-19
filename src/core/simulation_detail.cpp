#include "simulation_detail.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace klar2016::detail {

// Geometry and collider conversion helpers.

// Convert a lightweight config vector into the Eigen representation used by the solver.
Eigen::Vector3d to_eigen(const Vec3d &value) {
    return Eigen::Vector3d(value.x, value.y, value.z);
}

// Convert an Eigen vector back to the lightweight config representation.
Vec3d to_vec3(const Eigen::Vector3d &value) {
    return Vec3d{value.x(), value.y(), value.z()};
}

// Package runtime plane data into a bounded collider config object.
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

// Normalize, validate, and materialize a plane collider for runtime use.
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
        if ((runtime_collider.bounds_max.array() <= runtime_collider.bounds_min.array()).any()) {
            throw std::runtime_error("collider bounds must have positive extent");
        }
    }
    return runtime_collider;
}

// Expand the procedural hourglass shell into the plane colliders used by the solver.
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
        throw std::runtime_error("hourglass_shell.top_half_width must exceed neck_half_width");
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

// Expand a cylindrical shell into a ring of plane colliders.
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

// Check whether a projected point lies inside a collider's bounded active region.
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

// Remove inward normal motion and apply frictional tangential damping.
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

// Apply plane-collider velocity constraints to a point moving near the collider.
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

// Push a particle back to the collider surface and damp its boundary-relative motion.
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

// Material and plasticity helpers.

// Derive the isotropic Lame parameters from the configured elastic constants.
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

// Evaluate the friction angle implied by the current hardening state.
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

// Convert a friction angle in degrees to the Drucker-Prager alpha parameter.
double alpha_from_friction_angle_degrees(double friction_angle_degrees) {
    const double clamped_degrees = std::clamp(friction_angle_degrees, 0.0, 89.0);
    const double phi = clamped_degrees * kPi / 180.0;
    return
        std::sqrt(2.0 / 3.0) *
        (2.0 * std::sin(phi)) /
        (3.0 - std::sin(phi));
}

// Read the current particle alpha, including hardening when it is enabled.
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

// Evaluate the derivative of elastic energy with respect to the elastic deformation gradient.
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

// Project a trial elastic deformation gradient back onto the Drucker-Prager admissible set.
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

}  // namespace klar2016::detail
