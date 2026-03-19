#pragma once

#include <vector>

#include <Eigen/Dense>

#include "klar2016/core/simulation.hpp"

namespace klar2016::detail {

inline constexpr double kMinSingularValue = 1.0e-6;
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kContactSlop = 5.0e-4;

// Normalized runtime collider used by the solver step.
struct RuntimePlaneCollider {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitY();
    double friction_coefficient = 0.0;
    bool bounded = false;
    Eigen::Vector3d bounds_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d bounds_max = Eigen::Vector3d::Zero();
};

// Plasticity update result for one particle.
struct PlasticProjectionResult {
    Eigen::Matrix3d elastic_deformation_gradient = Eigen::Matrix3d::Identity();
    double delta_q = 0.0;
};

// Lame coefficients derived from the material settings.
struct LameParameters {
    double mu = 0.0;
    double lambda = 0.0;
};

// Scene and collider conversion helpers.
Eigen::Vector3d to_eigen(const Vec3d &value);
Vec3d to_vec3(const Eigen::Vector3d &value);
PlaneColliderConfig make_plane_collider_config(
    const Eigen::Vector3d &point,
    const Eigen::Vector3d &normal,
    double friction_coefficient,
    const Eigen::Vector3d &bounds_min,
    const Eigen::Vector3d &bounds_max);
RuntimePlaneCollider make_runtime_plane_collider(const PlaneColliderConfig &collider);
void append_hourglass_shell_colliders(
    const HourglassShellConfig &shell,
    std::vector<RuntimePlaneCollider> &runtime_colliders);
void append_cylinder_shell_colliders(
    const CylinderShellConfig &shell,
    std::vector<RuntimePlaneCollider> &runtime_colliders);
bool plane_collider_contains_projection(
    const RuntimePlaneCollider &collider,
    const Eigen::Vector3d &projection);
void apply_unilateral_boundary_response(
    const Eigen::Vector3d &normal,
    double friction_coefficient,
    Eigen::Vector3d &velocity);
void apply_plane_collider_velocity(
    const RuntimePlaneCollider &collider,
    const Eigen::Vector3d &position,
    Eigen::Vector3d &velocity);
void project_particle_against_plane(
    const RuntimePlaneCollider &collider,
    Eigen::Vector3d &position,
    Eigen::Vector3d &velocity);

// Material and plasticity helpers.
LameParameters compute_lame_parameters(const MaterialConfig &material);
double friction_angle_from_hardening_state(
    double hardening_state,
    const MaterialConfig &material);
double alpha_from_friction_angle_degrees(double friction_angle_degrees);
double current_alpha(const Simulation::Particle &particle, const MaterialConfig &material);
Eigen::Matrix3d elastic_energy_derivative(
    const Eigen::Matrix3d &elastic_deformation_gradient,
    const MaterialConfig &material);
PlasticProjectionResult project_drucker_prager(
    const Eigen::Matrix3d &candidate_elastic_deformation_gradient,
    double alpha,
    const MaterialConfig &material);

}  // namespace klar2016::detail
