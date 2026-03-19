#pragma once

#include <vector>

#include <Eigen/Dense>

#include "klar2016/core/simulation.hpp"

namespace klar2016::detail {

/// @brief Small positive clamp used to keep SVD-based strain computations finite.
inline constexpr double kMinSingularValue = 1.0e-6;
/// @brief Shared value of pi for geometry and sampling helpers.
inline constexpr double kPi = 3.14159265358979323846;
/// @brief Penetration tolerance used before collider response is applied.
inline constexpr double kContactSlop = 5.0e-4;

/// @brief Normalized collider representation consumed directly by the solver.
struct RuntimePlaneCollider {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitY();
    double friction_coefficient = 0.0;
    bool bounded = false;
    Eigen::Vector3d bounds_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d bounds_max = Eigen::Vector3d::Zero();
};

/// @brief Stores the plastic projection result for a single particle update.
struct PlasticProjectionResult {
    Eigen::Matrix3d elastic_deformation_gradient = Eigen::Matrix3d::Identity();
    double delta_q = 0.0;
};

/// @brief Stores the Lame coefficients derived from Young's modulus and Poisson ratio.
struct LameParameters {
    double mu = 0.0;
    double lambda = 0.0;
};

/// @brief Converts a config-space vector to Eigen for numeric kernels.
Eigen::Vector3d to_eigen(const Vec3d &value);
/// @brief Converts an Eigen vector back into config-space storage.
Vec3d to_vec3(const Eigen::Vector3d &value);
/// @brief Builds a bounded plane collider config from runtime geometry values.
PlaneColliderConfig make_plane_collider_config(
    const Eigen::Vector3d &point,
    const Eigen::Vector3d &normal,
    double friction_coefficient,
    const Eigen::Vector3d &bounds_min,
    const Eigen::Vector3d &bounds_max);
/// @brief Validates and normalizes a plane collider for runtime use.
RuntimePlaneCollider make_runtime_plane_collider(const PlaneColliderConfig &collider);
/// @brief Expands an hourglass shell description into a set of plane colliders.
void append_hourglass_shell_colliders(
    const HourglassShellConfig &shell,
    std::vector<RuntimePlaneCollider> &runtime_colliders);
/// @brief Expands a cylindrical shell description into a ring of plane colliders.
void append_cylinder_shell_colliders(
    const CylinderShellConfig &shell,
    std::vector<RuntimePlaneCollider> &runtime_colliders);
/// @brief Tests whether a point projected onto a collider lies within its active bounds.
bool plane_collider_contains_projection(
    const RuntimePlaneCollider &collider,
    const Eigen::Vector3d &projection);
/// @brief Removes inward normal velocity and applies Coulomb-style friction.
void apply_unilateral_boundary_response(
    const Eigen::Vector3d &normal,
    double friction_coefficient,
    Eigen::Vector3d &velocity);
/// @brief Applies plane-collider velocity constraints to a grid node or particle velocity.
void apply_plane_collider_velocity(
    const RuntimePlaneCollider &collider,
    const Eigen::Vector3d &position,
    Eigen::Vector3d &velocity);
/// @brief Projects a particle out of penetration and updates its velocity accordingly.
void project_particle_against_plane(
    const RuntimePlaneCollider &collider,
    Eigen::Vector3d &position,
    Eigen::Vector3d &velocity);

/// @brief Computes Lame parameters from the material configuration.
LameParameters compute_lame_parameters(const MaterialConfig &material);
/// @brief Evaluates the hardening-adjusted friction angle for a particle state.
double friction_angle_from_hardening_state(
    double hardening_state,
    const MaterialConfig &material);
/// @brief Converts a friction angle in degrees to the Drucker-Prager alpha parameter.
double alpha_from_friction_angle_degrees(double friction_angle_degrees);
/// @brief Returns the current particle alpha after accounting for hardening.
double current_alpha(const Simulation::Particle &particle, const MaterialConfig &material);
/// @brief Computes the derivative of elastic energy with respect to `F_e`.
Eigen::Matrix3d elastic_energy_derivative(
    const Eigen::Matrix3d &elastic_deformation_gradient,
    const MaterialConfig &material);
/// @brief Projects a trial elastic deformation gradient back onto the yield surface.
PlasticProjectionResult project_drucker_prager(
    const Eigen::Matrix3d &candidate_elastic_deformation_gradient,
    double alpha,
    const MaterialConfig &material);

}  // namespace klar2016::detail
