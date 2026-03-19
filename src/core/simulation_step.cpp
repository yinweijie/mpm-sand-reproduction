#include "klar2016/core/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "klar2016/core/kernel.hpp"

#include "simulation_detail.hpp"

namespace klar2016 {

// Grid indexing helpers.

// Clear the sparse grid before scattering particle state into it again.
void Simulation::reset_grid() {
    grid_.clear();
}

// Convert 3D grid coordinates to the flattened sparse-grid key.
int Simulation::grid_index(int i, int j, int k) const {
    return (i * grid_ny_ + j) * grid_nz_ + k;
}

// Recover 3D grid coordinates from a flattened sparse-grid key.
Eigen::Vector3i Simulation::unpack_grid_index(int flat_index) const {
    const int k = flat_index % grid_nz_;
    const int yz_index = flat_index / grid_nz_;
    const int j = yz_index % grid_ny_;
    const int i = yz_index / grid_ny_;
    return Eigen::Vector3i(i, j, k);
}

// Check whether a grid coordinate lies inside the allocated domain grid.
bool Simulation::inside_grid(int i, int j, int k) const {
    return i >= 0 && i < grid_nx_ && j >= 0 && j < grid_ny_ && k >= 0 && k < grid_nz_;
}

// Execute one full solver substep: emit, P2G, grid update, G2P, and collisions.

void Simulation::substep() {
    emit_particles();
    reset_grid();

    const Eigen::Vector3d domain_min = detail::to_eigen(config_.scene.domain_min);
    const Eigen::Vector3d domain_max = detail::to_eigen(config_.scene.domain_max);
    const Eigen::Vector3d gravity = detail::to_eigen(config_.scene.gravity);
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    const double inv_dx = 1.0 / config_.solver.dx;
    const double dt = config_.solver.dt_max;
    const double cubic_d_inv = 3.0 * inv_dx * inv_dx;
    std::vector<detail::RuntimePlaneCollider> runtime_colliders;
    std::size_t cylinder_segment_count = 0;
    for (const auto &shell : config_.scene.cylinder_shells) {
        cylinder_segment_count += static_cast<std::size_t>(std::max(0, shell.segments));
    }
    runtime_colliders.reserve(6 + config_.scene.colliders.size() + 9 + cylinder_segment_count);

    const double boundary_friction = std::max(0.0, config_.scene.domain_boundary_friction_coefficient);
    runtime_colliders.push_back(detail::RuntimePlaneCollider{
        Eigen::Vector3d(domain_min.x(), domain_min.y(), domain_min.z()),
        Eigen::Vector3d::UnitX(),
        boundary_friction});
    runtime_colliders.push_back(detail::RuntimePlaneCollider{
        Eigen::Vector3d(domain_max.x(), domain_min.y(), domain_min.z()),
        -Eigen::Vector3d::UnitX(),
        boundary_friction});
    runtime_colliders.push_back(detail::RuntimePlaneCollider{
        Eigen::Vector3d(domain_min.x(), domain_min.y(), domain_min.z()),
        Eigen::Vector3d::UnitY(),
        boundary_friction});
    runtime_colliders.push_back(detail::RuntimePlaneCollider{
        Eigen::Vector3d(domain_min.x(), domain_max.y(), domain_min.z()),
        -Eigen::Vector3d::UnitY(),
        boundary_friction});
    runtime_colliders.push_back(detail::RuntimePlaneCollider{
        Eigen::Vector3d(domain_min.x(), domain_min.y(), domain_min.z()),
        Eigen::Vector3d::UnitZ(),
        boundary_friction});
    runtime_colliders.push_back(detail::RuntimePlaneCollider{
        Eigen::Vector3d(domain_min.x(), domain_min.y(), domain_max.z()),
        -Eigen::Vector3d::UnitZ(),
        boundary_friction});

    for (const auto &collider : config_.scene.colliders) {
        runtime_colliders.push_back(detail::make_runtime_plane_collider(collider));
    }
    detail::append_hourglass_shell_colliders(config_.scene.hourglass_shell, runtime_colliders);
    for (const auto &shell : config_.scene.cylinder_shells) {
        detail::append_cylinder_shell_colliders(shell, runtime_colliders);
    }

    // P2G: scatter particle mass and momentum to the grid.
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

    // Internal force contribution from elastic stress.
    for (const auto &particle : particles_) {
        const Eigen::Vector3d position_in_grid = (particle.position - domain_min) * inv_dx;
        const auto sx = build_cubic_axis_stencil(position_in_grid.x(), inv_dx);
        const auto sy = build_cubic_axis_stencil(position_in_grid.y(), inv_dx);
        const auto sz = build_cubic_axis_stencil(position_in_grid.z(), inv_dx);
        const Eigen::Matrix3d elastic_gradient =
            detail::elastic_energy_derivative(
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

    // Grid update: apply gravity and collider response.
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
            detail::apply_plane_collider_velocity(collider, node_position, node.velocity);
        }
    }

    // G2P: gather updated velocity and deformation back to particles.
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
            detail::project_drucker_prager(
                candidate_elastic_deformation_gradient,
                particle.alpha,
                config_.material);
        particle.elastic_deformation_gradient = plastic_projection.elastic_deformation_gradient;
        particle.hardening_state += plastic_projection.delta_q;
        particle.alpha = detail::current_alpha(particle, config_.material);
        particle.position += dt * particle.velocity;
        for (const auto &collider : runtime_colliders) {
            detail::project_particle_against_plane(collider, particle.position, particle.velocity);
        }
    }

    ++current_step_;
}

}  // namespace klar2016
