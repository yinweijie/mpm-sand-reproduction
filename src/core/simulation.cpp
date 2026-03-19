#include "klar2016/core/simulation.hpp"

#include <utility>

namespace klar2016 {

// Simulation lifecycle and top-level stepping.

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

}  // namespace klar2016
