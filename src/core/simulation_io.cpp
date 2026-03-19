#include "klar2016/core/simulation.hpp"

#include <fstream>
#include <iomanip>
#include <limits>

#include <fmt/format.h>

namespace klar2016 {

// Solver observability and export helpers.

// Export the current particle state as an ASCII PLY point cloud.
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

// Aggregate particle and grid statistics for logs, tests, and regressions.
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

// Format the one-line summary used by preview runs and regression baselines.
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

}  // namespace klar2016
