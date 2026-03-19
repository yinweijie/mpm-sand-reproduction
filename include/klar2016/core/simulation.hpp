#pragma once

#include <cstddef>
#include <filesystem>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "klar2016/core/simulation_config.hpp"

namespace klar2016 {

/// @brief Aggregates solver statistics used by logs, tests, and baseline manifests.
struct SimulationStats {
    int particle_count = 0;
    int active_grid_nodes = 0;
    double total_mass = 0.0;
    double average_speed = 0.0;
    double average_det_fe = 1.0;
    double average_alpha = 0.0;
    double particle_bbox_volume = 0.0;
    double particle_height_span = 0.0;
};

/// @brief Owns the full MPM sand simulation state and advances it step by step.
class Simulation {
public:
    /// @brief Stores the mutable runtime state for one simulated sand particle.
    struct Particle {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        Eigen::Matrix3d affine_b = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d elastic_deformation_gradient = Eigen::Matrix3d::Identity();
        double mass = 1.0;
        double initial_volume = 1.0;
        double hardening_state = 0.0;
        double alpha = 0.0;
    };

    /// @brief Stores sparse grid mass, momentum, and velocity during one MPM step.
    struct GridNode {
        double mass = 0.0;
        Eigen::Vector3d momentum = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    };

    /// @brief Builds the simulation, sizes the grid, and seeds the initial particle set.
    explicit Simulation(SimulationConfig config);

    /// @brief Advances the simulation by the requested number of substeps.
    void advance(int steps);
    /// @brief Formats the summary line used by logs and regression baselines.
    std::string build_summary() const;
    /// @brief Estimates the number of grid cells implied by the scene bounds and `dx`.
    int estimated_grid_cells() const;
    /// @brief Computes aggregate runtime statistics over the current particle set.
    SimulationStats stats() const;
    /// @brief Returns the current substep index since the simulation started.
    int current_step() const;
    /// @brief Exports the current particles as an ASCII PLY file.
    void write_particle_ply(const std::filesystem::path &path) const;

private:
    /// @brief Seeds the initial particle set from the scene configuration.
    void initialize_particles();
    /// @brief Appends randomly sampled box particles to the simulation state.
    void append_seed_particles(
        const Eigen::Vector3d &box_min,
        const Eigen::Vector3d &box_max,
        int particle_count,
        const Eigen::Vector3d &initial_velocity,
        double particle_mass,
        double particle_volume);
    /// @brief Appends randomly sampled cylindrical particles to the simulation state.
    void append_cylindrical_particles(
        const Eigen::Vector3d &center,
        double radius,
        double y_min,
        double y_max,
        int particle_count,
        const Eigen::Vector3d &initial_velocity,
        double particle_mass,
        double particle_volume);
    /// @brief Appends layered cylindrical particles with controllable jitter.
    void append_stratified_cylindrical_particles(
        const Eigen::Vector3d &center,
        double radius,
        double y_min,
        double y_max,
        int particle_count,
        double jitter,
        const Eigen::Vector3d &initial_velocity,
        double particle_mass,
        double particle_volume);
    /// @brief Appends regularly spaced box particles for PPC-based seeding.
    void append_regular_seed_particles(
        const Eigen::Vector3d &box_min,
        const Eigen::Vector3d &box_max,
        double spacing,
        double jitter,
        const Eigen::Vector3d &initial_velocity,
        double particle_mass,
        double particle_volume);
    /// @brief Emits additional particles for scenes with a runtime emitter.
    void emit_particles();
    /// @brief Clears the sparse grid before the next P2G phase.
    void reset_grid();
    /// @brief Executes one full MPM substep.
    void substep();
    /// @brief Packs a 3D grid coordinate into the sparse-grid hash key.
    int grid_index(int i, int j, int k) const;
    /// @brief Unpacks a sparse-grid hash key back into a 3D grid coordinate.
    Eigen::Vector3i unpack_grid_index(int flat_index) const;
    /// @brief Returns whether a grid coordinate lies inside the simulation domain grid.
    bool inside_grid(int i, int j, int k) const;

    SimulationConfig config_;
    std::vector<Particle> particles_;
    std::unordered_map<int, GridNode> grid_;
    int grid_nx_ = 0;
    int grid_ny_ = 0;
    int grid_nz_ = 0;
    int current_step_ = 0;
    int emitted_particles_ = 0;
    std::mt19937 rng_{42};
};

}  // namespace klar2016
