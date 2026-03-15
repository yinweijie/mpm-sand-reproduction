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

class Simulation {
public:
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

    struct GridNode {
        double mass = 0.0;
        Eigen::Vector3d momentum = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    };

    explicit Simulation(SimulationConfig config);

    void advance(int steps);
    std::string build_summary() const;
    int estimated_grid_cells() const;
    SimulationStats stats() const;
    int current_step() const;
    void write_particle_ply(const std::filesystem::path &path) const;

private:
    void initialize_particles();
    void append_seed_particles(
        const Eigen::Vector3d &box_min,
        const Eigen::Vector3d &box_max,
        int particle_count,
        const Eigen::Vector3d &initial_velocity,
        double particle_mass,
        double particle_volume);
    void append_cylindrical_particles(
        const Eigen::Vector3d &center,
        double radius,
        double y_min,
        double y_max,
        int particle_count,
        const Eigen::Vector3d &initial_velocity,
        double particle_mass,
        double particle_volume);
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
    void append_regular_seed_particles(
        const Eigen::Vector3d &box_min,
        const Eigen::Vector3d &box_max,
        double spacing,
        double jitter,
        const Eigen::Vector3d &initial_velocity,
        double particle_mass,
        double particle_volume);
    void emit_particles();
    void reset_grid();
    void substep();
    int grid_index(int i, int j, int k) const;
    Eigen::Vector3i unpack_grid_index(int flat_index) const;
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
