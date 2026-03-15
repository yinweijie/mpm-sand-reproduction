#include <cmath>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include "klar2016/core/kernel.hpp"
#include "klar2016/core/simulation.hpp"
#include "klar2016/core/simulation_config.hpp"

namespace {

std::filesystem::path source_root() {
    return std::filesystem::path(KLAR2016_SOURCE_DIR);
}

}  // namespace

TEST(SimulationConfigTest, LoadsHourglassScene) {
    const auto config = klar2016::load_config(source_root() / "scenes/hourglass.json");

    EXPECT_EQ(config.scene.name, "hourglass");
    EXPECT_DOUBLE_EQ(config.solver.dx, 0.0025);
    EXPECT_DOUBLE_EQ(config.solver.dt_max, 0.0002);
    EXPECT_EQ(config.solver.particles_per_cell, 8);
    EXPECT_EQ(config.scene.seed_particles, 0);
    EXPECT_TRUE(config.scene.seed_from_particles_per_cell);
    EXPECT_NEAR(config.scene.seed_jitter, 0.15, 1e-12);
    EXPECT_EQ(config.scene.preview_steps, 3000);
    EXPECT_TRUE(config.scene.hourglass_shell.enabled);
    EXPECT_TRUE(config.scene.hourglass_shell.add_bottom_cap);
    EXPECT_DOUBLE_EQ(config.scene.hourglass_shell.neck_half_width, 0.018);
    ASSERT_TRUE(config.scene.colliders.empty());
    EXPECT_EQ(config.export_settings.interval_steps, 32);
    EXPECT_TRUE(config.solver.explicit_only);
}

TEST(SimulationConfigTest, LoadsPileEmitterScene) {
    const auto config = klar2016::load_config(source_root() / "scenes/pile_from_spout.json");

    EXPECT_EQ(config.scene.name, "pile_from_spout");
    EXPECT_EQ(config.scene.seed_particles, 0);
    EXPECT_TRUE(config.emitter.enabled);
    EXPECT_EQ(config.emitter.particles_per_step, 64);
    EXPECT_TRUE(config.emitter.max_particles > 0);
}

TEST(SimulationConfigTest, LoadsPileLabScene) {
    const auto config = klar2016::load_config(source_root() / "scenes/pile_lab.json");

    EXPECT_EQ(config.scene.name, "pile_lab");
    EXPECT_EQ(config.scene.seed_particles, 0);
    EXPECT_EQ(config.scene.colliders.size(), 1);
    EXPECT_EQ(config.scene.cylinder_shells.size(), 1);
    EXPECT_DOUBLE_EQ(config.scene.domain_boundary_friction_coefficient, 0.85);
    EXPECT_TRUE(config.emitter.enabled);
    EXPECT_EQ(config.emitter.shape, "cylinder");
    EXPECT_EQ(config.emitter.sampling_mode, "stratified");
    EXPECT_DOUBLE_EQ(config.emitter.radius, 0.0045);
    EXPECT_DOUBLE_EQ(config.emitter.jitter, 0.35);
    EXPECT_EQ(config.emitter.particles_per_step, 192);
    EXPECT_EQ(config.emitter.max_particles, 38400);
    EXPECT_EQ(config.scene.preview_steps, 2400);
    EXPECT_DOUBLE_EQ(config.solver.dx, 0.0025);
    EXPECT_DOUBLE_EQ(config.material.friction_angle_degrees, 38.0);
}

TEST(SimulationConfigTest, LoadsBoxCaseScene) {
    const auto config = klar2016::load_config(source_root() / "scenes/box_case.json");

    EXPECT_EQ(config.scene.name, "box_case");
    EXPECT_EQ(config.scene.seed_particles, 12000);
    EXPECT_DOUBLE_EQ(config.scene.domain_boundary_friction_coefficient, 0.55);
    EXPECT_EQ(config.scene.preview_steps, 400);
    EXPECT_EQ(config.solver.particles_per_cell, 16);
    EXPECT_EQ(config.export_settings.interval_steps, 4);
}

TEST(SimulationConfigTest, LoadsSceneColliders) {
    const auto temp_path =
        std::filesystem::temp_directory_path() / "klar2016_scene_with_colliders.json";
    std::ofstream output(temp_path);
    output
        << "{\n"
        << "  \"scene\": {\n"
        << "    \"name\": \"collider_test\",\n"
        << "    \"domain_min\": [0.0, 0.0, 0.0],\n"
        << "    \"domain_max\": [1.0, 1.0, 1.0],\n"
        << "    \"gravity\": [0.0, -9.81, 0.0],\n"
        << "    \"domain_boundary_friction_coefficient\": 0.4,\n"
        << "    \"seed_box_min\": [0.2, 0.2, 0.2],\n"
        << "    \"seed_box_max\": [0.3, 0.3, 0.3],\n"
        << "    \"seed_particles\": 8,\n"
        << "    \"duration\": 1.0,\n"
        << "    \"frame_stride\": 1,\n"
        << "    \"preview_steps\": 1,\n"
        << "    \"colliders\": [\n"
        << "      {\n"
        << "        \"point\": [0.0, 0.0, 0.0],\n"
        << "        \"normal\": [0.0, 1.0, 0.0],\n"
        << "        \"friction_coefficient\": 0.6,\n"
        << "        \"bounds_min\": [0.0, 0.0, 0.0],\n"
        << "        \"bounds_max\": [0.5, 0.1, 0.5]\n"
        << "      }\n"
        << "    ]\n"
        << "  },\n"
        << "  \"material\": {\n"
        << "    \"density\": 2200.0,\n"
        << "    \"youngs_modulus\": 353700.0,\n"
        << "    \"poisson_ratio\": 0.3,\n"
        << "    \"friction_angle_degrees\": 30.0,\n"
        << "    \"hardening_h0\": 35.0,\n"
        << "    \"hardening_h1\": 0.0,\n"
        << "    \"hardening_h2\": 0.0,\n"
        << "    \"hardening_h3\": 0.0\n"
        << "  },\n"
        << "  \"solver\": {\n"
        << "    \"dx\": 0.05,\n"
        << "    \"dt_max\": 0.0001,\n"
        << "    \"cfl\": 1.0,\n"
        << "    \"particles_per_cell\": 8,\n"
        << "    \"explicit_only\": true\n"
        << "  }\n"
        << "}\n";
    output.close();

    const auto config = klar2016::load_config(temp_path);
    std::filesystem::remove(temp_path);

    EXPECT_DOUBLE_EQ(config.scene.domain_boundary_friction_coefficient, 0.4);
    ASSERT_EQ(config.scene.colliders.size(), 1);
    EXPECT_DOUBLE_EQ(config.scene.colliders.front().friction_coefficient, 0.6);
    EXPECT_DOUBLE_EQ(config.scene.colliders.front().normal.y, 1.0);
    EXPECT_TRUE(config.scene.colliders.front().bounded);
    EXPECT_DOUBLE_EQ(config.scene.colliders.front().bounds_max.z, 0.5);
}

TEST(KernelTest, CubicWeightsSumToOne) {
    const auto stencil = klar2016::build_cubic_axis_stencil(7.35, 100.0);
    double sum = 0.0;
    for (double weight : stencil.weights) {
        sum += weight;
    }

    EXPECT_NEAR(sum, 1.0, 1e-12);
}

TEST(KernelTest, CubicGradientsSumToZero) {
    const auto stencil = klar2016::build_cubic_axis_stencil(4.2, 50.0);
    double sum = 0.0;
    for (double gradient : stencil.gradients) {
        sum += gradient;
    }

    EXPECT_NEAR(sum, 0.0, 1e-12);
}

TEST(SimulationTest, EstimatesPositiveGridCellCount) {
    const auto config = klar2016::load_config(source_root() / "scenes/pile_from_spout.json");
    const klar2016::Simulation simulation(config);

    EXPECT_GT(simulation.estimated_grid_cells(), 0);
}

TEST(SimulationTest, PreviewAdvanceKeepsParticlesAlive) {
    const auto config = klar2016::load_config(source_root() / "scenes/box_case.json");
    klar2016::Simulation simulation(config);

    simulation.advance(2);
    const auto stats = simulation.stats();

    EXPECT_EQ(stats.particle_count, config.scene.seed_particles);
    EXPECT_GT(stats.active_grid_nodes, 0);
    EXPECT_GT(stats.total_mass, 0.0);
    EXPECT_GT(stats.average_speed, 0.0);
    EXPECT_GT(stats.average_det_fe, 0.0);
    EXPECT_GT(stats.average_alpha, 0.0);
    EXPECT_GT(stats.particle_bbox_volume, 0.0);
    EXPECT_GT(stats.particle_height_span, 0.0);
    EXPECT_TRUE(std::isfinite(stats.average_speed));
    EXPECT_TRUE(std::isfinite(stats.average_det_fe));
    EXPECT_TRUE(std::isfinite(stats.average_alpha));
    EXPECT_TRUE(std::isfinite(stats.particle_bbox_volume));
    EXPECT_TRUE(std::isfinite(stats.particle_height_span));
}

TEST(SimulationTest, EmitterAddsParticlesDuringAdvance) {
    const auto config = klar2016::load_config(source_root() / "scenes/pile_from_spout.json");
    klar2016::Simulation simulation(config);

    EXPECT_EQ(simulation.stats().particle_count, 0);

    simulation.advance(2);
    const auto stats = simulation.stats();

    EXPECT_GT(stats.particle_count, 0);
    EXPECT_GT(stats.active_grid_nodes, 0);
    EXPECT_GT(stats.total_mass, 0.0);
    EXPECT_TRUE(std::isfinite(stats.average_speed));
    EXPECT_TRUE(std::isfinite(stats.particle_bbox_volume));
}
