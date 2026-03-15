#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "klar2016/core/simulation.hpp"
#include "klar2016/core/simulation_config.hpp"

namespace {

std::filesystem::path parse_scene_path(int argc, char **argv) {
    std::filesystem::path scene = "scenes/hourglass.json";

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--scene" && i + 1 < argc) {
            scene = argv[++i];
        }
    }

    return scene;
}

int parse_preview_steps(int argc, char **argv, int fallback_steps) {
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--steps" && i + 1 < argc) {
            return std::stoi(argv[++i]);
        }
    }

    return fallback_steps;
}

std::optional<double> parse_friction_angle_override(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--friction-angle" && i + 1 < argc) {
            return std::stod(argv[++i]);
        }
    }

    return std::nullopt;
}

std::vector<double> parse_friction_angle_sweep(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--sweep-friction-angles" && i + 1 < argc) {
            std::vector<double> values;
            std::stringstream input(argv[++i]);
            std::string token;
            while (std::getline(input, token, ',')) {
                if (!token.empty()) {
                    values.push_back(std::stod(token));
                }
            }
            return values;
        }
    }

    return {};
}

bool parse_export_enabled(int argc, char **argv, bool fallback_enabled) {
    bool export_enabled = fallback_enabled;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--export") {
            export_enabled = true;
        }
        if (arg == "--no-export") {
            export_enabled = false;
        }
    }

    return export_enabled;
}

std::optional<std::filesystem::path> parse_output_directory(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--output-dir" && i + 1 < argc) {
            return std::filesystem::path(argv[++i]);
        }
    }

    return std::nullopt;
}

std::filesystem::path default_output_directory(const klar2016::SimulationConfig &config) {
    return std::filesystem::path(config.export_settings.output_directory) / config.scene.name;
}

std::filesystem::path frame_path(const std::filesystem::path &output_directory, int step) {
    std::ostringstream filename;
    filename << "frame_" << std::setw(5) << std::setfill('0') << step << ".ply";
    return output_directory / filename.str();
}

std::string angle_tag(double friction_angle_degrees) {
    std::ostringstream out;
    out << "friction_" << std::fixed << std::setprecision(1) << friction_angle_degrees;
    return out.str();
}

void run_preview(
    const klar2016::SimulationConfig &config,
    int preview_steps,
    bool export_ply,
    const std::filesystem::path &output_directory) {
    klar2016::Simulation simulation(config);

    std::cout << klar2016::describe(config) << '\n';
    std::cout << klar2016::describe("domain_min", config.scene.domain_min) << '\n';
    std::cout << klar2016::describe("domain_max", config.scene.domain_max) << '\n';
    std::cout << "running preview steps=" << preview_steps << '\n';

    const int export_interval = std::max(
        1,
        (config.export_settings.interval_steps > 0)
            ? config.export_settings.interval_steps
            : config.scene.frame_stride);

    if (export_ply) {
        if (config.export_settings.export_initial_frame) {
            simulation.write_particle_ply(frame_path(output_directory, simulation.current_step()));
        }

        for (int step = 0; step < preview_steps; ++step) {
            simulation.advance(1);
            if (
                simulation.current_step() % export_interval == 0 ||
                simulation.current_step() == preview_steps) {
                simulation.write_particle_ply(
                    frame_path(output_directory, simulation.current_step()));
            }
        }

        std::cout << "exported ply frames to " << output_directory << '\n';
    } else {
        simulation.advance(preview_steps);
    }

    std::cout << simulation.build_summary() << '\n';
}

}  // namespace

int main(int argc, char **argv) {
    try {
        const auto scene_path = parse_scene_path(argc, argv);
        auto config = klar2016::load_config(scene_path);
        const int preview_steps = parse_preview_steps(argc, argv, config.scene.preview_steps);
        const auto friction_angle_override = parse_friction_angle_override(argc, argv);
        const auto friction_angle_sweep = parse_friction_angle_sweep(argc, argv);
        const bool export_ply =
            parse_export_enabled(argc, argv, config.export_settings.export_ply);
        const auto base_output_directory =
            parse_output_directory(argc, argv).value_or(default_output_directory(config));

        if (!friction_angle_sweep.empty()) {
            for (double friction_angle_degrees : friction_angle_sweep) {
                auto variant = config;
                variant.material.friction_angle_degrees = friction_angle_degrees;
                const auto output_directory =
                    export_ply
                        ? (base_output_directory / angle_tag(friction_angle_degrees))
                        : base_output_directory;
                std::cout << "=== sweep friction_angle=" << friction_angle_degrees << " ===\n";
                run_preview(variant, preview_steps, export_ply, output_directory);
            }
            return 0;
        }

        if (friction_angle_override.has_value()) {
            config.material.friction_angle_degrees = *friction_angle_override;
        }

        run_preview(config, preview_steps, export_ply, base_output_directory);
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "[klar2016_sand] " << e.what() << '\n';
        return 1;
    }
}
