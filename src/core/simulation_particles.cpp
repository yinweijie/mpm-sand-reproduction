#include "klar2016/core/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

#include "simulation_detail.hpp"

namespace klar2016 {

// Particle seeding and emitter logic.

void Simulation::initialize_particles() {
    if (config_.scene.seed_particles < 0) {
        throw std::runtime_error("scene.seed_particles must be non-negative");
    }

    if (
        !config_.scene.seed_from_particles_per_cell &&
        config_.scene.seed_particles == 0 &&
        !config_.emitter.enabled) {
        throw std::runtime_error("scene.seed_particles must be positive when emitter is disabled");
    }

    const Eigen::Vector3d seed_min(
        config_.scene.seed_box_min.x,
        config_.scene.seed_box_min.y,
        config_.scene.seed_box_min.z);
    const Eigen::Vector3d seed_max(
        config_.scene.seed_box_max.x,
        config_.scene.seed_box_max.y,
        config_.scene.seed_box_max.z);

    const Eigen::Vector3d extent = seed_max - seed_min;
    const double volume = extent.x() * extent.y() * extent.z();

    if (config_.scene.seed_from_particles_per_cell) {
        const int samples_per_axis =
            std::max(
                1,
                static_cast<int>(std::lround(
                    std::cbrt(static_cast<double>(config_.solver.particles_per_cell)))));
        if (
            samples_per_axis * samples_per_axis * samples_per_axis !=
            config_.solver.particles_per_cell) {
            throw std::runtime_error(
                "solver.particles_per_cell must be a perfect cube for regular ppc seeding");
        }

        const double spacing = config_.solver.dx / static_cast<double>(samples_per_axis);
        const double particle_volume =
            std::pow(config_.solver.dx, 3.0) /
            static_cast<double>(config_.solver.particles_per_cell);
        const double particle_mass = config_.material.density * particle_volume;
        const Eigen::Array3d counts =
            ((extent.array() / spacing) + 1.0e-9).floor().max(0.0);
        const int particle_count = static_cast<int>(counts.x() * counts.y() * counts.z());
        particles_.reserve(static_cast<std::size_t>(std::max(0, particle_count)));
        append_regular_seed_particles(
            seed_min,
            seed_max,
            spacing,
            config_.scene.seed_jitter,
            Eigen::Vector3d::Zero(),
            particle_mass,
            particle_volume);
    } else if (config_.scene.seed_particles > 0) {
        const double particle_volume = volume / static_cast<double>(config_.scene.seed_particles);
        const double particle_mass = config_.material.density * particle_volume;
        particles_.reserve(static_cast<std::size_t>(config_.scene.seed_particles));
        append_seed_particles(
            seed_min,
            seed_max,
            config_.scene.seed_particles,
            Eigen::Vector3d::Zero(),
            particle_mass,
            particle_volume);
    }
}

void Simulation::append_regular_seed_particles(
    const Eigen::Vector3d &box_min,
    const Eigen::Vector3d &box_max,
    double spacing,
    double jitter,
    const Eigen::Vector3d &initial_velocity,
    double particle_mass,
    double particle_volume) {
    if (spacing <= 0.0) {
        throw std::runtime_error("regular seed particle spacing must be positive");
    }

    if ((box_max.array() <= box_min.array()).any()) {
        throw std::runtime_error("particle source box must have positive extent");
    }

    const Eigen::Vector3d extent = box_max - box_min;
    const Eigen::Array3i counts =
        ((extent.array() / spacing) + 1.0e-9).floor().cast<int>().max(1);
    const double clamped_jitter = std::clamp(jitter, 0.0, 0.95);
    std::uniform_real_distribution<double> unit_dist(-0.5, 0.5);
    constexpr double kMarginFraction = 0.1;
    const double margin = kMarginFraction * spacing;

    for (int ix = 0; ix < counts.x(); ++ix) {
        for (int iy = 0; iy < counts.y(); ++iy) {
            for (int iz = 0; iz < counts.z(); ++iz) {
                Eigen::Vector3d position =
                    box_min +
                    spacing * Eigen::Vector3d(
                        static_cast<double>(ix) + 0.5,
                        static_cast<double>(iy) + 0.5,
                        static_cast<double>(iz) + 0.5);
                if (clamped_jitter > 0.0) {
                    const Eigen::Vector3d offset(
                        unit_dist(rng_),
                        unit_dist(rng_),
                        unit_dist(rng_));
                    position += clamped_jitter * spacing * offset;
                }
                const Eigen::Vector3d padded_min =
                    (box_min.array() + margin).matrix();
                const Eigen::Vector3d padded_max =
                    (box_max.array() - margin).matrix();
                position = position.cwiseMax(padded_min).cwiseMin(padded_max);

                Particle particle;
                particle.position = position;
                particle.velocity = initial_velocity;
                particle.mass = particle_mass;
                particle.initial_volume = particle_volume;
                particle.alpha = detail::current_alpha(particle, config_.material);
                particles_.push_back(particle);
            }
        }
    }
}

void Simulation::append_seed_particles(
    const Eigen::Vector3d &box_min,
    const Eigen::Vector3d &box_max,
    int particle_count,
    const Eigen::Vector3d &initial_velocity,
    double particle_mass,
    double particle_volume) {
    if (particle_count <= 0) {
        return;
    }

    if ((box_max.array() <= box_min.array()).any()) {
        throw std::runtime_error("particle source box must have positive extent");
    }

    std::uniform_real_distribution<double> dist_x(box_min.x(), box_max.x());
    std::uniform_real_distribution<double> dist_y(box_min.y(), box_max.y());
    std::uniform_real_distribution<double> dist_z(box_min.z(), box_max.z());

    for (int i = 0; i < particle_count; ++i) {
        Particle particle;
        particle.position = Eigen::Vector3d(dist_x(rng_), dist_y(rng_), dist_z(rng_));
        particle.velocity = initial_velocity;
        particle.mass = particle_mass;
        particle.initial_volume = particle_volume;
        particle.alpha = detail::current_alpha(particle, config_.material);
        particles_.push_back(particle);
    }
}

void Simulation::append_cylindrical_particles(
    const Eigen::Vector3d &center,
    double radius,
    double y_min,
    double y_max,
    int particle_count,
    const Eigen::Vector3d &initial_velocity,
    double particle_mass,
    double particle_volume) {
    if (particle_count <= 0) {
        return;
    }

    if (radius <= 0.0) {
        throw std::runtime_error("cylindrical particle source radius must be positive");
    }
    if (!(y_max > y_min)) {
        throw std::runtime_error("cylindrical particle source requires y_min < y_max");
    }

    std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * detail::kPi);
    std::uniform_real_distribution<double> radial_dist(0.0, 1.0);
    std::uniform_real_distribution<double> height_dist(y_min, y_max);

    for (int i = 0; i < particle_count; ++i) {
        const double angle = angle_dist(rng_);
        const double radial = radius * std::sqrt(radial_dist(rng_));
        Particle particle;
        particle.position = Eigen::Vector3d(
            center.x() + radial * std::cos(angle),
            height_dist(rng_),
            center.z() + radial * std::sin(angle));
        particle.velocity = initial_velocity;
        particle.mass = particle_mass;
        particle.initial_volume = particle_volume;
        particle.alpha = detail::current_alpha(particle, config_.material);
        particles_.push_back(particle);
    }
}

void Simulation::append_stratified_cylindrical_particles(
    const Eigen::Vector3d &center,
    double radius,
    double y_min,
    double y_max,
    int particle_count,
    double jitter,
    const Eigen::Vector3d &initial_velocity,
    double particle_mass,
    double particle_volume) {
    if (particle_count <= 0) {
        return;
    }

    if (radius <= 0.0) {
        throw std::runtime_error("cylindrical particle source radius must be positive");
    }
    if (!(y_max > y_min)) {
        throw std::runtime_error("cylindrical particle source requires y_min < y_max");
    }

    const double height = y_max - y_min;
    const double source_volume = detail::kPi * radius * radius * height;
    const double nominal_spacing =
        std::cbrt(source_volume / static_cast<double>(particle_count));
    const int y_layers = std::clamp(
        static_cast<int>(std::round(height / std::max(nominal_spacing, 1.0e-9))),
        1,
        particle_count);
    const double clamped_jitter = std::clamp(jitter, 0.0, 0.95);
    const double golden_angle = detail::kPi * (3.0 - std::sqrt(5.0));
    std::uniform_real_distribution<double> unit_dist(-0.5, 0.5);
    std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * detail::kPi);

    int generated_particles = 0;
    for (int layer = 0; layer < y_layers; ++layer) {
        const int layer_count =
            particle_count / y_layers + ((layer < (particle_count % y_layers)) ? 1 : 0);
        if (layer_count <= 0) {
            continue;
        }

        const double layer_phase = phase_dist(rng_) + golden_angle * static_cast<double>(layer);
        const double layer_fraction =
            (static_cast<double>(layer) + 0.5) / static_cast<double>(y_layers);
        const double base_y = y_min + layer_fraction * height;
        const double layer_spacing = radius / std::sqrt(static_cast<double>(layer_count));
        const double y_spacing = height / static_cast<double>(y_layers);

        for (int sample = 0; sample < layer_count; ++sample) {
            const double sample_fraction =
                (static_cast<double>(sample) + 0.5) / static_cast<double>(layer_count);
            double radial_fraction = std::sqrt(sample_fraction);
            double angle = layer_phase + golden_angle * static_cast<double>(sample);

            if (clamped_jitter > 0.0) {
                radial_fraction +=
                    clamped_jitter * 0.5 * (layer_spacing / std::max(radius, 1.0e-9)) *
                    unit_dist(rng_);
                angle +=
                    clamped_jitter *
                    (2.0 * detail::kPi / static_cast<double>(std::max(layer_count, 1))) *
                    unit_dist(rng_);
            }

            radial_fraction = std::clamp(radial_fraction, 0.0, 1.0);
            const double radial = radius * radial_fraction;
            double y = base_y;
            if (clamped_jitter > 0.0) {
                y += clamped_jitter * 0.35 * y_spacing * unit_dist(rng_);
            }
            y = std::clamp(y, y_min, y_max);

            Particle particle;
            particle.position = Eigen::Vector3d(
                center.x() + radial * std::cos(angle),
                y,
                center.z() + radial * std::sin(angle));
            particle.velocity = initial_velocity;
            particle.mass = particle_mass;
            particle.initial_volume = particle_volume;
            particle.alpha = detail::current_alpha(particle, config_.material);
            particles_.push_back(particle);
            ++generated_particles;
        }
    }

    if (generated_particles != particle_count) {
        throw std::runtime_error("stratified cylindrical emitter generated an unexpected count");
    }
}

void Simulation::emit_particles() {
    if (!config_.emitter.enabled || config_.emitter.particles_per_step <= 0) {
        return;
    }

    if (current_step_ < config_.emitter.start_step) {
        return;
    }

    if (config_.emitter.stop_step >= 0 && current_step_ > config_.emitter.stop_step) {
        return;
    }

    int particle_count = config_.emitter.particles_per_step;
    if (config_.emitter.max_particles > 0) {
        particle_count = std::min(
            particle_count,
            config_.emitter.max_particles - emitted_particles_);
    }

    if (particle_count <= 0) {
        return;
    }

    const Eigen::Vector3d emitter_min(
        config_.emitter.box_min.x,
        config_.emitter.box_min.y,
        config_.emitter.box_min.z);
    const Eigen::Vector3d emitter_max(
        config_.emitter.box_max.x,
        config_.emitter.box_max.y,
        config_.emitter.box_max.z);
    const Eigen::Vector3d initial_velocity(
        config_.emitter.initial_velocity.x,
        config_.emitter.initial_velocity.y,
        config_.emitter.initial_velocity.z);
    const Eigen::Vector3d emitter_extent = emitter_max - emitter_min;
    const double emitter_height = emitter_extent.y();
    double emitter_volume =
        emitter_extent.x() * emitter_extent.y() * emitter_extent.z();
    if (config_.emitter.shape == "cylinder") {
        if (config_.emitter.radius <= 0.0) {
            throw std::runtime_error("emitter.radius must be positive for cylinder emitters");
        }
        emitter_volume =
            detail::kPi * config_.emitter.radius * config_.emitter.radius * emitter_height;
    } else if (config_.emitter.shape != "box") {
        throw std::runtime_error("emitter.shape must be either 'box' or 'cylinder'");
    }
    const double particle_volume = emitter_volume / static_cast<double>(particle_count);
    const double particle_mass = config_.material.density * particle_volume;

    if (config_.emitter.shape == "cylinder") {
        const Eigen::Vector3d center(
            0.5 * (emitter_min.x() + emitter_max.x()),
            0.0,
            0.5 * (emitter_min.z() + emitter_max.z()));
        if (config_.emitter.sampling_mode == "stratified") {
            append_stratified_cylindrical_particles(
                center,
                config_.emitter.radius,
                emitter_min.y(),
                emitter_max.y(),
                particle_count,
                config_.emitter.jitter,
                initial_velocity,
                particle_mass,
                particle_volume);
        } else if (config_.emitter.sampling_mode == "random") {
            append_cylindrical_particles(
                center,
                config_.emitter.radius,
                emitter_min.y(),
                emitter_max.y(),
                particle_count,
                initial_velocity,
                particle_mass,
                particle_volume);
        } else {
            throw std::runtime_error(
                "emitter.sampling_mode must be either 'random' or 'stratified'");
        }
    } else {
        append_seed_particles(
            emitter_min,
            emitter_max,
            particle_count,
            initial_velocity,
            particle_mass,
            particle_volume);
    }
    emitted_particles_ += particle_count;
}

}  // namespace klar2016
