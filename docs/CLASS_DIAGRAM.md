# UML Class Diagram

This diagram summarizes the refactored solver-facing types and the main ownership
relationships between configuration, runtime state, and internal helper data.

```mermaid
classDiagram
direction LR

class SimulationStats {
  +int particle_count
  +int active_grid_nodes
  +double total_mass
  +double average_speed
  +double average_det_fe
  +double average_alpha
  +double particle_bbox_volume
  +double particle_height_span
}

class Simulation {
  +Simulation(SimulationConfig config)
  +void advance(int steps)
  +std::string build_summary() const
  +int estimated_grid_cells() const
  +SimulationStats stats() const
  +int current_step() const
  +void write_particle_ply(const std::filesystem::path &path) const
  -void initialize_particles()
  -void append_seed_particles(...)
  -void append_cylindrical_particles(...)
  -void append_stratified_cylindrical_particles(...)
  -void append_regular_seed_particles(...)
  -void emit_particles()
  -void reset_grid()
  -void substep()
  -int grid_index(int i, int j, int k) const
  -Eigen::Vector3i unpack_grid_index(int flat_index) const
  -bool inside_grid(int i, int j, int k) const
}

class SimulationParticle {
  +Eigen::Vector3d position
  +Eigen::Vector3d velocity
  +Eigen::Matrix3d affine_b
  +Eigen::Matrix3d elastic_deformation_gradient
  +double mass
  +double initial_volume
  +double hardening_state
  +double alpha
}

class SimulationGridNode {
  +double mass
  +Eigen::Vector3d momentum
  +Eigen::Vector3d velocity
}

class SimulationConfig {
  +SceneConfig scene
  +MaterialConfig material
  +SolverConfig solver
  +EmitterConfig emitter
  +ExportConfig export_settings
}

class SceneConfig {
  +std::string name
  +Vec3d domain_min
  +Vec3d domain_max
  +Vec3d gravity
  +double domain_boundary_friction_coefficient
  +Vec3d seed_box_min
  +Vec3d seed_box_max
  +bool seed_from_particles_per_cell
  +double seed_jitter
  +int seed_particles
  +double duration
  +int frame_stride
  +int preview_steps
  +HourglassShellConfig hourglass_shell
  +vector<CylinderShellConfig> cylinder_shells
  +vector<PlaneColliderConfig> colliders
}

class MaterialConfig {
  +double density
  +double youngs_modulus
  +double poisson_ratio
  +double friction_angle_degrees
  +double hardening_h0
  +double hardening_h1
  +double hardening_h2
  +double hardening_h3
}

class SolverConfig {
  +double dx
  +double dt_max
  +double cfl
  +int particles_per_cell
  +bool explicit_only
}

class EmitterConfig {
  +bool enabled
  +std::string shape
  +std::string sampling_mode
  +Vec3d box_min
  +Vec3d box_max
  +double radius
  +double jitter
  +Vec3d initial_velocity
  +int particles_per_step
  +int start_step
  +int stop_step
  +int max_particles
}

class ExportConfig {
  +bool export_ply
  +std::string output_directory
  +int interval_steps
  +bool export_initial_frame
}

class Vec3d {
  +double x
  +double y
  +double z
}

class PlaneColliderConfig {
  +Vec3d point
  +Vec3d normal
  +double friction_coefficient
  +bool bounded
  +Vec3d bounds_min
  +Vec3d bounds_max
}

class HourglassShellConfig {
  +bool enabled
  +Vec3d waist_center
  +double top_y
  +double bottom_y
  +double neck_half_width
  +double top_half_width
  +double bottom_half_width
  +double upper_friction_coefficient
  +double lower_friction_coefficient
  +bool add_bottom_cap
  +double bottom_cap_friction_coefficient
}

class CylinderShellConfig {
  +bool enabled
  +Vec3d center
  +double radius
  +double y_min
  +double y_max
  +double friction_coefficient
  +int segments
}

class RuntimePlaneCollider {
  +Eigen::Vector3d point
  +Eigen::Vector3d normal
  +double friction_coefficient
  +bool bounded
  +Eigen::Vector3d bounds_min
  +Eigen::Vector3d bounds_max
}

class LameParameters {
  +double mu
  +double lambda
}

class PlasticProjectionResult {
  +Eigen::Matrix3d elastic_deformation_gradient
  +double delta_q
}

Simulation *-- SimulationParticle : owns
Simulation *-- SimulationGridNode : sparse grid
Simulation *-- SimulationConfig : configured by
Simulation ..> SimulationStats : returns

SimulationConfig *-- SceneConfig
SimulationConfig *-- MaterialConfig
SimulationConfig *-- SolverConfig
SimulationConfig *-- EmitterConfig
SimulationConfig *-- ExportConfig

SceneConfig *-- HourglassShellConfig
SceneConfig *-- CylinderShellConfig
SceneConfig *-- PlaneColliderConfig

PlaneColliderConfig *-- Vec3d
HourglassShellConfig *-- Vec3d
CylinderShellConfig *-- Vec3d
SceneConfig *-- Vec3d
EmitterConfig *-- Vec3d

LameParameters ..> MaterialConfig : derived from
PlasticProjectionResult ..> SimulationParticle : updates

note for Simulation "Owns the mutable MPM runtime state and advances the solver."
note for SimulationParticle "Represents `Simulation::Particle`."
note for SimulationGridNode "Represents `Simulation::GridNode`."
note for SimulationConfig "Loaded from JSON and passed into Simulation as the input model."
note for RuntimePlaneCollider "Internal runtime-only collider expanded from scene config."
note for LameParameters "Represents `detail::LameParameters`."
note for PlasticProjectionResult "Returned by the plasticity projection helper."
```

## Reading Order

- Start with `SimulationConfig` to see how scene input is structured.
- Then read `Simulation` and its nested `Particle` and `GridNode` types.
- The `detail::` structs are internal implementation helpers and are not part of the public API.
