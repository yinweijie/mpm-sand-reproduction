#pragma once

#include <array>
#include <cmath>

namespace klar2016 {

struct CubicAxisStencil {
    std::array<int, 4> nodes{};
    std::array<double, 4> weights{};
    std::array<double, 4> gradients{};
};

inline double cubic_bspline_weight(double distance_in_grid_units) {
    const double x = std::abs(distance_in_grid_units);
    if (x < 1.0) {
        return 0.5 * x * x * x - x * x + (2.0 / 3.0);
    }
    if (x < 2.0) {
        const double t = 2.0 - x;
        return (t * t * t) / 6.0;
    }
    return 0.0;
}

inline double cubic_bspline_gradient(double distance_in_grid_units, double inv_dx) {
    const double x = distance_in_grid_units;
    const double abs_x = std::abs(x);
    double derivative = 0.0;

    if (abs_x < 1.0) {
        derivative = 1.5 * x * abs_x - 2.0 * x;
    } else if (abs_x < 2.0) {
        const double sign = (x >= 0.0) ? 1.0 : -1.0;
        const double t = 2.0 - abs_x;
        derivative = -0.5 * t * t * sign;
    }

    return derivative * inv_dx;
}

inline CubicAxisStencil build_cubic_axis_stencil(double particle_coordinate_in_grid_units, double inv_dx) {
    CubicAxisStencil stencil;
    const int center = static_cast<int>(std::floor(particle_coordinate_in_grid_units));

    for (int i = 0; i < 4; ++i) {
        const int node = center - 1 + i;
        const double distance = particle_coordinate_in_grid_units - static_cast<double>(node);
        stencil.nodes[static_cast<std::size_t>(i)] = node;
        stencil.weights[static_cast<std::size_t>(i)] = cubic_bspline_weight(distance);
        stencil.gradients[static_cast<std::size_t>(i)] = cubic_bspline_gradient(distance, inv_dx);
    }

    return stencil;
}

}  // namespace klar2016
