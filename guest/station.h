#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

// The station's colliders as an MJCF model, in Godot's frame (y up, metres), so the host never
// converts axes. A primitive is kPrimStride doubles: type, centre xyz, size xyz, quat wxyz,
// mocap flag. Type 0 is a box (half-extents), type 1 a vertical cylinder (radius, half-height).
namespace station {

constexpr int kPrimStride = 12;
constexpr int kBox = 0;
constexpr int kCylinder = 1;
constexpr double kPlayerRadius = 0.3;
constexpr double kPlayerHeight = 1.7;

// hsize is the terrain's x and z half-extents, then optionally its centre x, z. Row r of the
// nrow x ncol grid lies at z = cz - rz + 2 rz r / (nrow - 1), column c at x = cx - rx + 2 rx c / (ncol - 1).
// An empty grid means no terrain. Returns "" and fills error when the input is malformed.
std::string build_mjcf(const std::vector<double> &prims, const std::vector<double> &grid, int nrow, int ncol,
		const std::vector<double> &hsize, std::string &error);

mjModel *load_mjcf(const std::string &xml, std::string &error);

// The nearest hit along a ray within maxdist: hit, dist, point xyz, normal xyz, geom.
std::vector<double> ray(const mjModel *m, const mjData *d, const double origin[3], const double dir[3],
		double maxdist, int exclude_body);

} // namespace station
