/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Terrain in the navigator's energy (P5, docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md; doc 31 D9).
//
// From a height raster (metres, the sim grid's points, row 0 at min_y) and the vehicle's limits,
// terrain_build derives:
//   - the slope, dz/dx and dz/dy (m/m): gravity along a heading for the governors (H2);
//   - the grade cost r~_g in [0, 1]: sin(slope) over the sine of the steepest feasible slope, so
//     1 is the vehicle's limit. Through the material pipeline its gradient becomes the soft
//     force F = -lam_grade grad r~_g, which leans the drive toward gentler ground (H1);
//   - the infeasible cells: steeper than the vehicle can climb (max_grade), or, with a tip
//     ratio, than it can cross without rolling (tip_margin x the static stability factor, the
//     track over twice the centre of gravity's height). Any heading over such a cell could
//     fail, so the cell plans as an obstacle (the hard barrier, H1).
// Everything is per unit mass (gravity, not weight), so the trained coefficients keep their
// meaning (NAV_UNITS_ROADMAP).

#ifndef __CVC_NAV_TERRAIN_H__
#define __CVC_NAV_TERRAIN_H__

#include <cstdint>
#include <vector>

namespace cvc {
namespace nav {

struct terrain_config {
  double gravity = 9.80665; // m/s^2
  double max_grade = 0.6;   // tan of the steepest slope the vehicle climbs (0.6 = 60%, 31 deg)
  double tip_ratio = 0.0;   // static stability factor, track / (2 x CG height); 0 = no tip limit
  double tip_margin = 0.8;  // the share of the tip slope that is feasible
  double sigma = 1.0;       // cells: the grade cost's blur (material_build; the tree has metres)
  // Slope is a material risk, not a wall: the grade cost and the infeasible cells go through the
  // material channel like any other material, the grade cost's soft force leaning the drive
  // toward gentler ground and the infeasible cells its hard plane, a finite barrier the drive can
  // weigh against getting on (0 = off; the material channel's barrier shape). The hard weight is
  // three times the material channel's lam_hard: at 1 the Jolt Humvee crossing Austin's twisted
  // ramps rolled in most runs, at 3 its lead never did (cvcdbg's convoy A/B, 8 seeds). A vehicle
  // the planner sends over a steep cell it can physically drive is not pinned there as against a
  // building.
  float lam_grade = 0.5f, lam_grade_hard = 3.0f;
  float k_sharp = 1.25f;         // 1/m
  float d_hat_m = 12.0f;         // m
  bool block_infeasible = false; // true: infeasible cells also plan as obstacles (walls)
  bool governors = true;         // gravity along the path in v_stop and v_corner (H2)
};

struct terrain_planes {
  int rows = 0, cols = 0;
  std::vector<float> slope;             // [2, rows, cols]: dz/dx, then dz/dy (m/m)
  std::vector<float> grade_cost;        // [rows, cols], [0, 1]
  std::vector<std::uint8_t> infeasible; // [rows, cols], 1 = beyond the vehicle's limits
  double limit = 0.0;                   // tan of the steepest feasible slope
};

// `height_m` is rows x cols, row-major, row 0 at min_y; cells are cell_w metres square.
terrain_planes terrain_build(const float *height_m, int rows, int cols, double cell_w,
                             const terrain_config &c);

// The steepest feasible slope's tangent for `c`.
double terrain_limit(const terrain_config &c);

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_TERRAIN_H__
