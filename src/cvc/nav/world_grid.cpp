/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/world_grid.h>
#include <cvc/nav/world_planes.h>
#include <cvc/world/world_geometry.h>
#include <stdexcept>

namespace cvc {
namespace nav {

namespace {

void wall(world_grid &g) {
  for (int c = 0; c < g.cols; ++c) {
    g.occupancy[c] = 1;
    g.occupancy[static_cast<std::size_t>(g.rows - 1) * g.cols + c] = 1;
  }
  for (int r = 0; r < g.rows; ++r) {
    g.occupancy[static_cast<std::size_t>(r) * g.cols] = 1;
    g.occupancy[static_cast<std::size_t>(r) * g.cols + g.cols - 1] = 1;
  }
}

void size_grid(world_grid &g, double min_x, double min_y, double max_x, double max_y, double cell) {
  if (!(cell > 0.0) || !std::isfinite(cell))
    throw std::invalid_argument("world_grid: the cell must be > 0");
  const int cols = static_cast<int>(std::floor((max_x - min_x) / cell + 1e-9)) + 1;
  const int rows = static_cast<int>(std::floor((max_y - min_y) / cell + 1e-9)) + 1;
  if (!(cols >= 3 && rows >= 3))
    throw std::invalid_argument("world_grid: the extent is under 3 x 3 cells");
  g.rows = rows;
  g.cols = cols;
  g.min_x = min_x;
  g.min_y = min_y;
  g.max_x = min_x + cell * (cols - 1);
  g.max_y = min_y + cell * (rows - 1);
}

// splitmix64: the same draws on every platform (std::uniform_*_distribution are not).
struct draws {
  std::uint64_t s;
  std::uint64_t next() {
    std::uint64_t z = (s += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
  }
  std::size_t below(std::size_t n) { return static_cast<std::size_t>(next() % n); }
};

} // namespace

void world_grid::sample_of(double x, double y, int &r, int &c) const {
  const double h = cell();
  c = h > 0.0 ? static_cast<int>(std::lround((x - min_x) / h)) : 0;
  r = h > 0.0 ? static_cast<int>(std::lround((y - min_y) / h)) : 0;
  c = std::clamp(c, 0, std::max(cols - 1, 0));
  r = std::clamp(r, 0, std::max(rows - 1, 0));
}

bool world_grid::occupied(double x, double y) const {
  if (!(x >= min_x && x <= max_x && y >= min_y && y <= max_y) || occupancy.empty())
    return true;
  int r = 0, c = 0;
  sample_of(x, y, r, c);
  return occupancy[static_cast<std::size_t>(r) * cols + c] != 0;
}

std::size_t world_grid::occupied_count() const {
  return static_cast<std::size_t>(
      std::count_if(occupancy.begin(), occupancy.end(), [](std::uint8_t v) { return v != 0; }));
}

world_grid make_grid(const cvc::world::world_geometry &world, double min_x, double min_y,
                     double max_x, double max_y, double cell, int inflate) {
  world_grid g;
  size_grid(g, min_x, min_y, max_x, max_y, cell);
  world_planes p = rasterize(world, g.rows, g.cols, g.min_x, g.min_y, g.max_x, g.max_y, inflate);
  g.occupancy = std::move(p.occupancy);
  g.height = std::move(p.height);
  const std::size_t n = static_cast<std::size_t>(g.rows) * g.cols;
  g.occupancy.resize(n, 0);
  g.height.resize(n, 0.0f);
  wall(g);
  return g;
}

world_grid make_grid(const cvc::world::world_geometry &world, double cell, double half,
                     int inflate) {
  const auto terrain = world.terrain();
  if (!terrain)
    throw std::invalid_argument("world_grid: the world has no terrain to take its extent from");
  double mnx = terrain->x0(), mny = terrain->y0(), mxx = terrain->max_x(), mxy = terrain->max_y();
  if (half > 0.0) {
    const double cx = 0.5 * (mnx + mxx), cy = 0.5 * (mny + mxy);
    const double h = std::min({half, 0.5 * (mxx - mnx), 0.5 * (mxy - mny)});
    mnx = cx - h;
    mny = cy - h;
    mxx = cx + h;
    mxy = cy + h;
  }
  return make_grid(world, mnx, mny, mxx, mxy, cell, inflate);
}

world_grid flat_grid(double half, double cell) {
  if (!(half > 0.0))
    throw std::invalid_argument("world_grid: a flat square needs half > 0");
  world_grid g;
  size_grid(g, -half, -half, half, half, cell);
  const std::size_t n = static_cast<std::size_t>(g.rows) * g.cols;
  g.occupancy.assign(n, 0);
  g.height.assign(n, 0.0f);
  wall(g);
  return g;
}

std::vector<float> signed_distance(const world_grid &g) {
  const std::size_t n = static_cast<std::size_t>(g.rows) * g.cols;
  std::vector<float> out(n, 0.0f);
  if (n == 0 || g.occupancy.size() != n)
    return out;
  std::vector<std::uint8_t> blocked(n), free(n);
  for (std::size_t i = 0; i < n; ++i) {
    blocked[i] = g.occupancy[i] ? 1 : 0;
    free[i] = g.occupancy[i] ? 0 : 1;
  }
  const std::vector<double> to_blocked = edt2_squared(blocked.data(), g.rows, g.cols);
  const std::vector<double> to_free = edt2_squared(free.data(), g.rows, g.cols);
  const double h = g.cell();
  for (std::size_t i = 0; i < n; ++i)
    out[i] =
        static_cast<float>((blocked[i] ? -std::sqrt(to_free[i]) : std::sqrt(to_blocked[i])) * h);
  return out;
}

std::vector<std::array<double, 2>> free_points(const world_grid &g, const std::vector<float> &sdf,
                                               std::uint64_t seed, int n, double clearance,
                                               double min_sep) {
  const std::size_t cells = static_cast<std::size_t>(g.rows) * g.cols;
  if (sdf.size() != cells)
    throw std::invalid_argument("free_points: the distance field is not the grid's");
  std::vector<std::size_t> pool;
  for (std::size_t i = 0; i < cells; ++i)
    if (sdf[i] >= clearance)
      pool.push_back(i);
  if (pool.empty())
    for (std::size_t i = 0; i < cells; ++i)
      if (sdf[i] > 0.0f)
        pool.push_back(i);
  if (pool.empty())
    throw std::runtime_error("free_points: the world has no free ground");
  const double h = g.cell();
  auto point_of = [&](std::size_t i) {
    return std::array<double, 2>{g.min_x + static_cast<double>(i % g.cols) * h,
                                 g.min_y + static_cast<double>(i / g.cols) * h};
  };
  draws rng{seed};
  std::vector<std::array<double, 2>> pts;
  pts.reserve(static_cast<std::size_t>(std::max(n, 0)));
  for (int k = 0; k < n; ++k) {
    std::array<double, 2> p{};
    for (int t = 0; t < 400; ++t) {
      p = point_of(pool[rng.below(pool.size())]);
      const bool crowded = std::any_of(pts.begin(), pts.end(), [&](const std::array<double, 2> &q) {
        return std::hypot(p[0] - q[0], p[1] - q[1]) < min_sep;
      });
      if (!crowded)
        break;
    }
    pts.push_back(p);
  }
  return pts;
}

} // namespace nav
} // namespace cvc
