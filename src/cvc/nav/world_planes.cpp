/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <array>
#include <cmath>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/world_planes.h>
#include <limits>
#include <stdexcept>

namespace cvc {
namespace nav {

namespace {

// Visit every grid sample inside each triangle's ground projection (either
// winding), with the triangle's index and its plane's height there.
template <class F>
void for_covered(const cvc::geometry &mesh, const cvc::world::geometry_item *placement, int rows,
                 int cols, double min_x, double min_y, double max_x, double max_y, F &&visit) {
  const auto &pts = mesh.const_points();
  const double sx = (cols - 1) / (max_x - min_x), sy = (rows - 1) / (max_y - min_y);
  // The mesh's vertices in world coordinates (a placed item is rotated, then moved).
  std::vector<std::array<double, 3>> w(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    double x = pts[i][0], y = pts[i][1], z = pts[i][2];
    if (placement) {
      const auto &q = placement->rotation;
      const double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
      const double tx = 2 * (qy * z - qz * y), ty = 2 * (qz * x - qx * z),
                   tz = 2 * (qx * y - qy * x);
      const double rx = x + qw * tx + (qy * tz - qz * ty);
      const double ry = y + qw * ty + (qz * tx - qx * tz);
      const double rz = z + qw * tz + (qx * ty - qy * tx);
      x = rx + placement->position[0];
      y = ry + placement->position[1];
      z = rz + placement->position[2];
    }
    w[i] = {{x, y, z}};
  }
  auto triangle = [&](std::size_t face, std::uint64_t ia, std::uint64_t ib, std::uint64_t ic) {
    if (ia >= w.size() || ib >= w.size() || ic >= w.size())
      return;
    const std::uint64_t idx[3] = {ia, ib, ic};
    double px[3], py[3], pz[3];
    for (int k = 0; k < 3; ++k) {
      px[k] = (w[idx[k]][0] - min_x) * sx; // -> column
      py[k] = (w[idx[k]][1] - min_y) * sy; // -> row
      pz[k] = w[idx[k]][2];
    }
    int c0 = static_cast<int>(std::floor(std::min({px[0], px[1], px[2]})));
    int c1 = static_cast<int>(std::ceil(std::max({px[0], px[1], px[2]})));
    int r0 = static_cast<int>(std::floor(std::min({py[0], py[1], py[2]})));
    int r1 = static_cast<int>(std::ceil(std::max({py[0], py[1], py[2]})));
    c0 = std::max(0, c0);
    r0 = std::max(0, r0);
    c1 = std::min(cols - 1, c1);
    r1 = std::min(rows - 1, r1);
    const double d = (py[1] - py[2]) * (px[0] - px[2]) + (px[2] - px[1]) * (py[0] - py[2]);
    if (std::fabs(d) < 1e-12)
      return; // edge-on: a wall seen from above covers nothing
    const double inv = 1.0 / d;
    for (int r = r0; r <= r1; ++r)
      for (int c = c0; c <= c1; ++c) {
        const double x = c, y = r;
        const double a = ((py[1] - py[2]) * (x - px[2]) + (px[2] - px[1]) * (y - py[2])) * inv;
        const double b = ((py[2] - py[0]) * (x - px[2]) + (px[0] - px[2]) * (y - py[2])) * inv;
        if (a >= 0.0 && b >= 0.0 && 1.0 - a - b >= 0.0)
          visit(r, c, face, a * pz[0] + b * pz[1] + (1.0 - a - b) * pz[2]);
      }
  };
  const auto &tris = mesh.const_tris();
  for (std::size_t i = 0; i < tris.size(); ++i)
    triangle(i, tris[i][0], tris[i][1], tris[i][2]);
  const auto &quads = mesh.const_quads();
  for (std::size_t i = 0; i < quads.size(); ++i) {
    triangle(tris.size() + i, quads[i][0], quads[i][1], quads[i][3]);
    triangle(tris.size() + i, quads[i][1], quads[i][2], quads[i][3]);
  }
}

void check_grid(int rows, int cols, double min_x, double min_y, double max_x, double max_y) {
  if (rows < 2 || cols < 2)
    throw std::invalid_argument("nav::rasterize: a grid needs at least 2 x 2 samples");
  if (!(max_x > min_x) || !(max_y > min_y))
    throw std::invalid_argument("nav::rasterize: the extent is empty");
}

} // namespace

std::vector<std::uint8_t> mesh_footprint(const cvc::geometry &mesh, int rows, int cols,
                                         double min_x, double min_y, double max_x, double max_y) {
  check_grid(rows, cols, min_x, min_y, max_x, max_y);
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(rows) * cols, 0);
  for_covered(
      mesh, nullptr, rows, cols, min_x, min_y, max_x, max_y,
      [&](int r, int c, std::size_t, double) { occ[static_cast<std::size_t>(r) * cols + c] = 1; });
  return occ;
}

world_planes rasterize(const cvc::world::world_geometry &g, int rows, int cols, double min_x,
                       double min_y, double max_x, double max_y, int inflate) {
  check_grid(rows, cols, min_x, min_y, max_x, max_y);
  const std::size_t n = static_cast<std::size_t>(rows) * cols;
  world_planes p;
  p.rows = rows;
  p.cols = cols;
  p.min_x = min_x;
  p.min_y = min_y;
  p.max_x = max_x;
  p.max_y = max_y;
  p.occupancy.assign(n, 0);
  p.height.assign(n, 0.0f);
  p.surface.assign(n, 0);
  const double dx = (max_x - min_x) / (cols - 1), dy = (max_y - min_y) / (rows - 1);

  // The ground: the first navigate height field.
  const cvc::world::geometry_item *ground = nullptr;
  for (const auto &item : g.items())
    if (item.has(cvc::world::role_navigate) && item.height_field()) {
      ground = &item;
      break;
    }
  if (ground) {
    const auto field = ground->height_field();
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c) {
        const double x = min_x + c * dx, y = min_y + r * dy;
        const double h = field->height(x, y);
        const std::size_t i = static_cast<std::size_t>(r) * cols + c;
        p.height[i] = std::isnan(h) ? 0.0f : static_cast<float>(h);
        p.surface[i] = field->cell_materials().empty() ? ground->surface : field->material(x, y);
      }
  }

  // Meshes on top: each covered sample takes the highest covering face's surface.
  std::vector<double> top(n, -std::numeric_limits<double>::infinity());
  for (const auto &item : g.items()) {
    if (!item.has(cvc::world::role_navigate) || item.height_field())
      continue;
    const cvc::geometry &mesh = *item.geometry;
    const bool per_face = mesh.has_face_materials();
    for_covered(mesh, &item, rows, cols, min_x, min_y, max_x, max_y,
                [&](int r, int c, std::size_t face, double z) {
                  const std::size_t i = static_cast<std::size_t>(r) * cols + c;
                  p.occupancy[i] = 1;
                  if (z > top[i]) {
                    top[i] = z;
                    p.surface[i] = per_face ? mesh.face_material(face) : item.surface;
                  }
                });
  }

  const cvc::world::surface_registry &surfaces = g.surfaces();
  p.risk_raw.resize(n);
  p.hard.resize(n);
  p.mu.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    const cvc::world::surface_class &s = surfaces[p.surface[i]];
    p.risk_raw[i] = s.rho;
    p.hard[i] = s.hard ? 1 : 0;
    p.mu[i] = s.mu;
    if (s.hard)
      p.occupancy[i] = 1; // hard is a subset of occupancy (surface.h)
  }
  if (inflate > 0)
    p.occupancy = cvc::nav::inflate(p.occupancy.data(), rows, cols, inflate);
  return p;
}

world_planes rasterize(const cvc::world::world_geometry &g, int rows, int cols, int inflate) {
  const auto field = g.terrain();
  if (!field)
    throw std::invalid_argument("nav::rasterize: the world has no terrain to take the extent from");
  return rasterize(g, rows, cols, field->x0(), field->y0(), field->max_x(), field->max_y(),
                   inflate);
}

} // namespace nav
} // namespace cvc
