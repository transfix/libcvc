/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// water_bodies.cpp — the bodies of water and the shore grid (see water_bodies.h).

#include <algorithm>
#include <cmath>
#include <cvc/algo/distance_transform.h>
#include <cvc/world/terrain_source.h>
#include <cvc/world/water_bodies.h>
#include <limits>
#include <stdexcept>
#include <string>

namespace cvc {
namespace world {

namespace {

// A separable box average of radius (rx, ry) samples, the window clipped to the grid.
void boxBlur(std::vector<double> &a, int rows, int cols, int rx, int ry) {
  std::vector<double> tmp(a.size()), pre;
  if (rx > 0) {
    pre.resize(std::size_t(cols) + 1);
    for (int r = 0; r < rows; ++r) {
      const std::size_t o = std::size_t(r) * cols;
      pre[0] = 0.0;
      for (int c = 0; c < cols; ++c)
        pre[std::size_t(c) + 1] = pre[std::size_t(c)] + a[o + std::size_t(c)];
      for (int c = 0; c < cols; ++c) {
        const int lo = std::max(0, c - rx), hi = std::min(cols - 1, c + rx);
        tmp[o + std::size_t(c)] =
            (pre[std::size_t(hi) + 1] - pre[std::size_t(lo)]) / double(hi - lo + 1);
      }
    }
    a.swap(tmp);
  }
  if (ry > 0) {
    pre.resize(std::size_t(rows) + 1);
    for (int c = 0; c < cols; ++c) {
      pre[0] = 0.0;
      for (int r = 0; r < rows; ++r)
        pre[std::size_t(r) + 1] = pre[std::size_t(r)] + a[std::size_t(r) * cols + c];
      for (int r = 0; r < rows; ++r) {
        const int lo = std::max(0, r - ry), hi = std::min(rows - 1, r + ry);
        tmp[std::size_t(r) * cols + c] =
            (pre[std::size_t(hi) + 1] - pre[std::size_t(lo)]) / double(hi - lo + 1);
      }
    }
    a.swap(tmp);
  }
}

// The derivative of plane p along one axis at sample (r, c): central inside, one-sided at an edge.
template <class Plane>
double diff(const Plane &p, int rows, int cols, int r, int c, bool alongX, double h) {
  const int n = alongX ? cols : rows, i = alongX ? c : r;
  auto at = [&](int j) {
    return double(p[alongX ? std::size_t(r) * cols + j : std::size_t(j) * cols + c]);
  };
  if (n < 2)
    return 0.0;
  if (i == 0)
    return (at(1) - at(0)) / h;
  if (i == n - 1)
    return (at(n - 1) - at(n - 2)) / h;
  return (at(i + 1) - at(i - 1)) / (2.0 * h);
}

std::uint32_t mix32(std::uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

} // namespace

const char *water_kind_name(water_kind k) {
  switch (k) {
  case water_kind::sea:
    return "sea";
  case water_kind::lake:
    return "lake";
  case water_kind::pond:
    return "pond";
  case water_kind::stream:
    return "stream";
  }
  return "sea";
}

water_bodies build_water_bodies(std::vector<float> ground, const grid_spec &grid,
                                const water_bodies_params &p) {
  if (grid.rows < 2 || grid.cols < 2)
    throw std::invalid_argument("build_water_bodies: the grid must be at least 2 x 2");
  if (ground.size() != grid.count())
    throw std::invalid_argument("build_water_bodies: " + std::to_string(ground.size()) +
                                " ground samples for a " + std::to_string(grid.rows) + " x " +
                                std::to_string(grid.cols) + " grid");
  if (!grid.square(1e-9 * std::max(1.0, grid.cell_w())))
    throw std::invalid_argument("build_water_bodies: the grid's cells must be square");
  const int rows = grid.rows, cols = grid.cols;
  const std::size_t n = grid.count();
  const double cw = grid.cell_w(), ch = grid.cell_h();
  const double level = p.level_m;

  water_bodies wb;
  wb.grid = grid;
  wb.params = p;
  wb.ground = std::move(ground);
  wb.body.assign(n, -1);
  wb.level.assign(n, float(level));
  wb.depth.resize(n);
  std::vector<std::uint8_t> wet(n);
  std::size_t wetCount = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const float g = wb.ground[i];
    const bool w = std::isfinite(g) && double(g) < level;
    wet[i] = w ? 1 : 0;
    wetCount += w ? 1 : 0;
    wb.depth[i] =
        std::isfinite(g) ? float(level - double(g)) : std::numeric_limits<float>::quiet_NaN();
  }

  // The 8-connected runs of wet samples, numbered by their first sample in the grid, and whether
  // each touches the grid's edge.
  std::vector<std::int32_t> run(n, -1);
  std::vector<std::uint8_t> runOpen;
  {
    std::vector<std::size_t> stack;
    for (std::size_t seed = 0; seed < n; ++seed) {
      if (!wet[seed] || run[seed] >= 0)
        continue;
      const std::int32_t id = std::int32_t(runOpen.size());
      bool edge = false;
      run[seed] = id;
      stack.assign(1, seed);
      while (!stack.empty()) {
        const std::size_t i = stack.back();
        stack.pop_back();
        const int r = int(i / std::size_t(cols)), c = int(i % std::size_t(cols));
        edge = edge || r == 0 || c == 0 || r == rows - 1 || c == cols - 1;
        for (int dr = -1; dr <= 1; ++dr)
          for (int dc = -1; dc <= 1; ++dc) {
            const int rr = r + dr, cc = c + dc;
            if ((dr == 0 && dc == 0) || rr < 0 || cc < 0 || rr >= rows || cc >= cols)
              continue;
            const std::size_t j = std::size_t(rr) * cols + cc;
            if (wet[j] && run[j] < 0) {
              run[j] = id;
              stack.push_back(j);
            }
          }
      }
      runOpen.push_back(edge ? 1 : 0);
    }
  }

  // The bodies: every run that reaches the edge is the one sea (they meet past the world), each
  // other run a body of its own, in the runs' order.
  std::vector<std::int32_t> bodyOf(runOpen.size());
  std::int32_t sea = -1, nb = 0;
  for (std::size_t k = 0; k < runOpen.size(); ++k) {
    if (runOpen[k]) {
      if (sea < 0)
        sea = nb++;
      bodyOf[k] = sea;
    } else {
      bodyOf[k] = nb++;
    }
  }
  wb.bodies.assign(std::size_t(nb), water_body{});
  for (std::size_t i = 0; i < n; ++i)
    if (run[i] >= 0)
      wb.body[i] = bodyOf[std::size_t(run[i])];

  // Each body's samples, bounds and deepest point, then its fetch from the samples' second moments
  // about its bounds' corner: row-major passes, so the sums come in one order.
  {
    std::vector<int> r0(std::size_t(nb), rows), r1(std::size_t(nb), -1), c0(std::size_t(nb), cols),
        c1(std::size_t(nb), -1);
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c) {
        const std::size_t i = std::size_t(r) * cols + c;
        const std::int32_t id = wb.body[i];
        if (id < 0)
          continue;
        water_body &b = wb.bodies[std::size_t(id)];
        ++b.cells;
        b.max_depth_m = std::max(b.max_depth_m, double(wb.depth[i]));
        r0[std::size_t(id)] = std::min(r0[std::size_t(id)], r);
        r1[std::size_t(id)] = std::max(r1[std::size_t(id)], r);
        c0[std::size_t(id)] = std::min(c0[std::size_t(id)], c);
        c1[std::size_t(id)] = std::max(c1[std::size_t(id)], c);
      }
    for (std::int32_t k = 0; k < nb; ++k) {
      water_body &b = wb.bodies[std::size_t(k)];
      b.level_m = level;
      b.min_x = grid.world_x(c0[std::size_t(k)]);
      b.max_x = grid.world_x(c1[std::size_t(k)]);
      b.min_y = grid.world_y(r0[std::size_t(k)]);
      b.max_y = grid.world_y(r1[std::size_t(k)]);
      b.area_m2 = double(b.cells) * cw * ch;
      b.kind = k == sea ? water_kind::sea
                        : (b.area_m2 < p.pond_max_area_m2 ? water_kind::pond : water_kind::lake);
    }
    std::vector<double> sx(std::size_t(nb), 0.0), sy(std::size_t(nb), 0.0),
        sxx(std::size_t(nb), 0.0), syy(std::size_t(nb), 0.0), sxy(std::size_t(nb), 0.0);
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c) {
        const std::int32_t id = wb.body[std::size_t(r) * cols + c];
        if (id < 0)
          continue;
        const water_body &b = wb.bodies[std::size_t(id)];
        const double x = grid.world_x(c) - b.min_x, y = grid.world_y(r) - b.min_y;
        sx[std::size_t(id)] += x;
        sy[std::size_t(id)] += y;
        sxx[std::size_t(id)] += x * x;
        syy[std::size_t(id)] += y * y;
        sxy[std::size_t(id)] += x * y;
      }
    for (std::size_t k = 0; k < std::size_t(nb); ++k) {
      water_body &b = wb.bodies[k];
      const double m = double(b.cells), mx = sx[k] / m, my = sy[k] / m;
      const double a = std::max(0.0, sxx[k] / m - mx * mx), d = std::max(0.0, syy[k] / m - my * my);
      const double o = sxy[k] / m - mx * my;
      const double lmax = 0.5 * (a + d) + std::sqrt(0.25 * (a - d) * (a - d) + o * o);
      b.fetch_m = std::max(4.0 * std::sqrt(lmax), std::min(cw, ch));
    }
  }

  // The signed distance to the shore: the exact distance transform of the wet samples and of the
  // dry ones (cvc::algo, one pass for both), in metres, half a sample short (the shore lies between
  // a wet sample and its dry neighbour).
  const double diag = std::hypot(grid.max_x - grid.min_x, grid.max_y - grid.min_y);
  const double half = 0.5 * cw;
  wb.shore_m.resize(n);
  if (wetCount == 0 || wetCount == n) {
    std::fill(wb.shore_m.begin(), wb.shore_m.end(), float(wetCount == n ? diag : -diag));
  } else {
    std::vector<double> toWet(n), toDry(n);
    cvc::algo::edt2_squared_both(wet.data(), rows, cols, toWet.data(), toDry.data());
    for (std::size_t i = 0; i < n; ++i) {
      const double s = wet[i] ? std::sqrt(toDry[i]) * cw - half : half - std::sqrt(toWet[i]) * cw;
      wb.shore_m[i] = float(std::clamp(s, -diag, diag));
    }
  }

  // The shore normal: the distance's gradient, smoothed, then made unit.
  {
    std::vector<double> gx(n), gy(n);
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c) {
        const std::size_t i = std::size_t(r) * cols + c;
        gx[i] = diff(wb.shore_m, rows, cols, r, c, true, cw);
        gy[i] = diff(wb.shore_m, rows, cols, r, c, false, ch);
      }
    if (p.normal_smooth_m > 0.0) {
      const int rx = int(std::lround(p.normal_smooth_m / cw)),
                ry = int(std::lround(p.normal_smooth_m / ch));
      boxBlur(gx, rows, cols, rx, ry);
      boxBlur(gy, rows, cols, rx, ry);
    }
    wb.normal_x.assign(n, 0.0f);
    wb.normal_y.assign(n, 0.0f);
    for (std::size_t i = 0; i < n; ++i) {
      const double len = std::hypot(gx[i], gy[i]);
      if (len > 1e-6 && std::isfinite(len)) {
        wb.normal_x[i] = float(gx[i] / len);
        wb.normal_y[i] = float(gy[i] / len);
      }
    }
  }

  // The ground's slope (rise over run).
  wb.slope.assign(n, 0.0f);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      const double sx = diff(wb.ground, rows, cols, r, c, true, cw),
                   sy = diff(wb.ground, rows, cols, r, c, false, ch);
      const double s = std::hypot(sx, sy);
      wb.slope[std::size_t(r) * cols + c] = std::isfinite(s) ? float(s) : 0.0f;
    }
  return wb;
}

water_bodies build_water_bodies(const terrain_source &t, const grid_spec &grid,
                                const water_bodies_params &p) {
  std::vector<float> ground(grid.count());
  for (int r = 0; r < grid.rows; ++r)
    for (int c = 0; c < grid.cols; ++c)
      ground[std::size_t(r) * grid.cols + c] = float(t.sample(grid.world_x(c), grid.world_y(r)));
  return build_water_bodies(std::move(ground), grid, p);
}

int water_bodies::body_at(double x, double y) const {
  if (empty())
    return -1;
  const double fc = (x - grid.min_x) / grid.cell_w(), fr = (y - grid.min_y) / grid.cell_h();
  if (!(fc > -0.5 && fr > -0.5 && fc < grid.cols - 0.5 && fr < grid.rows - 0.5))
    return -1;
  const int c = std::clamp(int(std::lround(fc)), 0, grid.cols - 1),
            r = std::clamp(int(std::lround(fr)), 0, grid.rows - 1);
  return body[index(r, c)];
}

double water_bodies::sample(const std::vector<float> &plane, double x, double y) const {
  if (plane.size() != grid.count() || plane.empty())
    return std::numeric_limits<double>::quiet_NaN();
  const double fc = std::clamp((x - grid.min_x) / grid.cell_w(), 0.0, double(grid.cols - 1));
  const double fr = std::clamp((y - grid.min_y) / grid.cell_h(), 0.0, double(grid.rows - 1));
  const int c0 = std::min(grid.cols - 2, int(std::floor(fc))),
            r0 = std::min(grid.rows - 2, int(std::floor(fr)));
  const double u = fc - c0, v = fr - r0;
  const double a = plane[index(r0, c0)], b = plane[index(r0, c0 + 1)], c = plane[index(r0 + 1, c0)],
               d = plane[index(r0 + 1, c0 + 1)];
  return (a * (1 - u) + b * u) * (1 - v) + (c * (1 - u) + d * u) * v;
}

std::size_t water_bodies::count(water_kind k) const {
  std::size_t m = 0;
  for (const water_body &b : bodies)
    m += b.kind == k ? 1 : 0;
  return m;
}

std::size_t water_bodies::wet_cells() const {
  std::size_t m = 0;
  for (const water_body &b : bodies)
    m += b.cells;
  return m;
}

std::string water_bodies_summary(const water_bodies &wb) {
  std::string s;
  const water_kind kinds[4] = {water_kind::sea, water_kind::lake, water_kind::pond,
                               water_kind::stream};
  for (water_kind k : kinds) {
    const std::size_t m = wb.count(k);
    if (m == 0)
      continue;
    if (!s.empty())
      s += ", ";
    s += std::to_string(m) + " " + water_kind_name(k);
    if (m > 1)
      s += "s";
  }
  return s.empty() ? "no water" : s;
}

void water_bodies_debug_rgb(const water_bodies &wb, double x, double y, double band_m,
                            unsigned char rgb[3]) {
  double col[3] = {0.25, 0.0, 0.25}; // no grid: dark magenta
  if (!wb.empty()) {
    // Wet where the interpolated shore distance is positive, in the body of the wettest of the
    // four samples round (x, y): the waterline runs between the samples, not along their cells.
    const double s = wb.sample(wb.shore_m, x, y);
    std::int32_t id = -1;
    if (s > 0.0) {
      const grid_spec &g = wb.grid;
      const double fc = std::clamp((x - g.min_x) / g.cell_w(), 0.0, double(g.cols - 1));
      const double fr = std::clamp((y - g.min_y) / g.cell_h(), 0.0, double(g.rows - 1));
      const int c0 = std::min(g.cols - 2, int(std::floor(fc))),
                r0 = std::min(g.rows - 2, int(std::floor(fr)));
      float best = 0.0f;
      for (int k = 0; k < 4; ++k) {
        const std::size_t i = wb.index(r0 + k / 2, c0 + k % 2);
        if (wb.body[i] >= 0 && wb.shore_m[i] > best) {
          best = wb.shore_m[i];
          id = wb.body[i];
        }
      }
    }
    if (id < 0) {
      // Dry land in greys: low ground dark, high ground light.
      const double above = wb.sample(wb.ground, x, y) - wb.sample(wb.level, x, y);
      const double t = std::isfinite(above) ? 0.5 + 0.5 * std::tanh(above / 40.0) : 0.0;
      col[0] = col[1] = col[2] = 0.30 + 0.45 * t;
    } else {
      static const double hue[4][3] = {{0.10, 0.30, 0.80},  // sea
                                       {0.10, 0.62, 0.62},  // lake
                                       {0.25, 0.68, 0.22},  // pond
                                       {0.20, 0.80, 0.95}}; // stream
      const water_body &b = wb.bodies[std::size_t(id)];
      const double *h = hue[std::min<std::size_t>(3, std::size_t(b.kind))];
      // A shade of its own, so neighbouring bodies of a kind tell apart; darker with depth.
      const double shade = 0.75 + 0.5 * double(mix32(std::uint32_t(id) + 0x9e37u) & 0xffu) / 255.0;
      const double d = wb.sample(wb.depth, x, y);
      const double deep = std::exp(-std::max(0.0, std::isfinite(d) ? d : 0.0) / 25.0);
      for (int q = 0; q < 3; ++q)
        col[q] = h[q] * shade * (0.45 + 0.55 * deep);
    }
    // A light contour every band_m of shore distance, either side of the waterline.
    if (band_m > 0.0 && std::isfinite(s)) {
      const double a = std::fabs(s);
      const double w = 0.35 * std::min(wb.grid.cell_w(), wb.grid.cell_h());
      const double f = std::fmod(a, band_m);
      if (f < w || band_m - f < w)
        for (int q = 0; q < 3; ++q)
          col[q] = 0.6 * col[q] + 0.4;
    }
  }
  for (int q = 0; q < 3; ++q)
    rgb[q] = (unsigned char)std::lround(std::clamp(col[q], 0.0, 1.0) * 255.0);
}

} // namespace world
} // namespace cvc
