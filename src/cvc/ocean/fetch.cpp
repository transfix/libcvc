/*
  Copyright 2026 The University of Texas at Austin

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

// fetch.cpp — fetch and lee (see fetch.h).

#include <algorithm>
#include <cmath>
#include <cvc/algo/det_math.h>
#include <cvc/ocean/fetch.h>
#include <cvc/ocean/spectrum.h>
#include <stdexcept>

namespace cvc {
namespace ocean {

namespace det = ::cvc::algo::det;

double fetch_field::sample(double x, double y) const {
  if (fetch_m.empty())
    return open_m;
  if (rows < 2 || cols < 2)
    return fetch_m.front();
  const double fc = std::clamp((x - min_x) / cell, 0.0, double(cols - 1));
  const double fr = std::clamp((y - min_y) / cell, 0.0, double(rows - 1));
  const int c0 = std::min(cols - 2, int(std::floor(fc))),
            r0 = std::min(rows - 2, int(std::floor(fr)));
  const double u = fc - c0, v = fr - r0;
  const std::size_t i = std::size_t(r0) * std::size_t(cols) + std::size_t(c0);
  const double a = fetch_m[i], b = fetch_m[i + 1], c = fetch_m[i + std::size_t(cols)],
               d = fetch_m[i + std::size_t(cols) + 1];
  return (a * (1 - u) + b * u) * (1 - v) + (c * (1 - u) + d * u) * v;
}

std::uint64_t fetch_field::digest() const {
  std::uint64_t h = 0xcbf29ce484222325ull;
  auto bytes = [&h](const void *p, std::size_t n) {
    const auto *b = static_cast<const unsigned char *>(p);
    for (std::size_t i = 0; i < n; ++i) {
      h ^= b[i];
      h *= 0x100000001b3ull;
    }
  };
  const std::int64_t rc[2] = {rows, cols};
  bytes(rc, sizeof rc);
  const double d[5] = {min_x, min_y, cell, dir_rad, open_m};
  bytes(d, sizeof d);
  if (!fetch_m.empty())
    bytes(fetch_m.data(), fetch_m.size() * sizeof(float));
  return h;
}

fetch_field build_fetch(const shore_grid &g, double dir_rad, double open_fetch_m,
                        const std::vector<float> *cap, int stride) {
  if (g.rows < 1 || g.cols < 1 || !(g.cell > 0.0) || g.depth.size() != g.count())
    throw std::invalid_argument("build_fetch: the grid must have samples, a cell above 0 and a "
                                "depth plane of rows x cols");
  if (cap && cap->size() != g.count())
    throw std::invalid_argument("build_fetch: the cap plane must be rows x cols");
  if (stride < 1)
    throw std::invalid_argument("build_fetch: the stride must be at least 1");
  fetch_field f;
  f.rows = (g.rows - 1) / stride + 1;
  f.cols = (g.cols - 1) / stride + 1;
  f.min_x = g.min_x;
  f.min_y = g.min_y;
  f.cell = g.cell * stride;
  f.dir_rad = dir_rad;
  f.open_m = std::max(open_fetch_m, 0.0);
  const int R = f.rows, C = f.cols;
  f.fetch_m.assign(f.count(), 0.0f);
  // Each coarse sample's water and its most fetch.
  std::vector<double> most(f.count(), 0.0);
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c) {
      const std::size_t gi =
          std::size_t(r * stride) * std::size_t(g.cols) + std::size_t(c * stride);
      const double d = g.depth[gi];
      if (!(d > 0.0))
        continue; // dry (or NaN): most stays 0
      double m = f.open_m;
      if (cap) {
        const double cv = (*cap)[gi];
        if (cv >= 0.0)
          m = std::min(m, cv);
      }
      most[std::size_t(r) * C + c] = m;
    }

  const double ux = det::cos(dir_rad), uy = det::sin(dir_rad);
  const bool alongX = std::fabs(ux) >= std::fabs(uy);
  // March one column (or row) at a time from the upwind edge: the point one column upwind of a
  // sample lies `across` samples off along the other axis, `step` metres away.
  const double lead = alongX ? ux : uy, side = alongX ? uy : ux;
  const int sweep = lead >= 0.0 ? 1 : -1; // +1: the wind blows toward + along the sweep axis
  const double across = -side / std::fabs(lead);
  const double step = f.cell / std::fabs(lead);
  const int nLines = alongX ? C : R, nAlong = alongX ? R : C;
  auto at = [&](int line, int k) -> std::size_t {
    return alongX ? std::size_t(k) * C + line : std::size_t(line) * C + k;
  };
  for (int n = 0; n < nLines; ++n) {
    const int line = sweep > 0 ? n : nLines - 1 - n;
    const int up = line - sweep; // the column (row) upwind
    for (int k = 0; k < nAlong; ++k) {
      const std::size_t i = at(line, k);
      const double m = most[i];
      if (!(m > 0.0))
        continue; // dry: no fetch
      double upF;
      const double kk = double(k) + across;
      if (up < 0 || up >= nLines || kk < 0.0 || kk > double(nAlong - 1)) {
        upF = f.open_m; // past the grid's edge: the water goes on
      } else if (nAlong < 2) {
        upF = f.fetch_m[at(up, 0)];
      } else { // linear between the two samples one column upwind (kk is in [0, nAlong - 1])
        const int k0 = std::min(nAlong - 2, int(std::floor(kk)));
        const double w = kk - double(k0);
        upF = f.fetch_m[at(up, k0)] * (1.0 - w) + f.fetch_m[at(up, k0 + 1)] * w;
      }
      f.fetch_m[i] = float(std::min(upF + step, m));
    }
  }
  return f;
}

double spectrum_fetch(double wind_speed_mps, double fetch_m) {
  const double u = std::max(wind_speed_mps, 0.5);
  return std::max(std::min(fetch_m, 2.2e4 * u * u / gravity), 400.0);
}

double band_full_fetch(double wind_speed_mps, double k_min, double spectrum_fetch_m) {
  if (!(k_min > 0.0))
    return spectrum_fetch_m;
  const double u = std::max(wind_speed_mps, 0.5);
  // omega_p = 22 (g^2 / (U F))^(1/3) = omega_min  =>  F = g^2 / (U (omega_min / 22)^3).
  const double w = std::sqrt(gravity * k_min) / 22.0;
  const double F = gravity * gravity / (u * w * w * w);
  return std::min(F, spectrum_fetch_m);
}

double fetch_scale(double fetch_m, double full_m) {
  if (!(full_m > 0.0))
    return 1.0;
  return std::clamp(std::sqrt(std::max(fetch_m, 0.0) / full_m), 0.0, 1.0);
}

} // namespace ocean
} // namespace cvc
