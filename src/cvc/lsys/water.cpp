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

// water.cpp — the travelling-wave sea depth field (see water.h). Pure math, no RNG. Ported in
// shape from the demo.

#include <algorithm>
#include <cmath>
#include <vector>

#include <cvc/lsys/water.h>

namespace cvc {
namespace lsys {

double sea_surface(double x, double y, double t, double sea_level, double wave_amp) {
  struct Wave {
    double hx, hy, len, omega, amp;
  };
  static const Wave W[] = {{0.86, 0.51, 58.0, 0.52, 1.00},
                           {-0.30, 0.95, 37.0, 0.93, 0.55},
                           {0.99, -0.16, 26.0, 1.37, 0.32},
                           {0.42, 0.91, 71.0, 0.40, 0.62}};
  double h = 0.0, peak = 0.0;
  for (const Wave &w : W) {
    const double k = 2.0 * M_PI / w.len;
    const double s = 0.5 + 0.5 * std::sin(k * (w.hx * x + w.hy * y) - w.omega * t);
    h += w.amp * std::pow(s, 2.4); // crest: pinch peaks, broaden troughs
    peak = w.amp > peak ? w.amp : peak;
  }
  double crest = h / (peak > 1e-9 ? peak : 1.0);
  if (crest > 1.0)
    crest = 1.0;
  return sea_level + wave_amp * (crest - 0.72);
}

void sea_field(const sea_params &p, const std::vector<float> &terrain, double t,
               std::vector<float> &out) {
  const int N = p.n, NZ = p.nz;
  if (N <= 0 || NZ <= 0) {
    out.clear();
    return;
  }
  const double floorZ = sea_floor(p), topZ = sea_top(p);
  const bool haveTerrain = terrain.size() == static_cast<std::size_t>(N) * N;
  out.assign(static_cast<std::size_t>(N) * N * NZ, 0.0f);
  for (int k = 0; k < NZ; ++k) {
    const double z = floorZ + (topZ - floorZ) * k / (NZ - 1);
    for (int j = 0; j < N; ++j)
      for (int i = 0; i < N; ++i) {
        const double x = -p.half + 2.0 * p.half * i / (N - 1);
        const double y = -p.half + 2.0 * p.half * j / (N - 1);
        const double surf = sea_surface(x, y, t, p.sea_level, p.wave_amp);
        const double ground = haveTerrain ? terrain[j * N + i] : floorZ;
        const double below = surf - z, above = z - ground;
        out[(static_cast<std::size_t>(k) * N + j) * N + i] =
            (below > 0.0 && above > 0.0)
                ? static_cast<float>(std::min(1.0, std::max(0.0, below / 6.0)))
                : 0.0f;
      }
  }
}

} // namespace lsys
} // namespace cvc
