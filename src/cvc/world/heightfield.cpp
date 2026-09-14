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

#include <cmath>
#include <cvc/lsys/rng.h>
#include <cvc/world/heightfield.h>

namespace cvc {
namespace world {

namespace {

double smootherstep(double t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }

// Value in [-1,1] at an integer lattice point, keyed on the terrain stream.
double lattice(std::uint64_t seed, long ix, long iy) {
  double u = cvc::lsys::uni(seed, cvc::lsys::stream::terrain,
                            cvc::lsys::cell_id((std::int32_t)ix, (std::int32_t)iy), 0);
  return u * 2.0 - 1.0;
}

// 2D value noise in [-1,1], period-free (lattice keyed on absolute cell id).
double vnoise(std::uint64_t seed, double x, double y) {
  double fx = std::floor(x), fy = std::floor(y);
  long ix = (long)fx, iy = (long)fy;
  double tx = smootherstep(x - fx), ty = smootherstep(y - fy);
  double v00 = lattice(seed, ix, iy);
  double v10 = lattice(seed, ix + 1, iy);
  double v01 = lattice(seed, ix, iy + 1);
  double v11 = lattice(seed, ix + 1, iy + 1);
  double a = v00 + (v10 - v00) * tx;
  double b = v01 + (v11 - v01) * tx;
  return a + (b - a) * ty;
}

double fbm(std::uint64_t seed, double x, double y, int octaves, double lac, double gain) {
  double sum = 0, amp = 1, norm = 0, fx = x, fy = y;
  for (int o = 0; o < octaves; ++o) {
    sum += amp * vnoise(seed + 0x9E37u * (unsigned)o, fx, fy);
    norm += amp;
    amp *= gain;
    fx *= lac;
    fy *= lac;
  }
  return norm > 0 ? sum / norm : 0.0;
}

} // namespace

double heightfield::sample(double x, double y) const {
  double wx = x, wy = y;
  if (p_.warp_m > 0.0 && p_.warp_wavelength_m > 0.0) {
    double sxx = x / p_.warp_wavelength_m, syy = y / p_.warp_wavelength_m;
    wx += p_.warp_m * vnoise(p_.seed ^ 0x1111u, sxx, syy);
    wy += p_.warp_m * vnoise(p_.seed ^ 0x2222u, sxx + 5.2, syy + 1.3);
  }
  const double s = p_.base_wavelength_m > 0 ? 1.0 / p_.base_wavelength_m : 1.0;
  const double f = fbm(p_.seed, wx * s, wy * s, p_.octaves, p_.lacunarity, p_.gain); // [-1,1]

  if (p_.island) {
    double r2 = x * x + y * y;
    double R2 = p_.island_radius_m * p_.island_radius_m;
    double m = r2 < R2 ? 1.0 - r2 / R2 : 0.0;
    m = m * m * m; // Wyvill compact falloff
    // Dome dominates near centre; shelf pulls the edges below sea level, so an
    // island genuinely has water around it. relief (signed) rides the dome.
    return p_.sea_level_m + p_.shelf_m * (1.0 - m) + (p_.island_peak_m * m) + (p_.amp_m * f) * m;
  }
  // Rolling LAND: rise from sea level so the default scene is solvable terrain,
  // not half-submerged. Occupancy then comes from props (trees/buildings/rocks)
  // and only genuinely steep cells, never a spurious waterline at h == 0.
  return p_.sea_level_m + p_.amp_m * (0.5 + 0.5 * f);
}

double heightfield::slope_deg(double x, double y, double h_m) const {
  double dzdx = (sample(x + h_m, y) - sample(x - h_m, y)) / (2.0 * h_m);
  double dzdy = (sample(x, y + h_m) - sample(x, y - h_m)) / (2.0 * h_m);
  double g = std::sqrt(dzdx * dzdx + dzdy * dzdy);
  return std::atan(g) * 57.29577951308232;
}

} // namespace world
} // namespace cvc
