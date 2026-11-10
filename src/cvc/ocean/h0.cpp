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

// h0.cpp — a cascade's initial amplitudes (see h0.h).

#include <algorithm>
#include <cmath>
#include <cvc/ocean/h0.h>
#include <stdexcept>
#include <string>

namespace cvc {
namespace ocean {

namespace {

constexpr double kTau = 6.283185307179586;

std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9e3779b97f4a7c15ull;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
  return x ^ (x >> 31);
}

// A unit Gaussian pair for texel (c, r) of a tile seeded with `seed` (Box-Muller from a hash).
void gaussian(std::uint64_t seed, int c, int r, double &gx, double &gy) {
  const std::uint64_t h =
      splitmix64(seed ^ splitmix64((std::uint64_t(std::uint32_t(r)) << 32) | std::uint32_t(c)));
  // Two 32-bit uniforms: u1 in (0, 1], u2 in [0, 1).
  const double u1 = (double(std::uint32_t(h >> 32)) + 1.0) / 4294967296.0;
  const double u2 = double(std::uint32_t(h)) / 4294967296.0;
  const double rad = std::sqrt(-2.0 * std::log(u1));
  gx = rad * std::cos(kTau * u2);
  gy = rad * std::sin(kTau * u2);
}

} // namespace

std::vector<cascade> make_cascades(const std::vector<double> &tiles_m, std::uint64_t seed) {
  std::vector<cascade> cs(tiles_m.size());
  for (std::size_t i = 0; i < cs.size(); ++i) {
    cs[i].tile_m = tiles_m[i];
    cs[i].seed = splitmix64(seed + 0x632be59bd9b4e019ull * (i + 1));
    if (i > 0)
      cs[i].k_min = cs[i - 1].k_max = 4.0 * kTau / tiles_m[i];
  }
  return cs;
}

h0_field build_h0(int n, const cascade &c, const sea_state &s) {
  if (n < 2 || (n & (n - 1)) != 0)
    throw std::invalid_argument("build_h0: n must be a power of two, got " + std::to_string(n));
  if (!(c.tile_m > 0.0))
    throw std::invalid_argument("build_h0: the tile must be positive");
  h0_field f;
  f.n = n;
  f.tile_m = c.tile_m;
  const std::size_t count = std::size_t(n) * n;
  const double dk = kTau / c.tile_m;
  std::vector<double> h(count * 2, 0.0); // h0(k) at each texel
  for (int r = 0; r < n; ++r) {
    const double ky = texel_k(r, n, c.tile_m);
    for (int col = 0; col < n; ++col) {
      const double kx = texel_k(col, n, c.tile_m);
      const double kLen = std::hypot(kx, ky);
      if (kLen <= 1e-6 || kLen < c.k_min || kLen >= c.k_max)
        continue;
      const double S = spectrum(kx, ky, s);
      if (!(S > 0.0))
        continue;
      double gx, gy;
      gaussian(c.seed, col, r, gx, gy);
      const double amp = std::sqrt(S) * dk * c.amplitude;
      const std::size_t i = (std::size_t(r) * n + col) * 2;
      h[i] = gx * amp;
      h[i + 1] = gy * amp;
    }
  }
  // The conjugate partner: texel (c, r) also holds conj h0(-k), -k at ((n - c) % n, (n - r) % n).
  f.texels.assign(count * 4, 0.0f);
  for (int r = 0; r < n; ++r)
    for (int col = 0; col < n; ++col) {
      const std::size_t i = std::size_t(r) * n + col;
      const std::size_t m = std::size_t((n - r) % n) * n + std::size_t((n - col) % n);
      f.texels[i * 4] = float(h[i * 2]);
      f.texels[i * 4 + 1] = float(h[i * 2 + 1]);
      f.texels[i * 4 + 2] = float(h[m * 2]);
      f.texels[i * 4 + 3] = float(-h[m * 2 + 1]);
    }
  return f;
}

} // namespace ocean
} // namespace cvc
