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

// h0.h — a cascade's initial wave amplitudes h0(k), on the CPU, from the shared spectrum
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md, B1 and R3).
//
//   std::vector<cvc::ocean::cascade> cs = cvc::ocean::make_cascades({251, 83, 29}, seed);
//   cvc::ocean::h0_field h = cvc::ocean::build_h0(256, cs[i], sea);  // upload to the GPU
//
// An ocean is a few FFT tiles of different sizes (cascades), summed: the long swell in a large
// tile, ripples in a small one. Each cascade carries its own wavenumber band, and the bands
// partition the wavenumbers, so the sum holds the spectrum once (no band counted two or three
// times), and each cascade draws its own noise from its own seed (no two cascades sharing a
// pattern). The GPU ocean evolves and draws h0 as it comes; the physics band will evolve the same
// numbers on the CPU, so the two agree by construction.
//
// h0(k) = g(k) sqrt(S(k)) dk x amplitude inside the band and 0 outside it, with g a unit complex
// Gaussian pair from the seed, S the spectrum (spectrum.h) and dk = 2 pi / tile. Deterministic: the
// same inputs give the same bits in every run. Not yet the same bits on every platform (std::log,
// exp, sin and cos are libm's); the physics band (the roadmap's B2, O6) needs that, from
// correctly rounded versions of them.

#ifndef CVC_OCEAN_H0_H
#define CVC_OCEAN_H0_H

#include <cstddef>
#include <cstdint>
#include <cvc/ocean/spectrum.h>
#include <vector>

namespace cvc {
namespace ocean {

struct cascade {
  double tile_m = 250.0; // the FFT tile's side (metres)
  double k_min = 0.0;    // its wavenumber band, [k_min, k_max) rad/m
  double k_max = 1.0e300;
  std::uint64_t seed = 0; // its noise
  double amplitude = 1.0; // scales its waves (an artistic weight, seen alike by every consumer)
};

// Cascades for tiles from largest to smallest, the band edge between cascade i and i + 1 at four
// times tile i + 1's fundamental (8 pi / tile, ABYSSAL's): the next tile down takes over while its
// shortest wave of the band is still several samples across. Seeds are drawn from `seed`.
std::vector<cascade> make_cascades(const std::vector<double> &tiles_m, std::uint64_t seed);

// The wavenumber of texel (c, r) of an n x n tile: 2 pi / tile x ((c, r) - n / 2).
inline double texel_k(int i, int n, double tile_m) {
  return 6.283185307179586 / tile_m * (double(i) - 0.5 * double(n));
}

struct h0_field {
  int n = 0;
  double tile_m = 0.0;
  // n x n texels, row-major (row r is ky's index, column c kx's), 4 floats each: h0(k) (re, im)
  // and conj h0(-k) (re, im), the layout OceanFFT's time evolution reads.
  std::vector<float> texels;

  const float *texel(int c, int r) const { return &texels[(std::size_t(r) * n + c) * 4]; }
};

// h0 for cascade c of an n x n tile (n a power of two) under sea state s.
h0_field build_h0(int n, const cascade &c, const sea_state &s);

} // namespace ocean
} // namespace cvc

#endif // CVC_OCEAN_H0_H
