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

// fft.h — a deterministic radix-2 FFT (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md, B1 and B2).
//
//   cvc::ocean::fft_plan plan(64);
//   plan.run_2d(data, +1); // data: 64 x 64 complex values, (re, im) interleaved, row-major
//
// The physics band evolves the sea's longest waves on the CPU every step and transforms them
// here, and physics folds the result into its state_hash, which a native run and a wasm run must
// agree on. So the transform gives the same bits on every platform: its twiddles come from
// cvc::algo::det (det_math.h), never libm, and it is built without FMA contraction; the rest is
// IEEE additions and multiplications in a fixed order.

#ifndef CVC_OCEAN_FFT_H
#define CVC_OCEAN_FFT_H

#include <cstddef>
#include <vector>

namespace cvc {
namespace ocean {

class fft_plan {
public:
  // A plan for n-point transforms, n a power of two. Throws std::invalid_argument otherwise.
  explicit fft_plan(int n);

  int size() const noexcept { return _n; }

  // In place over n complex values `stride` complex values apart, (re, im) interleaved:
  // a[p] <- sum over m of a[m] exp(sign 2 pi i m p / n), unnormalized. sign +1 is the transform
  // OceanFFT's butterflies run (wavenumbers to positions); -1 the opposite.
  void run(double *re_im, std::size_t stride, int sign) const;
  // n x n values, row-major: every row, then every column.
  void run_2d(double *re_im, int sign) const;

private:
  int _n = 0;
  std::vector<double> _cos, _sin; // cos and sin of 2 pi j / n, j < n / 2
  std::vector<int> _rev;          // the bit-reversed index of each j < n
};

} // namespace ocean
} // namespace cvc

#endif // CVC_OCEAN_FFT_H
