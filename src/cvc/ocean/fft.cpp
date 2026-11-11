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

#include <cvc/algo/det_math.h>
#include <cvc/ocean/fft.h>
#include <stdexcept>
#include <string>

namespace cvc {
namespace ocean {

namespace det = ::cvc::algo::det; // the same bits on every platform (det_math.h)

fft_plan::fft_plan(int n) : _n(n) {
  if (n < 1 || (n & (n - 1)) != 0)
    throw std::invalid_argument("fft_plan: n must be a power of two, got " + std::to_string(n));
  int bits = 0;
  while ((1 << bits) < n)
    ++bits;
  _rev.resize(std::size_t(n));
  for (int j = 0; j < n; ++j) {
    int r = 0;
    for (int b = 0; b < bits; ++b)
      r |= ((j >> b) & 1) << (bits - 1 - b);
    _rev[std::size_t(j)] = r;
  }
  _cos.resize(std::size_t(n / 2));
  _sin.resize(std::size_t(n / 2));
  for (int j = 0; j < n / 2; ++j) {
    const double a = 6.283185307179586 * double(j) / double(n);
    _cos[std::size_t(j)] = det::cos(a);
    _sin[std::size_t(j)] = det::sin(a);
  }
}

void fft_plan::run(double *a, std::size_t stride, int sign) const {
  const std::size_t n = std::size_t(_n), s2 = 2 * stride;
  for (std::size_t j = 0; j < n; ++j) { // bit-reversal permutation
    const std::size_t r = std::size_t(_rev[j]);
    if (r > j) {
      double *p = a + j * s2, *q = a + r * s2;
      const double re = p[0], im = p[1];
      p[0] = q[0];
      p[1] = q[1];
      q[0] = re;
      q[1] = im;
    }
  }
  const double sg = sign >= 0 ? 1.0 : -1.0;
  for (std::size_t len = 2; len <= n; len <<= 1) { // Cooley-Tukey, decimation in time
    const std::size_t half = len / 2, step = n / len;
    for (std::size_t i = 0; i < n; i += len)
      for (std::size_t j = 0; j < half; ++j) {
        const double wr = _cos[j * step], wi = sg * _sin[j * step];
        double *p = a + (i + j) * s2, *q = a + (i + j + half) * s2;
        const double vr = q[0] * wr - q[1] * wi, vi = q[0] * wi + q[1] * wr;
        const double ur = p[0], ui = p[1];
        p[0] = ur + vr;
        p[1] = ui + vi;
        q[0] = ur - vr;
        q[1] = ui - vi;
      }
  }
}

void fft_plan::run_2d(double *a, int sign) const {
  const std::size_t n = std::size_t(_n);
  for (std::size_t r = 0; r < n; ++r)
    run(a + r * n * 2, 1, sign);
  // The columns, through a contiguous copy (the same arithmetic as a strided run, cache-friendly).
  std::vector<double> col(n * 2);
  for (std::size_t c = 0; c < n; ++c) {
    for (std::size_t r = 0; r < n; ++r) {
      col[r * 2] = a[(r * n + c) * 2];
      col[r * 2 + 1] = a[(r * n + c) * 2 + 1];
    }
    run(col.data(), 1, sign);
    for (std::size_t r = 0; r < n; ++r) {
      a[(r * n + c) * 2] = col[r * 2];
      a[(r * n + c) * 2 + 1] = col[r * 2 + 1];
    }
  }
}

} // namespace ocean
} // namespace cvc
