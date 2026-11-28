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

#include <cstdint>
#include <cstring>
#include <cvc/algo/det_math.h>
#include <cvc/ocean/fft.h>
#include <stdexcept>
#include <string>

namespace cvc {
namespace ocean {

namespace det = ::cvc::algo::det; // the same bits on every platform (det_math.h)

namespace {
// Whether all n doubles are +0 (every bit clear).
bool all_plus_zero(const double *a, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    std::uint64_t b;
    std::memcpy(&b, a + i, sizeof b);
    if (b != 0)
      return false;
  }
  return true;
}
} // namespace

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

void fft_plan::run_block(double *re, double *im, int sign) const {
  const std::size_t n = std::size_t(_n), B = kBlock;
  const double sg = sign >= 0 ? 1.0 : -1.0;
  const double *cs = _cos.data(), *sn = _sin.data();
  for (std::size_t len = 2; len <= n; len <<= 1) { // run()'s stages, kBlock transforms abreast
    const std::size_t half = len / 2, step = n / len;
    for (std::size_t i = 0; i < n; i += len)
      for (std::size_t j = 0; j < half; ++j) {
        const double wr = cs[j * step], wi = sg * sn[j * step];
        double *pr = re + (i + j) * B, *pi = im + (i + j) * B;
        double *qr = re + (i + j + half) * B, *qi = im + (i + j + half) * B;
        for (std::size_t b = 0; b < B; ++b) { // run()'s butterfly, operation for operation
          const double vr = qr[b] * wr - qi[b] * wi, vi = qr[b] * wi + qi[b] * wr;
          const double ur = pr[b], ui = pi[b];
          pr[b] = ur + vr;
          pi[b] = ui + vi;
          qr[b] = ur - vr;
          qi[b] = ui - vi;
        }
      }
  }
}

void fft_plan::run_2d(double *a, int sign) const {
  const std::size_t n = std::size_t(_n);
  if (n < kBlock) { // too small to block: run() over every row, then every column
    for (std::size_t r = 0; r < n; ++r)
      run(a + r * n * 2, 1, sign);
    for (std::size_t c = 0; c < n; ++c)
      run(a + c * 2, n, sign);
    return;
  }
  // kBlock rows or columns at a time, each gathered in bit-reversed order (run()'s permutation:
  // place j takes element rev[j]) into a split real / imaginary buffer, kBlock values to a place,
  // transformed abreast by run()'s butterflies in run()'s order, and put back. Each value meets
  // the same operations as under run(), so the result is the same bits; the inner loop over the
  // block vectorizes. (Through raw pointers: an unoptimized build calls std::vector's operator[]
  // for every element, and these loops touch each value twice a pass.)
  std::vector<double> re_buf(n * kBlock), im_buf(n * kBlock);
  double *re = re_buf.data(), *im = im_buf.data();
  const int *rev = _rev.data();
  // The rows. A row of +0 transforms to itself bit for bit (every butterfly adds a +-0 product to
  // +0 and subtracts it from +0, which gives +0), so a block of them is left as it is: a band of a
  // few wavenumbers (the sea's long cascades) fills only a few rows.
  for (std::size_t r0 = 0; r0 < n; r0 += kBlock) {
    if (all_plus_zero(a + r0 * n * 2, kBlock * n * 2))
      continue;
    for (std::size_t j = 0; j < n; ++j) {
      const std::size_t src = std::size_t(rev[j]) * 2;
      for (std::size_t b = 0; b < kBlock; ++b) {
        re[j * kBlock + b] = a[(r0 + b) * n * 2 + src];
        im[j * kBlock + b] = a[(r0 + b) * n * 2 + src + 1];
      }
    }
    run_block(re, im, sign);
    for (std::size_t j = 0; j < n; ++j)
      for (std::size_t b = 0; b < kBlock; ++b) {
        a[(r0 + b) * n * 2 + j * 2] = re[j * kBlock + b];
        a[(r0 + b) * n * 2 + j * 2 + 1] = im[j * kBlock + b];
      }
  }
  // The columns.
  for (std::size_t c0 = 0; c0 < n; c0 += kBlock) {
    for (std::size_t j = 0; j < n; ++j) {
      const double *row = a + std::size_t(rev[j]) * n * 2 + c0 * 2;
      for (std::size_t b = 0; b < kBlock; ++b) {
        re[j * kBlock + b] = row[b * 2];
        im[j * kBlock + b] = row[b * 2 + 1];
      }
    }
    run_block(re, im, sign);
    for (std::size_t j = 0; j < n; ++j) {
      double *row = a + j * n * 2 + c0 * 2;
      for (std::size_t b = 0; b < kBlock; ++b) {
        row[b * 2] = re[j * kBlock + b];
        row[b * 2 + 1] = im[j * kBlock + b];
      }
    }
  }
}

} // namespace ocean
} // namespace cvc
