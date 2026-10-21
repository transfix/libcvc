/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph::rff: random Fourier features of a Matern kernel, the basis of cvc::ph's Gaussian
// process subnetworks (LieSPHGP's features.py):
//
//   phi_k(z) = sqrt(2 / n) cos(omega_k . z + b_k),   omega_k = w_k / sqrt(s_k / (2 nu)) / ell,
//   w_k ~ N(0, I),  s_k ~ chi^2(2 nu),  b_k ~ U(0, 2 pi)
//
// the Student-t spectral sample of the Matern-nu kernel k(|z - z'| / ell). A GP is then
// GP(z) = phi(z)^T W (W is n x outputs). Every derivative is bounded and closed form:
//   d phi_k / dz = -sqrt(2 / n) sin(theta_k) omega_k,  d^2 phi_k / dz^2 = -phi_k omega_k omega_k^T,
// which is what the reverse passes and the filter's Jacobians use.

#ifndef __CVC_PH_RFF_H__
#define __CVC_PH_RFF_H__

#include <cmath>
#include <cvc/ph/rng.h>
#include <vector>

namespace cvc {
namespace ph {

struct rff {
  int input = 0;                 // the input's width
  int count = 0;                 // n, the features
  std::vector<double> frequency; // [count x input], omega_k in rows (1 / ell folded in)
  std::vector<double> phase;     // [count]

  // n features of a Matern-nu kernel with length scale ell on `input` dimensions.
  static rff draw(rng &r, int input, int count, double nu, double ell) {
    rff f;
    f.input = input;
    f.count = count;
    f.frequency.resize(static_cast<std::size_t>(count) * input);
    f.phase.resize(count);
    const double degrees = 2.0 * nu;
    for (int k = 0; k < count; ++k) {
      const double s = r.chi_square(degrees);
      const double inv = 1.0 / (std::sqrt(s / degrees) * ell);
      for (int d = 0; d < input; ++d)
        f.frequency[static_cast<std::size_t>(k) * input + d] = r.normal() * inv;
    }
    for (int k = 0; k < count; ++k)
      f.phase[k] = 6.283185307179586476925286766559 * r.uniform();
    return f;
  }

  double amplitude() const { return std::sqrt(2.0 / count); }

  // theta_k = omega_k . z + b_k, for k < count.
  template <class T> void angles(const T *z, T *theta) const {
    for (int k = 0; k < count; ++k) {
      const double *w = &frequency[static_cast<std::size_t>(k) * input];
      T t = static_cast<T>(phase[k]);
      for (int d = 0; d < input; ++d)
        t += static_cast<T>(w[d]) * z[d];
      theta[k] = t;
    }
  }

  // out[o] = sum_k phi_k(z) W[k][o] for o < outputs; W row-major [count x outputs].
  template <class T> void gp(const T *z, const T *W, int outputs, T *out, T *theta) const {
    using std::cos;
    angles(z, theta);
    const T a = static_cast<T>(amplitude());
    for (int o = 0; o < outputs; ++o)
      out[o] = T(0);
    for (int k = 0; k < count; ++k) {
      const T phi = a * cos(theta[k]);
      const T *row = W + static_cast<std::size_t>(k) * outputs;
      for (int o = 0; o < outputs; ++o)
        out[o] += phi * row[o];
    }
  }

  // The reverse of gp at z (theta from the forward): for dout = dLoss/dout,
  //   dW[k][o] += phi_k dout[o],   dz += sum_k (dphi_k/dz) (W[k] . dout).
  // dW or dz may be null.
  template <class T>
  void gp_vjp(const T *theta, const T *W, int outputs, const T *dout, T *dW, T *dz) const {
    using std::cos;
    using std::sin;
    const T a = static_cast<T>(amplitude());
    for (int k = 0; k < count; ++k) {
      const T *row = W + static_cast<std::size_t>(k) * outputs;
      if (dW) {
        const T phi = a * cos(theta[k]);
        T *drow = dW + static_cast<std::size_t>(k) * outputs;
        for (int o = 0; o < outputs; ++o)
          drow[o] += phi * dout[o];
      }
      if (dz) {
        T dot = T(0);
        for (int o = 0; o < outputs; ++o)
          dot += row[o] * dout[o];
        const T g = -a * sin(theta[k]) * dot;
        const double *w = &frequency[static_cast<std::size_t>(k) * input];
        for (int d = 0; d < input; ++d)
          dz[d] += g * static_cast<T>(w[d]);
      }
    }
  }
};

} // namespace ph
} // namespace cvc

#endif // __CVC_PH_RFF_H__
