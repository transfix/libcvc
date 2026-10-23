/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph::rng: the random numbers cvc::ph draws (random-feature frequencies, initial parameters,
// reparameterization draws, batches, observation noise).
//
// std::normal_distribution and std::gamma_distribution are not specified bit for bit, and
// libstdc++, libc++ and MSVC give different sequences for the same engine and seed. These are
// written out, so a seed means the same draws on every compiler: xoshiro256** seeded through
// splitmix64, Box-Muller normals, Marsaglia-Tsang gammas. (The transcendental functions can still
// differ in the last ulp between C libraries.)

#ifndef __CVC_PH_RNG_H__
#define __CVC_PH_RNG_H__

#include <cmath>
#include <cstdint>

namespace cvc {
namespace ph {

class rng {
public:
  explicit rng(std::uint64_t seed = 0) { reseed(seed); }

  void reseed(std::uint64_t seed) {
    std::uint64_t x = seed;
    for (auto &w : s_)
      w = splitmix64(x);
    has_spare_ = false;
  }

  // A stream derived from this seed and a tag: independent draws for each purpose (features,
  // initial values, batches, ...) that do not shift when another purpose draws more.
  static rng derive(std::uint64_t seed, std::uint64_t tag) {
    std::uint64_t x = seed ^ (0x9e3779b97f4a7c15ULL * (tag + 1));
    return rng(splitmix64(x));
  }

  std::uint64_t next() {
    const std::uint64_t result = rotl(s_[1] * 5, 7) * 9;
    const std::uint64_t t = s_[1] << 17;
    s_[2] ^= s_[0];
    s_[3] ^= s_[1];
    s_[1] ^= s_[2];
    s_[0] ^= s_[3];
    s_[2] ^= t;
    s_[3] = rotl(s_[3], 45);
    return result;
  }

  // Uniform on [0, 1), 53 bits.
  double uniform() { return static_cast<double>(next() >> 11) * 0x1.0p-53; }

  // Uniform integer in [0, n).
  std::uint64_t below(std::uint64_t n) {
    // Lemire's rejection keeps it unbiased.
    const std::uint64_t threshold = (0 - n) % n;
    for (;;) {
      const std::uint64_t x = next();
      if (x >= threshold)
        return x % n;
    }
  }

  // Standard normal (Box-Muller; the second value of each pair is kept for the next call).
  double normal() {
    if (has_spare_) {
      has_spare_ = false;
      return spare_;
    }
    double u1 = uniform();
    while (u1 <= 0.0)
      u1 = uniform();
    const double u2 = uniform();
    const double radius = std::sqrt(-2.0 * std::log(u1));
    const double angle = 6.283185307179586476925286766559 * u2;
    spare_ = radius * std::sin(angle);
    has_spare_ = true;
    return radius * std::cos(angle);
  }

  // Gamma(shape, 1) by Marsaglia and Tsang; shape < 1 by the boost U^(1/shape).
  double gamma(double shape) {
    if (shape < 1.0) {
      double u = uniform();
      while (u <= 0.0)
        u = uniform();
      return gamma(shape + 1.0) * std::pow(u, 1.0 / shape);
    }
    const double d = shape - 1.0 / 3.0;
    const double c = 1.0 / std::sqrt(9.0 * d);
    for (;;) {
      double x, v;
      do {
        x = normal();
        v = 1.0 + c * x;
      } while (v <= 0.0);
      v = v * v * v;
      const double u = uniform();
      if (u < 1.0 - 0.0331 * x * x * x * x)
        return d * v;
      if (u > 0.0 && std::log(u) < 0.5 * x * x + d * (1.0 - v + std::log(v)))
        return d * v;
    }
  }

  // Chi-square with k degrees of freedom.
  double chi_square(double k) { return 2.0 * gamma(0.5 * k); }

private:
  static std::uint64_t rotl(std::uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
  static std::uint64_t splitmix64(std::uint64_t &x) {
    std::uint64_t z = (x += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
  }
  std::uint64_t s_[4];
  double spare_ = 0.0;
  bool has_spare_ = false;
};

} // namespace ph
} // namespace cvc

#endif // __CVC_PH_RNG_H__
