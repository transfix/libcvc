/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// seeded_draw.h: whole numbers and orderings from a seeded std::mt19937 that are the same on
// every standard library. std::uniform_int_distribution and std::shuffle leave their algorithms
// to each library, so the same seed scattered other agents, drew other training routes and
// clustered other groups on macOS and wasm (libc++) than on Linux (libstdc++) and Windows (MSVC).
// Written out here, a seed is one scene everywhere.

#ifndef CVC_NAV_DETAIL_SEEDED_DRAW_H
#define CVC_NAV_DETAIL_SEEDED_DRAW_H

#include <algorithm>
#include <cstdint>
#include <random>

namespace cvc {
namespace nav {
namespace detail {

// A whole number in [0, n) from rng, 0 < n < 2^32: the high word of draw * n, redrawn while the
// low word falls in the short range (Lemire's multiply and reject). libstdc++ and MSVC map a
// 32-bit draw this way, so their std::uniform_int_distribution(0, n - 1) gives the same numbers
// from the same draws; libc++ masks a draw's low bits and rejects.
inline std::uint32_t pick_below(std::mt19937 &rng, std::uint32_t n) {
  std::uint64_t m = std::uint64_t{static_cast<std::uint32_t>(rng())} * n;
  if (static_cast<std::uint32_t>(m) < n) {
    const std::uint32_t threshold = (0u - n) % n;
    while (static_cast<std::uint32_t>(m) < threshold)
      m = std::uint64_t{static_cast<std::uint32_t>(rng())} * n;
  }
  return static_cast<std::uint32_t>(m >> 32);
}

inline int pick_below(std::mt19937 &rng, int n) {
  return static_cast<int>(pick_below(rng, static_cast<std::uint32_t>(n)));
}

// [first, last) in an order drawn from rng: libstdc++'s std::shuffle, written out. Fisher-Yates
// from the front, element i swapped with one of [0, i] from pick_below; while n * n fits in 32
// bits, two swaps come from one draw, x in [0, (i + 1)(i + 2)) giving x / (i + 2) for i and
// x % (i + 2) for i + 1, after a lone first swap when n is even. MSVC's std::shuffle maps each
// draw by its remainder instead, and libc++'s swaps from the other end, so each library ordered a
// seed's elements its own way.
template <class RandomIt> void seeded_shuffle(RandomIt first, RandomIt last, std::mt19937 &rng) {
  const std::uint32_t n = static_cast<std::uint32_t>(last - first);
  if (n < 2)
    return;
  if (n > 65535u) { // n * n would overflow 32 bits: one draw per swap
    for (std::uint32_t i = 1; i < n; ++i)
      std::iter_swap(first + i, first + pick_below(rng, i + 1));
    return;
  }
  std::uint32_t i = 1;
  if (n % 2 == 0) {
    std::iter_swap(first + 1, first + pick_below(rng, 2u));
    i = 2;
  }
  for (; i < n; i += 2) {
    const std::uint32_t x = pick_below(rng, (i + 1) * (i + 2));
    std::iter_swap(first + i, first + x / (i + 2));
    std::iter_swap(first + i + 1, first + x % (i + 2));
  }
}

} // namespace detail
} // namespace nav
} // namespace cvc

#endif // CVC_NAV_DETAIL_SEEDED_DRAW_H
