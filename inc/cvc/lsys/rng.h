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

// rng.h — hashed (noise-based) RNG for cvc::lsys / cvc::world.
//
// This is the single most important determinism decision in the module and it
// is a direct repair of the lsystem_forest predecessor, which walks ONE
// std::mt19937(20260817) sequentially for x/y/size/maturity/phase/sway inside a
// placement loop — so adding one tree, changing MAX_TREES, or hitting the
// below-waterline `continue` reshuffles every subsequent draw. The world there
// is a function of iteration order.
//
// Here every random value is `hash4(master, stream, element, draw)` — a pure
// function of four integers, with NO stream state and NO order dependency
// [Eiserloh 2017, "Noise-Based RNG", GDC]. Two consequences the tests enforce:
//
//   * INSERTION STABILITY. `element` is a STABLE identity (a Morton-packed cell
//     coordinate, or a derivation path_id) — NEVER a loop counter — so adding a
//     prop or a rule application cannot move any other one.
//   * STREAM INDEPENDENCE. `stream` is a named enum: perturbing the `sway`
//     stream provably cannot move a tree placed from the `placement` stream.
//     This is the "salt audit" canary (cosmetic streams must not touch the
//     exported contract planes).
//
// There is deliberately no std::mt19937 anywhere in cvc::lsys or cvc::world; a
// grep test enforces it.

#ifndef CVC_LSYS_RNG_H
#define CVC_LSYS_RNG_H

#include <cmath>
#include <cstdint>

namespace cvc {
namespace lsys {

// Named random streams. Independent by construction (folded into the hash as a
// distinct salt), so one stream's draws cannot correlate with another's.
enum class stream : std::uint32_t {
  placement = 0,
  species,
  maturity,
  size,
  phase,
  sway,
  rule_choice,
  param_jitter,
  surface,
  terrain,
  hydrology,
  building,
  floorplan,
  props,
  cloud,
  rock,
  _count
};

namespace detail {

// splitmix64 finalizer — a well-tested 64-bit avalanche mix.
constexpr std::uint64_t splitmix(std::uint64_t z) noexcept {
  z += 0x9E3779B97F4A7C15ull;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}

} // namespace detail

// 4-round mix of the four coordinates. Stateless, order-independent, and a pure
// function of its arguments. The stream and draw salts are distinct large odd
// constants so that (stream, draw) pairs decorrelate.
constexpr std::uint64_t hash4(std::uint64_t master, std::uint32_t strm, std::uint64_t element,
                              std::uint32_t draw) noexcept {
  std::uint64_t h = detail::splitmix(master ^ 0xA24BAED4963EE407ull);
  h = detail::splitmix(h ^ (std::uint64_t(strm) * 0x9E3779B97F4A7C15ull));
  h = detail::splitmix(h ^ element);
  h = detail::splitmix(h ^ (std::uint64_t(draw) * 0xD1B54A32D192ED03ull));
  return h;
}

// Uniform in [0, 1). Uses the top 53 bits so the mantissa is filled exactly.
constexpr double uni(std::uint64_t master, stream s, std::uint64_t el, std::uint32_t d) noexcept {
  const std::uint64_t h = hash4(master, static_cast<std::uint32_t>(s), el, d);
  return double(h >> 11) * (1.0 / 9007199254740992.0); // / 2^53
}

// Uniform in [lo, hi).
constexpr double uni(std::uint64_t master, stream s, std::uint64_t el, std::uint32_t d, double lo,
                     double hi) noexcept {
  return lo + (hi - lo) * uni(master, s, el, d);
}

// Standard-normal-derived draw with mean mu and standard deviation sigma
// (Box–Muller from two independent uniforms at draws 2d and 2d+1). Not constexpr
// because std::log/std::sqrt/std::cos are not constexpr in C++17.
inline double nrand(std::uint64_t master, stream s, std::uint64_t el, std::uint32_t d, double mu,
                    double sigma) noexcept {
  const double u1 = uni(master, s, el, 2u * d);
  const double u2 = uni(master, s, el, 2u * d + 1u);
  // Guard the log against u1 == 0 (uni is in [0,1), so 0 is reachable).
  const double r = std::sqrt(-2.0 * std::log(u1 > 0.0 ? u1 : 2.220446049250313e-16));
  const double z = r * std::cos(6.283185307179586476925286766559 * u2);
  return mu + sigma * z;
}

// Uniform integer in [lo, hi] inclusive.
constexpr int irand(std::uint64_t master, stream s, std::uint64_t el, std::uint32_t d, int lo,
                    int hi) noexcept {
  if (hi <= lo)
    return lo;
  const std::uint64_t span = std::uint64_t(hi - lo) + 1ull;
  const std::uint64_t h = hash4(master, static_cast<std::uint32_t>(s), el, d);
  return lo + static_cast<int>(h % span);
}

// ── Stable element ids. NEVER a loop counter. ────────────────────────────────

// Morton (Z-order) interleave of two 16-bit lanes.
constexpr std::uint64_t morton2(std::uint32_t x, std::uint32_t y) noexcept {
  auto spread = [](std::uint32_t v) -> std::uint64_t {
    std::uint64_t w = v & 0xFFFFull;
    w = (w | (w << 16)) & 0x0000FFFF0000FFFFull;
    w = (w | (w << 8)) & 0x00FF00FF00FF00FFull;
    w = (w | (w << 4)) & 0x0F0F0F0F0F0F0F0Full;
    w = (w | (w << 2)) & 0x3333333333333333ull;
    w = (w | (w << 1)) & 0x5555555555555555ull;
    return w;
  };
  return spread(x) | (spread(y) << 1);
}

// A stable id for a scatter cell (or any signed 2D grid coordinate). Signed
// coordinates are zig-zag mapped so negatives stay in the low bits. The zig-zag
// is done in the UNSIGNED domain (shifting a negative int is UB before C++20).
// Note morton2 keeps only the low 16 bits of each zig-zagged lane, so distinct
// ids are guaranteed for coordinates in [-32767, 32767] — far beyond any grid
// this module uses (scatter cells and terrain lattice indices are < 10^3).
constexpr std::uint64_t cell_id(std::int32_t gx, std::int32_t gy) noexcept {
  const std::uint32_t zx =
      (static_cast<std::uint32_t>(gx) << 1) ^ static_cast<std::uint32_t>(gx >> 31);
  const std::uint32_t zy =
      (static_cast<std::uint32_t>(gy) << 1) ^ static_cast<std::uint32_t>(gy >> 31);
  return morton2(zx, zy);
}

// A stable id for a module along the derivation DAG: the parent's id mixed with
// the child index. Independent of the flat output-vector position.
constexpr std::uint64_t path_id(std::uint64_t parent, std::uint32_t child) noexcept {
  return detail::splitmix(parent ^
                          (std::uint64_t(child) * 0x9E3779B97F4A7C15ull + 0x2545F4914F6CDD1Dull));
}

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_RNG_H
