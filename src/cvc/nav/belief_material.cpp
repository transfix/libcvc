/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// belief_material.cpp — the belief-material kernels (belief_material.h). BIT tier:
// built with -ffp-contract=off (CMake), and every float op below is pinned to its
// numpy twin's order. Do not "tidy" the arithmetic.

#include <algorithm>
#include <cmath>
#include <cvc/nav/belief_material.h>
#include <cvc/nav/grid_nav.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

void validate(const belief_material_params &p) {
  if (!std::isfinite(p.unknown_risk) || p.unknown_risk < 0.0f || p.unknown_risk > 1.0f)
    throw std::invalid_argument("cvc::nav::belief_material_params: unknown_risk must be in [0, 1]");
  if (p.unknown_hard > 1)
    throw std::invalid_argument("cvc::nav::belief_material_params: unknown_hard must be 0 or 1");
  if (!std::isfinite(p.material_range_m) || p.material_range_m < 0.0)
    throw std::invalid_argument(
        "cvc::nav::belief_material_params: material_range_m must be finite and >= 0");
}

namespace {
void check_grid(int rows, int cols, const char *who) {
  if (rows < 1 || cols < 1)
    throw std::invalid_argument(std::string("cvc::nav::") + who + ": empty grid");
}
} // namespace

void belief_material_inputs(const float *risk_raw, const std::uint8_t *hard,
                            const std::uint8_t *seen, int rows, int cols,
                            const belief_material_params &p, float *risk_out,
                            std::uint8_t *hard_out) {
  validate(p);
  check_grid(rows, cols, "belief_material_inputs");
  const std::size_t hw = static_cast<std::size_t>(rows) * cols;
  for (std::size_t k = 0; k < hw; ++k) {
    const bool s = seen[k] != 0;
    risk_out[k] = s ? risk_raw[k] : p.unknown_risk;
    hard_out[k] = s ? static_cast<std::uint8_t>(hard[k] != 0) : p.unknown_hard;
  }
}

void belief_gate_surface(const std::uint8_t *hard_b, const std::uint8_t *occ,
                         const std::uint8_t *seen, int rows, int cols, double cell_w,
                         const belief_material_params &p, std::uint8_t *gate_hard_out,
                         float *clear_m_out) {
  validate(p);
  check_grid(rows, cols, "belief_gate_surface");
  const std::size_t hw = static_cast<std::size_t>(rows) * cols;
  for (std::size_t k = 0; k < hw; ++k) {
    const bool g = hard_b[k] != 0 || (occ && occ[k] != 0) || (p.unknown_gate_hard && seen[k] == 0);
    gate_hard_out[k] = g ? 1 : 0;
  }
  const std::vector<double> d2 = edt2_squared(gate_hard_out, rows, cols);
  for (std::size_t k = 0; k < hw; ++k)
    clear_m_out[k] = static_cast<float>(std::sqrt(d2[k]) * cell_w);
}

namespace {
// np.linspace(lo, hi, n)[i]: (i * step) + lo, the last sample exactly hi.
inline double linspace_at(double lo, double hi, int n, int i) {
  if (n > 1 && i == n - 1)
    return hi;
  const double step = n > 1 ? (hi - lo) / static_cast<double>(n - 1) : 0.0;
  return static_cast<double>(i) * step + lo;
}
} // namespace

bool seen_disc(double px, double py, double R, int rows, int cols, double min_x, double min_y,
               double max_x, double max_y, std::uint8_t *seen) {
  check_grid(rows, cols, "seen_disc");
  if (!(R >= 0.0) || !std::isfinite(R) || !std::isfinite(px) || !std::isfinite(py))
    return false;
  const double RR = R * R;
  // A conservative index window around the disc, widened by one sample each side;
  // the exact test below decides every cell inside it, so the window only saves work.
  auto window = [](double lo, double hi, int n, double centre, double R, int &i0, int &i1) {
    if (n < 2) {
      i0 = 0;
      i1 = n - 1;
      return;
    }
    const double step = (hi - lo) / static_cast<double>(n - 1);
    if (!(step > 0.0)) {
      i0 = 0;
      i1 = n - 1;
      return;
    }
    const double a = std::floor((centre - R - lo) / step) - 1.0;
    const double b = std::ceil((centre + R - lo) / step) + 1.0;
    i0 = static_cast<int>(std::max(0.0, std::min(a, static_cast<double>(n - 1))));
    i1 = static_cast<int>(std::max(0.0, std::min(b, static_cast<double>(n - 1))));
  };
  int c0, c1, r0, r1;
  window(min_x, max_x, cols, px, R, c0, c1);
  window(min_y, max_y, rows, py, R, r0, r1);
  bool changed = false;
  for (int r = r0; r <= r1; ++r) {
    const double dy = linspace_at(min_y, max_y, rows, r) - py;
    const double dy2 = dy * dy;
    for (int c = c0; c <= c1; ++c) {
      const double dx = linspace_at(min_x, max_x, cols, c) - px;
      const double dx2 = dx * dx;
      if (dx2 + dy2 <= RR) {
        std::uint8_t &s = seen[static_cast<std::size_t>(r) * cols + c];
        if (s != 1) {
          s = 1;
          changed = true;
        }
      }
    }
  }
  return changed;
}

void material_patch_belief(const float *risk, const std::uint8_t *hard_b, const std::uint8_t *seen,
                           int rows, int cols, double min_x, double min_y, double max_x,
                           double max_y, double px, double py, double extent_m, int P,
                           const belief_material_params &p, float *out) {
  validate(p);
  check_grid(rows, cols, "material_patch_belief");
  if (P < 2)
    throw std::invalid_argument("cvc::nav::material_patch_belief: P must be >= 2");
  if (!std::isfinite(extent_m) || !(extent_m > 0.0))
    throw std::invalid_argument("cvc::nav::material_patch_belief: extent_m must be positive");
  const std::size_t pp = static_cast<std::size_t>(P) * P;
  float *ch0 = out, *ch1 = out + pp, *ch2 = out + 2 * pp;
  const double half = extent_m / 2.0;
  for (int i = 0; i < P; ++i) {
    const double y = py + linspace_at(-half, half, P, i);
    const double rr = (y - min_y) / (max_y - min_y) * (rows - 1);
    for (int j = 0; j < P; ++j) {
      const double x = px + linspace_at(-half, half, P, j);
      const double cc = (x - min_x) / (max_x - min_x) * (cols - 1);
      const std::size_t o = static_cast<std::size_t>(i) * P + j;
      const double nr = std::rint(rr), nc = std::rint(cc);
      const bool on = nr >= 0.0 && nr < rows && nc >= 0.0 && nc < cols;
      const std::size_t k =
          on ? static_cast<std::size_t>(nr) * cols + static_cast<std::size_t>(nc) : 0;
      const bool s = on && seen[k] != 0;
      ch2[o] = s ? 1.0f : 0.0f;
      ch1[o] = s ? (hard_b[k] ? 1.0f : 0.0f) : static_cast<float>(p.unknown_hard);
      if (!s) {
        ch0[o] = p.unknown_risk;
        continue;
      }
      const double rc = std::min(std::max(rr, 0.0), static_cast<double>(rows - 1));
      const double ccl = std::min(std::max(cc, 0.0), static_cast<double>(cols - 1));
      const int r0 = static_cast<int>(std::floor(rc)), c0 = static_cast<int>(std::floor(ccl));
      const int r1 = std::min(r0 + 1, rows - 1), c1 = std::min(c0 + 1, cols - 1);
      const double fr = rc - r0, fc = ccl - c0;
      const double v00 = risk[static_cast<std::size_t>(r0) * cols + c0];
      const double v01 = risk[static_cast<std::size_t>(r0) * cols + c1];
      const double v10 = risk[static_cast<std::size_t>(r1) * cols + c0];
      const double v11 = risk[static_cast<std::size_t>(r1) * cols + c1];
      const double top = v00 + (v01 - v00) * fc;
      const double bot = v10 + (v11 - v10) * fc;
      ch0[o] = static_cast<float>(top + (bot - top) * fr);
    }
  }
}

} // namespace nav
} // namespace cvc
