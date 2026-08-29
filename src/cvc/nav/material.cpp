/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// material.cpp — see material.h. BIT-identical twin of the Python normative
// reference (GRL-SNAM grl_snam/material.py): the derived-plane pipeline
// (material_build) and the float64 witness gate. This TU is compiled with
// -ffp-contract=off (set in src/cvc/CMakeLists.txt): the blur's float64
// accumulation with irrational exp-derived weights WOULD contract to FMA on
// aarch64 and change the f32-stored planes against the cross-platform
// goldens. Do not remove that flag, and do not "tidy" any arithmetic here —
// op order IS the contract.

#include <algorithm>
#include <cmath>
#include <cvc/nav/detail/grid_math.h>
#include <cvc/nav/detail/parallel.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/material.h>
#include <limits>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace nav {

namespace {

// The 16 gate ray directions (row, col) = (sin th, cos th), th = 2*pi*k/16 —
// the SAME exact float64 constants as the Python reference's _DIRS_16 table
// (libm sin/cos stays out of the BIT contract). Generated once; do not tidy.
const double kDirs16[16][2] = {
    {0.0, 1.0},
    {0.3826834323650898, 0.9238795325112867},
    {0.7071067811865475, 0.7071067811865476},
    {0.9238795325112867, 0.38268343236508984},
    {1.0, 6.123233995736766e-17},
    {0.9238795325112867, -0.3826834323650897},
    {0.7071067811865476, -0.7071067811865475},
    {0.3826834323650899, -0.9238795325112867},
    {1.2246467991473532e-16, -1.0},
    {-0.38268343236508967, -0.9238795325112868},
    {-0.7071067811865475, -0.7071067811865477},
    {-0.9238795325112865, -0.38268343236509034},
    {-1.0, -1.8369701987210297e-16},
    {-0.9238795325112866, 0.38268343236509},
    {-0.7071067811865477, 0.7071067811865474},
    {-0.3826834323650904, 0.9238795325112865},
};

// scipy 'reflect' (edge-repeating / symmetric) index map, single bounce —
// matches the reference's _reflect_idx exactly.
inline int reflect_idx(int idx, int n) {
  if (idx < 0)
    idx = -idx - 1;
  if (idx >= n)
    idx = 2 * n - 1 - idx;
  return idx;
}

// One ray of the witness gate (the reference's _ray_risk): sequential f64
// accumulation; the float point is bounds-checked BEFORE rounding; risk is
// recorded BEFORE the hard/clearance break. Cells: rint (half-even) + clip.
struct ray_out {
  double mean;
  bool feasible;
  double min_clear;
};

ray_out ray_risk(const float *risk, const std::uint8_t *hard, const float *clear_m, int rows,
                 int cols, double pr, double pc, double dr, double dc, int horizon_cells,
                 double hard_margin_m) {
  double acc = 0.0;
  int count = 0;
  double min_clear = std::numeric_limits<double>::infinity();
  bool feasible = true;
  for (int t = 1; t <= horizon_cells; ++t) {
    const double qr = pr + t * dr;
    const double qc = pc + t * dc;
    if (!(qr >= 0.0 && qr < static_cast<double>(rows) && qc >= 0.0 &&
          qc < static_cast<double>(cols))) {
      feasible = false;
      break;
    }
    int r = static_cast<int>(std::rint(qr));
    if (r < 0)
      r = 0;
    if (r > rows - 1)
      r = rows - 1;
    int c = static_cast<int>(std::rint(qc));
    if (c < 0)
      c = 0;
    if (c > cols - 1)
      c = cols - 1;
    const long idx = static_cast<long>(r) * cols + c;
    acc += static_cast<double>(risk[idx]);
    ++count;
    const double cl = static_cast<double>(clear_m[idx]);
    if (cl < min_clear)
      min_clear = cl;
    if (hard[idx] || cl < hard_margin_m) {
      feasible = false;
      break;
    }
  }
  ray_out o;
  o.mean = count ? acc / static_cast<double>(count) : std::numeric_limits<double>::infinity();
  o.feasible = feasible;
  o.min_clear = min_clear;
  return o;
}

} // namespace

// ── derived-plane pipeline ──────────────────────────────────────────────────

std::vector<float> material_planes::stacked() const {
  const std::size_t hw = static_cast<std::size_t>(rows) * cols;
  std::vector<float> out(6 * hw);
  const std::vector<float> *planes[6] = {&risk, &phi_m, &grad_rx, &grad_ry, &grad_px, &grad_py};
  for (int ch = 0; ch < 6; ++ch)
    std::copy(planes[ch]->begin(), planes[ch]->end(), out.begin() + ch * hw);
  return out;
}

material_planes material_build(const float *risk_raw, const std::uint8_t *hard, int rows, int cols,
                               double cell_w, double scale, double sigma) {
  const std::size_t hw = static_cast<std::size_t>(rows) * cols;
  material_planes out;
  out.rows = rows;
  out.cols = cols;

  // 1. risk = clip(gaussian_blur(risk_raw, sigma), 0, 1), f32 store.
  //    Taps: exp(-0.5/(sigma*sigma) * k*k), radius int(4*sigma + 0.5),
  //    sequential normalization sum — the reference's gaussian_kernel verbatim.
  out.risk.resize(hw);
  if (sigma <= 0.0) {
    for (std::size_t i = 0; i < hw; ++i)
      out.risk[i] = risk_raw[i];
  } else {
    const int radius = static_cast<int>(4.0 * sigma + 0.5);
    if (rows < radius + 1 || cols < radius + 1)
      throw std::invalid_argument("material_build: grid smaller than blur radius + 1");
    const int taps = 2 * radius + 1;
    std::vector<double> w(taps);
    const double inv = -0.5 / (sigma * sigma);
    double total = 0.0;
    for (int j = 0; j < taps; ++j) {
      const int k = j - radius;
      w[j] = std::exp(inv * static_cast<double>(k * k));
      total += w[j]; // sequential — never a pairwise sum
    }
    for (int j = 0; j < taps; ++j)
      w[j] = w[j] / total;

    // Separable conv, f64 through BOTH passes (rows axis then cols axis), the
    // per-element accumulation sequential in tap order — the same rounding
    // sequence as the reference's `acc = acc + w[j] * take(...)` loop.
    std::vector<double> buf(hw), acc(hw);
    for (std::size_t i = 0; i < hw; ++i)
      buf[i] = static_cast<double>(risk_raw[i]);
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c) {
        double a = 0.0;
        for (int j = 0; j < taps; ++j) {
          const int rr = reflect_idx(r + (j - radius), rows);
          a += w[j] * buf[static_cast<std::size_t>(rr) * cols + c];
        }
        acc[static_cast<std::size_t>(r) * cols + c] = a;
      }
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c) {
        double a = 0.0;
        for (int j = 0; j < taps; ++j) {
          const int cc = reflect_idx(c + (j - radius), cols);
          a += w[j] * acc[static_cast<std::size_t>(r) * cols + cc];
        }
        buf[static_cast<std::size_t>(r) * cols + c] = a;
      }
    for (std::size_t i = 0; i < hw; ++i) {
      float v = static_cast<float>(buf[i]);  // one f32 store...
      v = std::min(std::max(v, 0.0f), 1.0f); // ...then the clip, in f32
      out.risk[i] = v;
    }
  }

  // 2. phi_m = sqrt(edt2(hard)) * cell_w — f64 chain, one f32 store.
  const std::vector<double> d2 = edt2_squared(hard, rows, cols);
  out.phi_m.resize(hw);
  for (std::size_t i = 0; i < hw; ++i)
    out.phi_m[i] = static_cast<float>(std::sqrt(d2[i]) * cell_w);

  // 3. gradients of the f32-STORED planes (np.gradient / grad1d, f32),
  //    divided by the f32-cast denominators (float32(cell_w*scale) / cell_w).
  const float denom_n = static_cast<float>(cell_w * scale);
  const float denom_w = static_cast<float>(cell_w);
  out.grad_rx.resize(hw);
  out.grad_ry.resize(hw);
  out.grad_px.resize(hw);
  out.grad_py.resize(hw);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      const std::size_t i = static_cast<std::size_t>(r) * cols + c;
      out.grad_rx[i] =
          detail::grad1d(out.risk.data() + static_cast<std::size_t>(r) * cols, c, cols, 1) /
          denom_n;
      out.grad_ry[i] = detail::grad1d(out.risk.data() + c, r, rows, cols) / denom_n;
      out.grad_px[i] =
          detail::grad1d(out.phi_m.data() + static_cast<std::size_t>(r) * cols, c, cols, 1) /
          denom_w;
      out.grad_py[i] = detail::grad1d(out.phi_m.data() + c, r, rows, cols) / denom_w;
    }
  return out;
}

// ── witness gate ────────────────────────────────────────────────────────────

gate_decision witness_gate(const float *risk, const std::uint8_t *gate_hard, const float *clear_m,
                           int rows, int cols, double pos_r, double pos_c, double goal_r,
                           double goal_c, const gate_params &p) {
  const double dgr = goal_r - pos_r;
  const double dgc = goal_c - pos_c;
  // sqrt(x*x + y*y) — the reference deliberately avoids hypot (CPython's
  // hypot is its own correctly-rounded algorithm, not libm's).
  const double norm = std::sqrt(dgr * dgr + dgc * dgc);
  double ndr = 0.0, ndc = 0.0;
  if (!(norm < 1e-8)) {
    ndr = dgr / norm;
    ndc = dgc / norm;
  }
  const ray_out nom = ray_risk(risk, gate_hard, clear_m, rows, cols, pos_r, pos_c, ndr, ndc,
                               p.horizon_cells, p.hard_margin_m);

  double best = std::numeric_limits<double>::infinity();
  double best_dr = 0.0, best_dc = 0.0;
  double best_clear = std::numeric_limits<double>::quiet_NaN();
  int feasible_count = 0;
  for (int k = 0; k < p.primitive_count; ++k) {
    double dr, dc;
    if (p.primitive_count == 16) {
      dr = kDirs16[k][0];
      dc = kDirs16[k][1];
    } else {                                // outside the BIT contract, as in the reference
      const double tau = 6.283185307179586; // 2*pi (M_PI is not portable MSVC)
      const double a = tau * static_cast<double>(k) / static_cast<double>(p.primitive_count);
      dr = std::sin(a);
      dc = std::cos(a);
    }
    const double er = pos_r + p.horizon_cells * dr;
    const double ec = pos_c + p.horizon_cells * dc;
    const double per = goal_r - er;
    const double pec = goal_c - ec;
    if (std::sqrt(per * per + pec * pec) >= norm - p.progress_slack_cells)
      continue;
    const ray_out cand = ray_risk(risk, gate_hard, clear_m, rows, cols, pos_r, pos_c, dr, dc,
                                  p.horizon_cells, p.hard_margin_m);
    if (!cand.feasible)
      continue;
    ++feasible_count;
    if (cand.mean < best) {
      best = cand.mean;
      best_dr = dr;
      best_dc = dc;
      best_clear = cand.min_clear;
    }
  }
  gate_decision g;
  // inf - inf is NaN, and NaN >= margin is false — the reference's semantics.
  g.active = feasible_count > 0 && nom.mean >= p.material_trigger &&
             nom.mean - best >= p.improvement_margin;
  g.nominal_risk = nom.mean;
  g.best_risk = best;
  g.feasible_count = feasible_count;
  g.dir_r = best_dr;
  g.dir_c = best_dc;
  g.end_r = pos_r + p.horizon_cells * best_dr;
  g.end_c = pos_c + p.horizon_cells * best_dc;
  g.min_clearance_m = best_clear;
  return g;
}

void witness_gate_batch(const float *risk, const std::uint8_t *gate_hard, const float *clear_m,
                        int rows, int cols, const double *pos_rc, const double *goal_rc, int n,
                        const gate_params &p, std::uint8_t *active_out, double *nominal_out,
                        double *best_out, std::int32_t *count_out, int num_threads,
                        const int *map_id) {
  // Agents are independent; the batch is n serial gates fanned across threads,
  // byte-identical to the serial loop by construction. Each agent gates against
  // its own material plane (map_id[i]); map_id == nullptr collapses to plane 0.
  const std::size_t plane_sz = static_cast<std::size_t>(rows) * cols;
  detail::parallel_for(n, num_threads, [&](int i) {
    const std::size_t off = map_id ? static_cast<std::size_t>(map_id[i]) * plane_sz : 0;
    const gate_decision g =
        witness_gate(risk + off, gate_hard + off, clear_m + off, rows, cols, pos_rc[2 * i],
                     pos_rc[2 * i + 1], goal_rc[2 * i], goal_rc[2 * i + 1], p);
    active_out[i] = g.active ? 1 : 0;
    nominal_out[i] = g.nominal_risk;
    best_out[i] = g.best_risk;
    count_out[i] = g.feasible_count;
  });
}

// ── 6-channel sampler ───────────────────────────────────────────────────────
// The sdf_sample coordinate chain and bilinear weights (drive.cpp), applied to
// the 6-channel material stack. Kept in this TU so it compiles under
// -ffp-contract=off with the rest of the material math.

namespace detail {

void material_sample_point(const material_stack &m, int plane, float onx, float ony, float &risk,
                           float &phi, float &grx, float &gry, float &gpx, float &gpy) {
  const float S = static_cast<float>(m.S);
  const float wx = onx / S + static_cast<float>(m.cx);
  const float wy = ony / S + static_cast<float>(m.cy);
  const float gxc =
      2.0f * (wx - static_cast<float>(m.mnx)) / static_cast<float>(m.mxx - m.mnx) - 1.0f;
  const float gyc =
      2.0f * (wy - static_cast<float>(m.mny)) / static_cast<float>(m.mxy - m.mny) - 1.0f;
  const float Wf1 = static_cast<float>(m.W - 1);
  const float Hf1 = static_cast<float>(m.H - 1);
  float ix = (gxc + 1.0f) * 0.5f * Wf1;
  float iy = (gyc + 1.0f) * 0.5f * Hf1;
  ix = std::min(std::max(ix, 0.0f), Wf1);
  iy = std::min(std::max(iy, 0.0f), Hf1);
  const int ix0 = static_cast<int>(std::floor(ix));
  const int iy0 = static_cast<int>(std::floor(iy));
  const float wx1 = ix - static_cast<float>(ix0);
  const float wx0 = 1.0f - wx1;
  const float wy1 = iy - static_cast<float>(iy0);
  const float wy0 = 1.0f - wy1;
  const float nw = wx0 * wy0, ne = wx1 * wy0, sw = wx0 * wy1, se = wx1 * wy1;
  const int cx0 = std::min(std::max(ix0, 0), m.W - 1);
  const int cx1 = std::min(std::max(ix0 + 1, 0), m.W - 1);
  const int cy0 = std::min(std::max(iy0, 0), m.H - 1);
  const int cy1 = std::min(std::max(iy0 + 1, 0), m.H - 1);
  const long HW = static_cast<long>(m.H) * m.W;
  const float *pl = m.data + static_cast<long>(plane) * 6 * HW;
  const long nwi = static_cast<long>(cy0) * m.W + cx0;
  const long nei = static_cast<long>(cy0) * m.W + cx1;
  const long swi = static_cast<long>(cy1) * m.W + cx0;
  const long sei = static_cast<long>(cy1) * m.W + cx1;
  float ch[6];
  for (int c = 0; c < 6; ++c) {
    const float *pc = pl + static_cast<long>(c) * HW;
    ch[c] = pc[nwi] * nw + pc[nei] * ne + pc[swi] * sw + pc[sei] * se;
  }
  risk = ch[0];
  phi = ch[1];
  grx = ch[2];
  gry = ch[3];
  gpx = ch[4];
  gpy = ch[5];
}

} // namespace detail

void material_sample(const material_stack &m, const float *on, int n, const int *map_id,
                     float *risk_out, float *phi_out, float *grad_r_out, float *grad_phi_out,
                     int num_threads) {
  detail::parallel_for(n, num_threads, [&](int i) {
    const int plane = (map_id && m.M > 1) ? map_id[i] : 0;
    float rk, ph, grx, gry, gpx, gpy;
    detail::material_sample_point(m, plane, on[2 * i], on[2 * i + 1], rk, ph, grx, gry, gpx, gpy);
    risk_out[i] = rk;
    phi_out[i] = ph;
    grad_r_out[2 * i] = grx;
    grad_r_out[2 * i + 1] = gry;
    grad_phi_out[2 * i] = gpx;
    grad_phi_out[2 * i + 1] = gpy;
  });
}

} // namespace nav
} // namespace cvc
