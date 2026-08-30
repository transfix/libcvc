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

// coef_train.cpp — see coef_train.h. Self-supervised CoefMLP training with
// hand-written reverse-mode adjoints (no libtorch). The differentiable rollout
// primitives live in detail/diff_rollout.h (shared verbatim with the CUDA
// trainer); this TU wraps them in the per-agent window loss+gradient, the Adam
// loop, the scene source and the .cvcnav bake. The rollout integrator (point-mass
// surrogate vs full bicycle) is selected by train_config::rollout; everything
// else is identical. Built without -ffast-math so the float32 forward tracks the
// deployment math; the backward is validated by a finite-difference gradcheck.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/detail/diff_rollout.h>
#include <cvc/nav/grid_nav.h>
#include <random>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace nav {

namespace {
diff::field to_diff_field(const field_stack &fs) {
  diff::field F;
  F.data = fs.data;
  F.H = fs.H;
  F.W = fs.W;
  F.S = static_cast<float>(fs.S);
  F.cx = static_cast<float>(fs.cx);
  F.cy = static_cast<float>(fs.cy);
  F.mnx = static_cast<float>(fs.mnx);
  F.mny = static_cast<float>(fs.mny);
  F.mxx = static_cast<float>(fs.mxx);
  F.mxy = static_cast<float>(fs.mxy);
  return F;
}
} // namespace

// ── training_scene ───────────────────────────────────────────────────────────

field_stack training_scene::field() const {
  field_stack fs;
  fs.data = field_data.data();
  fs.M = 1;
  fs.H = rows;
  fs.W = cols;
  fs.mnx = min_x;
  fs.mny = min_y;
  fs.mxx = max_x;
  fs.mxy = max_y;
  fs.cx = cx;
  fs.cy = cy;
  fs.S = scale;
  return fs;
}

void training_scene::build() {
  const long hw = static_cast<long>(rows) * cols;
  // SDF field, no clip (matches coef_train.py's SDFField built straight from
  // build_sdf).
  const sdf_field f = build_sdf(occ.data(), rows, cols, min_x, min_y, max_x, max_y, scale);
  field_data.resize(3 * hw);
  for (long i = 0; i < hw; ++i) {
    field_data[i] = f.phi[i];
    field_data[hw + i] = f.normal_x[i];
    field_data[2 * hw + i] = f.normal_y[i];
  }
  // Largest 8-connected free component (starts/goals drawn from it are mutually
  // reachable — the point of coef_train's free_components pick).
  std::vector<int> label(hw, -1);
  int best_label = -1;
  std::size_t best_size = 0;
  std::vector<int> stack;
  int next = 0;
  for (long seed = 0; seed < hw; ++seed) {
    if (occ[seed] || label[seed] >= 0)
      continue;
    const int lab = next++;
    std::size_t sz = 0;
    stack.clear();
    stack.push_back(static_cast<int>(seed));
    label[seed] = lab;
    while (!stack.empty()) {
      const int cur = stack.back();
      stack.pop_back();
      ++sz;
      const int r = cur / cols, c = cur % cols;
      for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc) {
          if (dr == 0 && dc == 0)
            continue;
          const int nr = r + dr, nc = c + dc;
          if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
            continue;
          const int ni = nr * cols + nc;
          if (occ[ni] || label[ni] >= 0)
            continue;
          label[ni] = lab;
          stack.push_back(ni);
        }
    }
    if (sz > best_size) {
      best_size = sz;
      best_label = lab;
    }
  }
  free_cells.clear();
  if (best_label < 0) { // fully blocked: fall back to all cells
    for (long i = 0; i < hw; ++i)
      free_cells.push_back(static_cast<int>(i));
  } else {
    for (long i = 0; i < hw; ++i)
      if (label[i] == best_label)
        free_cells.push_back(static_cast<int>(i));
  }
}

void training_scene::sample_starts_goals(int n, unsigned seed, float *o, float *goal) const {
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> pick(0, static_cast<int>(free_cells.size()) - 1);
  auto to_on = [&](int cell, float &onx, float &ony) {
    const int r = cell / cols, c = cell % cols;
    const double x = min_x + static_cast<double>(c) / (cols - 1) * (max_x - min_x);
    const double y = min_y + static_cast<double>(r) / (rows - 1) * (max_y - min_y);
    onx = static_cast<float>((x - cx) * scale);
    ony = static_cast<float>((y - cy) * scale);
  };
  for (int i = 0; i < n; ++i) {
    to_on(free_cells[pick(rng)], o[2 * i], o[2 * i + 1]);
    to_on(free_cells[pick(rng)], goal[2 * i], goal[2 * i + 1]);
  }
}

// ── scene factories ──────────────────────────────────────────────────────────

training_scene occupancy_scene(const std::uint8_t *occ, int rows, int cols, double min_x,
                               double min_y, double max_x, double max_y, double scale, float rr,
                               float d_hat, float dt, float vmax) {
  training_scene s;
  s.rows = rows;
  s.cols = cols;
  s.occ.assign(occ, occ + static_cast<long>(rows) * cols);
  s.min_x = min_x;
  s.min_y = min_y;
  s.max_x = max_x;
  s.max_y = max_y;
  s.cx = 0.0;
  s.cy = 0.0;
  s.scale = scale;
  s.rr = rr;
  s.d_hat = d_hat;
  s.dt = dt;
  s.vmax = vmax;
  s.build();
  return s;
}

training_scene city_scene(int grid) {
  // city_blocks(96, rows=3, cols=3, gap=9, margin=14), then shrunk to `grid`.
  const int base = 96, brows = 3, bcols = 3, gap = 9, margin = 14;
  const double fscale = static_cast<double>(grid) / base;
  const int span = base - 2 * margin;
  const int rp = span / brows, cp = span / bcols;
  const int rb = std::max(2, rp - gap), cb = std::max(2, cp - gap);
  std::vector<std::uint8_t> occ(static_cast<long>(grid) * grid, 0);
  auto scv = [&](int v) { return static_cast<int>(std::lround(v * fscale)); };
  for (int i = 0; i < brows; ++i)
    for (int j = 0; j < bcols; ++j) {
      const int r0 = margin + i * rp, c0 = margin + j * cp;
      const int R0 = scv(r0), R1 = scv(std::min(r0 + rb, base));
      const int C0 = scv(c0), C1 = scv(std::min(c0 + cb, base));
      for (int r = std::max(0, R0); r < std::min(grid, R1); ++r)
        for (int c = std::max(0, C0); c < std::min(grid, C1); ++c)
          occ[r * grid + c] = 1;
    }
  // The city story's meta (fog_stories.py): bounds ±100, scale 0.05, rr 0.15, etc.
  return occupancy_scene(occ.data(), grid, grid, -100.0, -100.0, 100.0, 100.0, 0.05, 0.15f, 0.35f,
                         0.06f, 0.9f);
}

// ── coef_trainer ─────────────────────────────────────────────────────────────

coef_trainer::coef_trainer(const train_config &cfg, unsigned init_seed) : cfg_(cfg) {
  hidden_ = cfg.hidden;
  const int h = hidden_;
  lrows_[0] = h;
  lcols_[0] = 5;
  lrows_[1] = h;
  lcols_[1] = h;
  lrows_[2] = 3;
  lcols_[2] = h;
  int off = 0;
  for (int L = 0; L < 3; ++L) {
    off_w_[L] = off;
    off += lrows_[L] * lcols_[L];
    off_b_[L] = off;
    off += lrows_[L];
  }
  p_.assign(off, 0.0f);
  m_.assign(off, 0.0f);
  u_.assign(off, 0.0f);
  // Kaiming-uniform-ish init, seeded: break symmetry (zero-init would leave the
  // hidden units identical forever). Biases start at 0. Weights ~ U(-k, k),
  // k = 1/sqrt(fan_in). This is NOT torch's exact init — training doesn't need
  // it (the finite-difference gradcheck, not torch-parity, is the correctness
  // gate) — only a good, deterministic starting basin.
  std::mt19937 rng(init_seed);
  for (int L = 0; L < 3; ++L) {
    const float k = 1.0f / std::sqrt(static_cast<float>(lcols_[L]));
    std::uniform_real_distribution<float> u(-k, k);
    for (int i = 0; i < lrows_[L] * lcols_[L]; ++i)
      p_[off_w_[L] + i] = u(rng);
  }
}

void coef_trainer::coeffs(const float *feat, int n, float *out) const {
  const int h = hidden_;
  std::vector<float> a(h), b(h);
  for (int s = 0; s < n; ++s) {
    const float *x = feat + static_cast<std::size_t>(s) * 5;
    for (int o = 0; o < h; ++o) {
      float acc = p_[off_b_[0] + o];
      const float *w = &p_[off_w_[0] + static_cast<std::size_t>(o) * 5];
      for (int i = 0; i < 5; ++i)
        acc += w[i] * x[i];
      a[o] = diff::siluf_(acc);
    }
    for (int o = 0; o < h; ++o) {
      float acc = p_[off_b_[1] + o];
      const float *w = &p_[off_w_[1] + static_cast<std::size_t>(o) * h];
      for (int i = 0; i < h; ++i)
        acc += w[i] * a[i];
      b[o] = diff::siluf_(acc);
    }
    float *oo = out + static_cast<std::size_t>(s) * 3;
    for (int o = 0; o < 3; ++o) {
      float acc = p_[off_b_[2] + o];
      const float *w = &p_[off_w_[2] + static_cast<std::size_t>(o) * h];
      for (int i = 0; i < h; ++i)
        acc += w[i] * b[i];
      oo[o] = diff::softplusf_(acc + std::log(std::expm1(bias_[o])));
    }
  }
}

double coef_trainer::loss_and_grad(const training_scene &scene, const float *o_in,
                                   const float *v_in, const float *goal, int n, int window,
                                   std::vector<float> *grad, float *o_out, float *v_out) const {
  const diff::field F = to_diff_field(scene.field());
  const int h = hidden_;
  const bool bike = cfg_.rollout == rollout_kind::bicycle;
  const float rr = scene.rr, d_hat = scene.d_hat, vmax = scene.vmax, hdt = scene.dt;
  diff::bike_veh bv;
  bv.rr = rr;
  bv.d_hat = d_hat;
  bv.vmax = vmax;
  bv.L = cfg_.veh_L;
  bv.delta_max = cfg_.veh_delta_max;
  bv.a_max = cfg_.veh_a_max;
  bv.a_lat_max = cfg_.veh_a_lat_max;
  bv.k_steer = cfg_.veh_k_steer;
  bv.hdt = hdt;
  bv.allow_reverse = cfg_.veh_allow_reverse ? 1 : 0;
  const float off_bias[3] = {std::log(std::expm1(bias_[0])), std::log(std::expm1(bias_[1])),
                             std::log(std::expm1(bias_[2]))};
  if (grad)
    grad->assign(p_.size(), 0.0f);

  struct Step {
    diff::sample cf; // coef_feats sample at o_t
    float dx, dy, gd, inv, gdx, gdy;
    float feat[5];
    std::vector<float> z0, a0, z1, a1;
    float raw[3], coef[3];
    float ox, oy, auxx, auxy; // o_t, aux_t
  };
  std::vector<Step> st(window);
  for (int L = 0; L < window; ++L) {
    st[L].z0.resize(h);
    st[L].a0.resize(h);
    st[L].z1.resize(h);
    st[L].a1.resize(h);
  }

  double total_loss = 0.0;
  const double inv_n = 1.0 / n;
  const float coll_w = cfg_.w_coll / static_cast<float>(window);
  // Both loss terms are normalized to O(1) so w_coll is a PREFERENCE and not a
  // unit conversion. Must match sdf_nav / coef_train.py's train_bicycle: a
  // native trainer optimising a different objective from the torch reference is
  // the same class of divergence the rollout parity gates exist to prevent, and
  // no gate would catch it -- both paths would be internally consistent while
  // descending different hills.
  const float region_n =
      static_cast<float>(0.5 * (scene.max_x - scene.min_x) * scene.scale); // world half-extent
  const float d_safe = cfg_.d_safe > 0.0f ? cfg_.d_safe : d_hat;
  const float inv_d_safe = 1.0f / d_safe;
  std::vector<float> gacc(grad ? p_.size() : 0, 0.0f);

  for (int ag = 0; ag < n; ++ag) {
    float ox = o_in[2 * ag], oy = o_in[2 * ag + 1];
    float ax = v_in[2 * ag], ay = v_in[2 * ag + 1]; // aux (v, or th/sp)
    const float gx = goal[2 * ag], gy = goal[2 * ag + 1];

    // ── forward window ──
    for (int t = 0; t < window; ++t) {
      Step &S = st[t];
      S.ox = ox;
      S.oy = oy;
      S.auxx = ax;
      S.auxy = ay;
      S.cf = diff::sample_fwd(F, ox, oy);
      S.dx = gx - ox;
      S.dy = gy - oy;
      S.gd = std::sqrt(S.dx * S.dx + S.dy * S.dy);
      S.inv = 1.0f / (S.gd + 1e-6f);
      S.gdx = S.dx * S.inv;
      S.gdy = S.dy * S.inv;
      S.feat[0] = S.cf.phi;
      S.feat[1] = S.gd;
      S.feat[2] = S.gdx;
      S.feat[3] = S.gdy;
      S.feat[4] = S.gdx * S.cf.nx + S.gdy * S.cf.ny;
      for (int o = 0; o < h; ++o) {
        float acc = p_[off_b_[0] + o];
        const float *w = &p_[off_w_[0] + static_cast<std::size_t>(o) * 5];
        for (int i = 0; i < 5; ++i)
          acc += w[i] * S.feat[i];
        S.z0[o] = acc;
        S.a0[o] = diff::siluf_(acc);
      }
      for (int o = 0; o < h; ++o) {
        float acc = p_[off_b_[1] + o];
        const float *w = &p_[off_w_[1] + static_cast<std::size_t>(o) * h];
        for (int i = 0; i < h; ++i)
          acc += w[i] * S.a0[i];
        S.z1[o] = acc;
        S.a1[o] = diff::siluf_(acc);
      }
      for (int o = 0; o < 3; ++o) {
        float acc = p_[off_b_[2] + o];
        const float *w = &p_[off_w_[2] + static_cast<std::size_t>(o) * h];
        for (int i = 0; i < h; ++i)
          acc += w[i] * S.a1[i];
        S.raw[o] = acc;
        S.coef[o] = diff::softplusf_(acc + off_bias[o]);
      }
      const float al = S.coef[0], be = S.coef[1], ga = S.coef[2];

      float ox1, oy1, ax1, ay1;
      if (bike)
        diff::bike_step(F, ox, oy, ax, ay, gx, gy, al, be, ga, bv, ox1, oy1, ax1, ay1);
      else
        diff::surr_step(F, ox, oy, ax, ay, gx, gy, al, be, ga, rr, d_hat, vmax, hdt, ox1, oy1, ax1,
                        ay1);

      const diff::sample cs = diff::sample_fwd(F, ox1, oy1);
      // Margin shortfall, not breach depth. A breach depth is nonzero only for
      // an agent already INSIDE geometry -- ~0% of a free-space batch -- so it
      // averaged to nothing and no weight could rescue it.
      const float pen = d_safe - (cs.phi - rr);
      if (pen > 0.0f)
        total_loss += static_cast<double>(coll_w) * pen * inv_d_safe * inv_n;

      ox = ox1;
      oy = oy1;
      ax = ax1;
      ay = ay1;
    }

    const float fdx = ox - gx, fdy = oy - gy;
    const float Lgoal = std::sqrt(fdx * fdx + fdy * fdy);
    total_loss += static_cast<double>(Lgoal / region_n) * inv_n;
    if (o_out) {
      o_out[2 * ag] = ox;
      o_out[2 * ag + 1] = oy;
    }
    if (v_out) {
      v_out[2 * ag] = ax;
      v_out[2 * ag + 1] = ay;
    }
    if (!grad)
      continue;

    // ── backward window ──
    float go_x = 0.0f, go_y = 0.0f, gaux_x = 0.0f, gaux_y = 0.0f; // grad on (o1, aux1)
    if (Lgoal > 1e-9f) {
      go_x = static_cast<float>(inv_n) * fdx / (Lgoal * region_n);
      go_y = static_cast<float>(inv_n) * fdy / (Lgoal * region_n);
    }
    for (int t = window - 1; t >= 0; --t) {
      Step &S = st[t];
      const float al = S.coef[0], be = S.coef[1], ga = S.coef[2];

      // collision term at o_{t+1} = the recomputable step output; but we stored
      // only inputs, so recompute o_{t+1} via the step for its sample.
      float ox1, oy1, ax1, ay1;
      if (bike)
        diff::bike_step(F, S.ox, S.oy, S.auxx, S.auxy, gx, gy, al, be, ga, bv, ox1, oy1, ax1, ay1);
      else
        diff::surr_step(F, S.ox, S.oy, S.auxx, S.auxy, gx, gy, al, be, ga, rr, d_hat, vmax, hdt,
                        ox1, oy1, ax1, ay1);
      const diff::sample cs = diff::sample_fwd(F, ox1, oy1);
      const float pen = d_safe - (cs.phi - rr);
      if (pen > 0.0f) {
        float dox, doy;
        // d(pen/d_safe)/d(phi) = -1/d_safe, so the seed is the old one scaled.
        diff::sample_bwd(cs, -static_cast<float>(coll_w) * inv_d_safe * static_cast<float>(inv_n),
                         0.0f, 0.0f, dox, doy);
        go_x += dox;
        go_y += doy;
      }

      // rollout backward: grad on (o1, aux1) -> grad on (o_t, aux_t) + (al,be,ga)
      float gox, goy, gauxx, gauxy, g_al, g_be, g_ga;
      if (bike)
        diff::bike_step_bwd(F, S.ox, S.oy, S.auxx, S.auxy, gx, gy, al, be, ga, bv, go_x, go_y,
                            gaux_x, gaux_y, gox, goy, gauxx, gauxy, g_al, g_be, g_ga);
      else
        diff::surr_step_bwd(F, S.ox, S.oy, S.auxx, S.auxy, gx, gy, al, be, ga, rr, d_hat, vmax, hdt,
                            go_x, go_y, gaux_x, gaux_y, gox, goy, gauxx, gauxy, g_al, g_be, g_ga);

      // MLP backward: (g_al,g_be,g_ga) -> gacc + grad feat
      float graw[3];
      graw[0] = g_al * diff::sigmoidf_(S.raw[0] + off_bias[0]);
      graw[1] = g_be * diff::sigmoidf_(S.raw[1] + off_bias[1]);
      graw[2] = g_ga * diff::sigmoidf_(S.raw[2] + off_bias[2]);
      std::vector<float> ga1(h, 0.0f);
      for (int o = 0; o < 3; ++o) {
        const std::size_t wb = off_w_[2] + static_cast<std::size_t>(o) * h;
        for (int i = 0; i < h; ++i) {
          gacc[wb + i] += graw[o] * S.a1[i];
          ga1[i] += graw[o] * p_[wb + i];
        }
        gacc[off_b_[2] + o] += graw[o];
      }
      std::vector<float> gz1(h);
      for (int i = 0; i < h; ++i)
        gz1[i] = ga1[i] * diff::silu_grad_(S.z1[i]);
      std::vector<float> ga0(h, 0.0f);
      for (int o = 0; o < h; ++o) {
        const std::size_t wb = off_w_[1] + static_cast<std::size_t>(o) * h;
        const float g = gz1[o];
        for (int i = 0; i < h; ++i) {
          gacc[wb + i] += g * S.a0[i];
          ga0[i] += g * p_[wb + i];
        }
        gacc[off_b_[1] + o] += g;
      }
      float gfeat[5] = {0, 0, 0, 0, 0};
      for (int o = 0; o < h; ++o) {
        const std::size_t wb = off_w_[0] + static_cast<std::size_t>(o) * 5;
        const float g = ga0[o] * diff::silu_grad_(S.z0[o]);
        for (int i = 0; i < 5; ++i) {
          gacc[wb + i] += g * S.feat[i];
          gfeat[i] += g * p_[wb + i];
        }
        gacc[off_b_[0] + o] += g;
      }

      // coef_feats backward: gfeat -> grad on o_t (add to the rollout's gox/goy)
      float god_x = gox, god_y = goy;
      const float gphi_cf = gfeat[0];
      const float ggd = gfeat[1];
      const float ggdx = gfeat[2] + gfeat[4] * S.cf.nx;
      const float ggdy = gfeat[3] + gfeat[4] * S.cf.ny;
      const float gcfnx = gfeat[4] * S.gdx, gcfny = gfeat[4] * S.gdy;
      const float gd_safe = S.gd > 1e-9f ? S.gd : 1e-9f;
      const float inv2 = S.inv * S.inv;
      float g_dx = ggd * S.dx / gd_safe;
      float g_dy = ggd * S.dy / gd_safe;
      g_dx += ggdx * (S.inv - S.dx * S.dx * inv2 / gd_safe);
      g_dy += ggdx * (-S.dx * S.dy * inv2 / gd_safe);
      g_dy += ggdy * (S.inv - S.dy * S.dy * inv2 / gd_safe);
      g_dx += ggdy * (-S.dx * S.dy * inv2 / gd_safe);
      god_x += -g_dx;
      god_y += -g_dy;
      float cdox, cdoy;
      diff::sample_bwd(S.cf, gphi_cf, gcfnx, gcfny, cdox, cdoy);
      god_x += cdox;
      god_y += cdoy;

      go_x = god_x;
      go_y = god_y;
      gaux_x = gauxx;
      gaux_y = gauxy;
    }
  }

  if (grad) {
    for (std::size_t i = 0; i < gacc.size(); ++i)
      (*grad)[i] = gacc[i];
  }
  return total_loss;
}

void coef_trainer::adam_step(const std::vector<float> &grad) {
  ++adam_t_;
  const float b1 = 0.9f, b2 = 0.999f, eps = 1e-8f, lr = cfg_.lr;
  double sq = 0.0;
  for (float g : grad)
    sq += static_cast<double>(g) * g;
  const float norm = static_cast<float>(std::sqrt(sq));
  float gscale = 1.0f;
  if (cfg_.grad_clip > 0.0f && norm > cfg_.grad_clip)
    gscale = cfg_.grad_clip / norm;
  const float bc1 = 1.0f - std::pow(b1, static_cast<float>(adam_t_));
  const float bc2 = 1.0f - std::pow(b2, static_cast<float>(adam_t_));
  for (std::size_t i = 0; i < p_.size(); ++i) {
    const float g = grad[i] * gscale;
    m_[i] = b1 * m_[i] + (1.0f - b1) * g;
    u_[i] = b2 * u_[i] + (1.0f - b2) * g * g;
    const float mhat = m_[i] / bc1, uhat = u_[i] / bc2;
    p_[i] -= lr * mhat / (std::sqrt(uhat) + eps);
  }
}

void coef_trainer::train(const training_scene &scene, bool verbose) {
  const int n = cfg_.n, horizon = cfg_.horizon, window = cfg_.window;
  const bool bike = cfg_.rollout == rollout_kind::bicycle;
  std::vector<float> o(2 * n), goal(2 * n), aux(2 * n), o2(2 * n), a2(2 * n);
  std::vector<float> grad;
  for (int step = 0; step < cfg_.steps; ++step) {
    scene.sample_starts_goals(n, cfg_.seed + static_cast<unsigned>(step), o.data(), goal.data());
    // Initial aux: surrogate v=0; bicycle (th aimed at the goal, sp=0).
    for (int i = 0; i < n; ++i) {
      if (bike) {
        aux[2 * i] = std::atan2(goal[2 * i + 1] - o[2 * i + 1], goal[2 * i] - o[2 * i]);
        aux[2 * i + 1] = 0.0f;
      } else {
        aux[2 * i] = 0.0f;
        aux[2 * i + 1] = 0.0f;
      }
    }
    double last = 0.0;
    for (int w0 = 0; w0 < horizon; w0 += window) {
      const int wl = std::min(window, horizon - w0);
      last = loss_and_grad(scene, o.data(), aux.data(), goal.data(), n, wl, &grad, o2.data(),
                           a2.data());
      adam_step(grad);
      o.swap(o2);
      aux.swap(a2);
    }
    if (verbose && (step % 50 == 0 || step == cfg_.steps - 1))
      std::printf("  step %4d: window_loss %.4f\n", step, last);
  }
}

coef_mlp coef_trainer::to_coef_mlp() const {
  const int h = hidden_;
  std::vector<int> rows = {h, h, 3}, cols = {5, h, h};
  std::vector<std::uint32_t> act = {1, 1, 0};
  std::vector<std::vector<float>> w(3), b(3);
  for (int L = 0; L < 3; ++L) {
    w[L].assign(p_.begin() + off_w_[L], p_.begin() + off_w_[L] + lrows_[L] * lcols_[L]);
    b[L].assign(p_.begin() + off_b_[L], p_.begin() + off_b_[L] + lrows_[L]);
  }
  std::vector<float> ob = {bias_[0], bias_[1], bias_[2]};
  return coef_mlp::from_layers(5, 3, rows, cols, act, w, b, ob);
}

} // namespace nav
} // namespace cvc
