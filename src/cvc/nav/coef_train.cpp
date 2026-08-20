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
// hand-written reverse-mode adjoints (no libtorch). Built without -ffast-math so
// the float32 forward tracks the deployment sample/rollout math; the backward is
// validated by finite differences (nav_coef_train_test), so correctness is
// self-contained, not a torch-parity claim.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/grid_nav.h>
#include <random>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace nav {

namespace {

inline float sigmoidf(float x) { return 1.0f / (1.0f + std::exp(-x)); }
inline float siluf(float x) { return x * sigmoidf(x); }
inline float silu_grad(float x) { // d/dx [x*sigmoid(x)]
  const float s = sigmoidf(x);
  return s + x * s * (1.0f - s);
}
inline float softplusf(float x) { return x > 20.0f ? x : std::log1p(std::exp(x)); }

// IPC barrier derivative (ipc_dbdd, drive.cpp) and its d/dd.
inline float ipc_dbdd(float d, float d_hat) {
  const float dc = d < 1e-6f ? 1e-6f : d;
  if (!(dc < d_hat))
    return 0.0f;
  return (d_hat - dc) * (2.0f * std::log(dc / d_hat) - d_hat / dc) + 1.0f;
}
inline float ipc_dbdd_grad(float d, float d_hat) {
  if (d < 1e-6f)
    return 0.0f; // clamp_min(1e-6): gradient killed
  if (!(d < d_hat))
    return 0.0f; // where(d < d_hat, ., 0)
  const float A = d_hat - d;
  const float B = 2.0f * std::log(d / d_hat) - d_hat / d;
  const float dB = 2.0f / d + d_hat / (d * d);
  return -B + A * dB; // d/dd [A*B + 1]
}

// A cached bilinear sample (drive.cpp sample_unit) with everything the position
// VJP needs. `phi`, unit normal (`nx`,`ny`) are the forward outputs.
struct Sample {
  float phv[4], pxv[4], pyv[4]; // corner values (nw,ne,sw,se) per channel
  float wx0, wx1, wy0, wy1;
  bool clx, cly;  // ix/iy were clamped to the border
  float Wf1, Hf1; // W-1, H-1
  float cgx, cgy; // d gx/d onx, d gy/d ony
  float rnx, rny; // raw normal (pre-renorm)
  float r, mag;   // |raw|, |raw|+1e-6
  float phi, nx, ny;
};

Sample sample_fwd(const field_stack &f, float onx, float ony) {
  Sample s;
  const float S = static_cast<float>(f.S);
  const float mnx = static_cast<float>(f.mnx), mxx = static_cast<float>(f.mxx);
  const float mny = static_cast<float>(f.mny), mxy = static_cast<float>(f.mxy);
  const float wx = onx / S + static_cast<float>(f.cx);
  const float wy = ony / S + static_cast<float>(f.cy);
  const float gx = 2.0f * (wx - mnx) / (mxx - mnx) - 1.0f;
  const float gy = 2.0f * (wy - mny) / (mxy - mny) - 1.0f;
  s.cgx = 2.0f / ((mxx - mnx) * S);
  s.cgy = 2.0f / ((mxy - mny) * S);
  s.Wf1 = static_cast<float>(f.W - 1);
  s.Hf1 = static_cast<float>(f.H - 1);
  float ix = (gx + 1.0f) * 0.5f * s.Wf1;
  float iy = (gy + 1.0f) * 0.5f * s.Hf1;
  s.clx = (ix < 0.0f) || (ix > s.Wf1);
  s.cly = (iy < 0.0f) || (iy > s.Hf1);
  ix = std::min(std::max(ix, 0.0f), s.Wf1);
  iy = std::min(std::max(iy, 0.0f), s.Hf1);
  const int ix0 = static_cast<int>(std::floor(ix)), iy0 = static_cast<int>(std::floor(iy));
  s.wx1 = ix - static_cast<float>(ix0);
  s.wx0 = 1.0f - s.wx1;
  s.wy1 = iy - static_cast<float>(iy0);
  s.wy0 = 1.0f - s.wy1;
  const int cx0 = std::min(std::max(ix0, 0), f.W - 1),
            cx1 = std::min(std::max(ix0 + 1, 0), f.W - 1);
  const int cy0 = std::min(std::max(iy0, 0), f.H - 1),
            cy1 = std::min(std::max(iy0 + 1, 0), f.H - 1);
  const long HW = static_cast<long>(f.H) * f.W;
  const float *ph = f.data, *px = f.data + HW, *py = f.data + 2 * HW;
  const long nw = static_cast<long>(cy0) * f.W + cx0, ne = static_cast<long>(cy0) * f.W + cx1;
  const long sw = static_cast<long>(cy1) * f.W + cx0, se = static_cast<long>(cy1) * f.W + cx1;
  s.phv[0] = ph[nw];
  s.phv[1] = ph[ne];
  s.phv[2] = ph[sw];
  s.phv[3] = ph[se];
  s.pxv[0] = px[nw];
  s.pxv[1] = px[ne];
  s.pxv[2] = px[sw];
  s.pxv[3] = px[se];
  s.pyv[0] = py[nw];
  s.pyv[1] = py[ne];
  s.pyv[2] = py[sw];
  s.pyv[3] = py[se];
  const float nwW = s.wx0 * s.wy0, neW = s.wx1 * s.wy0, swW = s.wx0 * s.wy1, seW = s.wx1 * s.wy1;
  s.phi = s.phv[0] * nwW + s.phv[1] * neW + s.phv[2] * swW + s.phv[3] * seW;
  s.rnx = s.pxv[0] * nwW + s.pxv[1] * neW + s.pxv[2] * swW + s.pxv[3] * seW;
  s.rny = s.pyv[0] * nwW + s.pyv[1] * neW + s.pyv[2] * swW + s.pyv[3] * seW;
  s.r = std::sqrt(s.rnx * s.rnx + s.rny * s.rny);
  s.mag = s.r + 1e-6f;
  s.nx = s.rnx / s.mag;
  s.ny = s.rny / s.mag;
  return s;
}

// VJP of sample_fwd: (dL/dphi, dL/dnx, dL/dny) -> (dL/donx, dL/dony).
void sample_bwd(const Sample &s, float gphi, float gnx, float gny, float &gonx, float &gony) {
  float grnx = 0.0f, grny = 0.0f;
  if (s.r > 0.0f) {
    const float mag2 = s.mag * s.mag;
    const float dnx_drnx = (s.mag - s.rnx * s.rnx / s.r) / mag2;
    const float dny_drny = (s.mag - s.rny * s.rny / s.r) / mag2;
    const float dcross = -(s.rnx * s.rny) / (s.r * mag2); // dnx/drny == dny/drnx
    grnx = gnx * dnx_drnx + gny * dcross;
    grny = gnx * dcross + gny * dny_drny;
  }
  const float dphi_dix = s.wy0 * (s.phv[1] - s.phv[0]) + s.wy1 * (s.phv[3] - s.phv[2]);
  const float dphi_diy = s.wx0 * (s.phv[2] - s.phv[0]) + s.wx1 * (s.phv[3] - s.phv[1]);
  const float drnx_dix = s.wy0 * (s.pxv[1] - s.pxv[0]) + s.wy1 * (s.pxv[3] - s.pxv[2]);
  const float drnx_diy = s.wx0 * (s.pxv[2] - s.pxv[0]) + s.wx1 * (s.pxv[3] - s.pxv[1]);
  const float drny_dix = s.wy0 * (s.pyv[1] - s.pyv[0]) + s.wy1 * (s.pyv[3] - s.pyv[2]);
  const float drny_diy = s.wx0 * (s.pyv[2] - s.pyv[0]) + s.wx1 * (s.pyv[3] - s.pyv[1]);
  const float gix = gphi * dphi_dix + grnx * drnx_dix + grny * drny_dix;
  const float giy = gphi * dphi_diy + grnx * drnx_diy + grny * drny_diy;
  const float ggx = gix * (s.clx ? 0.0f : 0.5f * s.Wf1);
  const float ggy = giy * (s.cly ? 0.0f : 0.5f * s.Hf1);
  gonx = ggx * s.cgx;
  gony = ggy * s.cgy;
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
    // L0 (5->h, SiLU)
    for (int o = 0; o < h; ++o) {
      float acc = p_[off_b_[0] + o];
      const float *w = &p_[off_w_[0] + static_cast<std::size_t>(o) * 5];
      for (int i = 0; i < 5; ++i)
        acc += w[i] * x[i];
      a[o] = siluf(acc);
    }
    // L1 (h->h, SiLU)
    for (int o = 0; o < h; ++o) {
      float acc = p_[off_b_[1] + o];
      const float *w = &p_[off_w_[1] + static_cast<std::size_t>(o) * h];
      for (int i = 0; i < h; ++i)
        acc += w[i] * a[i];
      b[o] = siluf(acc);
    }
    // L2 (h->3, identity) + softplus(net + log(expm1(bias)))
    float *oo = out + static_cast<std::size_t>(s) * 3;
    for (int o = 0; o < 3; ++o) {
      float acc = p_[off_b_[2] + o];
      const float *w = &p_[off_w_[2] + static_cast<std::size_t>(o) * h];
      for (int i = 0; i < h; ++i)
        acc += w[i] * b[i];
      oo[o] = softplusf(acc + std::log(std::expm1(bias_[o])));
    }
  }
}

double coef_trainer::loss_and_grad(const training_scene &scene, const float *o_in,
                                   const float *v_in, const float *goal, int n, int window,
                                   std::vector<float> *grad, float *o_out, float *v_out) const {
  const field_stack fs = scene.field();
  const int h = hidden_;
  const float rr = scene.rr, d_hat = scene.d_hat, vmax = scene.vmax;
  const float hdt = scene.dt; // nsub == 1 (training)
  const float off_bias[3] = {std::log(std::expm1(bias_[0])), std::log(std::expm1(bias_[1])),
                             std::log(std::expm1(bias_[2]))};
  if (grad)
    grad->assign(p_.size(), 0.0f);

  // Per-step forward cache for one agent's window.
  struct Step {
    Sample cf; // coef_feats sample at o_t
    float dx, dy, gd, inv, gdx, gdy;
    float feat[5];
    std::vector<float> z0, a0, z1, a1; // MLP pre/post activations
    float raw[3], coef[3];
    float ox, oy, vx, vy;   // step inputs (o_t, v_t)
    Sample rl;              // rollout force sample at o_t
    float d, ipc;           // rollout barrier
    float ax, ay, vpx, vpy; // acceleration, pre-clamp velocity
    float sp;
    bool scaled;
    float ox1, oy1, vx1, vy1; // step outputs (o_{t+1}, v_{t+1})
    Sample coll;              // collision sample at o_{t+1}
    float phi_new;
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

  // Per-param gradient accumulator (summed over agents, averaged at the end).
  std::vector<float> gacc(grad ? p_.size() : 0, 0.0f);

  for (int ag = 0; ag < n; ++ag) {
    float ox = o_in[2 * ag], oy = o_in[2 * ag + 1];
    float vx = v_in[2 * ag], vy = v_in[2 * ag + 1];
    const float gx = goal[2 * ag], gy = goal[2 * ag + 1];

    // ── forward window ──
    for (int t = 0; t < window; ++t) {
      Step &S = st[t];
      S.ox = ox;
      S.oy = oy;
      S.vx = vx;
      S.vy = vy;

      // coef_feats at o_t
      S.cf = sample_fwd(fs, ox, oy);
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

      // MLP forward (cache pre-activations)
      for (int o = 0; o < h; ++o) {
        float acc = p_[off_b_[0] + o];
        const float *w = &p_[off_w_[0] + static_cast<std::size_t>(o) * 5];
        for (int i = 0; i < 5; ++i)
          acc += w[i] * S.feat[i];
        S.z0[o] = acc;
        S.a0[o] = siluf(acc);
      }
      for (int o = 0; o < h; ++o) {
        float acc = p_[off_b_[1] + o];
        const float *w = &p_[off_w_[1] + static_cast<std::size_t>(o) * h];
        for (int i = 0; i < h; ++i)
          acc += w[i] * S.a0[i];
        S.z1[o] = acc;
        S.a1[o] = siluf(acc);
      }
      for (int o = 0; o < 3; ++o) {
        float acc = p_[off_b_[2] + o];
        const float *w = &p_[off_w_[2] + static_cast<std::size_t>(o) * h];
        for (int i = 0; i < h; ++i)
          acc += w[i] * S.a1[i];
        S.raw[o] = acc;
        S.coef[o] = softplusf(acc + off_bias[o]);
      }
      const float al = S.coef[0], be = S.coef[1], ga = S.coef[2];

      // sdf_rollout one step (nsub == 1) — force sample at o_t
      S.rl = sample_fwd(fs, ox, oy);
      S.d = S.rl.phi - rr;
      S.ipc = ipc_dbdd(S.d, d_hat);
      const float Fbar_x = -(al * S.ipc) * S.rl.nx;
      const float Fbar_y = -(al * S.ipc) * S.rl.ny;
      const float Fgoal_x = -be * (ox - gx);
      const float Fgoal_y = -be * (oy - gy);
      S.ax = Fbar_x + Fgoal_x - ga * vx;
      S.ay = Fbar_y + Fgoal_y - ga * vy;
      S.vpx = vx + hdt * S.ax;
      S.vpy = vy + hdt * S.ay;
      S.sp = std::sqrt(S.vpx * S.vpx + S.vpy * S.vpy);
      float vcx = S.vpx, vcy = S.vpy;
      S.scaled = S.sp > vmax;
      if (S.scaled) {
        const float sc = vmax / S.sp;
        vcx = S.vpx * sc;
        vcy = S.vpy * sc;
      }
      S.vx1 = vcx;
      S.vy1 = vcy;
      S.ox1 = ox + hdt * vcx;
      S.oy1 = oy + hdt * vcy;

      // collision sample at o_{t+1}
      S.coll = sample_fwd(fs, S.ox1, S.oy1);
      S.phi_new = S.coll.phi;
      const float pen = rr - S.phi_new;
      if (pen > 0.0f)
        total_loss += static_cast<double>(coll_w) * pen * inv_n;

      ox = S.ox1;
      oy = S.oy1;
      vx = S.vx1;
      vy = S.vy1;
    }

    // terminal goal distance
    const float fdx = ox - gx, fdy = oy - gy;
    const float Lgoal = std::sqrt(fdx * fdx + fdy * fdy);
    total_loss += static_cast<double>(Lgoal) * inv_n;
    if (o_out) {
      o_out[2 * ag] = ox;
      o_out[2 * ag + 1] = oy;
    }
    if (v_out) {
      v_out[2 * ag] = vx;
      v_out[2 * ag + 1] = vy;
    }
    if (!grad)
      continue;

    // ── backward window ──
    // grad on the current step output (o_{t+1}, v_{t+1}); seeded at o_window.
    float go_x = 0.0f, go_y = 0.0f, gv_x = 0.0f, gv_y = 0.0f;
    if (Lgoal > 1e-9f) {
      go_x = static_cast<float>(inv_n) * fdx / Lgoal;
      go_y = static_cast<float>(inv_n) * fdy / Lgoal;
    }
    for (int t = window - 1; t >= 0; --t) {
      Step &S = st[t];
      const float al = S.coef[0], be = S.coef[1], ga = S.coef[2];

      // collision term samples o_{t+1}: add its position grad to (go_x, go_y).
      const float pen = rr - S.phi_new;
      if (pen > 0.0f) {
        const float gphi = -static_cast<float>(coll_w) * static_cast<float>(inv_n);
        float dox, doy;
        sample_bwd(S.coll, gphi, 0.0f, 0.0f, dox, doy);
        go_x += dox;
        go_y += doy;
      }

      // ── rollout backward ──
      // o_{t+1} = o_t + hdt*v''  ;  v_{t+1} = v''
      float gvpp_x = gv_x + hdt * go_x; // grad on clamped velocity v''
      float gvpp_y = gv_y + hdt * go_y;
      float god_x = go_x, god_y = go_y; // grad on o_t (direct), o appears in o'=o+hdt*v''
      // speed clamp backward: v'' = v' or v'*vmax/sp
      float gvp_x, gvp_y;
      if (S.scaled) {
        const float sp = S.sp, sp3 = sp * sp * sp;
        const float dot = gvpp_x * S.vpx + gvpp_y * S.vpy;
        gvp_x = vmax / sp * gvpp_x - vmax / sp3 * S.vpx * dot;
        gvp_y = vmax / sp * gvpp_y - vmax / sp3 * S.vpy * dot;
      } else {
        gvp_x = gvpp_x;
        gvp_y = gvpp_y;
      }
      // v' = v_t + hdt*a
      const float ga_x = hdt * gvp_x, ga_y = hdt * gvp_y; // grad on acceleration
      float gvt_x = gvp_x, gvt_y = gvp_y;                 // grad on v_t (from v'=v_t+hdt*a)
      // a = Fbar + Fgoal - ga*v_t
      float gFbar_x = ga_x, gFbar_y = ga_y;
      float gFgoal_x = ga_x, gFgoal_y = ga_y;
      float g_al = 0.0f, g_be = 0.0f, g_ga = 0.0f;
      g_ga += -(ga_x * S.vx + ga_y * S.vy);
      gvt_x += -ga * ga_x;
      gvt_y += -ga * ga_y;
      // Fgoal = -be*(o_t - goal)
      g_be += -(gFgoal_x * (S.ox - gx) + gFgoal_y * (S.oy - gy));
      god_x += -be * gFgoal_x;
      god_y += -be * gFgoal_y;
      // Fbar = -(al*ipc)*n
      const float aip = al * S.ipc;
      g_al += -S.ipc * (gFbar_x * S.rl.nx + gFbar_y * S.rl.ny);
      float g_ipc = -al * (gFbar_x * S.rl.nx + gFbar_y * S.rl.ny);
      float gnx = -aip * gFbar_x, gny = -aip * gFbar_y;
      // ipc = ipc_dbdd(d) ; d = phi - rr
      float gphi_force = g_ipc * ipc_dbdd_grad(S.d, d_hat);
      // force sample at o_t: (gphi_force, gnx, gny) -> grad o_t
      float sdox, sdoy;
      sample_bwd(S.rl, gphi_force, gnx, gny, sdox, sdoy);
      god_x += sdox;
      god_y += sdoy;

      // ── MLP backward: (g_al,g_be,g_ga) -> params + grad feat ──
      float graw[3];
      for (int o = 0; o < 3; ++o) {
        const float gc = (o == 0 ? g_al : (o == 1 ? g_be : g_ga));
        graw[o] = gc * sigmoidf(S.raw[o] + off_bias[o]); // softplus'
      }
      // L2: raw = W2*a1 + b2
      std::vector<float> ga1(h, 0.0f);
      for (int o = 0; o < 3; ++o) {
        const std::size_t wb = off_w_[2] + static_cast<std::size_t>(o) * h;
        for (int i = 0; i < h; ++i) {
          gacc[wb + i] += graw[o] * S.a1[i];
          ga1[i] += graw[o] * p_[wb + i];
        }
        gacc[off_b_[2] + o] += graw[o];
      }
      // SiLU at z1
      std::vector<float> gz1(h);
      for (int i = 0; i < h; ++i)
        gz1[i] = ga1[i] * silu_grad(S.z1[i]);
      // L1: z1 = W1*a0 + b1
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
      // SiLU at z0
      std::vector<float> gz0(h);
      for (int i = 0; i < h; ++i)
        gz0[i] = ga0[i] * silu_grad(S.z0[i]);
      // L0: z0 = W0*feat + b0
      float gfeat[5] = {0, 0, 0, 0, 0};
      for (int o = 0; o < h; ++o) {
        const std::size_t wb = off_w_[0] + static_cast<std::size_t>(o) * 5;
        const float g = gz0[o];
        for (int i = 0; i < 5; ++i) {
          gacc[wb + i] += g * S.feat[i];
          gfeat[i] += g * p_[wb + i];
        }
        gacc[off_b_[0] + o] += g;
      }

      // ── coef_feats backward: gfeat -> grad o_t ──
      float gphi_cf = gfeat[0];
      float ggd = gfeat[1];
      float ggdx = gfeat[2] + gfeat[4] * S.cf.nx;
      float ggdy = gfeat[3] + gfeat[4] * S.cf.ny;
      float gcfnx = gfeat[4] * S.gdx;
      float gcfny = gfeat[4] * S.gdy;
      // (dx,dy) grads from gd, gdx, gdy
      float g_dx = 0.0f, g_dy = 0.0f;
      const float gd_safe = S.gd > 1e-9f ? S.gd : 1e-9f;
      g_dx += ggd * S.dx / gd_safe;
      g_dy += ggd * S.dy / gd_safe;
      const float inv2 = S.inv * S.inv;
      // gdx = dx*inv, inv=1/(gd+1e-6); d inv/d dx = -inv^2 * dx/gd
      g_dx += ggdx * (S.inv - S.dx * S.dx * inv2 / gd_safe);
      g_dy += ggdx * (-S.dx * S.dy * inv2 / gd_safe);
      g_dy += ggdy * (S.inv - S.dy * S.dy * inv2 / gd_safe);
      g_dx += ggdy * (-S.dx * S.dy * inv2 / gd_safe);
      // dx = gx - o_t  -> d/do_t = -1
      god_x += -g_dx;
      god_y += -g_dy;
      // coef_feats sample at o_t
      float cdox, cdoy;
      sample_bwd(S.cf, gphi_cf, gcfnx, gcfny, cdox, cdoy);
      god_x += cdox;
      god_y += cdoy;

      // propagate to the previous step's output (o_t, v_t)
      go_x = god_x;
      go_y = god_y;
      gv_x = gvt_x;
      gv_y = gvt_y;
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
  // global-norm gradient clip
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
  std::vector<float> o(2 * n), goal(2 * n), v(2 * n), o2(2 * n), v2(2 * n);
  std::vector<float> grad;
  for (int step = 0; step < cfg_.steps; ++step) {
    scene.sample_starts_goals(n, cfg_.seed + static_cast<unsigned>(step), o.data(), goal.data());
    std::fill(v.begin(), v.end(), 0.0f);
    double last = 0.0;
    for (int w0 = 0; w0 < horizon; w0 += window) {
      const int wl = std::min(window, horizon - w0);
      last =
          loss_and_grad(scene, o.data(), v.data(), goal.data(), n, wl, &grad, o2.data(), v2.data());
      adam_step(grad);
      o.swap(o2);
      v.swap(v2);
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
