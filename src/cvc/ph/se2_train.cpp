/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph::se2 training (se2_train.h): the data, the rollout objective and its hand-written
// reverse pass, train.py's optimizer, and the training loop.
//
// The reverse pass, for one Lie-IMEX step (se2_vehicle.h's notation, a and b the two implicit
// systems' matrices and right-hand sides, hh = h / 2):
//
// clang-format off
//   v' = a_2^-1 b_2:   bb_2 = a_2^-T dv',  dK_2 = -hh bb_2 v'^T,
//                      dv += bb_2 - hh K_1^T bb_2,  de_1 += hh bb_2,  de_2 += hh bb_2,
//                      dK_1 -= hh bb_2 v^T
//   psi', (x, y)':     dpsi += dpsi',  dr += hh dpsi',  dr_p += hh dpsi';  d(x, y) += d(x, y)',
//                      ddxy_1 += hh d(x, y)',  ddxy_2 += hh d(x, y)'
//   stage 2 (at the predictor) gives d(x, y)_p, dpsi_p, dv_p; then (x, y)_p and psi_p feed back:
//                      d(x, y) += d(x, y)_p,  ddxy_1 += h d(x, y)_p,  dpsi += dpsi_p,
//                      dr += h dpsi_p
//   v_p = a_1^-1 b_1:  bb_1 = a_1^-T dv_p,  dK_1 -= h bb_1 v_p^T,  dv += bb_1,  de_1 += h bb_1
//   stage 1 (at the step's start).
// clang-format on
//
// and for a stage (K = M^-1 D, e = M^-1 q, q = ad*_v p - body + G u, p = M v):
//
// clang-format off
//   dM^-1 += dK D^T + de q^T - (M^-T dp) p^T;   dD = M^-T dK;   dq = M^-T de
//   dG = dq u^T;   dlevels_D = spd's reverse of dD;   both GPs' reverse give dW and dz;
//                  dv += dz / scale
//   ad*_v p's reverse gives dp and dv;   dv += M^-T dp
//   body = R^T grad V:  d grad V = R dbody,  dpsi += dbody . (body_y, -body_x)
//   grad V = level + sum_k f_k omega_k / ell_V:  dW_V,k, and d(x, y) from d grad V (its Hessian)
//   dxy = R v:  dv += R^T ddxy,  dpsi += ddxy . (-dxy_y, dxy_x)
// clang-format on
//
// M^-1's gradient is collected as a 3x3 over the whole window and turned into its six levels
// once. The reparameterization W = mean + e^log_std z gives dmean = dW, dlog_std = dW e^log_std z.

#include <algorithm>
#include <cmath>
#include <cvc/core/thread_pool.h>
#include <cvc/ph/se2_train.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace cvc {
namespace ph {

namespace {

using V3 = vec3<double>;
using M3 = mat3<double>;

// ── data helpers ───────────────────────────────────────────────────────────────────────────────

void check_flights(const se2_flights &f) {
  if (f.width < 7 || f.width > 6 + kSe2MaxControls)
    throw std::invalid_argument("se2_flights: rows must be 6 + 1-4 inputs wide");
  if (f.data.size() != f.count * f.rows * static_cast<std::size_t>(f.width))
    throw std::invalid_argument("se2_flights: data is not count x rows x width");
}

// ── the reverse pass ───────────────────────────────────────────────────────────────────────────

// The gradient with respect to one weight draw's values.
struct draw_grad {
  std::vector<double> wD, wG, wV;
  std::array<double, 6> d_level{};
  std::vector<double> g_level;
  std::array<double, 2> v_level{};
  M3 minv{};

  void reset(const se2_model &m) {
    wD.assign(m.params.D.mean.size(), 0.0);
    wG.assign(m.params.G.mean.size(), 0.0);
    wV.assign(m.params.V.mean.size(), 0.0);
    d_level.fill(0.0);
    g_level.assign(m.params.G.level.size(), 0.0);
    v_level.fill(0.0);
    minv.fill(0.0);
  }
  void add(const draw_grad &o, double s) {
    for (std::size_t i = 0; i < wD.size(); ++i)
      wD[i] += s * o.wD[i];
    for (std::size_t i = 0; i < wG.size(); ++i)
      wG[i] += s * o.wG[i];
    for (std::size_t i = 0; i < wV.size(); ++i)
      wV[i] += s * o.wV[i];
    for (int i = 0; i < 6; ++i)
      d_level[i] += s * o.d_level[i];
    for (std::size_t i = 0; i < g_level.size(); ++i)
      g_level[i] += s * o.g_level[i];
    for (int i = 0; i < 2; ++i)
      v_level[i] += s * o.v_level[i];
    for (int i = 0; i < 9; ++i)
      minv[i] += s * o.minv[i];
  }
};

// One stage's reverse: (ddxy, de, dK) on its outputs -> (dpos, dpsi, dv) on its inputs (added),
// and the draw's gradient (added).
void stage_vjp(const se2_vehicle<double> &veh, const se2_stage<double> &st,
               const std::array<double, 2> &ddxy, const V3 &de, const M3 &dK, double *dpos,
               double &dpsi, V3 &dv, draw_grad &g) {
  const se2_setup &s = veh.setup();
  const se2_weights<double> &w = veh.weights();
  const M3 &minv = veh.inverse_mass();
  const int nu = veh.control_dim();

  // K = M^-1 D
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      g.minv[3 * i + j] += dK[3 * i] * st.D[3 * j] + dK[3 * i + 1] * st.D[3 * j + 1] +
                           dK[3 * i + 2] * st.D[3 * j + 2];
  const M3 dD = mul(transpose(minv), dK);
  // e = M^-1 q
  add_outer(g.minv, de, st.q);
  const V3 dq = tmatvec(minv, de);
  // q = ad*_v p + G u - body
  const V3 &dcoad = dq;
  const double dbody[2] = {-dq[0], -dq[1]};
  std::array<double, 3 * kSe2MaxControls> dG{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < nu; ++j) {
      dG[i * nu + j] = dq[i] * st.u[j];
      g.g_level[i * nu + j] += dG[i * nu + j];
    }
  V3 dz{};
  s.G.gp_vjp(st.theta_G.data(), w.wG.data(), 3 * nu, dG.data(), g.wG.data(), dz.data());
  // D = spd(level + GP_D(z))
  double dl[6] = {};
  spd_from_levels_vjp(st.d_levels.data(), dD, dl);
  for (int i = 0; i < 6; ++i)
    g.d_level[i] += dl[i];
  s.D.gp_vjp(st.theta_D.data(), w.wD.data(), 6, dl, g.wD.data(), dz.data());
  for (int i = 0; i < 3; ++i)
    dv[i] += dz[i] / veh.twist_scale()[i];
  // ad*_v p = (p_y r, -p_x r, p_x v_y - p_y v_x)
  const V3 &p = st.p, &v = st.v;
  const V3 dp = {-v[2] * dcoad[1] + v[1] * dcoad[2], v[2] * dcoad[0] - v[0] * dcoad[2], 0.0};
  dv[0] -= p[1] * dcoad[2];
  dv[1] += p[0] * dcoad[2];
  dv[2] += p[1] * dcoad[0] - p[0] * dcoad[1];
  // p = M v = (M^-1)^-1 v
  const V3 y = solve3_transposed(minv, dp);
  for (int i = 0; i < 3; ++i)
    dv[i] += y[i];
  add_outer(g.minv, y, p, -1.0);
  // body = R^T grad V
  const double dg0 = st.c * dbody[0] - st.s * dbody[1];
  const double dg1 = st.s * dbody[0] + st.c * dbody[1];
  dpsi += dbody[0] * st.body[1] - dbody[1] * st.body[0];
  // grad V = level + sum_k f_k omega_k / ell_V
  g.v_level[0] += dg0;
  g.v_level[1] += dg1;
  if (s.v_gp) {
    const double inv = 1.0 / s.v_length, a = s.V.amplitude();
    for (int k = 0; k < s.V.count; ++k) {
      const double *om = &s.V.frequency[2 * static_cast<std::size_t>(k)];
      const double sk = (om[0] * dg0 + om[1] * dg1) * inv;
      g.wV[k] += -a * std::sin(st.theta_V[k]) * sk;
      const double dtheta = -a * std::cos(st.theta_V[k]) * w.wV[k] * sk;
      dpos[0] += dtheta * om[0] * inv;
      dpos[1] += dtheta * om[1] * inv;
    }
  }
  // dxy = R (v_x, v_y)
  dv[0] += st.c * ddxy[0] + st.s * ddxy[1];
  dv[1] += -st.s * ddxy[0] + st.c * ddxy[1];
  dpsi += -ddxy[0] * st.dxy[1] + ddxy[1] * st.dxy[0];
}

struct workspace {
  se2_stage<double> st1, st2;
  std::vector<double> path;
  draw_grad grad;
};

// One step's reverse: dnext (6) -> dstate (6, overwritten); the draw's gradient added. The
// forward is recomputed from `state`.
void step_vjp(const se2_vehicle<double> &veh, const double *state, const double *u, double h,
              const double *dnext, double *dstate, workspace &ws) {
  double next[6];
  V3 vp;
  veh.step(state, u, h, nullptr, next, ws.st1, ws.st2, vp);
  const se2_stage<double> &st1 = ws.st1, &st2 = ws.st2;
  const double hh = 0.5 * h;
  const V3 &v = st1.v;
  const V3 vn = {next[3], next[4], next[5]};
  M3 a1 = identity3<double>(), a2 = identity3<double>();
  for (int k = 0; k < 9; ++k) {
    a1[k] += h * st1.K[k];
    a2[k] += hh * st2.K[k];
  }
  // v' = a_2^-1 b_2
  const V3 bb2 = solve3_transposed(a2, V3{dnext[3], dnext[4], dnext[5]});
  M3 dK2{}, dK1{};
  add_outer(dK2, bb2, vn, -hh);
  V3 dv{}, de1{}, de2{}, dvp{};
  const V3 k1t = tmatvec(st1.K, bb2);
  for (int i = 0; i < 3; ++i) {
    dv[i] += bb2[i] - hh * k1t[i];
    de1[i] += hh * bb2[i];
    de2[i] += hh * bb2[i];
  }
  add_outer(dK1, bb2, v, -hh);
  // psi' and (x, y)'
  double dpsi = dnext[2];
  dv[2] += hh * dnext[2];
  dvp[2] += hh * dnext[2];
  double dpos[2] = {dnext[0], dnext[1]};
  std::array<double, 2> ddxy1 = {hh * dnext[0], hh * dnext[1]};
  const std::array<double, 2> ddxy2 = ddxy1;
  // stage 2, at the predictor
  double dpos_p[2] = {0.0, 0.0};
  double dpsi_p = 0.0;
  stage_vjp(veh, st2, ddxy2, de2, dK2, dpos_p, dpsi_p, dvp, ws.grad);
  dpos[0] += dpos_p[0];
  dpos[1] += dpos_p[1];
  ddxy1[0] += h * dpos_p[0];
  ddxy1[1] += h * dpos_p[1];
  dpsi += dpsi_p;
  dv[2] += h * dpsi_p;
  // v_p = a_1^-1 b_1
  const V3 bb1 = solve3_transposed(a1, dvp);
  add_outer(dK1, bb1, vp, -h);
  for (int i = 0; i < 3; ++i) {
    dv[i] += bb1[i];
    de1[i] += h * bb1[i];
  }
  // stage 1
  stage_vjp(veh, st1, ddxy1, de1, dK1, dpos, dpsi, dv, ws.grad);
  dstate[0] = dpos[0];
  dstate[1] = dpos[1];
  dstate[2] = dpsi;
  dstate[3] = dv[0];
  dstate[4] = dv[1];
  dstate[5] = dv[2];
}

// The window's loss (the mean over its later samples of the squared error) and, when `grad`,
// its gradient into ws.grad (reset by the caller). Not finite when the rollout leaves the finite
// numbers; the gradient is then not computed.
double window_loss(const se2_vehicle<double> &veh, const double *win, int points, int width,
                   double interval, int substeps, bool grad, workspace &ws) {
  const int intervals = points - 1;
  const int steps = intervals * substeps;
  const double h = interval / substeps;
  ws.path.resize(6 * static_cast<std::size_t>(steps + 1));
  double *path = ws.path.data();
  for (int i = 0; i < 6; ++i)
    path[i] = win[i];
  V3 vp;
  for (int t = 0; t < intervals; ++t) {
    const double *u = win + static_cast<std::size_t>(t + 1) * width + 6;
    for (int k = 0; k < substeps; ++k) {
      const int j = t * substeps + k;
      veh.step(path + 6 * j, u, h, nullptr, path + 6 * (j + 1), ws.st1, ws.st2, vp);
    }
  }
  // error at the end of every interval
  double loss = 0.0;
  for (int t = 1; t <= intervals; ++t) {
    const double *x = path + 6 * static_cast<std::size_t>(t * substeps);
    const double *y = win + static_cast<std::size_t>(t) * width;
    for (int i = 0; i < 6; ++i) {
      const double e = i == 2 ? wrap_angle(x[i] - y[i]) : x[i] - y[i];
      loss += e * e;
    }
  }
  loss /= intervals;
  if (!grad || !std::isfinite(loss))
    return loss;
  // backward through time
  const double scale = 2.0 / intervals;
  double g[6] = {}, prev[6];
  for (int t = intervals; t >= 1; --t) {
    const double *x = path + 6 * static_cast<std::size_t>(t * substeps);
    const double *y = win + static_cast<std::size_t>(t) * width;
    for (int i = 0; i < 6; ++i)
      g[i] += scale * (i == 2 ? wrap_angle(x[i] - y[i]) : x[i] - y[i]);
    const double *u = win + static_cast<std::size_t>(t) * width + 6;
    for (int k = substeps - 1; k >= 0; --k) {
      const int j = (t - 1) * substeps + k;
      step_vjp(veh, path + 6 * j, u, h, g, prev, ws);
      std::copy(prev, prev + 6, g);
    }
  }
  return loss;
}

double kl_of(const se2_gp &g) {
  double kl = 0.0;
  for (std::size_t i = 0; i < g.mean.size(); ++i)
    kl += 0.5 * (std::exp(2.0 * g.log_std[i]) + g.mean[i] * g.mean[i] - 1.0 - 2.0 * g.log_std[i]);
  return kl;
}

// dmean += dW + beta_scale mean;  dlog_std += dW e^log_std z + beta_scale (e^(2 log_std) - 1)
void gp_grad(const se2_gp &p, const std::vector<double> &dW, const std::vector<double> *z,
             double beta_scale, se2_gp &out) {
  for (std::size_t i = 0; i < p.mean.size(); ++i) {
    out.mean[i] += dW[i] + beta_scale * p.mean[i];
    out.log_std[i] += (z ? dW[i] * std::exp(p.log_std[i]) * (*z)[i] : 0.0) +
                      beta_scale * (std::exp(2.0 * p.log_std[i]) - 1.0);
  }
}

} // namespace

// ── data ─────────────────────────────────────────────────────────────────────────────────────────

std::array<double, 3> twist_rms(const se2_flights &f) {
  check_flights(f);
  std::array<double, 3> sum{};
  const std::size_t n = f.count * f.rows;
  for (std::size_t r = 0; r < n; ++r)
    for (int i = 0; i < 3; ++i) {
      const double x = f.data[r * f.width + 3 + i];
      sum[i] += x * x;
    }
  for (auto &x : sum)
    x = std::sqrt(x / std::max<std::size_t>(n, 1));
  return sum;
}

std::vector<double> control_rms(const se2_flights &f) {
  check_flights(f);
  std::vector<double> sum(f.control_dim(), 0.0);
  const std::size_t n = f.count * f.rows;
  for (std::size_t r = 0; r < n; ++r)
    for (int i = 0; i < f.control_dim(); ++i) {
      const double x = f.data[r * f.width + 6 + i];
      sum[i] += x * x;
    }
  for (auto &x : sum)
    x = std::sqrt(x / std::max<std::size_t>(n, 1));
  return sum;
}

se2_flights scale_controls(const se2_flights &f, const std::vector<double> &scale) {
  check_flights(f);
  if (static_cast<int>(scale.size()) != f.control_dim())
    throw std::invalid_argument("scale_controls: one scale per input");
  se2_flights out = f;
  const std::size_t n = f.count * f.rows;
  for (std::size_t r = 0; r < n; ++r)
    for (int i = 0; i < f.control_dim(); ++i)
      out.data[r * f.width + 6 + i] /= scale[i];
  return out;
}

se2_flights add_observation_noise(const se2_flights &f, double sigma, std::uint64_t seed) {
  check_flights(f);
  se2_flights out = f;
  rng r(seed);
  const std::size_t n = f.count * f.rows;
  for (int i = 0; i < 6; ++i) // channel by channel, as liesphgp_pack.py draws it
    for (std::size_t k = 0; k < n; ++k)
      out.data[k * f.width + i] += sigma * r.normal();
  return out;
}

se2_windows cut_windows(const se2_flights &f, int points, int stride) {
  check_flights(f);
  if (points < 2 || stride < 1)
    throw std::invalid_argument("cut_windows: at least 2 points and a positive stride");
  se2_windows w;
  w.points = points;
  w.width = f.width;
  std::vector<std::size_t> starts;
  for (std::size_t s = 0; s + points <= f.rows; s += stride)
    starts.push_back(s);
  w.count = f.count * starts.size();
  w.data.resize(w.count * points * f.width);
  std::size_t k = 0;
  for (std::size_t fl = 0; fl < f.count; ++fl) // flight-major, as LieSPHGP's data.py stacks them
    for (std::size_t s : starts) {
      const double *src = f.flight(fl) + s * f.width;
      std::copy(src, src + static_cast<std::size_t>(points) * f.width,
                w.data.begin() + k * points * f.width);
      ++k;
    }
  return w;
}

// ── the objective ────────────────────────────────────────────────────────────────────────────────

se2_draw_noise se2_draw_noise::standard_normal(const se2_model &m, rng &r) {
  se2_draw_noise z;
  for (auto [out, n] :
       {std::pair<std::vector<double> *, std::size_t>{&z.zD, m.params.D.mean.size()},
        {&z.zG, m.params.G.mean.size()},
        {&z.zV, m.params.V.mean.size()}}) {
    out->resize(n);
    for (auto &x : *out)
      x = r.normal();
  }
  return z;
}

se2_objective se2_rollout_objective(const se2_model &m, const se2_windows &w,
                                    const std::vector<std::size_t> *idx, const se2_draw_noise *z,
                                    double beta_scale, se2_params *grad, cvc::thread_pool *pool) {
  if (w.width != 6 + m.setup.control_dim)
    throw std::invalid_argument("se2_rollout_objective: the windows' inputs are not the model's");
  const se2_vehicle<double> veh(m.setup,
                                z ? se2_draw<double>(m, z->zD.data(), z->zG.data(), z->zV.data())
                                  : se2_draw<double>(m),
                                m.twist_scale);
  const std::size_t n = idx ? idx->size() : w.count;
  std::vector<double> losses(n);
  std::vector<workspace> ws(grad ? n : 0);
  const auto run = [&](int i) {
    const std::size_t win = idx ? (*idx)[i] : static_cast<std::size_t>(i);
    if (grad) {
      ws[i].grad.reset(m);
      losses[i] =
          window_loss(veh, w.window(win), w.points, w.width, m.interval, m.substeps, true, ws[i]);
    } else {
      thread_local workspace local;
      losses[i] =
          window_loss(veh, w.window(win), w.points, w.width, m.interval, m.substeps, false, local);
    }
  };
  if (pool)
    pool->parallel_for(static_cast<int>(n), run);
  else
    for (std::size_t i = 0; i < n; ++i)
      run(static_cast<int>(i));

  se2_objective out;
  double sum = 0.0;
  for (std::size_t i = 0; i < n; ++i)
    if (std::isfinite(losses[i])) {
      sum += losses[i];
      ++out.healthy;
    }
  out.rollout = sum / std::max<std::size_t>(out.healthy, 1);
  out.kl = kl_of(m.params.D) + kl_of(m.params.G) + kl_of(m.params.V);
  out.value = out.rollout + beta_scale * out.kl;
  if (!grad)
    return out;

  draw_grad total;
  total.reset(m);
  const double inv = 1.0 / std::max<std::size_t>(out.healthy, 1);
  for (std::size_t i = 0; i < n; ++i)
    if (std::isfinite(losses[i]))
      total.add(ws[i].grad, inv);
  *grad = m.params.zeros_like();
  gp_grad(m.params.D, total.wD, z ? &z->zD : nullptr, beta_scale, grad->D);
  gp_grad(m.params.G, total.wG, z ? &z->zG : nullptr, beta_scale, grad->G);
  gp_grad(m.params.V, total.wV, z ? &z->zV : nullptr, beta_scale, grad->V);
  for (int i = 0; i < 6; ++i)
    grad->D.level[i] = total.d_level[i];
  for (std::size_t i = 0; i < total.g_level.size(); ++i)
    grad->G.level[i] = total.g_level[i];
  for (int i = 0; i < 2; ++i)
    grad->V.level[i] = total.v_level[i];
  spd_from_levels_vjp(m.params.minv_level.data(), total.minv, grad->minv_level.data());
  return out;
}

// ── the optimizer ────────────────────────────────────────────────────────────────────────────────

namespace {
int level_index(se2_leaf_group g) {
  switch (g) {
  case se2_leaf_group::D_level:
    return 0;
  case se2_leaf_group::G_level:
    return 1;
  case se2_leaf_group::V_level:
    return 2;
  case se2_leaf_group::Minv_level:
    return 3;
  default:
    return -1;
  }
}
} // namespace

se2_optimizer::se2_optimizer(const se2_params &p0, const se2_optimizer_config &cfg) : cfg_(cfg) {
  adam_.beta1 = cfg.beta1;
  adam_.beta2 = cfg.beta2;
  adam_.eps = cfg.epsilon;
  adam_.init(p0.size());
  p0.for_each_leaf([&](const char *, const double *data, std::size_t size, se2_leaf_group g) {
    const int k = level_index(g);
    if (k < 0)
      return;
    prodigy &pr = level_[k];
    pr.beta1 = cfg.beta1;
    pr.beta2 = cfg.beta2;
    pr.eps = cfg.epsilon;
    pr.d0 = cfg.prodigy_d0;
    pr.safeguard_warmup = true;
    pr.init(data, size);
  });
}

double se2_optimizer::update(se2_params &p, se2_params &grad) {
  // the global norm over every leaf, then clip
  double sq = 0.0;
  grad.for_each_leaf([&](const char *, double *g, std::size_t n, se2_leaf_group) {
    for (std::size_t i = 0; i < n; ++i)
      sq += g[i] * g[i];
  });
  const double norm = std::sqrt(sq);
  if (cfg_.gradient_clip_norm > 0.0 && !(norm < cfg_.gradient_clip_norm))
    grad.for_each_leaf([&](const char *, double *g, std::size_t n, se2_leaf_group) {
      for (std::size_t i = 0; i < n; ++i)
        g[i] = g[i] / norm * cfg_.gradient_clip_norm;
    });
  const double lr = cosine_decay(cfg_.learning_rate, std::max(cfg_.total_steps, 1),
                                 cfg_.final_learning_fraction, count_);
  const double sched = cosine_decay(cfg_.level_learning_rate, std::max(cfg_.total_steps, 1),
                                    cfg_.final_learning_fraction, count_);
  // Adam's moments are kept over the flat leaf order; level leaves leave theirs at zero.
  std::vector<double *> pdata;
  std::vector<std::size_t> psize;
  p.for_each_leaf([&](const char *, double *d, std::size_t n, se2_leaf_group) {
    pdata.push_back(d);
    psize.push_back(n);
  });
  std::size_t offset = 0, leaf = 0;
  const int t = adam_.count + 1;
  const double c1 = 1.0 - std::pow(adam_.beta1, t), c2 = 1.0 - std::pow(adam_.beta2, t);
  grad.for_each_leaf([&](const char *, double *g, std::size_t n, se2_leaf_group group) {
    double *x = pdata[leaf];
    const int k = level_index(group);
    if (k >= 0) {
      level_[k].update(x, g, n, sched);
    } else {
      for (std::size_t i = 0; i < n; ++i) {
        double &m = adam_.m[offset + i], &v = adam_.v[offset + i];
        m = adam_.beta1 * m + (1.0 - adam_.beta1) * g[i];
        v = adam_.beta2 * v + (1.0 - adam_.beta2) * g[i] * g[i];
        x[i] -= lr * (m / c1) / (std::sqrt(v / c2) + adam_.eps);
      }
    }
    offset += n;
    ++leaf;
  });
  adam_.count = t;
  ++count_;
  return norm;
}

double se2_optimizer::prodigy_d(se2_leaf_group level) const {
  const int k = level_index(level);
  if (k < 0)
    throw std::invalid_argument("se2_optimizer::prodigy_d: not a level group");
  return level_[k].d;
}

// ── training ─────────────────────────────────────────────────────────────────────────────────────

void se2_train_rollout(se2_model &m, const se2_windows &train, const se2_windows *test,
                       const se2_train_config &cfg,
                       const std::function<void(const se2_train_event &)> &on_step) {
  if (train.count == 0)
    throw std::invalid_argument("se2_train_rollout: no training windows");
  if (train.width != 6 + m.setup.control_dim || (test && test->width != train.width))
    throw std::invalid_argument("se2_train_rollout: the windows' inputs are not the model's");
  // threads 1: inline, no pool (a pool of 0 workers means "hardware concurrency - 1")
  std::unique_ptr<cvc::thread_pool> owned;
  if (cfg.threads != 1)
    owned = std::make_unique<cvc::thread_pool>(
        cfg.threads > 1 ? static_cast<unsigned>(cfg.threads - 1) : 0u);
  cvc::thread_pool *pool = owned.get();
  se2_optimizer_config ocfg = cfg.optimizer;
  ocfg.total_steps = cfg.total_steps;
  se2_optimizer opt(m.params, ocfg);
  rng batches = rng::derive(cfg.seed, 101), draws = rng::derive(cfg.seed, 102);
  const double observations = static_cast<double>(train.count) * (train.points - 1);
  const std::size_t batch = std::min<std::size_t>(std::max(cfg.batch_size, 1), train.count);

  // the monitor's windows: eval_windows evenly spread test windows, the first training ones
  std::vector<std::size_t> test_idx, train_idx;
  if (test && test->count) {
    const std::size_t k =
        cfg.eval_windows > 0 ? std::min<std::size_t>(cfg.eval_windows, test->count) : test->count;
    for (std::size_t i = 0; i < k; ++i)
      test_idx.push_back(
          k > 1 ? static_cast<std::size_t>(static_cast<double>(i) * (test->count - 1) / (k - 1))
                : 0);
    for (std::size_t i = 0; i < std::min(k, train.count); ++i)
      train_idx.push_back(i);
  }

  std::vector<std::size_t> pick(train.count);
  se2_params grad;
  for (int step = 1; step <= cfg.total_steps; ++step) {
    // batch_size distinct windows (a partial Fisher-Yates shuffle), sorted
    for (std::size_t i = 0; i < train.count; ++i)
      pick[i] = i;
    for (std::size_t i = 0; i < batch; ++i)
      std::swap(pick[i], pick[i + batches.below(train.count - i)]);
    std::vector<std::size_t> idx(pick.begin(), pick.begin() + batch);
    std::sort(idx.begin(), idx.end());
    const double beta = cfg.kl_beta_max *
                        std::min(1.0, static_cast<double>(step) / std::max(1, cfg.kl_anneal_steps));
    const se2_draw_noise z = se2_draw_noise::standard_normal(m, draws);
    se2_train_event ev;
    ev.step = step;
    ev.beta = beta;
    ev.objective =
        se2_rollout_objective(m, train, &idx, &z, beta / (observations * 6.0), &grad, pool);
    ev.gradient_norm = opt.update(m.params, grad);
    if (!std::isfinite(ev.objective.value) || !std::isfinite(ev.gradient_norm))
      throw std::runtime_error("se2_train_rollout: non-finite loss or gradient at step " +
                               std::to_string(step));
    ev.prodigy_d = {opt.prodigy_d(se2_leaf_group::D_level), opt.prodigy_d(se2_leaf_group::G_level),
                    opt.prodigy_d(se2_leaf_group::V_level),
                    opt.prodigy_d(se2_leaf_group::Minv_level)};
    if (!test_idx.empty() && cfg.eval_every > 0 &&
        (step % cfg.eval_every == 0 || step == cfg.total_steps)) {
      ev.evaluated = true;
      ev.test_loss =
          se2_rollout_objective(m, *test, &test_idx, nullptr, 0.0, nullptr, pool).rollout;
      ev.train_loss =
          se2_rollout_objective(m, train, &train_idx, nullptr, 0.0, nullptr, pool).rollout;
    }
    if (on_step)
      on_step(ev);
  }
}

} // namespace ph
} // namespace cvc
