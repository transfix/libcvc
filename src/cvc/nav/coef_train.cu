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

// coef_train.cu — the CUDA trainer. One thread per agent runs the differentiable
// window forward + hand-written backward using the SHARED detail/diff_rollout.h
// primitives (the same __host__ __device__ source the CPU trainer and the CPU
// gradcheck validate), so the device backward is correct by construction. The
// per-param gradient is accumulated with atomicAdd; checkpoint/recompute stores
// only (o_t, aux_t) per step and recomputes the MLP / samples in the backward.
// The rollout integrator (surrogate vs full bicycle) is selected per run, same
// as the CPU trainer. Built without --use_fast_math so the float32 forward tracks
// the CPU trainer (validated float-equivalent, nav_coef_train_test).
//
// train_coef_mlp_cuda is FULLY DEVICE-RESIDENT: field, params, Adam moments and
// scratch stay on the GPU across the whole run (in-place device Adam; only a
// single-float D2H per window for the grad-clip norm). loss_and_grad_cuda is the
// per-call entry the CUDA-vs-CPU parity test uses.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cuda_runtime.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/detail/diff_rollout.h>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace nav {

namespace {

constexpr int kMaxH = 64; // hidden width cap (stack activation arrays)

// One thread per agent: the window forward+backward. `rollout` 0=surrogate (aux
// = velocity) / 1=bicycle (aux = th, sp). Everything but the integrator step is
// identical for both.
__global__ void train_kernel(diff::field F, const float *p, int h, float ob0, float ob1, float ob2,
                             const float *o_in, const float *aux_in, const float *goal,
                             diff::bike_veh bv, float coll_w, int window, int n, float inv_n,
                             int rollout, float *state, float *d_loss, float *grad, float *o_out,
                             float *aux_out) {
  const int ag = blockIdx.x * blockDim.x + threadIdx.x;
  if (ag >= n)
    return;
  const int ow0 = 0, ob0o = h * 5, ow1 = h * 5 + h, ob1o = ow1 + h * h, ow2 = ob1o + h,
            ob2o = ow2 + 3 * h;
  const float offb[3] = {ob0, ob1, ob2};
  const bool bike = rollout == 1;
  const float rr = bv.rr, d_hat = bv.d_hat, vmax = bv.vmax, hdt = bv.hdt;
  float *st = state + (long)ag * window * 4;
  const float gx = goal[2 * ag], gy = goal[2 * ag + 1];
  float ox = o_in[2 * ag], oy = o_in[2 * ag + 1];
  float axx = aux_in[2 * ag], axy = aux_in[2 * ag + 1];
  double loss = 0.0;

  // ── forward (store only o_t, aux_t) ──
  for (int t = 0; t < window; ++t) {
    st[t * 4 + 0] = ox;
    st[t * 4 + 1] = oy;
    st[t * 4 + 2] = axx;
    st[t * 4 + 3] = axy;
    diff::sample cf = diff::sample_fwd(F, ox, oy);
    const float dx = gx - ox, dy = gy - oy, gd = sqrtf(dx * dx + dy * dy),
                inv = 1.0f / (gd + 1e-6f);
    const float gdx = dx * inv, gdy = dy * inv;
    float feat[5] = {cf.phi, gd, gdx, gdy, gdx * cf.nx + gdy * cf.ny};
    float a0[kMaxH], a1[kMaxH];
    for (int o = 0; o < h; ++o) {
      float acc = p[ob0o + o];
      const float *w = p + ow0 + o * 5;
      for (int i = 0; i < 5; ++i)
        acc += w[i] * feat[i];
      a0[o] = diff::siluf_(acc);
    }
    for (int o = 0; o < h; ++o) {
      float acc = p[ob1o + o];
      const float *w = p + ow1 + o * h;
      for (int i = 0; i < h; ++i)
        acc += w[i] * a0[i];
      a1[o] = diff::siluf_(acc);
    }
    float coef[3];
    for (int o = 0; o < 3; ++o) {
      float acc = p[ob2o + o];
      const float *w = p + ow2 + o * h;
      for (int i = 0; i < h; ++i)
        acc += w[i] * a1[i];
      coef[o] = diff::softplusf_(acc + offb[o]);
    }
    float ox1, oy1, ax1, ay1;
    if (bike)
      diff::bike_step(F, ox, oy, axx, axy, gx, gy, coef[0], coef[1], coef[2], bv, ox1, oy1, ax1,
                      ay1);
    else
      diff::surr_step(F, ox, oy, axx, axy, gx, gy, coef[0], coef[1], coef[2], rr, d_hat, vmax, hdt,
                      ox1, oy1, ax1, ay1);
    diff::sample cs = diff::sample_fwd(F, ox1, oy1);
    const float pen = rr - cs.phi;
    if (pen > 0.0f)
      loss += (double)coll_w * pen * inv_n;
    ox = ox1;
    oy = oy1;
    axx = ax1;
    axy = ay1;
  }
  const float fdx = ox - gx, fdy = oy - gy, Lgoal = sqrtf(fdx * fdx + fdy * fdy);
  loss += (double)Lgoal * inv_n;
  d_loss[ag] = (float)loss;
  if (o_out) {
    o_out[2 * ag] = ox;
    o_out[2 * ag + 1] = oy;
  }
  if (aux_out) {
    aux_out[2 * ag] = axx;
    aux_out[2 * ag + 1] = axy;
  }

  // ── backward (recompute per step) ──
  float go_x = 0.0f, go_y = 0.0f, gaux_x = 0.0f, gaux_y = 0.0f;
  if (Lgoal > 1e-9f) {
    go_x = inv_n * fdx / Lgoal;
    go_y = inv_n * fdy / Lgoal;
  }
  for (int t = window - 1; t >= 0; --t) {
    const float otx = st[t * 4 + 0], oty = st[t * 4 + 1], atx = st[t * 4 + 2], aty = st[t * 4 + 3];
    diff::sample cf = diff::sample_fwd(F, otx, oty);
    const float dx = gx - otx, dy = gy - oty, gd = sqrtf(dx * dx + dy * dy),
                inv = 1.0f / (gd + 1e-6f);
    const float gdx = dx * inv, gdy = dy * inv;
    float feat[5] = {cf.phi, gd, gdx, gdy, gdx * cf.nx + gdy * cf.ny};
    float z0[kMaxH], a0[kMaxH], z1[kMaxH], a1[kMaxH];
    for (int o = 0; o < h; ++o) {
      float acc = p[ob0o + o];
      const float *w = p + ow0 + o * 5;
      for (int i = 0; i < 5; ++i)
        acc += w[i] * feat[i];
      z0[o] = acc;
      a0[o] = diff::siluf_(acc);
    }
    for (int o = 0; o < h; ++o) {
      float acc = p[ob1o + o];
      const float *w = p + ow1 + o * h;
      for (int i = 0; i < h; ++i)
        acc += w[i] * a0[i];
      z1[o] = acc;
      a1[o] = diff::siluf_(acc);
    }
    float raw[3], coef[3];
    for (int o = 0; o < 3; ++o) {
      float acc = p[ob2o + o];
      const float *w = p + ow2 + o * h;
      for (int i = 0; i < h; ++i)
        acc += w[i] * a1[i];
      raw[o] = acc;
      coef[o] = diff::softplusf_(acc + offb[o]);
    }
    const float al = coef[0], be = coef[1], ga = coef[2];

    // collision at o_{t+1}: recompute the step for its output sample
    float ox1, oy1, ax1, ay1;
    if (bike)
      diff::bike_step(F, otx, oty, atx, aty, gx, gy, al, be, ga, bv, ox1, oy1, ax1, ay1);
    else
      diff::surr_step(F, otx, oty, atx, aty, gx, gy, al, be, ga, rr, d_hat, vmax, hdt, ox1, oy1,
                      ax1, ay1);
    diff::sample cs = diff::sample_fwd(F, ox1, oy1);
    const float pen = rr - cs.phi;
    if (pen > 0.0f) {
      float dox, doy;
      diff::sample_bwd(cs, -coll_w * inv_n, 0.0f, 0.0f, dox, doy);
      go_x += dox;
      go_y += doy;
    }

    // rollout backward
    float gox, goy, gauxx, gauxy, g_al, g_be, g_ga;
    if (bike)
      diff::bike_step_bwd(F, otx, oty, atx, aty, gx, gy, al, be, ga, bv, go_x, go_y, gaux_x, gaux_y,
                          gox, goy, gauxx, gauxy, g_al, g_be, g_ga);
    else
      diff::surr_step_bwd(F, otx, oty, atx, aty, gx, gy, al, be, ga, rr, d_hat, vmax, hdt, go_x,
                          go_y, gaux_x, gaux_y, gox, goy, gauxx, gauxy, g_al, g_be, g_ga);

    // MLP backward (atomicAdd into grad) + grad feat
    float graw[3];
    graw[0] = g_al * diff::sigmoidf_(raw[0] + offb[0]);
    graw[1] = g_be * diff::sigmoidf_(raw[1] + offb[1]);
    graw[2] = g_ga * diff::sigmoidf_(raw[2] + offb[2]);
    float ga1[kMaxH];
    for (int i = 0; i < h; ++i)
      ga1[i] = 0.0f;
    for (int o = 0; o < 3; ++o) {
      const int wb = ow2 + o * h;
      for (int i = 0; i < h; ++i) {
        atomicAdd(&grad[wb + i], graw[o] * a1[i]);
        ga1[i] += graw[o] * p[wb + i];
      }
      atomicAdd(&grad[ob2o + o], graw[o]);
    }
    float gz1[kMaxH];
    for (int i = 0; i < h; ++i)
      gz1[i] = ga1[i] * diff::silu_grad_(z1[i]);
    float ga0[kMaxH];
    for (int i = 0; i < h; ++i)
      ga0[i] = 0.0f;
    for (int o = 0; o < h; ++o) {
      const int wb = ow1 + o * h;
      const float gg = gz1[o];
      for (int i = 0; i < h; ++i) {
        atomicAdd(&grad[wb + i], gg * a0[i]);
        ga0[i] += gg * p[wb + i];
      }
      atomicAdd(&grad[ob1o + o], gg);
    }
    float gfeat[5] = {0, 0, 0, 0, 0};
    for (int o = 0; o < h; ++o) {
      const int wb = ow0 + o * 5;
      const float gg = ga0[o] * diff::silu_grad_(z0[o]);
      for (int i = 0; i < 5; ++i) {
        atomicAdd(&grad[wb + i], gg * feat[i]);
        gfeat[i] += gg * p[wb + i];
      }
      atomicAdd(&grad[ob0o + o], gg);
    }

    // coef_feats backward -> grad o_t (add to the rollout's gox/goy)
    float god_x = gox, god_y = goy;
    const float gphi_cf = gfeat[0];
    const float ggd = gfeat[1];
    const float ggdx = gfeat[2] + gfeat[4] * cf.nx;
    const float ggdy = gfeat[3] + gfeat[4] * cf.ny;
    const float gcfnx = gfeat[4] * gdx, gcfny = gfeat[4] * gdy;
    const float gd_safe = gd > 1e-9f ? gd : 1e-9f;
    const float inv2 = inv * inv;
    float g_dx = ggd * dx / gd_safe;
    float g_dy = ggd * dy / gd_safe;
    g_dx += ggdx * (inv - dx * dx * inv2 / gd_safe);
    g_dy += ggdx * (-dx * dy * inv2 / gd_safe);
    g_dy += ggdy * (inv - dy * dy * inv2 / gd_safe);
    g_dx += ggdy * (-dx * dy * inv2 / gd_safe);
    god_x += -g_dx;
    god_y += -g_dy;
    float cdox, cdoy;
    diff::sample_bwd(cf, gphi_cf, gcfnx, gcfny, cdox, cdoy);
    god_x += cdox;
    god_y += cdoy;

    go_x = god_x;
    go_y = god_y;
    gaux_x = gauxx;
    gaux_y = gauxy;
  }
}

// Sum of squares of the gradient (for the global-norm clip).
__global__ void grad_sqnorm_kernel(const float *g, int P, float *out) {
  __shared__ float sh[256];
  float acc = 0.0f;
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < P; i += gridDim.x * blockDim.x)
    acc += g[i] * g[i];
  sh[threadIdx.x] = acc;
  __syncthreads();
  for (int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (threadIdx.x < s)
      sh[threadIdx.x] += sh[threadIdx.x + s];
    __syncthreads();
  }
  if (threadIdx.x == 0)
    atomicAdd(out, sh[0]);
}

// In-place Adam update (identical to coef_trainer::adam_step); gscale folds the
// global-norm clip, bc1/bc2 the bias correction. One thread per param.
__global__ void adam_kernel(float *p, float *m, float *u, const float *g, int P, float lr, float b1,
                            float b2, float eps, float bc1, float bc2, float gscale) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= P)
    return;
  const float gg = g[i] * gscale;
  m[i] = b1 * m[i] + (1.0f - b1) * gg;
  u[i] = b2 * u[i] + (1.0f - b2) * gg * gg;
  const float mhat = m[i] / bc1, uhat = u[i] / bc2;
  p[i] -= lr * mhat / (sqrtf(uhat) + eps);
}

void cuda_check(cudaError_t e, const char *what) {
  if (e != cudaSuccess)
    throw std::runtime_error(std::string("cvc::nav::coef_train CUDA: ") + what + ": " +
                             cudaGetErrorString(e));
}

diff::field to_diff_field(const field_stack &fs, const float *d_data) {
  diff::field F;
  F.data = d_data;
  F.H = fs.H;
  F.W = fs.W;
  F.S = (float)fs.S;
  F.cx = (float)fs.cx;
  F.cy = (float)fs.cy;
  F.mnx = (float)fs.mnx;
  F.mny = (float)fs.mny;
  F.mxx = (float)fs.mxx;
  F.mxy = (float)fs.mxy;
  return F;
}

diff::bike_veh to_bike_veh(const training_scene &sc, const train_config &cfg) {
  diff::bike_veh bv;
  bv.rr = sc.rr;
  bv.d_hat = sc.d_hat;
  bv.vmax = sc.vmax;
  bv.L = cfg.veh_L;
  bv.delta_max = cfg.veh_delta_max;
  bv.a_max = cfg.veh_a_max;
  bv.a_lat_max = cfg.veh_a_lat_max;
  bv.k_steer = cfg.veh_k_steer;
  bv.hdt = sc.dt;
  bv.allow_reverse = cfg.veh_allow_reverse ? 1 : 0;
  return bv;
}

} // namespace

bool train_cuda_available() {
  int c = 0;
  return cudaGetDeviceCount(&c) == cudaSuccess && c > 0;
}

double loss_and_grad_cuda(const training_scene &scene, const train_config &cfg,
                          const std::vector<float> &params, const float *o, const float *v,
                          const float *goal, int n, int window, std::vector<float> *grad,
                          float *o_out, float *v_out) {
  if (!train_cuda_available())
    throw std::runtime_error("cvc::nav::loss_and_grad_cuda: no CUDA device");
  const field_stack fs = scene.field();
  const int h = cfg.hidden;
  if (h > kMaxH)
    throw std::runtime_error("cvc::nav::loss_and_grad_cuda: hidden > 64 unsupported");
  const int P = static_cast<int>(params.size());
  const long hw = static_cast<long>(fs.H) * fs.W;
  const int rollout = cfg.rollout == rollout_kind::bicycle ? 1 : 0;
  const diff::bike_veh bv = to_bike_veh(scene, cfg);
  const float ob0 = std::log(std::expm1(1.0f)), ob1 = std::log(std::expm1(3.0f)),
              ob2 = std::log(std::expm1(4.0f));

  float *d_field, *d_p, *d_o, *d_v, *d_goal, *d_state, *d_loss, *d_grad, *d_oout = nullptr,
                                                                         *d_vout = nullptr;
  cuda_check(cudaMalloc(&d_field, 3 * hw * sizeof(float)), "malloc field");
  cuda_check(cudaMalloc(&d_p, P * sizeof(float)), "malloc p");
  cuda_check(cudaMalloc(&d_o, 2 * n * sizeof(float)), "malloc o");
  cuda_check(cudaMalloc(&d_v, 2 * n * sizeof(float)), "malloc v");
  cuda_check(cudaMalloc(&d_goal, 2 * n * sizeof(float)), "malloc goal");
  cuda_check(cudaMalloc(&d_state, (long)n * window * 4 * sizeof(float)), "malloc state");
  cuda_check(cudaMalloc(&d_loss, n * sizeof(float)), "malloc loss");
  cuda_check(cudaMalloc(&d_grad, P * sizeof(float)), "malloc grad");
  if (o_out)
    cuda_check(cudaMalloc(&d_oout, 2 * n * sizeof(float)), "malloc oout");
  if (v_out)
    cuda_check(cudaMalloc(&d_vout, 2 * n * sizeof(float)), "malloc vout");
  auto H2D = [&](void *d, const void *hst, size_t bytes, const char *w) {
    cuda_check(cudaMemcpy(d, hst, bytes, cudaMemcpyHostToDevice), w);
  };
  H2D(d_field, fs.data, 3 * hw * sizeof(float), "H2D field");
  H2D(d_p, params.data(), P * sizeof(float), "H2D p");
  H2D(d_o, o, 2 * n * sizeof(float), "H2D o");
  H2D(d_v, v, 2 * n * sizeof(float), "H2D v");
  H2D(d_goal, goal, 2 * n * sizeof(float), "H2D goal");
  cuda_check(cudaMemset(d_grad, 0, P * sizeof(float)), "memset grad");

  const diff::field F = to_diff_field(fs, d_field);
  const float inv_n = 1.0f / n, coll_w = cfg.w_coll / static_cast<float>(window);
  const int T = 128, B = (n + T - 1) / T;
  train_kernel<<<B, T>>>(F, d_p, h, ob0, ob1, ob2, d_o, d_v, d_goal, bv, coll_w, window, n, inv_n,
                         rollout, d_state, d_loss, d_grad, d_oout, d_vout);
  cuda_check(cudaGetLastError(), "train_kernel launch");

  std::vector<float> hloss(n);
  cuda_check(cudaMemcpy(hloss.data(), d_loss, n * sizeof(float), cudaMemcpyDeviceToHost),
             "D2H loss");
  if (grad) {
    grad->resize(P);
    cuda_check(cudaMemcpy(grad->data(), d_grad, P * sizeof(float), cudaMemcpyDeviceToHost),
               "D2H grad");
  }
  if (o_out)
    cuda_check(cudaMemcpy(o_out, d_oout, 2 * n * sizeof(float), cudaMemcpyDeviceToHost),
               "D2H oout");
  if (v_out)
    cuda_check(cudaMemcpy(v_out, d_vout, 2 * n * sizeof(float), cudaMemcpyDeviceToHost),
               "D2H vout");

  cudaFree(d_field);
  cudaFree(d_p);
  cudaFree(d_o);
  cudaFree(d_v);
  cudaFree(d_goal);
  cudaFree(d_state);
  cudaFree(d_loss);
  cudaFree(d_grad);
  if (d_oout)
    cudaFree(d_oout);
  if (d_vout)
    cudaFree(d_vout);

  double total = 0.0;
  for (int i = 0; i < n; ++i)
    total += hloss[i];
  return total;
}

coef_mlp train_coef_mlp_cuda(const training_scene &scene, const train_config &cfg, bool verbose) {
  if (!train_cuda_available())
    throw std::runtime_error("cvc::nav::train_coef_mlp_cuda: no CUDA device");
  const int h = cfg.hidden;
  if (h > kMaxH)
    throw std::runtime_error("cvc::nav::train_coef_mlp_cuda: hidden > 64 unsupported");
  coef_trainer tr(cfg, /*init_seed=*/1); // host: initial params + final bake only
  const int n = cfg.n, horizon = cfg.horizon, window = cfg.window, P = tr.num_params();
  const bool bike = cfg.rollout == rollout_kind::bicycle;
  const int rollout = bike ? 1 : 0;
  const field_stack fs = scene.field();
  const long hw = static_cast<long>(fs.H) * fs.W;
  const diff::bike_veh bv = to_bike_veh(scene, cfg);
  const float ob0 = std::log(std::expm1(1.0f)), ob1 = std::log(std::expm1(3.0f)),
              ob2 = std::log(std::expm1(4.0f));

  float *d_field, *d_p, *d_m, *d_u, *d_o, *d_v, *d_goal, *d_o2, *d_v2, *d_state, *d_loss, *d_grad,
      *d_sq;
  cuda_check(cudaMalloc(&d_field, 3 * hw * sizeof(float)), "malloc field");
  cuda_check(cudaMalloc(&d_p, P * sizeof(float)), "malloc p");
  cuda_check(cudaMalloc(&d_m, P * sizeof(float)), "malloc m");
  cuda_check(cudaMalloc(&d_u, P * sizeof(float)), "malloc u");
  cuda_check(cudaMalloc(&d_o, 2 * n * sizeof(float)), "malloc o");
  cuda_check(cudaMalloc(&d_v, 2 * n * sizeof(float)), "malloc v");
  cuda_check(cudaMalloc(&d_goal, 2 * n * sizeof(float)), "malloc goal");
  cuda_check(cudaMalloc(&d_o2, 2 * n * sizeof(float)), "malloc o2");
  cuda_check(cudaMalloc(&d_v2, 2 * n * sizeof(float)), "malloc v2");
  cuda_check(cudaMalloc(&d_state, (long)n * window * 4 * sizeof(float)), "malloc state");
  cuda_check(cudaMalloc(&d_loss, n * sizeof(float)), "malloc loss");
  cuda_check(cudaMalloc(&d_grad, P * sizeof(float)), "malloc grad");
  cuda_check(cudaMalloc(&d_sq, sizeof(float)), "malloc sq");

  cuda_check(cudaMemcpy(d_field, fs.data, 3 * hw * sizeof(float), cudaMemcpyHostToDevice),
             "H2D field");
  cuda_check(cudaMemcpy(d_p, tr.params().data(), P * sizeof(float), cudaMemcpyHostToDevice),
             "H2D p");
  cuda_check(cudaMemset(d_m, 0, P * sizeof(float)), "memset m");
  cuda_check(cudaMemset(d_u, 0, P * sizeof(float)), "memset u");
  const diff::field F = to_diff_field(fs, d_field);

  std::vector<float> o(2 * n), goal(2 * n), aux(2 * n);
  const float inv_n = 1.0f / n, coll_w = cfg.w_coll / static_cast<float>(window);
  const float b1 = 0.9f, b2 = 0.999f, eps = 1e-8f;
  long adam_t = 0;
  const int T = 128, B = (n + T - 1) / T, PT = 256, PB = (P + PT - 1) / PT;

  for (int step = 0; step < cfg.steps; ++step) {
    scene.sample_starts_goals(n, cfg.seed + (unsigned)step, o.data(), goal.data());
    for (int i = 0; i < n; ++i) { // aux init: surrogate v=0; bicycle (th->goal, sp=0)
      if (bike) {
        aux[2 * i] = std::atan2(goal[2 * i + 1] - o[2 * i + 1], goal[2 * i] - o[2 * i]);
        aux[2 * i + 1] = 0.0f;
      } else {
        aux[2 * i] = 0.0f;
        aux[2 * i + 1] = 0.0f;
      }
    }
    cuda_check(cudaMemcpy(d_o, o.data(), 2 * n * sizeof(float), cudaMemcpyHostToDevice), "H2D o");
    cuda_check(cudaMemcpy(d_goal, goal.data(), 2 * n * sizeof(float), cudaMemcpyHostToDevice),
               "H2D goal");
    cuda_check(cudaMemcpy(d_v, aux.data(), 2 * n * sizeof(float), cudaMemcpyHostToDevice), "H2D v");
    for (int w0 = 0; w0 < horizon; w0 += window) {
      const int wl = std::min(window, horizon - w0);
      cuda_check(cudaMemset(d_grad, 0, P * sizeof(float)), "memset grad");
      train_kernel<<<B, T>>>(F, d_p, h, ob0, ob1, ob2, d_o, d_v, d_goal, bv, coll_w, wl, n, inv_n,
                             rollout, d_state, d_loss, d_grad, d_o2, d_v2);
      cuda_check(cudaMemset(d_sq, 0, sizeof(float)), "memset sq");
      grad_sqnorm_kernel<<<32, 256>>>(d_grad, P, d_sq);
      float sq = 0.0f;
      cuda_check(cudaMemcpy(&sq, d_sq, sizeof(float), cudaMemcpyDeviceToHost), "D2H sq");
      const float norm = std::sqrt(sq);
      const float gscale =
          (cfg.grad_clip > 0.0f && norm > cfg.grad_clip) ? cfg.grad_clip / norm : 1.0f;
      ++adam_t;
      const float bc1 = 1.0f - std::pow(b1, (float)adam_t);
      const float bc2 = 1.0f - std::pow(b2, (float)adam_t);
      adam_kernel<<<PB, PT>>>(d_p, d_m, d_u, d_grad, P, cfg.lr, b1, b2, eps, bc1, bc2, gscale);
      std::swap(d_o, d_o2);
      std::swap(d_v, d_v2);
    }
    if (verbose && (step % 50 == 0 || step == cfg.steps - 1)) {
      std::vector<float> hl(n);
      cuda_check(cudaMemcpy(hl.data(), d_loss, n * sizeof(float), cudaMemcpyDeviceToHost),
                 "D2H loss");
      double L = 0.0;
      for (float x : hl)
        L += x;
      std::printf("  [cuda-resident] step %4d: window_loss %.4f\n", step, L);
    }
  }
  cuda_check(cudaGetLastError(), "resident train loop");

  std::vector<float> params(P);
  cuda_check(cudaMemcpy(params.data(), d_p, P * sizeof(float), cudaMemcpyDeviceToHost), "D2H p");
  tr.set_params(params);

  cudaFree(d_field);
  cudaFree(d_p);
  cudaFree(d_m);
  cudaFree(d_u);
  cudaFree(d_o);
  cudaFree(d_v);
  cudaFree(d_goal);
  cudaFree(d_o2);
  cudaFree(d_v2);
  cudaFree(d_state);
  cudaFree(d_loss);
  cudaFree(d_grad);
  cudaFree(d_sq);
  return tr.to_coef_mlp();
}

} // namespace nav
} // namespace cvc
