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
// window forward + hand-written backward (a device transcription of
// coef_train.cpp), accumulating the per-param gradient with atomicAdd. Built
// WITHOUT --use_fast_math (see src/cvc/CMakeLists.txt) so the float32 forward
// tracks the CPU trainer; validated float-equivalent to it (nav_coef_train_test,
// CVC_ENABLE_CUDA). Checkpoint/recompute: only (o_t, v_t) per step is stored;
// the MLP / samples are recomputed in the backward (tiny net, so cheap).
//
// The Adam optimizer, param init, scene sampling and .cvcnav bake stay on the
// host (coef_trainer) — only the loss+gradient is on the GPU. loss_and_grad_cuda
// re-uploads the field/params each call; the win is the per-agent BPTT, and this
// is the training analogue of drive_step_cuda (a device-resident training loop
// is a later optimization, mirroring sim_world_cuda).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cuda_runtime.h>
#include <cvc/nav/coef_train.h>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace nav {

namespace {

struct dtfield {
  const float *data;
  int H, W;
  float S, cx, cy, mnx, mny, mxx, mxy;
};

struct dSample {
  float phv[4], pxv[4], pyv[4];
  float wx0, wx1, wy0, wy1;
  int clx, cly;
  float Wf1, Hf1, cgx, cgy;
  float rnx, rny, r, mag;
  float phi, nx, ny;
};

__device__ inline float d_sigmoid(float x) { return 1.0f / (1.0f + expf(-x)); }
__device__ inline float d_silu(float x) { return x * d_sigmoid(x); }
__device__ inline float d_silu_grad(float x) {
  const float s = d_sigmoid(x);
  return s + x * s * (1.0f - s);
}
__device__ inline float d_softplus(float x) { return x > 20.0f ? x : log1pf(expf(x)); }
__device__ inline float d_ipc(float d, float d_hat) {
  const float dc = d < 1e-6f ? 1e-6f : d;
  if (!(dc < d_hat))
    return 0.0f;
  return (d_hat - dc) * (2.0f * logf(dc / d_hat) - d_hat / dc) + 1.0f;
}
__device__ inline float d_ipc_grad(float d, float d_hat) {
  if (d < 1e-6f)
    return 0.0f;
  if (!(d < d_hat))
    return 0.0f;
  const float A = d_hat - d;
  const float B = 2.0f * logf(d / d_hat) - d_hat / d;
  const float dB = 2.0f / d + d_hat / (d * d);
  return -B + A * dB;
}

__device__ dSample d_sample_fwd(const dtfield &f, float onx, float ony) {
  dSample s;
  const float wx = onx / f.S + f.cx, wy = ony / f.S + f.cy;
  const float gx = 2.0f * (wx - f.mnx) / (f.mxx - f.mnx) - 1.0f;
  const float gy = 2.0f * (wy - f.mny) / (f.mxy - f.mny) - 1.0f;
  s.cgx = 2.0f / ((f.mxx - f.mnx) * f.S);
  s.cgy = 2.0f / ((f.mxy - f.mny) * f.S);
  s.Wf1 = f.W - 1;
  s.Hf1 = f.H - 1;
  float ix = (gx + 1.0f) * 0.5f * s.Wf1;
  float iy = (gy + 1.0f) * 0.5f * s.Hf1;
  s.clx = (ix < 0.0f) || (ix > s.Wf1);
  s.cly = (iy < 0.0f) || (iy > s.Hf1);
  ix = fminf(fmaxf(ix, 0.0f), s.Wf1);
  iy = fminf(fmaxf(iy, 0.0f), s.Hf1);
  const int ix0 = (int)floorf(ix), iy0 = (int)floorf(iy);
  s.wx1 = ix - ix0;
  s.wx0 = 1.0f - s.wx1;
  s.wy1 = iy - iy0;
  s.wy0 = 1.0f - s.wy1;
  const int cx0 = min(max(ix0, 0), f.W - 1), cx1 = min(max(ix0 + 1, 0), f.W - 1);
  const int cy0 = min(max(iy0, 0), f.H - 1), cy1 = min(max(iy0 + 1, 0), f.H - 1);
  const long HW = (long)f.H * f.W;
  const float *ph = f.data, *px = f.data + HW, *py = f.data + 2 * HW;
  const long nw = (long)cy0 * f.W + cx0, ne = (long)cy0 * f.W + cx1;
  const long sw = (long)cy1 * f.W + cx0, se = (long)cy1 * f.W + cx1;
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
  s.r = sqrtf(s.rnx * s.rnx + s.rny * s.rny);
  s.mag = s.r + 1e-6f;
  s.nx = s.rnx / s.mag;
  s.ny = s.rny / s.mag;
  return s;
}

__device__ void d_sample_bwd(const dSample &s, float gphi, float gnx, float gny, float &gonx,
                             float &gony) {
  float grnx = 0.0f, grny = 0.0f;
  if (s.r > 0.0f) {
    const float mag2 = s.mag * s.mag;
    const float dnx_drnx = (s.mag - s.rnx * s.rnx / s.r) / mag2;
    const float dny_drny = (s.mag - s.rny * s.rny / s.r) / mag2;
    const float dcross = -(s.rnx * s.rny) / (s.r * mag2);
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

// Per-agent window forward + backward. Hidden width <= 64 (kMaxH).
constexpr int kMaxH = 64;

__global__ void train_kernel(dtfield F, const float *p, int h, float ob0, float ob1, float ob2,
                             const float *o_in, const float *v_in, const float *goal, float rr,
                             float d_hat, float vmax, float hdt, float coll_w, int window, int n,
                             float inv_n, float *state, float *d_loss, float *grad, float *o_out,
                             float *v_out) {
  const int ag = blockIdx.x * blockDim.x + threadIdx.x;
  if (ag >= n)
    return;
  const int ow0 = 0, ob0o = h * 5, ow1 = h * 5 + h, ob1o = ow1 + h * h, ow2 = ob1o + h,
            ob2o = ow2 + 3 * h;
  const float offb[3] = {ob0, ob1, ob2};
  float *st = state + (long)ag * window * 4;
  const float gx = goal[2 * ag], gy = goal[2 * ag + 1];
  float ox = o_in[2 * ag], oy = o_in[2 * ag + 1], vx = v_in[2 * ag], vy = v_in[2 * ag + 1];
  double loss = 0.0;

  // ── forward (store only o_t, v_t) ──
  for (int t = 0; t < window; ++t) {
    st[t * 4 + 0] = ox;
    st[t * 4 + 1] = oy;
    st[t * 4 + 2] = vx;
    st[t * 4 + 3] = vy;
    dSample cf = d_sample_fwd(F, ox, oy);
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
      a0[o] = d_silu(acc);
    }
    for (int o = 0; o < h; ++o) {
      float acc = p[ob1o + o];
      const float *w = p + ow1 + o * h;
      for (int i = 0; i < h; ++i)
        acc += w[i] * a0[i];
      a1[o] = d_silu(acc);
    }
    float coef[3];
    for (int o = 0; o < 3; ++o) {
      float acc = p[ob2o + o];
      const float *w = p + ow2 + o * h;
      for (int i = 0; i < h; ++i)
        acc += w[i] * a1[i];
      coef[o] = d_softplus(acc + offb[o]);
    }
    const float al = coef[0], be = coef[1], ga = coef[2];
    dSample rl = d_sample_fwd(F, ox, oy);
    const float d = rl.phi - rr, ipc = d_ipc(d, d_hat);
    const float ax = -(al * ipc) * rl.nx - be * (ox - gx) - ga * vx;
    const float ay = -(al * ipc) * rl.ny - be * (oy - gy) - ga * vy;
    float vpx = vx + hdt * ax, vpy = vy + hdt * ay;
    const float sp = sqrtf(vpx * vpx + vpy * vpy);
    float vcx = vpx, vcy = vpy;
    if (sp > vmax) {
      const float sc = vmax / sp;
      vcx = vpx * sc;
      vcy = vpy * sc;
    }
    ox = ox + hdt * vcx;
    oy = oy + hdt * vcy;
    vx = vcx;
    vy = vcy;
    dSample cs = d_sample_fwd(F, ox, oy);
    const float pen = rr - cs.phi;
    if (pen > 0.0f)
      loss += (double)coll_w * pen * inv_n;
  }
  const float fdx = ox - gx, fdy = oy - gy, Lgoal = sqrtf(fdx * fdx + fdy * fdy);
  loss += (double)Lgoal * inv_n;
  d_loss[ag] = (float)loss;
  if (o_out) {
    o_out[2 * ag] = ox;
    o_out[2 * ag + 1] = oy;
  }
  if (v_out) {
    v_out[2 * ag] = vx;
    v_out[2 * ag + 1] = vy;
  }

  // ── backward (recompute per step) ──
  float go_x = 0.0f, go_y = 0.0f, gv_x = 0.0f, gv_y = 0.0f;
  if (Lgoal > 1e-9f) {
    go_x = inv_n * fdx / Lgoal;
    go_y = inv_n * fdy / Lgoal;
  }
  for (int t = window - 1; t >= 0; --t) {
    const float otx = st[t * 4 + 0], oty = st[t * 4 + 1];
    const float vtx = st[t * 4 + 2], vty = st[t * 4 + 3];
    // recompute forward at step t
    dSample cf = d_sample_fwd(F, otx, oty);
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
      a0[o] = d_silu(acc);
    }
    for (int o = 0; o < h; ++o) {
      float acc = p[ob1o + o];
      const float *w = p + ow1 + o * h;
      for (int i = 0; i < h; ++i)
        acc += w[i] * a0[i];
      z1[o] = acc;
      a1[o] = d_silu(acc);
    }
    float raw[3], coef[3];
    for (int o = 0; o < 3; ++o) {
      float acc = p[ob2o + o];
      const float *w = p + ow2 + o * h;
      for (int i = 0; i < h; ++i)
        acc += w[i] * a1[i];
      raw[o] = acc;
      coef[o] = d_softplus(acc + offb[o]);
    }
    const float al = coef[0], be = coef[1], ga = coef[2];
    dSample rl = d_sample_fwd(F, otx, oty);
    const float dd = rl.phi - rr, ipc = d_ipc(dd, d_hat);
    const float ax = -(al * ipc) * rl.nx - be * (otx - gx) - ga * vtx;
    const float ay = -(al * ipc) * rl.ny - be * (oty - gy) - ga * vty;
    float vpx = vtx + hdt * ax, vpy = vty + hdt * ay;
    const float sp = sqrtf(vpx * vpx + vpy * vpy);
    const int scaled = sp > vmax;
    float vcx = vpx, vcy = vpy;
    if (scaled) {
      const float sc = vmax / sp;
      vcx = vpx * sc;
      vcy = vpy * sc;
    }
    const float ox1 = otx + hdt * vcx, oy1 = oty + hdt * vcy;
    dSample cs = d_sample_fwd(F, ox1, oy1);

    // collision grad at o_{t+1}
    const float pen = rr - cs.phi;
    if (pen > 0.0f) {
      float dox, doy;
      d_sample_bwd(cs, -coll_w * inv_n, 0.0f, 0.0f, dox, doy);
      go_x += dox;
      go_y += doy;
    }
    // rollout backward
    float gvpp_x = gv_x + hdt * go_x, gvpp_y = gv_y + hdt * go_y;
    float god_x = go_x, god_y = go_y;
    float gvp_x, gvp_y;
    if (scaled) {
      const float sp3 = sp * sp * sp;
      const float dot = gvpp_x * vpx + gvpp_y * vpy;
      gvp_x = vmax / sp * gvpp_x - vmax / sp3 * vpx * dot;
      gvp_y = vmax / sp * gvpp_y - vmax / sp3 * vpy * dot;
    } else {
      gvp_x = gvpp_x;
      gvp_y = gvpp_y;
    }
    const float ga_x = hdt * gvp_x, ga_y = hdt * gvp_y;
    float gvt_x = gvp_x, gvt_y = gvp_y;
    float gFbar_x = ga_x, gFbar_y = ga_y, gFgoal_x = ga_x, gFgoal_y = ga_y;
    float g_al = 0.0f, g_be = 0.0f, g_ga = 0.0f;
    g_ga += -(ga_x * vtx + ga_y * vty);
    gvt_x += -ga * ga_x;
    gvt_y += -ga * ga_y;
    g_be += -(gFgoal_x * (otx - gx) + gFgoal_y * (oty - gy));
    god_x += -be * gFgoal_x;
    god_y += -be * gFgoal_y;
    const float aip = al * ipc;
    g_al += -ipc * (gFbar_x * rl.nx + gFbar_y * rl.ny);
    const float g_ipc = -al * (gFbar_x * rl.nx + gFbar_y * rl.ny);
    const float gnx = -aip * gFbar_x, gny = -aip * gFbar_y;
    const float gphi_force = g_ipc * d_ipc_grad(dd, d_hat);
    float sdox, sdoy;
    d_sample_bwd(rl, gphi_force, gnx, gny, sdox, sdoy);
    god_x += sdox;
    god_y += sdoy;

    // MLP backward -> grad (atomicAdd), gfeat
    float graw[3];
    graw[0] = g_al * d_sigmoid(raw[0] + offb[0]);
    graw[1] = g_be * d_sigmoid(raw[1] + offb[1]);
    graw[2] = g_ga * d_sigmoid(raw[2] + offb[2]);
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
      gz1[i] = ga1[i] * d_silu_grad(z1[i]);
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
      const float gg = ga0[o] * d_silu_grad(z0[o]);
      for (int i = 0; i < 5; ++i) {
        atomicAdd(&grad[wb + i], gg * feat[i]);
        gfeat[i] += gg * p[wb + i];
      }
      atomicAdd(&grad[ob0o + o], gg);
    }

    // coef_feats backward -> grad o_t
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
    d_sample_bwd(cf, gphi_cf, gcfnx, gcfny, cdox, cdoy);
    god_x += cdox;
    god_y += cdoy;

    go_x = god_x;
    go_y = god_y;
    gv_x = gvt_x;
    gv_y = gvt_y;
  }
}

// Sum of squares of the gradient (for the global-norm clip), block-reduced then
// atomically summed into out[0] (which the caller zeroes first).
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

// In-place Adam update of the resident params (identical formula to
// coef_trainer::adam_step; `gscale` folds the global-norm clip, bc1/bc2 the
// bias correction). One thread per param.
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

  dtfield F;
  F.H = fs.H;
  F.W = fs.W;
  F.S = (float)fs.S;
  F.cx = (float)fs.cx;
  F.cy = (float)fs.cy;
  F.mnx = (float)fs.mnx;
  F.mny = (float)fs.mny;
  F.mxx = (float)fs.mxx;
  F.mxy = (float)fs.mxy;
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

  F.data = d_field;
  const float inv_n = 1.0f / n;
  const float coll_w = cfg.w_coll / static_cast<float>(window);
  const int T = 128, B = (n + T - 1) / T;
  train_kernel<<<B, T>>>(F, d_p, h, ob0, ob1, ob2, d_o, d_v, d_goal, scene.rr, scene.d_hat,
                         scene.vmax, scene.dt, coll_w, window, n, inv_n, d_state, d_loss, d_grad,
                         d_oout, d_vout);
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

// FULLY DEVICE-RESIDENT training loop: the field, the params, the Adam moments
// and all per-window scratch stay on the GPU across the ENTIRE run. Per outer
// step only the fresh agent batch (o, goal) is uploaded; each window runs the
// forward/backward kernel, an in-place device Adam (a one-float D2H for the
// grad-clip norm, never the gradient itself), and pointer-swaps the pose
// continuation. Only the final trained params come back to the host to bake the
// coef_mlp — the training never round-trips through host memory (contrast the
// per-call loss_and_grad_cuda, which re-uploads everything each window).
coef_mlp train_coef_mlp_cuda(const training_scene &scene, const train_config &cfg, bool verbose) {
  if (!train_cuda_available())
    throw std::runtime_error("cvc::nav::train_coef_mlp_cuda: no CUDA device");
  const int h = cfg.hidden;
  if (h > kMaxH)
    throw std::runtime_error("cvc::nav::train_coef_mlp_cuda: hidden > 64 unsupported");
  coef_trainer tr(cfg, /*init_seed=*/1); // host: initial params + final bake only
  const int n = cfg.n, horizon = cfg.horizon, window = cfg.window, P = tr.num_params();
  const field_stack fs = scene.field();
  const long hw = static_cast<long>(fs.H) * fs.W;

  dtfield F;
  F.H = fs.H;
  F.W = fs.W;
  F.S = (float)fs.S;
  F.cx = (float)fs.cx;
  F.cy = (float)fs.cy;
  F.mnx = (float)fs.mnx;
  F.mny = (float)fs.mny;
  F.mxx = (float)fs.mxx;
  F.mxy = (float)fs.mxy;
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
  F.data = d_field;

  std::vector<float> o(2 * n), goal(2 * n);
  const float inv_n = 1.0f / n, coll_w = cfg.w_coll / static_cast<float>(window);
  const float b1 = 0.9f, b2 = 0.999f, eps = 1e-8f;
  long adam_t = 0;
  const int T = 128, B = (n + T - 1) / T, PT = 256, PB = (P + PT - 1) / PT;

  for (int step = 0; step < cfg.steps; ++step) {
    scene.sample_starts_goals(n, cfg.seed + (unsigned)step, o.data(), goal.data());
    cuda_check(cudaMemcpy(d_o, o.data(), 2 * n * sizeof(float), cudaMemcpyHostToDevice), "H2D o");
    cuda_check(cudaMemcpy(d_goal, goal.data(), 2 * n * sizeof(float), cudaMemcpyHostToDevice),
               "H2D goal");
    cuda_check(cudaMemset(d_v, 0, 2 * n * sizeof(float)), "memset v");
    for (int w0 = 0; w0 < horizon; w0 += window) {
      const int wl = std::min(window, horizon - w0);
      cuda_check(cudaMemset(d_grad, 0, P * sizeof(float)), "memset grad");
      train_kernel<<<B, T>>>(F, d_p, h, ob0, ob1, ob2, d_o, d_v, d_goal, scene.rr, scene.d_hat,
                             scene.vmax, scene.dt, coll_w, wl, n, inv_n, d_state, d_loss, d_grad,
                             d_o2, d_v2);
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
