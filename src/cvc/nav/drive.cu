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

// drive.cu — the CUDA (GPU) drive, a device transcription of drive.cpp. Compiled
// only when CVC_ENABLE_CUDA, and (per src/cvc/CMakeLists.txt) WITHOUT
// --use_fast_math: -fmad=false and IEEE div/sqrt keep the float32 op order
// matching the CPU/torch reference, so the GPU drive stays float-equivalent (the
// deployment path when N outgrows the CPU; validated here, benchmarked on a GPU
// box). Manual bilinear — hardware texture filtering's 9-bit weights are ~1e-3,
// far outside the contract. One thread per agent.

#include <cmath>
#include <cuda_runtime.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/drive.h>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace nav {

namespace {

struct dev_field {
  const float *data;
  int H, W;
  float S, cx, cy, mnx, mny, mxx, mxy;
};

struct dev_veh {
  float rr, d_hat, dt, vmax, L, delta_max, a_max, a_lat_max, k_steer;
  int nsub;
  int allow_reverse;
};

// Bilinear sample (plane 0) + unit normal — mirrors drive.cpp sample_unit.
__device__ inline void d_sample_unit(const dev_field &f, float onx, float ony, float &phi,
                                     float &nxo, float &nyo) {
  const float wx = onx / f.S + f.cx, wy = ony / f.S + f.cy;
  const float gx = 2.0f * (wx - f.mnx) / (f.mxx - f.mnx) - 1.0f;
  const float gy = 2.0f * (wy - f.mny) / (f.mxy - f.mny) - 1.0f;
  const float Wf1 = f.W - 1, Hf1 = f.H - 1;
  float ix = (gx + 1.0f) * 0.5f * Wf1, iy = (gy + 1.0f) * 0.5f * Hf1;
  ix = fminf(fmaxf(ix, 0.0f), Wf1);
  iy = fminf(fmaxf(iy, 0.0f), Hf1);
  const int ix0 = (int)floorf(ix), iy0 = (int)floorf(iy);
  const float wx1 = ix - ix0, wx0 = 1.0f - wx1, wy1 = iy - iy0, wy0 = 1.0f - wy1;
  const float nw = wx0 * wy0, ne = wx1 * wy0, sw = wx0 * wy1, se = wx1 * wy1;
  const int cx0 = min(max(ix0, 0), f.W - 1), cx1 = min(max(ix0 + 1, 0), f.W - 1);
  const int cy0 = min(max(iy0, 0), f.H - 1), cy1 = min(max(iy0 + 1, 0), f.H - 1);
  const long HW = (long)f.H * f.W;
  const float *ph = f.data, *px = f.data + HW, *py = f.data + 2 * HW;
  const long a = (long)cy0 * f.W + cx0, b = (long)cy0 * f.W + cx1, c = (long)cy1 * f.W + cx0,
             d = (long)cy1 * f.W + cx1;
  phi = ph[a] * nw + ph[b] * ne + ph[c] * sw + ph[d] * se;
  float rnx = px[a] * nw + px[b] * ne + px[c] * sw + px[d] * se;
  float rny = py[a] * nw + py[b] * ne + py[c] * sw + py[d] * se;
  const float mag = sqrtf(rnx * rnx + rny * rny) + 1e-6f;
  nxo = rnx / mag;
  nyo = rny / mag;
}

__device__ inline float d_ipc(float dd, float d_hat) {
  const float dc = dd < 1e-6f ? 1e-6f : dd;
  if (!(dc < d_hat))
    return 0.0f;
  return (d_hat - dc) * (2.0f * logf(dc / d_hat) - d_hat / dc) + 1.0f;
}
__device__ inline float d_silu(float x) { return x * (1.0f / (1.0f + expf(-x))); }
__device__ inline float d_softplus(float x) { return x > 20.0f ? x : log1pf(expf(x)); }

// General MLP forward (loops uploaded layers) — mirrors coef_mlp::forward.
__device__ inline void d_mlp(const float *data, const int *rows, const int *cols, const int *act,
                             const long *w_off, const long *b_off, int num_layers,
                             const float *out_bias_off, int in, int out, const float *feat,
                             float *coef) {
  float a[64], b[64];
  for (int i = 0; i < in; ++i)
    a[i] = feat[i];
  for (int L = 0; L < num_layers; ++L) {
    const float *w = data + w_off[L];
    const float *bb = data + b_off[L];
    const int R = rows[L], C = cols[L];
    for (int o = 0; o < R; ++o) {
      float acc = bb[o];
      const float *wr = w + (long)o * C;
      for (int i = 0; i < C; ++i)
        acc += wr[i] * a[i];
      b[o] = (act[L] == 1) ? d_silu(acc) : acc;
    }
    for (int o = 0; o < R; ++o)
      a[o] = b[o];
  }
  for (int k = 0; k < out; ++k)
    coef[k] = d_softplus(a[k] + out_bias_off[k]);
}

// One agent's bicycle rollout (nsub substeps) — mirrors drive.cpp bicycle_rollout.
__device__ inline void d_bicycle(const dev_field &f, float &ox, float &oy, float &thi, float &spi,
                                 float gx, float gy, float al, float be, float ga, const dev_veh &v,
                                 float &minclr) {
  const float hdt = v.dt / (float)v.nsub;
  const float tan_dmax = tanf(v.delta_max);
  const float sp_min = v.allow_reverse ? -0.25f * v.vmax : 0.0f;
  const float v_creep_cap = 0.5f * sqrtf(v.a_lat_max * v.L / tan_dmax);
  const float dmax = v.delta_max;
  minclr = 9.9f;
  for (int s = 0; s < v.nsub; ++s) {
    float phi, nx, ny;
    d_sample_unit(f, ox, oy, phi, nx, ny);
    const float d = phi - v.rr;
    minclr = fminf(minclr, d);
    const float ipc = d_ipc(d, v.d_hat);
    const float Fbar_x = -(al * ipc) * nx, Fbar_y = -(al * ipc) * ny;
    const float Fx = Fbar_x - be * (ox - gx), Fy = Fbar_y - be * (oy - gy);
    const float ch = cosf(thi), sh = sinf(thi);
    float a_long = (Fx * ch + Fy * sh) - ga * spi;
    a_long = fminf(fmaxf(a_long, -v.a_max), v.a_max);
    const float tgx = gx - ox, tgy = gy - oy;
    float L_d = sqrtf(tgx * tgx + tgy * tgy);
    if (L_d < 1e-6f)
      L_d = 1e-6f;
    const float ang = atan2f(tgy, tgx) - thi;
    const float sin_a = sinf(ang), cos_a = cosf(ang);
    const bool behind = cos_a < 0.0f;
    const float turn_sign = sin_a >= 0.0f ? 1.0f : -1.0f;
    float L_d_eff = 1.2f * spi + 4.0f * v.L;
    if (L_d_eff < 4.0f * v.L)
      L_d_eff = 4.0f * v.L;
    L_d_eff = fminf(L_d, L_d_eff);
    float delta = behind ? turn_sign * dmax : atan2f(2.0f * v.L * sin_a, L_d_eff);
    const float ipc_rep = ipc < 0.0f ? ipc : 0.0f;
    const float Frep_x = -(al * ipc_rep) * nx, Frep_y = -(al * ipc_rep) * ny;
    delta = delta + v.k_steer * tanhf(Frep_x * (-sh) + Frep_y * ch);
    delta = fminf(fmaxf(delta, -dmax), dmax);
    float kappa = fabsf(tanf(delta)) / v.L;
    const float kfloor = tan_dmax / (v.L * 400.0f);
    if (kappa < kfloor)
      kappa = kfloor;
    const float v_corner = sqrtf(v.a_lat_max / kappa);
    float ds = d - 0.5f * v.rr;
    if (ds < 0.0f)
      ds = 0.0f;
    const float v_stop = sqrtf(2.0f * v.a_max * ds);
    const float motion_sign = spi >= 0.0f ? 1.0f : -1.0f;
    float approach = -(nx * ch + ny * sh) * motion_sign;
    approach = fminf(fmaxf(approach, 0.0f), 1.0f);
    const float v_stop_dir = approach > 0.05f ? v_stop / fmaxf(approach, 0.05f) : v.vmax;
    float v_lim = fminf(v.vmax, fminf(v_corner, v_stop_dir));
    const bool hard_steer = fabsf(delta) >= 0.7f * dmax;
    const bool can_move = d > 0.25f * v.rr;
    const float v_floor = (hard_steer && can_move) ? 0.08f : 0.0f;
    v_lim = fmaxf(v_lim, v_floor);
    const float v_creep = fminf(0.5f * v_corner, v_creep_cap);
    if (behind) {
      float fbh = Fbar_x * ch + Fbar_y * sh;
      if (fbh > 0.0f)
        fbh = 0.0f;
      a_long = (v_creep - spi) / hdt + fbh;
    }
    const bool stuck = hard_steer && can_move && fabsf(spi) < 0.06f;
    if (stuck)
      a_long = fmaxf(a_long, (0.08f - spi) / hdt);
    if (v.allow_reverse) {
      const float head_on = fminf(fmaxf(-(nx * ch + ny * sh), 0.0f), 1.0f);
      const bool nose_blocked = behind && (head_on > 0.6f) && (d < 0.5f * v.rr + 0.02f);
      if (nose_blocked) {
        a_long = (-0.10f - spi) / hdt;
        delta = -delta;
      }
    }
    a_long = fminf(fmaxf(a_long, -v.a_max), v.a_max);
    a_long = fminf(a_long, (v_lim - spi) / hdt);
    a_long = fminf(fmaxf(a_long, -v.a_max), v.a_max);
    spi = spi + hdt * a_long;
    spi = fminf(fmaxf(spi, sp_min), v.vmax);
    float sp2 = spi * spi;
    if (sp2 < 1e-9f)
      sp2 = 1e-9f;
    const float d_cap = atanf(v.a_lat_max * v.L / sp2);
    delta = fminf(fmaxf(delta, -d_cap), d_cap);
    thi = thi + hdt * (spi / v.L) * tanf(delta);
    ox = ox + hdt * spi * cosf(thi);
    oy = oy + hdt * spi * sinf(thi);
  }
}

__global__ void sample_kernel(dev_field f, const float *on, int n, float *phi_out, float *nrm_out) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n)
    return;
  float phi, nx, ny;
  d_sample_unit(f, on[2 * i], on[2 * i + 1], phi, nx, ny);
  phi_out[i] = phi;
  nrm_out[2 * i] = nx;
  nrm_out[2 * i + 1] = ny;
}

__global__ void drive_kernel(dev_field f, float *o, float *th, float *sp, const float *carrot,
                             const float *wdata, const int *rows, const int *cols, const int *act,
                             const long *w_off, const long *b_off, int num_layers,
                             const float *out_bias_off, int in, int out, dev_veh v, int n,
                             float *minclr) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n)
    return;
  float ox = o[2 * i], oy = o[2 * i + 1], thi = th[i], spi = sp[i];
  const float cx = carrot[2 * i], cy = carrot[2 * i + 1];
  // coef_feats at o toward the carrot.
  float phi, nx, ny;
  d_sample_unit(f, ox, oy, phi, nx, ny);
  const float dgx = cx - ox, dgy = cy - oy;
  const float gd = sqrtf(dgx * dgx + dgy * dgy);
  const float inv = 1.0f / (gd + 1e-6f);
  const float gdx = dgx * inv, gdy = dgy * inv;
  float feat[8];
  feat[0] = phi;
  feat[1] = gd;
  feat[2] = gdx;
  feat[3] = gdy;
  feat[4] = gdx * nx + gdy * ny;
  float coef[8];
  d_mlp(wdata, rows, cols, act, w_off, b_off, num_layers, out_bias_off, in, out, feat, coef);
  float mc;
  d_bicycle(f, ox, oy, thi, spi, cx, cy, coef[0], coef[1], coef[2], v, mc);
  o[2 * i] = ox;
  o[2 * i + 1] = oy;
  th[i] = thi;
  sp[i] = spi;
  minclr[i] = mc;
}

dev_field to_dev_field(const field_stack &f, const float *d_data) {
  dev_field df;
  df.data = d_data;
  df.H = f.H;
  df.W = f.W;
  df.S = (float)f.S;
  df.cx = (float)f.cx;
  df.cy = (float)f.cy;
  df.mnx = (float)f.mnx;
  df.mny = (float)f.mny;
  df.mxx = (float)f.mxx;
  df.mxy = (float)f.mxy;
  return df;
}

void cuda_check(cudaError_t e, const char *what) {
  if (e != cudaSuccess)
    throw std::runtime_error(std::string("cvc::nav CUDA: ") + what + ": " + cudaGetErrorString(e));
}

} // namespace

void sdf_sample_cuda(const field_stack &f, const float *on, int n, float *phi_out,
                     float *normal_out) {
  if (n <= 0)
    return;
  const size_t fsz = (size_t)f.H * f.W * 3 * sizeof(float); // plane 0
  float *d_field = nullptr, *d_on = nullptr, *d_phi = nullptr, *d_nrm = nullptr;
  cuda_check(cudaMalloc(&d_field, fsz), "malloc field");
  cuda_check(cudaMalloc(&d_on, (size_t)2 * n * sizeof(float)), "malloc on");
  cuda_check(cudaMalloc(&d_phi, (size_t)n * sizeof(float)), "malloc phi");
  cuda_check(cudaMalloc(&d_nrm, (size_t)2 * n * sizeof(float)), "malloc nrm");
  cuda_check(cudaMemcpy(d_field, f.data, fsz, cudaMemcpyHostToDevice), "H2D field");
  cuda_check(cudaMemcpy(d_on, on, (size_t)2 * n * sizeof(float), cudaMemcpyHostToDevice), "H2D on");
  const int threads = 256, blocks = (n + threads - 1) / threads;
  sample_kernel<<<blocks, threads>>>(to_dev_field(f, d_field), d_on, n, d_phi, d_nrm);
  cuda_check(cudaGetLastError(), "sample_kernel launch");
  cuda_check(cudaMemcpy(phi_out, d_phi, (size_t)n * sizeof(float), cudaMemcpyDeviceToHost),
             "D2H phi");
  cuda_check(cudaMemcpy(normal_out, d_nrm, (size_t)2 * n * sizeof(float), cudaMemcpyDeviceToHost),
             "D2H nrm");
  cudaFree(d_field);
  cudaFree(d_on);
  cudaFree(d_phi);
  cudaFree(d_nrm);
}

void drive_step_cuda(const field_stack &f, float *o, float *th, float *sp, const float *carrot,
                     const coef_mlp &model, int n, const veh_params &vp, float *minclr_out) {
  if (n <= 0)
    return;
  const coef_mlp::flat_layers fl = model.export_flat();
  const size_t fsz = (size_t)f.H * f.W * 3 * sizeof(float);
  float *d_field, *d_o, *d_th, *d_sp, *d_car, *d_mc, *d_w, *d_ob;
  int *d_rows, *d_cols, *d_act;
  long *d_woff, *d_boff;
  cuda_check(cudaMalloc(&d_field, fsz), "malloc field");
  cuda_check(cudaMalloc(&d_o, (size_t)2 * n * sizeof(float)), "malloc o");
  cuda_check(cudaMalloc(&d_th, (size_t)n * sizeof(float)), "malloc th");
  cuda_check(cudaMalloc(&d_sp, (size_t)n * sizeof(float)), "malloc sp");
  cuda_check(cudaMalloc(&d_car, (size_t)2 * n * sizeof(float)), "malloc carrot");
  cuda_check(cudaMalloc(&d_mc, (size_t)n * sizeof(float)), "malloc minclr");
  cuda_check(cudaMalloc(&d_w, fl.data.size() * sizeof(float)), "malloc weights");
  cuda_check(cudaMalloc(&d_ob, fl.out_bias_off.size() * sizeof(float)), "malloc out_bias");
  cuda_check(cudaMalloc(&d_rows, fl.num_layers * sizeof(int)), "malloc rows");
  cuda_check(cudaMalloc(&d_cols, fl.num_layers * sizeof(int)), "malloc cols");
  cuda_check(cudaMalloc(&d_act, fl.num_layers * sizeof(int)), "malloc act");
  cuda_check(cudaMalloc(&d_woff, fl.num_layers * sizeof(long)), "malloc woff");
  cuda_check(cudaMalloc(&d_boff, fl.num_layers * sizeof(long)), "malloc boff");
  auto H2D = [&](void *dst, const void *src, size_t bytes, const char *w) {
    cuda_check(cudaMemcpy(dst, src, bytes, cudaMemcpyHostToDevice), w);
  };
  H2D(d_field, f.data, fsz, "H2D field");
  H2D(d_o, o, (size_t)2 * n * sizeof(float), "H2D o");
  H2D(d_th, th, (size_t)n * sizeof(float), "H2D th");
  H2D(d_sp, sp, (size_t)n * sizeof(float), "H2D sp");
  H2D(d_car, carrot, (size_t)2 * n * sizeof(float), "H2D carrot");
  H2D(d_w, fl.data.data(), fl.data.size() * sizeof(float), "H2D weights");
  H2D(d_ob, fl.out_bias_off.data(), fl.out_bias_off.size() * sizeof(float), "H2D out_bias");
  H2D(d_rows, fl.rows.data(), fl.num_layers * sizeof(int), "H2D rows");
  H2D(d_cols, fl.cols.data(), fl.num_layers * sizeof(int), "H2D cols");
  H2D(d_act, fl.act.data(), fl.num_layers * sizeof(int), "H2D act");
  H2D(d_woff, fl.w_off.data(), fl.num_layers * sizeof(long), "H2D woff");
  H2D(d_boff, fl.b_off.data(), fl.num_layers * sizeof(long), "H2D boff");

  dev_veh v;
  v.rr = vp.rr;
  v.d_hat = vp.d_hat;
  v.dt = vp.dt;
  v.vmax = vp.vmax;
  v.L = vp.L;
  v.delta_max = vp.delta_max;
  v.a_max = vp.a_max;
  v.a_lat_max = vp.a_lat_max;
  v.k_steer = vp.k_steer;
  v.nsub = vp.nsub;
  v.allow_reverse = vp.allow_reverse ? 1 : 0;

  const int threads = 128, blocks = (n + threads - 1) / threads;
  drive_kernel<<<blocks, threads>>>(to_dev_field(f, d_field), d_o, d_th, d_sp, d_car, d_w, d_rows,
                                    d_cols, d_act, d_woff, d_boff, fl.num_layers, d_ob, fl.in,
                                    fl.out, v, n, d_mc);
  cuda_check(cudaGetLastError(), "drive_kernel launch");
  auto D2H = [&](void *dst, const void *src, size_t bytes, const char *w) {
    cuda_check(cudaMemcpy(dst, src, bytes, cudaMemcpyDeviceToHost), w);
  };
  D2H(o, d_o, (size_t)2 * n * sizeof(float), "D2H o");
  D2H(th, d_th, (size_t)n * sizeof(float), "D2H th");
  D2H(sp, d_sp, (size_t)n * sizeof(float), "D2H sp");
  D2H(minclr_out, d_mc, (size_t)n * sizeof(float), "D2H minclr");
  cudaFree(d_field);
  cudaFree(d_o);
  cudaFree(d_th);
  cudaFree(d_sp);
  cudaFree(d_car);
  cudaFree(d_mc);
  cudaFree(d_w);
  cudaFree(d_ob);
  cudaFree(d_rows);
  cudaFree(d_cols);
  cudaFree(d_act);
  cudaFree(d_woff);
  cudaFree(d_boff);
}

} // namespace nav
} // namespace cvc
