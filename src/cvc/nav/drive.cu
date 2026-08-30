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

#include <algorithm>
#include <cmath>
#include <cuda_runtime.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/drive.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/sim_world_cuda.h>
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

// Device view of a single-plane grip raster. `data == nullptr` => mu == 1.
struct dev_grip {
  const float *data = nullptr;
  int M = 0, H = 0, W = 0;
  float S = 1.0f, cx = 0, cy = 0, mnx = 0, mny = 0, mxx = 0, mxy = 0;
};

struct dev_veh {
  float rr, d_hat, dt, vmax, L, delta_max, a_max, a_lat_max, k_steer;
  int nsub;
  int allow_reverse;
  // The optional refinements. These carry in-class initializers ON PURPOSE:
  // every construction site here is a bare `dev_veh v;` followed by field
  // assignments, so without them the pointers would be indeterminate and the
  // kernel would dereference garbage. Zero/null is the legacy path bit-for-bit.
  const float *body_offsets = nullptr; // device [n_body]
  int n_body = 0;
  float body_rr = 0.0f;
  float track_width = 0.0f;
  dev_grip grip;
};

// Bilinear sample (plane 0) + unit normal — mirrors drive.cpp sample_unit.
// `plane` selects the belief plane in a [M,3,H,W] block (0 for the shared /
// single-plane case); the phi/nx/ny slabs live at f.data + plane*3*H*W.
__device__ inline void d_sample_unit(const dev_field &f, int plane, float onx, float ony,
                                     float &phi, float &nxo, float &nyo) {
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
  const float *base = f.data + (long)plane * 3 * HW;
  const float *ph = base, *px = base + HW, *py = base + 2 * HW;
  const long a = (long)cy0 * f.W + cx0, b = (long)cy0 * f.W + cx1, c = (long)cy1 * f.W + cx0,
             d = (long)cy1 * f.W + cx1;
  phi = ph[a] * nw + ph[b] * ne + ph[c] * sw + ph[d] * se;
  float rnx = px[a] * nw + px[b] * ne + px[c] * sw + px[d] * se;
  float rny = py[a] * nw + py[b] * ne + py[c] * sw + py[d] * se;
  const float mag = sqrtf(rnx * rnx + rny * rny) + 1e-6f;
  nxo = rnx / mag;
  nyo = rny / mag;
}

// Bilinear sample of a single-plane grip raster — mirrors drive.cpp
// sample_grip, using the same float-stored bounds convention as dev_field.
__device__ inline float d_sample_grip(const dev_grip &g, int plane, float onx, float ony) {
  const float wx = onx / g.S + g.cx;
  const float wy = ony / g.S + g.cy;
  const float gx = 2.0f * (wx - g.mnx) / (g.mxx - g.mnx) - 1.0f;
  const float gy = 2.0f * (wy - g.mny) / (g.mxy - g.mny) - 1.0f;
  const float Wf1 = (float)(g.W - 1), Hf1 = (float)(g.H - 1);
  const float ix = fminf(fmaxf((gx + 1.0f) * 0.5f * Wf1, 0.0f), Wf1);
  const float iy = fminf(fmaxf((gy + 1.0f) * 0.5f * Hf1, 0.0f), Hf1);
  const int ix0 = (int)floorf(ix), iy0 = (int)floorf(iy);
  const float wx1 = ix - (float)ix0, wx0 = 1.0f - wx1;
  const float wy1 = iy - (float)iy0, wy0 = 1.0f - wy1;
  const int cx0 = min(max(ix0, 0), g.W - 1), cx1 = min(max(ix0 + 1, 0), g.W - 1);
  const int cy0 = min(max(iy0, 0), g.H - 1), cy1 = min(max(iy0 + 1, 0), g.H - 1);
  const long HW = (long)g.H * g.W;
  const float *pl = g.data + (long)plane * HW;
  return pl[(long)cy0 * g.W + cx0] * (wx0 * wy0) + pl[(long)cy0 * g.W + cx1] * (wx1 * wy0) +
         pl[(long)cy1 * g.W + cx0] * (wx0 * wy1) + pl[(long)cy1 * g.W + cx1] * (wx1 * wy1);
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
__device__ inline void d_bicycle(const dev_field &f, int plane, float &ox, float &oy, float &thi,
                                 float &spi, float gx, float gy, float al, float be, float ga,
                                 const dev_veh &v, float &minclr) {
  const float hdt = v.dt / (float)v.nsub;
  // track_width == 0 returns delta_max unchanged, so tan_dmax and every
  // threshold built from it stay bit-identical on the legacy path.
  const float dmax = v.track_width > 0.0f
                         ? atanf(v.L / (v.L / tanf(v.delta_max) + 0.5f * v.track_width))
                         : v.delta_max;
  const float tan_dmax = tanf(dmax);
  const float sp_min = v.allow_reverse ? -0.25f * v.vmax : 0.0f;
  const float v_creep_cap = 0.5f * sqrtf(v.a_lat_max * v.L / tan_dmax);
  const bool has_fp = v.n_body > 0 && v.body_offsets != nullptr;
  const bool has_grip = v.grip.data != nullptr;
  minclr = 9.9f;
  for (int s = 0; s < v.nsub; ++s) {
    // Hoisted above the sample: the footprint places its discs along the
    // heading. Value-identical to computing it after.
    const float ch = cosf(thi), sh = sinf(thi);
    float nx, ny, d, gov_rr;
    float Fbar_x, Fbar_y, Frep_x = 0.0f, Frep_y = 0.0f;
    if (!has_fp) {
      float phi;
      d_sample_unit(f, plane, ox, oy, phi, nx, ny);
      d = phi - v.rr;
      const float ipc = d_ipc(d, v.d_hat);
      Fbar_x = -(al * ipc) * nx;
      Fbar_y = -(al * ipc) * ny;
      const float ipc_rep = ipc < 0.0f ? ipc : 0.0f;
      Frep_x = -(al * ipc_rep) * nx;
      Frep_y = -(al * ipc_rep) * ny;
      gov_rr = v.rr;
    } else {
      // MIN clearance over discs (its normal drives the governor), SUM force.
      d = 3.0e38f;
      nx = 0.0f;
      ny = 0.0f;
      Fbar_x = 0.0f;
      Fbar_y = 0.0f;
      for (int b = 0; b < v.n_body; ++b) {
        const float off = v.body_offsets[b];
        float bphi, bnx, bny;
        d_sample_unit(f, plane, ox + off * ch, oy + off * sh, bphi, bnx, bny);
        const float bd = bphi - v.body_rr;
        const float bipc = d_ipc(bd, v.d_hat);
        Fbar_x += -(al * bipc) * bnx;
        Fbar_y += -(al * bipc) * bny;
        const float brep = bipc < 0.0f ? bipc : 0.0f;
        Frep_x += -(al * brep) * bnx;
        Frep_y += -(al * brep) * bny;
        if (bd < d) {
          d = bd;
          nx = bnx;
          ny = bny;
        }
      }
      gov_rr = v.body_rr;
    }
    minclr = fminf(minclr, d);
    // Both actuator limits are grip-limited, so both scale with mu together.
    float a_max_e = v.a_max, a_lat_e = v.a_lat_max, creep_cap = v_creep_cap;
    if (has_grip) {
      const float mu = d_sample_grip(v.grip, v.grip.M > 1 ? plane : 0, ox, oy);
      a_max_e = v.a_max * mu;
      a_lat_e = v.a_lat_max * mu;
      creep_cap = 0.5f * sqrtf(a_lat_e * (v.L / tan_dmax));
    }
    const float Fx = Fbar_x - be * (ox - gx), Fy = Fbar_y - be * (oy - gy);
    float a_long = (Fx * ch + Fy * sh) - ga * spi;
    a_long = fminf(fmaxf(a_long, -a_max_e), a_max_e);
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
    // F_rep computed with the sample above (summed per disc when footprinted).
    delta = delta + v.k_steer * tanhf(Frep_x * (-sh) + Frep_y * ch);
    delta = fminf(fmaxf(delta, -dmax), dmax);
    float kappa = fabsf(tanf(delta)) / v.L;
    const float kfloor = tan_dmax / (v.L * 400.0f);
    if (kappa < kfloor)
      kappa = kfloor;
    const float v_corner = sqrtf(a_lat_e / kappa);
    float ds = d - 0.5f * gov_rr;
    if (ds < 0.0f)
      ds = 0.0f;
    // +1e-24f mirrors the torch reference (which needs it so sqrt'(0) does not
    // NaN its backward pass); matching keeps the residual at zero. Legacy form
    // preserved exactly on the untouched path.
    const float v_stop =
        (has_fp || has_grip) ? sqrtf(2.0f * a_max_e * ds + 1e-24f) : sqrtf(2.0f * v.a_max * ds);
    const float motion_sign = spi >= 0.0f ? 1.0f : -1.0f;
    float approach = -(nx * ch + ny * sh) * motion_sign;
    approach = fminf(fmaxf(approach, 0.0f), 1.0f);
    const float v_stop_dir = approach > 0.05f ? v_stop / fmaxf(approach, 0.05f) : v.vmax;
    float v_lim = fminf(v.vmax, fminf(v_corner, v_stop_dir));
    const bool hard_steer = fabsf(delta) >= 0.7f * dmax;
    const bool can_move = d > 0.25f * gov_rr;
    const float v_floor = (hard_steer && can_move) ? 0.08f : 0.0f;
    v_lim = fmaxf(v_lim, v_floor);
    const float v_creep = fminf(0.5f * v_corner, creep_cap);
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
      const bool nose_blocked = behind && (head_on > 0.6f) && (d < 0.5f * gov_rr + 0.02f);
      if (nose_blocked) {
        a_long = (-0.10f - spi) / hdt;
        delta = -delta;
      }
    }
    a_long = fminf(fmaxf(a_long, -a_max_e), a_max_e);
    a_long = fminf(a_long, (v_lim - spi) / hdt);
    a_long = fminf(fmaxf(a_long, -a_max_e), a_max_e);
    spi = spi + hdt * a_long;
    spi = fminf(fmaxf(spi, sp_min), v.vmax);
    float sp2 = spi * spi;
    if (sp2 < 1e-9f)
      sp2 = 1e-9f;
    const float d_cap = atanf(a_lat_e * v.L / sp2);
    delta = fminf(fmaxf(delta, -d_cap), d_cap);
    thi = thi + hdt * (spi / v.L) * tanf(delta);
    ox = ox + hdt * spi * cosf(thi);
    oy = oy + hdt * spi * sinf(thi);
  }
}

__global__ void sample_kernel(dev_field f, const int *map_id, const float *on, int n,
                              float *phi_out, float *nrm_out) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n)
    return;
  const int plane = map_id ? map_id[i] : 0;
  float phi, nx, ny;
  d_sample_unit(f, plane, on[2 * i], on[2 * i + 1], phi, nx, ny);
  phi_out[i] = phi;
  nrm_out[2 * i] = nx;
  nrm_out[2 * i + 1] = ny;
}

__global__ void drive_kernel(dev_field f, const int *map_id, float *o, float *th, float *sp,
                             const float *carrot, const float *wdata, const int *rows,
                             const int *cols, const int *act, const long *w_off, const long *b_off,
                             int num_layers, const float *out_bias_off, int in, int out, dev_veh v,
                             int n, float *minclr) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n)
    return;
  const int plane = map_id ? map_id[i] : 0;
  float ox = o[2 * i], oy = o[2 * i + 1], thi = th[i], spi = sp[i];
  const float cx = carrot[2 * i], cy = carrot[2 * i + 1];
  // coef_feats at o toward the carrot.
  float phi, nx, ny;
  d_sample_unit(f, plane, ox, oy, phi, nx, ny);
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
  d_bicycle(f, plane, ox, oy, thi, spi, cx, cy, coef[0], coef[1], coef[2], v, mc);
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

// The scalar half of dev_veh, shared by the fused and unfused entry points.
inline void fill_dev_veh(const veh_params &vp, dev_veh &v) {
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
  v.track_width = vp.track_width;
  v.body_rr = vp.body_rr;
}

// Device buffers for the optional refinements; both stay null when unused,
// which is the legacy kernel path bit-for-bit. The caller frees them
// (cudaFree(nullptr) is a documented no-op, so it needs no guard).
template <class H2DFn>
void upload_refinements(const veh_params &vp, dev_veh &v, float *&d_body, float *&d_grip,
                        H2DFn &&H2D) {
  if (vp.n_body > 0 && vp.body_offsets) {
    cuda_check(cudaMalloc(&d_body, (size_t)vp.n_body * sizeof(float)), "malloc body_offsets");
    H2D(d_body, vp.body_offsets, (size_t)vp.n_body * sizeof(float), "H2D body_offsets");
    v.body_offsets = d_body;
    v.n_body = vp.n_body;
  }
  if (vp.grip && vp.grip->data) {
    const size_t gsz = (size_t)vp.grip->M * vp.grip->H * vp.grip->W * sizeof(float);
    cuda_check(cudaMalloc(&d_grip, gsz), "malloc grip");
    H2D(d_grip, vp.grip->data, gsz, "H2D grip");
    v.grip.data = d_grip;
    v.grip.M = vp.grip->M;
    v.grip.H = vp.grip->H;
    v.grip.W = vp.grip->W;
    v.grip.S = (float)vp.grip->S;
    v.grip.cx = (float)vp.grip->cx;
    v.grip.cy = (float)vp.grip->cy;
    v.grip.mnx = (float)vp.grip->mnx;
    v.grip.mny = (float)vp.grip->mny;
    v.grip.mxx = (float)vp.grip->mxx;
    v.grip.mxy = (float)vp.grip->mxy;
  }
}

// Bicycle rollout with GIVEN coefficients — the device twin of the CPU
// bicycle_rollout. drive_kernel fuses coef_feats + the MLP + this; keeping an
// unfused entry point lets the vehicle math be validated against torch without
// dragging a trained net through the comparison, which is what the CPU side has
// always had and the GPU side did not.
__global__ void bicycle_kernel(dev_field f, const int *map_id, float *o, float *th, float *sp,
                               const float *goal, const float *al, const float *be, const float *ga,
                               dev_veh v, int n, float *minclr) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n)
    return;
  const int plane = map_id ? map_id[i] : 0;
  float ox = o[2 * i], oy = o[2 * i + 1], thi = th[i], spi = sp[i];
  float mc = 9.9f;
  d_bicycle(f, plane, ox, oy, thi, spi, goal[2 * i], goal[2 * i + 1], al[i], be[i], ga[i], v, mc);
  o[2 * i] = ox;
  o[2 * i + 1] = oy;
  th[i] = thi;
  sp[i] = spi;
  minclr[i] = mc;
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
  sample_kernel<<<blocks, threads>>>(to_dev_field(f, d_field), nullptr, d_on, n, d_phi, d_nrm);
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

void bicycle_rollout_cuda(const field_stack &f, float *o, float *th, float *sp, const float *goal,
                          const float *al, const float *be, const float *ga, int n,
                          const veh_params &vp, float *minclr_out) {
  if (n <= 0)
    return;
  const size_t fsz = (size_t)f.H * f.W * 3 * sizeof(float); // plane 0
  float *d_field = nullptr, *d_o = nullptr, *d_th = nullptr, *d_sp = nullptr, *d_goal = nullptr;
  float *d_al = nullptr, *d_be = nullptr, *d_ga = nullptr, *d_mc = nullptr;
  float *d_body = nullptr, *d_grip = nullptr;
  const size_t fn = (size_t)n * sizeof(float), f2n = (size_t)2 * n * sizeof(float);
  cuda_check(cudaMalloc(&d_field, fsz), "malloc field");
  cuda_check(cudaMalloc(&d_o, f2n), "malloc o");
  cuda_check(cudaMalloc(&d_th, fn), "malloc th");
  cuda_check(cudaMalloc(&d_sp, fn), "malloc sp");
  cuda_check(cudaMalloc(&d_goal, f2n), "malloc goal");
  cuda_check(cudaMalloc(&d_al, fn), "malloc al");
  cuda_check(cudaMalloc(&d_be, fn), "malloc be");
  cuda_check(cudaMalloc(&d_ga, fn), "malloc ga");
  cuda_check(cudaMalloc(&d_mc, fn), "malloc minclr");
  auto H2D = [&](void *dst, const void *src, size_t bytes, const char *w) {
    cuda_check(cudaMemcpy(dst, src, bytes, cudaMemcpyHostToDevice), w);
  };
  H2D(d_field, f.data, fsz, "H2D field");
  H2D(d_o, o, f2n, "H2D o");
  H2D(d_th, th, fn, "H2D th");
  H2D(d_sp, sp, fn, "H2D sp");
  H2D(d_goal, goal, f2n, "H2D goal");
  H2D(d_al, al, fn, "H2D al");
  H2D(d_be, be, fn, "H2D be");
  H2D(d_ga, ga, fn, "H2D ga");

  dev_veh v;
  fill_dev_veh(vp, v);
  upload_refinements(vp, v, d_body, d_grip, H2D);

  const int threads = 128, blocks = (n + threads - 1) / threads;
  bicycle_kernel<<<blocks, threads>>>(to_dev_field(f, d_field), nullptr, d_o, d_th, d_sp, d_goal,
                                      d_al, d_be, d_ga, v, n, d_mc);
  cuda_check(cudaGetLastError(), "bicycle_kernel launch");
  auto D2H = [&](void *dst, const void *src, size_t bytes, const char *w) {
    cuda_check(cudaMemcpy(dst, src, bytes, cudaMemcpyDeviceToHost), w);
  };
  D2H(o, d_o, f2n, "D2H o");
  D2H(th, d_th, fn, "D2H th");
  D2H(sp, d_sp, fn, "D2H sp");
  D2H(minclr_out, d_mc, fn, "D2H minclr");
  cudaFree(d_field);
  cudaFree(d_o);
  cudaFree(d_th);
  cudaFree(d_sp);
  cudaFree(d_goal);
  cudaFree(d_al);
  cudaFree(d_be);
  cudaFree(d_ga);
  cudaFree(d_mc);
  cudaFree(d_body); // cudaFree(nullptr) is a documented no-op
  cudaFree(d_grip);
}

void drive_step_cuda(const field_stack &f, float *o, float *th, float *sp, const float *carrot,
                     const coef_mlp &model, int n, const veh_params &vp, float *minclr_out) {
  if (n <= 0)
    return;
  const coef_mlp::flat_layers fl = model.export_flat();
  // The device MLP (d_mlp) keeps activations in float a[64]/b[64] — reject a net
  // whose any layer is wider than that rather than overflowing device memory.
  const int kDeviceMaxWidth = 64;
  if (fl.in > kDeviceMaxWidth)
    throw std::runtime_error("cvc::nav::drive_step_cuda: input width > 64 unsupported on GPU");
  // drive_kernel builds the 5-feature vector inline in registers; a grip-widened
  // net wants a 6th column the kernel does not assemble. Refuse rather than feed
  // the first layer a short vector — the arithmetic would succeed and be wrong.
  if (fl.in != 5)
    throw std::runtime_error("cvc::nav::drive_step_cuda: the fused kernel builds 5 features "
                             "inline; a 6-feature (grip-widened) net needs the CPU drive_step");
  for (int L = 0; L < fl.num_layers; ++L)
    if (fl.rows[L] > kDeviceMaxWidth || fl.cols[L] > kDeviceMaxWidth)
      throw std::runtime_error("cvc::nav::drive_step_cuda: layer width > 64 unsupported on GPU");
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
  fill_dev_veh(vp, v);
  float *d_boff_body = nullptr, *d_grip = nullptr;
  upload_refinements(vp, v, d_boff_body, d_grip, H2D);

  const int threads = 128, blocks = (n + threads - 1) / threads;
  drive_kernel<<<blocks, threads>>>(to_dev_field(f, d_field), nullptr, d_o, d_th, d_sp, d_car, d_w,
                                    d_rows, d_cols, d_act, d_woff, d_boff, fl.num_layers, d_ob,
                                    fl.in, fl.out, v, n, d_mc);
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
  cudaFree(d_boff_body); // cudaFree(nullptr) is a documented no-op
  cudaFree(d_grip);
}

// ─────────────────────────────────────────────────────────────────────────────
// Device-resident sim_world_cuda (see sim_world_cuda.h). Field, .cvcnav weights
// and every SoA agent column (pose + full carrot-FSM state) stay on the GPU
// across ticks; step() launches sample -> carrot FSM -> fused drive ->
// reached/park with no host round-trip. Float-equivalent to CPU sim_world::step
// with freeze_sense=true (shared static map): reuses the same device math above
// and transcribes carrot_step / the reached-park loop one thread per agent.

namespace {

// carrot_step transcription (drive.cpp) — reads nrm (phi unused), advances the
// per-agent FSM columns in place, writes the steering carrot. Purely per-agent.
__global__ void carrot_kernel(const float *o, const float *goal, const float *th, float *sp,
                              const float *nrm, int *stall, int *mode, float *turn, float *dhit,
                              float *best, float *wall_entry, unsigned char *we_valid,
                              const unsigned char *tracking, float *pos_hist, int *hist_count,
                              const unsigned char *parked, const unsigned char *active,
                              float reach_tol, float a_max, float dt, int n, float *carrot_out) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n)
    return;
  const int SEEK = 0, WALL = 1;
  const float ox = o[2 * i], oy = o[2 * i + 1];
  const float gx = goal[2 * i], gy = goal[2 * i + 1];
  const float nx = nrm[2 * i], ny = nrm[2 * i + 1];
  const float dgx = gx - ox, dgy = gy - oy;
  const float dg = sqrtf(dgx * dgx + dgy * dgy);
  const float inv = 1.0f / (dg + 1e-6f);
  const float gdx = dgx * inv, gdy = dgy * inv;
  const float tangx = -ny, tangy = nx;
  const bool tracked = tracking[i] && active[i];

  // Branch 1 — stall accounting (non-tracking closing test vs tracking ring).
  const bool closing = dg < best[i] - 1e-3f;
  const int stall_nt = closing ? 0 : stall[i] + 1;
  const float best_nt = closing ? dg : best[i];
  const int slot = hist_count[i] % 40;
  if (tracked) {
    pos_hist[i * 80 + slot * 2] = ox;
    pos_hist[i * 80 + slot * 2 + 1] = oy;
  }
  const int oldest = (hist_count[i] + 1) % 40;
  const float phx = pos_hist[i * 80 + oldest * 2];
  const float phy = pos_hist[i * 80 + oldest * 2 + 1];
  const float moved = sqrtf((ox - phx) * (ox - phx) + (oy - phy) * (oy - phy));
  const bool have = hist_count[i] >= 40;
  const bool frozen = have && (moved < 0.15f) && (dg > reach_tol);
  const int stall_tk = have ? (frozen ? stall[i] + 1 : 0) : stall[i];
  const float best_tk = fminf(best[i], dg);
  if (tracked)
    hist_count[i] += 1;
  stall[i] = tracking[i] ? stall_tk : stall_nt;
  best[i] = tracking[i] ? best_tk : best_nt;

  // Branch 2 — seek -> wall entry.
  if (mode[i] == SEEK && stall[i] > 70) {
    dhit[i] = dg;
    wall_entry[2 * i] = ox;
    wall_entry[2 * i + 1] = oy;
    we_valid[i] = 1;
    turn[i] = (tangx * gdx + tangy * gdy) >= 0.0f ? 1.0f : -1.0f;
    mode[i] = WALL;
    stall[i] = 0;
  }

  // Branch 3 — carrot placement (uses the just-updated mode).
  float cx, cy;
  if (mode[i] == WALL) {
    const float twx = turn[i] * tangx, twy = turn[i] * tangy;
    cx = ox + (0.6f * twx + 0.4f * nx) * 1.6f;
    cy = oy + (0.6f * twy + 0.4f * ny) * 1.6f;
  } else {
    const float m = fminf(1.8f, dg);
    cx = ox + gdx * m;
    cy = oy + gdy * m;
  }

  // Branch 4 — wall exit (affects the next tick's mode).
  const float wex = ox - wall_entry[2 * i], wey = oy - wall_entry[2 * i + 1];
  const bool esc_tk = we_valid[i] && (sqrtf(wex * wex + wey * wey) > 2.0f);
  const bool exit_tk = esc_tk || (stall[i] > 240);
  const bool exit_nt = (dg < dhit[i] - 1.2f) || (stall[i] > 240);
  if (mode[i] == WALL && (tracking[i] ? exit_tk : exit_nt)) {
    mode[i] = SEEK;
    best[i] = dg;
    stall[i] = 0;
  }

  // Branch 5 — parked (brake; carrot straight ahead so steering error is 0).
  if (parked[i]) {
    float spv = sp[i] - a_max * dt;
    if (spv < 0.0f)
      spv = 0.0f;
    sp[i] = spv;
    const float ax = cosf(th[i]), ay = sinf(th[i]);
    const float m = fmaxf(1e-3f, spv * 2.0f);
    cx = ox + ax * m;
    cy = oy + ay * m;
  }

  carrot_out[2 * i] = cx;
  carrot_out[2 * i + 1] = cy;
}

// reached/park (single-goal) — mirrors sim_world::step's metrics loop.
__global__ void reached_park_kernel(const float *o, const float *goal, float reach_tol,
                                    unsigned char *reached, unsigned char *parked,
                                    const unsigned char *active, int n) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n)
    return;
  const float dx = goal[2 * i] - o[2 * i], dy = goal[2 * i + 1] - o[2 * i + 1];
  const float dg = sqrtf(dx * dx + dy * dy);
  const bool r = dg < reach_tol;
  reached[i] = r ? 1 : 0;
  if (r && !parked[i] && active[i])
    parked[i] = 1;
}

// Live retarget of one agent — mirrors sim_world::retarget.
__global__ void retarget_kernel(int i, float gx, float gy, float *goal, const float *o, float *best,
                                float *init, unsigned char *tracking, unsigned char *reached,
                                unsigned char *parked) {
  goal[2 * i] = gx;
  goal[2 * i + 1] = gy;
  const float dx = gx - o[2 * i], dy = gy - o[2 * i + 1];
  const float d = sqrtf(dx * dx + dy * dy);
  best[i] = fminf(best[i], d);
  init[i] = fmaxf(fmaxf(init[i], d), 1e-6f);
  tracking[i] = 1;
  reached[i] = 0;
  parked[i] = 0;
}

} // namespace

struct sim_world_cuda::impl {
  int n = 0, rows = 0, cols = 0, M = 1;
  field_stack fs; // world<->grid constants; .data unused (device is d_field)
  dev_veh v;      // vehicle/integration params
  float reach_tol = 0.8f, a_max = 1.5f, dt = 0.06f;
  double scale = 1.0, cx = 0.0, cy = 0.0; // pose -> world for snapshot
  int in = 0, out = 0, num_layers = 0;

  float *d_field = nullptr;              // [M*3*H*W]: M static belief planes
  int *d_map_id = nullptr;               // [n] agent -> plane in [0,M); null == shared (M==1)
  float *d_w = nullptr, *d_ob = nullptr; // policy weights + out bias
  int *d_rows = nullptr, *d_cols = nullptr, *d_act = nullptr; // layer shapes/acts
  long *d_woff = nullptr, *d_boff = nullptr;                  // layer offsets
  float *d_o = nullptr, *d_goal = nullptr, *d_th = nullptr, *d_sp = nullptr, *d_color = nullptr;
  int *d_stall = nullptr, *d_mode = nullptr, *d_hist_count = nullptr;
  float *d_turn = nullptr, *d_dhit = nullptr, *d_best = nullptr, *d_init = nullptr;
  float *d_wall_entry = nullptr, *d_pos_hist = nullptr;
  unsigned char *d_we_valid = nullptr, *d_tracking = nullptr, *d_parked = nullptr;
  unsigned char *d_reached = nullptr, *d_active = nullptr;
  float *d_phi = nullptr, *d_nrm = nullptr, *d_carrot = nullptr, *d_minclr = nullptr; // scratch

  ~impl() {
    for (void *p :
         {(void *)d_field,      (void *)d_map_id,     (void *)d_w,        (void *)d_ob,
          (void *)d_rows,       (void *)d_cols,       (void *)d_act,      (void *)d_woff,
          (void *)d_boff,       (void *)d_o,          (void *)d_goal,     (void *)d_th,
          (void *)d_sp,         (void *)d_color,      (void *)d_stall,    (void *)d_mode,
          (void *)d_hist_count, (void *)d_turn,       (void *)d_dhit,     (void *)d_best,
          (void *)d_init,       (void *)d_wall_entry, (void *)d_pos_hist, (void *)d_we_valid,
          (void *)d_tracking,   (void *)d_parked,     (void *)d_reached,  (void *)d_active,
          (void *)d_phi,        (void *)d_nrm,        (void *)d_carrot,   (void *)d_minclr})
      cudaFree(p);
  }
};

bool sim_world_cuda::available() {
  int c = 0;
  return cudaGetDeviceCount(&c) == cudaSuccess && c > 0;
}

sim_world_cuda::sim_world_cuda(const sim_world::config &cfg, const std::uint8_t *occ,
                               coef_mlp model, const float *o, const float *goal,
                               const float *color, int n, const int *map_id, int n_planes)
    : n_(n) {
  if (!available())
    throw std::runtime_error("cvc::nav::sim_world_cuda: no CUDA device");
  p_ = new impl();
  impl &s = *p_;
  s.n = n;
  s.rows = cfg.rows;
  s.cols = cfg.cols;
  const long hw = static_cast<long>(cfg.rows) * cfg.cols;
  // M static belief planes. When map_id is given, `occ` is [M*hw] (one known map
  // per plane) and agent i drives on plane map_id[i]; when null it is the shared
  // single-map path (M == 1). Planes are static (no on-device sensing) — the GPU
  // analog of CPU sim_world's grouped belief under freeze_sense, but capable of
  // genuinely different per-group maps.
  const int M = map_id ? std::max(1, n_planes) : 1;
  s.M = M;
  if (map_id)
    for (int i = 0; i < n; ++i)
      if (map_id[i] < 0 || map_id[i] >= M)
        throw std::runtime_error("cvc::nav::sim_world_cuda: map_id out of range [0, n_planes)");

  // 1. Build each plane's field on the host (one EDT per plane) and clip phi —
  //    identical to sim_world::rebuild_field over that plane's (static) map.
  const float clip = static_cast<float>(2.0 * cfg.max_x * cfg.scale);
  std::vector<float> field(static_cast<size_t>(M) * 3 * hw);
  for (int m = 0; m < M; ++m) {
    const sdf_field f = build_sdf(occ + static_cast<long>(m) * hw, cfg.rows, cfg.cols, cfg.min_x,
                                  cfg.min_y, cfg.max_x, cfg.max_y, cfg.scale);
    float *pl = field.data() + static_cast<size_t>(m) * 3 * hw;
    for (long i = 0; i < hw; ++i) {
      pl[i] = std::min(std::max(f.phi[i], -clip), clip);
      pl[hw + i] = f.normal_x[i];
      pl[2 * hw + i] = f.normal_y[i];
    }
  }

  s.fs.M = M;
  s.fs.H = cfg.rows;
  s.fs.W = cfg.cols;
  s.fs.mnx = cfg.min_x;
  s.fs.mny = cfg.min_y;
  s.fs.mxx = cfg.max_x;
  s.fs.mxy = cfg.max_y;
  s.fs.cx = cfg.cx;
  s.fs.cy = cfg.cy;
  s.fs.S = cfg.scale;
  s.scale = cfg.scale;
  s.cx = cfg.cx;
  s.cy = cfg.cy;
  s.reach_tol = cfg.reach_tol;
  s.a_max = cfg.veh.a_max;
  s.dt = cfg.veh.dt;
  // The optional vehicle refinements are NOT plumbed into the device-resident
  // world yet: they need per-world device buffers with this object's lifetime,
  // not the per-call ones drive_step_cuda allocates. Refuse rather than run a
  // GPU world that honours fewer constraints than the CPU/torch reference —
  // that is a silent divergence, and no parity gate would catch it because the
  // gates hand both paths the same config.
  if (cfg.veh.n_body > 0 || cfg.veh.track_width > 0.0f || cfg.veh.grip)
    throw std::runtime_error("cvc::nav::sim_world_cuda: body_offsets / track_width / grip are not "
                             "supported by the device-resident world yet; use the CPU sim_world "
                             "or drive_step_cuda");
  s.v.rr = cfg.veh.rr;
  s.v.d_hat = cfg.veh.d_hat;
  s.v.dt = cfg.veh.dt;
  s.v.vmax = cfg.veh.vmax;
  s.v.L = cfg.veh.L;
  s.v.delta_max = cfg.veh.delta_max;
  s.v.a_max = cfg.veh.a_max;
  s.v.a_lat_max = cfg.veh.a_lat_max;
  s.v.k_steer = cfg.veh.k_steer;
  s.v.nsub = cfg.veh.nsub;
  s.v.allow_reverse = cfg.veh.allow_reverse ? 1 : 0;

  // 2. Host-side agent init (th, best, init) — identical to sim_world's ctor.
  std::vector<float> th(n), best(n), init(n), turn(n, 1.0f);
  std::vector<unsigned char> active(n, 1);
  for (int i = 0; i < n; ++i) {
    const float dx = goal[2 * i] - o[2 * i], dy = goal[2 * i + 1] - o[2 * i + 1];
    th[i] = std::atan2(dy, dx);
    const float d = std::sqrt(dx * dx + dy * dy);
    best[i] = d;
    init[i] = std::max(d, 1e-6f);
  }

  // 3. Flat policy weights (folded out-bias) — same layout as drive_step_cuda.
  const coef_mlp::flat_layers fl = model.export_flat();
  s.in = fl.in;
  s.out = fl.out;
  s.num_layers = fl.num_layers;

  // 4. Allocate every resident buffer + upload. Zero-init the FSM columns that
  //    start at 0 (stall/mode/hist_count/dhit/wall_entry/pos_hist/we_valid/
  //    tracking/parked/reached); turn=1 and active=1 are uploaded.
  auto MALLOC = [&](void **d, size_t bytes, const char *w) { cuda_check(cudaMalloc(d, bytes), w); };
  auto H2D = [&](void *d, const void *h, size_t bytes, const char *w) {
    cuda_check(cudaMemcpy(d, h, bytes, cudaMemcpyHostToDevice), w);
  };
  auto ZERO = [&](void *d, size_t bytes, const char *w) { cuda_check(cudaMemset(d, 0, bytes), w); };
  const size_t fn = static_cast<size_t>(n);

  const size_t fbytes = static_cast<size_t>(M) * 3 * hw * sizeof(float);
  MALLOC((void **)&s.d_field, fbytes, "field");
  H2D(s.d_field, field.data(), fbytes, "H2D field");
  if (map_id) {
    MALLOC((void **)&s.d_map_id, fn * sizeof(int), "map_id");
    H2D(s.d_map_id, map_id, fn * sizeof(int), "H2D map_id");
  }
  MALLOC((void **)&s.d_w, fl.data.size() * sizeof(float), "w");
  MALLOC((void **)&s.d_ob, fl.out_bias_off.size() * sizeof(float), "ob");
  MALLOC((void **)&s.d_rows, fl.num_layers * sizeof(int), "rows");
  MALLOC((void **)&s.d_cols, fl.num_layers * sizeof(int), "cols");
  MALLOC((void **)&s.d_act, fl.num_layers * sizeof(int), "act");
  MALLOC((void **)&s.d_woff, fl.num_layers * sizeof(long), "woff");
  MALLOC((void **)&s.d_boff, fl.num_layers * sizeof(long), "boff");
  H2D(s.d_w, fl.data.data(), fl.data.size() * sizeof(float), "H2D w");
  H2D(s.d_ob, fl.out_bias_off.data(), fl.out_bias_off.size() * sizeof(float), "H2D ob");
  H2D(s.d_rows, fl.rows.data(), fl.num_layers * sizeof(int), "H2D rows");
  H2D(s.d_cols, fl.cols.data(), fl.num_layers * sizeof(int), "H2D cols");
  H2D(s.d_act, fl.act.data(), fl.num_layers * sizeof(int), "H2D act");
  H2D(s.d_woff, fl.w_off.data(), fl.num_layers * sizeof(long), "H2D woff");
  H2D(s.d_boff, fl.b_off.data(), fl.num_layers * sizeof(long), "H2D boff");

  MALLOC((void **)&s.d_o, 2 * fn * sizeof(float), "o");
  MALLOC((void **)&s.d_goal, 2 * fn * sizeof(float), "goal");
  MALLOC((void **)&s.d_th, fn * sizeof(float), "th");
  MALLOC((void **)&s.d_sp, fn * sizeof(float), "sp");
  MALLOC((void **)&s.d_color, 3 * fn * sizeof(float), "color");
  H2D(s.d_o, o, 2 * fn * sizeof(float), "H2D o");
  H2D(s.d_goal, goal, 2 * fn * sizeof(float), "H2D goal");
  H2D(s.d_th, th.data(), fn * sizeof(float), "H2D th");
  ZERO(s.d_sp, fn * sizeof(float), "sp=0");
  H2D(s.d_color, color, 3 * fn * sizeof(float), "H2D color");

  MALLOC((void **)&s.d_stall, fn * sizeof(int), "stall");
  MALLOC((void **)&s.d_mode, fn * sizeof(int), "mode");
  MALLOC((void **)&s.d_hist_count, fn * sizeof(int), "hist_count");
  MALLOC((void **)&s.d_turn, fn * sizeof(float), "turn");
  MALLOC((void **)&s.d_dhit, fn * sizeof(float), "dhit");
  MALLOC((void **)&s.d_best, fn * sizeof(float), "best");
  MALLOC((void **)&s.d_init, fn * sizeof(float), "init");
  MALLOC((void **)&s.d_wall_entry, 2 * fn * sizeof(float), "wall_entry");
  MALLOC((void **)&s.d_pos_hist, 80 * fn * sizeof(float), "pos_hist");
  MALLOC((void **)&s.d_we_valid, fn, "we_valid");
  MALLOC((void **)&s.d_tracking, fn, "tracking");
  MALLOC((void **)&s.d_parked, fn, "parked");
  MALLOC((void **)&s.d_reached, fn, "reached");
  MALLOC((void **)&s.d_active, fn, "active");
  ZERO(s.d_stall, fn * sizeof(int), "stall=0");
  ZERO(s.d_mode, fn * sizeof(int), "mode=0");
  ZERO(s.d_hist_count, fn * sizeof(int), "hist_count=0");
  H2D(s.d_turn, turn.data(), fn * sizeof(float), "H2D turn");
  ZERO(s.d_dhit, fn * sizeof(float), "dhit=0");
  H2D(s.d_best, best.data(), fn * sizeof(float), "H2D best");
  H2D(s.d_init, init.data(), fn * sizeof(float), "H2D init");
  ZERO(s.d_wall_entry, 2 * fn * sizeof(float), "wall_entry=0");
  ZERO(s.d_pos_hist, 80 * fn * sizeof(float), "pos_hist=0");
  ZERO(s.d_we_valid, fn, "we_valid=0");
  ZERO(s.d_tracking, fn, "tracking=0");
  ZERO(s.d_parked, fn, "parked=0");
  ZERO(s.d_reached, fn, "reached=0");
  H2D(s.d_active, active.data(), fn, "H2D active");

  MALLOC((void **)&s.d_phi, fn * sizeof(float), "phi");
  MALLOC((void **)&s.d_nrm, 2 * fn * sizeof(float), "nrm");
  MALLOC((void **)&s.d_carrot, 2 * fn * sizeof(float), "carrot");
  MALLOC((void **)&s.d_minclr, fn * sizeof(float), "minclr");
}

sim_world_cuda sim_world_cuda::from_occupancy(const sim_world::config &cfg, const std::uint8_t *occ,
                                              coef_mlp model, int n, unsigned seed) {
  std::vector<float> o(2 * n), goal(2 * n), color(3 * n);
  sim_world::scatter_free(cfg, occ, n, seed, o.data(), goal.data(), color.data());
  return sim_world_cuda(cfg, occ, std::move(model), o.data(), goal.data(), color.data(), n);
}

sim_world_cuda::~sim_world_cuda() { delete p_; }

sim_world_cuda::sim_world_cuda(sim_world_cuda &&o) noexcept : p_(o.p_), n_(o.n_), gstep_(o.gstep_) {
  o.p_ = nullptr;
}

sim_world_cuda &sim_world_cuda::operator=(sim_world_cuda &&o) noexcept {
  if (this != &o) {
    delete p_;
    p_ = o.p_;
    n_ = o.n_;
    gstep_ = o.gstep_;
    o.p_ = nullptr;
  }
  return *this;
}

void sim_world_cuda::step() {
  impl &s = *p_;
  const int T = 128, B = (s.n + T - 1) / T;
  const dev_field df = to_dev_field(s.fs, s.d_field);
  // sample (nrm for the carrot) -> carrot FSM -> fused drive -> reached/park.
  // All on the default stream: kernel k+1 sees kernel k's writes (serialized).
  sample_kernel<<<B, T>>>(df, s.d_map_id, s.d_o, s.n, s.d_phi, s.d_nrm);
  carrot_kernel<<<B, T>>>(s.d_o, s.d_goal, s.d_th, s.d_sp, s.d_nrm, s.d_stall, s.d_mode, s.d_turn,
                          s.d_dhit, s.d_best, s.d_wall_entry, s.d_we_valid, s.d_tracking,
                          s.d_pos_hist, s.d_hist_count, s.d_parked, s.d_active, s.reach_tol,
                          s.a_max, s.dt, s.n, s.d_carrot);
  drive_kernel<<<B, T>>>(df, s.d_map_id, s.d_o, s.d_th, s.d_sp, s.d_carrot, s.d_w, s.d_rows,
                         s.d_cols, s.d_act, s.d_woff, s.d_boff, s.num_layers, s.d_ob, s.in, s.out,
                         s.v, s.n, s.d_minclr);
  reached_park_kernel<<<B, T>>>(s.d_o, s.d_goal, s.reach_tol, s.d_reached, s.d_parked, s.d_active,
                                s.n);
  cuda_check(cudaGetLastError(), "sim_world_cuda step launch");
  ++gstep_;
}

int sim_world_cuda::planes() const { return p_ ? p_->M : 1; }

void sim_world_cuda::snapshot(float *pos_world, float *heading, float *speed, int *mode,
                              std::uint8_t *reached) const {
  const impl &s = *p_;
  const size_t fn = static_cast<size_t>(s.n);
  auto D2H = [&](void *h, const void *d, size_t bytes, const char *w) {
    cuda_check(cudaMemcpy(h, d, bytes, cudaMemcpyDeviceToHost), w);
  };
  if (pos_world) {
    std::vector<float> o(2 * fn);
    D2H(o.data(), s.d_o, 2 * fn * sizeof(float), "D2H o");
    for (int i = 0; i < s.n; ++i) {
      pos_world[2 * i] = o[2 * i] / static_cast<float>(s.scale) + static_cast<float>(s.cx);
      pos_world[2 * i + 1] = o[2 * i + 1] / static_cast<float>(s.scale) + static_cast<float>(s.cy);
    }
  }
  if (heading)
    D2H(heading, s.d_th, fn * sizeof(float), "D2H th");
  if (speed) {
    D2H(speed, s.d_sp, fn * sizeof(float), "D2H sp");
    for (int i = 0; i < s.n; ++i)
      speed[i] /= static_cast<float>(s.scale);
  }
  if (mode)
    D2H(mode, s.d_mode, fn * sizeof(int), "D2H mode");
  if (reached)
    D2H(reached, s.d_reached, fn, "D2H reached");
}

void sim_world_cuda::retarget(int i, float gx_n, float gy_n) {
  if (i < 0 || i >= n_)
    return;
  impl &s = *p_;
  retarget_kernel<<<1, 1>>>(i, gx_n, gy_n, s.d_goal, s.d_o, s.d_best, s.d_init, s.d_tracking,
                            s.d_reached, s.d_parked);
  cuda_check(cudaGetLastError(), "sim_world_cuda retarget launch");
}

} // namespace nav
} // namespace cvc
