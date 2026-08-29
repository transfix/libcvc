/*
  Copyright 2007-2011 The University of Texas at Austin
        Authors: Joe Rivera <transfix@ices.utexas.edu>
  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// coef_energy_net.cu — the CUDA twin of coef_energy_net.cpp's forward: the
// learned material-coefficient transformer + CNN, one CUDA BLOCK per agent
// (blockDim = d_tok = 64, every per-dim loop strided by 64). The tokens and the
// transformer activations live in dynamic shared memory; the CNN activations are
// too large for shared, so each block uses its own slice of device scratch. ctx
// stays resident so the lambda heads need no round-trip. Float-equivalent to the
// CPU forward (rtol 1e-4, the FLOAT tier — attention/GEMM reduction order is
// free); validated by nav_coef_energy_cuda_test. Built precise (-fmad=false
// --prec-div/sqrt --ftz=false, set in CMake) like the other nav .cu.

#include <algorithm>
#include <cmath>
#include <cuda_runtime.h>
#include <cvc/nav/coef_energy_net.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

namespace {

constexpr int DT = 64;      // d_tok
constexpr int NH = 4;       // nhead
constexpr int HD = DT / NH; // 16
constexpr int NL = 2;       // num_layers
constexpr int FF = 128;     // FFN hidden / obs_enc.0 width

void cuda_check(cudaError_t e, const char *what) {
  if (e != cudaSuccess)
    throw std::runtime_error(std::string("cvc::nav coef_energy CUDA: ") + what + ": " +
                             cudaGetErrorString(e));
}

// Device weight pointers (all borrowed, uploaded once by the host wrapper).
struct DW {
  const float *ge0w, *ge0b, *ge2w, *ge2b;                     // goal_enc 4->64->64
  const float *oe0w, *oe0b, *oe2w, *oe2b;                     // obs_enc  6->128->64
  const float *inw[NL], *inb[NL];                             // self_attn.in_proj  [192,64]/[192]
  const float *outw[NL], *outb[NL];                           // self_attn.out_proj [64,64]/[64]
  const float *l1w[NL], *l1b[NL];                             // linear1 [128,64]/[128]
  const float *l2w[NL], *l2b[NL];                             // linear2 [64,128]/[64]
  const float *n1w[NL], *n1b[NL], *n2w[NL], *n2b[NL];         // norms [64]
  const float *a0w, *a0b, *a2w, *a2b;                         // alpha_head 64->64->1
  const float *b0w, *b0b, *b2w, *b2b;                         // beta_head
  const float *g0w, *g0b, *g2w, *g2b;                         // gamma_head
  const float *r0w, *r0b, *r2w, *r2b, *r4w, *r4b, *r8w, *r8b; // risk CNN
  const float *ls0w, *ls0b, *ls2w, *ls2b;                     // lam_soft_head 128->64->1
  const float *lh0w, *lh0b, *lh2w, *lh2b;                     // lam_hard_head
  const float *ml0w, *ml0b, *ml2w, *ml2b;                     // mu_lat_head
};

__device__ inline float drelu(float x) { return x > 0.0f ? x : 0.0f; }
__device__ inline float dsig(float x) { return 1.0f / (1.0f + expf(-x)); }
__device__ inline float dsoftplus(float x) {
  return x > 20.0f ? x : log1pf(expf(x < 20.0f ? x : 20.0f));
}

// Sum v across the 64 threads of the block; result broadcast to all. `sh` is a
// 64-float shared scratch. Assumes blockDim.x == 64.
__device__ inline float bsum64(float v, float *sh) {
  const int t = threadIdx.x;
  sh[t] = v;
  __syncthreads();
  for (int s = 32; s > 0; s >>= 1) {
    if (t < s)
      sh[t] += sh[t + s];
    __syncthreads();
  }
  const float r = sh[0];
  __syncthreads();
  return r;
}

// y[rows,out] = x[rows,in] @ W[out,in]^T + b, cooperatively (threads stride the
// out dim by 64). x,y in shared; W,b in global. Caller syncs around it.
__device__ inline void lin(const float *x, int rows, int in, const float *W, const float *b,
                           int out, float *y) {
  for (int r = 0; r < rows; ++r) {
    const float *xr = x + r * in;
    float *yr = y + r * out;
    for (int o = threadIdx.x; o < out; o += DT) {
      float acc = b[o];
      const float *w = W + (long)o * in;
      for (int i = 0; i < in; ++i)
        acc += xr[i] * w[i];
      yr[o] = acc;
    }
  }
}

__device__ inline void relu_inplace(float *x, int n) {
  for (int i = threadIdx.x; i < n; i += DT)
    x[i] = drelu(x[i]);
}

// Post-norm LayerNorm over the last dim (DT), per row: y = ((x-mu)/sqrt(var+eps))*g + b.
__device__ inline void layernorm_rows(const float *x, int rows, const float *g, const float *b,
                                      float eps, float *y, float *red) {
  const int t = threadIdx.x; // channel 0..63
  for (int r = 0; r < rows; ++r) {
    const float xv = x[r * DT + t];
    const float mu = bsum64(xv, red) / (float)DT;
    const float dv = xv - mu;
    const float var = bsum64(dv * dv, red) / (float)DT;
    const float inv = 1.0f / sqrtf(var + eps);
    y[r * DT + t] = dv * inv * g[t] + b[t];
    __syncthreads();
  }
}

// Cross-correlation conv2d (k=3, pad=1) + ReLU, cooperatively over Cout*Ho*Wo
// outputs (threads stride by 64). x/out in device global; weights global. Matches
// the CPU conv2d then relu. Caller syncs after.
__device__ inline void conv_relu_s(const float *x, int Cin, int H, int W, const float *wt,
                                   const float *b, int Cout, int stride, float *out, int Ho,
                                   int Wo) {
  for (int idx = threadIdx.x; idx < Cout * Ho * Wo; idx += DT) {
    const int oc = idx / (Ho * Wo);
    const int rem = idx - oc * (Ho * Wo);
    const int oy = rem / Wo, ox = rem % Wo;
    float acc = b[oc];
    for (int ic = 0; ic < Cin; ++ic) {
      const float *xc = x + (long)ic * H * W;
      const float *wc = wt + ((long)oc * Cin + ic) * 9;
      for (int ky = 0; ky < 3; ++ky) {
        const int iy = oy * stride + ky - 1;
        if (iy < 0 || iy >= H)
          continue;
        for (int kx = 0; kx < 3; ++kx) {
          const int ix = ox * stride + kx - 1;
          if (ix < 0 || ix >= W)
            continue;
          acc += xc[iy * W + ix] * wc[ky * 3 + kx];
        }
      }
    }
    out[idx] = drelu(acc);
  }
}

// One 2-layer scalar head: Linear(zin->64) ReLU Linear(64->1). Cooperative;
// returns the raw pre-activation on thread 0 (undefined on others). `ff` is a
// [64] shared scratch. Caller applies softplus / sigmoid.
__device__ inline float scalar_head(const float *h0w, const float *h0b, const float *h2w,
                                    const float *h2b, const float *z, int zin, float *ff) {
  lin(z, 1, zin, h0w, h0b, DT, ff);
  __syncthreads();
  ff[threadIdx.x] = drelu(ff[threadIdx.x]);
  __syncthreads();
  float out = 0.0f;
  if (threadIdx.x == 0) {
    out = h2b[0];
    for (int i = 0; i < DT; ++i)
      out += ff[i] * h2w[i];
  }
  __syncthreads();
  return out;
}

// One agent per block. See file header. smem layout (max_T = 1 + max_obs):
//   tok[max_T*64] qkv[max_T*192] cbuf[max_T*64] ff[max_T*128] red[64] pooled[1024]
//   risk_ctx[64] mat[128]
__global__ void ce_fwd_k(const float *obs_feats, const unsigned char *obs_mask,
                         const int *obs_offsets, const float *goal_feats, const float *risk_patch,
                         int P, int max_T, float eps, float ls_max, float lh_max, float ml_max,
                         DW w, float *scratch, long scratch_stride, float *alphas_out, float *beta,
                         float *gamma, float *lam_soft, float *lam_hard, float *mu_lat) {
  const int agent = blockIdx.x;
  const int t = threadIdx.x;
  const int o0 = obs_offsets[agent];
  const int n_obs = obs_offsets[agent + 1] - o0;
  const int T = 1 + n_obs;

  extern __shared__ float smem[];
  float *tok = smem;                     // [max_T*64]
  float *qkv = tok + max_T * DT;         // [max_T*192]
  float *cbuf = qkv + max_T * 3 * DT;    // [max_T*64]
  float *ff = cbuf + max_T * DT;         // [max_T*128]
  float *red = ff + max_T * FF;          // [64]
  float *pooled = red + DT;              // [1024]
  float *risk_ctx = pooled + 64 * 4 * 4; // [64]
  float *mat = risk_ctx + DT;            // [128]

  // ── tokens: goal_enc (row 0) then obs_enc (rows 1..n_obs) ──────────────────
  // goal token: Linear(4->64) relu Linear(64->64). Use ff[0:64] as the hidden.
  const float *gf = goal_feats + (long)agent * 4;
  for (int o = t; o < DT; o += DT)
    ff[o] = w.ge0b[o] + gf[0] * w.ge0w[o * 4 + 0] + gf[1] * w.ge0w[o * 4 + 1] +
            gf[2] * w.ge0w[o * 4 + 2] + gf[3] * w.ge0w[o * 4 + 3];
  __syncthreads();
  ff[t] = drelu(ff[t]);
  __syncthreads();
  lin(ff, 1, DT, w.ge2w, w.ge2b, DT, tok); // -> tok row 0
  __syncthreads();
  // obs tokens: Linear(6->128) relu Linear(128->64) per obstacle.
  for (int j = 0; j < n_obs; ++j) {
    const float *of = obs_feats + (long)(o0 + j) * 6;
    for (int o = t; o < FF; o += DT) {
      float acc = w.oe0b[o];
      const float *ww = w.oe0w + (long)o * 6;
      for (int i = 0; i < 6; ++i)
        acc += of[i] * ww[i];
      ff[o] = drelu(acc);
    }
    __syncthreads();
    lin(ff, 1, FF, w.oe2w, w.oe2b, DT, tok + (long)(1 + j) * DT);
    __syncthreads();
  }

  // ── transformer encoder: 2x POST-norm layer ────────────────────────────────
  for (int L = 0; L < NL; ++L) {
    // in_proj -> qkv (T, 192)
    lin(tok, T, DT, w.inw[L], w.inb[L], 3 * DT, qkv);
    __syncthreads();
    // attention -> cbuf (T,64): each (query i, head h) pair handled by one thread.
    const float scale = 1.0f / sqrtf((float)HD);
    for (int p = t; p < T * NH; p += DT) {
      const int i = p / NH, h = p % NH;
      const float *qi = qkv + (long)i * 3 * DT + h * HD;
      float sc[64];
      float m = -INFINITY;
      for (int j = 0; j < T; ++j) {
        if (j > 0 && obs_mask[o0 + j - 1] == 0) {
          sc[j] = -INFINITY;
          continue;
        }
        const float *kj = qkv + (long)j * 3 * DT + DT + h * HD;
        float s = 0.0f;
        for (int c = 0; c < HD; ++c)
          s += qi[c] * kj[c];
        s *= scale;
        sc[j] = s;
        if (s > m)
          m = s;
      }
      float denom = 0.0f;
      for (int j = 0; j < T; ++j) {
        if (sc[j] == -INFINITY) {
          sc[j] = 0.0f;
          continue;
        }
        const float e = expf(sc[j] - m);
        sc[j] = e;
        denom += e;
      }
      const float invd = 1.0f / denom;
      float *ci = cbuf + (long)i * DT + h * HD;
      for (int c = 0; c < HD; ++c)
        ci[c] = 0.0f;
      for (int j = 0; j < T; ++j) {
        if (sc[j] == 0.0f)
          continue;
        const float pj = sc[j] * invd;
        const float *vj = qkv + (long)j * 3 * DT + 2 * DT + h * HD;
        for (int c = 0; c < HD; ++c)
          ci[c] += pj * vj[c];
      }
    }
    __syncthreads();
    // out_proj(cbuf) -> ff[0:T*64]; residual + norm1 -> tok
    lin(cbuf, T, DT, w.outw[L], w.outb[L], DT, ff);
    __syncthreads();
    for (int idx = t; idx < T * DT; idx += DT)
      ff[idx] += tok[idx];
    __syncthreads();
    layernorm_rows(ff, T, w.n1w[L], w.n1b[L], eps, tok, red);
    __syncthreads();
    // FFN: linear1 -> relu -> linear2; residual + norm2 -> tok. qkv reused as tmp.
    lin(tok, T, DT, w.l1w[L], w.l1b[L], FF, ff);
    __syncthreads();
    relu_inplace(ff, T * FF);
    __syncthreads();
    lin(ff, T, FF, w.l2w[L], w.l2b[L], DT, qkv); // qkv[0:T*64] = ff2
    __syncthreads();
    for (int idx = t; idx < T * DT; idx += DT)
      qkv[idx] += tok[idx];
    __syncthreads();
    layernorm_rows(qkv, T, w.n2w[L], w.n2b[L], eps, tok, red);
    __syncthreads();
  }

  // ctx = tok row 0 (goal context). Keep resident.
  float *ctx = tok; // [64]

  // ── alpha per obstacle: Linear(64->64) relu Linear(64->1) softplus, masked ──
  for (int j = 0; j < n_obs; ++j) {
    lin(tok + (long)(1 + j) * DT, 1, DT, w.a0w, w.a0b, DT, ff); // ff[0:64] = a1
    __syncthreads();
    ff[t] = drelu(ff[t]);
    __syncthreads();
    if (t == 0) {
      float acc = w.a2b[0];
      for (int i = 0; i < DT; ++i)
        acc += ff[i] * w.a2w[i];
      alphas_out[o0 + j] = obs_mask[o0 + j] ? dsoftplus(acc) : 0.0f;
    }
    __syncthreads();
  }

  // ── beta / gamma: head(ctx) softplus ───────────────────────────────────────
  {
    float bv = scalar_head(w.b0w, w.b0b, w.b2w, w.b2b, ctx, DT, ff);
    if (t == 0)
      beta[agent] = dsoftplus(bv);
    float gv = scalar_head(w.g0w, w.g0b, w.g2w, w.g2b, ctx, DT, ff);
    if (t == 0)
      gamma[agent] = dsoftplus(gv);
  }

  // ── risk CNN: conv(2->16,s1) conv(16->32,s2) conv(32->64,s2) pool linear ────
  float *bufA = scratch + (long)agent * scratch_stride;
  float *bufB = bufA + scratch_stride / 2;
  // conv0 (2->16, s1) -> bufA (16,P,P); conv1 (16->32, s2) -> bufB; conv2
  // (32->64, s2) -> bufA. k=3 pad=1, ReLU fused. Ho=(H+2*pad-k)/stride+1.
  conv_relu_s(risk_patch + (long)agent * 2 * P * P, 2, P, P, w.r0w, w.r0b, 16, 1, bufA, P, P);
  __syncthreads();
  const int H2 = (P + 2 - 3) / 2 + 1, W2 = (P + 2 - 3) / 2 + 1; // stride 2
  conv_relu_s(bufA, 16, P, P, w.r2w, w.r2b, 32, 2, bufB, H2, W2);
  __syncthreads();
  const int H3 = (H2 + 2 - 3) / 2 + 1, W3 = (W2 + 2 - 3) / 2 + 1;
  conv_relu_s(bufB, 32, H2, W2, w.r4w, w.r4b, 64, 2, bufA, H3, W3);
  __syncthreads();
  // adaptive_avg_pool(64,H3,W3)->(64,4,4) into pooled[1024]
  for (int idx = t; idx < 64 * 16; idx += DT) {
    const int c = idx / 16, bin = idx % 16, bi = bin / 4, bj = bin % 4;
    const int r0 = (bi * H3) / 4, r1 = ((bi + 1) * H3 + 3) / 4;
    const int c0p = (bj * W3) / 4, c1p = ((bj + 1) * W3 + 3) / 4;
    float acc = 0.0f;
    int cnt = 0;
    const float *xc = bufA + (long)c * H3 * W3;
    for (int r = r0; r < r1; ++r)
      for (int cc = c0p; cc < c1p; ++cc) {
        acc += xc[r * W3 + cc];
        ++cnt;
      }
    pooled[(c * 4 + bi) * 4 + bj] = acc / (float)cnt;
  }
  __syncthreads();
  lin(pooled, 1, 1024, w.r8w, w.r8b, DT, risk_ctx);
  __syncthreads();
  risk_ctx[t] = drelu(risk_ctx[t]);
  __syncthreads();

  // ── lambda heads on [risk_ctx, ctx] (128) ──────────────────────────────────
  mat[t] = risk_ctx[t];
  mat[DT + t] = ctx[t];
  __syncthreads();
  {
    float sv = scalar_head(w.ls0w, w.ls0b, w.ls2w, w.ls2b, mat, 2 * DT, ff);
    if (t == 0)
      lam_soft[agent] = ls_max * dsig(sv);
    float hv = scalar_head(w.lh0w, w.lh0b, w.lh2w, w.lh2b, mat, 2 * DT, ff);
    if (t == 0)
      lam_hard[agent] = lh_max * dsig(hv);
    float mv = scalar_head(w.ml0w, w.ml0b, w.ml2w, w.ml2b, mat, 2 * DT, ff);
    if (t == 0)
      mu_lat[agent] = ml_max * dsig(mv);
  }
}

} // namespace

bool coef_energy_cuda_available() {
  int n = 0;
  return cudaGetDeviceCount(&n) == cudaSuccess && n > 0;
}

void coef_energy_net::forward_batch_cuda(const float *obs_feats, const std::uint8_t *obs_mask,
                                         const int *obs_offsets, int n, const float *goal_feats,
                                         const float *risk_patch, int patch_p, float *alphas_out,
                                         float *beta, float *gamma, float *lam_soft,
                                         float *lam_hard, float *mu_lat) const {
  if (!coef_energy_cuda_available())
    throw std::runtime_error("coef_energy_net::forward_batch_cuda: no CUDA device");
  if (patch_p != patch_size_)
    throw std::runtime_error("coef_energy_net::forward_batch_cuda: patch_p != patch_size");
  if (n <= 0)
    return;
  const int P = patch_p;
  const int total = obs_offsets[n];
  int max_obs = 0;
  for (int i = 0; i < n; ++i)
    max_obs = std::max(max_obs, obs_offsets[i + 1] - obs_offsets[i]);
  const int max_T = 1 + max_obs;

  std::vector<void *> allocs;
  auto dmalloc = [&](std::size_t bytes) -> void * {
    void *p = nullptr;
    cuda_check(cudaMalloc(&p, bytes ? bytes : 4), "cudaMalloc");
    allocs.push_back(p);
    return p;
  };
  auto up = [&](const void *src, std::size_t bytes) -> void * {
    void *p = dmalloc(bytes);
    if (bytes)
      cuda_check(cudaMemcpy(p, src, bytes, cudaMemcpyHostToDevice), "H2D");
    return p;
  };
  auto up_w = [&](const std::string &name) -> const float * {
    const auto &tt = t(name);
    return static_cast<const float *>(up(tt.data.data(), tt.data.size() * sizeof(float)));
  };

  const float *d_obs =
      static_cast<const float *>(up(obs_feats, (std::size_t)total * 6 * sizeof(float)));
  const unsigned char *d_mask =
      static_cast<const unsigned char *>(up(obs_mask, (std::size_t)total * sizeof(unsigned char)));
  const int *d_off = static_cast<const int *>(up(obs_offsets, (std::size_t)(n + 1) * sizeof(int)));
  const float *d_goal =
      static_cast<const float *>(up(goal_feats, (std::size_t)n * 4 * sizeof(float)));
  const float *d_risk =
      static_cast<const float *>(up(risk_patch, (std::size_t)n * 2 * P * P * sizeof(float)));

  DW w{};
  w.ge0w = up_w("goal_enc.0.weight");
  w.ge0b = up_w("goal_enc.0.bias");
  w.ge2w = up_w("goal_enc.2.weight");
  w.ge2b = up_w("goal_enc.2.bias");
  w.oe0w = up_w("obs_enc.0.weight");
  w.oe0b = up_w("obs_enc.0.bias");
  w.oe2w = up_w("obs_enc.2.weight");
  w.oe2b = up_w("obs_enc.2.bias");
  for (int L = 0; L < NL; ++L) {
    const std::string pre = "fuser.layers." + std::to_string(L) + ".";
    w.inw[L] = up_w(pre + "self_attn.in_proj_weight");
    w.inb[L] = up_w(pre + "self_attn.in_proj_bias");
    w.outw[L] = up_w(pre + "self_attn.out_proj.weight");
    w.outb[L] = up_w(pre + "self_attn.out_proj.bias");
    w.l1w[L] = up_w(pre + "linear1.weight");
    w.l1b[L] = up_w(pre + "linear1.bias");
    w.l2w[L] = up_w(pre + "linear2.weight");
    w.l2b[L] = up_w(pre + "linear2.bias");
    w.n1w[L] = up_w(pre + "norm1.weight");
    w.n1b[L] = up_w(pre + "norm1.bias");
    w.n2w[L] = up_w(pre + "norm2.weight");
    w.n2b[L] = up_w(pre + "norm2.bias");
  }
  w.a0w = up_w("alpha_head.0.weight");
  w.a0b = up_w("alpha_head.0.bias");
  w.a2w = up_w("alpha_head.2.weight");
  w.a2b = up_w("alpha_head.2.bias");
  w.b0w = up_w("beta_head.0.weight");
  w.b0b = up_w("beta_head.0.bias");
  w.b2w = up_w("beta_head.2.weight");
  w.b2b = up_w("beta_head.2.bias");
  w.g0w = up_w("gamma_head.0.weight");
  w.g0b = up_w("gamma_head.0.bias");
  w.g2w = up_w("gamma_head.2.weight");
  w.g2b = up_w("gamma_head.2.bias");
  w.r0w = up_w("risk_enc.net.0.weight");
  w.r0b = up_w("risk_enc.net.0.bias");
  w.r2w = up_w("risk_enc.net.2.weight");
  w.r2b = up_w("risk_enc.net.2.bias");
  w.r4w = up_w("risk_enc.net.4.weight");
  w.r4b = up_w("risk_enc.net.4.bias");
  w.r8w = up_w("risk_enc.net.8.weight");
  w.r8b = up_w("risk_enc.net.8.bias");
  w.ls0w = up_w("lam_soft_head.0.weight");
  w.ls0b = up_w("lam_soft_head.0.bias");
  w.ls2w = up_w("lam_soft_head.2.weight");
  w.ls2b = up_w("lam_soft_head.2.bias");
  w.lh0w = up_w("lam_hard_head.0.weight");
  w.lh0b = up_w("lam_hard_head.0.bias");
  w.lh2w = up_w("lam_hard_head.2.weight");
  w.lh2b = up_w("lam_hard_head.2.bias");
  w.ml0w = up_w("mu_lat_head.0.weight");
  w.ml0b = up_w("mu_lat_head.0.bias");
  w.ml2w = up_w("mu_lat_head.2.weight");
  w.ml2b = up_w("mu_lat_head.2.bias");

  float *d_alpha = static_cast<float *>(dmalloc((std::size_t)total * sizeof(float)));
  float *d_beta = static_cast<float *>(dmalloc((std::size_t)n * sizeof(float)));
  float *d_gamma = static_cast<float *>(dmalloc((std::size_t)n * sizeof(float)));
  float *d_ls = static_cast<float *>(dmalloc((std::size_t)n * sizeof(float)));
  float *d_lh = static_cast<float *>(dmalloc((std::size_t)n * sizeof(float)));
  float *d_ml = static_cast<float *>(dmalloc((std::size_t)n * sizeof(float)));
  const long scratch_stride = 2L * 16 * P * P;
  float *d_scratch = static_cast<float *>(dmalloc((std::size_t)n * scratch_stride * sizeof(float)));

  const std::size_t smem =
      ((std::size_t)max_T * (DT + 3 * DT + DT + FF) + DT + 1024 + DT + 2 * DT) * sizeof(float);
  if (smem > 48u * 1024u)
    cuda_check(
        cudaFuncSetAttribute(ce_fwd_k, cudaFuncAttributeMaxDynamicSharedMemorySize, (int)smem),
        "set max shared");
  ce_fwd_k<<<n, DT, smem>>>(d_obs, d_mask, d_off, d_goal, d_risk, P, max_T, eps_, lam_soft_max_,
                            lam_hard_max_, mu_lat_max_, w, d_scratch, scratch_stride, d_alpha,
                            d_beta, d_gamma, d_ls, d_lh, d_ml);
  cuda_check(cudaGetLastError(), "launch");
  cuda_check(cudaDeviceSynchronize(), "sync");

  auto d2h = [&](void *dst, const void *src, std::size_t bytes) {
    if (bytes)
      cuda_check(cudaMemcpy(dst, src, bytes, cudaMemcpyDeviceToHost), "D2H");
  };
  d2h(alphas_out, d_alpha, (std::size_t)total * sizeof(float));
  d2h(beta, d_beta, (std::size_t)n * sizeof(float));
  d2h(gamma, d_gamma, (std::size_t)n * sizeof(float));
  d2h(lam_soft, d_ls, (std::size_t)n * sizeof(float));
  d2h(lam_hard, d_lh, (std::size_t)n * sizeof(float));
  d2h(mu_lat, d_ml, (std::size_t)n * sizeof(float));

  for (void *p : allocs)
    cudaFree(p);
}

// ═══════════════════════════ backward ═══════════════════════════════════════
namespace {

// Grad pointers parallel to DW (all borrowed device buffers, zeroed by the host;
// the kernel atomicAdds into them).
struct DWg {
  float *ge0w, *ge0b, *ge2w, *ge2b;
  float *oe0w, *oe0b, *oe2w, *oe2b;
  float *inw[NL], *inb[NL], *outw[NL], *outb[NL];
  float *l1w[NL], *l1b[NL], *l2w[NL], *l2b[NL];
  float *n1w[NL], *n1b[NL], *n2w[NL], *n2b[NL];
  float *a0w, *a0b, *a2w, *a2b;
  float *b0w, *b0b, *b2w, *b2b;
  float *g0w, *g0b, *g2w, *g2b;
  float *r0w, *r0b, *r2w, *r2b, *r4w, *r4b, *r8w, *r8b;
  float *ls0w, *ls0b, *ls2w, *ls2b;
  float *lh0w, *lh0b, *lh2w, *lh2b;
  float *ml0w, *ml0b, *ml2w, *ml2b;
};

// Linear VJP. gy[rows,out] -> gx[rows,in] (+=, in shared/global), gW[out,in] and
// gb[out] via atomicAdd (shared across agents). Cooperative over the block.
__device__ inline void d_lin_bwd(const float *x, int rows, int in, const float *W, int out,
                                 const float *gy, float *gx, float *gW, float *gb) {
  // gx: per (row, i) accumulate over out.
  if (gx)
    for (int idx = threadIdx.x; idx < rows * in; idx += DT) {
      const int r = idx / in, i = idx - r * in;
      float acc = 0.0f;
      const float *gyr = gy + r * out;
      for (int o = 0; o < out; ++o)
        acc += gyr[o] * W[(long)o * in + i];
      gx[idx] += acc;
    }
  // gW[o,i] += sum_r gy[r,o]*x[r,i]; gb[o] += sum_r gy[r,o]. atomicAdd.
  for (int o = threadIdx.x; o < out; o += DT) {
    float gbo = 0.0f;
    for (int r = 0; r < rows; ++r)
      gbo += gy[r * out + o];
    if (gb)
      atomicAdd(&gb[o], gbo);
    if (gW)
      for (int i = 0; i < in; ++i) {
        float acc = 0.0f;
        for (int r = 0; r < rows; ++r)
          acc += gy[r * out + o] * x[r * in + i];
        atomicAdd(&gW[(long)o * in + i], acc);
      }
  }
}

// LayerNorm VJP (normalized-Jacobian). gy[rows,DT] -> gx (+=), gg/gb atomicAdd.
__device__ inline void d_ln_bwd(const float *x, int rows, const float *g, const float *gy,
                                float *gx, float *gg, float *gb, float eps, float *red) {
  const int t = threadIdx.x;
  for (int r = 0; r < rows; ++r) {
    const float xv = x[r * DT + t];
    const float mu = bsum64(xv, red) / (float)DT;
    const float dv = xv - mu;
    const float var = bsum64(dv * dv, red) / (float)DT;
    const float inv = rsqrtf(var + eps);
    const float xhat = dv * inv;
    const float gyt = gy[r * DT + t];
    const float gxhat = gyt * g[t];
    if (gg)
      atomicAdd(&gg[t], gyt * xhat);
    if (gb)
      atomicAdd(&gb[t], gyt);
    const float s1 = bsum64(gxhat, red) / (float)DT;
    const float s2 = bsum64(gxhat * xhat, red) / (float)DT;
    if (gx)
      gx[r * DT + t] += inv * (gxhat - s1 - xhat * s2);
    __syncthreads();
  }
}

// Conv2d VJP (k=3, pad=1). gout[Cout,Ho,Wo] -> gx[Cin,H,W] (+= atomicAdd, shared
// input positions), gW/gb atomicAdd. Cooperative over Cout*Ho*Wo.
__device__ inline void d_conv_bwd(const float *x, int Cin, int H, int W, const float *w, int Cout,
                                  int stride, int Ho, int Wo, const float *gout, float *gx,
                                  float *gW, float *gb) {
  for (int idx = threadIdx.x; idx < Cout * Ho * Wo; idx += DT) {
    const int oc = idx / (Ho * Wo);
    const int rem = idx - oc * (Ho * Wo);
    const int oy = rem / Wo, ox = rem % Wo;
    const float go = gout[idx];
    if (gb)
      atomicAdd(&gb[oc], go);
    for (int ic = 0; ic < Cin; ++ic) {
      const float *xc = x + (long)ic * H * W;
      float *gxc = gx ? gx + (long)ic * H * W : nullptr;
      float *gwc = gW ? gW + ((long)oc * Cin + ic) * 9 : nullptr;
      const float *wc = w + ((long)oc * Cin + ic) * 9;
      for (int ky = 0; ky < 3; ++ky) {
        const int iy = oy * stride + ky - 1;
        if (iy < 0 || iy >= H)
          continue;
        for (int kx = 0; kx < 3; ++kx) {
          const int ix = ox * stride + kx - 1;
          if (ix < 0 || ix >= W)
            continue;
          if (gwc)
            atomicAdd(&gwc[ky * 3 + kx], go * xc[iy * W + ix]);
          if (gxc)
            atomicAdd(&gxc[iy * W + ix], go * wc[ky * 3 + kx]);
        }
      }
    }
  }
}

// AdaptiveAvgPool VJP: gout[C,4,4] -> gx[C,H,W] (+=; disjoint bins, no atomic).
__device__ inline void d_pool_bwd(int C, int H, int W, const float *gout, float *gx) {
  for (int idx = threadIdx.x; idx < C * 16; idx += DT) {
    const int c = idx / 16, bin = idx - c * 16, bi = bin / 4, bj = bin % 4;
    const int r0 = (bi * H) / 4, r1 = ((bi + 1) * H + 3) / 4;
    const int c0 = (bj * W) / 4, c1 = ((bj + 1) * W + 3) / 4;
    const float gg = gout[(c * 4 + bi) * 4 + bj] / (float)((r1 - r0) * (c1 - c0));
    float *gxc = gx + (long)c * H * W;
    for (int r = r0; r < r1; ++r)
      for (int cc = c0; cc < c1; ++cc)
        gxc[r * W + cc] += gg; // disjoint across (c,bi,bj) threads
  }
}

// 2-linear head VJP: recompute h1/h1r/h2, apply the activation-grad, backprop
// Linear.2 -> ReLU -> Linear.0, accumulating into gz and atomicAdd-ing weights.
// act: 0 softplus, 1 sigmoid*cap. `ff`/`ff1` are [64] shared scratch.
__device__ inline void d_head_bwd(const float *h0w, const float *h0b, const float *h2w,
                                  const float *h2b, const float *z, int zin, float g_after, int act,
                                  float cap, float *gz, float *gh0w, float *gh0b, float *gh2w,
                                  float *gh2b, float *h1, float *h1r) {
  const int t = threadIdx.x;
  lin(z, 1, zin, h0w, h0b, DT, h1); // h1 = Linear.0(z)
  __syncthreads();
  h1r[t] = drelu(h1[t]);
  __syncthreads();
  __shared__ float g_h2;
  if (t == 0) {
    float h2 = h2b[0];
    for (int i = 0; i < DT; ++i)
      h2 += h1r[i] * h2w[i];
    if (act == 0)
      g_h2 = g_after * dsig(h2);
    else {
      const float s = dsig(h2);
      g_h2 = g_after * cap * s * (1.0f - s);
    }
  }
  __syncthreads();
  // Linear.2 bwd (1 output): gh2w[i] += g_h2*h1r[i]; gh2b += g_h2; g_h1r[i]=g_h2*h2w[i]
  atomicAdd(&gh2w[t], g_h2 * h1r[t]);
  if (t == 0)
    atomicAdd(&gh2b[0], g_h2);
  const float g_h1r = g_h2 * h2w[t];
  const float g_h1 = (h1[t] > 0.0f) ? g_h1r : 0.0f; // relu bwd -> reuse h1 as g_h1
  __syncthreads();
  h1[t] = g_h1; // stash g_h1 into h1 for the Linear.0 bwd
  __syncthreads();
  d_lin_bwd(z, 1, zin, h0w, DT, h1, gz, gh0w, gh0b);
  __syncthreads();
}

// Masked MHA VJP. x=[T,64] (the layer input), gout=[T,64] grad on the mha output;
// gx += grad on x; weight grads atomicAdd. qkv/P/ctx/gqkv/gctx are [T*192]/
// [NH*T*T]/[T*64]/[T*192]/[T*64] scratch. pad j: j>0 && obs_mask[o0+j-1]==0.
__device__ inline void d_mha_bwd(const float *x, int T, const float *w_in, const float *b_in,
                                 const float *w_out, const std::uint8_t *obs_mask, int o0,
                                 const float *gout, float *gx, float *gw_in, float *gb_in,
                                 float *gw_out, float *gb_out, float *qkv, float *P, float *ctx,
                                 float *gqkv, float *gctx) {
  const float scale = 1.0f / sqrtf((float)HD);
  lin(x, T, DT, w_in, b_in, 3 * DT, qkv);
  __syncthreads();
  for (int idx = threadIdx.x; idx < T * DT; idx += DT)
    ctx[idx] = 0.0f;
  for (int idx = threadIdx.x; idx < T * 3 * DT; idx += DT)
    gqkv[idx] = 0.0f;
  __syncthreads();
  // recompute P (softmax rows) + ctx
  for (int pr = threadIdx.x; pr < T * NH; pr += DT) {
    const int i = pr / NH, h = pr % NH;
    const float *qi = qkv + (long)i * 3 * DT + h * HD;
    float sc[64];
    float mx = -INFINITY;
    for (int j = 0; j < T; ++j) {
      if (j > 0 && obs_mask[o0 + j - 1] == 0) {
        sc[j] = -INFINITY;
        continue;
      }
      const float *kj = qkv + (long)j * 3 * DT + DT + h * HD;
      float s = 0.0f;
      for (int c = 0; c < HD; ++c)
        s += qi[c] * kj[c];
      s *= scale;
      sc[j] = s;
      if (s > mx)
        mx = s;
    }
    float den = 0.0f;
    for (int j = 0; j < T; ++j) {
      if (sc[j] == -INFINITY) {
        sc[j] = 0.0f;
        continue;
      }
      float e = expf(sc[j] - mx);
      sc[j] = e;
      den += e;
    }
    float invd = 1.0f / den;
    float *Pi = P + ((long)h * T + i) * T;
    float *ci = ctx + (long)i * DT + h * HD;
    for (int j = 0; j < T; ++j) {
      float pv = sc[j] * invd;
      Pi[j] = pv;
      if (pv == 0.0f)
        continue;
      const float *vj = qkv + (long)j * 3 * DT + 2 * DT + h * HD;
      for (int c = 0; c < HD; ++c)
        ci[c] += pv * vj[c];
    }
  }
  __syncthreads();
  // gctx = out_proj bwd
  for (int idx = threadIdx.x; idx < T * DT; idx += DT)
    gctx[idx] = 0.0f;
  __syncthreads();
  d_lin_bwd(ctx, T, DT, w_out, DT, gout, gctx, gw_out, gb_out);
  __syncthreads();
  // per (h,i): softmax VJP -> gq (unique write) + gv/gk (atomicAdd)
  for (int pr = threadIdx.x; pr < T * NH; pr += DT) {
    const int i = pr / NH, h = pr % NH;
    const float *Pi = P + ((long)h * T + i) * T;
    const float *gci = gctx + (long)i * DT + h * HD;
    float gp[64], gpp = 0.0f;
    for (int j = 0; j < T; ++j) {
      if (Pi[j] == 0.0f) {
        gp[j] = 0.0f;
        continue;
      }
      const float *vj = qkv + (long)j * 3 * DT + 2 * DT + h * HD;
      float dv = 0.0f;
      for (int c = 0; c < HD; ++c)
        dv += gci[c] * vj[c];
      gp[j] = dv;
      gpp += dv * Pi[j];
      float *gvj = gqkv + (long)j * 3 * DT + 2 * DT + h * HD;
      for (int c = 0; c < HD; ++c)
        atomicAdd(&gvj[c], Pi[j] * gci[c]);
    }
    const float *qi = qkv + (long)i * 3 * DT + h * HD;
    float *gqi = gqkv + (long)i * 3 * DT + h * HD; // q slice: unique per (h,i)
    for (int c = 0; c < HD; ++c) {
      float acc = 0.0f;
      for (int j = 0; j < T; ++j) {
        if (Pi[j] == 0.0f)
          continue;
        acc += Pi[j] * (gp[j] - gpp) * scale * (qkv + (long)j * 3 * DT + DT + h * HD)[c];
      }
      gqi[c] += acc; // += (zeroed) — q slice touched only by this (h,i)
    }
    for (int j = 0; j < T; ++j) {
      if (Pi[j] == 0.0f)
        continue;
      const float gsj = Pi[j] * (gp[j] - gpp) * scale;
      float *gkj = gqkv + (long)j * 3 * DT + DT + h * HD;
      for (int c = 0; c < HD; ++c)
        atomicAdd(&gkj[c], gsj * qi[c]);
    }
  }
  __syncthreads();
  d_lin_bwd(x, T, DT, w_in, 3 * DT, gqkv, gx, gw_in, gb_in);
  __syncthreads();
}

// Per-agent backward scratch size (floats), computed the same way the kernel
// bumps. The host sizes the stride at max_T; each agent bumps <= this.
__host__ __device__ inline long bwd_scratch_floats(int T, int P, int no) {
  const int H2 = (P + 2 - 3) / 2 + 1, H3 = (H2 + 2 - 3) / 2 + 1;
  long f = 0;
  f += 64 + 64;               // zg1, zg1r
  f += (long)no * 128 * 2;    // zo1, zo1r
  f += (long)T * 64 * 3;      // tin0, tin1, tokensF
  f += (long)T * 64 * 2 * 2;  // attnres[2], ln1[2]  wait counted below
  f += (long)T * 128 * 2 * 2; // ff1[2], ff1r[2]
  f += (long)T * 64 * 2;      // ff2res[2]
  f += (long)T * 64 * 2;      // attnres[2] (grouped)
  f += 16L * P * P * 2;       // c0, c0r
  f += 32L * H2 * H2 * 2;     // c1, c1r
  f += 64L * H3 * H3 * 2;     // c2, c2r
  f += 1024 + 64 + 64;        // pooled, risk1, risk1r
  // reverse (reused/transient)
  f += (long)T * 64 * 6;  // g_tok, g_ff2res, g_ln1, g_attnres, g_attn, g_tin
  f += (long)T * 128 * 2; // g_ff1r, g_ff1
  f += 128 + 64 + 128;    // g_mat, g_risk1, matbuf
  f += (long)no * 64 * 4; // a1, a1r, g_a1, g_a1r
  f += (long)T * 192 * 2; // qkv, gqkv
  f += (long)T * 64 * 2;  // ctx, gctx
  f += (long)4 * T * T;   // P (NH*T*T)
  f += 64 + 64;           // h1, h1r
  f += 1024;              // g_pooled
  f += 16L * P * P + 32L * H2 * H2 + 64L * H3 * H3; // g_c0(=g_c0r reuse chain, alloc max)
  f += 16L * P * P + 32L * H2 * H2 + 64L * H3 * H3; // g_c0r/g_c1/... second set
  return f + 256;                                   // margin
}

// One agent per block. Recompute forward (cache activations into per-agent
// device scratch) then reverse, atomicAdd-ing weight grads into g. blockDim=64.
__global__ void ce_bwd_k(const float *obs_feats, const std::uint8_t *obs_mask,
                         const int *obs_offsets, const float *goal_feats, const float *risk_patch,
                         int P, float eps, float ls_max, float lh_max, float ml_max, DW w, DWg g,
                         const float *g_alphas, const float *g_beta, const float *g_gamma,
                         const float *g_lam_soft, const float *g_lam_hard, const float *g_mu_lat,
                         float *scratch, long stride) {
  const int agent = blockIdx.x, t = threadIdx.x;
  const int o0 = obs_offsets[agent], no = obs_offsets[agent + 1] - o0, T = 1 + no;
  const int H2 = (P + 2 - 3) / 2 + 1, H3 = (H2 + 2 - 3) / 2 + 1;
  __shared__ float red[64];
  float *sp = scratch + (long)agent * stride;
#define BUMP(nm, n)                                                                                \
  float *nm = sp;                                                                                  \
  sp += (n)
  BUMP(zg1, 64);
  BUMP(zg1r, 64);
  BUMP(zo1, (long)no * 128);
  BUMP(zo1r, (long)no * 128);
  BUMP(tin0, (long)T * 64);
  BUMP(tin1, (long)T * 64);
  BUMP(tokF, (long)T * 64);
  BUMP(ar0, (long)T * 64);
  BUMP(ar1, (long)T * 64);
  BUMP(l10, (long)T * 64);
  BUMP(l11, (long)T * 64);
  BUMP(f10, (long)T * 128);
  BUMP(f11, (long)T * 128);
  BUMP(f1r0, (long)T * 128);
  BUMP(f1r1, (long)T * 128);
  BUMP(fr0, (long)T * 64);
  BUMP(fr1, (long)T * 64);
  BUMP(c0, 16L * P * P);
  BUMP(c0r, 16L * P * P);
  BUMP(c1, 32L * H2 * H2);
  BUMP(c1r, 32L * H2 * H2);
  BUMP(c2, 64L * H3 * H3);
  BUMP(c2r, 64L * H3 * H3);
  BUMP(pooled, 1024);
  BUMP(risk1, 64);
  BUMP(risk1r, 64);
  BUMP(gtok, (long)T * 64);
  BUMP(gmat, 128);
  BUMP(grisk1, 64);
  BUMP(matbuf, 128);
  BUMP(gff2res, (long)T * 64);
  BUMP(gln1, (long)T * 64);
  BUMP(gff1r, (long)T * 128);
  BUMP(gff1, (long)T * 128);
  BUMP(gar, (long)T * 64);
  BUMP(gattn, (long)T * 64);
  BUMP(gtin, (long)T * 64);
  BUMP(a1, (long)no * 64);
  BUMP(a1r, (long)no * 64);
  BUMP(qkv, (long)T * 192);
  BUMP(gqkv, (long)T * 192);
  BUMP(ctx, (long)T * 64);
  BUMP(gctx, (long)T * 64);
  BUMP(Pw, 4L * T * T);
  BUMP(h1, 64);
  BUMP(h1r, 64);
  BUMP(gpool, 1024);
  BUMP(gcA, 16L * P * P);
  BUMP(gcB, 32L * H2 * H2);
  BUMP(gcC, 64L * H3 * H3);
#undef BUMP

  // ── forward recompute (cache) ──────────────────────────────────────────────
  const float *gf = goal_feats + (long)agent * 4;
  for (int o = t; o < 64; o += DT)
    zg1[o] = gf[0] * w.ge0w[o * 4] + gf[1] * w.ge0w[o * 4 + 1] + gf[2] * w.ge0w[o * 4 + 2] +
             gf[3] * w.ge0w[o * 4 + 3] + w.ge0b[o];
  __syncthreads();
  zg1r[t] = drelu(zg1[t]);
  __syncthreads();
  lin(zg1r, 1, 64, w.ge2w, w.ge2b, 64, tin0);
  __syncthreads();
  for (int j = 0; j < no; ++j) {
    const float *of = obs_feats + (long)(o0 + j) * 6;
    for (int o = t; o < 128; o += DT) {
      float acc = w.oe0b[o];
      for (int i = 0; i < 6; ++i)
        acc += of[i] * w.oe0w[o * 6 + i];
      zo1[j * 128 + o] = acc;
      zo1r[j * 128 + o] = drelu(acc);
    }
    __syncthreads();
    lin(zo1r + (long)j * 128, 1, 128, w.oe2w, w.oe2b, 64, tin0 + (long)(1 + j) * 64);
    __syncthreads();
  }
  // transformer layer 0 (tin0) -> tin1 ; layer 1 (tin1) -> tokF
  float *tinL[2] = {tin0, tin1};
  float *arL[2] = {ar0, ar1}, *l1L[2] = {l10, l11}, *f1L[2] = {f10, f11};
  float *f1rL[2] = {f1r0, f1r1}, *frL[2] = {fr0, fr1};
  float *outL[2] = {tin1, tokF};
  const float *inw[2] = {w.inw[0], w.inw[1]}, *inb[2] = {w.inb[0], w.inb[1]};
  const float *ow[2] = {w.outw[0], w.outw[1]}, *ob[2] = {w.outb[0], w.outb[1]};
  const float *l1w[2] = {w.l1w[0], w.l1w[1]}, *l1b[2] = {w.l1b[0], w.l1b[1]};
  const float *l2w[2] = {w.l2w[0], w.l2w[1]}, *l2b[2] = {w.l2b[0], w.l2b[1]};
  const float *n1w[2] = {w.n1w[0], w.n1w[1]}, *n1b[2] = {w.n1b[0], w.n1b[1]};
  const float *n2w[2] = {w.n2w[0], w.n2w[1]}, *n2b[2] = {w.n2b[0], w.n2b[1]};
  for (int L = 0; L < NL; ++L) {
    // mha(tinL) -> ar (reuse ctx/qkv/Pw as scratch); ar = attn + tin
    const float scale = 1.0f / sqrtf((float)HD);
    lin(tinL[L], T, 64, inw[L], inb[L], 192, qkv);
    __syncthreads();
    for (int idx = t; idx < T * 64; idx += DT)
      ctx[idx] = 0.0f;
    __syncthreads();
    for (int pr = t; pr < T * NH; pr += DT) {
      const int i = pr / NH, h = pr % NH;
      const float *qi = qkv + (long)i * 192 + h * HD;
      float sc[64], mx = -INFINITY;
      for (int j = 0; j < T; ++j) {
        if (j > 0 && obs_mask[o0 + j - 1] == 0) {
          sc[j] = -INFINITY;
          continue;
        }
        const float *kj = qkv + (long)j * 192 + 64 + h * HD;
        float s = 0.f;
        for (int c = 0; c < HD; ++c)
          s += qi[c] * kj[c];
        sc[j] = s * scale;
        if (sc[j] > mx)
          mx = sc[j];
      }
      float den = 0.f;
      for (int j = 0; j < T; ++j) {
        if (sc[j] == -INFINITY) {
          sc[j] = 0.f;
          continue;
        }
        float e = expf(sc[j] - mx);
        sc[j] = e;
        den += e;
      }
      float *ci = ctx + (long)i * 64 + h * HD, invd = 1.f / den;
      for (int j = 0; j < T; ++j) {
        if (sc[j] == 0.f)
          continue;
        float pv = sc[j] * invd;
        const float *vj = qkv + (long)j * 192 + 128 + h * HD;
        for (int c = 0; c < HD; ++c)
          ci[c] += pv * vj[c];
      }
    }
    __syncthreads();
    lin(ctx, T, 64, ow[L], ob[L], 64, arL[L]); // attn
    __syncthreads();
    for (int idx = t; idx < T * 64; idx += DT)
      arL[L][idx] += tinL[L][idx]; // residual
    __syncthreads();
    layernorm_rows(arL[L], T, n1w[L], n1b[L], eps, l1L[L], red);
    __syncthreads();
    lin(l1L[L], T, 64, l1w[L], l1b[L], 128, f1L[L]);
    __syncthreads();
    for (int idx = t; idx < T * 128; idx += DT)
      f1rL[L][idx] = drelu(f1L[L][idx]);
    __syncthreads();
    lin(f1rL[L], T, 128, l2w[L], l2b[L], 64, frL[L]);
    __syncthreads();
    for (int idx = t; idx < T * 64; idx += DT)
      frL[L][idx] += l1L[L][idx]; // residual
    __syncthreads();
    layernorm_rows(frL[L], T, n2w[L], n2b[L], eps, outL[L], red);
    __syncthreads();
  }
  float *ctxF = tokF; // ctx = tokF row 0
  // CNN forward
  conv_relu_s(risk_patch + (long)agent * 2 * P * P, 2, P, P, w.r0w, w.r0b, 16, 1, c0r, P, P);
  __syncthreads();
  for (int idx = t; idx < 16 * P * P; idx += DT)
    c0[idx] = c0r[idx]; // pre-relu unavailable; store post
  __syncthreads();
  conv_relu_s(c0r, 16, P, P, w.r2w, w.r2b, 32, 2, c1r, H2, H2);
  __syncthreads();
  for (int idx = t; idx < 32 * H2 * H2; idx += DT)
    c1[idx] = c1r[idx];
  __syncthreads();
  conv_relu_s(c1r, 32, H2, H2, w.r4w, w.r4b, 64, 2, c2r, H3, H3);
  __syncthreads();
  for (int idx = t; idx < 64 * H3 * H3; idx += DT)
    c2[idx] = c2r[idx];
  __syncthreads();
  for (int idx = t; idx < 64 * 16; idx += DT) {
    const int c = idx / 16, bn = idx - c * 16, bi = bn / 4, bj = bn % 4;
    const int r0 = (bi * H3) / 4, r1 = ((bi + 1) * H3 + 3) / 4, cc0 = (bj * H3) / 4,
              cc1 = ((bj + 1) * H3 + 3) / 4;
    float acc = 0.f;
    int cnt = 0;
    const float *xc = c2r + (long)c * H3 * H3;
    for (int r = r0; r < r1; ++r)
      for (int cc = cc0; cc < cc1; ++cc) {
        acc += xc[r * H3 + cc];
        ++cnt;
      }
    pooled[(c * 4 + bi) * 4 + bj] = acc / (float)cnt;
  }
  __syncthreads();
  lin(pooled, 1, 1024, w.r8w, w.r8b, 64, risk1);
  __syncthreads();
  risk1r[t] = drelu(risk1[t]);
  __syncthreads();

  // ── reverse ────────────────────────────────────────────────────────────────
  for (int i = t; i < 128; i += DT)
    gmat[i] = 0.f;
  __syncthreads();
  // lam heads: mat = [risk1r ; ctxF]. Dedicated 128-float buffer — must NOT reuse
  // gtok, which is only T*64 floats (T=1 for a no-obstacle agent overflows into gmat).
  float *mat = matbuf;
  mat[t] = risk1r[t];
  mat[64 + t] = ctxF[t];
  __syncthreads();
  d_head_bwd(w.ls0w, w.ls0b, w.ls2w, w.ls2b, mat, 128, g_lam_soft[agent], 1, ls_max, gmat, g.ls0w,
             g.ls0b, g.ls2w, g.ls2b, h1, h1r);
  d_head_bwd(w.lh0w, w.lh0b, w.lh2w, w.lh2b, mat, 128, g_lam_hard[agent], 1, lh_max, gmat, g.lh0w,
             g.lh0b, g.lh2w, g.lh2b, h1, h1r);
  d_head_bwd(w.ml0w, w.ml0b, w.ml2w, w.ml2b, mat, 128, g_mu_lat[agent], 1, ml_max, gmat, g.ml0w,
             g.ml0b, g.ml2w, g.ml2b, h1, h1r);
  __syncthreads();
  // g_risk1r = gmat[0:64] ; g_ctx = gmat[64:128]
  grisk1[t] = gmat[t]; // temporarily g_risk1r
  float g_ctx_t = gmat[64 + t];
  __syncthreads();
  // beta/gamma on ctxF accumulate into g_ctx (use gtin[0:64] as g_ctx buffer)
  for (int i = t; i < 64; i += DT)
    gtin[i] = 0.f;
  __syncthreads();
  gtin[t] = g_ctx_t; // seed g_ctx with the lam contribution
  __syncthreads();
  d_head_bwd(w.b0w, w.b0b, w.b2w, w.b2b, ctxF, 64, g_beta[agent], 0, 0.f, gtin, g.b0w, g.b0b, g.b2w,
             g.b2b, h1, h1r);
  d_head_bwd(w.g0w, w.g0b, w.g2w, w.g2b, ctxF, 64, g_gamma[agent], 0, 0.f, gtin, g.g0w, g.g0b,
             g.g2w, g.g2b, h1, h1r);
  __syncthreads();
  // g_tokF: row0 = g_ctx (gtin), rows 1.. from alpha head
  for (int idx = t; idx < T * 64; idx += DT)
    gtok[idx] = 0.f;
  __syncthreads();
  gtok[t] = gtin[t]; // row 0
  __syncthreads();
  if (no > 0) {
    lin(tokF + 64, no, 64, w.a0w, w.a0b, 64, a1);
    __syncthreads();
    for (int idx = t; idx < no * 64; idx += DT)
      a1r[idx] = drelu(a1[idx]);
    __syncthreads();
    // g_a2[i] = mask? g_alphas[i]*sigmoid(a2[i]) : 0 ; then linear.2 bwd, relu, linear.0 bwd
    // recompute a2 + g_a2 into a per-obstacle value; use gqkv[0:no] as g_a2 scratch
    if (t == 0)
      for (int i = 0; i < no; ++i) {
        float a2 = w.a2b[0];
        for (int k = 0; k < 64; ++k)
          a2 += a1r[i * 64 + k] * w.a2w[k];
        gqkv[i] = obs_mask[o0 + i] ? g_alphas[o0 + i] * dsig(a2) : 0.f;
      }
    __syncthreads();
    // g_a1r = linear.2 bwd ; use gff1r[0:no*64] as g_a1r
    for (int idx = t; idx < no * 64; idx += DT) {
      const int i = idx / 64, k = idx - i * 64;
      gff1r[idx] = gqkv[i] * w.a2w[k];
      atomicAdd(&g.a2w[k], gqkv[i] * a1r[idx]);
    }
    if (t == 0)
      for (int i = 0; i < no; ++i)
        atomicAdd(&g.a2b[0], gqkv[i]);
    __syncthreads();
    // relu bwd -> gff1 ; then linear.0 bwd into gtok[64:]
    for (int idx = t; idx < no * 64; idx += DT)
      gff1[idx] = (a1[idx] > 0.f) ? gff1r[idx] : 0.f;
    __syncthreads();
    d_lin_bwd(tokF + 64, no, 64, w.a0w, 64, gff1, gtok + 64, g.a0w, g.a0b);
    __syncthreads();
  }
  // reverse transformer
  for (int idx = t; idx < T * 64; idx += DT)
    gtin[idx] = gtok[idx]; // g_cur = g_tokF
  __syncthreads();
  for (int L = NL - 1; L >= 0; --L) {
    for (int idx = t; idx < T * 64; idx += DT)
      gff2res[idx] = 0.f;
    __syncthreads();
    d_ln_bwd(frL[L], T, n2w[L], gtin, gff2res, g.n2w[L], g.n2b[L], eps, red);
    __syncthreads();
    // g_ln1 = g_ff2res (residual) ; g_ff2 = g_ff2res
    for (int idx = t; idx < T * 64; idx += DT)
      gln1[idx] = gff2res[idx];
    for (int idx = t; idx < T * 128; idx += DT)
      gff1r[idx] = 0.f;
    __syncthreads();
    d_lin_bwd(f1rL[L], T, 128, l2w[L], 64, gff2res, gff1r, g.l2w[L], g.l2b[L]);
    __syncthreads();
    for (int idx = t; idx < T * 128; idx += DT)
      gff1[idx] = (f1L[L][idx] > 0.f) ? gff1r[idx] : 0.f;
    __syncthreads();
    d_lin_bwd(l1L[L], T, 64, l1w[L], 128, gff1, gln1, g.l1w[L], g.l1b[L]); // accumulate into gln1
    __syncthreads();
    for (int idx = t; idx < T * 64; idx += DT)
      gar[idx] = 0.f;
    __syncthreads();
    d_ln_bwd(arL[L], T, n1w[L], gln1, gar, g.n1w[L], g.n1b[L], eps, red);
    __syncthreads();
    // g_attn = g_ar ; g_tin(next) = g_ar (residual) ; mha_bwd accumulates into it
    for (int idx = t; idx < T * 64; idx += DT) {
      gattn[idx] = gar[idx];
      gtok[idx] = gar[idx];
    }
    __syncthreads();
    d_mha_bwd(tinL[L], T, inw[L], inb[L], ow[L], obs_mask, o0, gattn, gtok, g.inw[L], g.inb[L],
              g.outw[L], g.outb[L], qkv, Pw, ctx, gqkv, gctx);
    __syncthreads();
    for (int idx = t; idx < T * 64; idx += DT)
      gtin[idx] = gtok[idx]; // g_cur
    __syncthreads();
  }
  // encoders: g_cur row0 -> goal_enc ; rows1.. -> obs_enc
  {
    for (int i = t; i < 64; i += DT)
      gattn[i] = 0.f; // g_zg1r
    __syncthreads();
    d_lin_bwd(zg1r, 1, 64, w.ge2w, 64, gtin, gattn, g.ge2w, g.ge2b);
    __syncthreads();
    float gzg1 = (zg1[t] > 0.f) ? gattn[t] : 0.f;
    gar[t] = gzg1; // g_zg1
    __syncthreads();
    d_lin_bwd(gf, 1, 4, w.ge0w, 64, gar, nullptr, g.ge0w, g.ge0b);
    __syncthreads();
  }
  if (no > 0) {
    for (int idx = t; idx < no * 128; idx += DT)
      gff1r[idx] = 0.f; // g_zo1r
    __syncthreads();
    d_lin_bwd(zo1r, no, 128, w.oe2w, 64, gtin + 64, gff1r, g.oe2w, g.oe2b);
    __syncthreads();
    for (int idx = t; idx < no * 128; idx += DT)
      gff1[idx] = (zo1[idx] > 0.f) ? gff1r[idx] : 0.f;
    __syncthreads();
    d_lin_bwd(obs_feats + (long)o0 * 6, no, 6, w.oe0w, 128, gff1, nullptr, g.oe0w, g.oe0b);
    __syncthreads();
  }
  // CNN backward: g_risk1r(grisk1) -> relu -> linear.8 -> pool -> conv4 -> conv2 -> conv0
  float g_r1 = (risk1[t] > 0.f) ? grisk1[t] : 0.f;
  grisk1[t] = g_r1; // g_risk1
  __syncthreads();
  for (int i = t; i < 1024; i += DT)
    gpool[i] = 0.f;
  __syncthreads();
  d_lin_bwd(pooled, 1, 1024, w.r8w, 64, grisk1, gpool, g.r8w, g.r8b);
  __syncthreads();
  for (int i = t; i < 64 * H3 * H3; i += DT)
    gcC[i] = 0.f;
  __syncthreads();
  d_pool_bwd(64, H3, H3, gpool, gcC); // g_c2r
  __syncthreads();
  for (int i = t; i < 64 * H3 * H3; i += DT)
    gcC[i] = (c2[i] > 0.f) ? gcC[i] : 0.f; // relu -> g_c2
  __syncthreads();
  for (int i = t; i < 32 * H2 * H2; i += DT)
    gcB[i] = 0.f;
  __syncthreads();
  d_conv_bwd(c1r, 32, H2, H2, w.r4w, 64, 2, H3, H3, gcC, gcB, g.r4w, g.r4b); // g_c1r
  __syncthreads();
  for (int i = t; i < 32 * H2 * H2; i += DT)
    gcB[i] = (c1[i] > 0.f) ? gcB[i] : 0.f; // g_c1
  __syncthreads();
  for (int i = t; i < 16 * P * P; i += DT)
    gcA[i] = 0.f;
  __syncthreads();
  d_conv_bwd(c0r, 16, P, P, w.r2w, 32, 2, H2, H2, gcB, gcA, g.r2w, g.r2b); // g_c0r
  __syncthreads();
  for (int i = t; i < 16 * P * P; i += DT)
    gcA[i] = (c0[i] > 0.f) ? gcA[i] : 0.f; // g_c0
  __syncthreads();
  d_conv_bwd(risk_patch + (long)agent * 2 * P * P, 2, P, P, w.r0w, 16, 1, P, P, gcA, nullptr, g.r0w,
             g.r0b);
}

} // namespace

void coef_energy_net::backward_batch_cuda(const float *obs_feats, const std::uint8_t *obs_mask,
                                          const int *obs_offsets, int n, const float *goal_feats,
                                          const float *risk_patch, int patch_p,
                                          const float *g_alphas, const float *g_beta,
                                          const float *g_gamma, const float *g_lam_soft,
                                          const float *g_lam_hard, const float *g_mu_lat,
                                          param_grads &grads) const {
  if (!coef_energy_cuda_available())
    throw std::runtime_error("coef_energy_net::backward_batch_cuda: no CUDA device");
  if (patch_p != patch_size_)
    throw std::runtime_error("coef_energy_net::backward_batch_cuda: patch_p != patch_size");
  if (n <= 0)
    return;
  const int P = patch_p, total = obs_offsets[n];
  int max_obs = 0;
  for (int i = 0; i < n; ++i)
    max_obs = std::max(max_obs, obs_offsets[i + 1] - obs_offsets[i]);

  std::vector<void *> allocs;
  auto dmalloc = [&](std::size_t b) -> void * {
    void *p = nullptr;
    cuda_check(cudaMalloc(&p, b ? b : 4), "cudaMalloc");
    allocs.push_back(p);
    return p;
  };
  auto up = [&](const void *src, std::size_t b) -> void * {
    void *p = dmalloc(b);
    if (b)
      cuda_check(cudaMemcpy(p, src, b, cudaMemcpyHostToDevice), "H2D");
    return p;
  };
  auto up_w = [&](const std::string &nm) -> const float * {
    const auto &tt = t(nm);
    return static_cast<const float *>(up(tt.data.data(), tt.data.size() * sizeof(float)));
  };
  std::vector<std::pair<std::string, float *>> gmap;
  auto gbuf = [&](const std::string &nm) -> float * {
    const auto &tt = t(nm);
    float *p = static_cast<float *>(dmalloc(tt.data.size() * sizeof(float)));
    cuda_check(cudaMemset(p, 0, tt.data.size() * sizeof(float)), "memset grad");
    gmap.emplace_back(nm, p);
    return p;
  };

  const float *d_obs = static_cast<const float *>(up(obs_feats, (std::size_t)total * 6 * 4));
  const unsigned char *d_mask =
      static_cast<const unsigned char *>(up(obs_mask, (std::size_t)total));
  const int *d_off = static_cast<const int *>(up(obs_offsets, (std::size_t)(n + 1) * 4));
  const float *d_goal = static_cast<const float *>(up(goal_feats, (std::size_t)n * 4 * 4));
  const float *d_risk = static_cast<const float *>(up(risk_patch, (std::size_t)n * 2 * P * P * 4));
  const float *dga = static_cast<const float *>(up(g_alphas, (std::size_t)total * 4));
  const float *dgb = static_cast<const float *>(up(g_beta, (std::size_t)n * 4));
  const float *dgg = static_cast<const float *>(up(g_gamma, (std::size_t)n * 4));
  const float *dgls = static_cast<const float *>(up(g_lam_soft, (std::size_t)n * 4));
  const float *dglh = static_cast<const float *>(up(g_lam_hard, (std::size_t)n * 4));
  const float *dgml = static_cast<const float *>(up(g_mu_lat, (std::size_t)n * 4));

  DW w{};
  DWg g{};
#define WB(field, name)                                                                            \
  w.field = up_w(name);                                                                            \
  g.field = gbuf(name)
  WB(ge0w, "goal_enc.0.weight");
  WB(ge0b, "goal_enc.0.bias");
  WB(ge2w, "goal_enc.2.weight");
  WB(ge2b, "goal_enc.2.bias");
  WB(oe0w, "obs_enc.0.weight");
  WB(oe0b, "obs_enc.0.bias");
  WB(oe2w, "obs_enc.2.weight");
  WB(oe2b, "obs_enc.2.bias");
  for (int L = 0; L < NL; ++L) {
    const std::string p = "fuser.layers." + std::to_string(L) + ".";
    w.inw[L] = up_w(p + "self_attn.in_proj_weight");
    g.inw[L] = gbuf(p + "self_attn.in_proj_weight");
    w.inb[L] = up_w(p + "self_attn.in_proj_bias");
    g.inb[L] = gbuf(p + "self_attn.in_proj_bias");
    w.outw[L] = up_w(p + "self_attn.out_proj.weight");
    g.outw[L] = gbuf(p + "self_attn.out_proj.weight");
    w.outb[L] = up_w(p + "self_attn.out_proj.bias");
    g.outb[L] = gbuf(p + "self_attn.out_proj.bias");
    w.l1w[L] = up_w(p + "linear1.weight");
    g.l1w[L] = gbuf(p + "linear1.weight");
    w.l1b[L] = up_w(p + "linear1.bias");
    g.l1b[L] = gbuf(p + "linear1.bias");
    w.l2w[L] = up_w(p + "linear2.weight");
    g.l2w[L] = gbuf(p + "linear2.weight");
    w.l2b[L] = up_w(p + "linear2.bias");
    g.l2b[L] = gbuf(p + "linear2.bias");
    w.n1w[L] = up_w(p + "norm1.weight");
    g.n1w[L] = gbuf(p + "norm1.weight");
    w.n1b[L] = up_w(p + "norm1.bias");
    g.n1b[L] = gbuf(p + "norm1.bias");
    w.n2w[L] = up_w(p + "norm2.weight");
    g.n2w[L] = gbuf(p + "norm2.weight");
    w.n2b[L] = up_w(p + "norm2.bias");
    g.n2b[L] = gbuf(p + "norm2.bias");
  }
  WB(a0w, "alpha_head.0.weight");
  WB(a0b, "alpha_head.0.bias");
  WB(a2w, "alpha_head.2.weight");
  WB(a2b, "alpha_head.2.bias");
  WB(b0w, "beta_head.0.weight");
  WB(b0b, "beta_head.0.bias");
  WB(b2w, "beta_head.2.weight");
  WB(b2b, "beta_head.2.bias");
  WB(g0w, "gamma_head.0.weight");
  WB(g0b, "gamma_head.0.bias");
  WB(g2w, "gamma_head.2.weight");
  WB(g2b, "gamma_head.2.bias");
  WB(r0w, "risk_enc.net.0.weight");
  WB(r0b, "risk_enc.net.0.bias");
  WB(r2w, "risk_enc.net.2.weight");
  WB(r2b, "risk_enc.net.2.bias");
  WB(r4w, "risk_enc.net.4.weight");
  WB(r4b, "risk_enc.net.4.bias");
  WB(r8w, "risk_enc.net.8.weight");
  WB(r8b, "risk_enc.net.8.bias");
  WB(ls0w, "lam_soft_head.0.weight");
  WB(ls0b, "lam_soft_head.0.bias");
  WB(ls2w, "lam_soft_head.2.weight");
  WB(ls2b, "lam_soft_head.2.bias");
  WB(lh0w, "lam_hard_head.0.weight");
  WB(lh0b, "lam_hard_head.0.bias");
  WB(lh2w, "lam_hard_head.2.weight");
  WB(lh2b, "lam_hard_head.2.bias");
  WB(ml0w, "mu_lat_head.0.weight");
  WB(ml0b, "mu_lat_head.0.bias");
  WB(ml2w, "mu_lat_head.2.weight");
  WB(ml2b, "mu_lat_head.2.bias");
#undef WB

  const long stride = bwd_scratch_floats(1 + max_obs, P, max_obs);
  float *d_scratch = static_cast<float *>(dmalloc((std::size_t)n * stride * sizeof(float)));

  ce_bwd_k<<<n, DT>>>(d_obs, d_mask, d_off, d_goal, d_risk, P, eps_, lam_soft_max_, lam_hard_max_,
                      mu_lat_max_, w, g, dga, dgb, dgg, dgls, dglh, dgml, d_scratch, stride);
  cuda_check(cudaGetLastError(), "launch");
  cuda_check(cudaDeviceSynchronize(), "sync");

  for (auto &kv : gmap) {
    std::vector<float> &dst = grads.at(kv.first);
    std::vector<float> tmp(dst.size());
    cuda_check(
        cudaMemcpy(tmp.data(), kv.second, dst.size() * sizeof(float), cudaMemcpyDeviceToHost),
        "D2H grad");
    for (std::size_t i = 0; i < dst.size(); ++i)
      dst[i] += tmp[i];
  }
  for (void *p : allocs)
    cudaFree(p);
}

} // namespace nav
} // namespace cvc
