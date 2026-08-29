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

constexpr int DT = 64;    // d_tok
constexpr int NH = 4;     // nhead
constexpr int HD = DT / NH; // 16
constexpr int NL = 2;     // num_layers
constexpr int FF = 128;   // FFN hidden / obs_enc.0 width

void cuda_check(cudaError_t e, const char *what) {
  if (e != cudaSuccess)
    throw std::runtime_error(std::string("cvc::nav coef_energy CUDA: ") + what + ": " +
                             cudaGetErrorString(e));
}

// Device weight pointers (all borrowed, uploaded once by the host wrapper).
struct DW {
  const float *ge0w, *ge0b, *ge2w, *ge2b;   // goal_enc 4->64->64
  const float *oe0w, *oe0b, *oe2w, *oe2b;   // obs_enc  6->128->64
  const float *inw[NL], *inb[NL];           // self_attn.in_proj  [192,64]/[192]
  const float *outw[NL], *outb[NL];         // self_attn.out_proj [64,64]/[64]
  const float *l1w[NL], *l1b[NL];           // linear1 [128,64]/[128]
  const float *l2w[NL], *l2b[NL];           // linear2 [64,128]/[64]
  const float *n1w[NL], *n1b[NL], *n2w[NL], *n2b[NL]; // norms [64]
  const float *a0w, *a0b, *a2w, *a2b;       // alpha_head 64->64->1
  const float *b0w, *b0b, *b2w, *b2b;       // beta_head
  const float *g0w, *g0b, *g2w, *g2b;       // gamma_head
  const float *r0w, *r0b, *r2w, *r2b, *r4w, *r4b, *r8w, *r8b; // risk CNN
  const float *ls0w, *ls0b, *ls2w, *ls2b;   // lam_soft_head 128->64->1
  const float *lh0w, *lh0b, *lh2w, *lh2b;   // lam_hard_head
  const float *ml0w, *ml0b, *ml2w, *ml2b;   // mu_lat_head
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
                         int P, int max_T, float eps, float ls_max, float lh_max, float ml_max, DW w,
                         float *scratch, long scratch_stride, float *alphas_out, float *beta,
                         float *gamma, float *lam_soft, float *lam_hard, float *mu_lat) {
  const int agent = blockIdx.x;
  const int t = threadIdx.x;
  const int o0 = obs_offsets[agent];
  const int n_obs = obs_offsets[agent + 1] - o0;
  const int T = 1 + n_obs;

  extern __shared__ float smem[];
  float *tok = smem;                 // [max_T*64]
  float *qkv = tok + max_T * DT;      // [max_T*192]
  float *cbuf = qkv + max_T * 3 * DT; // [max_T*64]
  float *ff = cbuf + max_T * DT;      // [max_T*128]
  float *red = ff + max_T * FF;       // [64]
  float *pooled = red + DT;           // [1024]
  float *risk_ctx = pooled + 64 * 4 * 4; // [64]
  float *mat = risk_ctx + DT;         // [128]

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
                                         float *beta, float *gamma, float *lam_soft, float *lam_hard,
                                         float *mu_lat) const {
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
  const float *d_risk = static_cast<const float *>(
      up(risk_patch, (std::size_t)n * 2 * P * P * sizeof(float)));

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
  float *d_scratch =
      static_cast<float *>(dmalloc((std::size_t)n * scratch_stride * sizeof(float)));

  const std::size_t smem =
      ((std::size_t)max_T * (DT + 3 * DT + DT + FF) + DT + 1024 + DT + 2 * DT) * sizeof(float);
  if (smem > 48u * 1024u)
    cuda_check(cudaFuncSetAttribute(ce_fwd_k, cudaFuncAttributeMaxDynamicSharedMemorySize,
                                    (int)smem),
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

} // namespace nav
} // namespace cvc
