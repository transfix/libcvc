/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// detail/nn_ops.h — the neural-net op primitives of CoefEnergyNetMaterial as
// matched forward+backward (VJP) pairs, for the P5 torch-free training backward.
//
// The forward ops are byte-for-byte the same arithmetic as coef_energy_net.cpp's
// (the FLOAT-parity-proven inference forward): linear, relu, layernorm, masked
// multi-head self-attention, conv2d (cross-correlation), adaptive-avg-pool. Each
// backward is the exact reverse-mode adjoint, validated op-by-op by a
// finite-difference gradcheck (nav_coef_energy_grad_test) — these ops all fail
// SILENTLY (a wrong LayerNorm mean term or a mis-scaled softmax still runs and
// merely descends the wrong loss), so per-op FD is the load-bearing check before
// the end-to-end model gradcheck.
//
// Backward buffers are ADDED into (accumulate; zero them first for a fresh grad).
// The transformer is POST-norm with a ReLU FFN and no final norm — that ordering
// lives in the model backward (coef_energy_net_backward.cpp), not here.

#ifndef CVC_NAV_DETAIL_NN_OPS_H
#define CVC_NAV_DETAIL_NN_OPS_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace cvc {
namespace nav {
namespace detail {
namespace nn {

// ── elementwise ──────────────────────────────────────────────────────────────
inline float relu(float x) { return x > 0.0f ? x : 0.0f; }
inline float sigmoidf(float x) { return 1.0f / (1.0f + std::exp(-x)); }
inline float softplusf(float x) {
  return x > 20.0f ? x : std::log1p(std::exp(x < 20.0f ? x : 20.0f));
}
// d(softplus)/dx = sigmoid(x); d(sigmoid)/dx = s(1-s).
inline float softplus_grad(float x) { return sigmoidf(x); }
inline float sigmoid_grad_from_y(float y) { return y * (1.0f - y); }

// ── linear: y[r,o] = b[o] + sum_i x[r,i]*W[o,in+i]  (W row-major [out,in]) ─────
inline void linear(const float *x, int rows, int in, const float *W, const float *b, int out,
                   float *y) {
  for (int r = 0; r < rows; ++r) {
    const float *xr = x + static_cast<std::size_t>(r) * in;
    float *yr = y + static_cast<std::size_t>(r) * out;
    for (int o = 0; o < out; ++o) {
      float acc = b[o];
      const float *w = W + static_cast<std::size_t>(o) * in;
      for (int i = 0; i < in; ++i)
        acc += xr[i] * w[i];
      yr[o] = acc;
    }
  }
}
// VJP: gy[rows,out] -> gx[rows,in] (+=), gW[out,in] (+=), gb[out] (+=).
inline void linear_bwd(const float *x, int rows, int in, const float *W, int out, const float *gy,
                       float *gx, float *gW, float *gb) {
  for (int r = 0; r < rows; ++r) {
    const float *xr = x + static_cast<std::size_t>(r) * in;
    const float *gyr = gy + static_cast<std::size_t>(r) * out;
    float *gxr = gx ? gx + static_cast<std::size_t>(r) * in : nullptr;
    for (int o = 0; o < out; ++o) {
      const float go = gyr[o];
      const float *w = W + static_cast<std::size_t>(o) * in;
      float *gw = gW ? gW + static_cast<std::size_t>(o) * in : nullptr;
      if (gb)
        gb[o] += go;
      for (int i = 0; i < in; ++i) {
        if (gxr)
          gxr[i] += go * w[i];
        if (gw)
          gw[i] += go * xr[i];
      }
    }
  }
}

// ── relu backward (elementwise over n): gx += gy * (x>0) ─────────────────────
inline void relu_bwd(const float *x, int n, const float *gy, float *gx) {
  for (int i = 0; i < n; ++i)
    gx[i] += (x[i] > 0.0f) ? gy[i] : 0.0f;
}

// ── layernorm over last dim d: y = (x-mu)/sqrt(var+eps) * g + b  (biased var) ──
inline void layernorm(const float *x, int rows, int d, const float *g, const float *b, float eps,
                      float *y) {
  for (int r = 0; r < rows; ++r) {
    const float *xr = x + static_cast<std::size_t>(r) * d;
    float *yr = y + static_cast<std::size_t>(r) * d;
    float mu = 0.0f;
    for (int i = 0; i < d; ++i)
      mu += xr[i];
    mu /= static_cast<float>(d);
    float var = 0.0f;
    for (int i = 0; i < d; ++i) {
      const float dv = xr[i] - mu;
      var += dv * dv;
    }
    var /= static_cast<float>(d);
    const float inv = 1.0f / std::sqrt(var + eps);
    for (int i = 0; i < d; ++i)
      yr[i] = (xr[i] - mu) * inv * g[i] + b[i];
  }
}
// VJP: gy[rows,d] -> gx[rows,d] (+=), gg[d] (+=), gb[d] (+=). Recomputes
// mu/inv/xhat. The normalized-Jacobian (both mean terms) — dropping either
// biases every gradient silently.
inline void layernorm_bwd(const float *x, int rows, int d, const float *g, float eps,
                          const float *gy, float *gx, float *gg, float *gb) {
  const float dd = static_cast<float>(d);
  std::vector<float> xhat(d), gxhat(d);
  for (int r = 0; r < rows; ++r) {
    const float *xr = x + static_cast<std::size_t>(r) * d;
    const float *gyr = gy + static_cast<std::size_t>(r) * d;
    float *gxr = gx ? gx + static_cast<std::size_t>(r) * d : nullptr;
    float mu = 0.0f;
    for (int i = 0; i < d; ++i)
      mu += xr[i];
    mu /= dd;
    float var = 0.0f;
    for (int i = 0; i < d; ++i) {
      const float dv = xr[i] - mu;
      var += dv * dv;
    }
    var /= dd;
    const float inv = 1.0f / std::sqrt(var + eps);
    float sum_gxhat = 0.0f, sum_gxhat_xhat = 0.0f;
    for (int i = 0; i < d; ++i) {
      xhat[i] = (xr[i] - mu) * inv;
      gxhat[i] = gyr[i] * g[i];
      if (gg)
        gg[i] += gyr[i] * xhat[i];
      if (gb)
        gb[i] += gyr[i];
      sum_gxhat += gxhat[i];
      sum_gxhat_xhat += gxhat[i] * xhat[i];
    }
    if (gxr)
      for (int i = 0; i < d; ++i)
        gxr[i] += inv * (gxhat[i] - sum_gxhat / dd - xhat[i] * sum_gxhat_xhat / dd);
  }
}

// ── masked multi-head self-attention ─────────────────────────────────────────
// Forward matches coef_energy_net.cpp::mha exactly (masked row-max softmax).
// qkv = linear(x, w_in[3d,d]); per head scores = scale*q.k, softmax over
// unpadded keys, ctx = p.v; out = linear(ctx, w_out[d,d]).
inline void mha(const float *x, int T, int d, int nhead, const std::uint8_t *pad, const float *w_in,
                const float *b_in, const float *w_out, const float *b_out, float *out) {
  const int hd = d / nhead;
  const float scale = 1.0f / std::sqrt(static_cast<float>(hd));
  std::vector<float> qkv(static_cast<std::size_t>(T) * 3 * d);
  linear(x, T, d, w_in, b_in, 3 * d, qkv.data());
  std::vector<float> ctx(static_cast<std::size_t>(T) * d, 0.0f);
  std::vector<float> scores(T);
  for (int h = 0; h < nhead; ++h) {
    const int qo = h * hd, ko = d + h * hd, vo = 2 * d + h * hd;
    for (int i = 0; i < T; ++i) {
      const float *qi = qkv.data() + static_cast<std::size_t>(i) * 3 * d + qo;
      float m = -std::numeric_limits<float>::infinity();
      for (int j = 0; j < T; ++j) {
        if (pad && pad[j]) {
          scores[j] = -std::numeric_limits<float>::infinity();
          continue;
        }
        const float *kj = qkv.data() + static_cast<std::size_t>(j) * 3 * d + ko;
        float s = 0.0f;
        for (int c = 0; c < hd; ++c)
          s += qi[c] * kj[c];
        s *= scale;
        scores[j] = s;
        if (s > m)
          m = s;
      }
      float denom = 0.0f;
      for (int j = 0; j < T; ++j) {
        if (pad && pad[j]) {
          scores[j] = 0.0f;
          continue;
        }
        const float e = std::exp(scores[j] - m);
        scores[j] = e;
        denom += e;
      }
      const float invd = 1.0f / denom;
      float *ci = ctx.data() + static_cast<std::size_t>(i) * d + h * hd;
      for (int j = 0; j < T; ++j) {
        if (pad && pad[j])
          continue;
        const float pconst = scores[j] * invd;
        const float *vj = qkv.data() + static_cast<std::size_t>(j) * 3 * d + vo;
        for (int c = 0; c < hd; ++c)
          ci[c] += pconst * vj[c];
      }
    }
  }
  linear(ctx.data(), T, d, w_out, b_out, d, out);
}

// VJP of mha: gout[T,d] -> gx[T,d] (+=) and weight grads
// gw_in[3d,d], gb_in[3d], gw_out[d,d], gb_out[d] (all +=). Recomputes qkv + the
// per-head softmax p (softmax-VJP dS = P (.) (g - (g.P)1)); padded keys carry
// exactly-zero grad (p=0 there). The 1/sqrt(hd) scale is folded into gscore.
inline void mha_bwd(const float *x, int T, int d, int nhead, const std::uint8_t *pad,
                    const float *w_in, const float *b_in, const float *w_out, const float *gout,
                    float *gx, float *gw_in, float *gb_in, float *gw_out, float *gb_out) {
  const int hd = d / nhead;
  const float scale = 1.0f / std::sqrt(static_cast<float>(hd));
  const std::size_t Td = static_cast<std::size_t>(T) * d;

  // recompute qkv and ctx (ctx needed for out_proj weight grad)
  std::vector<float> qkv(static_cast<std::size_t>(T) * 3 * d);
  linear(x, T, d, w_in, b_in, 3 * d, qkv.data());
  std::vector<float> ctx(Td, 0.0f);
  // store per (h,i) the softmax row p[i, :] to reuse in backward
  std::vector<float> Pw(static_cast<std::size_t>(nhead) * T * T, 0.0f);
  std::vector<float> scores(T);
  for (int h = 0; h < nhead; ++h) {
    const int qo = h * hd, ko = d + h * hd, vo = 2 * d + h * hd;
    for (int i = 0; i < T; ++i) {
      const float *qi = qkv.data() + static_cast<std::size_t>(i) * 3 * d + qo;
      float m = -std::numeric_limits<float>::infinity();
      for (int j = 0; j < T; ++j) {
        if (pad && pad[j]) {
          scores[j] = -std::numeric_limits<float>::infinity();
          continue;
        }
        const float *kj = qkv.data() + static_cast<std::size_t>(j) * 3 * d + ko;
        float s = 0.0f;
        for (int c = 0; c < hd; ++c)
          s += qi[c] * kj[c];
        s *= scale;
        scores[j] = s;
        if (s > m)
          m = s;
      }
      float denom = 0.0f;
      for (int j = 0; j < T; ++j) {
        if (pad && pad[j]) {
          scores[j] = 0.0f;
          continue;
        }
        const float e = std::exp(scores[j] - m);
        scores[j] = e;
        denom += e;
      }
      const float invd = 1.0f / denom;
      float *Pi = Pw.data() + (static_cast<std::size_t>(h) * T + i) * T;
      float *ci = ctx.data() + static_cast<std::size_t>(i) * d + h * hd;
      for (int j = 0; j < T; ++j) {
        if (pad && pad[j])
          continue;
        const float pval = scores[j] * invd;
        Pi[j] = pval;
        const float *vj = qkv.data() + static_cast<std::size_t>(j) * 3 * d + vo;
        for (int c = 0; c < hd; ++c)
          ci[c] += pval * vj[c];
      }
    }
  }

  // out = linear(ctx, w_out): gctx, gw_out, gb_out
  std::vector<float> gctx(Td, 0.0f);
  linear_bwd(ctx.data(), T, d, w_out, d, gout, gctx.data(), gw_out, gb_out);

  // gqkv accumulates q/k/v grads in the (T,3d) layout
  std::vector<float> gqkv(static_cast<std::size_t>(T) * 3 * d, 0.0f);
  std::vector<float> gp(T), gs(T);
  for (int h = 0; h < nhead; ++h) {
    const int qo = h * hd, ko = d + h * hd, vo = 2 * d + h * hd;
    for (int i = 0; i < T; ++i) {
      const float *Pi = Pw.data() + (static_cast<std::size_t>(h) * T + i) * T;
      const float *gci = gctx.data() + static_cast<std::size_t>(i) * d + h * hd;
      // ctx[i] = sum_j p[i,j] v[j]:  gp[i,j] = gci . v[j];  gv[j] += p[i,j] gci
      float gp_dot_p = 0.0f;
      for (int j = 0; j < T; ++j) {
        if (pad && pad[j]) {
          gp[j] = 0.0f;
          continue;
        }
        const float *vj = qkv.data() + static_cast<std::size_t>(j) * 3 * d + vo;
        float dotv = 0.0f;
        for (int c = 0; c < hd; ++c)
          dotv += gci[c] * vj[c];
        gp[j] = dotv;
        gp_dot_p += dotv * Pi[j];
        float *gvj = gqkv.data() + static_cast<std::size_t>(j) * 3 * d + vo;
        for (int c = 0; c < hd; ++c)
          gvj[c] += Pi[j] * gci[c];
      }
      // softmax VJP: gs[i,j] = p[i,j] (gp[i,j] - sum_k gp[i,k] p[i,k]); scale
      for (int j = 0; j < T; ++j) {
        if (pad && pad[j]) {
          gs[j] = 0.0f;
          continue;
        }
        gs[j] = Pi[j] * (gp[j] - gp_dot_p) * scale;
      }
      // scores[i,j] = q[i].k[j]:  gq[i] += sum_j gs[j] k[j];  gk[j] += gs[j] q[i]
      float *gqi = gqkv.data() + static_cast<std::size_t>(i) * 3 * d + qo;
      const float *qi = qkv.data() + static_cast<std::size_t>(i) * 3 * d + qo;
      for (int j = 0; j < T; ++j) {
        if (pad && pad[j])
          continue;
        const float g = gs[j];
        const float *kj = qkv.data() + static_cast<std::size_t>(j) * 3 * d + ko;
        float *gkj = gqkv.data() + static_cast<std::size_t>(j) * 3 * d + ko;
        for (int c = 0; c < hd; ++c) {
          gqi[c] += g * kj[c];
          gkj[c] += g * qi[c];
        }
      }
    }
  }
  // qkv = linear(x, w_in): gx, gw_in, gb_in
  linear_bwd(x, T, d, w_in, 3 * d, gqkv.data(), gx, gw_in, gb_in);
}

// ── conv2d (cross-correlation), stride+pad; out (Cout,Ho,Wo) ─────────────────
inline void conv2d(const float *x, int Cin, int H, int W, const float *w, const float *b, int Cout,
                   int kH, int kW, int stride, int pad, std::vector<float> &out, int &Ho, int &Wo) {
  const int Hp = H + 2 * pad, Wp = W + 2 * pad;
  Ho = (Hp - kH) / stride + 1;
  Wo = (Wp - kW) / stride + 1;
  out.assign(static_cast<std::size_t>(Cout) * Ho * Wo, 0.0f);
  for (int oc = 0; oc < Cout; ++oc) {
    for (int oy = 0; oy < Ho; ++oy) {
      for (int ox = 0; ox < Wo; ++ox) {
        float acc = b[oc];
        for (int ic = 0; ic < Cin; ++ic) {
          const float *xc = x + static_cast<std::size_t>(ic) * H * W;
          const float *wc = w + (static_cast<std::size_t>(oc) * Cin + ic) * kH * kW;
          for (int ky = 0; ky < kH; ++ky) {
            const int iy = oy * stride + ky - pad;
            if (iy < 0 || iy >= H)
              continue;
            for (int kx = 0; kx < kW; ++kx) {
              const int ix = ox * stride + kx - pad;
              if (ix < 0 || ix >= W)
                continue;
              acc += xc[iy * W + ix] * wc[ky * kW + kx];
            }
          }
        }
        out[(static_cast<std::size_t>(oc) * Ho + oy) * Wo + ox] = acc;
      }
    }
  }
}
// VJP: gout[Cout,Ho,Wo] -> gx[Cin,H,W] (+=), gw[Cout,Cin,kH,kW] (+=), gb[Cout] (+=).
inline void conv2d_bwd(const float *x, int Cin, int H, int W, const float *w, int Cout, int kH,
                       int kW, int stride, int pad, int Ho, int Wo, const float *gout, float *gx,
                       float *gw, float *gb) {
  for (int oc = 0; oc < Cout; ++oc) {
    for (int oy = 0; oy < Ho; ++oy) {
      for (int ox = 0; ox < Wo; ++ox) {
        const float go = gout[(static_cast<std::size_t>(oc) * Ho + oy) * Wo + ox];
        if (gb)
          gb[oc] += go;
        for (int ic = 0; ic < Cin; ++ic) {
          const float *xc = x + static_cast<std::size_t>(ic) * H * W;
          float *gxc = gx ? gx + static_cast<std::size_t>(ic) * H * W : nullptr;
          float *gwc = gw ? gw + (static_cast<std::size_t>(oc) * Cin + ic) * kH * kW : nullptr;
          for (int ky = 0; ky < kH; ++ky) {
            const int iy = oy * stride + ky - pad;
            if (iy < 0 || iy >= H)
              continue;
            for (int kx = 0; kx < kW; ++kx) {
              const int ix = ox * stride + kx - pad;
              if (ix < 0 || ix >= W)
                continue;
              if (gwc)
                gwc[ky * kW + kx] += go * xc[iy * W + ix];
              if (gxc)
                gxc[iy * W + ix] +=
                    go * w[(static_cast<std::size_t>(oc) * Cin + ic) * kH * kW + ky * kW + kx];
            }
          }
        }
      }
    }
  }
}

// ── adaptive average pool to (o,o) ───────────────────────────────────────────
inline void adaptive_avg_pool(const std::vector<float> &x, int C, int H, int W, int o,
                              std::vector<float> &out) {
  out.assign(static_cast<std::size_t>(C) * o * o, 0.0f);
  for (int c = 0; c < C; ++c) {
    const float *xc = x.data() + static_cast<std::size_t>(c) * H * W;
    for (int i = 0; i < o; ++i) {
      const int r0 = (i * H) / o, r1 = ((i + 1) * H + o - 1) / o;
      for (int j = 0; j < o; ++j) {
        const int c0 = (j * W) / o, c1 = ((j + 1) * W + o - 1) / o;
        float acc = 0.0f;
        int cnt = 0;
        for (int r = r0; r < r1; ++r)
          for (int cc = c0; cc < c1; ++cc) {
            acc += xc[r * W + cc];
            ++cnt;
          }
        out[(static_cast<std::size_t>(c) * o + i) * o + j] = acc / static_cast<float>(cnt);
      }
    }
  }
}
// VJP: gout[C,o,o] -> gx[C,H,W] (+=). Each bin spreads its grad / cnt.
inline void adaptive_avg_pool_bwd(int C, int H, int W, int o, const float *gout, float *gx) {
  for (int c = 0; c < C; ++c) {
    float *gxc = gx + static_cast<std::size_t>(c) * H * W;
    for (int i = 0; i < o; ++i) {
      const int r0 = (i * H) / o, r1 = ((i + 1) * H + o - 1) / o;
      for (int j = 0; j < o; ++j) {
        const int c0 = (j * W) / o, c1 = ((j + 1) * W + o - 1) / o;
        const int cnt = (r1 - r0) * (c1 - c0);
        const float g =
            gout[(static_cast<std::size_t>(c) * o + i) * o + j] / static_cast<float>(cnt);
        for (int r = r0; r < r1; ++r)
          for (int cc = c0; cc < c1; ++cc)
            gxc[r * W + cc] += g;
      }
    }
  }
}

} // namespace nn
} // namespace detail
} // namespace nav
} // namespace cvc

#endif
