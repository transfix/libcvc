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

// coef_energy_net.cpp — see coef_energy_net.h. Float-equivalent (rtol 1e-4)
// transcription of GRL-SNAM matnet_export.matnet_forward_numpy, itself the
// verified math-path reference for CoefEnergyNetMaterial. Ops in float32
// (deployment tier). Built with -ffp-contract=off (CMake). POST-norm
// transformer, ReLU FFN, no final encoder norm, masked row-max softmax — do
// not "tidy" the op order.

#include <cmath>
#include <cstring>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/detail/parallel.h>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace cvc {
namespace nav {

namespace {

inline float relu(float x) { return x > 0.0f ? x : 0.0f; }
inline float sigmoidf(float x) { return 1.0f / (1.0f + std::exp(-x)); }
inline float softplusf(float x) {
  return x > 20.0f ? x : std::log1p(std::exp(x < 20.0f ? x : 20.0f));
}

// y[o] = b[o] + sum_i x[i]*W[o*in + i]  (W row-major [out,in]); n rows of x.
void linear(const float *x, int rows, int in, const float *W, const float *b, int out, float *y) {
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

// LayerNorm over the last dim d, in place-ish (out may alias in). Biased var.
void layernorm(const float *x, int rows, int d, const float *g, const float *b, float eps,
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

// Multi-head self-attention over one sequence x (T,d). pad[j] nonzero = ignore
// key j. Writes out (T,d). Scratch qkv/scores provided by caller.
void mha(const float *x, int T, int d, int nhead, const std::uint8_t *pad, const float *w_in,
         const float *b_in, const float *w_out, const float *b_out, float *out) {
  const int hd = d / nhead;
  const float scale = 1.0f / std::sqrt(static_cast<float>(hd));
  std::vector<float> qkv(static_cast<std::size_t>(T) * 3 * d);
  linear(x, T, d, w_in, b_in, 3 * d, qkv.data()); // (T, 3d)
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
        const float p = scores[j] * invd;
        const float *vj = qkv.data() + static_cast<std::size_t>(j) * 3 * d + vo;
        for (int c = 0; c < hd; ++c)
          ci[c] += p * vj[c];
      }
    }
  }
  linear(ctx.data(), T, d, w_out, b_out, d, out); // out_proj
}

// Cross-correlation conv2d. x (Cin,H,W), w (Cout,Cin,kH,kW), out (Cout,Ho,Wo).
void conv2d(const float *x, int Cin, int H, int W, const float *w, const float *b, int Cout, int kH,
            int kW, int stride, int pad, std::vector<float> &out, int &Ho, int &Wo) {
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

// AdaptiveAvgPool2d to (o,o): bin i spans [floor(i*H/o), ceil((i+1)*H/o)).
void adaptive_avg_pool(const std::vector<float> &x, int C, int H, int W, int o,
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

std::uint32_t rd_u32(const unsigned char *&p) {
  std::uint32_t v;
  std::memcpy(&v, p, 4);
  p += 4;
  return v;
}
std::uint64_t rd_u64(const unsigned char *&p) {
  std::uint64_t v;
  std::memcpy(&v, p, 8);
  p += 8;
  return v;
}
float rd_f32(const unsigned char *&p) {
  float v;
  std::memcpy(&v, p, 4);
  p += 4;
  return v;
}

} // namespace

const coef_energy_net::tensor &coef_energy_net::t(const std::string &name) const {
  auto it = tensors_.find(name);
  if (it == tensors_.end())
    throw std::runtime_error("coef_energy_net: missing tensor '" + name + "'");
  return it->second;
}

coef_energy_net coef_energy_net::load_from_memory(const void *data, std::size_t nbytes) {
  const unsigned char *p = static_cast<const unsigned char *>(data);
  const unsigned char *end = p + nbytes;
  if (nbytes < 8 || std::memcmp(p, "CVNM", 4) != 0)
    throw std::runtime_error("coef_energy_net: bad magic (not a .cvcnm)");
  p += 4;
  coef_energy_net m;
  const std::uint32_t ver = rd_u32(p);
  if (ver != kFormatVersion)
    throw std::runtime_error("coef_energy_net: unsupported .cvcnm version");
  m.arch_hash_ = rd_u64(p);
  m.d_tok_ = static_cast<int>(rd_u32(p));
  m.nhead_ = static_cast<int>(rd_u32(p));
  m.num_layers_ = static_cast<int>(rd_u32(p));
  m.d_risk_ = static_cast<int>(rd_u32(p));
  m.patch_size_ = static_cast<int>(rd_u32(p));
  m.lam_soft_max_ = rd_f32(p);
  m.lam_hard_max_ = rd_f32(p);
  m.mu_lat_max_ = rd_f32(p);
  m.eps_ = rd_f32(p);
  const std::uint32_t n_tensors = rd_u32(p);
  for (std::uint32_t k = 0; k < n_tensors; ++k) {
    const std::uint32_t nl = rd_u32(p);
    if (p + nl > end)
      throw std::runtime_error("coef_energy_net: truncated .cvcnm (name)");
    std::string name(reinterpret_cast<const char *>(p), nl);
    p += nl;
    const std::uint32_t ndim = rd_u32(p);
    tensor tn;
    std::size_t count = 1;
    for (std::uint32_t di = 0; di < ndim; ++di) {
      const int dv = static_cast<int>(rd_u32(p));
      tn.dims.push_back(dv);
      count *= static_cast<std::size_t>(dv);
    }
    if (p + count * 4 > end)
      throw std::runtime_error("coef_energy_net: truncated .cvcnm (data)");
    tn.data.resize(count);
    std::memcpy(tn.data.data(), p, count * 4);
    p += count * 4;
    m.tensors_.emplace(std::move(name), std::move(tn));
  }
  return m;
}

coef_energy_net coef_energy_net::load(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("coef_energy_net: cannot open " + path);
  std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  return load_from_memory(buf.data(), buf.size());
}

void coef_energy_net::forward_one(const float *obs_feats, const std::uint8_t *obs_mask, int n_obs,
                                  const float *goal_feats, const float *risk_patch, int patch_p,
                                  float *alphas_out, float *beta, float *gamma, float *lam_soft,
                                  float *lam_hard, float *mu_lat) const {
  const int d = d_tok_;

  // goal token
  std::vector<float> zg1(128), zg(d);
  linear(goal_feats, 1, 4, t("goal_enc.0.weight").data.data(), t("goal_enc.0.bias").data.data(), 64,
         zg1.data());
  for (int i = 0; i < 64; ++i)
    zg1[i] = relu(zg1[i]);
  linear(zg1.data(), 1, 64, t("goal_enc.2.weight").data.data(), t("goal_enc.2.bias").data.data(), d,
         zg.data());

  const int T = 1 + n_obs;
  std::vector<float> tokens(static_cast<std::size_t>(T) * d);
  std::vector<std::uint8_t> pad(T, 0);
  std::memcpy(tokens.data(), zg.data(), sizeof(float) * d);
  if (n_obs > 0) {
    std::vector<float> zo1(static_cast<std::size_t>(n_obs) * 128);
    linear(obs_feats, n_obs, 6, t("obs_enc.0.weight").data.data(), t("obs_enc.0.bias").data.data(),
           128, zo1.data());
    for (auto &v : zo1)
      v = relu(v);
    linear(zo1.data(), n_obs, 128, t("obs_enc.2.weight").data.data(),
           t("obs_enc.2.bias").data.data(), d, tokens.data() + d);
    for (int i = 0; i < n_obs; ++i)
      pad[1 + i] = obs_mask[i] ? 0 : 1;
  }

  // transformer encoder (POST-norm, ReLU FFN, no final norm)
  std::vector<float> attn(static_cast<std::size_t>(T) * d);
  std::vector<float> ff1(static_cast<std::size_t>(T) * 128), ff2(static_cast<std::size_t>(T) * d);
  for (int L = 0; L < num_layers_; ++L) {
    const std::string pre = "fuser.layers." + std::to_string(L) + ".";
    mha(tokens.data(), T, d, nhead_, pad.data(), t(pre + "self_attn.in_proj_weight").data.data(),
        t(pre + "self_attn.in_proj_bias").data.data(),
        t(pre + "self_attn.out_proj.weight").data.data(),
        t(pre + "self_attn.out_proj.bias").data.data(), attn.data());
    for (std::size_t i = 0; i < tokens.size(); ++i)
      attn[i] += tokens[i]; // residual
    layernorm(attn.data(), T, d, t(pre + "norm1.weight").data.data(),
              t(pre + "norm1.bias").data.data(), eps_, tokens.data());
    linear(tokens.data(), T, d, t(pre + "linear1.weight").data.data(),
           t(pre + "linear1.bias").data.data(), 128, ff1.data());
    for (auto &v : ff1)
      v = relu(v);
    linear(ff1.data(), T, 128, t(pre + "linear2.weight").data.data(),
           t(pre + "linear2.bias").data.data(), d, ff2.data());
    for (std::size_t i = 0; i < tokens.size(); ++i)
      ff2[i] += tokens[i]; // residual
    layernorm(ff2.data(), T, d, t(pre + "norm2.weight").data.data(),
              t(pre + "norm2.bias").data.data(), eps_, tokens.data());
  }

  const float *ctx = tokens.data(); // goal context token (row 0)

  // alpha per obstacle
  if (n_obs > 0) {
    std::vector<float> a1(static_cast<std::size_t>(n_obs) * 64), a2(n_obs);
    linear(tokens.data() + d, n_obs, d, t("alpha_head.0.weight").data.data(),
           t("alpha_head.0.bias").data.data(), 64, a1.data());
    for (auto &v : a1)
      v = relu(v);
    linear(a1.data(), n_obs, 64, t("alpha_head.2.weight").data.data(),
           t("alpha_head.2.bias").data.data(), 1, a2.data());
    for (int i = 0; i < n_obs; ++i)
      alphas_out[i] = obs_mask[i] ? softplusf(a2[i]) : 0.0f;
  }

  auto head = [&](const std::string &prefix, const float *z, int zin) -> float {
    std::vector<float> h1(64), h2(1);
    linear(z, 1, zin, t(prefix + ".0.weight").data.data(), t(prefix + ".0.bias").data.data(), 64,
           h1.data());
    for (auto &v : h1)
      v = relu(v);
    linear(h1.data(), 1, 64, t(prefix + ".2.weight").data.data(), t(prefix + ".2.bias").data.data(),
           1, h2.data());
    return h2[0];
  };

  *beta = softplusf(head("beta_head", ctx, d));
  *gamma = softplusf(head("gamma_head", ctx, d));

  // risk CNN
  const int P = patch_p;
  std::vector<float> c0, c1, c2, pooled;
  int Ho, Wo;
  conv2d(risk_patch, 2, P, P, t("risk_enc.net.0.weight").data.data(),
         t("risk_enc.net.0.bias").data.data(), 16, 3, 3, 1, 1, c0, Ho, Wo);
  for (auto &v : c0)
    v = relu(v);
  conv2d(c0.data(), 16, Ho, Wo, t("risk_enc.net.2.weight").data.data(),
         t("risk_enc.net.2.bias").data.data(), 32, 3, 3, 2, 1, c1, Ho, Wo);
  for (auto &v : c1)
    v = relu(v);
  conv2d(c1.data(), 32, Ho, Wo, t("risk_enc.net.4.weight").data.data(),
         t("risk_enc.net.4.bias").data.data(), 64, 3, 3, 2, 1, c2, Ho, Wo);
  for (auto &v : c2)
    v = relu(v);
  adaptive_avg_pool(c2, 64, Ho, Wo, 4, pooled); // (64,4,4) row-major flatten -> 1024
  std::vector<float> risk_ctx(d_risk_);
  linear(pooled.data(), 1, 64 * 4 * 4, t("risk_enc.net.8.weight").data.data(),
         t("risk_enc.net.8.bias").data.data(), d_risk_, risk_ctx.data());
  for (auto &v : risk_ctx)
    v = relu(v);

  std::vector<float> mat_feats(static_cast<std::size_t>(d_risk_) + d);
  std::memcpy(mat_feats.data(), risk_ctx.data(), sizeof(float) * d_risk_);
  std::memcpy(mat_feats.data() + d_risk_, ctx, sizeof(float) * d);
  const int zin = d_risk_ + d;
  *lam_soft = lam_soft_max_ * sigmoidf(head("lam_soft_head", mat_feats.data(), zin));
  *lam_hard = lam_hard_max_ * sigmoidf(head("lam_hard_head", mat_feats.data(), zin));
  *mu_lat = mu_lat_max_ * sigmoidf(head("mu_lat_head", mat_feats.data(), zin));
}

void coef_energy_net::forward_batch(const float *obs_feats, const std::uint8_t *obs_mask,
                                    const int *obs_offsets, int n, const float *goal_feats,
                                    const float *risk_patch, int patch_p, float *alphas_out,
                                    float *beta, float *gamma, float *lam_soft, float *lam_hard,
                                    float *mu_lat, int num_threads) const {
  const int pp = patch_p * patch_p * 2;
  detail::parallel_for(n, num_threads, [&](int i) {
    const int o0 = obs_offsets[i], n_obs = obs_offsets[i + 1] - obs_offsets[i];
    forward_one(obs_feats + static_cast<std::size_t>(o0) * 6, obs_mask + o0, n_obs,
                goal_feats + static_cast<std::size_t>(i) * 4,
                risk_patch + static_cast<std::size_t>(i) * pp, patch_p, alphas_out + o0, beta + i,
                gamma + i, lam_soft + i, lam_hard + i, mu_lat + i);
  });
}

} // namespace nav
} // namespace cvc
