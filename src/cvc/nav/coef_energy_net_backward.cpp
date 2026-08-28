/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// coef_energy_net_backward.cpp — the torch-free reverse-mode adjoint of
// CoefEnergyNetMaterial (coef_energy_net.cpp is the forward). Produces the
// per-weight gradients the P5 training loop descends: seeded by upstream grads
// on the outputs (which integrate_surrogate_material_vjp supplies from the
// rollout), it recomputes the forward with cached activations then reverses,
// composing the detail/nn_ops.h op VJPs in the exact POST-norm transformer /
// ReLU-FFN / no-final-norm order of the forward. Every op VJP and the whole
// composition are finite-difference gradchecked (nav_coef_energy_grad_test) —
// the torch-independent ground truth. Built with -ffp-contract=off (CMake).

#include <cmath>
#include <cstring>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/detail/nn_ops.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

using namespace detail::nn;

coef_energy_net::param_grads coef_energy_net::zero_grads() const {
  param_grads g;
  for (const auto &kv : tensors_)
    g[kv.first] = std::vector<float>(kv.second.data.size(), 0.0f);
  return g;
}

std::vector<std::string> coef_energy_net::param_names() const {
  std::vector<std::string> names;
  names.reserve(tensors_.size());
  for (const auto &kv : tensors_)
    names.push_back(kv.first);
  return names;
}

const std::vector<float> &coef_energy_net::param(const std::string &name) const {
  return t(name).data;
}

std::vector<float> &coef_energy_net::mutable_param(const std::string &name) {
  auto it = tensors_.find(name);
  if (it == tensors_.end())
    throw std::runtime_error("coef_energy_net::mutable_param: missing '" + name + "'");
  return it->second.data;
}

void coef_energy_net::backward_one(const float *obs_feats, const std::uint8_t *obs_mask, int n_obs,
                                   const float *goal_feats, const float *risk_patch, int patch_p,
                                   const float *g_alphas, float g_beta, float g_gamma,
                                   float g_lam_soft, float g_lam_hard, float g_mu_lat,
                                   param_grads &grads) const {
  const int d = d_tok_;
  const int T = 1 + n_obs;
  auto W = [&](const std::string &n) { return t(n).data.data(); };
  auto G = [&](const std::string &n) { return grads.at(n).data(); };

  // ═══════════════ forward (cache activations) ═══════════════
  // goal token
  std::vector<float> zg1(64), zg1r(64), zg(d);
  linear(goal_feats, 1, 4, W("goal_enc.0.weight"), W("goal_enc.0.bias"), 64, zg1.data());
  for (int i = 0; i < 64; ++i)
    zg1r[i] = relu(zg1[i]);
  linear(zg1r.data(), 1, 64, W("goal_enc.2.weight"), W("goal_enc.2.bias"), d, zg.data());

  std::vector<float> zo1, zo1r;
  std::vector<float> tokens0(static_cast<std::size_t>(T) * d);
  std::vector<std::uint8_t> pad(T, 0);
  std::memcpy(tokens0.data(), zg.data(), sizeof(float) * d);
  if (n_obs > 0) {
    zo1.resize(static_cast<std::size_t>(n_obs) * 128);
    zo1r.resize(static_cast<std::size_t>(n_obs) * 128);
    linear(obs_feats, n_obs, 6, W("obs_enc.0.weight"), W("obs_enc.0.bias"), 128, zo1.data());
    for (std::size_t i = 0; i < zo1.size(); ++i)
      zo1r[i] = relu(zo1[i]);
    linear(zo1r.data(), n_obs, 128, W("obs_enc.2.weight"), W("obs_enc.2.bias"), d,
           tokens0.data() + d);
    for (int i = 0; i < n_obs; ++i)
      pad[1 + i] = obs_mask[i] ? 0 : 1;
  }

  // transformer (POST-norm), cache per layer
  const int L = num_layers_;
  std::vector<std::vector<float>> tin(L), attn(L), attn_res(L), ln1(L), ff1(L), ff1r(L), ff2(L),
      ff2_res(L), ln2(L);
  std::vector<float> cur = tokens0;
  for (int l = 0; l < L; ++l) {
    const std::string pre = "fuser.layers." + std::to_string(l) + ".";
    tin[l] = cur;
    attn[l].assign(static_cast<std::size_t>(T) * d, 0.0f);
    mha(cur.data(), T, d, nhead_, pad.data(), W(pre + "self_attn.in_proj_weight"),
        W(pre + "self_attn.in_proj_bias"), W(pre + "self_attn.out_proj.weight"),
        W(pre + "self_attn.out_proj.bias"), attn[l].data());
    attn_res[l] = attn[l];
    for (std::size_t i = 0; i < attn_res[l].size(); ++i)
      attn_res[l][i] += tin[l][i];
    ln1[l].assign(static_cast<std::size_t>(T) * d, 0.0f);
    layernorm(attn_res[l].data(), T, d, W(pre + "norm1.weight"), W(pre + "norm1.bias"), eps_,
              ln1[l].data());
    ff1[l].assign(static_cast<std::size_t>(T) * 128, 0.0f);
    linear(ln1[l].data(), T, d, W(pre + "linear1.weight"), W(pre + "linear1.bias"), 128,
           ff1[l].data());
    ff1r[l] = ff1[l];
    for (std::size_t i = 0; i < ff1r[l].size(); ++i)
      ff1r[l][i] = relu(ff1r[l][i]);
    ff2[l].assign(static_cast<std::size_t>(T) * d, 0.0f);
    linear(ff1r[l].data(), T, 128, W(pre + "linear2.weight"), W(pre + "linear2.bias"), d,
           ff2[l].data());
    ff2_res[l] = ff2[l];
    for (std::size_t i = 0; i < ff2_res[l].size(); ++i)
      ff2_res[l][i] += ln1[l][i];
    ln2[l].assign(static_cast<std::size_t>(T) * d, 0.0f);
    layernorm(ff2_res[l].data(), T, d, W(pre + "norm2.weight"), W(pre + "norm2.bias"), eps_,
              ln2[l].data());
    cur = ln2[l];
  }
  const std::vector<float> &tokensF = cur; // final tokens; ctx = row 0

  // risk CNN
  const int P = patch_p;
  std::vector<float> c0, c1, c2, pooled;
  int Ho0, Wo0, Ho1, Wo1, Ho2, Wo2;
  conv2d(risk_patch, 2, P, P, W("risk_enc.net.0.weight"), W("risk_enc.net.0.bias"), 16, 3, 3, 1, 1,
         c0, Ho0, Wo0);
  std::vector<float> c0r = c0;
  for (auto &v : c0r)
    v = relu(v);
  conv2d(c0r.data(), 16, Ho0, Wo0, W("risk_enc.net.2.weight"), W("risk_enc.net.2.bias"), 32, 3, 3,
         2, 1, c1, Ho1, Wo1);
  std::vector<float> c1r = c1;
  for (auto &v : c1r)
    v = relu(v);
  conv2d(c1r.data(), 32, Ho1, Wo1, W("risk_enc.net.4.weight"), W("risk_enc.net.4.bias"), 64, 3, 3,
         2, 1, c2, Ho2, Wo2);
  std::vector<float> c2r = c2;
  for (auto &v : c2r)
    v = relu(v);
  adaptive_avg_pool(c2r, 64, Ho2, Wo2, 4, pooled);
  std::vector<float> risk1(d_risk_), risk1r(d_risk_);
  linear(pooled.data(), 1, 64 * 4 * 4, W("risk_enc.net.8.weight"), W("risk_enc.net.8.bias"),
         d_risk_, risk1.data());
  for (int i = 0; i < d_risk_; ++i)
    risk1r[i] = relu(risk1[i]);

  // ═══════════════ reverse ═══════════════
  // A 2-linear head with ReLU: z(zin) -> Linear.0 -> ReLU -> Linear.2 -> h2(1),
  // then an activation. Recompute + backprop; g_z accumulates.
  enum act_kind { SOFTPLUS, SIGMOID_CAP };
  auto head_bwd = [&](const std::string &pfx, const float *z, int zin, float g_after, act_kind act,
                      float cap, float *g_z) {
    std::vector<float> h1(64), h1r(64), h2(1);
    linear(z, 1, zin, W(pfx + ".0.weight"), W(pfx + ".0.bias"), 64, h1.data());
    for (int i = 0; i < 64; ++i)
      h1r[i] = relu(h1[i]);
    linear(h1r.data(), 1, 64, W(pfx + ".2.weight"), W(pfx + ".2.bias"), 1, h2.data());
    float g_h2;
    if (act == SOFTPLUS) {
      g_h2 = g_after * sigmoidf(h2[0]);
    } else {
      const float s = sigmoidf(h2[0]);
      g_h2 = g_after * cap * s * (1.0f - s);
    }
    std::vector<float> g_h1r(64, 0.0f), g_h1(64, 0.0f);
    linear_bwd(h1r.data(), 1, 64, W(pfx + ".2.weight"), 1, &g_h2, g_h1r.data(),
               G(pfx + ".2.weight"), G(pfx + ".2.bias"));
    relu_bwd(h1.data(), 64, g_h1r.data(), g_h1.data());
    linear_bwd(z, 1, zin, W(pfx + ".0.weight"), 64, g_h1.data(), g_z, G(pfx + ".0.weight"),
               G(pfx + ".0.bias"));
  };

  // material heads: mat_feats = [risk1r(d_risk) ; ctx(d)]
  const int zin_mat = d_risk_ + d;
  std::vector<float> mat_feats(zin_mat), g_matfeats(zin_mat, 0.0f);
  std::memcpy(mat_feats.data(), risk1r.data(), sizeof(float) * d_risk_);
  std::memcpy(mat_feats.data() + d_risk_, tokensF.data(), sizeof(float) * d);
  head_bwd("lam_soft_head", mat_feats.data(), zin_mat, g_lam_soft, SIGMOID_CAP, lam_soft_max_,
           g_matfeats.data());
  head_bwd("lam_hard_head", mat_feats.data(), zin_mat, g_lam_hard, SIGMOID_CAP, lam_hard_max_,
           g_matfeats.data());
  head_bwd("mu_lat_head", mat_feats.data(), zin_mat, g_mu_lat, SIGMOID_CAP, mu_lat_max_,
           g_matfeats.data());

  std::vector<float> g_risk1r(g_matfeats.begin(), g_matfeats.begin() + d_risk_);
  std::vector<float> g_ctx(g_matfeats.begin() + d_risk_, g_matfeats.end()); // grad on tokensF[0]

  // beta / gamma heads on ctx (accumulate into g_ctx)
  head_bwd("beta_head", tokensF.data(), d, g_beta, SOFTPLUS, 0.0f, g_ctx.data());
  head_bwd("gamma_head", tokensF.data(), d, g_gamma, SOFTPLUS, 0.0f, g_ctx.data());

  // grad on the final tokens: row 0 = g_ctx; rows 1.. from alpha_head
  std::vector<float> g_tokensF(static_cast<std::size_t>(T) * d, 0.0f);
  std::memcpy(g_tokensF.data(), g_ctx.data(), sizeof(float) * d);
  if (n_obs > 0) {
    // forward alpha: a1 = Linear.0(tokens[1:]); a1r = relu; a2 = Linear.2(a1r)
    std::vector<float> a1(static_cast<std::size_t>(n_obs) * 64), a1r, a2(n_obs);
    linear(tokensF.data() + d, n_obs, d, W("alpha_head.0.weight"), W("alpha_head.0.bias"), 64,
           a1.data());
    a1r = a1;
    for (auto &v : a1r)
      v = relu(v);
    linear(a1r.data(), n_obs, 64, W("alpha_head.2.weight"), W("alpha_head.2.bias"), 1, a2.data());
    // alphas_out[i] = mask? softplus(a2[i]) : 0
    std::vector<float> g_a2(n_obs, 0.0f);
    for (int i = 0; i < n_obs; ++i)
      g_a2[i] = obs_mask[i] ? g_alphas[i] * sigmoidf(a2[i]) : 0.0f;
    std::vector<float> g_a1r(static_cast<std::size_t>(n_obs) * 64, 0.0f),
        g_a1(static_cast<std::size_t>(n_obs) * 64, 0.0f);
    linear_bwd(a1r.data(), n_obs, 64, W("alpha_head.2.weight"), 1, g_a2.data(), g_a1r.data(),
               G("alpha_head.2.weight"), G("alpha_head.2.bias"));
    relu_bwd(a1.data(), n_obs * 64, g_a1r.data(), g_a1.data());
    linear_bwd(tokensF.data() + d, n_obs, d, W("alpha_head.0.weight"), 64, g_a1.data(),
               g_tokensF.data() + d, G("alpha_head.0.weight"), G("alpha_head.0.bias"));
  }

  // reverse transformer
  std::vector<float> g_cur = g_tokensF;
  for (int l = L - 1; l >= 0; --l) {
    const std::string pre = "fuser.layers." + std::to_string(l) + ".";
    // ln2: layernorm(ff2_res) -> g_ff2_res
    std::vector<float> g_ff2_res(static_cast<std::size_t>(T) * d, 0.0f);
    layernorm_bwd(ff2_res[l].data(), T, d, W(pre + "norm2.weight"), eps_, g_cur.data(),
                  g_ff2_res.data(), G(pre + "norm2.weight"), G(pre + "norm2.bias"));
    // ff2_res = ff2 + ln1
    std::vector<float> g_ff2 = g_ff2_res;
    std::vector<float> g_ln1 = g_ff2_res; // residual copy; linear1_bwd will add more
    // ff2 = linear2(ff1r)
    std::vector<float> g_ff1r(static_cast<std::size_t>(T) * 128, 0.0f);
    linear_bwd(ff1r[l].data(), T, 128, W(pre + "linear2.weight"), d, g_ff2.data(), g_ff1r.data(),
               G(pre + "linear2.weight"), G(pre + "linear2.bias"));
    // ff1r = relu(ff1)
    std::vector<float> g_ff1(static_cast<std::size_t>(T) * 128, 0.0f);
    relu_bwd(ff1[l].data(), T * 128, g_ff1r.data(), g_ff1.data());
    // ff1 = linear1(ln1)  (adds into g_ln1)
    linear_bwd(ln1[l].data(), T, d, W(pre + "linear1.weight"), 128, g_ff1.data(), g_ln1.data(),
               G(pre + "linear1.weight"), G(pre + "linear1.bias"));
    // ln1 = layernorm(attn_res) -> g_attn_res
    std::vector<float> g_attn_res(static_cast<std::size_t>(T) * d, 0.0f);
    layernorm_bwd(attn_res[l].data(), T, d, W(pre + "norm1.weight"), eps_, g_ln1.data(),
                  g_attn_res.data(), G(pre + "norm1.weight"), G(pre + "norm1.bias"));
    // attn_res = attn + tin
    std::vector<float> g_attn = g_attn_res;
    std::vector<float> g_tin = g_attn_res; // residual copy; mha_bwd will add more
    // attn = mha(tin)
    mha_bwd(tin[l].data(), T, d, nhead_, pad.data(), W(pre + "self_attn.in_proj_weight"),
            W(pre + "self_attn.in_proj_bias"), W(pre + "self_attn.out_proj.weight"), g_attn.data(),
            g_tin.data(), G(pre + "self_attn.in_proj_weight"), G(pre + "self_attn.in_proj_bias"),
            G(pre + "self_attn.out_proj.weight"), G(pre + "self_attn.out_proj.bias"));
    g_cur = g_tin;
  }

  // g_cur = grad on the initial tokens: row 0 -> goal_enc, rows 1.. -> obs_enc
  std::vector<float> g_zg1r(64, 0.0f), g_zg1(64, 0.0f);
  {
    std::vector<float> g_zg(d, 0.0f);
    std::memcpy(g_zg.data(), g_cur.data(), sizeof(float) * d);
    linear_bwd(zg1r.data(), 1, 64, W("goal_enc.2.weight"), d, g_zg.data(), g_zg1r.data(),
               G("goal_enc.2.weight"), G("goal_enc.2.bias"));
    relu_bwd(zg1.data(), 64, g_zg1r.data(), g_zg1.data());
    std::vector<float> g_goal_feats(4, 0.0f); // data (discarded)
    linear_bwd(goal_feats, 1, 4, W("goal_enc.0.weight"), 64, g_zg1.data(), g_goal_feats.data(),
               G("goal_enc.0.weight"), G("goal_enc.0.bias"));
  }
  if (n_obs > 0) {
    std::vector<float> g_zo1r(static_cast<std::size_t>(n_obs) * 128, 0.0f),
        g_zo1(static_cast<std::size_t>(n_obs) * 128, 0.0f);
    linear_bwd(zo1r.data(), n_obs, 128, W("obs_enc.2.weight"), d, g_cur.data() + d, g_zo1r.data(),
               G("obs_enc.2.weight"), G("obs_enc.2.bias"));
    relu_bwd(zo1.data(), n_obs * 128, g_zo1r.data(), g_zo1.data());
    std::vector<float> g_obs_feats(static_cast<std::size_t>(n_obs) * 6, 0.0f); // data (discarded)
    linear_bwd(obs_feats, n_obs, 6, W("obs_enc.0.weight"), 128, g_zo1.data(), g_obs_feats.data(),
               G("obs_enc.0.weight"), G("obs_enc.0.bias"));
  }

  // risk CNN backward: g_risk1r -> relu -> Linear.8 -> pool -> conv.4 -> conv.2 -> conv.0
  std::vector<float> g_risk1(d_risk_, 0.0f);
  relu_bwd(risk1.data(), d_risk_, g_risk1r.data(), g_risk1.data());
  std::vector<float> g_pooled(static_cast<std::size_t>(64) * 4 * 4, 0.0f);
  linear_bwd(pooled.data(), 1, 64 * 4 * 4, W("risk_enc.net.8.weight"), d_risk_, g_risk1.data(),
             g_pooled.data(), G("risk_enc.net.8.weight"), G("risk_enc.net.8.bias"));
  std::vector<float> g_c2r(static_cast<std::size_t>(64) * Ho2 * Wo2, 0.0f);
  adaptive_avg_pool_bwd(64, Ho2, Wo2, 4, g_pooled.data(), g_c2r.data());
  std::vector<float> g_c2(g_c2r.size(), 0.0f);
  relu_bwd(c2.data(), static_cast<int>(c2.size()), g_c2r.data(), g_c2.data());
  std::vector<float> g_c1r(static_cast<std::size_t>(32) * Ho1 * Wo1, 0.0f);
  conv2d_bwd(c1r.data(), 32, Ho1, Wo1, W("risk_enc.net.4.weight"), 64, 3, 3, 2, 1, Ho2, Wo2,
             g_c2.data(), g_c1r.data(), G("risk_enc.net.4.weight"), G("risk_enc.net.4.bias"));
  std::vector<float> g_c1(g_c1r.size(), 0.0f);
  relu_bwd(c1.data(), static_cast<int>(c1.size()), g_c1r.data(), g_c1.data());
  std::vector<float> g_c0r(static_cast<std::size_t>(16) * Ho0 * Wo0, 0.0f);
  conv2d_bwd(c0r.data(), 16, Ho0, Wo0, W("risk_enc.net.2.weight"), 32, 3, 3, 2, 1, Ho1, Wo1,
             g_c1.data(), g_c0r.data(), G("risk_enc.net.2.weight"), G("risk_enc.net.2.bias"));
  std::vector<float> g_c0(g_c0r.size(), 0.0f);
  relu_bwd(c0.data(), static_cast<int>(c0.size()), g_c0r.data(), g_c0.data());
  std::vector<float> g_patch(static_cast<std::size_t>(2) * P * P, 0.0f); // data (discarded)
  conv2d_bwd(risk_patch, 2, P, P, W("risk_enc.net.0.weight"), 16, 3, 3, 1, 1, Ho0, Wo0, g_c0.data(),
             g_patch.data(), G("risk_enc.net.0.weight"), G("risk_enc.net.0.bias"));
}

} // namespace nav
} // namespace cvc
