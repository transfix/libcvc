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

// coef_energy_net.h — torch-free forward of GRL-SNAM's learned material
// coefficient network `CoefEnergyNetMaterial`: a transformer over
// obstacle/goal tokens + a CNN risk-patch encoder that predicts the per-tick
// navigation coefficients (alpha per obstacle, beta, gamma, lam_soft,
// lam_hard, mu_lat) from context. This is the learned producer of the
// lam_soft/lam_hard that material.h's drive consumes as inputs, and the
// inference half of the training loop (see coef_train's material twin).
//
// The reference / weight format is `.cvcnm` (magic "CVNM"), written by
// GRL-SNAM grl_snam/tools/matnet_export.py. `.cvcnav` (a Linear chain) cannot
// express this net's conv/attention/DAG topology, so this is a sibling format.
//
// Parity tier: FLOAT-equivalent (rtol 1e-4) against the numpy math-path
// reference `matnet_forward_numpy` — NOT bit-identical (attention/GEMM
// reduction-order freedom). The numpy reference is the oracle; torch's fused
// attention fast path is explicitly out of the contract (it skips padded
// tokens and rounds differently). This TU builds with -ffp-contract=off.
//
// The transformer is POST-norm with a ReLU FFN and no final encoder norm
// (torch TransformerEncoderLayer defaults, not overridden in the source) —
// these are hardcoded here; getting pre-vs-post norm wrong silently corrupts
// every output.

#ifndef CVC_NAV_COEF_ENERGY_NET_H
#define CVC_NAV_COEF_ENERGY_NET_H

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

class coef_energy_net {
public:
  static constexpr std::uint32_t kFormatVersion = 1;

  // Load a .cvcnm from disk / memory. Throws std::runtime_error on bad
  // magic / version / truncation / missing tensor.
  static coef_energy_net load(const std::string &path);
  static coef_energy_net load_from_memory(const void *data, std::size_t nbytes);

  // Per-agent forward. Inputs for one agent:
  //   obs_feats  [n_obs*6]   per-obstacle [cx,cy,R_eff,W,goal_x-cx,goal_y-cy]
  //   obs_mask   [n_obs]     nonzero = valid obstacle (n_obs may be 0)
  //   goal_feats [4]         [dx,dy,dist,1.0]
  //   risk_patch [2*P*P]     ch0 = smoothed risk, ch1 = hard mask (P = patch_size)
  // Outputs:
  //   alphas_out [n_obs]     per-obstacle barrier weight (0 for masked)
  //   *beta,*gamma           goal spring / damping
  //   *lam_soft,*lam_hard    material soft/hard weights (already capped)
  //   *mu_lat                highway lateral weight (unused off-highway)
  void forward_one(const float *obs_feats, const std::uint8_t *obs_mask, int n_obs,
                   const float *goal_feats, const float *risk_patch, int patch_p, float *alphas_out,
                   float *beta, float *gamma, float *lam_soft, float *lam_hard,
                   float *mu_lat) const;

  // Batched over n agents with ragged obstacle lists. obs_offsets[n+1] indexes
  // into obs_feats [total*6] / obs_mask [total] / alphas_out [total];
  // goal_feats [n*4]; risk_patch [n*2*P*P]; beta/.../mu_lat [n]. Agents are
  // independent — fanned across num_threads. patch_p = P.
  void forward_batch(const float *obs_feats, const std::uint8_t *obs_mask, const int *obs_offsets,
                     int n, const float *goal_feats, const float *risk_patch, int patch_p,
                     float *alphas_out, float *beta, float *gamma, float *lam_soft, float *lam_hard,
                     float *mu_lat, int num_threads = 0) const;

  int patch_size() const { return patch_size_; }
  std::uint64_t arch_hash() const { return arch_hash_; }
  float lam_soft_max() const { return lam_soft_max_; }
  float lam_hard_max() const { return lam_hard_max_; }
  float mu_lat_max() const { return mu_lat_max_; }

  // ── P5 training backward (torch-free) ──────────────────────────────────────
  // Per-weight gradients, keyed by the same names as the loaded tensors.
  using param_grads = std::map<std::string, std::vector<float>>;

  // A param_grads with every weight tensor present and zeroed — the accumulator
  // backward_one adds into.
  param_grads zero_grads() const;

  // Reverse-mode adjoint of forward_one. Given upstream grads on the outputs
  // (g_alphas[n_obs], and the scalar coefficient grads), ADDS the weight
  // gradients into `grads` (obs_feats/goal_feats/risk_patch are data — no input
  // grad is returned). Recomputes the forward internally (caches activations,
  // then one reverse pass), composing the detail/nn_ops.h op VJPs in the
  // POST-norm transformer order. Validated by nav_coef_energy_grad_test (per-op
  // FD gradchecks + an end-to-end model gradcheck). mu_lat is unused in training
  // (pass 0). Thread-safe over agents (const; writes only into `grads`).
  void backward_one(const float *obs_feats, const std::uint8_t *obs_mask, int n_obs,
                    const float *goal_feats, const float *risk_patch, int patch_p,
                    const float *g_alphas, float g_beta, float g_gamma, float g_lam_soft,
                    float g_lam_hard, float g_mu_lat, param_grads &grads) const;

  // Weight access for the optimizer (P5 training) and the gradcheck. Names come
  // from param_names(); throws if a name is unknown.
  std::vector<std::string> param_names() const;
  const std::vector<float> &param(const std::string &name) const;
  std::vector<float> &mutable_param(const std::string &name);

  // Serialize to the .cvcnm container — the exact byte layout load_from_memory
  // reads and GRL-SNAM matnet_export.py writes (magic "CVNM", the hyperparams,
  // then name-keyed f32 tensors). The arch_hash is preserved (the architecture
  // is unchanged by training). Round-trips: load_from_memory(serialize()) has
  // byte-identical weights. Used to checkpoint a C++-trained policy.
  std::vector<unsigned char> serialize() const;
  void save(const std::string &path) const;

  // ── CUDA twin (defined in coef_energy_net.cu; CVC_ENABLE_CUDA only) ──────────
  // Batched device forward, float-equivalent to forward_batch (same ragged
  // obs_offsets layout: obs_feats [total*6] / obs_mask [total] / alphas_out
  // [total]; goal_feats [n*4]; risk_patch [n*2*P*P]; beta/.../mu_lat [n]). One
  // CUDA block per agent, d_tok threads cooperating; the CNN activations live in
  // per-agent device scratch. Validated vs the CPU forward by
  // nav_coef_energy_cuda_test (FLOAT tier). Throws if built without CUDA or no
  // device is present (guard with coef_energy_cuda_available()).
  void forward_batch_cuda(const float *obs_feats, const std::uint8_t *obs_mask,
                          const int *obs_offsets, int n, const float *goal_feats,
                          const float *risk_patch, int patch_p, float *alphas_out, float *beta,
                          float *gamma, float *lam_soft, float *lam_hard, float *mu_lat) const;

  // Batched device backward, the CUDA twin of backward_one accumulated over the
  // ragged batch: one block per agent recomputes the forward (activations to
  // per-agent device scratch) then reverses, atomicAdd-ing every agent's weight
  // gradients into the SAME `grads` accumulator (add into it; zero it first for a
  // fresh gradient). Upstream grads: g_alphas [total] (obs_offsets layout),
  // g_beta/.../g_mu_lat [n]. FLOAT tier vs CPU backward_one (rel<5e-3 / cos>0.9999
  // — the atomicAdd sum order differs from the CPU's sequential accumulate);
  // validated by nav_coef_energy_cuda_test. Throws without a CUDA device.
  void backward_batch_cuda(const float *obs_feats, const std::uint8_t *obs_mask,
                           const int *obs_offsets, int n, const float *goal_feats,
                           const float *risk_patch, int patch_p, const float *g_alphas,
                           const float *g_beta, const float *g_gamma, const float *g_lam_soft,
                           const float *g_lam_hard, const float *g_mu_lat,
                           param_grads &grads) const;

private:
  struct tensor {
    std::vector<int> dims;
    std::vector<float> data;
    int dim(int i) const { return dims[i]; }
    std::size_t size() const { return data.size(); }
  };
  const tensor &t(const std::string &name) const;

  std::map<std::string, tensor> tensors_;
  int d_tok_ = 64, nhead_ = 4, num_layers_ = 2, d_risk_ = 64, patch_size_ = 32;
  float lam_soft_max_ = 5.0f, lam_hard_max_ = 10.0f, mu_lat_max_ = 5.0f, eps_ = 1e-5f;
  std::uint64_t arch_hash_ = 0;
};

// True when this build has CUDA AND a device is present (so forward_batch_cuda
// will run). Mirrors material_rollout_cuda_available().
bool coef_energy_cuda_available();

} // namespace nav
} // namespace cvc

#endif
