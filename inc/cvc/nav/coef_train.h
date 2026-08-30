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

// coef_train.h — the SELF-SUPERVISED CoefMLP trainer, in pure C++, NO libtorch
// and NO Python (the training twin of the inference port). This is the port of
// grl_snam/tools/coef_train.py: there is no dataset and no labels — the gradient
// comes straight from a differentiable rollout over a scene's SDF ("did the
// agent reach its goal without hitting a wall") back into the coefficient net.
//
// The training rollout is the SIMPLE point-mass surrogate `sdf_rollout`
// (sdf_nav.py) — sample -> IPC wall force + goal spring + damping -> integrate —
// NOT the branch-heavy bicycle used at deployment, so the whole computation
// graph is small and smooth. Reverse-mode gradients are hand-written adjoints
// (the bilinear-sample position VJP, the MLP backward, the IPC-barrier
// derivative, and the rollout chain) and validated by a finite-difference
// gradcheck (nav_coef_train_test) — a torch-independent ground truth, so
// correctness never depends on matching torch's autograd bit-for-bit (it only
// has to descend the loss). Truncated BPTT (detach every `window` steps) bounds
// the graph, exactly as the Python trainer does.
//
// Trained weights export to the same versioned `.cvcnav` (coef_mlp::save) that
// torch, the CPU forward, and the CUDA forward all read — so a policy can be
// (re)trained on the deployment box with zero torch, then dropped into the
// pure-C++ swarm (sim_world / sim_world_cuda).

#ifndef __CVC_NAV_COEF_TRAIN_H__
#define __CVC_NAV_COEF_TRAIN_H__

#include <cstdint>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/drive.h> // field_stack
#include <vector>

namespace cvc {
namespace nav {

// A self-supervised training scene: a static occupancy world + the vehicle /
// integration meta the differentiable rollout runs under, plus reachable
// start/goal sampling. The SDF field is built once (build_sdf, no clip — matching
// the training reference in coef_train.py) and sampled by the rollout; starts and
// goals are drawn from the largest 8-connected free component so a goal is always
// reachable from its start. Build one with city_scene() (the Python "city" story)
// or occupancy_scene() (any rasterized scene, e.g. an lsystem_forest).
struct training_scene {
  std::vector<std::uint8_t> occ; // rows*cols row-major (0 = free, != 0 = obstacle)
  int rows = 0, cols = 0;
  double min_x = 0, min_y = 0, max_x = 0, max_y = 0, cx = 0, cy = 0, scale = 1.0;
  float rr = 0.15f, d_hat = 0.35f, dt = 0.06f, vmax = 0.9f;

  std::vector<float> field_data; // [3*rows*cols] built SDF (phi, nx, ny), no clip
  std::vector<int> free_cells;   // cells of the largest 8-connected free component

  field_stack field() const; // borrows field_data
  // n normalized (centered) start/goal pairs from the largest free component
  // (seed-deterministic). Writes o[n*2] and goal[n*2].
  void sample_starts_goals(int n, unsigned seed, float *o, float *goal) const;
  // Build field_data + free_cells from occ (called by the factories).
  void build();
};

// The Python STORIES["city"] training scene shrunk to a grid×grid raster
// (city_blocks(96, rows=3, cols=3, gap=9, margin=14) rects scaled by grid/96,
// rasterized), with the city story's meta (scale 0.05, bounds ±100, rr 0.15,
// d_hat 0.35, dt 0.06, vmax 0.9). This is the SAME scene coef_train.py trains on.
training_scene city_scene(int grid = 96);

// An arbitrary caller-provided occupancy scene (0 = free). `occ` is copied. Use
// this to train directly on the map you deploy into (e.g. a rasterized
// lsystem_forest terrain) rather than the Python city.
training_scene occupancy_scene(const std::uint8_t *occ, int rows, int cols, double min_x,
                               double min_y, double max_x, double max_y, double scale,
                               float rr = 0.15f, float d_hat = 0.35f, float dt = 0.06f,
                               float vmax = 0.9f);

// Which differentiable integrator the training rollout backprops through.
//   surrogate — the smooth point-mass sdf_rollout (coef_train.py's choice); clean
//               gradient, the default. State is (o, v).
//   bicycle   — the FULL deployment kinematic-bicycle integrator, differentiated;
//               closes the surrogate->deployment gap but has non-smooth governor
//               branches. State is (o, th, sp).
// Both share the SAME coef_feats -> CoefMLP -> loss; only the integrator differs.
enum class rollout_kind { surrogate, bicycle };

// Self-supervised training hyperparameters (mirror coef_train.py train()).
struct train_config {
  int steps = 400;  // outer optimization steps (each a fresh agent batch)
  int horizon = 28; // rollout steps per outer step
  int n = 192;      // agents per batch
  int window = 7;   // truncated-BPTT window (detach + optimizer step every `window`)
  int hidden = 64;  // CoefMLP hidden width
  // Adam step. This is a REFINEMENT of the hand-tuned (1,3,4) basin the net is
  // centered on, not from-scratch learning: the point-mass training surrogate
  // has no turning limits, so a large step drives the goal-spring far higher than
  // the deployment bicycle can execute and wrecks navigation. Empirically (city
  // scene, sim_world reach) 2e-4 refines the basin and IMPROVES reach; 1e-3
  // (coef_train.py's never-validated default) collapses it. Lower is safer.
  float lr = 2e-4f;
  // The safety-for-reach dial, NOT a hyperparameter to tune away. Both loss
  // terms are normalized to O(1) (goal distance by the world half-extent, the
  // penalty by d_safe), so this expresses a preference directly. Measured on
  // the torch reference over three seeds: 1-3 improves on the seed in reach AND
  // penetration, 10 is bimodal, 30 cuts penetration ~94% for two thirds of the
  // reach. It was 6.0 when the terms were unnormalized and the collision term
  // was 0.2-0.7% of the loss -- at which point no value of this worked.
  float w_coll = 3.0f;
  // Clearance below which the penalty engages, in normalized units. 0 = use the
  // scene's d_hat, matching train_bicycle's default.
  float d_safe = 0.0f;
  float grad_clip = 5.0f; // global-norm gradient clip
  unsigned seed = 0;

  rollout_kind rollout = rollout_kind::surrogate; // which integrator to train on
  // Bicycle vehicle params (used only when rollout == bicycle); defaults are the
  // SdfNavigator VEHICLE_DEFAULTS, so bicycle training matches deployment.
  float veh_L = 0.035f, veh_delta_max = 0.6f, veh_a_max = 1.5f, veh_a_lat_max = 1.0f,
        veh_k_steer = 0.8f;
  bool veh_allow_reverse = true;
};

// The trainable CoefMLP: its dense layers as flat params + Adam state. Distinct
// from coef_mlp (the frozen inference policy); to_coef_mlp() bakes one. No
// libtorch — the forward caches activations and hand-written adjoints give the
// gradient (validated by the gradcheck test).
class coef_trainer {
public:
  explicit coef_trainer(const train_config &cfg, unsigned init_seed = 0);

  // Full self-supervised training over `scene` (mutates the params). `verbose`
  // prints a periodic loss trace.
  void train(const training_scene &scene, bool verbose = false);

  // Bake the frozen inference policy: softplus(net + log(expm1(bias))), the same
  // topology and semantics a loaded .cvcnav has. Persist with its save().
  coef_mlp to_coef_mlp() const;

  // ── exposed for the finite-difference gradcheck (nav_coef_train_test) ──
  // One truncated-BPTT window of `window` steps from (o, v, goal): returns the
  // scalar window loss and, if `grad` != nullptr, writes d(loss)/d(params) into
  // it (size num_params()). PURE — no Adam, params unchanged, inputs const. o/v
  // are the window's starting state ([n*2] normalized; v is zero at a batch
  // start). If `o_out`/`v_out` are non-null, the window-final (o, v) are written
  // there (the detached continuation the next window starts from).
  double loss_and_grad(const training_scene &scene, const float *o, const float *v,
                       const float *goal, int n, int window, std::vector<float> *grad,
                       float *o_out = nullptr, float *v_out = nullptr) const;

  const std::vector<float> &params() const { return p_; }
  void set_params(const std::vector<float> &p) { p_ = p; }
  int num_params() const { return static_cast<int>(p_.size()); }

  // One Adam step (global-norm clip) from an externally-computed gradient — lets
  // the CUDA trainer reuse the host optimizer with device-computed gradients.
  void apply_grad(const std::vector<float> &grad) { adam_step(grad); }

  // (alpha, beta, gamma) for a feature batch (frozen forward) — reach eval / debug.
  // feat[n*5] row-major -> out[n*3].
  void coeffs(const float *feat, int n, float *out) const;

private:
  train_config cfg_;
  int hidden_ = 64;
  std::vector<float> p_;     // flat params: [L0.w(h*5) L0.b(h) L1.w(h*h) L1.b(h) L2.w(3*h) L2.b(3)]
  std::vector<float> m_, u_; // Adam first/second moments (u_ = "v", renamed to avoid the vel v)
  long adam_t_ = 0;
  // Per-layer flat offsets into p_ (w then b), plus (rows, cols).
  int off_w_[3] = {0, 0, 0}, off_b_[3] = {0, 0, 0};
  int lrows_[3] = {0, 0, 0}, lcols_[3] = {0, 0, 0};
  float bias_[3] = {1.0f, 3.0f, 4.0f}; // the CoefMLP basin (alpha, beta, gamma)

  void adam_step(const std::vector<float> &grad);
};

// ── CUDA trainer (defined in coef_train.cu; CVC_ENABLE_CUDA only) ─────────────
// The same self-supervised training on the GPU: the field, params and per-window
// activations stay device-resident; the backward is a device transcription of
// the CPU adjoints. Returns the baked policy. Gated by available() (built with
// CUDA AND a device present); throws otherwise. Validated against the CPU trainer
// on the same seed/scene (nav_coef_train_test, CVC_ENABLE_CUDA).
coef_mlp train_coef_mlp_cuda(const training_scene &scene, const train_config &cfg,
                             bool verbose = false);
bool train_cuda_available();

// One-window loss+grad on the GPU (exposed for the CUDA-vs-CPU gradient test and
// reused by train_coef_mlp_cuda). Same contract as coef_trainer::loss_and_grad;
// `params` is the flat param vector; `o_out`/`v_out` (optional) receive the
// window-final state. `cfg.hidden`/`w_coll` set the architecture / loss weight.
double loss_and_grad_cuda(const training_scene &scene, const train_config &cfg,
                          const std::vector<float> &params, const float *o, const float *v,
                          const float *goal, int n, int window, std::vector<float> *grad,
                          float *o_out = nullptr, float *v_out = nullptr);

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_COEF_TRAIN_H__
