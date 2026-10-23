/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph::se2 training: the learned SE(2) vehicle (se2_vehicle.h) fit to logged flights in C++,
// with no Python and no autodiff library. The recipe is LieSPHGP's SE(2) trainer (train.py) with
// its rollout loss:
//
//   L = rollout(windows | W) + beta KL(q || p) / (N_obs 6),   W = mean + e^log_std z,  z ~ N(0, I)
//
// rollout(): from each window's first sample, roll the model out under the recorded inputs and
// compare every later sample (position, wrapped heading, twist): the mean over the window of the
// squared error summed over the six channels, averaged over the windows that stay finite. One
// reparameterized weight draw per step; beta rises linearly to kl_beta_max over kl_anneal_steps;
// N_obs is the training windows times their intervals.
//
// The gradient is a hand-written reverse pass through every Lie-IMEX step of the window
// (backpropagation through time, as nav's coef_train does): the 3x3 implicit solves, the SPD
// parameterization, and the random-feature GPs, including grad V's own derivative in position.
// ph_se2_train_test holds it to central differences and to JAX's autodiff of the same objective.
//
// The optimizer is train.py's: clip the whole gradient to global norm 1, Adam (cosine-decayed)
// on the GP weights and the noise scales, Prodigy (one per subnetwork) on the levels.

#ifndef __CVC_PH_SE2_TRAIN_H__
#define __CVC_PH_SE2_TRAIN_H__

#include <array>
#include <cstddef>
#include <cstdint>
#include <cvc/ph/optim.h>
#include <cvc/ph/se2_vehicle.h>
#include <functional>
#include <string>
#include <vector>

namespace cvc {
class thread_pool;
namespace ph {

// ── data
// ──────────────────────────────────────────────────────────────────────────────────────────

// Equal-length flights: rows [x, y, psi, v_x, v_y, r, u (control_dim)], row k's inputs acting
// from t_k-1 to t_k (cvc::physics::plant_log's layout; row 0's inputs unused).
struct se2_flights {
  std::size_t count = 0, rows = 0;
  int width = 8;            // 6 + control_dim
  std::vector<double> data; // [count x rows x width]
  std::vector<int> seed;    // each flight's seed, when known
  double dt = 0.02;         // s between rows

  int control_dim() const { return width - 6; }
  const double *flight(std::size_t f) const { return data.data() + f * rows * width; }
};

// The RMS of each twist channel and of each input over every row of every flight: the model's
// twist_scale and control_scale (LieSPHGP's data statistics, rows 0 included).
std::array<double, 3> twist_rms(const se2_flights &f);
std::vector<double> control_rms(const se2_flights &f);

// The flights with their inputs divided by `scale`.
se2_flights scale_controls(const se2_flights &f, const std::vector<double> &scale);

// Absolute observation noise N(0, sigma) on x, y, psi, v_x, v_y and r of every row, the inputs
// left exact (the noise a plant-learning dataset is given; cvc's liesphgp_pack.py --noise).
se2_flights add_observation_noise(const se2_flights &f, double sigma, std::uint64_t seed);

// cvc-plant-log run directories (cvc::physics::plant_log's flights.npy and meta.json, format
// cvc-plant-log-1), concatenated in order, each flight's seed from meta.json. Row 0's inputs are
// set to zero, as LieSPHGP's datasets have them (they act before the flight; no model reads them,
// but they count in control_rms). Throws std::runtime_error.
se2_flights load_plant_log_runs(const std::vector<std::string> &dirs);

// The flights whose seed is (not) in `seeds`.
se2_flights select_seeds(const se2_flights &f, const std::vector<int> &seeds, bool keep);

// A little-endian .npy array of float64 (or float32, widened), C order: its shape and values.
// Throws std::runtime_error.
std::vector<double> read_npy(const std::string &path, std::vector<std::size_t> &shape);
// Writes float64, C order. Throws std::runtime_error.
void write_npy(const std::string &path, const std::vector<double> &data,
               const std::vector<std::size_t> &shape);

// Training windows: `points` rows every `stride` rows of every flight, window-major.
struct se2_windows {
  int points = 0, width = 8;
  std::size_t count = 0;
  std::vector<double> data; // [count x points x width]
  const double *window(std::size_t w) const { return data.data() + w * points * width; }
};
se2_windows cut_windows(const se2_flights &f, int points, int stride);

// ── the objective ────────────────────────────────────────────────────────────────────────────────

// One reparameterization draw: z for D, G and V ([features x outputs] each).
struct se2_draw_noise {
  std::vector<double> zD, zG, zV;
  static se2_draw_noise standard_normal(const se2_model &m, rng &r);
};

struct se2_objective {
  double value = 0.0;      // rollout + beta_scale KL
  double rollout = 0.0;    // the rollout loss
  double kl = 0.0;         // KL(q || N(0, I)) of the three GPs
  std::size_t healthy = 0; // windows whose rollout stayed finite
};

// The objective over windows `idx` (all when null) with the draw z (the posterior mean when null)
// and beta_scale = beta / (N_obs 6); its gradient into `grad` (shaped like m.params) when given.
// Pure. The windows run on `pool` when one is given; the result is the same bit for bit either
// way (each window's gradient is its own, summed in window order).
se2_objective se2_rollout_objective(const se2_model &m, const se2_windows &w,
                                    const std::vector<std::size_t> *idx, const se2_draw_noise *z,
                                    double beta_scale, se2_params *grad = nullptr,
                                    cvc::thread_pool *pool = nullptr);

// ── the optimizer ────────────────────────────────────────────────────────────────────────────────

struct se2_optimizer_config {
  int total_steps = 5000;      // the cosine decay's length
  double learning_rate = 3e-3; // Adam's
  double final_learning_fraction = 0.1;
  double level_learning_rate = 0.25; // Prodigy's (its base is 1)
  double beta1 = 0.9, beta2 = 0.999, epsilon = 1e-8;
  double gradient_clip_norm = 1.0; // <= 0: no clipping
  double prodigy_d0 = 1e-3;        // Prodigy's initial d
};

// train.py's make_optimizer: clip, then Adam on every leaf but the levels, Prodigy on each level.
class se2_optimizer {
public:
  se2_optimizer(const se2_params &p0, const se2_optimizer_config &cfg);
  // One update of p by grad (which is clipped in place). Returns the gradient's norm before.
  double update(se2_params &p, se2_params &grad);
  double prodigy_d(se2_leaf_group level) const;
  int count() const { return count_; }

private:
  se2_optimizer_config cfg_;
  adam adam_;
  prodigy level_[4];
  int count_ = 0;
};

// ── training ─────────────────────────────────────────────────────────────────────────────────────

struct se2_train_config {
  std::uint64_t seed = 0;
  int total_steps = 5000;
  int batch_size = 16;
  double kl_beta_max = 1.0;
  int kl_anneal_steps = 1000;
  int eval_every = 500; // the monitor: the mean model's loss on eval_windows test windows
  int eval_windows = 64;
  int threads = 0; // <= 0: hardware concurrency
  se2_optimizer_config optimizer;
};

struct se2_train_event {
  int step = 0;
  se2_objective objective; // this step's batch, this step's draw
  double beta = 0.0;
  double gradient_norm = 0.0;
  bool evaluated = false; // test_loss and train_loss are set
  double test_loss = 0.0, train_loss = 0.0;
  std::array<double, 4> prodigy_d{}; // D, G, V, Minv
};

// Fits `m` (an se2_model::initialize'd one, or a checkpoint to continue) to `train` in place.
// `test` (may be null) is only monitored, never trained on. on_step (may be empty) sees every
// step. Throws std::runtime_error on a non-finite loss or gradient, as train.py aborts.
void se2_train_rollout(se2_model &m, const se2_windows &train, const se2_windows *test,
                       const se2_train_config &cfg,
                       const std::function<void(const se2_train_event &)> &on_step = {});

} // namespace ph
} // namespace cvc

#endif // __CVC_PH_SE2_TRAIN_H__
