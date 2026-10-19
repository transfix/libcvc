/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::es_finetune_on_plant: a zeroth-order fine-tune of a nav policy on the physics
// plant's reach (the vehicle-physics plan's P8b). The differentiable trainers move Jolt reach by
// a point at most (docs/NAV_TRAINING.md), so this climbs the deployment number itself, with no
// gradient through the plant: antithetic evolution strategies over the policy's flat weights
// (Salimans et al. 2017).
//
//   cvc::physics::es_options o;
//   o.reach.seeds = validation_seeds;  // e.g. 200..231, never the evaluation's
//   cvc::physics::es_result r = cvc::physics::es_finetune_on_plant(
//       start, cvc::nav::city_scene(96), cvc::physics::vehicle_desc::roboracer(), 0.035, o);
//   r.best.save("tuned.cvcnav");
//
// Each iteration draws `pairs` Gaussian directions e_k and scores theta + sigma e_k and
// theta - sigma e_k by reach_on_plant over `train_seeds` fresh city seeds, the same seeds for the
// whole population (common random numbers). The centred ranks of those 2 x pairs scores weight
// the directions into a gradient estimate, which Adam follows uphill. Before the first
// iteration, every `validate_every` iterations and after the last, the centre is scored on
// `reach.seeds`; the result is the best of those, the start included, so it never ends below
// where it started on validation. The noise comes from mt19937 through Box-Muller, not
// std::normal_distribution, whose output differs by standard library.
//
// Over every weight, start from a trained policy: coef_mlp::default_biased() has all-zero
// weights, and small perturbations of a zero network barely change its output (`tail` = 3
// tunes its basin instead). Needs Jolt (reach_on_plant).

#ifndef __CVC_PHYSICS_NAV_ES_H__
#define __CVC_PHYSICS_NAV_ES_H__

#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/physics/nav_eval.h>
#include <cvc/physics/vehicle.h>
#include <functional>
#include <vector>

namespace cvc {
namespace physics {

struct es_options {
  int iterations = 40;
  int pairs = 16;                   // antithetic pairs an iteration: 2 x pairs policies scored
  float sigma = 0.02f;              // the perturbations' standard deviation, in weight units
  float lr = 0.005f;                // Adam's step
  int train_seeds = 4;              // city seeds an iteration, fresh each iteration
  unsigned train_seed_base = 1000u; // iteration i (from 0) uses base + i * train_seeds + k
  int validate_every = 5;           // 0 = only the start and the end
  // Perturb only the last `tail` flat weights (0 = all). 3 is the output layer's bias, which
  // shifts the coefficient basin (alpha, beta, gamma): reach's leverage is mostly there, and a
  // 3-dimensional estimate is far less noisy than one over every weight. It also works from
  // coef_mlp::default_biased(), whose hidden layers are zero.
  int tail = 0;
  unsigned noise_seed = 0;
  plant_reach_options reach; // agents, ticks, threads, ...; its `seeds` are the validation seeds
};

struct es_iteration {
  int iteration;          // updates taken so far (0 = the start)
  double population_mean; // the mean reach of this update's 2 x pairs policies (NaN at 0)
  double validation;      // the centre's reach on reach.seeds after it (NaN when not scored)
};

struct es_result {
  cvc::nav::coef_mlp best; // the best-validated centre
  int best_iteration = 0;
  double best_validation = 0.0;
  std::vector<es_iteration> trace;
};

es_result es_finetune_on_plant(const cvc::nav::coef_mlp &start,
                               const cvc::nav::training_scene &scene, const vehicle_desc &d,
                               double wheelbase, const es_options &o = es_options(),
                               const std::function<void(const es_iteration &)> &progress =
                                   std::function<void(const es_iteration &)>());

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_NAV_ES_H__
