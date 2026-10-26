/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// See nav_es.h.

#include <algorithm>
#include <cmath>
#include <cvc/physics/nav_es.h>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>

namespace cvc {
namespace physics {

namespace {

// A standard normal from mt19937's raw output through Box-Muller: the same numbers on every
// standard library, which std::normal_distribution does not promise.
struct portable_normal {
  std::mt19937 rng;
  bool have = false;
  double spare = 0.0;
  explicit portable_normal(unsigned seed) : rng(seed) {}
  double uniform() { return (static_cast<double>(rng()) + 0.5) / 4294967296.0; } // (0, 1)
  double operator()() {
    if (have) {
      have = false;
      return spare;
    }
    const double r = std::sqrt(-2.0 * std::log(uniform())), t = 6.283185307179586 * uniform();
    spare = r * std::sin(t);
    have = true;
    return r * std::cos(t);
  }
};

// Centred ranks in [-0.5, 0.5]; tied scores share their mean rank (reach is a count, so ties
// are common).
std::vector<double> centred_ranks(const std::vector<double> &f) {
  const std::size_t n = f.size();
  std::vector<std::size_t> idx(n);
  std::iota(idx.begin(), idx.end(), std::size_t(0));
  std::sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) { return f[a] < f[b]; });
  std::vector<double> r(n);
  for (std::size_t i = 0; i < n;) {
    std::size_t j = i;
    while (j + 1 < n && f[idx[j + 1]] == f[idx[i]])
      ++j;
    for (std::size_t k = i; k <= j; ++k)
      r[idx[k]] = 0.5 * static_cast<double>(i + j);
    i = j + 1;
  }
  const double top = n > 1 ? static_cast<double>(n - 1) : 1.0;
  for (double &x : r)
    x = x / top - 0.5;
  return r;
}

} // namespace

es_fitness es_energy_at_held_reach(double price, double slack) {
  if (!(price >= 0.0) || !(slack >= 0.0))
    throw std::invalid_argument("cvc::physics::es_energy_at_held_reach: needs price, slack >= 0");
  return [price, slack](const plant_drive_score &p, const plant_drive_score &start) {
    const double lost = 100.0 * (start.reach - p.reach) - slack;
    return -p.wh_per_km() - price * (lost > 0.0 ? lost : 0.0);
  };
}

es_result es_finetune_on_plant(const cvc::nav::coef_mlp &start,
                               const cvc::nav::training_scene &scene, const vehicle_desc &d,
                               double wheelbase, const es_options &o,
                               const std::function<void(const es_iteration &)> &progress) {
  if (o.iterations < 0 || o.pairs <= 0 || !(o.sigma > 0.0f) || o.train_seeds <= 0 || o.tail < 0 ||
      o.reach.seeds.empty())
    throw std::invalid_argument("cvc::physics::es_finetune_on_plant: needs iterations, tail >= 0, "
                                "pairs, sigma and train_seeds > 0, and validation seeds");
  std::vector<float> theta = start.export_flat().data;
  const std::size_t N = theta.size();
  // The searched weights: the last D of the flat vector.
  const std::size_t D = o.tail > 0 ? std::min(static_cast<std::size_t>(o.tail), N) : N, off = N - D;
  const float b1 = 0.9f, b2 = 0.999f, adam_eps = 1e-8f;
  std::vector<float> m(D, 0.0f), v(D, 0.0f), g(D), w(N);
  portable_normal gauss(o.noise_seed);
  const double nan = std::numeric_limits<double>::quiet_NaN();

  // Scores policies on `opts`' seeds: reach, or the fitness beside `ref` (the start's scores on
  // the same seeds; with ref_out, the start is scored alongside and returned there).
  const auto score = [&](const std::vector<cvc::nav::coef_mlp> &pols,
                         const plant_reach_options &opts, const plant_drive_score *ref,
                         plant_drive_score *ref_out, std::vector<plant_drive_score> *scores) {
    if (!o.fitness)
      return reach_on_plant(pols, scene, d, wheelbase, opts);
    std::vector<cvc::nav::coef_mlp> all = pols;
    if (ref_out)
      all.push_back(start);
    std::vector<plant_drive_score> m = score_on_plant(all, scene, d, wheelbase, opts);
    if (ref_out) {
      *ref_out = m.back();
      m.pop_back();
      ref = ref_out;
    }
    std::vector<double> f(m.size());
    for (std::size_t i = 0; i < m.size(); ++i)
      f[i] = o.fitness(m[i], *ref);
    if (scores)
      *scores = m;
    return f;
  };

  es_result res;
  res.best = start;
  res.best_iteration = 0;
  plant_drive_score start_val; // the start on the validation seeds (with a fitness)
  if (o.fitness) {
    start_val = score_on_plant({start}, scene, d, wheelbase, o.reach)[0];
    res.best_validation = o.fitness(start_val, start_val);
  } else
    res.best_validation = reach_on_plant(start, scene, d, wheelbase, o.reach);
  res.trace.push_back({0, nan, res.best_validation});
  if (o.fitness) {
    res.trace.back().validation_reach = start_val.reach;
    res.trace.back().validation_wh_per_km = start_val.wh_per_km();
  }
  if (progress)
    progress(res.trace.back());

  plant_reach_options train = o.reach;
  train.seeds.assign(static_cast<std::size_t>(o.train_seeds), 0u);
  std::vector<std::vector<float>> dir(static_cast<std::size_t>(o.pairs), std::vector<float>(D));
  std::vector<cvc::nav::coef_mlp> pop(static_cast<std::size_t>(2 * o.pairs));
  for (int it = 0; it < o.iterations; ++it) {
    for (int k = 0; k < o.train_seeds; ++k)
      train.seeds[k] = o.train_seed_base + static_cast<unsigned>(it * o.train_seeds + k);
    // The population: theta +- sigma e_k.
    for (int k = 0; k < o.pairs; ++k) {
      for (std::size_t j = 0; j < D; ++j)
        dir[k][j] = static_cast<float>(gauss());
      for (int side = 0; side < 2; ++side) {
        const float s = side == 0 ? o.sigma : -o.sigma;
        w = theta;
        for (std::size_t j = 0; j < D; ++j)
          w[off + j] = theta[off + j] + s * dir[k][j];
        pop[2 * k + side] = start.with_flat_weights(w);
      }
    }
    plant_drive_score start_train; // the start on this iteration's seeds
    const std::vector<double> f = score(pop, train, nullptr, &start_train, nullptr);
    const std::vector<double> u = centred_ranks(f);
    // The gradient estimate, then an Adam step uphill.
    std::fill(g.begin(), g.end(), 0.0f);
    for (int k = 0; k < o.pairs; ++k) {
      const float c = static_cast<float>(u[2 * k] - u[2 * k + 1]);
      for (std::size_t j = 0; j < D; ++j)
        g[j] += c * dir[k][j];
    }
    const float scale = 1.0f / (2.0f * static_cast<float>(o.pairs) * o.sigma);
    const float c1 = 1.0f - std::pow(b1, static_cast<float>(it + 1)),
                c2 = 1.0f - std::pow(b2, static_cast<float>(it + 1));
    for (std::size_t j = 0; j < D; ++j) {
      const float gj = g[j] * scale;
      m[j] = b1 * m[j] + (1.0f - b1) * gj;
      v[j] = b2 * v[j] + (1.0f - b2) * gj * gj;
      theta[off + j] += o.lr * (m[j] / c1) / (std::sqrt(v[j] / c2) + adam_eps);
    }
    double mean = 0.0;
    for (const double x : f)
      mean += x;
    mean /= static_cast<double>(f.size());
    double val = nan, val_reach = nan, val_whkm = nan;
    if (it + 1 == o.iterations || (o.validate_every > 0 && (it + 1) % o.validate_every == 0)) {
      const cvc::nav::coef_mlp centre = start.with_flat_weights(theta);
      std::vector<plant_drive_score> cs;
      val = score({centre}, o.reach, &start_val, nullptr, &cs)[0];
      if (o.fitness) {
        val_reach = cs[0].reach;
        val_whkm = cs[0].wh_per_km();
      }
      if (val > res.best_validation) { // ties keep the earlier
        res.best_validation = val;
        res.best = centre;
        res.best_iteration = it + 1;
      }
    }
    res.trace.push_back({it + 1, mean, val, val_reach, val_whkm});
    if (progress)
      progress(res.trace.back());
  }
  return res;
}

} // namespace physics
} // namespace cvc
