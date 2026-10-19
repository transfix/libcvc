/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// See nav_eval.h.

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/nav_eval.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/world.h>
#include <exception>
#include <stdexcept>
#include <thread>

namespace cvc {
namespace physics {

namespace {

// One seed: the fraction of `o.agents` that reached their goals in `o.ticks`. `step_threads`
// is sim_world::step's (its per-agent kernels give the same result on any count).
double reach_once(const cvc::nav::coef_mlp &policy, const cvc::nav::training_scene &sc,
                  const vehicle_desc &d, double mpw, const plant_reach_options &o, unsigned seed,
                  int step_threads) {
  cvc::nav::sim_world::config cfg;
  cfg.rows = sc.rows;
  cfg.cols = sc.cols;
  cfg.min_x = sc.min_x;
  cfg.min_y = sc.min_y;
  cfg.max_x = sc.max_x;
  cfg.max_y = sc.max_y;
  cfg.scale = sc.scale;
  cfg.veh.rr = sc.rr;
  cfg.veh.d_hat = sc.d_hat;
  cfg.veh.dt = sc.dt;
  cfg.veh.vmax = sc.vmax;
  cfg.veh.nsub = o.nsub;
  cfg.reach_tol = o.reach_tol;
  cfg.freeze_sense = true;
  cvc::nav::sim_world w =
      cvc::nav::sim_world::from_occupancy(cfg, sc.occ.data(), policy, o.agents, seed);

  world_config wc;
  wc.backend = backend::jolt;
  world phys(wc);
  const material_id m = phys.add_material({"ground", static_cast<float>(o.mu), 0.0f, 0.015f});
  body_desc g;
  g.shape = box{{(sc.max_x - sc.min_x) * mpw, (sc.max_y - sc.min_y) * mpw, 0.5}};
  g.motion = motion_type::static_;
  g.position = {0.0, 0.0, -0.5};
  g.material = m;
  phys.add_body(g);
  const double dx = (sc.max_x - sc.min_x) / (sc.cols - 1),
               dy = (sc.max_y - sc.min_y) / (sc.rows - 1);
  const double hh = 0.5 * o.building_height;
  for (int r = 0; r < sc.rows; ++r)
    for (int c = 0; c < sc.cols;) {
      if (!sc.occ[static_cast<std::size_t>(r) * sc.cols + c]) {
        ++c;
        continue;
      }
      int e = c;
      while (e + 1 < sc.cols && sc.occ[static_cast<std::size_t>(r) * sc.cols + e + 1])
        ++e;
      body_desc b;
      b.shape = box{{0.5 * (e - c + 1) * dx * mpw, 0.5 * dy * mpw, hh}};
      b.motion = motion_type::static_;
      b.position = {(sc.min_x + 0.5 * (c + e) * dx) * mpw, (sc.min_y + r * dy) * mpw, hh};
      b.material = m;
      phys.add_body(b);
      c = e + 1;
    }
  nav_plant_options po;
  po.plan_with_vehicle = o.plan_with_vehicle;
  po.metres_per_world_unit = mpw;
  nav_plant plant(phys, {d}, po);
  w.set_plant(&plant);
  for (int t = 0; t < o.ticks; ++t)
    w.step(step_threads);
  std::vector<std::uint8_t> reached(static_cast<std::size_t>(o.agents));
  w.snapshot(nullptr, nullptr, nullptr, nullptr, reached.data());
  w.set_plant(nullptr);
  int k = 0;
  for (const std::uint8_t r : reached)
    k += r ? 1 : 0;
  return static_cast<double>(k) / o.agents;
}

} // namespace

double reach_on_plant(const cvc::nav::coef_mlp &policy, const cvc::nav::training_scene &scene,
                      const vehicle_desc &d, double wheelbase, const plant_reach_options &o,
                      std::vector<double> *per_seed) {
  std::vector<std::vector<double>> per;
  const std::vector<double> m = reach_on_plant(std::vector<cvc::nav::coef_mlp>{policy}, scene, d,
                                               wheelbase, o, per_seed ? &per : nullptr);
  if (per_seed)
    *per_seed = per[0];
  return m[0];
}

std::vector<double> reach_on_plant(const std::vector<cvc::nav::coef_mlp> &policies,
                                   const cvc::nav::training_scene &scene, const vehicle_desc &d,
                                   double wheelbase, const plant_reach_options &o,
                                   std::vector<std::vector<double>> *per_seed) {
  if (!backend_available(backend::jolt))
    throw std::runtime_error("cvc::physics::reach_on_plant: this libcvc was built without Jolt");
  if (!(wheelbase > 0.0) || !(d.wheelbase() > 0.0) || o.agents <= 0 || o.ticks <= 0 ||
      o.seeds.empty())
    throw std::invalid_argument("cvc::physics::reach_on_plant: needs a positive wheelbase, agents, "
                                "ticks and at least one seed");
  const double s_per_m = wheelbase / d.wheelbase();
  const double mpw = scene.scale / s_per_m; // metres per world unit
  const int ns = static_cast<int>(o.seeds.size());
  const int n = static_cast<int>(policies.size()) * ns; // one run per (policy, seed)
  const int hw = std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
  const int workers = std::min(n, o.threads > 0 ? o.threads : hw);
  std::vector<double> r(static_cast<std::size_t>(n));
  const auto run = [&](int k, int step_threads) {
    r[k] = reach_once(policies[k / ns], scene, d, mpw, o, o.seeds[k % ns], step_threads);
  };
  if (workers <= 1) {
    for (int k = 0; k < n; ++k)
      run(k, 0);
  } else {
    // Each worker takes the next run. A run's world, sim_world and plant are its own, and its
    // agents step on the worker's thread.
    std::atomic<int> next{0};
    std::vector<std::exception_ptr> errors(static_cast<std::size_t>(workers));
    std::vector<std::thread> pool;
    for (int t = 0; t < workers; ++t)
      pool.emplace_back([&, t] {
        try {
          for (int k = next++; k < n; k = next++)
            run(k, 1);
        } catch (...) {
          errors[t] = std::current_exception();
          next = n; // the others stop after their current seed
        }
      });
    for (std::thread &th : pool)
      th.join();
    for (const std::exception_ptr &e : errors)
      if (e)
        std::rethrow_exception(e);
  }
  std::vector<double> mean(policies.size(), 0.0);
  if (per_seed)
    per_seed->assign(policies.size(), std::vector<double>());
  for (std::size_t p = 0; p < policies.size(); ++p) {
    for (int s = 0; s < ns; ++s)
      mean[p] += r[p * ns + s];
    mean[p] /= ns;
    if (per_seed)
      (*per_seed)[p].assign(r.begin() + static_cast<long>(p * ns),
                            r.begin() + static_cast<long>((p + 1) * ns));
  }
  return mean;
}

} // namespace physics
} // namespace cvc
