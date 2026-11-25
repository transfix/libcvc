/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// CoPH drives a sim_world's vehicles (C5 part 2b of docs/roadmap/NAV-COPH-ROADMAP.md, §4.5); see
// coph_driver.h. Each step names the GRL-SNAM code it follows (grl_snam.ariadne.coph_cars).
// Built with -ffp-contract=off.

#include <algorithm>
#include <cmath>
#include <cvc/nav/belief_material_controller.h>
#include <cvc/nav/coph_driver.h>
#include <cvc/nav/material.h>
#include <cvc/nav/sim_world.h>
#include <stdexcept>
#include <string>

namespace cvc {
namespace nav {

namespace {

double hypot2(double x, double y) { return std::sqrt(x * x + y * y); }

// numpy.allclose's rule (rtol 1e-5, atol 1e-8), elementwise.
bool allclose(const coph_vec2 &a, const coph_vec2 &b) {
  for (int k = 0; k < 2; ++k)
    if (!(std::fabs(a[k] - b[k]) <= 1e-8 + 1e-5 * std::fabs(b[k])))
      return false;
  return true;
}

} // namespace

coph_driver::coph_driver(sim_world &sim, coph_driver_config cfg,
                         std::shared_ptr<coph_channel> channel)
    : sim_(sim), cfg_(std::move(cfg)), channel_(std::move(channel)) {
  cfg_.cfg.validate();
  cfg_.comm.validate();
  if (!cfg_.comm.outside_profile().empty())
    throw std::invalid_argument("coph_driver: the communication is outside the demos' profile");
  if (!(cfg_.froude > 0) || !(cfg_.local_goal > 0) || !(cfg_.speed_floor > 0))
    throw std::invalid_argument("coph_driver: froude, local_goal and speed_floor must be positive");
  const sim_world::config &sc = sim_.current_config();
  if (!(sc.veh.dt > 0))
    throw std::invalid_argument("coph_driver: the sim's veh.dt must be positive");
  origin_ = cfg_.origin ? *cfg_.origin : coph_vec2{sc.cx, sc.cy};
  // Froude.ticks_per_step: CoPH's dt in world time over the tick (D6).
  k_ = std::max(1, static_cast<int>(std::lround(cfg_.cfg.dt * std::sqrt(cfg_.froude) / sc.veh.dt)));
}

void coph_driver::attach(int i) {
  if (fleet_)
    throw std::logic_error("coph_driver::attach: attach before the first apply()");
  if (i < 0 || i >= sim_.size())
    throw std::out_of_range("coph_driver::attach: no such agent");
  if (std::find(attached_.begin(), attached_.end(), i) != attached_.end())
    throw std::invalid_argument("coph_driver::attach: already attached");
  attached_.push_back(i);
  car c;
  c.agent = i;
  cars_.push_back(c);
  ledgers_.emplace_back();
}

bool coph_driver::done(int vehicle) const { return cars_.at(vehicle).done; }

double coph_driver::ledger::allocation_residual() const {
  return u2 > 0 ? std::sqrt(e2 / u2) : 0.0;
}

std::vector<coph_driver::pose> coph_driver::read() const {
  const int n = sim_.size();
  std::vector<float> pos(2 * static_cast<std::size_t>(n)), head(n), spd(n);
  std::vector<std::uint8_t> reached(n);
  sim_.snapshot(pos.data(), head.data(), spd.data(), nullptr, reached.data());
  std::vector<pose> out(n);
  for (int i = 0; i < n; ++i)
    out[i] = {{pos[2 * i], pos[2 * i + 1]}, head[i], spd[i], reached[i] != 0};
  return out;
}

coph_vec2 coph_driver::to_coph(const coph_vec2 &w) const {
  return {(w[0] - origin_[0]) / cfg_.froude, (w[1] - origin_[1]) / cfg_.froude};
}

coph_vec2 coph_driver::to_world(const coph_vec2 &q) const {
  return {q[0] * cfg_.froude + origin_[0], q[1] * cfg_.froude + origin_[1]};
}

// CoPH's state of a vehicle: q from its pose, v = its speed along the heading (D5's v h).
coph_state coph_driver::state(const pose &p) const {
  const coph_vec2 q = to_coph(p.p);
  const double r = std::sqrt(cfg_.froude);
  return {q[0], q[1], p.speed * std::cos(p.heading) / r, p.speed * std::sin(p.heading) / r};
}

// local_goal (D12): the goal, or the point local_goal along the straight line when it is farther.
coph_vec2 coph_driver::local_goal(const coph_vec2 &p, const coph_vec2 &goal) const {
  const double dx = goal[0] - p[0], dy = goal[1] - p[1];
  const double dist = hypot2(dx, dy);
  if (dist <= cfg_.local_goal)
    return goal;
  return {p[0] + dx * (cfg_.local_goal / dist), p[1] + dy * (cfg_.local_goal / dist)};
}

// The material plane's r~ at a world point (the agent's plane under belief material), else
// base_risk.
double coph_driver::risk_at(int agent, const coph_vec2 &world) const {
  if (!sim_.has_material())
    return cfg_.base_risk;
  const sim_world::config &sc = sim_.current_config();
  const int *planes = sim_.agent_planes();
  const int plane = sim_.has_material_belief() && planes ? planes[agent] : 0;
  material_stack m{sim_.material_plane_data(plane),
                   1,
                   sc.rows,
                   sc.cols,
                   sc.min_x,
                   sc.min_y,
                   sc.max_x,
                   sc.max_y,
                   sc.cx,
                   sc.cy,
                   sc.scale};
  if (!m.data)
    return cfg_.base_risk;
  const float on[2] = {static_cast<float>((world[0] - sc.cx) * sc.scale),
                       static_cast<float>((world[1] - sc.cy) * sc.scale)};
  float risk = 0, phi = 0, gr[2] = {0, 0}, gp[2] = {0, 0};
  material_sample(m, on, 1, nullptr, &risk, &phi, gr, gp, 1);
  return risk;
}

// The occupied cells near a vehicle as CoPH's disks: visible_obstacles on its plane's last-visible
// cells (the truth under freeze_sense), those whose surface is within the scaled sensor radius.
std::vector<coph_shape> coph_driver::obstacles_near(int agent, const coph_vec2 &world) const {
  const sim_world::config &sc = sim_.current_config();
  const int *planes = sim_.agent_planes();
  const std::uint8_t *truth = sim_.truth();
  const std::uint8_t *seen =
      sc.freeze_sense ? truth : sim_.last_visible(planes ? planes[agent] : 0);
  std::vector<coph_shape> out;
  if (!truth || !seen)
    return out;
  const int cap = cfg_.cfg.obstacle_cap;
  std::vector<float> c(2 * static_cast<std::size_t>(cap)), r(cap), w(cap);
  const int k = visible_obstacles(seen, truth, sc.rows, sc.cols, sc.min_x, sc.min_y, sc.max_x,
                                  sc.max_y, world[0], world[1], cap, c.data(), r.data(), w.data());
  const double sensor = cfg_.cfg.sensor_radius * cfg_.froude;
  for (int j = 0; j < k; ++j) {
    const coph_vec2 cw{c[2 * j], c[2 * j + 1]};
    if (hypot2(cw[0] - world[0], cw[1] - world[1]) - r[j] > sensor)
      continue;
    const coph_vec2 q = to_coph(cw);
    out.push_back(coph_shape::disk(q[0], q[1], r[j] / cfg_.froude));
  }
  return out;
}

// _Run.coph_step: one CoPH step for every vehicle.
void coph_driver::coph_step(const std::vector<pose> &poses) {
  const coph_config &cfg = cfg_.cfg;
  const int n = static_cast<int>(cars_.size());
  std::vector<float> goals(2 * static_cast<std::size_t>(sim_.size()));
  sim_.goals_world(goals.data());
  if (!fleet_) {
    coph_fleet_config fc;
    fc.cfg = cfg;
    fc.comm = cfg_.comm;
    fc.network = coph_network{}; // a car's learner
    fc.seed = cfg_.seed;
    if (cfg_.senders.empty())
      for (int v = 0; v < n; ++v)
        fc.senders.push_back(v);
    else
      fc.senders = cfg_.senders;
    std::vector<coph_fleet::agent_spec> specs;
    for (car &c : cars_) {
      const pose &p = poses[c.agent];
      c.goal = {goals[2 * c.agent], goals[2 * c.agent + 1]};
      c.local = local_goal(p.p, c.goal);
      specs.push_back({state(p), to_coph(c.local)});
    }
    fleet_ = std::make_unique<coph_fleet>(fc, specs, channel_);
  }
  const long long dt_us = std::llround(cfg.dt * 1e6);
  const double now = static_cast<double>(steps_ * dt_us) / 1e6;
  std::vector<coph_fleet::input> in(n);
  for (int v = 0; v < n; ++v) {
    car &c = cars_[v];
    const pose &p = poses[c.agent];
    const coph_vec2 g{goals[2 * c.agent], goals[2 * c.agent + 1]};
    if (c.done) {
      // _regoal: a car at its goal that the sim gives a new one runs again, its learner keeping
      // what it learned (coph_fleet::resume). One ended by a hit stays done.
      if (allclose(g, c.goal) || !fleet_->resume(v))
        continue;
      c.done = false;
      c.x_prev.reset();
      c.used_sum = {0, 0};
      c.e_sum = {0, 0};
      c.held = 0;
    }
    coph_fleet::input &x = in[v];
    x.x = state(p);
    // _regoal: a new sim goal, or the local goal slid on along the line (D12).
    const bool moved = !allclose(g, c.goal);
    const bool near = hypot2(p.p[0] - c.local[0], p.p[1] - c.local[1]) < 0.5 * cfg_.local_goal &&
                      !allclose(c.local, g);
    if (moved || near) {
      c.goal = g;
      c.local = local_goal(p.p, g);
      x.goal = to_coph(c.local);
    }
    // The held step's ledger (Ledger.add_step), at the learner's damping before it observes.
    if (c.x_prev && c.held > 0) {
      const coph_vec2 u_used{c.used_sum[0] / c.held, c.used_sum[1] / c.held};
      const coph_vec2 e_mean{c.e_sum[0] / c.held, c.e_sum[1] / c.held};
      const coph_state &x0 = *c.x_prev, &x1 = x.x;
      const double damping = fleet_->agents()[v].state().damping;
      coph_balance_terms b;
      b.work = u_used[0] * (x1[0] - x0[0]) + u_used[1] * (x1[1] - x0[1]);
      b.kinetic_change = 0.5 * ((x1[2] * x1[2] + x1[3] * x1[3]) - (x0[2] * x0[2] + x0[3] * x0[3]));
      b.heat = coph_balance(x0, u_used, damping, cfg.dt).heat;
      b.residual = b.kinetic_change - b.work + b.heat;
      ledger &l = ledgers_[v];
      l.energy.add(b);
      l.energy.add_unallocated(e_mean, {x0[0], x0[1]}, {x1[0], x1[1]});
      l.max_step_residual = std::max(l.max_step_residual, std::fabs(b.residual));
      const double risk = cfg.dt * 4.0 * risk_at(c.agent, p.p);
      l.risk_cost += risk;
      // The same terms for nav_stats' CoPH ledger (D11), so its sums are the account's.
      coph_sample &t = terms_[v];
      t.step = 1;
      t.work = b.work;
      t.heat = b.heat;
      t.kinetic_change = b.kinetic_change;
      t.residual = b.residual;
      t.unallocated_work = e_mean[0] * (x1[0] - x0[0]) + e_mean[1] * (x1[1] - x0[1]);
      t.risk_cost = risk;
      x.u_used = u_used;
    }
    // sense: CoPH's 21 lattice points of the risk, and the cells the vehicle sees.
    coph_observation &o = x.observation;
    o.time = now;
    o.sensor_center = coph_vec2{x.x[0], x.x[1]};
    o.sensor_radius = cfg.sensor_radius;
    const double grid[] = {-1.0, -0.5, 0.0, 0.5, 1.0};
    const std::string prefix = std::to_string(cfg_.seed) + ":" + std::to_string(steps_) + ":";
    int j = 0;
    for (double a : grid)
      for (double b : grid) {
        if (!(a * a + b * b <= 1.0))
          continue;
        const coph_vec2 q{x.x[0] + a * cfg.sensor_radius, x.x[1] + b * cfg.sensor_radius};
        const double noise = cfg_.noise ? cfg_.noise(v, steps_, j) : 0.0;
        o.readings.push_back({prefix + std::to_string(j), now, q,
                              std::max(0.001, risk_at(c.agent, to_world(q)) + noise)});
        ++j;
      }
    x.obstacles = obstacles_near(c.agent, p.p);
    x.arrived = p.reached;
    c.used_sum = {0, 0};
    c.e_sum = {0, 0};
    c.held = 0;
  }
  const std::vector<std::optional<coph_vec2>> held = fleet_->boundary(steps_, now, in);
  for (int v = 0; v < n; ++v) {
    car &c = cars_[v];
    if (c.done)
      continue;
    if (held[v]) {
      c.u = *held[v];
      c.x_prev = in[v].x;
    } else {
      c.done = true;
      c.u = {0, 0};
      c.x_prev.reset();
    }
    const double jumps = fleet_->energy(v).shaping_jumps;
    terms_[v].shaping_jumps = jumps - ledgers_[v].energy.shaping_jumps;
    ledgers_[v].energy.shaping_jumps = jumps;
  }
  ++steps_;
}

// _Run.__call__: before each sim step.
void coph_driver::apply() {
  if (cars_.empty())
    return; // nothing attached: the sim's run is untouched
  terms_.assign(cars_.size(), coph_sample{});
  for (coph_sample &t : terms_)
    t.attached = 1;
  const std::vector<pose> poses = read();
  if (ticks_ % k_ == 0)
    coph_step(poses);
  const sim_world::config &sc = sim_.current_config();
  const int n = sim_.size();
  std::vector<float> vmax(n), a_max(n), wheelbase(n);
  sim_.vehicle_kinematics(vmax.data(), a_max.data(), wheelbase.data());
  const double dt = sc.veh.dt;
  const double dt_coph = dt / std::sqrt(cfg_.froude);
  for (std::size_t v = 0; v < cars_.size(); ++v) {
    car &c = cars_[v];
    const pose &p = poses[c.agent];
    const double accel_max = a_max[c.agent] / sc.scale; // world units / s^2
    if (c.done) {
      const double a = -std::clamp(p.speed / std::max(dt, 0.25), -accel_max, accel_max);
      sim_.override_command(c.agent, 0.0f, static_cast<float>(a * sc.scale));
      continue;
    }
    const coph_allocation al = coph_allocate(c.u, p.heading, p.speed, wheelbase[c.agent] / sc.scale,
                                             sc.veh.delta_max, accel_max, cfg_.speed_floor);
    sim_.override_command(c.agent, static_cast<float>(al.steer),
                          static_cast<float>(al.accel * sc.scale));
    c.used_sum = {c.used_sum[0] + al.used[0], c.used_sum[1] + al.used[1]};
    c.e_sum = {c.e_sum[0] + al.unallocated[0], c.e_sum[1] + al.unallocated[1]};
    ++c.held;
    ledger &l = ledgers_[v];
    terms_[v].e2 =
        (al.unallocated[0] * al.unallocated[0] + al.unallocated[1] * al.unallocated[1]) * dt_coph;
    terms_[v].u2 = (c.u[0] * c.u[0] + c.u[1] * c.u[1]) * dt_coph;
    l.e2 += terms_[v].e2;
    l.u2 += terms_[v].u2;
  }
  ++ticks_;
  // nav_stats' CoPH ledger, when the sim's nav_stats is armed (sim_world::feed_coph).
  if (sim_.nav_stats_armed()) {
    feed_.assign(static_cast<std::size_t>(sim_.size()), coph_sample{});
    for (std::size_t v = 0; v < cars_.size(); ++v)
      feed_[static_cast<std::size_t>(cars_[v].agent)] = terms_[v];
    sim_.feed_coph(feed_);
  }
}

} // namespace nav
} // namespace cvc
