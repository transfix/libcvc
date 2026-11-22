/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A fleet of CoPH agents and the crossing world (C5 of docs/roadmap/NAV-COPH-ROADMAP.md, §4.5);
// see coph_fleet.h. Built with -ffp-contract=off.

#include <algorithm>
#include <cmath>
#include <cvc/nav/coph_fleet.h>
#include <cvc/nav/coph_model.h>
#include <cvc/nav/detail/coph_math.h>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace nav {

namespace {

double distance(const coph_vec2 &a, const coph_vec2 &b) {
  const double dx = a[0] - b[0], dy = a[1] - b[1];
  return std::sqrt(dx * dx + dy * dy);
}

} // namespace

// ---------------------------------------------------------------------------
// The crossing world
// ---------------------------------------------------------------------------

coph_crossing_world::coph_crossing_world(coph_crossing_world_config cfg) : cfg_(std::move(cfg)) {
  if (!(cfg_.damping > 0))
    throw std::invalid_argument("coph_crossing_world: the damping must be positive");
}

coph_crossing_world::step_result coph_crossing_world::step(const coph_state &x,
                                                           const coph_vec2 &u) const {
  step_result r;
  r.after = coph_flow<double>(x, u, cfg_.damping, cfg_.cfg.dt);
  const coph_vec2 a{x[0], x[1]}, b{r.after[0], r.after[1]};
  const double s0 = b[0] - a[0], s1 = b[1] - a[1];
  for (const coph_shape &o : cfg_.obstacles) {
    if (o.slab) {
      r.hit = r.hit || coph_slab_sweep(a, b, o) <= 0.12;
      continue;
    }
    const double t = std::min(
        std::max(((o.x - a[0]) * s0 + (o.y - a[1]) * s1) / ((s0 * s0 + s1 * s1) + 1e-20), 0.0),
        1.0);
    r.hit = r.hit || distance({a[0] + t * s0, a[1] + t * s1}, {o.x, o.y}) <= o.r + 0.12;
  }
  if (!cfg_.network.nodes.empty())
    r.hit = r.hit || !coph_corridor_clear(a, b, cfg_.network, 0.12);
  return r;
}

coph_crossing_world::sensed coph_crossing_world::sense(int agent, const coph_state &x,
                                                       long long tick, double now) const {
  sensed out;
  coph_observation &o = out.observation;
  o.time = now;
  o.sensor_center = coph_vec2{x[0], x[1]};
  o.sensor_radius = cfg_.cfg.sensor_radius;
  const double grid[] = {-1.0, -0.5, 0.0, 0.5, 1.0}; // numpy.linspace(-1, 1, 5)
  const std::string prefix =
      std::to_string(cfg_.seeds.at(agent)) + ":" + std::to_string(tick) + ":";
  int j = 0;
  for (double a : grid)
    for (double b : grid) {
      if (!(a * a + b * b <= 1.0))
        continue;
      const coph_vec2 q{x[0] + a * cfg_.cfg.sensor_radius, x[1] + b * cfg_.cfg.sensor_radius};
      const double n = cfg_.noise ? cfg_.noise(agent, tick, j) : 0.0;
      double risk = cfg_.base_risk;
      for (const auto &[px, py, sx, sy, amplitude] : cfg_.patches) {
        const double zx = (q[0] - px) / sx, zy = (q[1] - py) / sy;
        risk = risk + amplitude * std::exp(-0.5 * (zx * zx + zy * zy));
      }
      o.readings.push_back({prefix + std::to_string(j), now, q, std::max(0.001, risk + n)});
      ++j;
    }
  for (const coph_shape &sh : cfg_.obstacles) {
    const double gap = sh.slab ? detail::coph_slab_distance({x[0], x[1]}, sh)
                               : distance({x[0], x[1]}, {sh.x, sh.y}) - sh.r;
    if (gap <= cfg_.cfg.sensor_radius)
      out.obstacles.push_back(sh);
  }
  return out;
}

// ---------------------------------------------------------------------------
// The fleet
// ---------------------------------------------------------------------------

coph_fleet::coph_fleet(coph_fleet_config cfg, const std::vector<agent_spec> &agents,
                       std::shared_ptr<coph_channel> channel, coph_draw_source draws)
    : cfg_(std::move(cfg)), channel_(std::move(channel)), draws_(std::move(draws)),
      rng_(cfg_.seed) {
  if (!channel_)
    channel_ = std::make_shared<coph_ideal_channel>();
  for (std::size_t k = 0; k < agents.size(); ++k) {
    agents_.emplace_back(cfg_.cfg, cfg_.comm, agents[k].initial, agents[k].goal, std::to_string(k),
                         cfg_.network);
    x_.push_back(agents[k].initial);
  }
  const std::size_t n = agents.size();
  active_.assign(n, false);
  done_.assign(n, false);
  hit_.assign(n, false);
  success_.assign(n, false);
  held_.assign(n, std::nullopt);
  sent_bytes_.assign(n, 0);
  samples_.resize(n);
  controls_.resize(n);
  updates_.resize(n);
  energy_.resize(n);
}

std::vector<std::optional<coph_vec2>> coph_fleet::boundary(long long tick, double now,
                                                           const std::vector<input> &in) {
  const std::size_t n = agents_.size();
  if (in.size() != n)
    throw std::invalid_argument("coph_fleet::boundary: one input per agent");
  const coph_config &cfg = cfg_.cfg;
  std::vector<bool> running(n);
  for (std::size_t i = 0; i < n; ++i)
    running[i] = !done_[i];

  // 1. The held steps complete; every agent senses.
  for (std::size_t i = 0; i < n; ++i) {
    if (!running[i])
      continue;
    coph_agent &a = agents_[i];
    if (held_[i]) {
      const held &h = *held_[i];
      const coph_transition tr{tick - 1, now, h.x, h.u, in[i].x, h.version};
      x_[i] = in[i].x;
      a.observe(in[i].observation, in[i].obstacles, &tr);
      hit_[i] = in[i].hit;
      success_[i] = distance({x_[i][0], x_[i][1]}, a.state().goal) < 0.25;
      done_[i] = hit_[i] || success_[i] || tick >= cfg.steps;
      if (!done_[i])
        a.advance(x_[i]);
    } else {
      active_[i] = true;
      x_[i] = in[i].x;
      a.observe(in[i].observation, in[i].obstacles);
      events_.push_back({"activation", static_cast<int>(i), -1, "", "", 0, {}, {}, now});
    }
    samples_[i].push_back({now, x_[i], a.state().stages.back().id, a.state().version, done_[i]});
  }

  // 2. Exports, on each sender's update cadence.
  if (now + 1e-9 >= cfg_.communication_start_s && tick % cfg.update_every == 0) {
    for (std::size_t i = 0; i < n; ++i) {
      if (!running[i] || done_[i] ||
          std::find(cfg_.senders.begin(), cfg_.senders.end(), static_cast<int>(i)) ==
              cfg_.senders.end())
        continue;
      for (std::size_t j = 0; j < n; ++j) {
        if (j == i || done_[j] || !active_[j])
          continue;
        coph_agent &a = agents_[i];
        const std::string to = std::to_string(j);
        // A draw source is asked only for an export that makes a packet.
        if (draws_ && !coph_can_export(a.state(), x_[i]))
          continue;
        const std::optional<coph_packet> p =
            draws_ ? a.export_to(x_[i], to, draws_(a, x_[i])) : a.export_to(x_[i], to, rng_);
        if (!p)
          continue;
        const std::size_t bytes = p->bytes();
        if (sent_bytes_[i] + bytes > static_cast<std::size_t>(cfg_.comm.byte_budget)) {
          events_.push_back({"budget_skip",
                             static_cast<int>(i),
                             static_cast<int>(j),
                             p->identity,
                             "",
                             bytes,
                             {},
                             {},
                             now});
          continue;
        }
        sent_bytes_[i] += bytes;
        channel_->send(static_cast<int>(i), static_cast<int>(j),
                       std::make_shared<const coph_packet>(*p), bytes, now);
        events_.push_back({"sent",
                           static_cast<int>(i),
                           static_cast<int>(j),
                           p->identity,
                           "",
                           bytes,
                           {},
                           {},
                           now});
      }
    }
  }

  // 3. Deliveries, updates, actions.
  std::vector<std::optional<coph_vec2>> out(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (!running[i] || done_[i])
      continue;
    coph_agent &a = agents_[i];
    std::vector<coph_delivery> due = channel_->poll(static_cast<int>(i), now);
    std::stable_sort(due.begin(), due.end(), [](const coph_delivery &p, const coph_delivery &q) {
      if (p.packet->timestamp != q.packet->timestamp)
        return p.packet->timestamp < q.packet->timestamp;
      return p.packet->identity < q.packet->identity;
    });
    std::vector<std::string> delivered;
    std::vector<std::size_t> event_index;
    for (const coph_delivery &d : due) {
      const coph_packet_event ev = a.assimilate(*d.packet);
      delivered.push_back(d.packet->identity);
      event_index.push_back(events_.size());
      events_.push_back({"delivered",
                         d.from,
                         static_cast<int>(i),
                         d.packet->identity,
                         ev.reason,
                         d.bytes,
                         {},
                         {},
                         now});
    }
    if (tick % cfg.update_every == 0 || !delivered.empty()) {
      update_record r{tick, delivered, a.update(x_[i])};
      energy_[i].add_shaping_jump(r.report.shaping_jump);
      for (std::size_t k : event_index) {
        events_[k].version_after = a.state().version;
        events_[k].accepted = r.report.accepted;
      }
      updates_[i].push_back(std::move(r));
    }
    const coph_vec2 u = a.action(x_[i]);
    controls_[i].push_back({tick, u, a.last_control()});
    held_[i] = held{x_[i], u, a.state().version};
    out[i] = u;
  }
  return out;
}

void coph_fleet::run(const coph_crossing_world &world) {
  const std::size_t n = agents_.size();
  const long long dt_us = std::llround(cfg_.cfg.dt * 1e6);
  for (long long tick = 0;; ++tick) {
    if (std::all_of(done_.begin(), done_.end(), [](bool d) { return d; }))
      return;
    const double now = static_cast<double>(tick * dt_us) / 1e6;
    std::vector<input> in(n);
    for (std::size_t i = 0; i < n; ++i) {
      if (done_[i])
        continue;
      in[i].x = x_[i];
      if (held_[i]) {
        const coph_crossing_world::step_result s = world.step(x_[i], held_[i]->u);
        energy_[i].add(coph_balance(x_[i], held_[i]->u, world.config().damping, cfg_.cfg.dt));
        in[i].x = s.after;
        in[i].hit = s.hit;
      }
      coph_crossing_world::sensed obs = world.sense(static_cast<int>(i), in[i].x, tick, now);
      in[i].observation = std::move(obs.observation);
      in[i].obstacles = std::move(obs.obstacles);
    }
    boundary(tick, now, in);
  }
}

} // namespace nav
} // namespace cvc
