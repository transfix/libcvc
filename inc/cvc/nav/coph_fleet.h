/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A fleet of CoPH agents and the world the demos run them in: part of C5 of
// docs/roadmap/NAV-COPH-ROADMAP.md (§4.5).
//
// coph_fleet is the demos' runner (GRL-SNAM's grl_snam.coph.staggered_demo.execute) for agents
// that start together, over a coph_channel (D10: it lives outside sim_world). At each boundary,
// for every agent still running, in order:
//   1. the held step completes: the agent observes the world with the transition that led there
//      (just observes at the first boundary); it is done on a hit, within 0.25 of its goal, or at
//      cfg.steps; else it advances;
//   2. exports: at every update_every-th boundary each sending agent exports to each other agent
//      still running, through the channel (payload bytes count against the byte budget);
//   3. each agent takes what reached it by now, in order of (timestamp, identity), updates when
//      the boundary is an update boundary or a packet arrived, and acts: the force it holds to
//      the next boundary.
// The world is the caller's: coph_crossing_world is the crossing demo's, and run() drives a
// fleet through it; a vehicle driver steps its plants between boundaries and hands the fleet
// what they sensed (the fleet's boundary()).
//
// Where it differs from the Python runner: the runner's status replies and acknowledgements (which
// cost bytes but never bind at the demos' budgets) and its evaluation audits are not modelled.
//
// coph_fleet.cpp is compiled with -ffp-contract=off.

#ifndef CVC_NAV_COPH_FLEET_H
#define CVC_NAV_COPH_FLEET_H

#include <array>
#include <cstdint>
#include <cvc/nav/coph_agent.h>
#include <cvc/nav/coph_channel.h>
#include <functional>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

// The crossing demo's world (crossing_demo.CrossingWorld) for several agents: each a unit point
// mass in the exact flow at the world's damping; a step hits when the motion comes within 0.12 of
// an obstacle (a door's slab_sweep, or a disk's surface) or leaves the corridors
// (maze_geometry.corridor_clear, margin 0.12). The sensor reads the risk at the 21 lattice points
// x + sensor_radius (a, b), a, b in linspace(-1, 1, 5), a^2 + b^2 <= 1: the risk plus the noise,
// at least 0.001, with uid '<world seed>:<tick>:<j>'; it sees the obstacles within sensor_radius.
// With an empty network and risk patches it is CooperativeWorld's open field, where the cars'
// learners run (GRL-SNAM's G13): no corridor bounds it.
struct coph_crossing_world_config {
  coph_config cfg;
  double damping = 0.85;
  std::vector<coph_shape> obstacles; // the doors
  coph_network network = coph_crossing_network();
  std::vector<long long> seeds; // each agent's world seed (its readings' uid prefix)
  double base_risk = 0.02;
  // Risk patches (x, y, sx, sy, amplitude): each adds amplitude exp(-((qx - x)^2 / sx^2 + (qy -
  // y)^2 / sy^2) / 2) to base_risk (CooperativeWorld.risk_truth).
  std::vector<std::array<double, 5>> patches;
  // The sensor noise of reading j at a tick; none when unset.
  std::function<double(int agent, long long tick, int j)> noise;
};

class coph_crossing_world {
public:
  explicit coph_crossing_world(coph_crossing_world_config cfg);
  const coph_crossing_world_config &config() const { return cfg_; }

  struct step_result {
    coph_state after{};
    bool hit = false;
  };
  step_result step(const coph_state &x, const coph_vec2 &u) const;

  struct sensed {
    coph_observation observation;
    std::vector<coph_shape> obstacles;
  };
  sensed sense(int agent, const coph_state &x, long long tick, double now) const;

private:
  coph_crossing_world_config cfg_;
};

// Where an export's bootstrap indices come from (D8): the exporting agent and its state, asked only
// for an export that makes a packet (coph_can_export). Unset, the fleet draws them from its own
// seeded std::mt19937.
using coph_draw_source =
    std::function<std::array<coph_draws, 2>(const coph_agent &agent, const coph_state &x)>;

struct coph_fleet_config {
  coph_config cfg;                // every agent's
  coph_communication_config comm; // every agent's, and the byte budget
  coph_network network = coph_crossing_network();
  std::vector<int> senders; // the agents that export (the demos' reciprocal arm: all)
  double communication_start_s = 0.0;
  std::uint32_t seed = 0; // the fleet's own draws, when no draw source is given
};

class coph_fleet {
public:
  struct agent_spec {
    coph_state initial{};
    coph_vec2 goal{};
  };
  // Agent k is '<k>'. Throws std::invalid_argument when the configuration is refused or outside
  // the demos' profile. The channel defaults to a coph_ideal_channel with no delay or loss.
  coph_fleet(coph_fleet_config cfg, const std::vector<agent_spec> &agents,
             std::shared_ptr<coph_channel> channel = nullptr, coph_draw_source draws = {});

  // What one agent's world reports at a boundary: its state, whether the step that ended there
  // hit, and what it sensed.
  struct input {
    coph_state x{};
    bool hit = false;
    coph_observation observation;
    std::vector<coph_shape> obstacles;
    // A vehicle's: the force its plant took over the held step (D5), which the transition carries
    // in place of the held force; whether its world says it arrived, in place of the goal radius;
    // and a new goal, taken before it observes (D12's sliding local goal).
    std::optional<coph_vec2> u_used;
    std::optional<bool> arrived;
    std::optional<coph_vec2> goal;
  };
  // One boundary at `tick`, time `now` (the runner's clock, (tick dt_us) / 1e6 with dt_us =
  // round(dt 1e6)); one input per agent, read only for agents still running. Returns each agent's
  // held force, empty for an agent that is done.
  std::vector<std::optional<coph_vec2>> boundary(long long tick, double now,
                                                 const std::vector<input> &in);

  // Run on the crossing world from tick 0 until every agent is done.
  void run(const coph_crossing_world &world);

  const std::vector<coph_agent> &agents() const { return agents_; }
  bool done(int agent) const { return done_.at(agent); }
  const coph_state &state(int agent) const { return x_.at(agent); }

  // What happened, as the runner records it.
  struct sample {
    double time = 0;
    coph_state x{};
    std::int64_t stage = 0, version = 0;
    bool done = false;
  };
  struct control {
    long long tick = 0;
    coph_vec2 u{};
    coph_control_meta meta;
  };
  struct update_record {
    long long tick = 0;
    std::vector<std::string> packets; // delivered at this boundary
    coph_update_report report;
  };
  struct event {
    std::string kind; // activation, sent, budget_skip or delivered
    int sender = -1, receiver = -1;
    std::string packet, reason;
    std::size_t bytes = 0;
    std::optional<std::int64_t> version_after; // delivered: the receiver's, after its update
    std::optional<bool> accepted;
    double time = 0; // the boundary's
  };
  const std::vector<sample> &samples(int agent) const { return samples_.at(agent); }
  const std::vector<control> &controls(int agent) const { return controls_.at(agent); }
  const std::vector<update_record> &updates(int agent) const { return updates_.at(agent); }
  const std::vector<event> &events() const { return events_; }
  bool success(int agent) const { return success_.at(agent); }
  bool hit(int agent) const { return hit_.at(agent); }
  // Each agent's energy account (§4.2): the shaping jumps of its commits, and, when run() drives
  // the plant, each step's balance at the world's damping. A vehicle driver adds its own plant's
  // terms.
  const coph_energy &energy(int agent) const { return energy_.at(agent); }
  coph_energy &energy(int agent) { return energy_.at(agent); }

private:
  coph_fleet_config cfg_;
  std::vector<coph_agent> agents_;
  std::shared_ptr<coph_channel> channel_;
  coph_draw_source draws_;
  std::mt19937 rng_;
  std::vector<coph_state> x_;
  std::vector<bool> active_, done_, hit_, success_;
  struct held {
    coph_state x{};
    coph_vec2 u{};
    std::int64_t version = 0;
  };
  std::vector<std::optional<held>> held_;
  std::vector<std::size_t> sent_bytes_;
  std::vector<std::vector<sample>> samples_;
  std::vector<std::vector<control>> controls_;
  std::vector<std::vector<update_record>> updates_;
  std::vector<event> events_;
  std::vector<coph_energy> energy_;
};

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_COPH_FLEET_H
