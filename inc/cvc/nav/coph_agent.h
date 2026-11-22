/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// One CoPH agent, whole: part of C5 of docs/roadmap/NAV-COPH-ROADMAP.md (§4.5). coph_agent is the
// demos' learner (crossing_demo.CrossingLearner: maze_learning.MazeLearner on
// terrain_warning.TerrainLearner on cooperative.CooperativeLearner on learner.Learner, in
// GRL-SNAM's grl_snam.coph) as one object, over C1's ledger (coph_ledger.h), C3's update
// (coph_learner.h) and C4's packets (coph_exchange.h):
//
// - observe: CooperativeLearner prefixes the readings' uids with the agent id; Learner.observe
//   (navigation.observe_geometry: the footprints, then the geometry store: what intersects a kept
//   stage or lies within the stopping distance, nearest first; then the readings and the
//   transition, C1's ledger); the received packets expire; TerrainLearner drops expired route
//   evidence; MazeLearner keeps one report per sensed shape (at most 32) and refreshes the
//   geometry.
// - advance: MazeLearner.advance (C1's ledger), then the geometry refresh.
// - action: Learner.action, the routed control (C3's coph_control) on the committed weights, in
//   the live navigation context, which keeps the route and the waypoint between steps.
// - update: CooperativeLearner.update counts the guard steps of a diagnostic control before and
//   after Learner.update (C3's coph_update, with C4's Gauss-Newton step on the held packets); then
//   TerrainLearner rebuilds its own terrain anchors from the committed fields, one per 0.2 m cell,
//   at most 256.
// - export_to and assimilate: C4's, on this agent's state.
//
// coph_agent.cpp is compiled with -ffp-contract=off.

#ifndef CVC_NAV_COPH_AGENT_H
#define CVC_NAV_COPH_AGENT_H

#include <array>
#include <cstdint>
#include <cvc/nav/coph_exchange.h>
#include <cvc/nav/coph_learner.h>
#include <cvc/nav/coph_ledger.h>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

// A shape this agent sensed for the first time, or learned of from a packet (MazeLearner's
// geometry_events).
struct coph_geometry_event {
  std::string kind; // locally_sensed or received_geometry
  double time = 0;
  coph_shape obstacle;
  std::string source; // the report's source (locally_sensed) or the packet's identity
};

class coph_agent {
public:
  // A fresh learner at initial, heading for goal, on the surveyed network. Throws
  // std::invalid_argument when cfg or comm is refused or outside the demos' profile (D7).
  coph_agent(const coph_config &cfg, const coph_communication_config &comm,
             const coph_state &initial, const coph_vec2 &goal, std::string agent_id,
             coph_network network);

  const std::string &id() const { return s_.agent_id; }
  // The learner's state between calls: the ledger's stages and store, the committed values and
  // what it holds of packets.
  const coph_learner_state &state() const { return s_; }
  const coph_ledger &ledger() const { return ledger_; }
  const std::vector<coph_geometry_event> &geometry_events() const { return geometry_events_; }
  const coph_control_meta &last_control() const { return last_control_; }

  // The learner's goal from now on (a car's sliding local goal, D12): the routing, the packets and
  // the stages it enters next take it.
  void set_goal(const coph_vec2 &goal) {
    s_.goal = goal;
    ledger_.set_goal(goal);
  }

  // CrossingLearner.observe: the observation (its readings carry the world's uids, which the
  // agent prefixes with '<id>:'), the shapes the sensor sees, and the transition that led here.
  void observe(const coph_observation &observation, const std::vector<coph_shape> &obstacles,
               const coph_transition *transition = nullptr);

  // MazeLearner.advance at x; whether a new stage began.
  bool advance(const coph_state &x);

  // Learner.action at x: the force to hold until the next step (the routed control's), with its
  // record in last_control().
  coph_vec2 action(const coph_state &x);

  // CrossingLearner.update at x. A commit reaches the ledger.
  coph_update_report update(const coph_state &x);

  // CrossingLearner.export at x to recipient, with the bootstraps' indices given, or drawn from
  // rng (D8).
  std::optional<coph_packet> export_to(const coph_state &x, const std::string &recipient,
                                       const std::array<coph_draws, 2> &draws);
  std::optional<coph_packet> export_to(const coph_state &x, const std::string &recipient,
                                       std::mt19937 &rng);

  // CrossingLearner.assimilate (coph_assimilate), keeping the ledger's geometry in step.
  coph_packet_event assimilate(const coph_packet &m);

private:
  void sync_stages();
  // A car's learner (no network): CooperativeLearner on Learner, routing with the profile's
  // own planner.
  bool car() const { return s_.network.nodes.empty(); }
  void refresh_geometry();
  coph_vec2 routed_control(const coph_state &x);

  coph_ledger ledger_;
  coph_learner_state s_;
  coph_state current_state_{};
  coph_route_context navigation_; // the live context: route, blocked edges, waypoint
  int navigation_step_ = 0;
  coph_control_meta last_control_;
  std::vector<coph_geometry_event> geometry_events_;
};

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_COPH_AGENT_H
