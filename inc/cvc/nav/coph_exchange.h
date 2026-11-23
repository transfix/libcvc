/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// CoPH-Stagewise's packet exchange, in C++: C4 of docs/roadmap/NAV-COPH-ROADMAP.md (§4.4). The
// demos' learner (crossing_demo.CrossingLearner: maze_learning.MazeLearner on
// terrain_warning.TerrainLearner on cooperative.CooperativeLearner, in GRL-SNAM's grl_snam.coph)
// exports and assimilates in three layers, and this follows all three:
//
// - coph_export: CooperativeLearner.export fits the newest stage's field to its readings, with
//   an 8-member block bootstrap over update batches, and sends the field's value and gradient at
//   up to four supported candidates around the agent, with the ensemble's covariance.
//   TerrainLearner.export replaces those anchors with the four riskiest supported readings,
//   0.3 apart, and a second bootstrap's covariance. MazeLearner.export adds the geometry the
//   learner sensed (G9).
// - coph_assimilate: MazeLearner checks the geometry, CooperativeLearner checks the convention,
//   provenance, age, order and payload and stages the packet under the received cap, and
//   TerrainLearner keeps route evidence, the packets' anchors in 0.2 m cells (G10).
// - coph_fit_messages: CooperativeLearner._field_fit's legacy receiver, one whitened Gauss-Newton
//   step per stage from the received anchors; coph_update commits it (G10).
//
// The bootstrap's resample indices are inputs (D8): GRL-SNAM's golden injects the Python's own,
// and in production coph_draw_blocks draws them from a seeded std::mt19937 with
// detail::pick_below, the same numbers on every standard library (the Python seeds numpy's
// generator from a hash, which this does not port).
//
// Not here: observe's packet-side bookkeeping (MazeLearner's local geometry reports,
// TerrainLearner's expiring route evidence), the rate decision and the fleet's sending, which
// come with the fleet driver (C5). The state is the dynamic=False learner's: no temporal window.
//
// coph_exchange.cpp is compiled with -ffp-contract=off.

#ifndef CVC_NAV_COPH_EXCHANGE_H
#define CVC_NAV_COPH_EXCHANGE_H

#include <array>
#include <cstdint>
#include <cvc/nav/coph_learner.h>
#include <cvc/nav/coph_packet.h>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace nav {

// ---------------------------------------------------------------------------
// G9: export
// ---------------------------------------------------------------------------

// The resample indices of one bootstrap: for each ensemble member, as many block indices as
// there are blocks, drawn with replacement.
using coph_draws = std::vector<std::vector<int>>;

// cooperative.bootstrap_field's result. The readings are the stage's with j % 5 != 0, in order,
// grouped into blocks by int(time / (dt update_every)) in order of first appearance. fit(rs)
// solves (A^T A + fit_ridge I) w = A^T y + fit_ridge w0, with A the RBF features of rs,
// y = log(expm1(4 clip(value, 0.001, 20))) and w0 = -1/m each. mean = fit(every reading); member
// k = fit(the readings of blocks draws[k][0], draws[k][1], ... in that order).
struct coph_bootstrap {
  std::vector<std::pair<long long, int>> blocks; // [key, readings], in order of first appearance
  std::vector<double> mean;
  std::vector<std::vector<double>> ensemble;
};

// cooperative.field_vector: (value, gx, gy) of G3's risk (softplus(phi . w) / 4) and its
// gradient at each point, flattened, on the stage's RBF centres with the given weights.
std::vector<double> coph_field_vector(const coph_stage &stage, const std::vector<double> &weights,
                                      const coph_config &cfg, const std::vector<coph_vec2> &points);

// How many blocks the newest stage's export readings make (0 when it has none).
int coph_bootstrap_blocks(const coph_learner_state &s);

// bootstrap_field on a stage with the given draws (one list per member). Throws
// std::invalid_argument when a draw names no block.
coph_bootstrap coph_bootstrap_field(const coph_stage &stage, const coph_config &cfg,
                                    const coph_draws &draws);

// members x blocks indices in [0, blocks), drawn from rng with detail::pick_below.
coph_draws coph_draw_blocks(std::mt19937 &rng, int blocks, int members);

// What coph_export computed on the way (G9 records it).
struct coph_export_trace {
  coph_bootstrap cooperative; // draws[0]
  coph_bootstrap terrain;     // draws[1]; empty when TerrainLearner had nothing to replace
  std::vector<std::pair<coph_vec2, bool>> candidates; // around x, and whether the stage supports it
  std::vector<coph_anchor> cooperative_anchors;
  std::vector<std::vector<double>> cooperative_covariance;
  std::vector<const coph_reading *> picked; // TerrainLearner's anchor readings
  std::vector<coph_anchor> terrain_anchors;
  std::vector<std::vector<double>> terrain_covariance;
  std::string path; // "terrain", or "cooperative" when no reading qualified
};

// CrossingLearner.export at x to recipient: CooperativeLearner's packet with draws[0], whose
// anchors and covariance TerrainLearner replaces using draws[1], with MazeLearner's geometry.
// nullopt when the newest stage has no export readings or supports none of the candidates (the
// Python returns None). H uses G3's potential at q with the bootstrap's mean field, the target
// (navigation_target, else the goal), the policy and the geometry store; V_task = |q -
// target|^2 / 2. The trace, when given, gets the intermediate results.
std::optional<coph_packet> coph_export(const coph_learner_state &s, const coph_state &x,
                                       const std::string &recipient,
                                       const std::array<coph_draws, 2> &draws,
                                       coph_export_trace *trace = nullptr);

// Whether coph_export at x makes a packet: the newest stage has export readings and supports one
// of the candidates. It does not depend on the draws, so a caller can ask before drawing them.
bool coph_can_export(const coph_learner_state &s, const coph_state &x);

// The same, with both bootstraps' indices drawn from rng (D8).
std::optional<coph_packet> coph_export(const coph_learner_state &s, const coph_state &x,
                                       const std::string &recipient, std::mt19937 &rng);

// ---------------------------------------------------------------------------
// G10: assimilate
// ---------------------------------------------------------------------------

// What assimilate returns (CooperativeLearner's event, with TerrainLearner's scope).
struct coph_packet_event {
  std::string packet; // the packet's identity
  std::string sender;
  std::string recipient; // this learner
  double time = 0;
  // eligible, incompatible_geometry, invalid_geometry, incompatible_convention,
  // returned_evidence, wrong_recipient, invalid_provenance, invalid_age, duplicate_or_reordered,
  // invalid_payload, invalid_support, unsupported (never returned: route evidence or refused),
  // or invalid_remote_support.
  std::string reason;
  // For an eligible packet: retained_and_remote_route (staged for the field fit and kept as
  // route evidence), or remote_route_only (none of its anchors is in a kept stage). Else empty.
  std::string scope;
  std::int64_t version_before = 0;
};

// CrossingLearner.assimilate(m): MazeLearner, then TerrainLearner, then CooperativeLearner.
// packet_rules in the golden states the checks in order. An eligible packet replaces its
// sender's earlier one in received, updates the route evidence, cells and terrain anchors
// (coph_refresh_terrain), and adds its geometry to the remote reports (coph_refresh_geometry).
coph_packet_event coph_assimilate(coph_learner_state &s, const coph_packet &m);

// CooperativeLearner._expire: drops each received packet that expired before now, or none of
// whose anchors is in a kept stage.
void coph_expire(coph_learner_state &s);

// TerrainLearner.refresh_terrain: drops the cells that expired before now; the navigation
// context's terrain anchors become the cells' anchors, in order, then the local model anchors.
void coph_refresh_terrain(coph_learner_state &s);

// MazeLearner.refresh_geometry: drops the remote reports that expired before now;
// reported_geometry becomes the sorted union of the local and remote shapes; those that intersect
// a kept stage join the geometry store (sorted, at most obstacle_cap, overflow flagged), and each
// stage's obstacles are the store's shapes that intersect it.
void coph_refresh_geometry(coph_learner_state &s);

// coph_message_fit (coph_learner.h) for each kept stage, from G5's fit (coph_field_fit).
std::vector<coph_message_fit> coph_fit_messages(const coph_learner_state &s,
                                                const std::vector<coph_stage_fit> &fit);

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_COPH_EXCHANGE_H
