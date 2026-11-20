/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// CoPH-Stagewise's learner, in C++: C3 of docs/roadmap/NAV-COPH-ROADMAP.md (§4.3). It is the update
// of the demos' own learner, crossing_demo.CrossingLearner (maze_learning.MazeLearner on
// terrain_warning.TerrainLearner on cooperative.CooperativeLearner on learner.Learner, in
// GRL-SNAM's grl_snam.coph), in the demos' profile:
//
//   coph_field_fit        Learner._field_fit: the ridge fit of each kept stage's field (G5).
//   coph_fit_damping      Learner._fit_damping: Newton steps on the plant loss, on hyper-duals
//   (G6). coph_maze_route       maze_learning.maze_route: the route on the surveyed corridor
//   network. coph_control          navigation.guarded_control: the routed command, its CBF
//   projection and
//                         the guard, with maze_graph's corridor checks.
//   coph_prepare          replay_budget.prepare: each stage's replay horizon and the budget.
//   coph_replay           stage_prediction.retained_replay: the replay cost, and on duals its
//                         gradient in the policy (G7).
//   coph_update           Learner.update: the fits, the policy step and its halvings, validation,
//                         and the commit or rollback (G8).
//
// The learner's state is plain data (coph_learner_state): the kept stages as C1's ledger holds
// them, the geometry store, the navigation context the maze router reads (the network, the
// reported geometry, the terrain anchors), and the committed damping, policy and version. The
// fleet driver (C5) owns the ledger and hands the state in.
//
// The demos' profile only (D7): routed control with progress shaping, the mission objective, the
// retained replay domain, supported_boundary horizons with an update cap, the 'spatial' stage
// mode, no receiver fit (C4 adds packets) and no local residual.
//
// GRL-SNAM's golden (grl_snam.tools.coph_golden, committed as src/cvc/tests/data/coph_golden.json)
// pins G5-G8 at three update snapshots of a scripted crossing run; its learner_rules text states
// the rules. Values are compared at rtol 1e-9, atol 1e-12, and decisions exactly.
// coph_learner.cpp is compiled with -ffp-contract=off.

#ifndef CVC_NAV_COPH_LEARNER_H
#define CVC_NAV_COPH_LEARNER_H

#include <array>
#include <cstdint>
#include <cvc/nav/coph_config.h>
#include <cvc/nav/coph_dual.h>
#include <cvc/nav/coph_ledger.h>
#include <cvc/nav/coph_model.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

// The surveyed corridor network (maze_geometry.network): nodes, edges between them, and the
// corridor's radius about each edge.
struct coph_network {
  std::vector<coph_vec2> nodes;
  std::vector<std::array<int, 2>> edges;
  double corridor_radius = 0.85; // maze_geometry.CORRIDOR_RADIUS
};

// The crossing demo's network (crossing_demo.NODES, EDGES): west and east columns at x = 0 and
// 12, joined by rows at y = 0, 4 and 7.5.
coph_network coph_crossing_network();

// A terrain anchor (terrain_warning): a first-order risk estimate value + g . (p - (x, y)),
// supported within 0.65 of (x, y).
struct coph_anchor {
  double x = 0, y = 0, value = 0, gx = 0, gy = 0;
};

// What the maze router reads of a navigation context, and the route it leaves there.
struct coph_route_context {
  std::vector<coph_shape> reported_geometry; // closures: shapes reported by sensors or packets
  std::vector<coph_anchor> terrain_anchors;
  double risk_weight = 1.0;  // the agent's own price of risk
  double unknown_risk = 1.0; // the risk of terrain no anchor supports
  std::optional<coph_vec2> target;
  std::vector<int> route_nodes;
  std::vector<int> blocked_edges;
};

// One learner's state, between updates.
struct coph_learner_state {
  coph_config cfg;
  coph_vec2 goal{};
  coph_network network;
  std::vector<coph_stage> stages;   // the kept stages, oldest first (coph_ledger::stages)
  std::vector<coph_shape> geometry; // the geometry store (coph_ledger::geometry)
  bool geometry_overflow = false;
  std::vector<coph_shape> reported_geometry; // the navigation context's
  std::vector<coph_anchor> terrain_anchors;  // the navigation context's
  double damping = 0.6;
  coph_vec2 policy{1.0, 2.0};
  std::int64_t version = 0;
  std::int64_t counterfactual_steps = 0; // replay and pilot steps, guard steps included
};

// A reading whose uid ends in a multiple of 5 is held out for validation.
bool coph_is_validation(const std::string &uid);

// ---------------------------------------------------------------------------
// G5: the field fit
// ---------------------------------------------------------------------------

// One stage's ridge fit. Its rows are its readings that are not held out, in order: the RBF
// features phi_m(q) = exp(-0.5 |(q - c_m) / rbf_width|^2) and the target log(expm1(4 max(value,
// 0.001))). normal = A^T A + fit_ridge I and rhs = A^T y + fit_ridge w (w the committed weights);
// weights = normal^-1 rhs, covariance = normal^-1 (the design's curvature, not a calibrated
// posterior). The profile has no continuity rows, so the stages' systems are independent.
struct coph_stage_fit {
  int rows = 0;
  std::vector<double> targets;
  std::vector<double> normal; // m x m, row-major
  std::vector<double> rhs;
  std::vector<double> weights;
  std::vector<double> covariance; // m x m, row-major
};
std::vector<coph_stage_fit> coph_field_fit(const coph_learner_state &s);

// Learner._field_validation: the mean of (risk(q) - value)^2 over every kept stage's held-out
// readings, with weights[k] for stage k; 0 with none.
double coph_field_validation(const coph_learner_state &s,
                             const std::vector<std::vector<double>> &weights);

// ---------------------------------------------------------------------------
// G6: the damping fit
// ---------------------------------------------------------------------------

// Learner._plant_loss over the kept stages' transitions (oldest stage first) with uid % 5 != 0,
// or == 0 for validation: the mean over records and components of (flow(x, u, d, dt) - after)^2,
// plus 0.05 times the mean of (delta - dt power)^2 with delta = (|v1|^2 - |v0|^2) / 2 and power =
// u . (v0 + v1) / 2 - d (|v0|^2 + |v1|^2) / 2. 0 with no records.
double coph_plant_loss(const coph_learner_state &s, double damping, bool validation);

struct coph_damping_iterate {
  double damping = 0, loss = 0, gradient = 0, curvature = 0, step = 0, next = 0;
};
// From the committed damping, solver_steps times: step = d - g / (|h| + 1e-5) with g and h the
// loss's first and second derivatives (a hyper-dual evaluation), next = clamp(step, 0.05, 3).
// With fewer than 3 records the result is the committed damping.
struct coph_damping_fit {
  int transitions = 0;
  std::vector<coph_damping_iterate> iterates;
  double result = 0;
};
coph_damping_fit coph_fit_damping(const coph_learner_state &s);

// update's reconciliation: clamp(result, max(damping_bounds[0], committed - damping_trust),
// min(damping_bounds[1], committed + damping_trust)).
double coph_reconcile_damping(double result, double committed, const coph_config &cfg);

// ---------------------------------------------------------------------------
// The routed controller
// ---------------------------------------------------------------------------

// A replay or pilot left the supported domain: unsupported_state, unavailable_stage,
// unsupported_overlap, safety_infeasible, unsupported_motion, post_boundary_cap,
// pilot_safety_infeasible or pilot_domain_exit.
struct coph_domain_exit : std::runtime_error {
  using std::runtime_error::runtime_error;
};

// One stage frozen for prediction (stage_prediction.StageModel): its field (the fitted weights in
// an update), its obstacles and its support, the distinct positions of its readings that are not
// held out.
struct coph_frozen_stage {
  std::int64_t id = 0;
  coph_vec2 center{};
  coph_state entry{};
  std::vector<coph_vec2> centers;
  std::vector<double> weights;
  std::vector<coph_shape> obstacles;
  std::vector<coph_vec2> support;
  // Within the box (plus 1e-10) and within 0.65 of a support point.
  bool supported(const coph_vec2 &q, const coph_config &cfg) const;
};

// The kept stages frozen with the given weights (FrozenStages.freeze), in 'spatial' stage mode.
struct coph_atlas {
  std::vector<coph_frozen_stage> stages;
  std::vector<coph_shape> geometry;
  bool overflow = false;
  coph_vec2 goal{};
  const coph_config *cfg = nullptr;
  const coph_network *network = nullptr;

  // FrozenStages.advance: the stage index for q, handing over to the next stage when q leaves the
  // inner box along either axis. Throws coph_domain_exit.
  int advance(const coph_vec2 &q, int index) const;
};
coph_atlas coph_freeze(const coph_learner_state &s,
                       const std::vector<std::vector<double>> &weights);

// maze_learning.maze_route: Dijkstra on the network from q, with edges closed by the reported
// geometry and priced by length (1 + 4 risk_weight risk) over the terrain anchors, then a
// waypoint up to 0.65 along the route, on the 0.1 m grid. Leaves its route in ctx.
struct coph_route {
  coph_vec2 target{};
  bool no_route = false;
  std::string reason; // maze_graph, maze_no_route or unsafe_connector
  std::vector<int> nodes;
  std::vector<int> blocked;
};
coph_route coph_maze_route(const coph_vec2 &q, const coph_vec2 &goal, const coph_network &network,
                           coph_route_context &ctx, int step);

// navigation.guarded_control in the profile: one control step.
struct coph_control_meta {
  std::string reason; // nominal, projected, brake or geometry_overflow
  std::string route_reason;
  int guard_steps = 0;
  int projection_candidates = 0;
  std::vector<int> route_nodes;
  coph_vec2 waypoint{};
  coph_vec2 applied{};
  bool infeasible = false;
};
template <class T> struct coph_control_of {
  coph_vec2_of<T> applied;
  coph_control_meta meta;
};
template <class T>
coph_control_of<T> coph_control(const coph_atlas &atlas, const coph_state_of<T> &x, int index,
                                const coph_vec2_of<T> &policy, double damping,
                                coph_route_context &ctx, int step);

// ---------------------------------------------------------------------------
// G7: the plan and the replay
// ---------------------------------------------------------------------------

struct coph_plan_entry {
  std::int64_t stage = 0;
  bool eligible = false;
  int horizon = 0;
  std::string reason;
  long long pilot_steps = 0;
  std::vector<std::array<std::int64_t, 3>> handoffs; // [step, from, to]
};
struct coph_replay_plan {
  std::vector<coph_plan_entry> entries;
  long long budget_start = 0;
  long long external_reserve = 0;
  long long pilot_steps = 0;
  bool baseline_reserved = false;
  // The planned horizon of a stage's replay, replay_steps if it has none.
  int horizon(std::int64_t stage, const coph_config &cfg) const;
};

// The update's budget: replay_update_cap less the steps since the plan began and the reserve.
long long coph_remaining(const coph_learner_state &s, const coph_replay_plan &plan);
// One batch: 7 times the planned horizons of the stages whose entry is supported (routed).
long long coph_batch_steps(const coph_atlas &atlas, const coph_replay_plan &plan);

// replay_budget.prepare, with the committed policy, from the atlas with the fitted weights.
// Counts its pilots' steps into s.counterfactual_steps.
coph_replay_plan coph_prepare(coph_learner_state &s, const coph_atlas &atlas, double damping,
                              long long external_reserve);

struct coph_eligibility {
  std::int64_t stage = 0;
  bool eligible = false;
  std::string reason;
};
struct coph_stage_event {
  std::int64_t entry = 0;
  int step = 0;
  std::int64_t from = 0, to = 0;
  double shaping_jump = 0;
};
struct coph_control_record {
  std::int64_t entry = 0;
  int step = 0;
  coph_control_meta meta;
};
struct coph_replay_result {
  bool replayed = false; // false when a domain exit ended the batch, or nothing was eligible
  double cost = 0;
  coph_vec2 gradient{}; // d cost / d policy, with gradient requested
  bool depends_on_policy = false;
  std::vector<coph_eligibility> eligibility;
  std::vector<coph_stage_event> events;
  std::vector<coph_control_record> controls;
  std::vector<std::pair<std::int64_t, std::vector<coph_state>>> paths;
  long long steps = 0; // counterfactual steps, guard steps included
};

// stage_prediction.retained_replay at `policy`: each kept stage from its entry, for the plan's
// horizon. With gradient, the replay runs on duals and also gives d cost / d policy. Counts its
// steps into s.counterfactual_steps.
coph_replay_result coph_replay(coph_learner_state &s, const coph_atlas &atlas,
                               const coph_replay_plan &plan, const coph_vec2 &policy,
                               double damping, bool gradient);

// ---------------------------------------------------------------------------
// G8: the update
// ---------------------------------------------------------------------------

struct coph_update_attempt {
  int power = 0;
  coph_vec2 policy{};
  std::optional<double> cost;
  std::string reason; // accepted, replay_cost or unsupported_or_nonfinite
  bool accepted = false;
};

struct coph_update_report {
  std::vector<coph_stage_fit> fit;
  coph_damping_fit damping_fit;
  double damping_proposal = 0;
  coph_replay_plan plan;
  double field_validation_before = 0, field_validation_after = 0;
  double plant_validation_before = 0, plant_validation_after = 0;
  std::optional<double> replay_before;
  std::optional<coph_vec2> gradient, unconstrained, bounded;
  bool policy_independent = false;
  std::vector<coph_update_attempt> attempts;
  bool exhausted = false;
  bool finite = false, feasible = false, valid = false, accepted = false;
  std::string reason; // accepted, nonfinite or validation_or_feasibility
  std::int64_t version_after = 0;
  coph_vec2 policy_after{};
  double damping_after = 0;
  double shaping_jump = 0; // V(x) after - V(x) before on the newest stage
};

// Learner.update at the state x, as the demos' learner runs it: the fits, the plan with the
// routed controller's reserve of 12 steps, the baseline and gradient replays, the projected policy
// step and up to solver_steps halvings, validation and feasibility, then the commit (every stage
// takes its fitted weights and the new version) or nothing.
coph_update_report coph_update(coph_learner_state &s, const coph_state &x);

extern template coph_control_of<double> coph_control<double>(const coph_atlas &,
                                                             const coph_state_of<double> &, int,
                                                             const coph_vec2_of<double> &, double,
                                                             coph_route_context &, int);
extern template coph_control_of<coph_dual2>
coph_control<coph_dual2>(const coph_atlas &, const coph_state_of<coph_dual2> &, int,
                         const coph_vec2_of<coph_dual2> &, double, coph_route_context &, int);

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_COPH_LEARNER_H
