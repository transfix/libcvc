/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// One CoPH agent, whole (C5 of docs/roadmap/NAV-COPH-ROADMAP.md, §4.5); see coph_agent.h. Each
// step names the GRL-SNAM (grl_snam.coph) code it follows. Built with -ffp-contract=off.

#include <algorithm>
#include <cmath>
#include <cvc/nav/coph_agent.h>
#include <cvc/nav/detail/coph_math.h>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace nav {

namespace {

constexpr std::size_t kGeometryReports = 32;
constexpr std::size_t kLocalCells = 256;
constexpr double kCell = 0.2;

double distance(const coph_vec2 &a, const coph_vec2 &b) {
  const double dx = a[0] - b[0], dy = a[1] - b[1];
  return std::sqrt(dx * dx + dy * dy);
}

} // namespace

coph_agent::coph_agent(const coph_config &cfg, const coph_communication_config &comm,
                       const coph_state &initial, const coph_vec2 &goal, std::string agent_id,
                       coph_network network)
    : ledger_(cfg, initial, goal) {
  comm.validate();
  if (!comm.outside_profile().empty())
    throw std::invalid_argument("coph_agent: the communication is outside the demos' profile");
  s_.cfg = ledger_.config();
  s_.goal = goal;
  s_.network = std::move(network);
  s_.damping = cfg.damping_initial;
  s_.policy = {cfg.policy_initial.first, cfg.policy_initial.second};
  s_.agent_id = std::move(agent_id);
  s_.communication = comm;
  current_state_ = initial;
  sync_stages();
}

void coph_agent::sync_stages() {
  s_.stages = ledger_.stages();
  s_.geometry = ledger_.geometry();
}

// MazeLearner.refresh_geometry, with the ledger's store kept in step.
void coph_agent::refresh_geometry() {
  coph_refresh_geometry(s_);
  ledger_.set_geometry(s_.geometry);
  s_.stages = ledger_.stages();
}

void coph_agent::observe(const coph_observation &observation,
                         const std::vector<coph_shape> &obstacles,
                         const coph_transition *transition) {
  const coph_config &cfg = s_.cfg;
  // CooperativeLearner.observe: the clock, the state, the agent's own uids.
  s_.now = observation.time;
  if (transition)
    current_state_ = transition->after;
  coph_observation o = observation;
  for (coph_reading &r : o.readings)
    r.uid = s_.agent_id + ":" + r.uid;

  // Learner.observe: the footprints, readings and transition (C1's ledger), and between them
  // navigation.observe_geometry's store. The store depends on the stages' boxes only, which the
  // readings do not move.
  ledger_.observe(o, transition);
  std::vector<coph_shape> incoming = ledger_.geometry();
  for (const coph_shape &sh : obstacles)
    if (std::find(incoming.begin(), incoming.end(), sh) == incoming.end())
      incoming.push_back(sh);
  const coph_vec2 center =
      o.sensor_center ? *o.sensor_center : coph_vec2{current_state_[0], current_state_[1]};
  const double speed =
      std::sqrt(current_state_[2] * current_state_[2] + current_state_[3] * current_state_[3]);
  const double stop = speed * speed / (2 * cfg.force_limit) + cfg.sensor_radius;
  std::vector<coph_shape> kept;
  for (const coph_shape &sh : incoming) {
    bool keep = distance(center, {sh.x, sh.y}) <= stop + sh.reach() + 0.17;
    for (const coph_stage &st : ledger_.stages())
      keep = keep || coph_intersects(sh, st.center, cfg);
    if (keep)
      kept.push_back(sh);
  }
  std::sort(kept.begin(), kept.end(), detail::coph_shape_less);
  s_.geometry_overflow = s_.geometry_overflow || static_cast<int>(kept.size()) > cfg.obstacle_cap;
  // Nearest surface first (ties in the shapes' order); then the cap.
  auto gap = [&](const coph_shape &sh) {
    return sh.slab ? detail::coph_slab_distance(center, sh) : distance(center, {sh.x, sh.y}) - sh.r;
  };
  std::stable_sort(kept.begin(), kept.end(), [&](const coph_shape &a, const coph_shape &b) {
    const double ga = gap(a), gb = gap(b);
    return ga != gb ? ga < gb : detail::coph_shape_less(a, b);
  });
  if (static_cast<int>(kept.size()) > cfg.obstacle_cap)
    kept.resize(static_cast<std::size_t>(cfg.obstacle_cap));
  ledger_.set_geometry(kept);
  sync_stages();
  coph_expire(s_);
  if (car())
    return; // a car's learner is CooperativeLearner on Learner

  // TerrainLearner.observe: route evidence that expired goes; the terrain anchors refresh.
  auto &e = s_.exchange;
  e.remote_terrain.erase(
      std::remove_if(e.remote_terrain.begin(), e.remote_terrain.end(),
                     [&](const auto &r) { return !(r.packet.expires >= s_.now); }),
      e.remote_terrain.end());
  coph_refresh_terrain(s_);

  // MazeLearner.observe: one report per sensed shape, the newest sighting's.
  for (const coph_shape &sh : obstacles) {
    auto it = std::find_if(e.local_geometry.begin(), e.local_geometry.end(),
                           [&](const coph_geometry_report &r) { return r.shape == sh; });
    std::string source;
    if (it == e.local_geometry.end()) {
      source = s_.agent_id + ":geometry:" + std::to_string(e.local_geometry.size());
      geometry_events_.push_back({"locally_sensed", s_.now, sh, source});
    } else {
      source = it->source;
    }
    const coph_geometry_report report{sh, o.sensor_center ? *o.sensor_center : center,
                                      o.sensor_radius, s_.now, source};
    if (it == e.local_geometry.end())
      e.local_geometry.push_back(report);
    else
      *it = report;
  }
  if (e.local_geometry.size() > kGeometryReports) {
    s_.geometry_overflow = true;
    e.local_geometry.erase(e.local_geometry.begin(),
                           e.local_geometry.end() - static_cast<std::ptrdiff_t>(kGeometryReports));
  }
  refresh_geometry();
}

bool coph_agent::advance(const coph_state &x) {
  if (!ledger_.advance(x, s_.version))
    return false;
  sync_stages();
  if (!car()) {
    refresh_geometry();
    return true;
  }
  // Learner.advance: the new stage's entry context is the live one, due to replan; the kept
  // stages keep theirs.
  coph_route_context entry = navigation_;
  entry.planned_at = -10000;
  s_.entry_navigation.push_back({s_.stages.back().id, std::move(entry)});
  auto &en = s_.entry_navigation;
  en.erase(std::remove_if(en.begin(), en.end(),
                          [&](const auto &e) {
                            return std::none_of(
                                s_.stages.begin(), s_.stages.end(),
                                [&](const coph_stage &st) { return st.id == e.first; });
                          }),
           en.end());
  return true;
}

// Learner.action: FrozenStages.freeze on the committed weights, then the routed control in the
// live context.
coph_vec2 coph_agent::routed_control(const coph_state &x) {
  std::vector<std::vector<double>> weights;
  for (const coph_stage &st : s_.stages)
    weights.push_back(st.weights);
  const coph_atlas atlas = coph_freeze(s_, weights);
  navigation_.reported_geometry = s_.reported_geometry;
  navigation_.terrain_anchors = s_.terrain_anchors;
  const coph_control_of<double> c =
      coph_control<double>(atlas, x, static_cast<int>(atlas.stages.size()) - 1, s_.policy,
                           s_.damping, navigation_, navigation_step_);
  ++navigation_step_;
  last_control_ = c.meta;
  s_.navigation_target = navigation_.target;
  return c.applied;
}

coph_vec2 coph_agent::action(const coph_state &x) { return routed_control(x); }

coph_update_report coph_agent::update(const coph_state &x) {
  const coph_config &cfg = s_.cfg;
  coph_expire(s_);
  // CooperativeLearner.update: a diagnostic control before and after Learner.update, whose guard
  // steps count against the replay budget; the live context and the step count are put back.
  auto diagnostic = [&]() {
    const coph_route_context navigation = navigation_;
    const int step = navigation_step_;
    const coph_control_meta last = last_control_;
    const std::optional<coph_vec2> target = s_.navigation_target;
    routed_control(x);
    const int guards = last_control_.guard_steps;
    navigation_ = navigation;
    navigation_step_ = step;
    last_control_ = last;
    s_.navigation_target = target;
    return guards;
  };
  s_.counterfactual_steps += diagnostic();
  coph_update_report rep = coph_update(s_, x);
  if (rep.accepted) {
    std::vector<std::vector<double>> weights;
    for (const coph_stage &st : s_.stages)
      weights.push_back(st.weights);
    ledger_.commit(weights, s_.version);
  }
  s_.counterfactual_steps += diagnostic();
  if (car())
    return rep;

  // TerrainLearner.update: the learner's own anchors from the committed fields, at up to 80 of
  // each stage's fitting positions, one per 0.2 m cell.
  auto &e = s_.exchange;
  for (const coph_stage &st : s_.stages) {
    std::vector<coph_vec2> points;
    for (const coph_reading &r : st.readings)
      if (!coph_is_validation(r.uid) &&
          std::find(points.begin(), points.end(), r.q) == points.end())
        points.push_back(r.q);
    const std::size_t stride = std::max<std::size_t>(1, points.size() / 80);
    std::vector<coph_vec2> chosen;
    for (std::size_t k = 0; k < points.size() && chosen.size() < 80; k += stride)
      chosen.push_back(points[k]);
    if (chosen.empty())
      continue;
    const std::vector<double> v = coph_field_vector(st, st.weights, cfg, chosen);
    for (std::size_t k = 0; k < chosen.size(); ++k) {
      const coph_anchor a{chosen[k][0], chosen[k][1], v[3 * k], v[3 * k + 1], v[3 * k + 2]};
      const long long ix = static_cast<long long>(std::nearbyint(a.x / kCell));
      const long long iy = static_cast<long long>(std::nearbyint(a.y / kCell));
      auto it = std::find_if(e.local_cells.begin(), e.local_cells.end(),
                             [&](const auto &c) { return c.ix == ix && c.iy == iy; });
      if (it == e.local_cells.end())
        e.local_cells.push_back({ix, iy, s_.now, a});
      else {
        it->time = s_.now;
        it->anchor = a;
      }
    }
  }
  while (e.local_cells.size() > kLocalCells) {
    auto oldest = e.local_cells.begin();
    for (auto c = e.local_cells.begin(); c != e.local_cells.end(); ++c)
      if (c->time < oldest->time)
        oldest = c;
    e.local_cells.erase(oldest);
  }
  e.local_model_anchors.clear();
  for (const auto &c : e.local_cells)
    e.local_model_anchors.push_back(c.anchor);
  coph_refresh_terrain(s_);
  return rep;
}

std::optional<coph_packet> coph_agent::export_to(const coph_state &x, const std::string &recipient,
                                                 const std::array<coph_draws, 2> &draws) {
  return coph_export(s_, x, recipient, draws);
}

std::optional<coph_packet> coph_agent::export_to(const coph_state &x, const std::string &recipient,
                                                 std::mt19937 &rng) {
  return coph_export(s_, x, recipient, rng);
}

coph_packet_event coph_agent::assimilate(const coph_packet &m) {
  // MazeLearner.assimilate's received_geometry events: each shape neither this agent nor an
  // earlier packet reported.
  const auto &e = s_.exchange;
  std::vector<coph_shape> fresh;
  for (const coph_geometry_report &o : m.geometry) {
    const bool known = std::any_of(e.remote_geometry.begin(), e.remote_geometry.end(),
                                   [&](const auto &r) { return r.shape == o.shape; }) ||
                       std::any_of(e.local_geometry.begin(), e.local_geometry.end(),
                                   [&](const auto &r) { return r.shape == o.shape; }) ||
                       std::find(fresh.begin(), fresh.end(), o.shape) != fresh.end();
    if (!known)
      fresh.push_back(o.shape);
  }
  const coph_packet_event ev = coph_assimilate(s_, m);
  if (car())
    return ev;
  ledger_.set_geometry(s_.geometry);
  s_.stages = ledger_.stages();
  if (ev.reason == "eligible")
    for (const coph_shape &sh : fresh)
      geometry_events_.push_back({"received_geometry", s_.now, sh, m.identity});
  return ev;
}

} // namespace nav
} // namespace cvc
