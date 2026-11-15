/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// coph_config.h: Config.__post_init__ and CommunicationConfig.__post_init__, line for line
// (grl_snam/coph/records.py, grl_snam/coph/cooperative.py).

#include <cmath>
#include <cvc/nav/coph_config.h>
#include <initializer_list>
#include <stdexcept>

namespace cvc {
namespace nav {

namespace {

// Python's builtin min(): the first item, replaced by each later item that compares less. So a NaN
// first is kept, and a NaN later is skipped.
template <class T> T py_min(std::initializer_list<T> items) {
  auto it = items.begin();
  T m = *it;
  for (++it; it != items.end(); ++it)
    if (*it < m)
      m = *it;
  return m;
}

bool one_of(const std::string &s, std::initializer_list<const char *> options) {
  for (const char *o : options)
    if (s == o)
      return true;
  return false;
}

[[noreturn]] void refuse(const char *message) { throw std::invalid_argument(message); }

} // namespace

void coph_config::validate() {
  coph_config c = *this; // __post_init__ fills replay_update_cap; commit only on success
  if (!one_of(c.stage_blend_mode, {"legacy", "smooth"}))
    refuse("invalid stage blend mode");
  if (c.stage_blend_mode == "smooth" &&
      (c.replay_domain != "retained" || !one_of(c.controller_mode, {"protected", "routed"})))
    refuse("smooth stage blending requires retained protected/routed control");
  if (c.stage_blend_mode == "smooth" && c.overlap <= 0)
    refuse("smooth stage blending requires positive overlap");
  if (!one_of(c.route_field_mode, {"anchors", "supported_field"}))
    refuse("invalid route field mode");
  if (c.route_field_mode == "supported_field" && c.controller_mode != "routed")
    refuse("field routing requires routed control");
  if (!one_of(c.shaping_mode, {"legacy", "progress"}))
    refuse("invalid shaping mode");
  if (c.shaping_mode == "progress") {
    if (!one_of(c.controller_mode, {"protected", "routed"}))
      refuse("progress requires protected or routed control");
    if (c.shaping_k != 1.0 || c.shaping_ell != 0.5 || c.shaping_eta != 0.5)
      refuse("fixed progress settings required");
    const auto [lo, hi] = c.policy_bounds;
    const auto in_box = [&](double p) { return lo <= p && p <= hi; };
    if (lo < 0 || !std::isfinite(lo) || !std::isfinite(hi) || !in_box(c.policy_initial.first) ||
        !in_box(c.policy_initial.second))
      refuse("progress requires bounded nonnegative shaping");
  }
  if (!one_of(c.controller_mode, {"legacy", "geometry", "protected", "routed"}))
    refuse("invalid controller mode");
  if (c.controller_mode != "legacy") {
    if (c.replay_domain != "retained" || c.replay_objective != "mission")
      refuse("new controller requires retained mission replay");
    if (!c.replay_update_cap)
      c.replay_update_cap = 1024;
  }
  if (py_min({c.obstacle_cap, c.footprint_cap, c.route_expansion_cap}) < 1 || c.safety_margin < 0)
    refuse("invalid navigation limits");
  if (c.replay_horizon_mode == "supported_boundary" && !c.replay_update_cap)
    c.replay_update_cap = 1024;
  if (!one_of(c.replay_horizon_mode, {"fixed", "supported_boundary"}))
    refuse("invalid replay_horizon_mode");
  if (c.replay_horizon_mode == "supported_boundary" && c.replay_domain != "retained")
    refuse("boundary replay requires retained models");
  if (c.replay_update_cap && *c.replay_update_cap < 1)
    refuse("positive replay_update_cap required");
  if (c.replay_horizon_mode == "supported_boundary" &&
      (c.boundary_max_steps < c.replay_steps || c.boundary_follow_steps < 1))
    refuse("invalid boundary horizon");
  if (!one_of(c.replay_domain, {"single", "retained"}))
    refuse("replay_domain must be single or retained");
  if (c.replay_domain == "retained" && c.replay_objective != "mission")
    refuse("retained replay requires mission objective");
  if (!one_of(c.replay_objective, {"legacy", "mission"}))
    refuse("replay_objective must be legacy or mission");
  if (!one_of(c.navigation_target, {"mission", "stage"}))
    refuse("navigation_target must be mission or stage");
  if (!(0 <= c.overlap && c.overlap < 1) ||
      !(c.previous == 0 || c.previous == 2 || c.previous == 3))
    refuse("overlap in [0,1); previous must be 0, 2, or 3");
  if (py_min({c.dt, c.stage_width, c.stage_height, c.rbf_width, c.sensor_radius}) <= 0)
    refuse("positive physical scales required");
  if (py_min({c.steps, c.sample_cap, c.transition_cap, c.anchor_cap, c.update_every, c.replay_steps,
              c.solver_steps}) < 1 ||
      c.rbf_side < 2)
    refuse("positive budgets and rbf_side >= 2 required");
  *this = c;
}

std::vector<std::string> coph_config::outside_profile() const {
  const auto value = [this](const std::string &field) -> const std::string & {
    if (field == "stage_blend_mode")
      return stage_blend_mode;
    if (field == "route_field_mode")
      return route_field_mode;
    if (field == "controller_mode")
      return controller_mode;
    if (field == "shaping_mode")
      return shaping_mode;
    if (field == "navigation_target")
      return navigation_target;
    if (field == "replay_objective")
      return replay_objective;
    if (field == "replay_domain")
      return replay_domain;
    return replay_horizon_mode;
  };
  std::vector<std::string> out;
  for (const auto &[field, mode] : coph_profile_modes())
    if (value(field) != mode)
      out.push_back(field);
  return out;
}

coph_config coph_profile() {
  coph_config c;
  c.controller_mode = "routed";
  c.shaping_mode = "progress";
  c.replay_objective = "mission";
  c.replay_domain = "retained";
  c.replay_horizon_mode = "supported_boundary";
  c.replay_update_cap = 1024;
  c.validate();
  return c;
}

void coph_communication_config::validate() const {
  if (!one_of(prediction_variant, {"legacy", "baseline", "compact", "cross_stage", "both"}))
    refuse("invalid prediction_variant");
}

std::vector<std::string> coph_communication_config::outside_profile() const {
  if (prediction_variant != "legacy")
    return {"prediction_variant"};
  return {};
}

coph_communication_config coph_communication_profile() {
  coph_communication_config c;
  c.byte_budget = 4000000;
  c.ttl = 300.0;
  c.validate();
  return c;
}

namespace {

void refuse_outside(const char *record, const std::vector<std::string> &fields) {
  if (fields.empty())
    return;
  std::string message = std::string(record) + " outside the demos' profile (NAV-COPH-ROADMAP D7):";
  for (std::size_t i = 0; i < fields.size(); ++i)
    message += (i ? ", " : " ") + fields[i];
  throw std::invalid_argument(message);
}

} // namespace

void require_profile(const coph_config &cfg) {
  refuse_outside("coph_config", cfg.outside_profile());
}

void require_profile(const coph_communication_config &comm) {
  refuse_outside("coph_communication_config", comm.outside_profile());
}

} // namespace nav
} // namespace cvc
