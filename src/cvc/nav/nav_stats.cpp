/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// nav_stats.cpp — see nav_stats.h. The base (RF-free) telemetry collector +
// corpus scorecard, extracted from the two-layer nav-stats design.
#include <algorithm>
#include <cmath>
#include <cvc/nav/nav_stats.h>
#include <sstream>

namespace cvc {
namespace nav {

namespace {
constexpr double kPi = 3.14159265358979323846;
// Shortest signed angle a-b into (-pi, pi].
double wrap_pi(double a) {
  while (a > kPi)
    a -= 2.0 * kPi;
  while (a <= -kPi)
    a += 2.0 * kPi;
  return a;
}
void num(std::ostringstream &o, double v) {
  if (std::isfinite(v))
    o << v;
  else
    o << "null";
}
// Like num(), but also nulls the "unmeasured" sentinel (>=1e29). min_clearance_m and
// min_sep_m sit at 1e30 until a sampler / a second vehicle lowers them; a raw 1e+30 in
// the record reads as *enormous* clearance/separation — the inverse of "missing" — so a
// trainer would misread it. aggregate_nav guards min_sep_m the same way; keep the
// per-episode record honest too (a null means "not measured").
void num_sentinel(std::ostringstream &o, double v) {
  if (v >= 1e29)
    o << "null";
  else
    num(o, v);
}
} // namespace

// ── base collector ──────────────────────────────────────────────────────────

void nav_stats_collector::begin_episode(int n, double dt_s, const float *start_pos,
                                        const float *goal_pos, const budget_policy &budget,
                                        std::string scene_id, unsigned seed,
                                        std::string checkpoint) {
  n_ = n < 0 ? 0 : n;
  dt_ = dt_s;
  budget_ = budget;
  step_i_ = 0;
  ep_ = episode_nav_stats{};
  ep_.scene_id = std::move(scene_id);
  ep_.seed = seed;
  ep_.checkpoint = std::move(checkpoint);
  ep_.n_vehicles = n_;
  ep_.dt_s = dt_s;
  ep_.per_vehicle.assign(n_, veh_nav_stats{});
  start_pos_.assign(start_pos, start_pos + 2 * n_);
  prev_pos_.assign(start_pos, start_pos + 2 * n_);
  prev_head_.assign(n_, 0.0f);
  prev_spd_.assign(n_, 0.0f);
  prev_mode_.assign(n_, 0);
  speed_sum_.assign(n_, 0.0);
  conv_.assign(n_, 0);
  cls_.assign(n_, 0);
  rr_.assign(n_, 0.0);
  mass_.assign(n_, 0.0);
  parent_.assign(n_, -1);
  slot_err_sum_.assign(n_, 0.0);
  slot_err_max_.assign(n_, 0.0);
  slot_err_last_.assign(n_, 0.0);
  slot_err_cnt_.assign(n_, 0);
  for (int i = 0; i < n_; ++i) {
    auto &v = ep_.per_vehicle[i];
    v.veh_index = i;
    const double dx = goal_pos[2 * i] - start_pos[2 * i];
    const double dy = goal_pos[2 * i + 1] - start_pos[2 * i + 1];
    v.straight_m = std::hypot(dx, dy);
  }
}

void nav_stats_collector::set_identity(int i, int convoy_id, int vehicle_class,
                                       double robot_radius_m, double mass_kg,
                                       int formation_parent) {
  if (i < 0 || i >= n_)
    return;
  conv_[i] = convoy_id;
  cls_[i] = vehicle_class;
  rr_[i] = robot_radius_m;
  mass_[i] = mass_kg;
  parent_[i] = formation_parent;
}

void nav_stats_collector::step(const float *pos, const float *head, const float *spd,
                               const int *mode, const std::uint8_t *reached,
                               const nav_samplers &smp) {
  const bool first = (step_i_ == 0);
  const double t_end = (step_i_ + 1) * dt_; // elapsed at the end of this step
  for (int i = 0; i < n_; ++i) {
    auto &v = ep_.per_vehicle[i];
    const double x = pos[2 * i], y = pos[2 * i + 1];
    // prev_pos_ is seeded to the real start, so the first segment (start -> step 0)
    // counts; only turn/accel below skip the first step (their prev_ start at 0).
    const double seg = std::hypot(x - prev_pos_[2 * i], y - prev_pos_[2 * i + 1]);
    v.total_path_m += seg;

    if (!first) {
      const double dh = std::fabs(wrap_pi(head[i] - prev_head_[i]));
      v.turn_total_rad += dh;
      if (dh > p_.turn_event_rad)
        ++v.turn_events;
      v.accel_integral += std::fabs((double)spd[i] - (double)prev_spd_[i]);
    }

    const double vmag = spd[i];
    speed_sum_[i] += vmag;
    if (vmag > v.speed_max)
      v.speed_max = vmag;
    if (vmag > p_.speed_eps_mps)
      v.time_moving_s += dt_;
    else
      v.time_stopped_s += dt_;

    if (mode[i] == 1) {
      v.time_in_wall_s += dt_;
      if (prev_mode_[i] != 1)
        ++v.wall_entries;
    }

    if (smp.min_clearance_m) {
      const double c = smp.min_clearance_m[i];
      if (c < v.min_clearance_m)
        v.min_clearance_m = c;
      if (c < p_.clear_safety_m)
        v.time_below_clear_s += dt_;
    }
    if (smp.occupied && smp.occupied(x, y))
      ++v.penetration_steps;
    if (smp.material_id) {
      const int m = smp.material_id(x, y);
      if (m >= 0 && m < kNumMaterials) {
        v.time_over_material_s[m] += dt_;
        v.dist_over_material_m[m] += seg;
      }
    }
    if (smp.formation_slot) { // formation-holding: distance to this vehicle's slot this step
      double sx = 0, sy = 0;
      if (smp.formation_slot(i, sx, sy)) {
        const double e = std::hypot(x - sx, y - sy);
        slot_err_sum_[i] += e;
        ++slot_err_cnt_[i];
        if (e > slot_err_max_[i])
          slot_err_max_[i] = e;
        slot_err_last_[i] = e;
      }
    }

    if (reached[i] && v.time_to_goal_s < 0) {
      v.time_to_goal_s = t_end;
      v.arrived = true;
      v.goals_reached = 1;
    }

    prev_pos_[2 * i] = x;
    prev_pos_[2 * i + 1] = y;
    prev_head_[i] = head[i];
    prev_spd_[i] = spd[i];
    prev_mode_[i] = mode[i];
  }

  // pairwise separation + per-vehicle contact (once per frame)
  if (n_ > 1) {
    for (int i = 0; i < n_; ++i) {
      bool contact = false;
      for (int j = 0; j < n_; ++j) {
        if (j == i)
          continue;
        const double d = std::hypot(pos[2 * i] - pos[2 * j], pos[2 * i + 1] - pos[2 * j + 1]);
        if (i < j && d < ep_.min_sep_m)
          ep_.min_sep_m = d;
        if (p_.min_gap_m > 0 && d < p_.min_gap_m)
          contact = true;
      }
      if (contact)
        ++ep_.per_vehicle[i].veh_contacts;
    }
  }

  ++step_i_;
}

episode_nav_stats nav_stats_collector::finish() {
  ep_.ticks = step_i_;
  const double steps = step_i_ > 0 ? (double)step_i_ : 1.0;
  const double elapsed = step_i_ * dt_;
  int arrived = 0, npen = 0, contacts = 0;
  double ttg_sum = 0, makespan = 0, ratio_sum = 0, turn_sum = 0, fuel_sum = 0;
  int ratio_n = 0;
  for (int i = 0; i < n_; ++i) {
    auto &v = ep_.per_vehicle[i];
    v.convoy_id = conv_[i];
    v.vehicle_class = cls_[i];
    v.robot_radius_m = rr_[i];
    v.mass_kg = mass_[i];
    v.formation_parent = parent_[i];
    v.speed_mean = speed_sum_[i] / steps;
    v.fuel_used = v.accel_integral; // PR 1 proxy
    v.timed_out = !v.arrived;
    // formation holding — only when a slot sampler fed samples this episode (else all zero, off)
    if (slot_err_cnt_[i] > 0) {
      v.slot_error_mean_m = slot_err_sum_[i] / slot_err_cnt_[i];
      v.slot_error_max_m = slot_err_max_[i];
      v.formation_arrived = p_.formation_tol_m > 0 && slot_err_last_[i] < p_.formation_tol_m;
    }

    const double used_t = v.arrived ? v.time_to_goal_s : elapsed;
    bool over = false;
    if (budget_.time_budget_s > 0 && used_t > budget_.time_budget_s)
      over = true;
    if (budget_.eta_multiple > 0 && budget_.speed_ref_mps > 0) {
      const double eta = budget_.eta_multiple * v.straight_m / budget_.speed_ref_mps;
      if (used_t > eta)
        over = true;
    }
    if (budget_.fuel_budget > 0 && v.fuel_used > budget_.fuel_budget)
      over = true;
    v.over_budget = over;

    if (v.arrived) {
      ++arrived;
      ttg_sum += v.time_to_goal_s;
      if (v.time_to_goal_s > makespan)
        makespan = v.time_to_goal_s;
    }
    npen += v.penetration_steps;
    contacts += v.veh_contacts;
    if (v.straight_m > 1e-9) {
      ratio_sum += v.total_path_m / v.straight_m;
      ++ratio_n;
    }
    turn_sum += v.turn_total_rad;
    fuel_sum += v.fuel_used;
  }
  ep_.arrived = arrived;
  ep_.makespan_s = makespan;
  ep_.mean_ttg_s = arrived > 0 ? ttg_sum / arrived : 0;
  ep_.penetration_pct = n_ > 0 ? 100.0 * npen / (n_ * steps) : 0;
  ep_.total_veh_contacts = contacts;
  ep_.mean_path_ratio = ratio_n > 0 ? ratio_sum / ratio_n : 0;
  ep_.mean_turn_total_rad = n_ > 0 ? turn_sum / n_ : 0;
  ep_.total_fuel = fuel_sum;
  bool any_over = false;
  for (const auto &v : ep_.per_vehicle)
    if (v.over_budget)
      any_over = true;
  ep_.success = (n_ > 0) && (arrived == n_) && !any_over;
  return ep_;
}

// ── JSON ────────────────────────────────────────────────────────────────────

namespace {
void write_array(std::ostringstream &o, const double *a, int n) {
  o << '[';
  for (int i = 0; i < n; ++i) {
    if (i)
      o << ',';
    num(o, a[i]);
  }
  o << ']';
}
} // namespace

std::string episode_nav_stats::to_json() const {
  std::ostringstream o;
  o.precision(6);
  o << "{\"scene_id\":\"" << scene_id << "\",\"seed\":" << seed << ",\"checkpoint\":\""
    << checkpoint << "\",\"n_vehicles\":" << n_vehicles << ",\"ticks\":" << ticks << ",\"dt_s\":";
  num(o, dt_s);
  o << ",\"arrived\":" << arrived << ",\"success\":" << (success ? "true" : "false")
    << ",\"makespan_s\":";
  num(o, makespan_s);
  o << ",\"mean_ttg_s\":";
  num(o, mean_ttg_s);
  o << ",\"penetration_pct\":";
  num(o, penetration_pct);
  o << ",\"total_veh_contacts\":" << total_veh_contacts << ",\"min_sep_m\":";
  num_sentinel(o, min_sep_m);
  o << ",\"mean_path_ratio\":";
  num(o, mean_path_ratio);
  o << ",\"mean_turn_total_rad\":";
  num(o, mean_turn_total_rad);
  o << ",\"total_fuel\":";
  num(o, total_fuel);
  o << ",\"per_vehicle\":[";
  for (std::size_t k = 0; k < per_vehicle.size(); ++k) {
    const auto &v = per_vehicle[k];
    if (k)
      o << ',';
    o << "{\"veh_index\":" << v.veh_index << ",\"convoy_id\":" << v.convoy_id
      << ",\"vehicle_class\":" << v.vehicle_class << ",\"robot_radius_m\":";
    num(o, v.robot_radius_m);
    o << ",\"mass_kg\":";
    num(o, v.mass_kg);
    o << ",\"arrived\":" << (v.arrived ? "true" : "false") << ",\"time_to_goal_s\":";
    num(o, v.time_to_goal_s);
    o << ",\"timed_out\":" << (v.timed_out ? "true" : "false")
      << ",\"over_budget\":" << (v.over_budget ? "true" : "false")
      << ",\"goals_reached\":" << v.goals_reached << ",\"total_path_m\":";
    num(o, v.total_path_m);
    o << ",\"straight_m\":";
    num(o, v.straight_m);
    o << ",\"turn_total_rad\":";
    num(o, v.turn_total_rad);
    o << ",\"turn_events\":" << v.turn_events << ",\"time_in_wall_s\":";
    num(o, v.time_in_wall_s);
    o << ",\"wall_entries\":" << v.wall_entries << ",\"time_moving_s\":";
    num(o, v.time_moving_s);
    o << ",\"time_stopped_s\":";
    num(o, v.time_stopped_s);
    o << ",\"speed_mean\":";
    num(o, v.speed_mean);
    o << ",\"speed_max\":";
    num(o, v.speed_max);
    o << ",\"min_clearance_m\":";
    num_sentinel(o, v.min_clearance_m);
    o << ",\"time_below_clear_s\":";
    num(o, v.time_below_clear_s);
    o << ",\"penetration_steps\":" << v.penetration_steps << ",\"veh_contacts\":" << v.veh_contacts
      << ",\"accel_integral\":";
    num(o, v.accel_integral);
    o << ",\"fuel_used\":";
    num(o, v.fuel_used);
    o << ",\"time_over_material_s\":";
    write_array(o, v.time_over_material_s.data(), kNumMaterials);
    o << ",\"dist_over_material_m\":";
    write_array(o, v.dist_over_material_m.data(), kNumMaterials);
    o << ",\"formation_parent\":" << v.formation_parent << ",\"slot_error_mean_m\":";
    num(o, v.slot_error_mean_m);
    o << ",\"slot_error_max_m\":";
    num(o, v.slot_error_max_m);
    o << ",\"formation_arrived\":" << (v.formation_arrived ? "true" : "false");
    o << '}';
  }
  o << "]}";
  return o.str();
}

// ── scorecard ───────────────────────────────────────────────────────────────

nav_scorecard aggregate_nav(const std::vector<episode_nav_stats> &episodes,
                            std::string checkpoint) {
  nav_scorecard s;
  s.checkpoint = std::move(checkpoint);
  s.n_episodes = static_cast<int>(episodes.size());
  int succ = 0, runs = 0, arrived = 0, sep_n = 0;
  double ttg_sum = 0, makespan_sum = 0, ratio_sum = 0, turn_sum = 0, fuel_sum = 0;
  double pen_sum = 0, sep_sum = 0;
  int ratio_n = 0, contacts = 0;
  double mat_total = 0;
  std::array<double, kNumMaterials> mat_sum{};
  std::vector<double> ttgs;
  // formation holding (followers = formation_parent >= 0; anchors = -1)
  int foll_runs = 0, foll_arr = 0, form_episodes = 0, form_mission_ok = 0;
  double slot_sum = 0;
  for (const auto &e : episodes) {
    if (e.success)
      ++succ;
    // per-episode formation mission: every convoy (that has followers) has its anchor arrived AND
    // all its followers in-slot. Keyed by convoy_id via a small vector (ids are small contiguous).
    int maxConvoy = -1;
    for (const auto &v : e.per_vehicle)
      maxConvoy = std::max(maxConvoy, v.convoy_id);
    std::vector<char> convoyHasFollowers(maxConvoy + 1, 0), convoyOk(maxConvoy + 1, 1);
    for (const auto &v : e.per_vehicle) {
      if (v.formation_parent >= 0) { // follower
        convoyHasFollowers[v.convoy_id] = 1;
        if (!v.formation_arrived)
          convoyOk[v.convoy_id] = 0;
      } else if (!v.arrived) { // anchor/lead must reach the objective
        convoyOk[v.convoy_id] = 0;
      }
    }
    bool anyFormation = false, allOk = true;
    for (int c = 0; c <= maxConvoy; ++c)
      if (convoyHasFollowers[c]) {
        anyFormation = true;
        if (!convoyOk[c])
          allOk = false;
      }
    if (anyFormation) {
      ++form_episodes;
      if (allOk)
        ++form_mission_ok;
    }
    makespan_sum += e.makespan_s;
    pen_sum += e.penetration_pct;
    if (e.min_sep_m < 1e29) {
      sep_sum += e.min_sep_m;
      ++sep_n;
    }
    for (const auto &v : e.per_vehicle) {
      ++runs;
      if (v.arrived) {
        ++arrived;
        ttg_sum += v.time_to_goal_s;
        ttgs.push_back(v.time_to_goal_s);
      }
      if (v.straight_m > 1e-9) {
        ratio_sum += v.total_path_m / v.straight_m;
        ++ratio_n;
      }
      turn_sum += v.turn_total_rad;
      fuel_sum += v.fuel_used;
      contacts += v.veh_contacts;
      for (int m = 0; m < kNumMaterials; ++m) {
        mat_sum[m] += v.time_over_material_s[m];
        mat_total += v.time_over_material_s[m];
      }
      if (v.formation_parent >= 0) { // followers only (the anchor has no slot)
        ++foll_runs;
        if (v.formation_arrived)
          ++foll_arr;
        slot_sum += v.slot_error_mean_m;
      }
    }
  }
  s.n_vehicle_runs = runs;
  s.success_rate = s.n_episodes > 0 ? (double)succ / s.n_episodes : 0;
  s.arrival_rate = runs > 0 ? (double)arrived / runs : 0;
  s.mean_time_to_goal_s = arrived > 0 ? ttg_sum / arrived : 0;
  if (!ttgs.empty()) {
    std::sort(ttgs.begin(), ttgs.end());
    const std::size_t k = (std::size_t)std::ceil(0.95 * ttgs.size()) - 1;
    s.p95_time_to_goal_s = ttgs[std::min(k, ttgs.size() - 1)];
  }
  s.mean_makespan_s = s.n_episodes > 0 ? makespan_sum / s.n_episodes : 0;
  s.mean_path_ratio = ratio_n > 0 ? ratio_sum / ratio_n : 0;
  s.mean_turn_total_rad = runs > 0 ? turn_sum / runs : 0;
  s.mean_fuel = runs > 0 ? fuel_sum / runs : 0;
  s.mean_penetration_pct = s.n_episodes > 0 ? pen_sum / s.n_episodes : 0;
  s.veh_contacts_per_run = runs > 0 ? (double)contacts / runs : 0;
  s.mean_min_sep_m = sep_n > 0 ? sep_sum / sep_n : 0;
  if (mat_total > 0)
    for (int m = 0; m < kNumMaterials; ++m)
      s.material_time_share[m] = mat_sum[m] / mat_total;
  s.form_arrival_rate = foll_runs > 0 ? (double)foll_arr / foll_runs : 0;
  s.form_mission_rate = form_episodes > 0 ? (double)form_mission_ok / form_episodes : 0;
  s.mean_slot_error_m = foll_runs > 0 ? slot_sum / foll_runs : 0;
  return s;
}

std::string nav_scorecard::to_json() const {
  std::ostringstream o;
  o.precision(6);
  o << "{\"checkpoint\":\"" << checkpoint << "\",\"n_episodes\":" << n_episodes
    << ",\"n_vehicle_runs\":" << n_vehicle_runs << ",\"success_rate\":";
  num(o, success_rate);
  o << ",\"arrival_rate\":";
  num(o, arrival_rate);
  o << ",\"mean_time_to_goal_s\":";
  num(o, mean_time_to_goal_s);
  o << ",\"p95_time_to_goal_s\":";
  num(o, p95_time_to_goal_s);
  o << ",\"mean_makespan_s\":";
  num(o, mean_makespan_s);
  o << ",\"mean_path_ratio\":";
  num(o, mean_path_ratio);
  o << ",\"mean_turn_total_rad\":";
  num(o, mean_turn_total_rad);
  o << ",\"mean_fuel\":";
  num(o, mean_fuel);
  o << ",\"mean_penetration_pct\":";
  num(o, mean_penetration_pct);
  o << ",\"veh_contacts_per_run\":";
  num(o, veh_contacts_per_run);
  o << ",\"mean_min_sep_m\":";
  num(o, mean_min_sep_m);
  o << ",\"material_time_share\":";
  write_array(o, material_time_share.data(), kNumMaterials);
  o << ",\"form_arrival_rate\":";
  num(o, form_arrival_rate);
  o << ",\"form_mission_rate\":";
  num(o, form_mission_rate);
  o << ",\"mean_slot_error_m\":";
  num(o, mean_slot_error_m);
  o << "}";
  return o.str();
}

} // namespace nav
} // namespace cvc
