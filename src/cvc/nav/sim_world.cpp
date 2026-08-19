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

// sim_world.cpp — see sim_world.h. The step() mirrors grl_snam/swarm.py Swarm.step
// (shared belief): sense (gated) -> occupancy/field rebuild -> carrot FSM -> fused
// drive -> reached/park. Every heavy op is an already-parity-tested cvc::nav call.

#include <algorithm>
#include <cmath>
#include <cvc/nav/belief_occupancy.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/sim_world.h>
#include <limits>
#include <random>
#include <vector>

namespace cvc {
namespace nav {

sim_world::sim_world(const config &cfg, const std::uint8_t *truth, const std::uint8_t *prior_occ,
                     coef_mlp model, const float *o, const float *goal, const float *color, int n)
    : cfg_(cfg), n_(n), rows_(cfg.rows), cols_(cfg.cols), model_(std::move(model)) {
  const long hw = static_cast<long>(rows_) * cols_;
  truth_.assign(truth, truth + hw);

  // Initial shared belief from the prior map: log-odds saturated to +/- l_clamp
  // so to_occupancy(logodds) == prior_occ.
  logodds_.resize(hw);
  lastvis_.assign(hw, 0);
  everseen_.assign(hw, 0);
  for (long i = 0; i < hw; ++i)
    logodds_[i] = prior_occ[i] ? static_cast<float>(cfg.l_clamp) : -static_cast<float>(cfg.l_clamp);
  version_ = 0;
  last_version_ = 0;
  dyn_stamp_.assign(hw, -std::numeric_limits<double>::infinity());

  // Initial occupancy + field.
  occ_.resize(hw);
  const unknown_policy pol =
      cfg.optimistic ? unknown_policy::optimistic : unknown_policy::pessimistic;
  composite_occupancy(logodds_.data(), rows_, cols_, pol, cfg.p_thresh, cfg.band, dyn_stamp_.data(),
                      0.0, cfg.ttl_s, occ_.data());
  field_.resize(3 * hw);
  rebuild_field();

  // SoA agent columns.
  o_.assign(o, o + 2 * n);
  goal_.assign(goal, goal + 2 * n);
  color_.assign(color, color + 3 * n);
  th_.resize(n);
  sp_.assign(n, 0.0f);
  stall_.assign(n, 0);
  mode_.assign(n, 0);
  turn_.assign(n, 1.0f);
  dhit_.assign(n, 0.0f);
  best_.resize(n);
  init_.resize(n);
  wall_entry_.assign(2 * n, 0.0f);
  we_valid_.assign(n, 0);
  tracking_.assign(n, 0);
  pos_hist_.assign(static_cast<long>(40) * 2 * n, 0.0f);
  hist_count_.assign(n, 0);
  parked_.assign(n, 0);
  reached_.assign(n, 0);
  active_.assign(n, 1);
  for (int i = 0; i < n; ++i) {
    const float dx = goal[2 * i] - o[2 * i], dy = goal[2 * i + 1] - o[2 * i + 1];
    th_[i] = std::atan2(dy, dx);
    const float d = std::sqrt(dx * dx + dy * dy);
    best_[i] = d;
    init_[i] = std::max(d, 1e-6f);
  }
}

sim_world sim_world::from_occupancy(const config &cfg, const std::uint8_t *occ, coef_mlp model,
                                    int n, unsigned seed) {
  const long hw = static_cast<long>(cfg.rows) * cfg.cols;
  std::vector<int> free_cells;
  for (long i = 0; i < hw; ++i)
    if (!occ[i])
      free_cells.push_back(static_cast<int>(i));
  if (free_cells.empty())
    for (long i = 0; i < hw; ++i)
      free_cells.push_back(static_cast<int>(i));

  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> pick(0, static_cast<int>(free_cells.size()) - 1);
  std::uniform_real_distribution<float> col(0.2f, 1.0f);
  auto cell_to_on = [&](int cell, float &onx, float &ony) {
    const int r = cell / cfg.cols, c = cell % cfg.cols;
    const double x = cfg.min_x + (double)c / (cfg.cols - 1) * (cfg.max_x - cfg.min_x);
    const double y = cfg.min_y + (double)r / (cfg.rows - 1) * (cfg.max_y - cfg.min_y);
    onx = static_cast<float>((x - cfg.cx) * cfg.scale);
    ony = static_cast<float>((y - cfg.cy) * cfg.scale);
  };
  std::vector<float> o(2 * n), goal(2 * n), color(3 * n);
  for (int i = 0; i < n; ++i) {
    cell_to_on(free_cells[pick(rng)], o[2 * i], o[2 * i + 1]);
    cell_to_on(free_cells[pick(rng)], goal[2 * i], goal[2 * i + 1]);
    color[3 * i] = col(rng);
    color[3 * i + 1] = col(rng);
    color[3 * i + 2] = col(rng);
  }
  return sim_world(cfg, occ, occ, std::move(model), o.data(), goal.data(), color.data(), n);
}

field_stack sim_world::field_view() const {
  field_stack fs;
  fs.data = field_.data();
  fs.M = 1;
  fs.H = rows_;
  fs.W = cols_;
  fs.mnx = cfg_.min_x;
  fs.mny = cfg_.min_y;
  fs.mxx = cfg_.max_x;
  fs.mxy = cfg_.max_y;
  fs.cx = cfg_.cx;
  fs.cy = cfg_.cy;
  fs.S = cfg_.scale;
  return fs;
}

void sim_world::rebuild_field() {
  const sdf_field f = build_sdf(occ_.data(), rows_, cols_, cfg_.min_x, cfg_.min_y, cfg_.max_x,
                                cfg_.max_y, cfg_.scale);
  // _finalize_field: clip phi to +/- 2*region_n (region = bounds max_x).
  const float clip = static_cast<float>(2.0 * cfg_.max_x * cfg_.scale);
  const long hw = static_cast<long>(rows_) * cols_;
  for (long i = 0; i < hw; ++i) {
    float p = f.phi[i];
    p = std::min(std::max(p, -clip), clip);
    field_[i] = p;
    field_[hw + i] = f.normal_x[i];
    field_[2 * hw + i] = f.normal_y[i];
  }
  ++field_ver_;
}

void sim_world::step(int num_threads) {
  if (!cfg_.freeze_sense && (gstep_ % cfg_.sense_every == 0)) {
    // ── SENSE (all agents into the single shared plane) ──
    std::vector<double> pos(2 * n_), head(n_), rng(n_), fov(n_);
    std::vector<int> nray(n_), amap(n_, 0);
    for (int i = 0; i < n_; ++i) {
      pos[2 * i] = o_[2 * i] / cfg_.scale + cfg_.cx;
      pos[2 * i + 1] = o_[2 * i + 1] / cfg_.scale + cfg_.cy;
      head[i] = th_[i];
      rng[i] = cfg_.range_m;
      fov[i] = cfg_.fov_rad;
      nray[i] = cfg_.n_rays;
    }
    sense_agents ag;
    ag.pos = pos.data();
    ag.heading = head.data();
    ag.range_m = rng.data();
    ag.fov_rad = fov.data();
    ag.n_rays = nray.data();
    ag.agent_map = amap.data();
    ag.n = n_;
    belief_planes pl;
    pl.logodds = logodds_.data();
    pl.last_visible = lastvis_.data();
    pl.ever_seen = everseen_.data();
    pl.version = &version_;
    pl.K = 1;
    std::vector<std::int32_t> flips(n_);
    sense_batch(truth_.data(), rows_, cols_, cfg_.min_x, cfg_.min_y, cfg_.max_x, cfg_.max_y, ag,
                nullptr, 0, nullptr, 0, pl, cfg_.l_occ, cfg_.l_free, cfg_.l_clamp, flips.data(),
                num_threads);

    // ── REBUILD the field iff the planning surface changed ──
    const double t_now = gstep_ * static_cast<double>(cfg_.veh.dt);
    const unknown_policy pol =
        cfg_.optimistic ? unknown_policy::optimistic : unknown_policy::pessimistic;
    std::vector<std::uint8_t> occ2(static_cast<long>(rows_) * cols_);
    composite_occupancy(logodds_.data(), rows_, cols_, pol, cfg_.p_thresh, cfg_.band,
                        dyn_stamp_.data(), t_now, cfg_.ttl_s, occ2.data());
    if (version_ != last_version_ || occ2 != occ_) {
      occ_.swap(occ2);
      rebuild_field();
      last_version_ = version_;
    }
  }

  const field_stack fs = field_view();

  // ── SAMPLE at the start-of-tick pose (for the carrot FSM's wall normal) ──
  std::vector<float> phi(n_), nrm(2 * n_);
  sdf_sample(fs, o_.data(), n_, nullptr, phi.data(), nrm.data(), num_threads);

  // ── CARROT FSM ──
  fsm_state s;
  s.stall = stall_.data();
  s.mode = mode_.data();
  s.turn = turn_.data();
  s.dhit = dhit_.data();
  s.best = best_.data();
  s.wall_entry = wall_entry_.data();
  s.we_valid = we_valid_.data();
  s.tracking = tracking_.data();
  s.pos_hist = pos_hist_.data();
  s.hist_count = hist_count_.data();
  s.parked = parked_.data();
  s.active = active_.data();
  carrot_params cp;
  cp.reach_tol = cfg_.reach_tol;
  cp.a_max = cfg_.veh.a_max;
  cp.dt = cfg_.veh.dt;
  std::vector<float> carrot(2 * n_);
  carrot_step(o_.data(), goal_.data(), th_.data(), sp_.data(), phi.data(), nrm.data(), s, n_, cp,
              carrot.data(), num_threads);

  // ── DRIVE (fused sample -> coef_feats -> coef_mlp -> bicycle) ──
  std::vector<float> minclr(n_);
  drive_step(fs, o_.data(), th_.data(), sp_.data(), carrot.data(), model_, n_, nullptr, cfg_.veh,
             minclr.data(), num_threads);

  // ── METRICS + REACHED/PARK (single-goal) ──
  for (int i = 0; i < n_; ++i) {
    const float dx = goal_[2 * i] - o_[2 * i], dy = goal_[2 * i + 1] - o_[2 * i + 1];
    const float dg = std::sqrt(dx * dx + dy * dy);
    const bool r = dg < cfg_.reach_tol;
    reached_[i] = r ? 1 : 0;
    if (r && !parked_[i] && active_[i])
      parked_[i] = 1;
  }
  ++gstep_;
}

void sim_world::snapshot(float *pos_world, float *heading, float *speed, int *mode,
                         std::uint8_t *reached) const {
  for (int i = 0; i < n_; ++i) {
    if (pos_world) {
      pos_world[2 * i] = o_[2 * i] / static_cast<float>(cfg_.scale) + static_cast<float>(cfg_.cx);
      pos_world[2 * i + 1] =
          o_[2 * i + 1] / static_cast<float>(cfg_.scale) + static_cast<float>(cfg_.cy);
    }
    if (heading)
      heading[i] = th_[i];
    if (speed)
      speed[i] = sp_[i] / static_cast<float>(cfg_.scale);
    if (mode)
      mode[i] = mode_[i];
    if (reached)
      reached[i] = reached_[i];
  }
}

void sim_world::retarget(int i, float gx_n, float gy_n) {
  if (i < 0 || i >= n_)
    return;
  goal_[2 * i] = gx_n;
  goal_[2 * i + 1] = gy_n;
  const float dx = gx_n - o_[2 * i], dy = gy_n - o_[2 * i + 1];
  const float d = std::sqrt(dx * dx + dy * dy);
  best_[i] = std::min(best_[i], d);
  init_[i] = std::max(std::max(init_[i], d), 1e-6f);
  tracking_[i] = 1;
  reached_[i] = 0;
  parked_[i] = 0;
}

void sim_world::add_obstacle(int r0, int r1, int c0, int c1) {
  const double t_now = gstep_ * static_cast<double>(cfg_.veh.dt);
  const int rr = (r0 + r1) / 2, cc = (c0 + c1) / 2;
  const int rad = std::max(1, std::max(std::abs(r1 - r0) / 2, std::abs(c1 - c0) / 2));
  for (int r = std::max(0, rr - rad); r < std::min(rows_, rr + rad + 1); ++r)
    for (int c = std::max(0, cc - rad); c < std::min(cols_, cc + rad + 1); ++c)
      dyn_stamp_[static_cast<long>(r) * cols_ + c] = t_now;
}

} // namespace nav
} // namespace cvc
