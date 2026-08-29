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

// sim_world.cpp — see sim_world.h. The step() mirrors grl_snam/swarm.py Swarm.step:
// sense (gated) -> per-plane occupancy/field rebuild -> carrot FSM -> fused drive
// -> reached/park. Belief is M planes (shared M=1 / clustered / private M=N) via a
// per-agent map_id, exactly like the Python Swarm's grouped belief; sense_batch,
// sdf_sample and drive_step all take map_id, so the only per-plane bookkeeping is
// the M log-odds / occupancy / SDF blocks here.

#include <algorithm>
#include <cmath>
#include <cvc/nav/belief_occupancy.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/sim_world.h>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace nav {

sim_world::sim_world(const config &cfg, const std::uint8_t *truth, const std::uint8_t *prior_occ,
                     coef_mlp model, const float *o, const float *goal, const float *color, int n,
                     const int *map_id, int n_planes)
    : cfg_(cfg), n_(n), rows_(cfg.rows), cols_(cfg.cols), model_(std::move(model)) {
  const long hw = static_cast<long>(rows_) * cols_;
  M_ = map_id ? std::max(1, n_planes) : 1;
  map_id_.assign(n, 0);
  if (map_id)
    for (int i = 0; i < n; ++i) {
      if (map_id[i] < 0 || map_id[i] >= M_)
        throw std::runtime_error("cvc::nav::sim_world: map_id out of range [0, n_planes)");
      map_id_[i] = map_id[i];
    }
  truth_.assign(truth, truth + hw);

  // M belief planes, each seeded from the prior map (log-odds saturated to +/-
  // l_clamp so to_occupancy(logodds) == prior_occ); they diverge as agents sense.
  const long Mhw = static_cast<long>(M_) * hw;
  logodds_.resize(Mhw);
  lastvis_.assign(Mhw, 0);
  everseen_.assign(Mhw, 0);
  for (int m = 0; m < M_; ++m)
    for (long i = 0; i < hw; ++i)
      logodds_[static_cast<long>(m) * hw + i] =
          prior_occ[i] ? static_cast<float>(cfg.l_clamp) : -static_cast<float>(cfg.l_clamp);
  version_.assign(M_, 0);
  last_version_.assign(M_, 0);
  dyn_stamp_.assign(Mhw, -std::numeric_limits<double>::infinity());

  occ_.resize(Mhw);
  const unknown_policy pol =
      cfg.optimistic ? unknown_policy::optimistic : unknown_policy::pessimistic;
  for (int m = 0; m < M_; ++m)
    composite_occupancy(logodds_.data() + static_cast<long>(m) * hw, rows_, cols_, pol,
                        cfg.p_thresh, cfg.band, dyn_stamp_.data() + static_cast<long>(m) * hw, 0.0,
                        cfg.ttl_s, occ_.data() + static_cast<long>(m) * hw);
  field_.resize(static_cast<long>(M_) * 3 * hw);
  rebuild_all_fields();

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

void sim_world::scatter_free(const config &cfg, const std::uint8_t *occ, int n, unsigned seed,
                             float *o, float *goal, float *color) {
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
  for (int i = 0; i < n; ++i) {
    cell_to_on(free_cells[pick(rng)], o[2 * i], o[2 * i + 1]);
    cell_to_on(free_cells[pick(rng)], goal[2 * i], goal[2 * i + 1]);
    color[3 * i] = col(rng);
    color[3 * i + 1] = col(rng);
    color[3 * i + 2] = col(rng);
  }
}

sim_world sim_world::from_occupancy(const config &cfg, const std::uint8_t *occ, coef_mlp model,
                                    int n, unsigned seed, belief_mode mode, int clusters) {
  std::vector<float> o(2 * n), goal(2 * n), color(3 * n);
  scatter_free(cfg, occ, n, seed, o.data(), goal.data(), color.data());
  if (mode == belief_mode::shared)
    return sim_world(cfg, occ, occ, std::move(model), o.data(), goal.data(), color.data(), n);

  std::vector<int> map_id(n, 0);
  int M = 1;
  if (mode == belief_mode::private_belief) {
    for (int i = 0; i < n; ++i)
      map_id[i] = i;
    M = n;
  } else { // clustered: k-means-lite on start positions, then densify labels
    const int K = std::max(1, std::min(clusters, n));
    std::mt19937 rng(seed + 1);
    std::vector<int> idx(n);
    for (int i = 0; i < n; ++i)
      idx[i] = i;
    std::shuffle(idx.begin(), idx.end(), rng);
    std::vector<float> cen(2 * K);
    for (int k = 0; k < K; ++k) {
      cen[2 * k] = o[2 * idx[k]];
      cen[2 * k + 1] = o[2 * idx[k] + 1];
    }
    for (int it = 0; it < 15; ++it) {
      for (int i = 0; i < n; ++i) {
        int bestk = 0;
        float bestd = std::numeric_limits<float>::infinity();
        for (int k = 0; k < K; ++k) {
          const float dx = o[2 * i] - cen[2 * k], dy = o[2 * i + 1] - cen[2 * k + 1];
          const float d = dx * dx + dy * dy;
          if (d < bestd) {
            bestd = d;
            bestk = k;
          }
        }
        map_id[i] = bestk;
      }
      std::vector<double> sx(K, 0), sy(K, 0);
      std::vector<int> cnt(K, 0);
      for (int i = 0; i < n; ++i) {
        sx[map_id[i]] += o[2 * i];
        sy[map_id[i]] += o[2 * i + 1];
        ++cnt[map_id[i]];
      }
      for (int k = 0; k < K; ++k)
        if (cnt[k]) {
          cen[2 * k] = static_cast<float>(sx[k] / cnt[k]);
          cen[2 * k + 1] = static_cast<float>(sy[k] / cnt[k]);
        }
    }
    // Densify to a gapless [0, M): a cluster may end up empty.
    std::vector<int> remap(K, -1);
    M = 0;
    for (int i = 0; i < n; ++i) {
      if (remap[map_id[i]] < 0)
        remap[map_id[i]] = M++;
      map_id[i] = remap[map_id[i]];
    }
  }
  return sim_world(cfg, occ, occ, std::move(model), o.data(), goal.data(), color.data(), n,
                   map_id.data(), M);
}

field_stack sim_world::field_view() const {
  field_stack fs;
  fs.data = field_.data();
  fs.M = M_;
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

void sim_world::rebuild_plane(int m) {
  const long hw = static_cast<long>(rows_) * cols_;
  const sdf_field f = build_sdf(occ_.data() + static_cast<long>(m) * hw, rows_, cols_, cfg_.min_x,
                                cfg_.min_y, cfg_.max_x, cfg_.max_y, cfg_.scale);
  // _finalize_field: clip phi to +/- 2*region_n (region = bounds max_x).
  const float clip = static_cast<float>(2.0 * cfg_.max_x * cfg_.scale);
  float *fp = field_.data() + static_cast<long>(m) * 3 * hw;
  for (long i = 0; i < hw; ++i) {
    fp[i] = std::min(std::max(f.phi[i], -clip), clip);
    fp[hw + i] = f.normal_x[i];
    fp[2 * hw + i] = f.normal_y[i];
  }
}

void sim_world::rebuild_all_fields() {
  for (int m = 0; m < M_; ++m)
    rebuild_plane(m);
  ++field_ver_;
}

void sim_world::step(int num_threads) {
  const long hw = static_cast<long>(rows_) * cols_;
  if (!cfg_.freeze_sense && (gstep_ % cfg_.sense_every == 0)) {
    // ── SENSE (each agent into its own plane map_id[i]) ──
    std::vector<double> pos(2 * n_), head(n_), rng(n_), fov(n_);
    std::vector<int> nray(n_);
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
    ag.agent_map = map_id_.data();
    ag.n = n_;
    belief_planes pl;
    pl.logodds = logodds_.data();
    pl.last_visible = lastvis_.data();
    pl.ever_seen = everseen_.data();
    pl.version = version_.data();
    pl.K = M_;
    std::vector<std::int32_t> flips(n_);
    sense_batch(truth_.data(), rows_, cols_, cfg_.min_x, cfg_.min_y, cfg_.max_x, cfg_.max_y, ag,
                nullptr, 0, nullptr, 0, pl, cfg_.l_occ, cfg_.l_free, cfg_.l_clamp, flips.data(),
                num_threads);

    // ── REBUILD each plane whose planning surface changed ──
    const double t_now = gstep_ * static_cast<double>(cfg_.veh.dt);
    const unknown_policy pol =
        cfg_.optimistic ? unknown_policy::optimistic : unknown_policy::pessimistic;
    std::vector<std::uint8_t> occ2(hw);
    bool any = false;
    for (int m = 0; m < M_; ++m) {
      const long off = static_cast<long>(m) * hw;
      composite_occupancy(logodds_.data() + off, rows_, cols_, pol, cfg_.p_thresh, cfg_.band,
                          dyn_stamp_.data() + off, t_now, cfg_.ttl_s, occ2.data());
      if (version_[m] != last_version_[m] ||
          !std::equal(occ2.begin(), occ2.end(), occ_.begin() + off)) {
        std::copy(occ2.begin(), occ2.end(), occ_.begin() + off);
        rebuild_plane(m);
        last_version_[m] = version_[m];
        any = true;
      }
    }
    if (any)
      ++field_ver_;
  }

  const field_stack fs = field_view();

  // ── SAMPLE at the start-of-tick pose (per-agent plane) for the carrot FSM ──
  std::vector<float> phi(n_), nrm(2 * n_);
  sdf_sample(fs, o_.data(), n_, map_id_.data(), phi.data(), nrm.data(), num_threads);

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
  // carrot_ is a member so renderers can visualize each agent's live steering
  // target (carrots_world) — the FSM's output, not just its consequence.
  carrot_.assign(static_cast<std::size_t>(2) * n_, 0.0f);
  carrot_step(o_.data(), goal_.data(), th_.data(), sp_.data(), phi.data(), nrm.data(), s, n_, cp,
              carrot_.data(), num_threads);

  // ── INTER-AGENT SEPARATION (optional) ──
  // Nudge each agent's carrot away from peers within sep_radius so a crowd
  // steers AROUND itself instead of driving through. Reactive and per-agent, so
  // it works in every belief mode (the shared-plane scale path can't stamp peers
  // per agent). Off unless the host opts in (sep_gain > 0); the neighbour query
  // is the same CGAL kd-tree the squad uses (grid_nav.h neighbors_within_radius).
  if (cfg_.sep_gain > 0.0f && cfg_.sep_radius > 0.0f && n_ > 1) {
    std::vector<double> pw(static_cast<std::size_t>(2) * n_);
    for (int i = 0; i < 2 * n_; ++i)
      pw[i] = o_[i]; // separation works in the normalized frame (o_/carrot_)
    const neighbor_csr csr = neighbors_within_radius(pw.data(), n_, cfg_.sep_radius);
    const float R = cfg_.sep_radius;
    for (int i = 0; i < n_; ++i) {
      float sx = 0.0f, sy = 0.0f;
      for (int k = csr.offsets[i]; k < csr.offsets[i + 1]; ++k) {
        const int j = csr.indices[k];
        float dx = o_[2 * i] - o_[2 * j], dy = o_[2 * i + 1] - o_[2 * j + 1];
        float d2 = dx * dx + dy * dy;
        if (d2 < 1e-10f) { // coincident: split deterministically by index
          dx = (i < j) ? 1.0f : -1.0f;
          dy = 0.0f;
          d2 = 1.0f;
        }
        const float d = std::sqrt(d2);
        float w = (R - d) / R; // 1 at contact -> 0 at the radius
        if (w < 0.0f)
          w = 0.0f;
        const float inv = w * w / d; // quadratic falloff along the unit separation dir
        sx += dx * inv;
        sy += dy * inv;
      }
      carrot_[2 * i] += cfg_.sep_gain * sx;
      carrot_[2 * i + 1] += cfg_.sep_gain * sy;
    }
  }

  // ── MATERIAL GATE (optional): frame-wise witness per agent vs its own goal ──
  // The gate multiplies lam_soft ONLY; lam_hard is always on. Evaluated in
  // CONTINUOUS CELL coords (world computed in f32 like the Python n2w, then
  // widened) against the static gate surface (material hard | truth).
  if (mat_on_) {
    if (mat_cfg_.gate_enabled) {
      std::vector<double> pos_rc(static_cast<std::size_t>(2) * n_),
          goal_rc(static_cast<std::size_t>(2) * n_);
      const double sx = (cols_ - 1) / (cfg_.max_x - cfg_.min_x);
      const double sy = (rows_ - 1) / (cfg_.max_y - cfg_.min_y);
      for (int i = 0; i < n_; ++i) {
        const float wx = o_[2 * i] / static_cast<float>(cfg_.scale) + static_cast<float>(cfg_.cx);
        const float wy =
            o_[2 * i + 1] / static_cast<float>(cfg_.scale) + static_cast<float>(cfg_.cy);
        const float gwx =
            goal_[2 * i] / static_cast<float>(cfg_.scale) + static_cast<float>(cfg_.cx);
        const float gwy =
            goal_[2 * i + 1] / static_cast<float>(cfg_.scale) + static_cast<float>(cfg_.cy);
        pos_rc[2 * i] = (static_cast<double>(wy) - cfg_.min_y) * sy;
        pos_rc[2 * i + 1] = (static_cast<double>(wx) - cfg_.min_x) * sx;
        goal_rc[2 * i] = (static_cast<double>(gwy) - cfg_.min_y) * sy;
        goal_rc[2 * i + 1] = (static_cast<double>(gwx) - cfg_.min_x) * sx;
      }
      gate_params gp = mat_cfg_.gate;
      gp.hard_margin_m = mat_hard_margin_m_;
      std::vector<double> nom(n_), best(n_);
      std::vector<std::int32_t> cnt(n_);
      // Per-plane gate: risk/gate_hard/clear_m are [mat_planes_,H,W]; each agent
      // gates against its own material plane via map_id (nullptr => plane 0 in the
      // shared case). mat_risk_ is the contiguous channel-0 risk view.
      witness_gate_batch(mat_risk_.data(), mat_gate_hard_.data(), mat_clear_m_.data(), rows_, cols_,
                         pos_rc.data(), goal_rc.data(), n_, gp, mat_gate_active_.data(), nom.data(),
                         best.data(), cnt.data(), num_threads,
                         mat_planes_ > 1 ? map_id_.data() : nullptr);
    } else {
      std::fill(mat_gate_active_.begin(), mat_gate_active_.end(), std::uint8_t(1));
    }
    for (int i = 0; i < n_; ++i) {
      mat_lam_soft_[i] = mat_gate_active_[i] ? mat_cfg_.lam_soft : 0.0f;
      mat_lam_hard_[i] = mat_cfg_.lam_hard;
    }
  }

  // ── DRIVE (fused sample -> coef_feats -> coef_mlp -> bicycle, per-agent plane) ──
  std::vector<float> minclr(n_);
  if (mat_on_) {
    const material_stack ms = material_view();
    material_drive md;
    md.stack = &ms;
    md.lam_soft = mat_lam_soft_.data();
    md.lam_hard = mat_lam_hard_.data();
    md.k_sharp = mat_cfg_.k_sharp;
    md.d_hat_m = mat_cfg_.d_hat_m;
    drive_step_material(fs, o_.data(), th_.data(), sp_.data(), carrot_.data(), model_, n_,
                        map_id_.data(), cfg_.veh, md, minclr.data(), num_threads);
  } else {
    drive_step(fs, o_.data(), th_.data(), sp_.data(), carrot_.data(), model_, n_, map_id_.data(),
               cfg_.veh, minclr.data(), num_threads);
  }

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

material_stack sim_world::material_view() const {
  material_stack m;
  m.data = mat_stack_.data();
  m.M = mat_planes_;
  m.H = rows_;
  m.W = cols_;
  m.mnx = cfg_.min_x;
  m.mny = cfg_.min_y;
  m.mxx = cfg_.max_x;
  m.mxy = cfg_.max_y;
  m.cx = cfg_.cx;
  m.cy = cfg_.cy;
  m.S = cfg_.scale;
  return m;
}

void sim_world::set_material(const float *risk_raw, const std::uint8_t *hard,
                             const material_config &mc, int planes) {
  if (!risk_raw || !hard) {
    mat_on_ = false;
    return;
  }
  if (planes < 1)
    throw std::runtime_error("cvc::nav::sim_world::set_material: planes must be >= 1");
  // With per-plane material, agents index their plane by the SAME map_id as their
  // belief plane, so every map_id must be a valid material-plane index.
  if (planes > 1)
    for (int i = 0; i < n_; ++i)
      if (map_id_[i] >= planes)
        throw std::runtime_error("cvc::nav::sim_world::set_material: agent map_id >= planes");
  mat_planes_ = planes;
  mat_cfg_ = mc;
  const double cell_w = (cfg_.max_x - cfg_.min_x) / (cols_ - 1);
  const std::size_t hw = static_cast<std::size_t>(rows_) * cols_;
  // One [1,6,H,W] derived stack + a contiguous channel-0 risk view + a gate/
  // clearance surface PER plane, concatenated. Plane m reads risk_raw/hard at
  // m*hw; its gate feasibility surface is (that plane's hard) | TRUTH occupancy
  // (the oracle setting; truth is shared across planes), and its metres clearance
  // plane is one EDT of that — computed here, at set time.
  mat_stack_.assign(static_cast<std::size_t>(planes) * 6 * hw, 0.0f);
  mat_risk_.assign(static_cast<std::size_t>(planes) * hw, 0.0f);
  mat_gate_hard_.assign(static_cast<std::size_t>(planes) * hw, 0);
  mat_clear_m_.assign(static_cast<std::size_t>(planes) * hw, 0.0f);
  for (int m = 0; m < planes; ++m) {
    const std::size_t roff = static_cast<std::size_t>(m) * hw;
    const material_planes mp =
        material_build(risk_raw + roff, hard + roff, rows_, cols_, cell_w, cfg_.scale, mc.sigma);
    const std::vector<float> stacked = mp.stacked(); // [1,6,H,W]
    std::copy(stacked.begin(), stacked.end(),
              mat_stack_.begin() + static_cast<std::size_t>(m) * 6 * hw);
    std::copy(stacked.begin(), stacked.begin() + hw, mat_risk_.begin() + roff); // ch 0 = risk
    std::uint8_t *gh = mat_gate_hard_.data() + roff;
    for (std::size_t i = 0; i < hw; ++i)
      gh[i] = (hard[roff + i] || truth_[i]) ? 1 : 0;
    const std::vector<double> d2 = edt2_squared(gh, rows_, cols_);
    float *cm = mat_clear_m_.data() + roff;
    for (std::size_t i = 0; i < hw; ++i)
      cm[i] = static_cast<float>(std::sqrt(d2[i]) * cell_w);
  }
  mat_hard_margin_m_ = mc.gate.hard_margin_m > 0.0 ? mc.gate.hard_margin_m : 2.0 * cell_w;
  mat_lam_soft_.assign(n_, 0.0f);
  mat_lam_hard_.assign(n_, 0.0f);
  mat_gate_active_.assign(n_, 0);
  mat_on_ = true;
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

void sim_world::goals_world(float *out) const {
  for (int i = 0; i < n_; ++i) {
    out[2 * i] = goal_[2 * i] / static_cast<float>(cfg_.scale) + static_cast<float>(cfg_.cx);
    out[2 * i + 1] =
        goal_[2 * i + 1] / static_cast<float>(cfg_.scale) + static_cast<float>(cfg_.cy);
  }
}

void sim_world::carrots_world(float *out) const {
  // Before the first step() the FSM hasn't run; fall back to the goals so a
  // renderer's first frame points somewhere sensible.
  const std::vector<float> &src = carrot_.empty() ? goal_ : carrot_;
  for (int i = 0; i < n_; ++i) {
    out[2 * i] = src[2 * i] / static_cast<float>(cfg_.scale) + static_cast<float>(cfg_.cx);
    out[2 * i + 1] = src[2 * i + 1] / static_cast<float>(cfg_.scale) + static_cast<float>(cfg_.cy);
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
  const long hw = static_cast<long>(rows_) * cols_;
  const double t_now = gstep_ * static_cast<double>(cfg_.veh.dt);
  const int rr = (r0 + r1) / 2, cc = (c0 + c1) / 2;
  const int rad = std::max(1, std::max(std::abs(r1 - r0) / 2, std::abs(c1 - c0) / 2));
  // A real obstacle enters every plane's dynamic layer (all agents route around
  // it once they sense it). It only takes effect on the sense/rebuild path.
  for (int m = 0; m < M_; ++m)
    for (int r = std::max(0, rr - rad); r < std::min(rows_, rr + rad + 1); ++r)
      for (int c = std::max(0, cc - rad); c < std::min(cols_, cc + rad + 1); ++c)
        dyn_stamp_[static_cast<long>(m) * hw + static_cast<long>(r) * cols_ + c] = t_now;
}

} // namespace nav
} // namespace cvc
