/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// CoPH-Stagewise's packet exchange (C4 of docs/roadmap/NAV-COPH-ROADMAP.md, §4.4); see
// coph_exchange.h. Each function names the GRL-SNAM (grl_snam.coph) function it follows. Built
// with -ffp-contract=off, so its arithmetic is the Python's, operation for operation.

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cvc/algo/det_math.h>
#include <cvc/nav/coph_exchange.h>
#include <cvc/nav/detail/coph_math.h>
#include <cvc/nav/detail/seeded_draw.h>
#include <limits>
#include <set>
#include <stdexcept>
#include <tuple>

namespace cvc {
namespace nav {

namespace {

using detail::coph_cholesky;
using detail::coph_cholesky_solve;
using detail::coph_features;
using detail::coph_forward;
using detail::coph_shape_less;
using detail::coph_spd_inverse;

constexpr double kSupportRadius = 0.65; // Stage.supported, TerrainLearner's route support
constexpr double kAnchorSpacing = 0.3;  // TerrainLearner.export
constexpr double kCell = 0.2;           // TerrainLearner's cells
constexpr std::size_t kRemoteCells = 128;
constexpr std::size_t kRemoteTerrain = 8;
constexpr std::size_t kGeometryReports = 32;

double distance(const coph_vec2 &a, const coph_vec2 &b) {
  const double dx = a[0] - b[0], dy = a[1] - b[1];
  return std::sqrt(dx * dx + dy * dy);
}

// Stage.contains.
bool contains(const coph_stage &st, const coph_vec2 &q, const coph_config &cfg) {
  return std::fabs(q[0] - st.center[0]) <= cfg.stage_width / 2 + 1e-10 &&
         std::fabs(q[1] - st.center[1]) <= cfg.stage_height / 2 + 1e-10;
}

// Stage.supported: inside the box, and within 0.65 of a fitting reading.
bool supported(const coph_stage &st, const coph_vec2 &q, const coph_config &cfg) {
  if (!contains(st, q, cfg))
    return false;
  for (const coph_reading &r : st.readings)
    if (!coph_is_validation(r.uid) && distance(q, r.q) <= kSupportRadius)
      return true;
  return false;
}

bool starts_with(const std::string &s, const std::string &prefix) {
  return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

// numpy.isclose's default test, a against b: |a - b| <= 1e-8 + 1e-5 |b|; equal infinities pass.
bool close(double a, double b) {
  if (a == b)
    return true;
  if (!std::isfinite(a) || !std::isfinite(b))
    return false;
  return std::fabs(a - b) <= 1e-8 + 1e-5 * std::fabs(b);
}

// Python's f'{v:.9f}'.
std::string fixed9(double v) {
  char buf[512];
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
  const std::to_chars_result r =
      std::to_chars(buf, buf + sizeof buf, v, std::chars_format::fixed, 9);
  return std::string(buf, r.ptr);
#else
  std::snprintf(buf, sizeof buf, "%.9f", v);
  return buf;
#endif
}

std::vector<const coph_reading *> export_readings(const coph_stage &st) {
  std::vector<const coph_reading *> out;
  for (const coph_reading &r : st.readings)
    if (!coph_is_validation(r.uid))
      out.push_back(&r);
  return out;
}

long long block_key(const coph_reading &r, const coph_config &cfg) {
  return static_cast<long long>(r.time / (cfg.dt * cfg.update_every));
}

// bootstrap_field's fit: the ridge solution towards w0 = -1/m.
std::vector<double> ridge_fit(const std::vector<const coph_reading *> &rs, const coph_stage &st,
                              const coph_config &cfg) {
  const int m = static_cast<int>(st.centers.size());
  std::vector<double> normal(static_cast<std::size_t>(m) * m, 0.0), rhs(m, 0.0);
  for (const coph_reading *r : rs) {
    const std::vector<double> phi = coph_features(r->q, st.centers, cfg.rbf_width);
    const double y =
        algo::det::log(algo::det::expm1(4 * std::max(0.001, std::min(r->value, 20.0))));
    for (int i = 0; i < m; ++i) {
      for (int k = 0; k < m; ++k)
        normal[i * m + k] += phi[i] * phi[k];
      rhs[i] += phi[i] * y;
    }
  }
  const double prior = -1.0 / m;
  for (int i = 0; i < m; ++i) {
    normal[i * m + i] += cfg.fit_ridge;
    rhs[i] += cfg.fit_ridge * prior;
  }
  coph_cholesky(normal, m, "coph_bootstrap_field: the normal matrix is not positive definite");
  return coph_cholesky_solve(normal, m, rhs);
}

std::vector<double> field_vector(const std::vector<double> &w, const coph_stage &st,
                                 const coph_config &cfg, const std::vector<coph_vec2> &points) {
  coph_field_view f;
  f.centers = st.centers;
  f.weights = w;
  f.width = cfg.rbf_width;
  std::vector<double> out;
  for (const coph_vec2 &p : points) {
    coph_vec2_of<coph_dual2> q;
    q[0] = coph_dual2::variable(p[0], 0);
    q[1] = coph_dual2::variable(p[1], 1);
    const coph_dual2 r = coph_risk<coph_dual2>(q, f);
    out.push_back(r.v);
    out.push_back(r.d[0]);
    out.push_back(r.d[1]);
  }
  return out;
}

// scale (numpy.cov(vectors, rowvar=False) + floor I): rows are the members, divided by n - 1.
std::vector<std::vector<double>> ensemble_covariance(const std::vector<std::vector<double>> &x,
                                                     double scale, double floor) {
  const std::size_t n = x.size(), d = x.empty() ? 0 : x[0].size();
  std::vector<double> mean(d, 0.0);
  for (const auto &row : x)
    for (std::size_t j = 0; j < d; ++j)
      mean[j] += row[j];
  for (double &m : mean)
    m /= static_cast<double>(n);
  const double fact = 1.0 / static_cast<double>(n - 1);
  std::vector<std::vector<double>> c(d, std::vector<double>(d, 0.0));
  for (std::size_t j = 0; j < d; ++j)
    for (std::size_t k = 0; k < d; ++k) {
      double s = 0.0;
      for (const auto &row : x)
        s += (row[j] - mean[j]) * (row[k] - mean[k]);
      c[j][k] = scale * (s * fact + (j == k ? floor : 0.0));
    }
  return c;
}

std::vector<coph_anchor> anchors_at(const std::vector<coph_vec2> &points,
                                    const std::vector<double> &vector) {
  std::vector<coph_anchor> out;
  for (std::size_t i = 0; i < points.size(); ++i)
    out.push_back(
        {points[i][0], points[i][1], vector[3 * i], vector[3 * i + 1], vector[3 * i + 2]});
  return out;
}

coph_vec2 target_of(const coph_learner_state &s) {
  return s.navigation_target ? *s.navigation_target : s.goal;
}

template <class Entry, class Key>
typename std::vector<Entry>::iterator find_key(std::vector<Entry> &v, const Key &key) {
  return std::find_if(v.begin(), v.end(), [&](const Entry &e) { return e.sender == key; });
}

} // namespace

// ---------------------------------------------------------------------------
// G9: export
// ---------------------------------------------------------------------------

std::vector<double> coph_field_vector(const coph_stage &stage, const std::vector<double> &weights,
                                      const coph_config &cfg,
                                      const std::vector<coph_vec2> &points) {
  return field_vector(weights, stage, cfg, points);
}

int coph_bootstrap_blocks(const coph_learner_state &s) {
  if (s.stages.empty())
    return 0;
  std::vector<long long> keys;
  for (const coph_reading *r : export_readings(s.stages.back())) {
    const long long k = block_key(*r, s.cfg);
    if (std::find(keys.begin(), keys.end(), k) == keys.end())
      keys.push_back(k);
  }
  return static_cast<int>(keys.size());
}

coph_bootstrap coph_bootstrap_field(const coph_stage &stage, const coph_config &cfg,
                                    const coph_draws &draws) {
  coph_bootstrap out;
  const std::vector<const coph_reading *> rs = export_readings(stage);
  std::vector<std::vector<const coph_reading *>> blocks;
  for (const coph_reading *r : rs) {
    const long long k = block_key(*r, cfg);
    auto it = std::find_if(out.blocks.begin(), out.blocks.end(),
                           [&](const auto &b) { return b.first == k; });
    if (it == out.blocks.end()) {
      out.blocks.push_back({k, 0});
      blocks.emplace_back();
      it = out.blocks.end() - 1;
    }
    ++it->second;
    blocks[static_cast<std::size_t>(it - out.blocks.begin())].push_back(r);
  }
  if (rs.empty())
    return out;
  out.mean = ridge_fit(rs, stage, cfg);
  for (const std::vector<int> &member : draws) {
    std::vector<const coph_reading *> sample;
    for (int k : member) {
      if (k < 0 || k >= static_cast<int>(blocks.size()))
        throw std::invalid_argument("coph_bootstrap_field: a draw names no block");
      sample.insert(sample.end(), blocks[k].begin(), blocks[k].end());
    }
    out.ensemble.push_back(ridge_fit(sample, stage, cfg));
  }
  return out;
}

coph_draws coph_draw_blocks(std::mt19937 &rng, int blocks, int members) {
  coph_draws out(members);
  for (std::vector<int> &member : out)
    for (int i = 0; i < blocks; ++i)
      member.push_back(detail::pick_below(rng, blocks));
  return out;
}

std::optional<coph_packet> coph_export(const coph_learner_state &s, const coph_state &x,
                                       const std::string &recipient,
                                       const std::array<coph_draws, 2> &draws,
                                       coph_export_trace *trace) {
  if (s.stages.empty())
    return std::nullopt;
  const coph_config &cfg = s.cfg;
  const coph_communication_config &comm = s.communication;
  const coph_stage &st = s.stages.back();
  const std::vector<const coph_reading *> rs = export_readings(st);
  if (rs.empty())
    return std::nullopt;
  coph_export_trace local;
  coph_export_trace &t = trace ? *trace : local;

  // CooperativeLearner.export.
  t.cooperative = coph_bootstrap_field(st, cfg, draws[0]);
  const coph_vec2 q{x[0], x[1]};
  const coph_vec2 offsets[] = {{0.0, 0.0}, {0.4, 0.0}, {0.0, 0.4}, {0.0, -0.4}};
  std::vector<coph_vec2> points;
  for (const coph_vec2 &o : offsets) {
    const coph_vec2 c{q[0] + o[0], q[1] + o[1]};
    const bool ok = supported(st, c, cfg);
    t.candidates.push_back({c, ok});
    if (ok && static_cast<int>(points.size()) < comm.anchors)
      points.push_back(c);
  }
  if (points.empty())
    return std::nullopt;
  std::vector<std::vector<double>> vectors;
  for (const auto &w : t.cooperative.ensemble)
    vectors.push_back(field_vector(w, st, cfg, points));
  t.cooperative_anchors = anchors_at(points, field_vector(t.cooperative.mean, st, cfg, points));
  t.cooperative_covariance =
      ensemble_covariance(vectors, comm.covariance_scale, comm.covariance_floor);

  coph_packet p;
  const coph_vec2 target = target_of(s);
  coph_field_view f;
  f.centers = st.centers;
  f.weights = t.cooperative.mean;
  f.width = cfg.rbf_width;
  f.obstacles = s.geometry; // control_obstacles: the geometry store, under routed control
  const double kinetic = 0.5 * (x[2] * x[2] + x[3] * x[3]);
  p.H = kinetic + coph_potential<double>(q, target, s.policy, f, coph_shaping_for(f, cfg));
  const double tx = q[0] - target[0], ty = q[1] - target[1];
  p.V_task = 0.5 * (tx * tx + ty * ty);
  p.schema = 1;
  p.sender = s.agent_id;
  p.recipient = recipient;
  p.identity = s.agent_id + ":" + fixed9(s.now) + ":" + std::to_string(st.id);
  p.timestamp = s.now;
  p.expires = s.now + comm.ttl;
  p.version = s.version;
  p.convention = coph_convention;
  p.q = q;
  p.p = {x[2], x[3]};
  p.v = {x[2], x[3]};
  p.goal = s.goal;
  p.bounds = {st.center[0] - cfg.stage_width / 2, st.center[0] + cfg.stage_width / 2,
              st.center[1] - cfg.stage_height / 2, st.center[1] + cfg.stage_height / 2};
  p.anchors = t.cooperative_anchors;
  p.covariance = t.cooperative_covariance;
  std::set<std::string> uids;
  for (const coph_reading *r : rs)
    uids.insert(r->uid);
  p.sources.assign(uids.begin(), uids.end());
  p.stage = st.id;
  for (const coph_reading *r : rs)
    if (std::find(p.support_points.begin(), p.support_points.end(), r->q) == p.support_points.end())
      p.support_points.push_back(r->q);
  p.uncertainty = "block_bootstrap; calibration_scale=" + coph_repr(comm.covariance_scale);
  t.path = "cooperative";
  if (s.network.nodes.empty()) { // a car's learner: CooperativeLearner's packet alone
    p.maze = false;
    return p;
  }

  // TerrainLearner.export: the supported readings with the most risk, 0.3 apart.
  t.terrain = coph_bootstrap_field(st, cfg, draws[1]);
  std::vector<const coph_reading *> order = rs;
  std::stable_sort(order.begin(), order.end(), [](const coph_reading *a, const coph_reading *b) {
    if (a->value != b->value)
      return -a->value < -b->value;
    return a->uid < b->uid;
  });
  std::vector<coph_vec2> picked;
  for (const coph_reading *r : order) {
    bool apart = true;
    for (const coph_vec2 &other : picked)
      apart = apart && distance(r->q, other) > kAnchorSpacing;
    if (supported(st, r->q, cfg) && apart) {
      picked.push_back(r->q);
      t.picked.push_back(r);
    }
    if (static_cast<int>(picked.size()) == comm.anchors)
      break;
  }
  if (!picked.empty()) {
    std::vector<std::vector<double>> vs;
    for (const auto &w : t.terrain.ensemble)
      vs.push_back(field_vector(w, st, cfg, picked));
    t.terrain_anchors = anchors_at(picked, field_vector(t.terrain.mean, st, cfg, picked));
    t.terrain_covariance = ensemble_covariance(vs, 1.0, comm.covariance_floor);
    p.anchors = t.terrain_anchors;
    p.covariance = t.terrain_covariance;
    t.path = "terrain";
  }

  // MazeLearner.export.
  p.geometry_schema = 1;
  p.geometry = s.exchange.local_geometry;
  return p;
}

bool coph_can_export(const coph_learner_state &s, const coph_state &x) {
  if (s.stages.empty())
    return false;
  const coph_stage &st = s.stages.back();
  if (export_readings(st).empty())
    return false;
  const coph_vec2 offsets[] = {{0.0, 0.0}, {0.4, 0.0}, {0.0, 0.4}, {0.0, -0.4}};
  for (const coph_vec2 &o : offsets)
    if (supported(st, {x[0] + o[0], x[1] + o[1]}, s.cfg))
      return true;
  return false;
}

std::optional<coph_packet> coph_export(const coph_learner_state &s, const coph_state &x,
                                       const std::string &recipient, std::mt19937 &rng) {
  const int blocks = coph_bootstrap_blocks(s);
  if (blocks == 0)
    return std::nullopt;
  const int members = s.communication.ensemble_size;
  std::array<coph_draws, 2> draws;
  draws[0] = coph_draw_blocks(rng, blocks, members);
  draws[1] = coph_draw_blocks(rng, blocks, members);
  return coph_export(s, x, recipient, draws);
}

// ---------------------------------------------------------------------------
// G10: assimilate
// ---------------------------------------------------------------------------

void coph_expire(coph_learner_state &s) {
  auto &received = s.exchange.received;
  received.erase(std::remove_if(received.begin(), received.end(),
                                [&](const auto &e) {
                                  const coph_packet &m = e.second;
                                  if (m.expires < s.now)
                                    return true;
                                  for (const coph_stage &st : s.stages)
                                    for (const coph_anchor &a : m.anchors)
                                      if (contains(st, {a.x, a.y}, s.cfg))
                                        return false;
                                  return true;
                                }),
                 received.end());
}

void coph_refresh_terrain(coph_learner_state &s) {
  auto &cells = s.exchange.remote_cells;
  cells.erase(std::remove_if(cells.begin(), cells.end(),
                             [&](const auto &c) { return !(c.expires >= s.now); }),
              cells.end());
  s.terrain_anchors.clear();
  for (const auto &c : cells)
    s.terrain_anchors.push_back(c.anchor);
  for (const coph_anchor &a : s.exchange.local_model_anchors)
    s.terrain_anchors.push_back(a);
}

void coph_refresh_geometry(coph_learner_state &s) {
  auto &remote = s.exchange.remote_geometry;
  remote.erase(std::remove_if(remote.begin(), remote.end(),
                              [&](const auto &r) { return !(r.expires >= s.now); }),
               remote.end());
  std::vector<coph_shape> geometry;
  auto add = [](std::vector<coph_shape> &v, const coph_shape &o) {
    if (std::find(v.begin(), v.end(), o) == v.end())
      v.push_back(o);
  };
  for (const coph_geometry_report &r : s.exchange.local_geometry)
    add(geometry, r.shape);
  for (const auto &r : remote)
    add(geometry, r.shape);
  std::sort(geometry.begin(), geometry.end(), coph_shape_less);
  s.reported_geometry = geometry;
  std::vector<coph_shape> relevant;
  for (const coph_shape &o : geometry)
    for (const coph_stage &st : s.stages)
      if (coph_intersects(o, st.center, s.cfg)) {
        relevant.push_back(o);
        break;
      }
  if (static_cast<int>(relevant.size()) > s.cfg.obstacle_cap)
    s.geometry_overflow = true;
  std::vector<coph_shape> store = s.geometry;
  for (const coph_shape &o : relevant)
    add(store, o);
  std::sort(store.begin(), store.end(), coph_shape_less);
  if (static_cast<int>(store.size()) > s.cfg.obstacle_cap)
    store.resize(static_cast<std::size_t>(s.cfg.obstacle_cap));
  s.geometry = store;
  for (coph_stage &st : s.stages) {
    st.obstacles.clear();
    for (const coph_shape &o : s.geometry)
      if (coph_intersects(o, st.center, s.cfg))
        st.obstacles.push_back(o);
  }
}

namespace {

// MazeLearner.assimilate's geometry check.
bool valid_geometry(const coph_packet &m) {
  bool valid = m.geometry.size() <= kGeometryReports;
  const std::string prefix = m.sender + ":geometry:";
  for (const coph_geometry_report &o : m.geometry) {
    const coph_shape &sh = o.shape;
    bool finite = std::isfinite(sh.x) && std::isfinite(sh.y) && std::isfinite(sh.r) &&
                  std::isfinite(o.sensor[0]) && std::isfinite(o.sensor[1]) &&
                  std::isfinite(o.sensor_radius) && std::isfinite(o.time);
    if (sh.slab)
      finite = finite && std::isfinite(sh.hx) && std::isfinite(sh.hy);
    valid = valid && finite && sh.r > 0 && o.sensor_radius > 0 && o.time <= m.timestamp + 1e-9 &&
            o.time >= 0 && starts_with(o.source, prefix);
    const double gap = sh.slab ? detail::coph_slab_distance(o.sensor, sh)
                               : distance({sh.x, sh.y}, o.sensor) - sh.r;
    valid = valid && gap <= o.sensor_radius + 1e-9;
  }
  return valid;
}

// CooperativeLearner.assimilate's checks; the reason.
std::string cooperative_reason(const coph_learner_state &s, const coph_packet &m) {
  const std::string own = s.agent_id + ":";
  if (m.schema != 1 || m.convention != coph_convention)
    return "incompatible_convention";
  if (m.sender == s.agent_id ||
      std::any_of(m.sources.begin(), m.sources.end(),
                  [&](const std::string &src) { return starts_with(src, own); }))
    return "returned_evidence";
  if (m.recipient != s.agent_id)
    return "wrong_recipient";
  const std::string theirs = m.sender + ":";
  if (m.sources.empty() ||
      std::any_of(m.sources.begin(), m.sources.end(),
                  [&](const std::string &src) { return !starts_with(src, theirs); }))
    return "invalid_provenance";
  if (!std::isfinite(m.timestamp) || !std::isfinite(m.expires))
    return "invalid_age";
  if (m.timestamp > s.now + 1e-9 || m.expires < s.now || m.expires < m.timestamp)
    return "invalid_age";
  double last_t = -1.0;
  std::int64_t last_stage = -1;
  for (const auto &q : s.exchange.last_sequences)
    if (q.sender == m.sender) {
      last_t = q.timestamp;
      last_stage = q.stage;
    }
  if (m.timestamp < last_t || (m.timestamp == last_t && m.stage <= last_stage))
    return "duplicate_or_reordered";

  // The payload.
  const std::size_t n = m.anchors.size();
  bool finite = std::isfinite(m.H) && std::isfinite(m.V_task);
  for (double v : {m.q[0], m.q[1], m.p[0], m.p[1], m.v[0], m.v[1]})
    finite = finite && std::isfinite(v);
  for (double v : m.bounds)
    finite = finite && std::isfinite(v);
  for (const coph_anchor &a : m.anchors)
    for (double v : {a.x, a.y, a.value, a.gx, a.gy})
      finite = finite && std::isfinite(v);
  for (const auto &row : m.covariance)
    for (double v : row)
      finite = finite && std::isfinite(v);
  for (const coph_vec2 &sp : m.support_points)
    finite = finite && std::isfinite(sp[0]) && std::isfinite(sp[1]);
  bool valid =
      n > 0 && static_cast<int>(n) <= s.communication.anchors && m.covariance.size() == 3 * n;
  for (const auto &row : m.covariance)
    valid = valid && row.size() == 3 * n;
  if (valid) {
    const int d = static_cast<int>(3 * n);
    for (int i = 0; i < d && valid; ++i)
      for (int j = 0; j < d && valid; ++j)
        valid = close(m.covariance[i][j], m.covariance[j][i]);
    if (valid) { // numpy.linalg.eigvalsh(cov).min() > 0, on the lower triangle
      std::vector<double> a(static_cast<std::size_t>(d) * d);
      for (int i = 0; i < d; ++i)
        for (int j = 0; j < d; ++j)
          a[i * d + j] = i >= j ? m.covariance[i][j] : m.covariance[j][i];
      try {
        coph_cholesky(a, d);
      } catch (const std::runtime_error &) {
        valid = false;
      }
    }
  }
  bool negative = false;
  for (const coph_anchor &a : m.anchors)
    negative = negative || a.value < 0;
  if (!finite || !valid || !close(m.p[0], m.v[0]) || !close(m.p[1], m.v[1]) || negative ||
      static_cast<int>(m.support_points.size()) > s.cfg.sample_cap)
    return "invalid_payload";
  for (const coph_anchor &a : m.anchors)
    if (!(m.bounds[0] <= a.x && a.x <= m.bounds[1] && m.bounds[2] <= a.y && a.y <= m.bounds[3]))
      return "invalid_support";
  for (const coph_stage &st : s.stages)
    for (const coph_anchor &a : m.anchors)
      if (contains(st, {a.x, a.y}, s.cfg))
        return "eligible";
  return "unsupported";
}

void set_sequence(coph_exchange_state &e, const coph_packet &m) {
  auto it = find_key(e.last_sequences, m.sender);
  if (it == e.last_sequences.end())
    e.last_sequences.push_back({m.sender, m.timestamp, m.stage});
  else {
    it->timestamp = m.timestamp;
    it->stage = m.stage;
  }
}

} // namespace

coph_packet_event coph_assimilate(coph_learner_state &s, const coph_packet &m) {
  coph_packet_event ev;
  ev.packet = m.identity;
  ev.sender = m.sender;
  ev.recipient = s.agent_id;
  ev.time = s.now;
  ev.version_before = s.version;

  // A car's learner (no network): CooperativeLearner.assimilate alone.
  const bool car = s.network.nodes.empty();
  // MazeLearner.assimilate: the geometry first.
  if (!car && m.geometry_schema != 1) {
    ev.reason = "incompatible_geometry";
    return ev;
  }
  if (!car && !valid_geometry(m)) {
    ev.reason = "invalid_geometry";
    return ev;
  }

  // TerrainLearner.assimilate wraps CooperativeLearner's, undoing it when the route support
  // fails.
  coph_exchange_state &e = s.exchange;
  const auto received_before = e.received;
  const auto sequences_before = e.last_sequences;
  ev.reason = cooperative_reason(s, m);
  if (ev.reason == "eligible") {
    auto it = std::find_if(e.received.begin(), e.received.end(),
                           [&](const auto &r) { return r.first == m.sender; });
    if (it == e.received.end())
      e.received.push_back({m.sender, m});
    else
      it->second = m;
    set_sequence(e, m);
    while (static_cast<int>(e.received.size()) > s.communication.received_cap) {
      auto oldest = e.received.begin();
      for (auto r = e.received.begin(); r != e.received.end(); ++r)
        if (r->second.timestamp < oldest->second.timestamp)
          oldest = r;
      e.received.erase(oldest);
    }
  }
  if (car)
    return ev;

  if (ev.reason == "eligible" || ev.reason == "unsupported") {
    bool valid = !m.support_points.empty();
    for (const coph_anchor &a : m.anchors) {
      double best = std::numeric_limits<double>::infinity();
      for (const coph_vec2 &sp : m.support_points)
        best = std::min(best, distance(sp, {a.x, a.y}));
      valid = valid && best <= kSupportRadius;
    }
    auto previous =
        std::find_if(e.remote_terrain.begin(), e.remote_terrain.end(),
                     [&](const auto &r) { return r.sender == m.sender && r.stage == m.stage; });
    const bool stale =
        previous != e.remote_terrain.end() && previous->packet.timestamp >= m.timestamp;
    if (!valid || stale) {
      e.received = received_before;
      e.last_sequences = sequences_before;
      ev.reason = !valid ? "invalid_remote_support" : "duplicate_or_reordered";
    } else {
      ev.scope = ev.reason == "eligible" ? "retained_and_remote_route" : "remote_route_only";
      if (previous == e.remote_terrain.end())
        e.remote_terrain.push_back({m.sender, m.stage, m});
      else
        previous->packet = m;
      for (const coph_anchor &a : m.anchors) {
        const long long ix = static_cast<long long>(std::nearbyint(a.x / kCell));
        const long long iy = static_cast<long long>(std::nearbyint(a.y / kCell));
        auto c = std::find_if(e.remote_cells.begin(), e.remote_cells.end(), [&](const auto &x) {
          return x.sender == m.sender && x.ix == ix && x.iy == iy;
        });
        coph_exchange_state::cell cell{m.sender, ix, iy, m.expires, a, m.identity, m.sources};
        if (c == e.remote_cells.end())
          e.remote_cells.push_back(std::move(cell));
        else
          *c = std::move(cell);
      }
      while (e.remote_cells.size() > kRemoteCells) {
        auto oldest = e.remote_cells.begin();
        for (auto c = e.remote_cells.begin(); c != e.remote_cells.end(); ++c)
          if (c->expires < oldest->expires)
            oldest = c;
        e.remote_cells.erase(oldest);
      }
      while (e.remote_terrain.size() > kRemoteTerrain) {
        auto oldest = e.remote_terrain.begin();
        for (auto r = e.remote_terrain.begin(); r != e.remote_terrain.end(); ++r)
          if (r->packet.timestamp < oldest->packet.timestamp)
            oldest = r;
        e.remote_terrain.erase(oldest);
      }
      set_sequence(e, m);
      ev.reason = "eligible";
      coph_refresh_terrain(s);
    }
  }

  // MazeLearner: an eligible packet's geometry joins the remote reports.
  if (ev.reason == "eligible") {
    for (const coph_geometry_report &o : m.geometry) {
      coph_exchange_state::remote_report r{o.shape,  m.expires, m.identity,
                                           o.source, o.time,    m.sender};
      auto it = std::find_if(e.remote_geometry.begin(), e.remote_geometry.end(),
                             [&](const auto &x) { return x.shape == o.shape; });
      if (it == e.remote_geometry.end())
        e.remote_geometry.push_back(std::move(r));
      else
        *it = std::move(r);
    }
    if (e.remote_geometry.size() > kGeometryReports) {
      s.geometry_overflow = true;
      e.remote_geometry.erase(e.remote_geometry.begin(),
                              e.remote_geometry.end() -
                                  static_cast<std::ptrdiff_t>(kGeometryReports));
    }
    coph_refresh_geometry(s);
  }
  return ev;
}

// ---------------------------------------------------------------------------
// The receiver's Gauss-Newton step
// ---------------------------------------------------------------------------

std::vector<coph_message_fit> coph_fit_messages(const coph_learner_state &s,
                                                const std::vector<coph_stage_fit> &fit) {
  const coph_config &cfg = s.cfg;
  const double width2 = cfg.rbf_width * cfg.rbf_width;
  std::vector<coph_message_fit> out;
  for (std::size_t k = 0; k < s.stages.size(); ++k) {
    const coph_stage &st = s.stages[k];
    const std::vector<double> &w = fit[k].weights;
    const int m = static_cast<int>(w.size());
    coph_field_view f;
    f.centers = st.centers;
    f.weights = w;
    f.width = cfg.rbf_width;
    coph_message_fit mf;
    std::vector<std::vector<double>> rows; // each of length m
    std::vector<double> targets;
    for (const auto &[sender, packet] : s.exchange.received) {
      std::vector<int> ids;
      for (std::size_t i = 0; i < packet.anchors.size(); ++i)
        if (contains(st, {packet.anchors[i].x, packet.anchors[i].y}, cfg))
          ids.push_back(static_cast<int>(i));
      if (ids.empty())
        continue;
      mf.messages.push_back({sender, ids});
      const int d = static_cast<int>(3 * ids.size());
      // The residual z - f(w) and its Jacobian, three rows per anchor.
      std::vector<double> residual;
      std::vector<std::vector<double>> jac;
      for (int i : ids) {
        const coph_anchor &a = packet.anchors[i];
        const coph_vec2 q{a.x, a.y};
        const std::vector<double> phi = coph_features(q, st.centers, cfg.rbf_width);
        double raw = 0.0, sx = 0.0, sy = 0.0;
        std::vector<double> dx(m), dy(m);
        for (int j = 0; j < m; ++j) {
          raw += phi[j] * w[j];
          dx[j] = -(q[0] - st.centers[j][0]) / width2 * phi[j];
          dy[j] = -(q[1] - st.centers[j][1]) / width2 * phi[j];
          sx += dx[j] * w[j];
          sy += dy[j] * w[j];
        }
        const double sig = 1.0 / (1.0 + algo::det::exp(-raw));
        const double value = coph_risk<double>(q, f);
        residual.push_back(a.value - value);
        residual.push_back(a.gx - sig / 4 * sx);
        residual.push_back(a.gy - sig / 4 * sy);
        std::vector<double> jv(m), jx(m), jy(m);
        const double slope = sig * (1 - sig) / 4;
        for (int j = 0; j < m; ++j) {
          jv[j] = sig / 4 * phi[j];
          jx[j] = slope * phi[j] * sx + sig / 4 * dx[j];
          jy[j] = slope * phi[j] * sy + sig / 4 * dy[j];
        }
        jac.push_back(std::move(jv));
        jac.push_back(std::move(jx));
        jac.push_back(std::move(jy));
      }
      std::vector<int> idx;
      for (int i : ids)
        for (int j = 0; j < 3; ++j)
          idx.push_back(3 * i + j);
      const double age = 0.01 + 0.005 * (s.now - packet.timestamp);
      std::vector<double> c(static_cast<std::size_t>(d) * d);
      for (int r = 0; r < d; ++r)
        for (int q = 0; q < d; ++q)
          c[r * d + q] = packet.covariance[idx[r]][idx[q]] + (r == q ? age : 0.0);
      coph_cholesky(c, d, "coph_fit_messages: a packet's covariance is not positive definite");
      // L^-1 J, column by column, and L^-1 residual.
      std::vector<std::vector<double>> whitened(d, std::vector<double>(m));
      for (int j = 0; j < m; ++j) {
        std::vector<double> col(d);
        for (int r = 0; r < d; ++r)
          col[r] = jac[r][j];
        col = coph_forward(c, d, std::move(col));
        for (int r = 0; r < d; ++r)
          whitened[r][j] = col[r];
      }
      const std::vector<double> y = coph_forward(c, d, residual);
      for (int r = 0; r < d; ++r) {
        rows.push_back(std::move(whitened[r]));
        targets.push_back(y[r]);
      }
    }
    mf.rows = static_cast<int>(targets.size());
    if (rows.empty()) {
      mf.weights = w;
      out.push_back(std::move(mf));
      continue;
    }
    mf.gram.assign(static_cast<std::size_t>(m) * m, 0.0);
    mf.rhs.assign(m, 0.0);
    for (std::size_t r = 0; r < rows.size(); ++r)
      for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j)
          mf.gram[i * m + j] += rows[r][i] * rows[r][j];
        mf.rhs[i] += rows[r][i] * targets[r];
      }
    std::vector<double> system = coph_spd_inverse(fit[k].covariance, m); // the local curvature
    for (std::size_t i = 0; i < system.size(); ++i)
      system[i] += mf.gram[i];
    coph_cholesky(system, m, "coph_fit_messages: the Gauss-Newton system is not positive definite");
    mf.delta = coph_cholesky_solve(system, m, mf.rhs);
    mf.weights = w;
    for (int i = 0; i < m; ++i)
      mf.weights[i] += std::min(std::max(mf.delta[i], -0.5), 0.5);
    out.push_back(std::move(mf));
  }
  return out;
}

} // namespace nav
} // namespace cvc
