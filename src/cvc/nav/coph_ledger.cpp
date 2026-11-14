/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// coph_ledger.h: the stage ledger of grl_snam.coph's MazeLearner. Each expression keeps the
// Python's operation order (records.py, navigation.py, maze_learning.py, learner.py), and the TU
// is built with -ffp-contract=off, so the ledger matches the golden bit for bit.

#include <algorithm>
#include <cmath>
#include <cvc/nav/coph_ledger.h>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace cvc {
namespace nav {

namespace {

// numpy.linspace(start, stop, num): i * step + start with step = (stop - start) / (num - 1), and
// the last exactly stop.
std::vector<double> linspace(double start, double stop, int num) {
  std::vector<double> y(static_cast<std::size_t>(num));
  const double step = (stop - start) / static_cast<double>(num - 1);
  for (int i = 0; i < num; ++i)
    y[static_cast<std::size_t>(i)] = static_cast<double>(i) * step + start;
  y.back() = stop;
  return y;
}

// The last `cap` items.
template <class T> void keep_last(std::vector<T> &v, int cap) {
  const std::size_t n = static_cast<std::size_t>(std::max(cap, 0));
  if (v.size() > n)
    v.erase(v.begin(), v.end() - static_cast<std::ptrdiff_t>(n));
}

} // namespace

double coph_shape::reach() const { return slab ? r + std::hypot(hx, hy) : r; }

bool coph_stage::contains(const coph_vec2 &q, const coph_config &cfg) const {
  return std::abs(q[0] - center[0]) <= cfg.stage_width / 2 + 1e-10 &&
         std::abs(q[1] - center[1]) <= cfg.stage_height / 2 + 1e-10;
}

coph_stage make_coph_stage(std::int64_t id, const coph_vec2 &center, const coph_state &entry,
                           const coph_vec2 &goal, const coph_config &cfg) {
  coph_stage s;
  s.id = id;
  s.center = center;
  s.entry = entry;
  const std::vector<double> xs =
      linspace(center[0] - cfg.stage_width / 2, center[0] + cfg.stage_width / 2, cfg.rbf_side);
  const std::vector<double> ys =
      linspace(center[1] - cfg.stage_height / 2, center[1] + cfg.stage_height / 2, cfg.rbf_side);
  for (double x : xs)
    for (double y : ys)
      s.centers.push_back({x, y});
  // min(goal.x, center.x + stage_width / 2 - .1): Python's min keeps the first unless the second
  // is less.
  const double edge = center[0] + cfg.stage_width / 2 - 0.1;
  s.target = {edge < goal[0] ? edge : goal[0], goal[1]};
  s.weights.assign(s.centers.size(), -1.0 / static_cast<double>(s.centers.size()));
  return s;
}

bool coph_intersects(const coph_shape &shape, const coph_vec2 &center, const coph_config &cfg) {
  const double d0 = std::max(std::abs(shape.x - center[0]) - cfg.stage_width / 2, 0.0);
  const double d1 = std::max(std::abs(shape.y - center[1]) - cfg.stage_height / 2, 0.0);
  const double dd = d0 * d0 + d1 * d1; // float(d @ d)
  return dd <= std::pow(shape.reach() + 0.12 + cfg.safety_margin, 2.0) + 1e-12;
}

coph_ledger::coph_ledger(coph_config cfg, const coph_state &initial, const coph_vec2 &goal)
    : cfg_(std::move(cfg)), goal_(goal) {
  cfg_.validate();
  require_profile(cfg_);
  stages_.push_back(make_coph_stage(0, {initial[0], initial[1]}, initial, goal_, cfg_));
}

void coph_ledger::observe(const coph_observation &observation, const coph_transition *transition) {
  // navigation.observe_geometry's footprints.
  if (observation.sensor_center) {
    const coph_vec2 &c = *observation.sensor_center;
    const coph_footprint f{c[0], c[1], observation.sensor_radius, observation.time};
    const coph_shape sensor = coph_shape::disk(c[0], c[1], observation.sensor_radius);
    for (coph_stage &s : stages_)
      if (coph_intersects(sensor, s.center, cfg_)) {
        s.footprints.push_back(f);
        keep_last(s.footprints, cfg_.footprint_cap);
      }
  }
  // Learner.observe's readings: the uids held before this observation.
  for (coph_stage &s : stages_) {
    std::unordered_set<std::string> known;
    for (const coph_reading &r : s.readings)
      known.insert(r.uid);
    for (const coph_reading &r : observation.readings)
      if (!known.count(r.uid) && s.contains(r.q, cfg_))
        s.readings.push_back(r);
    keep_last(s.readings, cfg_.sample_cap);
  }
  if (transition) {
    coph_stage &s = stages_.back();
    s.transitions.push_back(*transition);
    keep_last(s.transitions, cfg_.transition_cap);
  }
}

void coph_ledger::set_geometry(std::vector<coph_shape> store) {
  geometry_ = std::move(store);
  refresh_obstacles();
}

void coph_ledger::refresh_obstacles() {
  for (coph_stage &s : stages_) {
    s.obstacles.clear();
    for (const coph_shape &o : geometry_)
      if (coph_intersects(o, s.center, cfg_))
        s.obstacles.push_back(o);
  }
}

bool coph_ledger::advance(const coph_state &x, std::int64_t version) {
  const coph_stage &s = stages_.back();
  const coph_vec2 size{cfg_.stage_width, cfg_.stage_height};
  const coph_vec2 delta{std::abs(x[0] - s.center[0]), std::abs(x[1] - s.center[1])};
  // np.array([stage_width, stage_height]) * (1 - overlap) / 2
  const double keep = 1 - cfg_.overlap;
  if (!(delta[0] > size[0] * keep / 2 || delta[1] > size[1] * keep / 2))
    return false;
  // argmax(delta / size): the first of equal maxima, or the first NaN, as numpy's.
  const double r0 = delta[0] / size[0], r1 = delta[1] / size[1];
  const int axis = std::isnan(r0) ? 0 : std::isnan(r1) ? 1 : r1 > r0 ? 1 : 0;
  coph_vec2 center = s.center;
  const double away = x[static_cast<std::size_t>(axis)] - center[static_cast<std::size_t>(axis)];
  const double sign = std::isnan(away) ? away : away > 0 ? 1.0 : away < 0 ? -1.0 : 0.0; // np.sign
  center[static_cast<std::size_t>(axis)] += sign * size[static_cast<std::size_t>(axis)] * keep;

  coph_stage n = make_coph_stage(s.id + 1, center, x, goal_, cfg_);
  // Every kept stage's readings that n contains, oldest first. {r.uid: r for r in ...}.values():
  // a repeated uid keeps its first place and takes its last reading.
  std::unordered_map<std::string, std::size_t> place;
  for (const coph_stage &old : stages_)
    for (const coph_reading &r : old.readings) {
      if (!n.contains(r.q, cfg_))
        continue;
      const auto [it, fresh] = place.emplace(r.uid, n.readings.size());
      if (fresh)
        n.readings.push_back(r);
      else
        n.readings[it->second] = r;
    }
  keep_last(n.readings, cfg_.sample_cap);
  // Every kept stage's footprints, oldest first, each once (dict.fromkeys).
  for (const coph_stage &old : stages_)
    for (const coph_footprint &f : old.footprints)
      if (std::find(n.footprints.begin(), n.footprints.end(), f) == n.footprints.end())
        n.footprints.push_back(f);
  keep_last(n.footprints, cfg_.footprint_cap);
  n.version = version;

  stages_.push_back(std::move(n));
  const std::size_t kept = static_cast<std::size_t>(cfg_.previous) + 1;
  if (stages_.size() > kept)
    stages_.erase(stages_.begin(), stages_.end() - static_cast<std::ptrdiff_t>(kept));
  refresh_obstacles();
  return true;
}

} // namespace nav
} // namespace cvc
