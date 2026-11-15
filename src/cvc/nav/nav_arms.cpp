/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/config.h> // CVC_VERSION_STRING
#include <cvc/nav/nav_arms.h>
#include <cvc/nav/plant_ledger.h>
#include <cvc/nav/sim_world.h>
#include <limits>
#include <set>
#include <stdexcept>

namespace cvc {
namespace nav {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kUnmeasured = 1e20; // libcvc's clearance before one is measured (a sentinel)
constexpr const char *kWho = "cvc::nav::arms_run: ";

constexpr const char *kBooks[] = {"dU", "drive", "projection", "context", "dKE", "residual"};
constexpr const char *kStats[] = {"clear_min_m", "clear_mean_m", "pen_steps", "path_m",
                                  "elapsed_s",   "ms_step",      "reached"};
// The pose, goal and size columns, before tel.*, book.* and plant.*.
constexpr const char *kPose[] = {"x",       "y",    "heading", "speed", "goal_x",   "goal_y",
                                 "reached", "mode", "length",  "width", "wheelbase"};
constexpr std::size_t kPoseN = sizeof(kPose) / sizeof(kPose[0]);

#define CVC_TEL(field) [](const drive_telemetry &t) { return static_cast<double>(t.field); }
constexpr telemetry_field kTelemetry[] = {
    {"alpha", CVC_TEL(alpha)},
    {"beta", CVC_TEL(beta)},
    {"gamma", CVC_TEL(gamma)},
    {"lam_soft", CVC_TEL(lam_soft)},
    {"lam_hard", CVC_TEL(lam_hard)},
    {"mu", CVC_TEL(mu)},
    {"mrisk", CVC_TEL(mrisk)},
    {"v_risk", CVC_TEL(v_risk)},
    {"ext_fx", CVC_TEL(ext_fx)},
    {"ext_fy", CVC_TEL(ext_fy)},
    {"steer", CVC_TEL(steer)},
    {"curvature", CVC_TEL(curvature)},
    {"clearance", CVC_TEL(clearance)},
    {"binding", CVC_TEL(binding)},
    {"potential", CVC_TEL(potential)},
    {"u_goal", CVC_TEL(terms.goal)},
    {"u_barrier", CVC_TEL(terms.barrier)},
    {"u_soft", CVC_TEL(terms.soft)},
    {"u_hard", CVC_TEL(terms.hard)},
    {"u_terrain", CVC_TEL(terms.terrain)},
    {"force_x", CVC_TEL(force_x)},
    {"force_y", CVC_TEL(force_y)},
    {"potential_end", CVC_TEL(potential_end)},
    {"projection_work", CVC_TEL(projection_work)},
    {"context_work", CVC_TEL(context_work)},
};
#undef CVC_TEL

std::vector<std::string> agent_columns() {
  std::vector<std::string> c(kPose, kPose + kPoseN);
  for (const telemetry_field &f : kTelemetry)
    c.push_back(std::string("tel.") + f.name);
  for (const char *b : kBooks)
    c.push_back(std::string("book.") + b);
  for (const plant_ledger_item &it : plant_ledger_vehicle())
    c.push_back(std::string("plant.") + it.key);
  return c;
}

std::vector<std::string> arm_columns() {
  std::vector<std::string> c = {"ms"};
  for (const char *s : kStats)
    c.push_back(std::string("stats.") + s);
  return c;
}

bool measured(double c) { return std::isfinite(c) && std::abs(c) < kUnmeasured; }

// A float as the shortest decimal that reads back as it, as a double: a tick of 0.05f is 0.05 s,
// not 0.0500000007 s, so a run's times are the decimals its settings were written in.
double decimal(float f) {
  char b[32];
  for (int prec = 6; prec <= 9; ++prec) {
    std::snprintf(b, sizeof b, "%.*g", prec, static_cast<double>(f));
    if (std::strtof(b, nullptr) == f)
      break;
  }
  return std::strtod(b, nullptr);
}

} // namespace

// ── the books ───────────────────────────────────────────────────────────────────────────────────

std::span<const char *const> book_names() { return kBooks; }

double book_value(const nav_books &b, int i) {
  switch (i) {
  case 0:
    return b.dU;
  case 1:
    return b.drive;
  case 2:
    return b.projection;
  case 3:
    return b.context;
  case 4:
    return b.dKE;
  case 5:
    return b.residual;
  default:
    return kNaN;
  }
}

nav_books books_of(const drive_telemetry &t, const double *prev_end, double speed_n,
                   double prev_speed_n) {
  const double u0 = t.potential, u1 = t.potential_end;
  nav_books b;
  b.projection = t.projection_work;
  b.context = t.context_work;
  b.drive = (u1 - b.projection) - u0;
  b.dU = u1 - (prev_end ? *prev_end : u0);
  b.dKE = 0.5 * (speed_n * speed_n - prev_speed_n * prev_speed_n);
  b.residual = b.dU - (b.drive + b.projection + b.context);
  return b;
}

std::span<const telemetry_field> telemetry_fields() { return kTelemetry; }

// ── the Stats row ───────────────────────────────────────────────────────────────────────────────

std::span<const char *const> stats_columns() { return kStats; }

double arm_stats::value(int i) const {
  switch (i) {
  case 0:
    return clear_min_m;
  case 1:
    return clear_mean_m;
  case 2:
    return pen_steps;
  case 3:
    return path_m;
  case 4:
    return elapsed_s;
  case 5:
    return ms_step;
  case 6:
    return reached;
  default:
    return kNaN;
  }
}

arm_stats stats_of(const episode_nav_stats &e, double clear_min_m, double clear_sum_m, long clear_n,
                   double ms_sum, long ms_n) {
  arm_stats s;
  double cmin = std::numeric_limits<double>::infinity();
  bool any = false, all = !e.per_vehicle.empty();
  double ttg = 0.0, path = 0.0;
  for (const veh_nav_stats &v : e.per_vehicle) {
    if (measured(v.min_clearance_m)) {
      cmin = std::min(cmin, v.min_clearance_m);
      any = true;
    }
    s.pen_steps += v.penetration_steps;
    path += v.total_path_m;
    if (v.arrived) {
      s.reached += 1;
      ttg = std::max(ttg, v.time_to_goal_s);
    } else {
      all = false;
    }
  }
  s.clear_min_m = any ? cmin : (clear_n > 0 ? clear_min_m : kNaN);
  s.clear_mean_m = clear_n > 0 ? clear_sum_m / static_cast<double>(clear_n) : kNaN;
  s.path_m = e.per_vehicle.empty() ? 0.0 : path / static_cast<double>(e.per_vehicle.size());
  s.elapsed_s = all && ttg >= 0.0 ? ttg : static_cast<double>(e.ticks) * e.dt_s;
  s.ms_step = ms_n > 0 ? ms_sum / static_cast<double>(ms_n) : kNaN;
  return s;
}

// ── arms ────────────────────────────────────────────────────────────────────────────────────────

arms_run::arms_run(std::vector<sim_world *> worlds, std::vector<arm> arms, arms_options o)
    : worlds_(std::move(worlds)) {
  init(std::move(arms), std::move(o));
}

arms_run::arms_run(std::vector<std::unique_ptr<sim_world>> worlds, std::vector<arm> arms,
                   arms_options o)
    : owned_(std::move(worlds)) {
  for (const std::unique_ptr<sim_world> &w : owned_)
    worlds_.push_back(w.get());
  init(std::move(arms), std::move(o));
}

arms_run::~arms_run() = default;

int arms_run::agents() const { return worlds_.front()->size(); }

double arms_run::dt() const { return decimal(worlds_.front()->current_config().veh.dt); }

void arms_run::init(std::vector<arm> arms, arms_options o) {
  if (arms.empty())
    throw std::invalid_argument(std::string(kWho) + "no arm");
  if (worlds_.size() != arms.size())
    throw std::invalid_argument(std::string(kWho) + "one world per arm");
  std::set<std::string> names;
  std::set<const sim_world *> seen;
  for (std::size_t a = 0; a < arms.size(); ++a) {
    if (!worlds_[a])
      throw std::invalid_argument(std::string(kWho) + "arm '" + arms[a].name + "' has no world");
    if (arms[a].name.empty() || !names.insert(arms[a].name).second)
      throw std::invalid_argument(std::string(kWho) + "each arm needs a name of its own ('" +
                                  arms[a].name + "')");
    if (!seen.insert(worlds_[a]).second)
      throw std::invalid_argument(std::string(kWho) + "two arms share a world");
  }
  const sim_world::config &c0 = worlds_[0]->current_config();
  for (const sim_world *w : worlds_) {
    const sim_world::config &c = w->current_config();
    if (w->size() != worlds_[0]->size() || c.veh.dt != c0.veh.dt || c.scale != c0.scale ||
        c.cx != c0.cx || c.cy != c0.cy)
      throw std::invalid_argument(std::string(kWho) +
                                  "the arms' worlds differ in agents, tick, frame or centre");
  }
  opt_ = std::move(o);
  const int n = agents();
  run_log_header h;
  h.libcvc = CVC_VERSION_STRING;
  h.dt = decimal(c0.veh.dt);
  h.scale = c0.scale;
  h.cx = c0.cx;
  h.cy = c0.cy;
  h.seed = opt_.seed;
  h.scene = opt_.scene;
  h.plant = !opt_.plant.empty() ? opt_.plant : (worlds_[0]->plant() ? "plant" : "kinematic");
  h.agents = n;
  h.world = opt_.world;
  h.settings = opt_.settings;
  h.meta = opt_.meta;
  h.agent_columns = agent_columns();
  h.arm_columns = arm_columns();
  std::vector<float> pos(static_cast<std::size_t>(2) * n), head(n), goals(2 * n);
  worlds_[0]->snapshot(pos.data(), head.data(), nullptr, nullptr, nullptr);
  worlds_[0]->goals_world(goals.data());
  for (int i = 0; i < n; ++i) {
    h.start.push_back(pos[2 * i]);
    h.start.push_back(pos[2 * i + 1]);
    h.start.push_back(head[i]);
  }
  st_.resize(arms.size());
  for (std::size_t a = 0; a < arms.size(); ++a) {
    sim_world &w = *worlds_[a];
    run_log_arm la;
    la.name = arms[a].name;
    la.slot = static_cast<int>(a);
    la.key = n > 0 ? w.policy_key(w.policy_of(0)) : std::string();
    la.uri = arms[a].uri;
    la.sha256 = arms[a].sha256;
    la.meta = arms[a].meta;
    h.arms.push_back(std::move(la));
    w.set_capture_drive_telemetry(true);
    arm_state &s = st_[a];
    s.prev_end.assign(n, 0.0);
    s.prev_speed.assign(w.speeds_normalized(), w.speeds_normalized() + n);
  }
  log_ = std::make_shared<run_log>(std::move(h));
  std::vector<double> g(goals.begin(), goals.end());
  log_->begin_leg(0, std::move(g));
  begin_stats();
  if (!opt_.log_path.empty()) {
    writer_ = std::make_unique<run_log_writer>(opt_.log_path);
    stream(log_->header().to_json());
    stream(log_->leg_json(0));
  }
}

void arms_run::begin_stats() {
  for (std::size_t a = 0; a < worlds_.size(); ++a)
    worlds_[a]->begin_nav_stats(opt_.stats, {}, opt_.scene, opt_.seed, log_->header().arms[a].name);
  for (arm_state &s : st_) {
    s.clear_min = std::numeric_limits<double>::infinity();
    s.clear_sum = s.ms_sum = 0.0;
    s.clear_n = s.ms_n = 0;
  }
}

void arms_run::stream(const std::string &line) {
  if (writer_)
    writer_->line(line);
}

void arms_run::step(int num_threads) {
  if (finished_)
    throw std::logic_error(std::string(kWho) + "the run is finished");
  const int na = arms();
  for (int j = 0; j < na; ++j) {
    const int a = static_cast<int>((j + tick_) % na);
    const auto t0 = std::chrono::steady_clock::now();
    worlds_[static_cast<std::size_t>(a)]->step(num_threads);
    const auto t1 = std::chrono::steady_clock::now();
    record(a, std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  end_tick();
}

void arms_run::record(int a, double ms) {
  if (finished_)
    throw std::logic_error(std::string(kWho) + "the run is finished");
  if (a < 0 || a >= arms())
    throw std::out_of_range(std::string(kWho) + "no arm " + std::to_string(a));
  arm_state &s = st_[static_cast<std::size_t>(a)];
  if (s.recorded)
    throw std::logic_error(std::string(kWho) + "arm " + std::to_string(a) +
                           " recorded twice in a tick");
  sim_world &w = *worlds_[static_cast<std::size_t>(a)];
  const int n = agents();
  const double scale = w.current_config().scale;
  std::vector<float> pos(static_cast<std::size_t>(2) * n), head(n), spd(n), goals(2 * n);
  std::vector<int> mode(n);
  std::vector<std::uint8_t> reached(n);
  w.snapshot(pos.data(), head.data(), spd.data(), mode.data(), reached.data());
  w.goals_world(goals.data());
  // sizes: the host's, else the world's vehicle dims, else its footprint disc
  std::vector<float> wid(n), len(n), L(n);
  w.vehicle_dims_m(wid.data(), len.data());
  w.vehicle_kinematics(nullptr, nullptr, L.data());
  const double rr_m = w.current_config().veh.rr / scale;
  const drive_telemetry *tel = w.drive_telemetry_data();
  const float *sp_n = w.speeds_normalized();
  const episode_nav_stats e = w.nav_stats();
  const std::span<const plant_ledger_item> items = plant_ledger_vehicle();
  const std::size_t sa = log_->header().agent_columns.size();
  s.agent_values.assign(static_cast<std::size_t>(n) * sa, kNaN);
  for (int i = 0; i < n; ++i) {
    double *v = s.agent_values.data() + static_cast<std::size_t>(i) * sa;
    const double wb = s.wheelbase.empty() ? L[i] / scale : s.wheelbase[i];
    const double width = !s.width.empty() ? s.width[i] : (wid[i] > 0 ? wid[i] : 2.0 * rr_m);
    const double length = !s.length.empty() ? s.length[i] : (len[i] > 0 ? len[i] : wb + width);
    const double pose[kPoseN] = {pos[2 * i],
                                 pos[2 * i + 1],
                                 head[i],
                                 spd[i],
                                 goals[2 * i],
                                 goals[2 * i + 1],
                                 reached[i] ? 1.0 : 0.0,
                                 static_cast<double>(mode[i]),
                                 length,
                                 width,
                                 wb};
    std::size_t c = 0;
    for (; c < kPoseN; ++c)
      v[c] = pose[c];
    const drive_telemetry t = tel ? tel[i] : drive_telemetry{};
    for (const telemetry_field &f : kTelemetry)
      v[c++] = f.get(t);
    const nav_books b =
        books_of(t, s.has_prev ? &s.prev_end[i] : nullptr, sp_n[i], s.prev_speed[i]);
    for (int k = 0; k < static_cast<int>(std::size(kBooks)); ++k)
      v[c++] = book_value(b, k);
    s.prev_end[i] = t.potential_end;
    s.prev_speed[i] = sp_n[i];
    const veh_nav_stats *vs =
        i < static_cast<int>(e.per_vehicle.size()) ? &e.per_vehicle[i] : nullptr;
    for (const plant_ledger_item &it : items)
      v[c++] = vs && plant_ledger_applies(it, *vs) ? it.veh(*vs) : kNaN;
    const double clear_m = t.clearance / scale;
    if (tel && measured(t.clearance)) {
      s.clear_min = std::min(s.clear_min, clear_m);
      s.clear_sum += clear_m;
      ++s.clear_n;
    }
  }
  s.has_prev = true;
  s.ms_sum += ms;
  ++s.ms_n;
  const arm_stats row = stats_of(e, s.clear_min, s.clear_sum, s.clear_n, s.ms_sum, s.ms_n);
  s.arm_values.assign(1 + std::size(kStats), kNaN);
  s.arm_values[0] = ms;
  for (int k = 0; k < static_cast<int>(std::size(kStats)); ++k)
    s.arm_values[1 + k] = row.value(k);
  s.recorded = true;
}

void arms_run::end_tick() {
  std::vector<double> agent_values, arm_values;
  for (std::size_t a = 0; a < st_.size(); ++a) {
    if (!st_[a].recorded)
      throw std::logic_error(std::string(kWho) + "arm " + std::to_string(a) +
                             " was not recorded this tick");
    agent_values.insert(agent_values.end(), st_[a].agent_values.begin(), st_[a].agent_values.end());
    arm_values.insert(arm_values.end(), st_[a].arm_values.begin(), st_[a].arm_values.end());
  }
  ++tick_;
  log_->add_tick(static_cast<double>(tick_) * dt(), agent_values, arm_values);
  for (arm_state &s : st_)
    s.recorded = false;
  stream(log_->tick_json(log_->ticks() - 1));
}

void arms_run::retarget(const float *goals_world, int n) {
  if (finished_)
    throw std::logic_error(std::string(kWho) + "the run is finished");
  if (!goals_world || n != agents())
    throw std::invalid_argument(std::string(kWho) + "retarget: a goal per agent");
  for (const arm_state &s : st_)
    if (s.recorded)
      throw std::logic_error(std::string(kWho) + "retarget between ticks, not inside one");
  std::vector<std::string> ends;
  for (sim_world *w : worlds_)
    ends.push_back(w->nav_stats().to_json());
  log_->end_leg(std::move(ends));
  stream(log_->leg_end_json(log_->legs().size() - 1));
  for (sim_world *w : worlds_) {
    const sim_world::config &c = w->current_config();
    for (int i = 0; i < n; ++i)
      w->retarget(i, static_cast<float>((goals_world[2 * i] - c.cx) * c.scale),
                  static_cast<float>((goals_world[2 * i + 1] - c.cy) * c.scale));
  }
  begin_stats();
  std::vector<float> g(static_cast<std::size_t>(2) * n);
  worlds_[0]->goals_world(g.data());
  log_->begin_leg(tick_, std::vector<double>(g.begin(), g.end()));
  stream(log_->leg_json(log_->legs().size() - 1));
}

void arms_run::retarget(int i, double gx, double gy) {
  const int n = agents();
  if (i < 0 || i >= n)
    throw std::out_of_range(std::string(kWho) + "no agent " + std::to_string(i));
  std::vector<float> g(static_cast<std::size_t>(2) * n);
  worlds_[0]->goals_world(g.data());
  g[2 * i] = static_cast<float>(gx);
  g[2 * i + 1] = static_cast<float>(gy);
  retarget(g.data(), n);
}

void arms_run::set_size(int a, const float *length, const float *width, const float *wheelbase) {
  if (a < 0 || a >= arms())
    throw std::out_of_range(std::string(kWho) + "no arm " + std::to_string(a));
  arm_state &s = st_[static_cast<std::size_t>(a)];
  const int n = agents();
  auto take = [n](const float *src, std::vector<float> &dst) {
    if (src)
      dst.assign(src, src + n);
    else
      dst.clear();
  };
  take(length, s.length);
  take(width, s.width);
  take(wheelbase, s.wheelbase);
}

void arms_run::finish() {
  if (finished_)
    return;
  std::vector<std::string> ends;
  for (sim_world *w : worlds_)
    ends.push_back(w->nav_stats().to_json());
  log_->finish(std::move(ends));
  stream(log_->end_json());
  finished_ = true;
}

} // namespace nav
} // namespace cvc
