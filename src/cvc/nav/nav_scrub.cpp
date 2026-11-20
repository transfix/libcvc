/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "../core/input_trace_json.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/core/config.h> // CVC_VERSION_STRING
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/nav_arms.h>
#include <cvc/nav/nav_scrub.h>
#include <cvc/nav/policy_library.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/sim_world_trace.h>
#include <cvc/state/state_blob_store.h> // sha256_hex
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <locale>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace nav {
namespace {

namespace fs = std::filesystem;
namespace tj = cvc::trace_json;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr const char *kWho = "cvc::nav::scrub: ";

constexpr scrub_metric_info kMetrics[] = {
    {"reached", "reached the goal (share)", 2},
    {"time_s", "time (s)", 1},
    {"path_m", "path (m)", 1},
    {"pen_steps", "ticks penetrating", 0},
    {"clear_min_m", "clearance min (m)", 2},
    {"clear_mean_m", "clearance mean (m)", 2},
    {"goal_m", "left to the goal (m)", 1},
};

std::string file_sha256(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error(std::string(kWho) + "cannot read " + path);
  std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(f)),
                                   std::istreambuf_iterator<char>());
  return sha256_hex(bytes);
}

std::string step_text(double step) {
  char b[32];
  std::snprintf(b, sizeof b, "%g", step);
  return b;
}

// Every numeric leaf of `v` under `path`, as <path>.<key>...
void leaves(const tj::value &v, const std::string &path, const std::vector<std::string> &skip,
            std::vector<std::pair<std::string, double>> &out) {
  if (v.is_number()) {
    out.emplace_back(path, v.number("a value"));
    return;
  }
  if (!v.is_object())
    return;
  for (std::size_t i = 0; i < v.keys.size(); ++i) {
    if (path.find('.') == std::string::npos &&
        std::find(skip.begin(), skip.end(), v.keys[i]) != skip.end())
      continue;
    leaves(v.a[i], path + "." + v.keys[i], skip, out);
  }
}

std::string str_member(const tj::value &o, const char *key) {
  const tj::value *v = o.get(key);
  return v && v->is_string() ? v->s : std::string();
}

bool true_member(const tj::value &o, const char *key) {
  const tj::value *v = o.get(key);
  return v && v->t == tj::value::type::boolean && v->b;
}

// The built-in scenario's map: a walled square with a 3 x 3 grid of buildings, each nudged and
// sized by the seed, and streets between them a reactive navigator can weave through.
std::vector<std::uint8_t> blocks(int n, std::mt19937 &rng) {
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(n) * n, 0);
  auto set = [&](int r, int c) {
    if (r >= 0 && r < n && c >= 0 && c < n)
      occ[static_cast<std::size_t>(r) * n + c] = 1;
  };
  for (int i = 0; i < n; ++i) {
    set(0, i);
    set(n - 1, i);
    set(i, 0);
    set(i, n - 1);
  }
  // the raw engine's draws, not std::uniform_int_distribution's: the same map on every library
  const auto pick = [&rng](int lo, int hi) {
    return lo + static_cast<int>(rng() % static_cast<unsigned>(hi - lo + 1));
  };
  const int cell = n / 4; // 3 blocks in 4 strides: the streets between and round them
  for (int bi = 1; bi <= 3; ++bi)
    for (int bj = 1; bj <= 3; ++bj) {
      const int cr = bi * cell + pick(-3, 3), cc = bj * cell + pick(-3, 3);
      const int hr = pick(cell / 4, cell / 3), hc = pick(cell / 4, cell / 3);
      for (int r = cr - hr; r <= cr + hr; ++r)
        for (int c = cc - hc; c <= cc + hc; ++c)
          set(r, c);
    }
  return occ;
}

// A scenario's plant, "" read as kinematic.
std::string plant_name(const std::string &plant) {
  return plant.empty() ? std::string(kScrubKinematic) : plant;
}

} // namespace

// ── training histories ──────────────────────────────────────────────────────────────────────────

const training_series *training_history::find(const std::string &name) const {
  for (const training_series &s : series)
    if (s.name == name)
      return &s;
  return nullptr;
}

std::string resolve_blob(const std::string &history, const std::string &blob) {
  if (blob.empty())
    return {};
  const fs::path p(blob);
  const fs::path here = fs::absolute(fs::path(history)).parent_path();
  std::vector<fs::path> tries;
  if (p.is_absolute())
    tries = {p};
  else
    tries = {here / p, fs::current_path() / p, here / p.filename()};
  for (const fs::path &c : tries) {
    std::error_code ec;
    if (fs::is_regular_file(c, ec))
      return fs::weakly_canonical(c, ec).string();
  }
  return {};
}

training_history read_training_history(const std::string &path, int run) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error(std::string(kWho) + "cannot read " + path);
  if (f.peek() == 0x1f)
    throw std::runtime_error(std::string(kWho) + path + " is gzipped: gunzip it first");
  // the runs: each from a header (a file without one is one run)
  std::vector<std::vector<tj::value>> runs(1);
  std::string line;
  long n = 0;
  while (std::getline(f, line)) {
    ++n;
    if (line.find_first_not_of(" \t\r") == std::string::npos)
      continue;
    tj::value v;
    try {
      v = tj::parse(line);
    } catch (const std::exception &e) {
      throw std::runtime_error(path + ": line " + std::to_string(n) + ": " + e.what());
    }
    if (!v.is_object())
      continue;
    if (str_member(v, "kind") == "header" && !runs.back().empty())
      runs.emplace_back();
    runs.back().push_back(std::move(v));
  }
  if (runs.back().empty() && runs.size() > 1)
    runs.pop_back();
  const int count = static_cast<int>(runs.size());
  const int r = run < 0 ? count + run : run;
  if (r < 0 || r >= count)
    throw std::runtime_error(std::string(kWho) + path + " holds " + std::to_string(count) +
                             " runs, not run " + std::to_string(run));
  training_history h;
  h.path = path;
  h.run = r;
  h.runs = count;
  std::map<std::string, std::size_t> index;
  double selected = kNaN;
  double last_x = 0.0;
  for (const tj::value &rec : runs[static_cast<std::size_t>(r)]) {
    const std::string kind = rec.get("kind") ? str_member(rec, "kind") : "record";
    if (kind == "header") {
      h.schema = str_member(rec, "schema");
      continue;
    }
    std::string axis = "step";
    double x = last_x;
    if (const tj::value *s = rec.get("step"); s && s->is_number()) {
      x = s->number("step");
    } else if (const tj::value *e = rec.get("epoch"); e && e->is_number()) {
      x = e->number("epoch");
      axis = "epoch";
    }
    last_x = x;
    if (kind == "end")
      if (const tj::value *s = rec.get("selected_step"); s && s->is_number())
        selected = s->number("selected_step");
    std::vector<std::pair<std::string, double>> pts;
    leaves(rec, kind, {"kind", "step", "epoch", "blob", "checkpoint"}, pts);
    for (const auto &[name, y] : pts) {
      auto it = index.find(name);
      if (it == index.end()) {
        it = index.emplace(name, h.series.size()).first;
        h.series.push_back({name, {}, {}});
      }
      h.series[it->second].x.push_back(x);
      h.series[it->second].y.push_back(y);
    }
    if (kind != "checkpoint")
      continue;
    std::string blob = str_member(rec, "blob");
    if (blob.empty())
      blob = str_member(rec, "checkpoint");
    if (blob.empty())
      continue;
    const std::string found = resolve_blob(path, blob);
    if (found.empty()) {
      h.missing.push_back(blob);
      continue;
    }
    if (fs::path(found).extension() != ".cvcnav") {
      h.skipped.push_back(blob);
      continue;
    }
    training_checkpoint c;
    c.step = x;
    c.axis = axis;
    c.blob = blob;
    c.path = found;
    c.best = true_member(rec, "best");
    c.selected = true_member(rec, "selected");
    const tj::value *fit = rec.get("fitness");
    c.fitness = fit && fit->is_number() ? fit->number("fitness") : kNaN;
    h.checkpoints.push_back(std::move(c));
  }
  if (std::isfinite(selected))
    for (training_checkpoint &c : h.checkpoints)
      c.selected = c.selected || c.step == selected;
  return h;
}

// ── the scenario ────────────────────────────────────────────────────────────────────────────────

scrub_scenario default_scenario(int agents, unsigned seed, long ticks) {
  if (agents < 1)
    throw std::invalid_argument(std::string(kWho) + "a scenario needs an agent");
  constexpr int n = 121;                  // 1 m cells over 120 m
  constexpr double half = 60.0, S = 0.05; // the nav demos' frame: 1 unit = 20 m
  std::mt19937 rng(seed);
  const std::vector<std::uint8_t> occ = blocks(n, rng);
  sim_world::config c;
  c.rows = c.cols = n;
  c.min_x = c.min_y = -half;
  c.max_x = c.max_y = half;
  c.scale = S;
  c.veh.rr = static_cast<float>(1.2 * S);
  c.veh.d_hat = static_cast<float>(3.0 * S);
  c.veh.dt = 0.05f;
  c.veh.vmax = static_cast<float>(8.0 * S);
  c.veh.a_max = static_cast<float>(2.0 * S);
  c.veh.L = static_cast<float>(3.3 * S);
  c.range_m = 30.0;
  c.n_rays = 120;
  c.freeze_sense = true; // the map is known: the scenario is the policy's, not the sensor's
  c.reach_tol = static_cast<float>(3.0 * S);
  const auto lane = [&rng] { // a lane in [-52, 52] m, the same on every library
    return -half + 8.0 + (2.0 * half - 16.0) * (static_cast<double>(rng()) / 4294967296.0);
  };
  std::vector<float> o, goal, color;
  for (int i = 0; i < agents; ++i) {
    const double y0 = lane(), y1 = lane();
    o.insert(o.end(), {static_cast<float>(-50.0 * S), static_cast<float>(y0 * S)});
    goal.insert(goal.end(), {static_cast<float>(50.0 * S), static_cast<float>(y1 * S)});
    color.insert(color.end(), {1.0f, 1.0f, 1.0f});
  }
  sim_world w(c, occ.data(), occ.data(), coef_mlp::default_biased(), o.data(), goal.data(),
              color.data(), agents);
  for (int i = 0; i < agents; ++i) // facing their goals
    w.set_pose(
        i, o[2 * i], o[2 * i + 1],
        static_cast<float>(std::atan2(goal[2 * i + 1] - o[2 * i + 1], goal[2 * i] - o[2 * i])));
  trace_options t;
  t.world_id = "nav-lab:blocks";
  t.seed = seed;
  t.policies[0] = policy_ref{};
  t.meta = "{\"scenario\":\"blocks\",\"agents\":" + std::to_string(agents) + "}";
  scrub_scenario s;
  s.world = trace_header(w, t);
  s.ticks = ticks;
  s.name = "blocks, " + std::to_string(agents) + (agents == 1 ? " agent" : " agents") + ", seed " +
           std::to_string(seed);
  return s;
}

scrub_scenario scenario_from_trace(const std::string &trace_path, long ticks) {
  scrub_scenario s;
  s.world = input_trace::read(trace_path).header;
  policy_library lib;
  (void)world_from_trace(s.world, lib); // refuses a world v1 cannot rebuild, now
  s.ticks = ticks;
  s.name =
      (s.world.world_id.empty() ? fs::path(trace_path).filename().string() : s.world.world_id) +
      ", " + std::to_string(s.world.agents.size()) + " agents";
  return s;
}

std::string scenario_hash(const scrub_scenario &s) {
  input_trace_header h = s.world;
  h.build = "{}"; // the recording build is not the scenario
  h.meta = "{}";
  const std::string text = "cvc.nav.scrub/v" + std::to_string(kScrubVersion) + "\n" + h.to_json() +
                           "\nplant " + plant_name(s.plant) + "\nticks " + std::to_string(s.ticks) +
                           "\nlibcvc " + CVC_VERSION_STRING;
  return sha256_hex(reinterpret_cast<const unsigned char *>(text.data()), text.size())
      .substr(0, 16);
}

std::string scrub_cache_path(const std::string &history, const std::string &blob_sha256,
                             const std::string &scenario_hash) {
  return (fs::path(history + ".scrub") /
          (blob_sha256.substr(0, 16) + "-" + scenario_hash + ".cvcrun"))
      .string();
}

// ── a deployed checkpoint ───────────────────────────────────────────────────────────────────────

double scrub_metrics::value(int i) const {
  const double v[] = {reached, time_s, path_m, pen_steps, clear_min_m, clear_mean_m, goal_m};
  return i >= 0 && i < static_cast<int>(std::size(v)) ? v[i] : kNaN;
}

std::span<const scrub_metric_info> scrub_metric_names() { return kMetrics; }

scrub_metrics metrics_of(const run_log &log, int a) {
  scrub_metrics m;
  const long k = log.ticks() - 1;
  const int n = std::max(log.agents(), 1);
  m.reached = log.arm_at(k, a, "stats.reached") / n;
  m.time_s = log.arm_at(k, a, "stats.elapsed_s");
  m.path_m = log.arm_at(k, a, "stats.path_m");
  m.pen_steps = log.arm_at(k, a, "stats.pen_steps");
  m.clear_min_m = log.arm_at(k, a, "stats.clear_min_m");
  m.clear_mean_m = log.arm_at(k, a, "stats.clear_mean_m");
  double left = 0.0;
  for (int i = 0; i < log.agents(); ++i)
    left += std::hypot(log.at(k, a, i, "goal_x") - log.at(k, a, i, "x"),
                       log.at(k, a, i, "goal_y") - log.at(k, a, i, "y"));
  m.goal_m = k < 0 ? kNaN : left / n;
  return m;
}

run_log deploy_checkpoint(const scrub_scenario &s, const std::string &blob, const std::string &name,
                          policy_library &lib, const std::string &meta,
                          const scrub_plant_fn &plant) {
  const std::string want = plant_name(s.plant);
  if (want != kScrubKinematic && !plant)
    throw std::invalid_argument(std::string(kWho) + "the scenario's plant '" + want +
                                "' needs a scrub_options::plant hook (cvc::physics::scrub_plant)");
  std::vector<std::unique_ptr<sim_world>> worlds;
  worlds.push_back(std::make_unique<sim_world>(world_from_trace(s.world, lib, {{0, blob}})));
  // declared before the run, so the plant outlives the world the run owns
  scrub_plant_attached on{kScrubKinematic, nullptr};
  if (want != kScrubKinematic)
    on = plant(*worlds.front(), want);
  arm a;
  a.name = name;
  a.uri = blob;
  a.sha256 = blob.empty() ? std::string() : file_sha256(blob);
  a.meta = meta;
  arms_options o;
  o.seed = static_cast<unsigned>(s.world.seed);
  o.scene = s.name;
  o.plant = on.driving.empty() ? std::string(kScrubKinematic) : on.driving;
  std::string wj = "{\"world_id\":";
  tj::append_string(wj, s.world.world_id);
  wj += ",\"scenario\":";
  tj::append_string(wj, s.name);
  wj += ",\"scenario_hash\":";
  tj::append_string(wj, scenario_hash(s));
  wj += ",\"plant\":";
  tj::append_string(wj, want);
  wj += ",\"ticks\":" + std::to_string(s.ticks) + "}";
  o.world = wj;
  arms_run run(std::move(worlds), {a}, o);
  for (long k = 0; k < s.ticks; ++k)
    run.step(1);
  run.finish();
  return run.log();
}

std::vector<scrub_entry> scrub(const training_history &h, const scrub_scenario &s,
                               const scrub_options &o) {
  const std::string shash = scenario_hash(s);
  policy_library lib;
  std::vector<scrub_entry> out;
  for (std::size_t i = 0; i < h.checkpoints.size(); ++i) {
    scrub_entry e;
    e.checkpoint = h.checkpoints[i];
    e.sha256 = file_sha256(e.checkpoint.path);
    e.cache = scrub_cache_path(h.path, e.sha256, shash);
    std::error_code ec;
    if (o.use_cache && fs::is_regular_file(e.cache, ec)) {
      try {
        run_log l = run_log::read(e.cache);
        if (l.finished() && l.ticks() == s.ticks && l.arms() == 1) {
          e.log = std::make_shared<const run_log>(std::move(l));
          e.cached = true;
        }
      } catch (const std::exception &) {
        // a broken cache entry: deploy again
      }
    }
    if (!e.log) {
      const std::string meta = "{\"step\":" + tj::number(e.checkpoint.step) + "}";
      e.log = std::make_shared<const run_log>(deploy_checkpoint(
          s, e.checkpoint.path, "step " + step_text(e.checkpoint.step), lib, meta, o.plant));
      if (o.use_cache) {
        fs::create_directories(fs::path(e.cache).parent_path(), ec);
        e.log->write(e.cache);
      }
    }
    e.metrics = metrics_of(*e.log);
    if (o.say) {
      char b[256];
      std::snprintf(b, sizeof b,
                    "scrub: step %s (%zu/%zu) on %s, %s: reached %.2f, path %.1f m, %.0f ticks "
                    "penetrating, %.1f m to go",
                    step_text(e.checkpoint.step).c_str(), i + 1, h.checkpoints.size(),
                    plant_name(s.plant).c_str(), e.cached ? "cached" : "deployed",
                    e.metrics.reached, e.metrics.path_m, e.metrics.pen_steps, e.metrics.goal_m);
      o.say(b);
    }
    out.push_back(std::move(e));
  }
  return out;
}

std::string scrub_report_json(const training_history &h, const scrub_scenario &s,
                              const std::vector<scrub_entry> &entries) {
  std::string out = "{\"schema\":";
  tj::append_string(out, kScrubReportSchema);
  out += ",\"libcvc\":";
  tj::append_string(out, CVC_VERSION_STRING);
  out += ",\"history\":";
  tj::append_string(out, h.path);
  out += ",\"history_schema\":";
  tj::append_string(out, h.schema);
  out += ",\"run\":" + std::to_string(h.run) + ",\"scenario\":{\"name\":";
  tj::append_string(out, s.name);
  out += ",\"plant\":";
  tj::append_string(out, plant_name(s.plant));
  out += ",\"hash\":";
  tj::append_string(out, scenario_hash(s));
  out += ",\"ticks\":" + std::to_string(s.ticks) + ",\"world\":" + s.world.to_json() +
         "},\"checkpoints\":[";
  for (std::size_t i = 0; i < entries.size(); ++i) {
    const scrub_entry &e = entries[i];
    out += std::string(i ? "," : "") + "{\"step\":" + tj::number(e.checkpoint.step) + ",\"axis\":";
    tj::append_string(out, e.checkpoint.axis);
    out += ",\"blob\":";
    tj::append_string(out, e.checkpoint.path);
    out += ",\"sha256\":";
    tj::append_string(out, e.sha256);
    out += ",\"best\":" + std::string(e.checkpoint.best ? "true" : "false") +
           ",\"selected\":" + std::string(e.checkpoint.selected ? "true" : "false") +
           ",\"fitness\":" + tj::number(e.checkpoint.fitness) + ",\"cache\":";
    tj::append_string(out, e.cache);
    out += ",\"metrics\":{";
    for (std::size_t m = 0; m < std::size(kMetrics); ++m) {
      out += std::string(m ? "," : "") + "\"" + kMetrics[m].name +
             "\":" + tj::number(e.metrics.value(static_cast<int>(m)));
    }
    out += "}}";
  }
  out += "]}";
  return out;
}

panel_table scrub_table(const std::vector<scrub_entry> &entries, int k) {
  panel_table t{
      "scrub.table",
      {"at", "step", "flag", "reached", "time_s", "path_m", "pen_steps", "clear_min_m", "goal_m"},
      {}};
  for (std::size_t i = 0; i < entries.size(); ++i) {
    const scrub_entry &e = entries[i];
    const scrub_metrics &m = e.metrics;
    t.rows.push_back({static_cast<int>(i) == k ? ">" : "", step_text(e.checkpoint.step),
                      e.checkpoint.selected ? "pick" : (e.checkpoint.best ? "best" : ""),
                      panel_number(m.reached, 2), panel_number(m.time_s, 1),
                      panel_number(m.path_m, 1), panel_number(m.pen_steps, 0),
                      panel_number(m.clear_min_m, 2), panel_number(m.goal_m, 1)});
  }
  return t;
}

chart_plot metric_chart(const std::vector<scrub_entry> &entries, const std::string &metric,
                        std::string *text) {
  int mi = 2; // path_m
  for (int i = 0; i < static_cast<int>(std::size(kMetrics)); ++i)
    if (metric == kMetrics[i].name)
      mi = i;
  const scrub_metric_info &info = kMetrics[mi];
  chart_plot c{"scrub.metric", {}, {}};
  chart_series line{info.label, "c1", {}, {}}, base{"step 0", "c0 dashed", {}, {}};
  for (const scrub_entry &e : entries) {
    line.x.push_back(e.checkpoint.step);
    line.y.push_back(e.metrics.value(mi));
    if (e.checkpoint.selected)
      c.markers.push_back({e.checkpoint.step, "pick", "best"});
  }
  const double y0 = line.y.empty() ? kNaN : line.y.front();
  base.x = line.x;
  base.y.assign(line.y.size(), y0);
  c.series = {line, base};
  if (text) {
    *text = std::string(info.label) + ": step 0 " + panel_number(y0, info.digits);
    int lo = -1, hi = -1;
    for (int i = 0; i < static_cast<int>(line.y.size()); ++i) {
      if (!std::isfinite(line.y[i]))
        continue;
      if (lo < 0 || line.y[i] < line.y[lo])
        lo = i;
      if (hi < 0 || line.y[i] > line.y[hi])
        hi = i;
    }
    if (lo >= 0)
      *text += "; lowest " + panel_number(line.y[lo], info.digits) + " (step " +
               step_text(line.x[lo]) + "), highest " + panel_number(line.y[hi], info.digits) +
               " (step " + step_text(line.x[hi]) + ")";
  }
  return c;
}

chart_plot curve_chart(const training_history &h, const std::vector<scrub_entry> &entries,
                       const std::string &series, bool *log_y, std::string *text) {
  chart_plot c{"scrub.curve", {}, {}};
  const training_series *s = h.find(series);
  if (!s) {
    if (log_y)
      *log_y = false;
    if (text)
      *text = h.series.empty() ? "the history has no curve" : "no curve " + series;
    return c;
  }
  c.series.push_back({s->name, "c0", s->x, s->y});
  bool positive = !s->y.empty();
  double lo = std::numeric_limits<double>::infinity(), hi = -lo;
  for (double y : s->y)
    if (std::isfinite(y)) {
      positive = positive && y > 0;
      lo = std::min(lo, y);
      hi = std::max(hi, y);
    }
  for (const scrub_entry &e : entries)
    if (e.checkpoint.selected)
      c.markers.push_back({e.checkpoint.step, "pick", "best"});
  // spanning two decades or more of a positive quantity: a log axis
  if (log_y)
    *log_y = positive && lo > 0 && hi / lo >= 100.0;
  if (text) {
    *text = s->name + ": " + std::to_string(s->y.size()) + " points";
    if (std::isfinite(lo))
      *text += ", from " + panel_number(s->y.front(), 4) + " to " + panel_number(s->y.back(), 4);
  }
  return c;
}

// ── the view ────────────────────────────────────────────────────────────────────────────────────

scrub_view::scrub_view(training_history h, std::vector<scrub_entry> entries, int k,
                       std::string scenario)
    : scrub_view(std::move(h), std::vector<scrub_plant_runs>{{"", std::move(entries)}}, k,
                 std::move(scenario)) {}

scrub_view::scrub_view(training_history h, std::vector<scrub_plant_runs> plants, int k,
                       std::string scenario)
    : h_(std::move(h)), plants_(std::move(plants)), scenario_(std::move(scenario)) {
  if (plants_.empty() || plants_.front().entries.empty())
    throw std::invalid_argument(std::string(kWho) + "no checkpoint to show");
  const std::vector<scrub_entry> &first = plants_.front().entries;
  for (const scrub_plant_runs &p : plants_) {
    // unnamed: what its runs drove, else kinematic
    std::string name = p.plant;
    if (name.empty() && !p.entries.empty() && p.entries.front().log)
      name = p.entries.front().log->header().plant;
    name = plant_name(name);
    if (std::find(plant_labels_.begin(), plant_labels_.end(), name) != plant_labels_.end())
      throw std::invalid_argument(std::string(kWho) + "the plant '" + name + "' is named twice");
    bool same = p.entries.size() == first.size();
    for (std::size_t i = 0; same && i < first.size(); ++i)
      same = p.entries[i].sha256 == first[i].sha256;
    if (!same)
      throw std::invalid_argument(std::string(kWho) + "the plant '" + name +
                                  "' has other checkpoints than the first plant's");
    plant_labels_.push_back(name);
  }
  for (const scrub_entry &e : first)
    labels_.push_back("step " + step_text(e.checkpoint.step) +
                      (e.checkpoint.selected ? " (pick)" : ""));
  int pick = static_cast<int>(first.size()) - 1;
  for (int i = 0; i < static_cast<int>(first.size()); ++i)
    if (first[i].checkpoint.selected)
      pick = i;
  k_ = std::clamp(k < 0 ? pick : k, 0, static_cast<int>(first.size()) - 1);
}

void scrub_view::pick(int k) { k_ = std::clamp(k, 0, static_cast<int>(entries().size()) - 1); }

void scrub_view::use_plant(int p) {
  const int q = std::clamp(p, 0, static_cast<int>(plants_.size()) - 1);
  if (q == p_)
    return;
  p_ = q;
  shown_ = -1; // the pair's logs, the table and the charts: the new plant's
  metric_.clear();
  curve_.clear();
}

void scrub_view::pick_step(cvc::app &app, const std::string &prefix) {
  const std::string v = panel_get(app, panel_path(prefix, "scrub.pick"));
  std::istringstream in(v);
  in.imbue(std::locale::classic());
  double s = kNaN;
  if (!(in >> s) || !std::isfinite(s))
    return;
  int best = 0;
  for (int i = 1; i < static_cast<int>(entries().size()); ++i)
    if (std::abs(entries()[i].checkpoint.step - s) < std::abs(entries()[best].checkpoint.step - s))
      best = i;
  pick(best);
}

void scrub_view::show(cvc::app &app, const std::string &prefix, bool charts) {
  const scrub_entry &first = entries().front(), &now = entries()[static_cast<std::size_t>(k_)];
  std::vector<std::string> names = {"step " + step_text(first.checkpoint.step),
                                    "step " + step_text(now.checkpoint.step)};
  if (names[0] == names[1])
    names[1] += " "; // the same checkpoint twice: two arms all the same
  auto both = std::make_shared<const run_log>(
      run_log::stack({first.log.get(), now.log.get()}, {0, 0}, names));
  const double at = replay_ ? replay_->head().time() : 0.0;
  replay_ = std::make_unique<replay_view>(both, at);
  for (const char *key : {"view.arm", "ledger.agent"})
    panel_set(app, panel_path(prefix, key), names[1]);
  const std::string flag =
      now.checkpoint.selected ? ", the pick" : (now.checkpoint.best ? ", a best so far" : "");
  panel_set(app, panel_path(prefix, "scrub.now"),
            "checkpoint " + std::to_string(k_ + 1) + " of " + std::to_string(entries().size()) +
                ": step " + step_text(now.checkpoint.step) + flag + "; " +
                fs::path(now.checkpoint.path).filename().string());
  panel_set(app, panel_path(prefix, "scrub.step"), step_text(now.checkpoint.step));
  const std::string &plant = plant_labels_[static_cast<std::size_t>(p_)];
  char b[200];
  std::snprintf(b, sizeof b, "%s (blue) and %s (orange) on %s, %.1f s each", names[0].c_str(),
                names[1].c_str(), plant.c_str(), both->duration());
  panel_set(app, panel_path(prefix, "play.title"), b);
  panel_set(app, panel_path(prefix, "scrub.plant"), plant);
  write_table(app, prefix, scrub_table(entries(), k_));
  (void)charts;
  shown_ = k_;
}

scrub_view::frame scrub_view::update(cvc::app &app, const std::string &prefix, double dt,
                                     bool charts) {
  const int n = static_cast<int>(entries().size());
  if (!wrote_) {
    std::string labels, curves;
    for (std::size_t i = 0; i < labels_.size(); ++i)
      labels += (i ? "\n" : "") + labels_[i];
    for (std::size_t i = 0; i < h_.series.size(); ++i)
      curves += (i ? "\n" : "") + h_.series[i].name;
    std::string plants;
    for (std::size_t i = 0; i < plant_labels_.size(); ++i)
      plants += (i ? "\n" : "") + plant_labels_[i];
    panel_set(app, panel_path(prefix, "scrub.labels"), labels);
    panel_set(app, panel_path(prefix, "scrub.curve_names"), curves);
    panel_set(app, panel_path(prefix, "scrub.plant_labels"), plants);
    panel_set(app, panel_path(prefix, "scrub.plant_count"), std::to_string(plant_labels_.size()));
    const std::string have = panel_get(app, panel_path(prefix, "scrub.curve_name"));
    if (!h_.find(have))
      panel_set(app, panel_path(prefix, "scrub.curve_name"),
                h_.find("step.loss") ? "step.loss"
                                     : (h_.series.empty() ? "" : h_.series.front().name));
    char b[256];
    std::snprintf(b, sizeof b, "%s: %d checkpoints, steps %s to %s",
                  fs::path(h_.path).filename().string().c_str(), n,
                  step_text(entries().front().checkpoint.step).c_str(),
                  step_text(entries().back().checkpoint.step).c_str());
    panel_set(app, panel_path(prefix, "scrub.title"), b);
    if (!scenario_.empty())
      panel_set(app, panel_path(prefix, "scrub.scenario"), scenario_);
  }
  // the plant: a new choice shows the same checkpoint on it
  const std::string plant = panel_get(app, panel_path(prefix, "scrub.plant_choice"));
  if (wrote_ && plant != plant_choice_)
    for (int i = 0; i < static_cast<int>(plant_labels_.size()); ++i)
      if (plant_labels_[static_cast<std::size_t>(i)] == plant)
        use_plant(i);
  plant_choice_ = plant_labels_[static_cast<std::size_t>(p_)];
  panel_set(app, panel_path(prefix, "scrub.plant_choice"), plant_choice_);
  // whichever control moved since the last frame picks k; then every control shows it
  std::istringstream in(panel_get(app, panel_path(prefix, "scrub.pos"), "0"));
  in.imbue(std::locale::classic());
  double pos = 0.0;
  in >> pos;
  const std::string choice = panel_get(app, panel_path(prefix, "scrub.choice"));
  if (wrote_ && std::isfinite(pos) && std::abs(pos - pos_) > 1e-6) {
    pick(static_cast<int>(std::lround(std::clamp(pos, 0.0, 1.0) * (n - 1))));
  } else if (wrote_ && choice != choice_) {
    for (int i = 0; i < n; ++i)
      if (labels_[static_cast<std::size_t>(i)] == choice)
        pick(i);
  }
  char p[32];
  std::snprintf(p, sizeof p, "%.6f", n > 1 ? static_cast<double>(k_) / (n - 1) : 0.0);
  panel_set(app, panel_path(prefix, "scrub.pos"), p);
  pos_ = std::atof(p);
  choice_ = labels_[static_cast<std::size_t>(k_)];
  panel_set(app, panel_path(prefix, "scrub.choice"), choice_);
  wrote_ = true;
  if (shown_ != k_)
    show(app, prefix, charts);
  const std::string metric = panel_get(app, panel_path(prefix, "scrub.metric_name"), "path_m");
  if (metric != metric_) {
    std::string text;
    const chart_plot c = metric_chart(entries(), metric, &text);
    panel_set(app, panel_path(prefix, "scrub.metric_text"), text);
    if (charts)
      write_chart(app, prefix, c);
    metric_ = metric;
  }
  const std::string curve = panel_get(app, panel_path(prefix, "scrub.curve_name"));
  if (curve != curve_) {
    bool log_y = false;
    std::string text;
    const chart_plot c = curve_chart(h_, entries(), curve, &log_y, &text);
    panel_set(app, panel_path(prefix, "scrub.curve_text"), text);
    panel_set(app, panel_path(prefix, "scrub.curve_log"), log_y ? "1" : "0");
    if (charts)
      write_chart(app, prefix, c);
    curve_ = curve;
  }
  frame f;
  f.k = k_;
  f.replay = replay_->update(app, prefix, dt, charts);
  return f;
}

} // namespace nav
} // namespace cvc
