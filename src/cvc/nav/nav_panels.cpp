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
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/nav/nav_arms.h>
#include <cvc/nav/nav_panels.h>
#include <cvc/nav/plant_ledger.h>
#include <cvc/nav/run_log.h>
#include <cvc/state/state.h>
#include <limits>
#include <locale>
#include <sstream>

namespace cvc {
namespace nav {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kUnmeasured = 1e20;
constexpr const char *kSlots[] = {"blue", "orange",     "green",  "amber",
                                  "pink", "dark green", "purple", "red"};
constexpr const char *kCoef[] = {"alpha", "beta", "gamma", "lam_soft", "lam_hard", "mu"};
constexpr const char *kTerms[] = {"potential", "u_goal", "u_barrier",
                                  "u_soft",    "u_hard", "u_terrain"};

double parse_number(const std::string &s, double dflt) {
  if (s.empty())
    return dflt;
  std::istringstream in(s);
  in.imbue(std::locale::classic());
  double v = 0.0;
  if (!(in >> v))
    return dflt;
  return v;
}

bool truthy(const std::string &s) {
  return s == "1" || s == "true" || s == "True" || s == "yes" || s == "on";
}

// A number as Host.set_plot writes an x: an integer as one, else its shortest round trip.
std::string exact(double v) {
  if (!std::isfinite(v))
    return "nan";
  if (v == std::floor(v) && std::abs(v) < 9007199254740992.0) {
    char b[32];
    std::snprintf(b, sizeof b, "%.0f", v);
    return b;
  }
  return cvc::trace_json::number(v);
}

// A y: six significant digits, as format(v, ".6g").
std::string six(double v) {
  if (!std::isfinite(v))
    return "nan";
  char b[32];
  std::snprintf(b, sizeof b, "%.6g", v);
  return b;
}

// A position written and read back, as play.pos holds it.
std::string fixed6(double v) {
  char b[32];
  std::snprintf(b, sizeof b, "%.6f", v);
  std::string s = b;
  while (!s.empty() && s.back() == '0')
    s.pop_back();
  if (!s.empty() && s.back() == '.')
    s.pop_back();
  return s.empty() || s == "-" ? "0" : s;
}

std::string cell(const run_log &log, long k, int a, int i, const std::string &column, int digits) {
  return panel_number(log.at(k, a, i, column), digits);
}

std::string reached_text(double v, int agents) {
  if (!std::isfinite(v))
    return "-";
  if (agents == 1)
    return v >= 0.5 ? "yes" : "no";
  return std::to_string(static_cast<long>(std::lround(v))) + "/" + std::to_string(agents);
}

state::state_ptr find(cvc::app &app, const std::string &path) {
  return state::instance(app).findDescendantShared(path);
}

// Keep the ends, then each bucket's lowest and highest y (in their order) and its first gap.
void decimate(std::vector<double> &x, std::vector<double> &y, int max_points) {
  const std::size_t n = y.size();
  max_points = std::max(max_points, 8);
  if (n <= static_cast<std::size_t>(max_points))
    return;
  const std::size_t buckets = std::max<std::size_t>(1, (max_points - 2) / 3);
  const std::size_t size = (n - 2 + buckets - 1) / buckets;
  std::vector<std::size_t> keep = {0};
  for (std::size_t a = 1; a < n - 1; a += size) {
    const std::size_t b = std::min(a + size, n - 1);
    std::size_t lo = n, hi = n, gap = n;
    for (std::size_t k = a; k < b; ++k) {
      if (!std::isfinite(y[k])) {
        if (gap == n)
          gap = k;
        continue;
      }
      if (lo == n || y[k] < y[lo])
        lo = k;
      if (hi == n || y[k] > y[hi])
        hi = k;
    }
    std::vector<std::size_t> picks;
    for (std::size_t p : {gap, lo, hi})
      if (p != n && std::find(picks.begin(), picks.end(), p) == picks.end())
        picks.push_back(p);
    std::sort(picks.begin(), picks.end());
    keep.insert(keep.end(), picks.begin(), picks.end());
  }
  keep.push_back(n - 1);
  std::vector<double> xs, ys;
  for (std::size_t k : keep) {
    xs.push_back(x[k]);
    ys.push_back(y[k]);
  }
  x.swap(xs);
  y.swap(ys);
}

std::string joined(const std::vector<double> &v, std::string (*fmt)(double)) {
  std::string out;
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i)
      out += ',';
    out += fmt(v[i]);
  }
  return out;
}

// Remove `path`'s children past the first `keep` rows, and any child not in `names`.
void trim(cvc::app &app, const std::string &path, std::size_t keep) {
  const state::state_ptr node = find(app, path);
  if (!node)
    return;
  const std::string prefix = node->fullName() + ".";
  std::vector<std::string> drop;
  for (const std::string &full : node->children()) {
    if (full.compare(0, prefix.size(), prefix) != 0)
      continue;
    const std::string rest = full.substr(prefix.size());
    if (rest.find('.') != std::string::npos)
      continue;
    char *end = nullptr;
    const unsigned long i = std::strtoul(rest.c_str(), &end, 10);
    if (rest.empty() || *end != '\0' || i >= keep)
      drop.push_back(rest);
  }
  for (const std::string &name : drop)
    node->removeChild(name);
}

std::vector<chart_marker> leg_marks(const run_log &log) {
  std::vector<chart_marker> m;
  for (std::size_t j = 1; j < log.legs().size(); ++j)
    m.push_back({static_cast<double>(log.legs()[j].tick) * log.dt(), "leg " + std::to_string(j + 1),
                 "note"});
  return m;
}

} // namespace

std::string panel_number(double v, int digits) {
  if (!std::isfinite(v))
    return "-";
  char b[64];
  std::snprintf(b, sizeof b, "%.*f", std::max(digits, 0), v);
  std::string s = b;
  if (s.size() > 1 && s[0] == '-' && s.find_first_not_of("-0.") == std::string::npos)
    s.erase(0, 1); // no "-0.00"
  return s;
}

const char *slot_colour(int slot) {
  const int n = static_cast<int>(std::size(kSlots));
  return kSlots[((slot % n) + n) % n];
}

std::string panel_table::cell(std::size_t r, const std::string &name) const {
  for (std::size_t c = 0; c < columns.size(); ++c)
    if (columns[c] == name && r < rows.size() && c < rows[r].size())
      return rows[r][c];
  return {};
}

std::vector<panel_table> arms_tables(const run_log &log, long k, int agent) {
  const bool any = k >= 0 && k < log.ticks();
  panel_table stats{
      "arms.stats",
      {"arm", "colour", "clear_min", "clear_mean", "pen", "path", "elapsed", "ms", "reached"},
      {}};
  panel_table coef{"ledger.coef", {"arm"}, {}};
  panel_table terms{"ledger.terms", {"arm"}, {}};
  panel_table books{"ledger.books", {"arm"}, {}};
  panel_table tick{"ledger.tick", {"arm"}, {}};
  for (const char *c : kCoef)
    coef.columns.push_back(c);
  for (const char *c : kTerms)
    terms.columns.push_back(c);
  for (const char *b : book_names()) {
    books.columns.push_back(b);
    tick.columns.push_back(b);
  }
  for (int a = 0; a < log.arms(); ++a) {
    const run_log_arm &arm = log.header().arms[static_cast<std::size_t>(a)];
    auto stat = [&](const char *col) {
      return any ? log.arm_at(k, a, std::string("stats.") + col) : kNaN;
    };
    stats.rows.push_back({arm.name, slot_colour(arm.slot), panel_number(stat("clear_min_m"), 2),
                          panel_number(stat("clear_mean_m"), 2), panel_number(stat("pen_steps"), 0),
                          panel_number(stat("path_m"), 1), panel_number(stat("elapsed_s"), 1),
                          panel_number(stat("ms_step"), 3),
                          reached_text(stat("reached"), log.agents())});
    std::vector<std::string> r = {arm.name};
    for (const char *c : kCoef)
      r.push_back(any ? cell(log, k, a, agent, std::string("tel.") + c, 3) : "-");
    coef.rows.push_back(r);
    r = {arm.name};
    for (const char *c : kTerms)
      r.push_back(any ? cell(log, k, a, agent, std::string("tel.") + c, 3) : "-");
    terms.rows.push_back(r);
    std::vector<std::string> sum = {arm.name}, now = {arm.name};
    for (const char *b : book_names()) {
      const std::string col = std::string("book.") + b;
      sum.push_back(any ? panel_number(log.sum_to(k, a, agent, col), 3) : "-");
      now.push_back(any ? cell(log, k, a, agent, col, 3) : "-");
    }
    books.rows.push_back(sum);
    tick.rows.push_back(now);
  }
  return {stats, coef, terms, books, tick};
}

panel_table plant_table(const run_log &log, long k, int a, int agent) {
  panel_table t{"ledger.plant", {"key", "value", "unit"}, {}};
  for (const plant_ledger_item &it : plant_ledger_vehicle()) {
    const double v = log.at(k, a, agent, std::string("plant.") + it.key);
    if (!std::isfinite(v)) {
      t.rows.push_back({it.key, "n/a", ""});
      continue;
    }
    std::string text;
    if (it.type == plant_ledger_item::kind::flag)
      text = v >= 0.5 ? "yes" : "no";
    else if (it.type == plant_ledger_item::kind::count)
      text = panel_number(v, 0);
    else
      text = panel_number(v, 3);
    t.rows.push_back({it.key, text, it.unit});
  }
  return t;
}

std::string plant_note(const run_log &log, long k, int a, int agent) {
  const panel_table t = plant_table(log, k, a, agent);
  std::size_t applies = 0;
  for (const std::vector<std::string> &r : t.rows)
    applies += r[1] != "n/a";
  const std::string who =
      (a >= 0 && a < log.arms() ? log.header().arms[static_cast<std::size_t>(a)].name : "?") +
      (log.agents() > 1 ? " (agent " + std::to_string(agent) + ")" : "") + " on " +
      (log.header().plant.empty() ? std::string("kinematic") : log.header().plant) + ": ";
  if (!applies)
    return who + "no item applies (a physics plant feeds them)";
  return who + std::to_string(applies) + " of " + std::to_string(t.rows.size()) + " items apply";
}

std::vector<chart_plot> arms_charts(const run_log &log, int a, int agent, const std::string &book) {
  const std::vector<double> &t = log.times();
  const std::vector<chart_marker> marks = leg_marks(log);
  std::string b = "dU";
  for (const char *n : book_names())
    if (book == n)
      b = book;
  const double scale = log.header().scale > 0 ? log.header().scale : 1.0;
  chart_plot clear{"arms.clearance", {}, marks}, cum{"ledger.cum", {}, marks},
      cmp{"ledger.compare", {}, marks};
  for (int j = 0; j < log.arms(); ++j) {
    const run_log_arm &arm = log.header().arms[static_cast<std::size_t>(j)];
    std::vector<double> y = log.series(j, agent, "tel.clearance");
    for (double &v : y)
      v = std::isfinite(v) && std::abs(v) < kUnmeasured ? v / scale : kNaN;
    const std::string style = "c" + std::to_string(arm.slot);
    clear.series.push_back({arm.name, style, t, y});
    cmp.series.push_back({arm.name, style, t, log.cumulative(j, agent, "book." + b)});
  }
  if (a >= 0 && a < log.arms())
    for (const char *n : book_names())
      cum.series.push_back({n, "", t, log.cumulative(a, agent, std::string("book.") + n)});
  return {clear, cum, cmp};
}

// ── the state tree ──────────────────────────────────────────────────────────────────────────────

std::string panel_path(const std::string &prefix, const std::string &key) {
  return prefix.empty() ? key : prefix + "." + key;
}

std::string panel_get(cvc::app &app, const std::string &path, const std::string &dflt) {
  const state::state_ptr node = find(app, path);
  return node ? node->value() : dflt;
}

void panel_set(cvc::app &app, const std::string &path, const std::string &value) {
  state::instance(app)(path).value(value);
}

void write_table(cvc::app &app, const std::string &prefix, const panel_table &t) {
  const std::string path = panel_path(prefix, t.key);
  panel_set(app, path, std::to_string(t.rows.size()));
  for (std::size_t r = 0; r < t.rows.size(); ++r)
    for (std::size_t c = 0; c < t.columns.size(); ++c)
      panel_set(app, path + "." + std::to_string(r) + "." + t.columns[c],
                c < t.rows[r].size() ? t.rows[r][c] : std::string());
  trim(app, path, t.rows.size());
  const std::string cols = path + "_columns";
  panel_set(app, cols, std::to_string(t.columns.size()));
  for (std::size_t c = 0; c < t.columns.size(); ++c)
    panel_set(app, cols + "." + std::to_string(c), t.columns[c]);
  trim(app, cols, t.columns.size());
}

void write_chart(cvc::app &app, const std::string &prefix, const chart_plot &c, int max_points) {
  const std::string path = panel_path(prefix, c.key);
  state::instance(app)(path).removeChild("s");
  state::instance(app)(path).removeChild("m");
  panel_set(app, path + ".s", std::to_string(c.series.size()));
  for (std::size_t i = 0; i < c.series.size(); ++i) {
    std::vector<double> x = c.series[i].x, y = c.series[i].y;
    y.resize(x.size(), kNaN);
    decimate(x, y, max_points);
    const std::string s = path + ".s." + std::to_string(i);
    panel_set(app, s + ".label", c.series[i].label);
    panel_set(app, s + ".x", joined(x, exact));
    panel_set(app, s + ".y", joined(y, six));
    if (!c.series[i].style.empty())
      panel_set(app, s + ".style", c.series[i].style);
  }
  panel_set(app, path + ".m", std::to_string(c.markers.size()));
  for (std::size_t j = 0; j < c.markers.size(); ++j) {
    const std::string m = path + ".m." + std::to_string(j);
    panel_set(app, m + ".x", exact(c.markers[j].x));
    panel_set(app, m + ".label", c.markers[j].label);
    panel_set(app, m + ".kind", c.markers[j].kind);
  }
}

int arm_index(const run_log &log, const std::string &name, int dflt) {
  for (int a = 0; a < log.arms(); ++a)
    if (log.header().arms[static_cast<std::size_t>(a)].name == name)
      return a;
  return dflt;
}

panel_choice read_choice(cvc::app &app, const std::string &prefix, const run_log &log) {
  panel_choice c;
  c.arm = arm_index(log, panel_get(app, panel_path(prefix, "ledger.agent")), 0);
  const double v = parse_number(panel_get(app, panel_path(prefix, "ledger.vehicle")), 0.0);
  c.agent =
      std::clamp(static_cast<int>(std::isfinite(v) ? v : 0.0), 0, std::max(log.agents() - 1, 0));
  const std::string book = panel_get(app, panel_path(prefix, "ledger.book"), "dU");
  for (const char *n : book_names())
    if (book == n)
      c.book = book;
  return c;
}

void publish_tables(cvc::app &app, const std::string &prefix, const run_log &log, long k,
                    const panel_choice &c) {
  std::string names;
  for (int a = 0; a < log.arms(); ++a)
    names += (a ? "\n" : "") + log.header().arms[static_cast<std::size_t>(a)].name;
  panel_set(app, panel_path(prefix, "arms.names"), names);
  panel_set(app, panel_path(prefix, "arms.agents"), std::to_string(log.agents()));
  for (const panel_table &t : arms_tables(log, k, c.agent))
    write_table(app, prefix, t);
  write_table(app, prefix, plant_table(log, k, c.arm, c.agent));
  panel_set(app, panel_path(prefix, "ledger.plant_note"), plant_note(log, k, c.arm, c.agent));
}

void publish_charts(cvc::app &app, const std::string &prefix, const run_log &log,
                    const panel_choice &c) {
  for (const chart_plot &p : arms_charts(log, c.arm, c.agent, c.book))
    write_chart(app, prefix, p);
}

// ── the play head ───────────────────────────────────────────────────────────────────────────────

play_head::play_head(double duration, double dt, double at)
    : duration_(std::max(duration, 1e-6)), dt_(dt), t_(0.0) {
  t_ = clamp(at);
}

double play_head::clamp(double t) const {
  return std::isfinite(t) ? std::clamp(t, 0.0, duration_) : 0.0;
}

void play_head::step() { t_ = clamp(t_ + dt_); }

void play_head::restart(cvc::app &app, const std::string &prefix) {
  t_ = 0.0;
  panel_set(app, panel_path(prefix, "sim.paused"), "0");
}

void play_head::seek(cvc::app &app, const std::string &prefix) {
  const double t = parse_number(panel_get(app, panel_path(prefix, "play.seek")), kNaN);
  if (std::isfinite(t))
    t_ = clamp(t);
}

void play_head::set(double t) { t_ = clamp(t); }

double play_head::update(cvc::app &app, const std::string &prefix, double frame_dt) {
  const double pos = parse_number(panel_get(app, panel_path(prefix, "play.pos")), 0.0);
  if (wrote_ && std::abs(pos - pos_) > 1e-6) { // dragged: the slider wins
    t_ = clamp(pos * duration_);
  } else if (wrote_ && !truthy(panel_get(app, panel_path(prefix, "sim.paused")))) {
    const double speed = parse_number(panel_get(app, panel_path(prefix, "sim.speed")), 1.0);
    t_ += frame_dt * std::max(std::isfinite(speed) ? speed : 1.0, 0.0);
    if (t_ > duration_) {
      if (truthy(panel_get(app, panel_path(prefix, "play.loop")))) {
        t_ = 0.0;
      } else {
        t_ = duration_;
        panel_set(app, panel_path(prefix, "sim.paused"), "1");
      }
    }
  }
  const std::string p = fixed6(t_ / duration_);
  panel_set(app, panel_path(prefix, "play.pos"), p);
  pos_ = parse_number(p, 0.0);
  wrote_ = true;
  char b[64];
  std::snprintf(b, sizeof b, "%.4f", t_);
  panel_set(app, panel_path(prefix, "play.t"), b);
  std::snprintf(b, sizeof b, "%6.1f / %.1f s", t_, duration_);
  panel_set(app, panel_path(prefix, "play.time"), b);
  return t_;
}

std::vector<std::vector<ghost_pose>> poses_at(const run_log &log, double t) {
  std::vector<std::vector<ghost_pose>> out(static_cast<std::size_t>(log.arms()));
  if (log.empty())
    return out;
  const std::vector<double> &ts = log.times();
  const long n = log.ticks();
  long k = static_cast<long>(std::upper_bound(ts.begin(), ts.end(), t + 1e-6) - ts.begin()) - 1;
  double f = 0.0;
  long k1 = 0;
  if (k < 0) {
    k = 0;
  } else {
    k1 = std::min(k + 1, n - 1);
    if (k1 != k)
      f = std::clamp((t - ts[k]) / (ts[k1] - ts[k]), 0.0, 1.0);
  }
  if (k1 < k)
    k1 = k;
  for (int a = 0; a < log.arms(); ++a)
    for (int i = 0; i < log.agents(); ++i) {
      ghost_pose g;
      const double x0 = log.at(k, a, i, "x"), y0 = log.at(k, a, i, "y");
      const double h0 = log.at(k, a, i, "heading"), h1 = log.at(k1, a, i, "heading");
      const double dh = std::atan2(std::sin(h1 - h0), std::cos(h1 - h0));
      g.x = x0 + f * (log.at(k1, a, i, "x") - x0);
      g.y = y0 + f * (log.at(k1, a, i, "y") - y0);
      g.heading = h0 + f * dh;
      g.length = log.at(k, a, i, "length");
      g.width = log.at(k, a, i, "width");
      g.wheelbase = log.at(k, a, i, "wheelbase");
      out[static_cast<std::size_t>(a)].push_back(g);
    }
  return out;
}

// ── a replay ────────────────────────────────────────────────────────────────────────────────────

replay_view::replay_view(std::shared_ptr<const run_log> log, double at)
    : log_(std::move(log)), head_(log_ ? log_->duration() : 0.0, log_ ? log_->dt() : 0.05, at) {
  if (!log_)
    throw std::invalid_argument("cvc::nav::replay_view: no run log");
}

replay_view::frame replay_view::update(cvc::app &app, const std::string &prefix, double dt,
                                       bool charts) {
  frame f;
  f.t = head_.update(app, prefix, dt);
  f.k = log_->index(f.t);
  f.leg = log_->leg_of(f.k);
  f.choice = read_choice(app, prefix, *log_);
  publish_tables(app, prefix, *log_, log_->empty() ? -1 : f.k, f.choice);
  char b[128];
  std::snprintf(b, sizeof b, "t = %.1f s, tick %ld of %ld, leg %d",
                log_->empty() ? 0.0 : log_->times()[static_cast<std::size_t>(f.k)], f.k + 1,
                log_->ticks(), f.leg + 1);
  panel_set(app, panel_path(prefix, "arms.time"), b);
  if (charts && (!charted_ || !(f.choice == shown_))) {
    publish_charts(app, prefix, *log_, f.choice);
    charted_ = true;
    shown_ = f.choice;
  }
  f.poses = poses_at(*log_, f.t);
  return f;
}

} // namespace nav
} // namespace cvc
