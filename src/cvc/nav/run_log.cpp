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
#include <cvc/nav/run_log.h>
#include <fstream>
#include <istream>
#include <limits>
#include <ostream>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace nav {
namespace {

namespace tj = cvc::trace_json;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr const char *kWho = "cvc::nav::run_log: ";

std::string json_str(std::string_view s) {
  std::string out;
  tj::append_string(out, s);
  return out;
}

// A JSON object's text, checked: "{}" for an empty one.
std::string object_text(const std::string &text, const char *what) {
  if (text.empty())
    return "{}";
  tj::value v;
  try {
    v = tj::parse(text);
  } catch (const std::exception &e) {
    throw std::invalid_argument(std::string(kWho) + what + " is not JSON: " + e.what());
  }
  if (!v.is_object())
    throw std::invalid_argument(std::string(kWho) + what + " is not a JSON object");
  return v.dump();
}

std::string numbers(const double *v, std::size_t n) {
  std::string out = "[";
  for (std::size_t i = 0; i < n; ++i) {
    if (i)
      out += ',';
    out += tj::number(v[i]);
  }
  out += ']';
  return out;
}

// [[x,y],...] or [[x,y,h],...]: `per` numbers each.
std::string tuples(const std::vector<double> &v, std::size_t per) {
  std::string out = "[";
  for (std::size_t i = 0; i + per <= v.size(); i += per) {
    if (i)
      out += ',';
    out += numbers(v.data() + i, per);
  }
  out += ']';
  return out;
}

std::string strings(const std::vector<std::string> &v) {
  std::string out = "[";
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i)
      out += ',';
    tj::append_string(out, v[i]);
  }
  out += ']';
  return out;
}

// A list of JSON objects' texts as a JSON array ("{}" for an empty text).
std::string objects(const std::vector<std::string> &v) {
  std::string out = "[";
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i)
      out += ',';
    out += v[i].empty() ? "{}" : v[i];
  }
  out += ']';
  return out;
}

double num_or_nan(const tj::value &v) {
  if (v.t == tj::value::type::null)
    return kNaN;
  return v.number("a run log value");
}

std::string str_of(const tj::value &o, const char *key) {
  const tj::value *v = o.get(key);
  return v && v->is_string() ? v->s : std::string();
}

double num_of(const tj::value &o, const char *key, double dflt) {
  const tj::value *v = o.get(key);
  return v && v->is_number() ? v->number(key) : dflt;
}

std::string obj_of(const tj::value &o, const char *key) {
  const tj::value *v = o.get(key);
  return v && v->is_object() ? v->dump() : std::string("{}");
}

std::vector<double> flat(const tj::value *v, std::size_t per, const char *what) {
  std::vector<double> out;
  if (!v)
    return out;
  if (!v->is_array())
    throw std::runtime_error(std::string(kWho) + what + " is not a list");
  for (const tj::value &row : v->a) {
    if (!row.is_array() || row.a.size() != per)
      throw std::runtime_error(std::string(kWho) + what + ": each needs " + std::to_string(per) +
                               " numbers");
    for (const tj::value &x : row.a)
      out.push_back(num_or_nan(x));
  }
  return out;
}

std::vector<std::string> string_list(const tj::value *v) {
  std::vector<std::string> out;
  if (v && v->is_array())
    for (const tj::value &s : v->a)
      out.push_back(s.is_string() ? s.s : std::string());
  return out;
}

std::vector<std::string> object_list(const tj::value *v) {
  std::vector<std::string> out;
  if (v && v->is_array())
    for (const tj::value &o : v->a)
      out.push_back(o.dump());
  return out;
}

} // namespace

// ── the header ──────────────────────────────────────────────────────────────────────────────────

std::string run_log_header::to_json() const {
  std::string out = "{\"schema\":" + json_str(schema) + ",\"libcvc\":" + json_str(libcvc) +
                    ",\"dt\":" + tj::number(dt) + ",\"scale\":" + tj::number(scale) +
                    ",\"center\":[" + tj::number(cx) + "," + tj::number(cy) +
                    "],\"seed\":" + tj::number(seed) + ",\"scene\":" + json_str(scene) +
                    ",\"plant\":" + json_str(plant) + ",\"agents\":" + std::to_string(agents) +
                    ",\"arms\":[";
  for (std::size_t j = 0; j < arms.size(); ++j) {
    const run_log_arm &a = arms[j];
    out += std::string(j ? "," : "") + "{\"name\":" + json_str(a.name) +
           ",\"slot\":" + std::to_string(a.slot) + ",\"key\":" + json_str(a.key) +
           ",\"policy\":{\"uri\":" + json_str(a.uri) + ",\"sha256\":" + json_str(a.sha256) +
           "},\"meta\":" + object_text(a.meta, "an arm's meta") + "}";
  }
  out += "],\"start\":" + tuples(start, 3) + ",\"world\":" + object_text(world, "world") +
         ",\"settings\":" + object_text(settings, "settings") +
         ",\"meta\":" + object_text(meta, "meta") +
         ",\"columns\":{\"agent\":" + strings(agent_columns) + ",\"arm\":" + strings(arm_columns) +
         "}}";
  return out;
}

run_log_header run_log_header::from_json(const std::string &json) {
  const tj::value v = tj::parse(json);
  if (!v.is_object())
    throw std::runtime_error(std::string(kWho) + "the header is not a JSON object");
  run_log_header h;
  h.schema = str_of(v, "schema");
  if (h.schema != kRunLogSchema)
    throw std::runtime_error(std::string(kWho) + "not a run log (" + kRunLogSchema + "): schema '" +
                             h.schema + "'");
  h.libcvc = str_of(v, "libcvc");
  h.dt = num_of(v, "dt", 0.0);
  h.scale = num_of(v, "scale", 1.0);
  if (const tj::value *c = v.get("center"); c && c->is_array() && c->a.size() == 2) {
    h.cx = num_or_nan(c->a[0]);
    h.cy = num_or_nan(c->a[1]);
  }
  if (const tj::value *s = v.get("seed"); s && s->is_number())
    h.seed = s->u64("seed");
  h.scene = str_of(v, "scene");
  h.plant = str_of(v, "plant");
  h.agents = static_cast<int>(num_of(v, "agents", 0.0));
  if (h.agents < 0)
    throw std::runtime_error(std::string(kWho) + "a negative agent count");
  if (const tj::value *arms = v.get("arms"); arms && arms->is_array()) {
    for (const tj::value &a : arms->a) {
      run_log_arm arm;
      arm.name = str_of(a, "name");
      arm.slot = static_cast<int>(num_of(a, "slot", static_cast<double>(h.arms.size())));
      arm.key = str_of(a, "key");
      if (const tj::value *p = a.get("policy"); p && p->is_object()) {
        arm.uri = str_of(*p, "uri");
        arm.sha256 = str_of(*p, "sha256");
      }
      arm.meta = obj_of(a, "meta");
      h.arms.push_back(std::move(arm));
    }
  }
  h.start = flat(v.get("start"), 3, "start");
  h.world = obj_of(v, "world");
  h.settings = obj_of(v, "settings");
  h.meta = obj_of(v, "meta");
  if (const tj::value *c = v.get("columns"); c && c->is_object()) {
    h.agent_columns = string_list(c->get("agent"));
    h.arm_columns = string_list(c->get("arm"));
  }
  return h;
}

// ── the log ─────────────────────────────────────────────────────────────────────────────────────

run_log::run_log(run_log_header h) : h_(std::move(h)) {}

int run_log::agent_column(std::string_view name) const {
  for (std::size_t c = 0; c < h_.agent_columns.size(); ++c)
    if (h_.agent_columns[c] == name)
      return static_cast<int>(c);
  return -1;
}

int run_log::arm_column(std::string_view name) const {
  for (std::size_t c = 0; c < h_.arm_columns.size(); ++c)
    if (h_.arm_columns[c] == name)
      return static_cast<int>(c);
  return -1;
}

double run_log::at(long k, int a, int i, int c) const {
  if (k < 0 || k >= ticks() || a < 0 || a >= arms() || i < 0 || i >= agents() || c < 0 ||
      c >= static_cast<int>(agent_stride()))
    return kNaN;
  const std::size_t idx =
      ((static_cast<std::size_t>(k) * arms() + a) * agents() + i) * agent_stride() + c;
  return agent_[idx];
}

double run_log::arm_at(long k, int a, int c) const {
  if (k < 0 || k >= ticks() || a < 0 || a >= arms() || c < 0 || c >= static_cast<int>(arm_stride()))
    return kNaN;
  return arm_[(static_cast<std::size_t>(k) * arms() + a) * arm_stride() + c];
}

double run_log::at(long k, int a, int i, std::string_view column) const {
  return at(k, a, i, agent_column(column));
}

double run_log::arm_at(long k, int a, std::string_view column) const {
  return arm_at(k, a, arm_column(column));
}

std::vector<double> run_log::series(int a, int i, std::string_view column) const {
  const int c = agent_column(column);
  std::vector<double> out(static_cast<std::size_t>(ticks()));
  for (long k = 0; k < ticks(); ++k)
    out[static_cast<std::size_t>(k)] = at(k, a, i, c);
  return out;
}

std::vector<double> run_log::arm_series(int a, std::string_view column) const {
  const int c = arm_column(column);
  std::vector<double> out(static_cast<std::size_t>(ticks()));
  for (long k = 0; k < ticks(); ++k)
    out[static_cast<std::size_t>(k)] = arm_at(k, a, c);
  return out;
}

std::vector<double> run_log::cumulative(int a, int i, std::string_view column) const {
  std::vector<double> out = series(a, i, column);
  double s = 0.0;
  for (double &v : out) {
    s += v;
    v = s;
  }
  return out;
}

double run_log::sum_to(long k, int a, int i, std::string_view column) const {
  const int c = agent_column(column);
  double s = 0.0;
  for (long j = 0; j <= k && j < ticks(); ++j)
    s += at(j, a, i, c);
  return s;
}

long run_log::index(double t) const {
  if (t_.empty())
    return 0;
  // a microsecond's slack: a tick's end is ticks x a float dt, a hair past the decimal time
  const auto it = std::upper_bound(t_.begin(), t_.end(), t + 1e-6);
  const long k = static_cast<long>(it - t_.begin()) - 1;
  return std::clamp(k, 0L, ticks() - 1);
}

int run_log::leg_of(long k) const {
  int leg = 0;
  for (std::size_t j = 0; j < legs_.size(); ++j)
    if (legs_[j].tick <= k)
      leg = static_cast<int>(j);
  return leg;
}

void run_log::begin_leg(long tick, std::vector<double> goals) {
  if (goals.size() != static_cast<std::size_t>(agents()) * 2)
    throw std::invalid_argument(std::string(kWho) + "a leg needs a goal per agent");
  run_log_leg leg;
  leg.tick = tick;
  leg.goals = std::move(goals);
  legs_.push_back(std::move(leg));
}

void run_log::end_leg(std::vector<std::string> nav_stats) {
  if (legs_.empty())
    throw std::logic_error(std::string(kWho) + "no leg to end");
  legs_.back().nav_stats = std::move(nav_stats);
}

void run_log::add_tick(double t, const std::vector<double> &agent_values,
                       const std::vector<double> &arm_values) {
  const std::size_t na = static_cast<std::size_t>(arms());
  if (agent_values.size() != na * agents() * agent_stride() ||
      arm_values.size() != na * arm_stride())
    throw std::invalid_argument(std::string(kWho) + "a tick has the wrong number of values");
  t_.push_back(t);
  agent_.insert(agent_.end(), agent_values.begin(), agent_values.end());
  arm_.insert(arm_.end(), arm_values.begin(), arm_values.end());
}

void run_log::finish(std::vector<std::string> nav_stats) {
  end_stats_ = std::move(nav_stats);
  finished_ = true;
}

std::string run_log::leg_json(std::size_t leg) const {
  const run_log_leg &l = legs_.at(leg);
  return "{\"leg\":{\"index\":" + std::to_string(leg) + ",\"tick\":" + std::to_string(l.tick) +
         ",\"goals\":" + tuples(l.goals, 2) + "}}";
}

std::string run_log::leg_end_json(std::size_t leg) const {
  const run_log_leg &l = legs_.at(leg);
  const long end = leg + 1 < legs_.size() ? legs_[leg + 1].tick : ticks();
  return "{\"leg_end\":{\"index\":" + std::to_string(leg) + ",\"tick\":" + std::to_string(end) +
         ",\"nav_stats\":" + objects(l.nav_stats) + "}}";
}

std::string run_log::tick_json(long k) const {
  if (k < 0 || k >= ticks())
    throw std::out_of_range(std::string(kWho) + "no tick " + std::to_string(k));
  std::string out =
      "{\"tick\":" + std::to_string(k + 1) + ",\"t\":" + tj::number(t_[k]) + ",\"agent\":[";
  for (int a = 0; a < arms(); ++a) {
    out += a ? ",[" : "[";
    for (int i = 0; i < agents(); ++i) {
      if (i)
        out += ',';
      const std::size_t idx =
          ((static_cast<std::size_t>(k) * arms() + a) * agents() + i) * agent_stride();
      out += numbers(agent_.data() + idx, agent_stride());
    }
    out += ']';
  }
  out += "],\"arm\":[";
  for (int a = 0; a < arms(); ++a) {
    if (a)
      out += ',';
    out += numbers(arm_.data() + (static_cast<std::size_t>(k) * arms() + a) * arm_stride(),
                   arm_stride());
  }
  out += "]}";
  return out;
}

std::string run_log::end_json() const {
  return "{\"end\":{\"ticks\":" + std::to_string(ticks()) +
         ",\"nav_stats\":" + objects(end_stats_) + "}}";
}

void run_log::write(std::ostream &out) const {
  out << h_.to_json() << '\n';
  std::size_t leg = 0;
  for (long k = 0; k <= ticks(); ++k) {
    // a leg's lines go before its first tick (a leg begun after the last tick, at its end)
    while (leg < legs_.size() && legs_[leg].tick == k) {
      if (leg > 0 && !legs_[leg - 1].nav_stats.empty())
        out << leg_end_json(leg - 1) << '\n';
      out << leg_json(leg) << '\n';
      ++leg;
    }
    if (k < ticks())
      out << tick_json(k) << '\n';
  }
  if (finished_)
    out << end_json() << '\n';
}

void run_log::write(const std::string &path) const {
  std::ofstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error(std::string(kWho) + "cannot write " + path);
  write(f);
  if (!f.flush())
    throw std::runtime_error(std::string(kWho) + "cannot write " + path);
}

run_log run_log::read(std::istream &in) {
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);)
    if (line.find_first_not_of(" \t\r") != std::string::npos)
      lines.push_back(std::move(line));
  if (lines.empty())
    throw std::runtime_error(std::string(kWho) + "an empty file is not a run log");
  run_log log(run_log_header::from_json(lines[0]));
  const std::size_t na = static_cast<std::size_t>(log.arms());
  const std::size_t nn = static_cast<std::size_t>(log.agents());
  const std::size_t sa = log.agent_stride(), sm = log.arm_stride();
  std::vector<double> agent_values, arm_values;
  for (std::size_t li = 1; li < lines.size(); ++li) {
    tj::value v;
    try {
      v = tj::parse(lines[li]);
    } catch (const std::exception &e) {
      if (li + 1 == lines.size()) { // the writer stopped mid-line: keep what came before
        log.notes_.push_back("line " + std::to_string(li + 1) + " is cut short, skipped");
        break;
      }
      throw std::runtime_error(std::string(kWho) + "line " + std::to_string(li + 1) + ": " +
                               e.what());
    }
    if (!v.is_object())
      throw std::runtime_error(std::string(kWho) + "line " + std::to_string(li + 1) +
                               " is not a JSON object");
    if (const tj::value *leg = v.get("leg")) {
      log.begin_leg(static_cast<long>(num_of(*leg, "tick", static_cast<double>(log.ticks()))),
                    flat(leg->get("goals"), 2, "a leg's goals"));
    } else if (const tj::value *le = v.get("leg_end")) {
      if (!log.legs_.empty())
        log.legs_.back().nav_stats = object_list(le->get("nav_stats"));
    } else if (const tj::value *end = v.get("end")) {
      log.finish(object_list(end->get("nav_stats")));
    } else if (v.get("tick")) {
      const tj::value *ag = v.get("agent"), *am = v.get("arm");
      if (!ag || !ag->is_array() || ag->a.size() != na || !am || !am->is_array() ||
          am->a.size() != na)
        throw std::runtime_error(std::string(kWho) + "line " + std::to_string(li + 1) +
                                 ": a tick needs a value list per arm");
      agent_values.assign(na * nn * sa, kNaN);
      arm_values.assign(na * sm, kNaN);
      for (std::size_t a = 0; a < na; ++a) {
        const tj::value &arm_agents = ag->a[a];
        if (!arm_agents.is_array() || arm_agents.a.size() != nn)
          throw std::runtime_error(std::string(kWho) + "line " + std::to_string(li + 1) +
                                   ": a tick needs a value list per agent");
        for (std::size_t i = 0; i < nn; ++i) {
          const tj::value &row = arm_agents.a[i];
          if (!row.is_array() || row.a.size() != sa)
            throw std::runtime_error(std::string(kWho) + "line " + std::to_string(li + 1) +
                                     ": an agent's values do not match the columns");
          for (std::size_t c = 0; c < sa; ++c)
            agent_values[(a * nn + i) * sa + c] = num_or_nan(row.a[c]);
        }
        const tj::value &row = am->a[a];
        if (!row.is_array() || row.a.size() != sm)
          throw std::runtime_error(std::string(kWho) + "line " + std::to_string(li + 1) +
                                   ": an arm's values do not match the columns");
        for (std::size_t c = 0; c < sm; ++c)
          arm_values[a * sm + c] = num_or_nan(row.a[c]);
      }
      log.add_tick(num_of(v, "t", static_cast<double>(log.ticks() + 1) * log.dt()), agent_values,
                   arm_values);
    } // a line of another kind: a later writer's, skipped
  }
  return log;
}

run_log run_log::read(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error(std::string(kWho) + "cannot read " + path);
  try {
    return read(f);
  } catch (const std::runtime_error &e) {
    throw std::runtime_error(path + ": " + e.what());
  }
}

run_log run_log::stack(const std::vector<const run_log *> &logs, const std::vector<int> &arms,
                       const std::vector<std::string> &names) {
  if (logs.empty() || logs.size() != arms.size() || logs.size() != names.size())
    throw std::invalid_argument(std::string(kWho) + "stack: a log, an arm and a name each");
  if (!logs[0])
    throw std::invalid_argument(std::string(kWho) + "stack: no such arm");
  const run_log &first = *logs[0];
  for (std::size_t j = 0; j < logs.size(); ++j) {
    if (!logs[j] || arms[j] < 0 || arms[j] >= logs[j]->arms())
      throw std::invalid_argument(std::string(kWho) + "stack: no such arm");
    const run_log &l = *logs[j];
    if (l.agents() != first.agents() || l.ticks() != first.ticks() ||
        l.h_.agent_columns != first.h_.agent_columns || l.h_.arm_columns != first.h_.arm_columns)
      throw std::invalid_argument(std::string(kWho) +
                                  "stack: the logs differ in agents, columns or ticks");
  }
  run_log_header h = first.h_;
  h.arms.clear();
  for (std::size_t j = 0; j < logs.size(); ++j) {
    run_log_arm a = logs[j]->h_.arms[static_cast<std::size_t>(arms[j])];
    a.name = names[j];
    a.slot = static_cast<int>(j);
    h.arms.push_back(std::move(a));
  }
  run_log out(std::move(h));
  // each arm's own nav_stats records, where its log kept them
  const auto pick = [&](auto get) {
    std::vector<std::string> v;
    for (std::size_t j = 0; j < logs.size(); ++j) {
      const std::vector<std::string> &s = get(*logs[j]);
      if (static_cast<std::size_t>(arms[j]) >= s.size())
        return std::vector<std::string>();
      v.push_back(s[static_cast<std::size_t>(arms[j])]);
    }
    return v;
  };
  const std::vector<std::string> none;
  out.legs_ = first.legs_;
  for (std::size_t L = 0; L < out.legs_.size(); ++L)
    out.legs_[L].nav_stats = pick([&](const run_log &l) -> const std::vector<std::string> & {
      return L < l.legs_.size() ? l.legs_[L].nav_stats : none;
    });
  if (first.finished_)
    out.finish(
        pick([](const run_log &l) -> const std::vector<std::string> & { return l.end_stats_; }));
  const std::size_t sa = first.agent_stride() * first.agents(), sm = first.arm_stride();
  std::vector<double> av, mv;
  for (long k = 0; k < first.ticks(); ++k) {
    av.clear();
    mv.clear();
    for (std::size_t j = 0; j < logs.size(); ++j) {
      const run_log &l = *logs[j];
      const std::size_t a = static_cast<std::size_t>(arms[j]);
      const std::size_t ai = (static_cast<std::size_t>(k) * l.arms() + a) * sa;
      av.insert(av.end(), l.agent_.begin() + ai, l.agent_.begin() + ai + sa);
      const std::size_t mi = (static_cast<std::size_t>(k) * l.arms() + a) * sm;
      mv.insert(mv.end(), l.arm_.begin() + mi, l.arm_.begin() + mi + sm);
    }
    out.add_tick(first.t_[k], av, mv);
  }
  return out;
}

// ── streaming ───────────────────────────────────────────────────────────────────────────────────

run_log_writer::run_log_writer(const std::string &path)
    : path_(path), out_(std::make_unique<std::ofstream>(path, std::ios::binary)) {
  if (!*out_)
    throw std::runtime_error(std::string(kWho) + "cannot write " + path);
}

run_log_writer::~run_log_writer() = default;

void run_log_writer::line(const std::string &json) {
  *out_ << json << '\n';
  out_->flush();
  if (!*out_)
    throw std::runtime_error(std::string(kWho) + "cannot write " + path_);
}

} // namespace nav
} // namespace cvc
