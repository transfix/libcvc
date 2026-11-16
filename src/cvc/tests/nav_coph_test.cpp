/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// CoPH's records and stage ledger (coph_config.h, coph_ledger.h: C1 of
// docs/roadmap/NAV-COPH-ROADMAP.md) against GRL-SNAM's grl_snam.coph. The golden,
// src/cvc/tests/data/coph_golden.json, is written by GRL-SNAM (python -m
// grl_snam.tools.coph_golden --out DIR); this side reproduces it:
//   G1  config   every acceptance and refusal (Python's message), every field's value after
//                validation, and the fields outside the demos' profile, which the ledger refuses
//   G4  stages   the demos' learner's ledger along a 360-tick scripted path under three
//                configurations: each advance, each new stage's fixed fields, and each kept
//                stage's readings, transitions, footprints and obstacles, every tick
// Both are BIT: doubles in the golden are raw bits, 16 hex digits of the uint64.

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/nav/coph_config.h>
#include <cvc/nav/coph_ledger.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

using namespace cvc::nav;
namespace pt = boost::property_tree;
namespace fs = std::filesystem;

namespace {

const fs::path kData = fs::path(__FILE__).parent_path() / "data";

const pt::ptree &golden() {
  static const pt::ptree t = [] {
    pt::ptree r;
    pt::read_json((kData / "coph_golden.json").string(), r);
    return r;
  }();
  return t;
}

std::string str(const pt::ptree &n) { return n.get_value<std::string>(); }

// A double from its 16 hex digits, and back.
double from_hex(const std::string &h) {
  const std::uint64_t u = std::stoull(h, nullptr, 16);
  double d;
  std::memcpy(&d, &u, sizeof d);
  return d;
}

std::string hex(double d) {
  std::uint64_t u;
  std::memcpy(&u, &d, sizeof u);
  char buf[17];
  std::snprintf(buf, sizeof buf, "%016llx", static_cast<unsigned long long>(u));
  return buf;
}

double num(const pt::ptree &n) { return from_hex(str(n)); }

std::vector<std::string> strings(const pt::ptree &arr) {
  std::vector<std::string> v;
  for (const auto &c : arr)
    v.push_back(str(c.second));
  return v;
}

std::vector<double> doubles(const pt::ptree &arr) {
  std::vector<double> v;
  for (const auto &c : arr)
    v.push_back(num(c.second));
  return v;
}

std::vector<std::int64_t> ints(const pt::ptree &arr) {
  std::vector<std::int64_t> v;
  for (const auto &c : arr)
    v.push_back(std::stoll(str(c.second)));
  return v;
}

const pt::ptree &at(const pt::ptree &arr, std::size_t i) {
  auto it = arr.begin();
  std::advance(it, static_cast<std::ptrdiff_t>(i));
  return it->second;
}

// ---------------------------------------------------------------------------
// The records' fields, by Python name, each read and written as the golden encodes it.
// ---------------------------------------------------------------------------

template <class R>
using member = std::variant<std::string R::*, double R::*, int R::*, std::pair<double, double> R::*,
                            std::optional<int> R::*>;

template <class R> using field_table = std::vector<std::pair<std::string, member<R>>>;

const field_table<coph_config> &config_fields() {
  using C = coph_config;
  static const field_table<C> t = {
      {"stage_blend_mode", &C::stage_blend_mode},
      {"route_field_mode", &C::route_field_mode},
      {"controller_mode", &C::controller_mode},
      {"shaping_mode", &C::shaping_mode},
      {"shaping_k", &C::shaping_k},
      {"shaping_ell", &C::shaping_ell},
      {"shaping_eta", &C::shaping_eta},
      {"safety_margin", &C::safety_margin},
      {"obstacle_cap", &C::obstacle_cap},
      {"footprint_cap", &C::footprint_cap},
      {"route_expansion_cap", &C::route_expansion_cap},
      {"navigation_target", &C::navigation_target},
      {"replay_objective", &C::replay_objective},
      {"replay_domain", &C::replay_domain},
      {"replay_horizon_mode", &C::replay_horizon_mode},
      {"replay_update_cap", &C::replay_update_cap},
      {"boundary_max_steps", &C::boundary_max_steps},
      {"boundary_follow_steps", &C::boundary_follow_steps},
      {"dt", &C::dt},
      {"steps", &C::steps},
      {"stage_width", &C::stage_width},
      {"stage_height", &C::stage_height},
      {"overlap", &C::overlap},
      {"previous", &C::previous},
      {"sample_cap", &C::sample_cap},
      {"transition_cap", &C::transition_cap},
      {"anchor_cap", &C::anchor_cap},
      {"rbf_side", &C::rbf_side},
      {"rbf_width", &C::rbf_width},
      {"update_every", &C::update_every},
      {"replay_steps", &C::replay_steps},
      {"solver_steps", &C::solver_steps},
      {"damping_initial", &C::damping_initial},
      {"policy_initial", &C::policy_initial},
      {"damping_bounds", &C::damping_bounds},
      {"policy_bounds", &C::policy_bounds},
      {"damping_trust", &C::damping_trust},
      {"policy_trust", &C::policy_trust},
      {"fit_ridge", &C::fit_ridge},
      {"validation_slack", &C::validation_slack},
      {"sensor_radius", &C::sensor_radius},
      {"force_limit", &C::force_limit},
  };
  return t;
}

const field_table<coph_communication_config> &communication_fields() {
  using C = coph_communication_config;
  static const field_table<C> t = {
      {"ensemble_size", &C::ensemble_size},
      {"anchors", &C::anchors},
      {"received_cap", &C::received_cap},
      {"ttl", &C::ttl},
      {"temporal_window", &C::temporal_window},
      {"covariance_floor", &C::covariance_floor},
      {"covariance_scale", &C::covariance_scale},
      {"byte_budget", &C::byte_budget},
      {"byte_price", &C::byte_price},
      {"processing_price", &C::processing_price},
      {"rate_regularizer", &C::rate_regularizer},
      {"max_rate", &C::max_rate},
      {"delay_ticks", &C::delay_ticks},
      {"prediction_variant", &C::prediction_variant},
  };
  return t;
}

template <class R> const member<R> &field(const field_table<R> &t, const std::string &name) {
  for (const auto &[n, m] : t)
    if (n == name)
      return m;
  throw std::out_of_range("no field " + name);
}

// Set one field from its golden encoding.
template <class R> void set_field(R &r, const member<R> &m, const pt::ptree &v) {
  std::visit(
      [&](auto p) {
        using T = std::decay_t<decltype(r.*p)>;
        if constexpr (std::is_same_v<T, std::string>)
          r.*p = str(v);
        else if constexpr (std::is_same_v<T, double>)
          r.*p = num(v);
        else if constexpr (std::is_same_v<T, int>)
          r.*p = std::stoi(str(v));
        else if constexpr (std::is_same_v<T, std::pair<double, double>>)
          r.*p = {num(at(v, 0)), num(at(v, 1))};
        else if (str(v) == "null")
          r.*p = std::nullopt;
        else
          r.*p = std::stoi(str(v));
      },
      m);
}

// One field in the golden's encoding: a list for a pair, a string otherwise.
template <class R> std::vector<std::string> encode(const R &r, const member<R> &m) {
  return std::visit(
      [&](auto p) -> std::vector<std::string> {
        const auto &x = r.*p;
        using T = std::decay_t<decltype(x)>;
        if constexpr (std::is_same_v<T, std::string>)
          return {x};
        else if constexpr (std::is_same_v<T, double>)
          return {hex(x)};
        else if constexpr (std::is_same_v<T, int>)
          return {std::to_string(x)};
        else if constexpr (std::is_same_v<T, std::pair<double, double>>)
          return {hex(x.first), hex(x.second)};
        else
          return {x ? std::to_string(*x) : "null"};
      },
      m);
}

std::vector<std::string> golden_encoding(const pt::ptree &v) {
  if (v.empty())
    return {str(v)};
  return strings(v);
}

template <class R> R with_overrides(R r, const field_table<R> &t, const pt::ptree &overrides) {
  for (const auto &[name, v] : overrides)
    set_field(r, field(t, name), v);
  return r;
}

template <class R>
void expect_values(const R &r, const field_table<R> &t, const pt::ptree &values,
                   const std::string &where) {
  std::set<std::string> golden_names, table_names;
  for (const auto &[name, v] : values) {
    golden_names.insert(name);
    EXPECT_EQ(encode(r, field(t, name)), golden_encoding(v)) << where << ": " << name;
  }
  for (const auto &[name, m] : t)
    table_names.insert(name);
  EXPECT_EQ(table_names, golden_names) << where << ": the record's fields are Python's";
}

// validate(), as (accepted, Python's message).
template <class R> std::pair<bool, std::string> verdict(R &r) {
  try {
    r.validate();
    return {true, ""};
  } catch (const std::invalid_argument &e) {
    return {false, e.what()};
  }
}

bool ledger_accepts(const coph_config &cfg) {
  try {
    coph_ledger l(cfg, {0.0, 4.0, 0.0, 0.0}, {12.0, 4.0});
    return true;
  } catch (const std::invalid_argument &) {
    return false;
  }
}

} // namespace

// ---------------------------------------------------------------------------
// G1: the configuration
// ---------------------------------------------------------------------------

TEST(NavCophConfig, ProfileIsTheDemos) {
  const pt::ptree &g = golden().get_child("config");
  const coph_config p = coph_profile();
  for (const auto &[name, v] : g.get_child("profile"))
    EXPECT_EQ(encode(p, field(config_fields(), name)), golden_encoding(v)) << name;
  std::vector<std::pair<std::string, std::string>> modes;
  for (const auto &[name, v] : g.get_child("profile_modes"))
    modes.emplace_back(name, str(v));
  EXPECT_EQ(modes, coph_profile_modes());
  EXPECT_TRUE(p.outside_profile().empty());

  const coph_communication_config c = coph_communication_profile();
  for (const auto &[name, v] : g.get_child("communication_profile"))
    EXPECT_EQ(encode(c, field(communication_fields(), name)), golden_encoding(v)) << name;
  ASSERT_EQ(g.get_child("communication_profile_modes").size(), 1u);
  EXPECT_EQ(g.get<std::string>("communication_profile_modes.prediction_variant"),
            c.prediction_variant);
  EXPECT_TRUE(c.outside_profile().empty());
}

TEST(NavCophConfig, Golden) {
  const pt::ptree &cases = golden().get_child("config.config");
  ASSERT_EQ(cases.size(), 74u);
  int refused = 0, outside = 0;
  for (const auto &[_, c] : cases) {
    const std::string name = c.get<std::string>("name");
    coph_config cfg = with_overrides(coph_config{}, config_fields(), c.get_child("overrides"));
    const coph_config before = cfg;
    const auto [accepted, error] = verdict(cfg);
    ASSERT_EQ(accepted, c.get<bool>("accepted")) << name << ": " << error;
    if (!accepted) {
      ++refused;
      EXPECT_EQ(error, c.get<std::string>("error")) << name;
      // A refused config is left as it was (Python never builds one).
      EXPECT_EQ(cfg.replay_update_cap, before.replay_update_cap) << name;
      continue;
    }
    expect_values(cfg, config_fields(), c.get_child("values"), name);
    const std::vector<std::string> out = strings(c.get_child("outside_profile"));
    EXPECT_EQ(cfg.outside_profile(), out) << name;
    // D7: the ledger takes the profile only.
    EXPECT_EQ(ledger_accepts(cfg), out.empty()) << name;
    outside += !out.empty();
  }
  EXPECT_EQ(refused, 48);
  EXPECT_EQ(outside, 9);
}

TEST(NavCophConfig, CommunicationGolden) {
  const pt::ptree &cases = golden().get_child("config.communication");
  ASSERT_EQ(cases.size(), 5u);
  for (const auto &[_, c] : cases) {
    const std::string name = c.get<std::string>("name");
    coph_communication_config comm = with_overrides(
        coph_communication_config{}, communication_fields(), c.get_child("overrides"));
    const auto [accepted, error] = verdict(comm);
    ASSERT_EQ(accepted, c.get<bool>("accepted")) << name << ": " << error;
    if (!accepted) {
      EXPECT_EQ(error, c.get<std::string>("error")) << name;
      continue;
    }
    expect_values(comm, communication_fields(), c.get_child("values"), name);
    const std::vector<std::string> out = strings(c.get_child("outside_profile"));
    EXPECT_EQ(comm.outside_profile(), out) << name;
    bool in_profile = true;
    try {
      require_profile(comm);
    } catch (const std::invalid_argument &) {
      in_profile = false;
    }
    EXPECT_EQ(in_profile, out.empty()) << name;
  }
}

// Upstream's NaN behaviour, checked against Python on 2026-11-12: min() keeps a NaN that comes
// first and skips one that comes later, so NaN scales pass; the interval and box checks refuse it.
TEST(NavCophConfig, NanAsUpstream) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (double coph_config::*f : {&coph_config::dt, &coph_config::stage_width,
                                 &coph_config::sensor_radius, &coph_config::safety_margin}) {
    coph_config c = coph_profile();
    c.*f = nan;
    EXPECT_TRUE(verdict(c).first);
  }
  coph_config c = coph_profile();
  c.overlap = nan;
  EXPECT_EQ(verdict(c).second, "overlap in [0,1); previous must be 0, 2, or 3");
  c = coph_profile();
  c.shaping_k = nan;
  EXPECT_EQ(verdict(c).second, "fixed progress settings required");
  c = coph_profile();
  c.policy_initial.first = nan;
  EXPECT_EQ(verdict(c).second, "progress requires bounded nonnegative shaping");
}

TEST(NavCophConfig, RequireProfileNamesTheFields) {
  coph_config c; // Python's defaults: valid, outside the profile
  c.validate();
  try {
    require_profile(c);
    FAIL() << "the defaults are outside the profile";
  } catch (const std::invalid_argument &e) {
    const std::string m = e.what();
    for (const char *f : {"controller_mode", "shaping_mode", "replay_objective", "replay_domain",
                          "replay_horizon_mode"})
      EXPECT_NE(m.find(f), std::string::npos) << m;
    EXPECT_EQ(m.find("stage_blend_mode"), std::string::npos) << m;
  }
}

// ---------------------------------------------------------------------------
// G4: the stage ledger
// ---------------------------------------------------------------------------

namespace {

coph_shape shape(const pt::ptree &o) {
  const std::vector<double> v = doubles(o);
  if (v.size() == 5)
    return coph_shape::segment(v[0], v[1], v[2], v[3], v[4]);
  EXPECT_EQ(v.size(), 3u);
  return coph_shape::disk(v[0], v[1], v[2]);
}

coph_vec2 vec2(const pt::ptree &arr) {
  const std::vector<double> v = doubles(arr);
  return {v.at(0), v.at(1)};
}

coph_state state(const pt::ptree &arr) {
  const std::vector<double> v = doubles(arr);
  return {v.at(0), v.at(1), v.at(2), v.at(3)};
}

// [dropped, [[start, count], ...]] applied to a list of indices.
std::vector<std::int64_t> apply_delta(std::vector<std::int64_t> list, const pt::ptree &delta) {
  const std::size_t dropped = std::stoul(str(at(delta, 0)));
  EXPECT_LE(dropped, list.size());
  list.erase(list.begin(), list.begin() + static_cast<std::ptrdiff_t>(dropped));
  for (const auto &[_, run] : at(delta, 1)) {
    const std::int64_t start = std::stoll(str(at(run, 0))), count = std::stoll(str(at(run, 1)));
    for (std::int64_t i = 0; i < count; ++i)
      list.push_back(start + i);
  }
  return list;
}

void expect_created(const pt::ptree &c, const coph_stage &s, std::int64_t tick,
                    const std::string &where) {
  EXPECT_EQ(c.get<std::int64_t>("id"), s.id) << where;
  EXPECT_EQ(c.get<std::int64_t>("tick"), tick) << where;
  EXPECT_EQ(strings(c.get_child("center")),
            (std::vector<std::string>{hex(s.center[0]), hex(s.center[1])}))
      << where;
  std::vector<std::string> entry, centers, weights;
  for (double v : s.entry)
    entry.push_back(hex(v));
  for (const coph_vec2 &q : s.centers) {
    centers.push_back(hex(q[0]));
    centers.push_back(hex(q[1]));
  }
  for (double w : s.weights)
    weights.push_back(hex(w));
  EXPECT_EQ(strings(c.get_child("entry")), entry) << where;
  EXPECT_EQ(strings(c.get_child("target")),
            (std::vector<std::string>{hex(s.target[0]), hex(s.target[1])}))
      << where;
  EXPECT_EQ(strings(c.get_child("centers")), centers) << where;
  EXPECT_EQ(strings(c.get_child("weights")), weights) << where;
  EXPECT_EQ(c.get<std::int64_t>("version"), s.version) << where;
}

struct golden_script {
  std::string agent_id;
  coph_vec2 goal{};
  double sensor_radius = 0;
  std::vector<coph_shape> obstacles;
  std::vector<coph_state> states;
  std::vector<std::int64_t> versions;
  std::vector<std::vector<coph_vec2>> readings;
};

const golden_script &script() {
  static const golden_script s = [] {
    const pt::ptree &g = golden().get_child("stages");
    golden_script r;
    r.agent_id = g.get<std::string>("agent_id");
    r.goal = vec2(g.get_child("goal"));
    r.sensor_radius = num(g.get_child("sensor_radius"));
    for (const auto &[_, o] : g.get_child("obstacles"))
      r.obstacles.push_back(shape(o));
    for (const auto &[_, t] : g.get_child("script")) {
      r.states.push_back(state(t.get_child("state")));
      r.versions.push_back(t.get<std::int64_t>("version"));
      std::vector<coph_vec2> q;
      for (const auto &[key, xy] : t.get_child("readings"))
        q.push_back(vec2(xy));
      r.readings.push_back(std::move(q));
    }
    return r;
  }();
  return s;
}

// The golden's index of a stored reading: 5 * tick + j for uid "<agent>:<tick>:<j>".
std::int64_t reading_index(const std::string &uid, const std::string &agent) {
  const std::string rest = uid.substr(agent.size() + 1);
  const std::size_t colon = rest.find(':');
  const std::int64_t tick = std::stoll(rest.substr(0, colon)),
                     j = std::stoll(rest.substr(colon + 1));
  return static_cast<std::int64_t>(script().readings.at(static_cast<std::size_t>(tick)).size()) *
             tick +
         j;
}

struct ledger_counts {
  int advances = 0;
  int axis_moves[2][2] = {{0, 0}, {0, 0}}; // [axis][negative, positive]
  std::size_t max_readings = 0, max_transitions = 0, max_footprints = 0;
};

ledger_counts replay_variant(const pt::ptree &variant) {
  const golden_script &sc = script();
  const std::string name = variant.get<std::string>("name");
  coph_config cfg = coph_profile();
  cfg.steps = static_cast<int>(sc.states.size());
  cfg = with_overrides(cfg, config_fields(), variant.get_child("overrides"));
  coph_ledger ledger(cfg, sc.states.at(0), sc.goal);
  const pt::ptree &created = variant.get_child("created");
  const pt::ptree &ticks = variant.get_child("ticks");
  EXPECT_EQ(ticks.size(), sc.states.size()) << name;
  expect_created(at(created, 0), ledger.current(), 0, name + " stage 0");
  std::size_t next_created = 1;

  std::map<std::uint64_t, std::int64_t> footprint_tick; // footprint time bits -> its tick
  std::map<std::int64_t, std::array<std::vector<std::int64_t>, 3>> held;
  ledger_counts counts;
  std::int64_t tick = 0;
  for (const auto &[_, t] : ticks) {
    const std::string where = name + " tick " + std::to_string(tick);
    EXPECT_EQ(t.get<std::int64_t>("tick"), tick) << where;
    const std::size_t k = static_cast<std::size_t>(tick);
    const coph_state &x = sc.states[k];
    coph_observation obs;
    obs.time = static_cast<double>(tick) * cfg.dt;
    for (std::size_t j = 0; j < sc.readings[k].size(); ++j)
      obs.readings.push_back(
          {sc.agent_id + ":" + std::to_string(tick) + ":" + std::to_string(j), obs.time,
           sc.readings[k][j],
           (0.05 + 0.01 * static_cast<double>(j)) + 0.001 * static_cast<double>(tick % 7)});
    obs.sensor_center = coph_vec2{x[0], x[1]};
    obs.sensor_radius = sc.sensor_radius;
    std::uint64_t time_bits;
    std::memcpy(&time_bits, &obs.time, sizeof time_bits);
    footprint_tick[time_bits] = tick;

    bool advanced = false;
    coph_vec2 from_center = ledger.current().center;
    const std::int64_t from_id = ledger.current().id;
    if (tick == 0) {
      ledger.observe(obs);
    } else {
      const coph_transition tr{tick - 1,   obs.time, sc.states[k - 1],
                               {0.0, 0.0}, x,        sc.versions[k - 1]};
      ledger.observe(obs, &tr);
      advanced = ledger.advance(x, sc.versions[k]);
    }
    std::vector<coph_shape> store;
    for (std::int64_t i : ints(t.get_child("geometry")))
      store.push_back(sc.obstacles.at(static_cast<std::size_t>(i)));
    ledger.set_geometry(store);

    // The advance.
    const pt::ptree &adv = t.get_child("advanced");
    EXPECT_EQ(advanced, !adv.empty()) << where; // null has no children
    if (advanced && !adv.empty()) {
      const coph_vec2 &c = ledger.current().center;
      const std::size_t axis = c[1] != from_center[1] ? 1 : 0;
      EXPECT_NE(c[axis], from_center[axis]) << where;
      EXPECT_EQ(c[1 - axis], from_center[1 - axis]) << where;
      EXPECT_EQ(adv.get<std::size_t>("axis"), axis) << where;
      EXPECT_EQ(adv.get<std::int64_t>("id"), ledger.current().id) << where;
      EXPECT_EQ(adv.get<std::int64_t>("from_id"), from_id) << where;
      ++counts.advances;
      ++counts.axis_moves[axis][c[axis] > from_center[axis] ? 1 : 0];
      if (next_created < created.size())
        expect_created(at(created, next_created++), ledger.current(), tick, where + " new stage");
    }

    // The kept stages.
    const pt::ptree &stages = t.get_child("stages");
    EXPECT_EQ(stages.size(), ledger.stages().size()) << where;
    std::map<std::int64_t, std::array<std::vector<std::int64_t>, 3>> now_held;
    std::size_t i = 0;
    for (const auto &[key, gs] : stages) {
      if (i >= ledger.stages().size())
        break;
      const coph_stage &s = ledger.stages()[i++];
      const std::int64_t id = std::stoll(str(at(gs, 0)));
      const std::string sw = where + " stage " + std::to_string(id);
      EXPECT_EQ(s.id, id) << sw;
      std::array<std::vector<std::int64_t>, 3> expected, actual;
      const auto prev = held.find(id);
      for (std::size_t list = 0; list < 3; ++list)
        expected[list] =
            apply_delta(prev == held.end() ? std::vector<std::int64_t>{} : prev->second[list],
                        at(gs, list + 1));
      for (const coph_reading &r : s.readings)
        actual[0].push_back(reading_index(r.uid, sc.agent_id));
      for (const coph_transition &tr : s.transitions)
        actual[1].push_back(tr.uid);
      for (const coph_footprint &f : s.footprints) {
        std::uint64_t b;
        std::memcpy(&b, &f.time, sizeof b);
        const auto ft = footprint_tick.find(b);
        actual[2].push_back(ft == footprint_tick.end() ? -1 : ft->second);
        EXPECT_EQ(f.radius, sc.sensor_radius) << sw;
      }
      EXPECT_EQ(actual[0], expected[0]) << sw << " readings";
      EXPECT_EQ(actual[1], expected[1]) << sw << " transitions";
      EXPECT_EQ(actual[2], expected[2]) << sw << " footprints";
      std::vector<std::int64_t> obstacles;
      for (const coph_shape &o : s.obstacles)
        for (std::size_t m = 0; m < sc.obstacles.size(); ++m)
          if (sc.obstacles[m] == o)
            obstacles.push_back(static_cast<std::int64_t>(m));
      EXPECT_EQ(obstacles, ints(at(gs, 4))) << sw << " obstacles";
      counts.max_readings = std::max(counts.max_readings, s.readings.size());
      counts.max_transitions = std::max(counts.max_transitions, s.transitions.size());
      counts.max_footprints = std::max(counts.max_footprints, s.footprints.size());
      now_held[id] = expected;
    }
    held = std::move(now_held);
    if (::testing::Test::HasFatalFailure())
      break;
    ++tick;
  }
  EXPECT_EQ(next_created, created.size()) << name << ": every new stage was made";
  return counts;
}

const pt::ptree &variant(const std::string &name) {
  for (const auto &[_, v] : golden().get_child("stages.variants"))
    if (v.get<std::string>("name") == name)
      return v;
  throw std::out_of_range("no variant " + name);
}

} // namespace

TEST(NavCophLedger, GoldenProfile) {
  const ledger_counts c = replay_variant(variant("profile"));
  EXPECT_GT(c.advances, 0);
  for (int axis = 0; axis < 2; ++axis)
    for (int dir = 0; dir < 2; ++dir)
      EXPECT_GT(c.axis_moves[axis][dir], 0) << "axis " << axis << " direction " << dir;
  // The profile's reading and footprint caps fill. A stage advances long before it holds 120
  // transitions, so that cap fills only in the small-caps variant.
  const coph_config p = coph_profile();
  EXPECT_EQ(c.max_readings, static_cast<std::size_t>(p.sample_cap));
  EXPECT_GT(c.max_transitions, 0u);
  EXPECT_LT(c.max_transitions, static_cast<std::size_t>(p.transition_cap));
  EXPECT_EQ(c.max_footprints, static_cast<std::size_t>(p.footprint_cap));
}

TEST(NavCophLedger, GoldenSmallCapsPrevious3) {
  const ledger_counts c = replay_variant(variant("small_caps_previous_3"));
  EXPECT_GT(c.advances, 0);
  EXPECT_EQ(c.max_readings, 30u);
  EXPECT_EQ(c.max_transitions, 15u);
  EXPECT_EQ(c.max_footprints, 6u);
}

TEST(NavCophLedger, GoldenPrevious0NoOverlap) {
  const ledger_counts c = replay_variant(variant("previous_0_no_overlap"));
  EXPECT_GT(c.advances, 0);
}

TEST(NavCophLedger, GoldenHasOnlyThoseVariants) {
  std::vector<std::string> names;
  for (const auto &[_, v] : golden().get_child("stages.variants"))
    names.push_back(v.get<std::string>("name"));
  EXPECT_EQ(names, (std::vector<std::string>{"profile", "small_caps_previous_3",
                                             "previous_0_no_overlap"}));
}

// ---------------------------------------------------------------------------
// C++ only
// ---------------------------------------------------------------------------

TEST(NavCophLedger, RefusesConfigsOutsideTheProfile) {
  EXPECT_THROW(coph_ledger(coph_config{}, {0, 0, 0, 0}, {1, 0}), std::invalid_argument);
  coph_config c = coph_profile();
  c.overlap = 1.0;
  try {
    coph_ledger l(c, {0, 0, 0, 0}, {1, 0});
    FAIL() << "overlap 1 is refused";
  } catch (const std::invalid_argument &e) {
    EXPECT_STREQ(e.what(), "overlap in [0,1); previous must be 0, 2, or 3");
  }
  // replay_update_cap is filled when the ledger validates.
  c = coph_profile();
  c.replay_update_cap.reset();
  EXPECT_EQ(coph_ledger(c, {0, 0, 0, 0}, {1, 0}).config().replay_update_cap, 1024);
}

TEST(NavCophLedger, NoSensorCentreNoFootprint) {
  coph_ledger l(coph_profile(), {0.0, 4.0, 0.0, 0.0}, {12.0, 4.0});
  coph_observation obs;
  obs.time = 0.0;
  obs.readings.push_back({"0:0:1", 0.0, {0.5, 4.0}, 0.1});
  obs.readings.push_back({"0:0:2", 0.0, {9.0, 4.0}, 0.1}); // outside the stage
  obs.readings.push_back({"0:0:1", 0.0, {0.5, 4.0}, 0.2}); // the same uid twice in one observation
  l.observe(obs);
  EXPECT_TRUE(l.current().footprints.empty());
  ASSERT_EQ(l.current().readings.size(), 2u); // as Python: uids held before this observation
  l.observe(obs);
  EXPECT_EQ(l.current().readings.size(), 2u);
}

TEST(NavCophLedger, AnAdvancePoolsReadingsAndARepeatedUidTakesItsLastReading) {
  coph_ledger l(coph_profile(), {0.0, 4.0, 0.0, 0.0}, {12.0, 4.0});
  coph_observation obs;
  obs.readings.push_back({"0:0:1", 0.0, {-1.4, 4.0}, 0.1}); // stage 0 only
  l.observe(obs);
  ASSERT_TRUE(l.advance({1.2, 4.0, 1.0, 0.0}, 7)); // east, by 3 * (1 - 0.3)
  EXPECT_EQ(l.current().id, 1);
  EXPECT_EQ(l.current().version, 7);
  EXPECT_EQ(l.current().center[0], 3.0 * (1 - 0.3));
  EXPECT_TRUE(l.current().readings.empty());        // -1.4 is outside stage 1
  EXPECT_FALSE(l.advance({1.2, 4.0, 1.0, 0.0}, 8)); // inside stage 1's inner box
  // The same uid again, now inside stage 1: stage 0 already holds it, stage 1 takes it.
  obs.readings = {{"0:0:1", 0.12, {1.0, 4.0}, 0.2}};
  l.observe(obs);
  ASSERT_EQ(l.stages()[0].readings.size(), 1u);
  ASSERT_EQ(l.stages()[1].readings.size(), 1u);
  // Back west: stage 2 sits where stage 0 was and contains both. Python's
  // {r.uid: r for r in ...}.values() keeps the first place and the last reading.
  ASSERT_TRUE(l.advance({0.9, 4.0, -1.0, 0.0}, 9));
  EXPECT_EQ(l.current().center[0], 0.0);
  ASSERT_EQ(l.current().readings.size(), 1u);
  EXPECT_EQ(l.current().readings[0].value, 0.2);
  EXPECT_EQ(l.current().readings[0].q[0], 1.0);
  EXPECT_TRUE(l.current().transitions.empty());
}
