/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// C4 (docs/roadmap/NAV-COPH-ROADMAP.md §4.4) against GRL-SNAM's golden, data/coph_golden.json:
// G9 (exports, with the bootstraps' indices injected), the packets' byte count and float rule,
// and G10 (a receiver assimilating 37 packets, its Gauss-Newton field fit and its update)
// (grl_snam.tools.coph_golden_packets; its packet_rules text states the rules). Senders and the
// receiver are rebuilt from learner.snapshots as nav_coph_learner_test does. Values ("out") are
// compared at the section's tier, rtol 1e-9 and atol 1e-12; everything else exactly, the byte
// counts included. Then the C++-only checks: the channel's delay, drop, duplicate and range
// statistics, production draws, and the JSON writer's escapes.

#include <algorithm>
#include <bit>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/coph_channel.h>
#include <cvc/nav/coph_exchange.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <string>
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

const pt::ptree &learner() { return golden().get_child("learner"); }
const pt::ptree &packets() { return golden().get_child("packets"); }

std::string str(const pt::ptree &n) { return n.get_value<std::string>(); }
bool is_null(const pt::ptree &n) { return n.empty() && str(n) == "null"; }
double from_hex(const std::string &h) {
  return std::bit_cast<double>(static_cast<std::uint64_t>(std::stoull(h, nullptr, 16)));
}
double num(const pt::ptree &n) { return from_hex(str(n)); }
long long integer(const pt::ptree &n) { return std::stoll(str(n)); }
bool boolean(const pt::ptree &n) { return str(n) == "true"; }

std::vector<const pt::ptree *> items(const pt::ptree &arr) {
  std::vector<const pt::ptree *> v;
  for (const auto &c : arr)
    v.push_back(&c.second);
  return v;
}
std::vector<double> doubles(const pt::ptree &arr) {
  std::vector<double> v;
  for (const auto &c : arr)
    v.push_back(num(c.second));
  return v;
}
std::vector<std::string> strings(const pt::ptree &arr) {
  std::vector<std::string> v;
  for (const auto &c : arr)
    v.push_back(str(c.second));
  return v;
}
coph_vec2 vec2(const pt::ptree &arr) {
  const std::vector<double> v = doubles(arr);
  return {v.at(0), v.at(1)};
}
coph_state state4(const pt::ptree &arr) {
  const std::vector<double> v = doubles(arr);
  return {v.at(0), v.at(1), v.at(2), v.at(3)};
}
std::vector<long long> expand(const pt::ptree &runs) {
  std::vector<long long> out;
  for (const pt::ptree *r : items(runs)) {
    const auto p = items(*r);
    for (long long i = integer(*p[0]); i < integer(*p[0]) + integer(*p[1]); ++i)
      out.push_back(i);
  }
  return out;
}
coph_shape shape(const pt::ptree &arr) {
  const std::vector<double> v = doubles(arr);
  return v.size() == 5 ? coph_shape::segment(v[0], v[1], v[2], v[3], v[4])
                       : coph_shape::disk(v[0], v[1], v[2]);
}

// The FLOAT64 comparison, with a running account of how close each section came.
struct tier_account {
  double rtol = 1e-9, atol = 1e-12;
  long long values = 0, exact = 0;
  double worst = 0; // the largest |mine - golden| / allowance

  void check(double mine, double theirs, const std::string &what, double scale = 0) {
    const double allowance = rtol * std::max(std::fabs(theirs), scale) + atol;
    const double diff = std::fabs(mine - theirs);
    ++values;
    exact += mine == theirs;
    worst = std::max(worst, diff / allowance);
    EXPECT_LE(diff, allowance) << what << ": " << mine << " vs " << theirs;
  }
  void check(const std::vector<double> &mine, const pt::ptree &theirs, const std::string &what,
             bool scaled = false) {
    check(mine, doubles(theirs), what, scaled);
  }
  void check(const std::vector<double> &mine, const std::vector<double> &t, const std::string &what,
             bool scaled = false) {
    ASSERT_EQ(mine.size(), t.size()) << what;
    double scale = 0;
    if (scaled) // a Gram matrix: its small entries carry its large ones' rounding
      for (double x : t)
        scale = std::max(scale, std::fabs(x));
    for (std::size_t i = 0; i < t.size(); ++i)
      check(mine[i], t[i], what + "[" + std::to_string(i) + "]", scale);
  }
  void report(const char *name) const {
    std::printf("%s: %lld of %lld values bit for bit; the worst is %.3g of the tier's allowance\n",
                name, exact, values, worst);
  }
};

tier_account account() {
  tier_account a;
  a.rtol = packets().get<double>("tier.rtol");
  a.atol = packets().get<double>("tier.atol");
  return a;
}

coph_config profile() {
  coph_config c;
  c.controller_mode = "routed";
  c.shaping_mode = "progress";
  c.replay_objective = "mission";
  c.replay_domain = "retained";
  c.replay_horizon_mode = "supported_boundary";
  c.replay_update_cap = 1024;
  c.steps = static_cast<int>(learner().get<long long>("steps"));
  c.validate();
  return c;
}

const std::vector<coph_shape> &obstacles() {
  static const std::vector<coph_shape> v = [] {
    std::vector<coph_shape> out;
    for (const pt::ptree *o : items(learner().get_child("world.obstacles")))
      out.push_back(shape(*o));
    return out;
  }();
  return v;
}

std::size_t lattice() { return learner().get_child("world.lattice").size(); }

// Script reading i's position (i = 21 tick + j).
coph_vec2 reading_xy(long long i) {
  static const std::vector<const pt::ptree *> script = items(learner().get_child("script"));
  const long long n = static_cast<long long>(lattice());
  const std::vector<double> r = doubles(*items(script[i / n]->get_child("readings"))[i % n]);
  return {r[0], r[1]};
}

std::string reading_uid(const std::string &agent, long long i) {
  const long long n = static_cast<long long>(lattice());
  return agent + ":" + std::to_string(i / n) + ":" + std::to_string(i % n);
}

std::vector<coph_geometry_report> reports(const pt::ptree &entries) {
  std::vector<coph_geometry_report> out;
  for (const pt::ptree *e : items(entries)) {
    const auto v = items(*e);
    out.push_back(
        {obstacles().at(integer(*v[0])), vec2(*v[1]), num(*v[2]), num(*v[3]), str(*v[4])});
  }
  return out;
}

// packet_rules' packet: the file's compact form back into a coph_packet.
coph_packet decode(const pt::ptree &d) {
  coph_packet p;
  p.schema = integer(d.get_child("schema"));
  p.sender = str(d.get_child("sender"));
  p.recipient = str(d.get_child("recipient"));
  p.identity = str(d.get_child("identity"));
  p.timestamp = num(d.get_child("timestamp"));
  p.expires = num(d.get_child("expires"));
  p.version = integer(d.get_child("version"));
  p.convention = str(d.get_child("convention"));
  p.q = vec2(d.get_child("q"));
  p.p = vec2(d.get_child("p"));
  p.v = vec2(d.get_child("v"));
  p.H = num(d.get_child("H"));
  p.V_task = num(d.get_child("V_task"));
  p.goal = vec2(d.get_child("goal"));
  const std::vector<double> b = doubles(d.get_child("bounds"));
  p.bounds = {b.at(0), b.at(1), b.at(2), b.at(3)};
  for (const pt::ptree *a : items(d.get_child("anchors"))) {
    const std::vector<double> v = doubles(*a);
    p.anchors.push_back({v.at(0), v.at(1), v.at(2), v.at(3), v.at(4)});
  }
  for (const pt::ptree *r : items(d.get_child("covariance")))
    p.covariance.push_back(doubles(*r));
  const std::string agent = str(d.get_child("sources.agent"));
  for (long long i : expand(d.get_child("sources.readings")))
    p.sources.push_back(reading_uid(agent, i));
  std::sort(p.sources.begin(), p.sources.end());
  for (const std::string &x : strings(d.get_child("sources.extra")))
    p.sources.push_back(x);
  p.stage = integer(d.get_child("stage"));
  for (long long i : expand(d.get_child("support_points")))
    p.support_points.push_back(reading_xy(i));
  p.uncertainty = str(d.get_child("uncertainty"));
  p.acknowledged_sources = strings(d.get_child("acknowledged_sources"));
  p.geometry_schema = integer(d.get_child("geometry_schema"));
  p.geometry = reports(d.get_child("geometry"));
  return p;
}

// learner_rules' rebuild, as agent `agent`: the snapshot's state from the script and its inputs,
// with CooperativeLearner's identity, clock and communication (the profile's).
coph_learner_state rebuild(const pt::ptree &snap, const std::string &agent) {
  const pt::ptree &world = learner().get_child("world");
  const auto script = items(learner().get_child("script"));
  const double radius = num(world.get_child("sensor_radius"));
  const long long n = static_cast<long long>(lattice());
  coph_learner_state s;
  s.cfg = profile();
  s.goal = vec2(world.get_child("goal"));
  s.network = coph_crossing_network();
  auto tick_state = [&](long long k) { return state4(script[k]->get_child("state")); };
  for (const pt::ptree *st : items(snap.get_child("stages"))) {
    coph_stage stage = make_coph_stage(integer(st->get_child("id")), vec2(st->get_child("center")),
                                       state4(st->get_child("entry")), s.goal, s.cfg);
    stage.weights = doubles(st->get_child("weights"));
    stage.version = integer(st->get_child("version"));
    for (long long i : expand(st->get_child("readings"))) {
      const std::vector<double> r = doubles(*items(script[i / n]->get_child("readings"))[i % n]);
      stage.readings.push_back({reading_uid(agent, i), (i / n) * s.cfg.dt, {r[0], r[1]}, r[2]});
    }
    for (long long k : expand(st->get_child("transitions"))) {
      coph_transition t;
      t.uid = k;
      t.time = (k + 1) * s.cfg.dt;
      t.x = tick_state(k);
      t.u = vec2(script[k]->get_child("u"));
      t.after = tick_state(k + 1);
      t.version = integer(script[k]->get_child("version"));
      stage.transitions.push_back(t);
    }
    for (long long k : expand(st->get_child("footprints"))) {
      const coph_state x = tick_state(k);
      stage.footprints.push_back({x[0], x[1], radius, k * s.cfg.dt});
    }
    for (const pt::ptree *o : items(st->get_child("obstacles")))
      stage.obstacles.push_back(obstacles().at(integer(*o)));
    s.stages.push_back(std::move(stage));
  }
  for (const pt::ptree *o : items(snap.get_child("geometry")))
    s.geometry.push_back(obstacles().at(integer(*o)));
  for (const pt::ptree *o : items(snap.get_child("reported_geometry")))
    s.reported_geometry.push_back(obstacles().at(integer(*o)));
  for (const pt::ptree *a : items(snap.get_child("terrain_anchors"))) {
    const std::vector<double> v = doubles(*a);
    s.terrain_anchors.push_back({v[0], v[1], v[2], v[3], v[4]});
  }
  s.exchange.local_model_anchors = s.terrain_anchors; // nothing was received
  s.damping = num(snap.get_child("damping"));
  s.policy = vec2(snap.get_child("policy"));
  s.version = integer(snap.get_child("version"));
  s.agent_id = agent;
  s.now = num(snap.get_child("now"));
  s.communication = coph_communication_profile();
  return s;
}

const pt::ptree &snapshot(long long k) { return *items(learner().get_child("snapshots")).at(k); }

std::vector<double> flat(const std::vector<std::vector<double>> &m) {
  std::vector<double> out;
  for (const auto &row : m)
    out.insert(out.end(), row.begin(), row.end());
  return out;
}

std::vector<double> anchor_values(const std::vector<coph_anchor> &a) {
  std::vector<double> out;
  for (const coph_anchor &x : a) {
    out.push_back(x.value);
    out.push_back(x.gx);
    out.push_back(x.gy);
  }
  return out;
}

// A matrix of hex doubles (anchors' values, a covariance), row after row.
std::vector<double> golden_anchor_values(const pt::ptree &rows) {
  std::vector<double> out;
  for (const pt::ptree *a : items(rows))
    for (double v : doubles(*a))
      out.push_back(v);
  return out;
}

coph_draws draws_of(const pt::ptree &d) {
  coph_draws out;
  for (const pt::ptree *member : items(d)) {
    std::vector<int> m;
    for (const pt::ptree *i : items(*member))
      m.push_back(static_cast<int>(integer(*i)));
    out.push_back(m);
  }
  return out;
}

} // namespace

// ---------------------------------------------------------------------------
// The float rule and the byte count
// ---------------------------------------------------------------------------

TEST(CophPacketGolden, ReprIsPythons) {
  const auto cases = items(packets().get_child("repr"));
  ASSERT_GE(cases.size(), 40u);
  for (const pt::ptree *c : cases) {
    const auto v = items(*c);
    EXPECT_EQ(coph_repr(num(*v[0])), str(*v[1])) << str(*v[0]);
  }
}

TEST(CophPacketGolden, BytesIsTheWirePacketsJsonLength) {
  const auto exports = items(packets().get_child("exports"));
  ASSERT_EQ(exports.size(), 3u);
  for (const pt::ptree *e : exports)
    EXPECT_EQ(decode(e->get_child("packet")).bytes(),
              static_cast<std::size_t>(integer(e->get_child("bytes"))));
  // Export 0's JSON text, character for character.
  EXPECT_EQ(decode(exports[0]->get_child("packet")).json(), str(packets().get_child("json")));
}

// ---------------------------------------------------------------------------
// G9: exports
// ---------------------------------------------------------------------------

TEST(CophPacketGolden, Exports) {
  tier_account acc = account();
  for (const pt::ptree *e : items(packets().get_child("exports"))) {
    const long long k = integer(e->get_child("snapshot"));
    const pt::ptree &snap = snapshot(k);
    const std::string where = "export " + std::to_string(k);
    coph_learner_state s = rebuild(snap, "0");
    s.exchange.local_geometry = reports(e->get_child("geometry"));
    const auto d = items(e->get_child("draws"));
    ASSERT_EQ(d.size(), 2u);
    coph_export_trace t;
    const std::optional<coph_packet> got =
        coph_export(s, state4(snap.get_child("x")), str(e->get_child("recipient")),
                    {draws_of(*d[0]), draws_of(*d[1])}, &t);
    ASSERT_TRUE(got.has_value()) << where;
    const coph_packet &p = *got;
    const coph_packet wire = decode(e->get_child("packet"));

    // Decisions.
    const auto blocks = items(e->get_child("blocks"));
    ASSERT_EQ(t.cooperative.blocks.size(), blocks.size()) << where;
    for (std::size_t b = 0; b < blocks.size(); ++b) {
      const auto v = items(*blocks[b]);
      EXPECT_EQ(t.cooperative.blocks[b].first, integer(*v[0])) << where;
      EXPECT_EQ(t.cooperative.blocks[b].second, integer(*v[1])) << where;
    }
    const auto candidates = items(e->get_child("candidates"));
    ASSERT_EQ(t.candidates.size(), candidates.size()) << where;
    for (std::size_t c = 0; c < candidates.size(); ++c) {
      const auto v = items(*candidates[c]);
      EXPECT_EQ(t.candidates[c].first, vec2(*v[0])) << where;
      EXPECT_EQ(t.candidates[c].second, boolean(*v[1])) << where;
    }
    const auto picked = items(e->get_child("picked"));
    ASSERT_EQ(t.picked.size(), picked.size()) << where;
    for (std::size_t c = 0; c < picked.size(); ++c)
      EXPECT_EQ(t.picked[c]->uid, reading_uid("0", integer(*picked[c]))) << where;
    EXPECT_EQ(t.path, str(e->get_child("path"))) << where;
    EXPECT_EQ(p.schema, wire.schema);
    EXPECT_EQ(p.sender, wire.sender);
    EXPECT_EQ(p.recipient, wire.recipient);
    EXPECT_EQ(p.identity, wire.identity) << where;
    EXPECT_EQ(p.timestamp, wire.timestamp);
    EXPECT_EQ(p.expires, wire.expires);
    EXPECT_EQ(p.version, wire.version);
    EXPECT_EQ(p.convention, wire.convention);
    EXPECT_EQ(p.q, wire.q);
    EXPECT_EQ(p.p, wire.p);
    EXPECT_EQ(p.v, wire.v);
    EXPECT_EQ(p.goal, wire.goal);
    EXPECT_EQ(p.bounds, wire.bounds) << where;
    EXPECT_EQ(p.sources, wire.sources) << where;
    EXPECT_EQ(p.stage, wire.stage);
    EXPECT_EQ(p.support_points, wire.support_points) << where;
    EXPECT_EQ(p.uncertainty, wire.uncertainty);
    EXPECT_EQ(p.acknowledged_sources, wire.acknowledged_sources);
    EXPECT_EQ(p.geometry_schema, wire.geometry_schema);
    ASSERT_EQ(p.geometry.size(), wire.geometry.size());
    for (std::size_t g = 0; g < p.geometry.size(); ++g) {
      EXPECT_EQ(p.geometry[g].shape, wire.geometry[g].shape);
      EXPECT_EQ(p.geometry[g].sensor, wire.geometry[g].sensor);
      EXPECT_EQ(p.geometry[g].source, wire.geometry[g].source);
    }
    ASSERT_EQ(p.anchors.size(), wire.anchors.size()) << where;
    for (std::size_t a = 0; a < p.anchors.size(); ++a) {
      EXPECT_EQ(p.anchors[a].x, wire.anchors[a].x) << where;
      EXPECT_EQ(p.anchors[a].y, wire.anchors[a].y) << where;
    }

    // Values.
    const pt::ptree &out = e->get_child("out");
    acc.check(t.cooperative.mean, out.get_child("mean"), where + " mean");
    const pt::ptree *sets[] = {&out.get_child("cooperative"), &out.get_child("terrain")};
    const coph_bootstrap *boots[] = {&t.cooperative, &t.terrain};
    const std::vector<coph_anchor> *anchors[] = {&t.cooperative_anchors, &t.terrain_anchors};
    const std::vector<std::vector<double>> *covs[] = {&t.cooperative_covariance,
                                                      &t.terrain_covariance};
    for (int b = 0; b < 2; ++b) {
      const auto ens = items(sets[b]->get_child("ensemble"));
      ASSERT_EQ(boots[b]->ensemble.size(), ens.size()) << where;
      for (std::size_t m = 0; m < ens.size(); ++m)
        acc.check(boots[b]->ensemble[m], *ens[m], where + " ensemble");
      acc.check(anchor_values(*anchors[b]), golden_anchor_values(sets[b]->get_child("anchors")),
                where + " anchors");
      acc.check(flat(*covs[b]), golden_anchor_values(sets[b]->get_child("covariance")),
                where + " covariance");
    }
    acc.check(p.H, num(out.get_child("H")), where + " H");
    acc.check(p.V_task, num(out.get_child("V_task")), where + " V_task");
    // The wire packet is the export on the 2^-24 grid.
    const double grid = std::ldexp(1.0, -24);
    for (std::size_t a = 0; a < p.anchors.size(); ++a)
      EXPECT_LE(std::fabs(p.anchors[a].value - wire.anchors[a].value), grid) << where;
  }
  acc.report("G9");
}

// ---------------------------------------------------------------------------
// G10: assimilate
// ---------------------------------------------------------------------------

namespace {

void check_fit(tier_account &acc, const coph_learner_state &s, const pt::ptree &fit,
               const std::string &where) {
  const std::vector<coph_message_fit> mine = coph_fit_messages(s, coph_field_fit(s));
  const auto stages = items(fit.get_child("stages"));
  ASSERT_EQ(mine.size(), stages.size()) << where;
  for (std::size_t k = 0; k < stages.size(); ++k) {
    const pt::ptree &st = *stages[k];
    const auto messages = items(st.get_child("messages"));
    ASSERT_EQ(mine[k].messages.size(), messages.size()) << where << " stage " << k;
    for (std::size_t m = 0; m < messages.size(); ++m) {
      const auto v = items(*messages[m]);
      EXPECT_EQ(mine[k].messages[m].first, str(*v[0])) << where;
      std::vector<int> ids;
      for (const pt::ptree *i : items(*v[1]))
        ids.push_back(static_cast<int>(integer(*i)));
      EXPECT_EQ(mine[k].messages[m].second, ids) << where;
    }
    EXPECT_EQ(mine[k].rows, integer(st.get_child("rows"))) << where;
    const pt::ptree &out = st.get_child("out");
    if (is_null(out.get_child("delta"))) {
      EXPECT_TRUE(mine[k].delta.empty()) << where;
    } else {
      acc.check(mine[k].gram, out.get_child("gram"), where + " gram", true);
      acc.check(mine[k].rhs, out.get_child("rhs"), where + " rhs");
      acc.check(mine[k].delta, out.get_child("delta"), where + " delta");
    }
    acc.check(mine[k].weights, out.get_child("weights"), where + " weights");
  }
}

} // namespace

TEST(CophPacketGolden, Assimilate) {
  tier_account acc = account();
  const pt::ptree &a = packets().get_child("assimilate");
  const pt::ptree &receiver = a.get_child("receiver");
  const pt::ptree &snap = snapshot(integer(receiver.get_child("snapshot")));
  coph_learner_state s = rebuild(snap, str(receiver.get_child("agent_id")));
  s.exchange.local_geometry = reports(receiver.get_child("geometry"));
  const auto fits = items(a.get_child("field_fit"));
  std::size_t next_fit = 0;
  const auto cases = items(a.get_child("cases"));
  ASSERT_EQ(cases.size(), 37u);
  for (const pt::ptree *c : cases) {
    const std::string name = str(c->get_child("name"));
    const coph_packet_event ev = coph_assimilate(s, decode(c->get_child("packet")));
    EXPECT_EQ(ev.reason, str(c->get_child("reason"))) << name;
    const pt::ptree &scope = c->get_child("scope");
    EXPECT_EQ(ev.scope, is_null(scope) ? std::string() : str(scope)) << name;
    std::vector<std::string> held;
    for (const auto &r : s.exchange.received)
      held.push_back(r.first);
    EXPECT_EQ(held, strings(c->get_child("received"))) << name;
    if (next_fit < fits.size() && str(fits[next_fit]->get_child("after")) == name)
      check_fit(acc, s, *fits[next_fit++], "field fit after " + name);
  }
  EXPECT_EQ(next_fit, fits.size());

  // What assimilation left behind.
  const pt::ptree &state = a.get_child("state");
  std::vector<std::string> seq_senders;
  for (const pt::ptree *q : items(state.get_child("last_sequences"))) {
    const auto v = items(*q);
    auto it = std::find_if(s.exchange.last_sequences.begin(), s.exchange.last_sequences.end(),
                           [&](const auto &x) { return x.sender == str(*v[0]); });
    ASSERT_NE(it, s.exchange.last_sequences.end()) << str(*v[0]);
    EXPECT_EQ(it->timestamp, num(*v[1]));
    EXPECT_EQ(it->stage, integer(*v[2]));
    seq_senders.push_back(str(*v[0]));
  }
  std::vector<std::string> mine_senders;
  for (const auto &q : s.exchange.last_sequences)
    mine_senders.push_back(q.sender);
  EXPECT_EQ(mine_senders, seq_senders);
  const auto terrain = items(state.get_child("remote_terrain"));
  ASSERT_EQ(s.exchange.remote_terrain.size(), terrain.size());
  for (std::size_t i = 0; i < terrain.size(); ++i) {
    const auto v = items(*terrain[i]);
    EXPECT_EQ(s.exchange.remote_terrain[i].sender, str(*v[0]));
    EXPECT_EQ(s.exchange.remote_terrain[i].stage, integer(*v[1]));
  }
  const auto cells = items(state.get_child("remote_cells"));
  ASSERT_EQ(s.exchange.remote_cells.size(), cells.size());
  for (std::size_t i = 0; i < cells.size(); ++i) {
    const auto v = items(*cells[i]);
    const auto &c = s.exchange.remote_cells[i];
    EXPECT_EQ(c.sender, str(*v[0]));
    EXPECT_EQ(c.ix, integer(*v[1]));
    EXPECT_EQ(c.iy, integer(*v[2]));
    EXPECT_EQ(c.expires, num(*v[3]));
    const std::vector<double> an = doubles(*v[4]);
    EXPECT_EQ(
        (std::vector<double>{c.anchor.x, c.anchor.y, c.anchor.value, c.anchor.gx, c.anchor.gy}),
        an);
    EXPECT_EQ(c.identity, str(*v[5]));
  }
  const auto remote = items(state.get_child("remote_geometry"));
  ASSERT_EQ(s.exchange.remote_geometry.size(), remote.size());
  for (std::size_t i = 0; i < remote.size(); ++i) {
    const auto v = items(*remote[i]);
    const auto &r = s.exchange.remote_geometry[i];
    EXPECT_EQ(r.shape, obstacles().at(integer(*v[0])));
    EXPECT_EQ(r.expires, num(*v[1]));
    EXPECT_EQ(r.packet, str(*v[2]));
    EXPECT_EQ(r.source, str(*v[3]));
    EXPECT_EQ(r.observation_time, num(*v[4]));
    EXPECT_EQ(r.sender, str(*v[5]));
  }
  std::vector<coph_shape> reported, store;
  for (const pt::ptree *o : items(state.get_child("reported_geometry")))
    reported.push_back(obstacles().at(integer(*o)));
  for (const pt::ptree *o : items(state.get_child("geometry")))
    store.push_back(obstacles().at(integer(*o)));
  EXPECT_EQ(s.reported_geometry, reported);
  EXPECT_EQ(s.geometry, store);
  EXPECT_EQ(static_cast<long long>(s.terrain_anchors.size()),
            integer(state.get_child("terrain_anchors")));

  // The update, with the received packets' Gauss-Newton step.
  const pt::ptree &g = a.get_child("update");
  const pt::ptree &out = g.get_child("out");
  const coph_update_report r = coph_update(s, state4(snap.get_child("x")));
  EXPECT_EQ(r.messages, strings(g.get_child("messages")));
  ASSERT_EQ(r.message_fit.size(), s.stages.size());
  const auto plan = items(g.get_child("plan"));
  ASSERT_EQ(r.plan.entries.size(), plan.size());
  for (std::size_t k = 0; k < plan.size(); ++k) {
    const auto v = items(*plan[k]);
    EXPECT_EQ(r.plan.entries[k].stage, integer(*v[0]));
    EXPECT_EQ(r.plan.entries[k].eligible, boolean(*v[1]));
    EXPECT_EQ(r.plan.entries[k].horizon, integer(*v[2]));
    EXPECT_EQ(r.plan.entries[k].reason, str(*v[3]));
    EXPECT_EQ(r.plan.entries[k].pilot_steps, integer(*v[4]));
  }
  const auto attempts = items(g.get_child("attempts"));
  ASSERT_EQ(r.attempts.size(), attempts.size());
  for (std::size_t k = 0; k < attempts.size(); ++k) {
    const auto v = items(*attempts[k]);
    EXPECT_EQ(r.attempts[k].power, integer(*v[0]));
    EXPECT_EQ(r.attempts[k].reason, str(*v[1]));
    EXPECT_EQ(r.attempts[k].accepted, boolean(*v[2]));
  }
  EXPECT_EQ(r.exhausted, boolean(g.get_child("exhausted")));
  EXPECT_EQ(r.policy_independent, boolean(g.get_child("independent")));
  EXPECT_EQ(r.accepted, boolean(g.get_child("accepted")));
  EXPECT_EQ(r.reason, str(g.get_child("reason")));
  EXPECT_EQ(r.version_after, integer(g.get_child("version_after")));
  acc.check({r.field_validation_before, r.field_validation_after},
            out.get_child("field_validation"), "update field validation");
  acc.check({r.plant_validation_before, r.plant_validation_after},
            out.get_child("plant_validation"), "update plant validation");
  ASSERT_TRUE(r.replay_before && r.gradient);
  acc.check(*r.replay_before, num(out.get_child("replay_before")), "update replay before");
  acc.check({(*r.gradient)[0], (*r.gradient)[1]}, out.get_child("gradient"), "update gradient");
  acc.check({r.policy_after[0], r.policy_after[1]}, out.get_child("policy_after"),
            "update policy after");
  acc.check(r.damping_after, num(out.get_child("damping_after")), "update damping after");
  acc.check(r.shaping_jump, num(out.get_child("shaping_jump")), "update shaping jump");
  const auto weights = items(out.get_child("weights"));
  ASSERT_EQ(s.stages.size(), weights.size());
  for (std::size_t k = 0; k < weights.size(); ++k)
    acc.check(s.stages[k].weights, *weights[k], "update weights");
  acc.report("G10");
}

// ---------------------------------------------------------------------------
// C++ only
// ---------------------------------------------------------------------------

TEST(CophPacket, JsonEscapesAsPython) {
  coph_packet p;
  p.sender = "a\"b\\c\n\x01\xc3\xa9\xf0\x9f\x98\x80"; // a"b\c, newline, U+0001, e-acute, U+1F600
  const std::string j = p.json();
  EXPECT_NE(j.find("\"sender\":\"a\\\"b\\\\c\\n\\u0001\\u00e9\\ud83d\\ude00\""), std::string::npos)
      << j;
  p.H = std::nan("");
  EXPECT_THROW(p.json(), std::domain_error);
}

TEST(CophExchange, ProductionDrawsAreSeeded) {
  const pt::ptree &snap = snapshot(0);
  coph_learner_state s = rebuild(snap, "0");
  const coph_state x = state4(snap.get_child("x"));
  std::mt19937 a(7), b(7), c(8);
  const auto pa = coph_export(s, x, "1", a), pb = coph_export(s, x, "1", b),
             pc = coph_export(s, x, "1", c);
  ASSERT_TRUE(pa && pb && pc);
  EXPECT_EQ(pa->json(), pb->json());
  EXPECT_EQ(pa->anchors.size(), pc->anchors.size());
  EXPECT_NE(pa->covariance, pc->covariance); // other blocks, another ensemble
  const coph_draws d = coph_draw_blocks(a, 5, 8);
  ASSERT_EQ(d.size(), 8u);
  for (const auto &m : d) {
    ASSERT_EQ(m.size(), 5u);
    for (int i : m)
      EXPECT_TRUE(i >= 0 && i < 5);
  }
}

TEST(CophChannel, DelayDropsDuplicatesAndRange) {
  coph_ideal_channel_config cfg;
  cfg.seed = 3;
  cfg.delay_s = 0.25;
  cfg.jitter_s = 0.5;
  cfg.drop = 0.2;
  cfg.duplicate = 0.1;
  coph_ideal_channel ch(cfg);
  auto packet = std::make_shared<const coph_packet>();
  const int n = 20000;
  for (int i = 0; i < n; ++i)
    ch.send(0, 1, packet, 100, i * 0.01);
  EXPECT_TRUE(ch.poll(1, -1.0).empty());
  EXPECT_TRUE(ch.poll(0, 1e9).empty()); // nothing was sent to agent 0
  const std::vector<coph_delivery> got = ch.poll(1, 1e9);
  const auto &st = ch.statistics();
  EXPECT_EQ(st.sends, n);
  EXPECT_EQ(st.copies, st.dropped + st.delivered);
  EXPECT_EQ(static_cast<long long>(got.size()), st.delivered);
  const double dup = static_cast<double>(st.copies - n) / n;
  const double drop = static_cast<double>(st.dropped) / st.copies;
  EXPECT_NEAR(dup, 0.1, 4 * std::sqrt(0.1 * 0.9 / n));
  EXPECT_NEAR(drop, 0.2, 4 * std::sqrt(0.2 * 0.8 / st.copies));
  double mean_delay = 0;
  for (std::size_t i = 0; i < got.size(); ++i) {
    const double delay = got[i].at_s - got[i].sent_s;
    EXPECT_GE(delay, 0.25);
    EXPECT_LT(delay, 0.75);
    mean_delay += delay;
    if (i)
      EXPECT_LE(got[i - 1].at_s, got[i].at_s); // by arrival
    EXPECT_EQ(got[i].bytes, 100u);
  }
  mean_delay /= static_cast<double>(got.size());
  EXPECT_NEAR(mean_delay, 0.5, 0.01);
  EXPECT_TRUE(ch.poll(1, 1e9).empty()); // each delivery once

  // The same seed is the same run.
  coph_ideal_channel a(cfg), b(cfg);
  for (int i = 0; i < 100; ++i) {
    a.send(0, 1, packet, 1, i * 0.1);
    b.send(0, 1, packet, 1, i * 0.1);
  }
  const auto da = a.poll(1, 1e9), db = b.poll(1, 1e9);
  ASSERT_EQ(da.size(), db.size());
  for (std::size_t i = 0; i < da.size(); ++i)
    EXPECT_EQ(da[i].at_s, db[i].at_s);

  // Range: beyond it, or without a position, nothing arrives.
  coph_ideal_channel_config rc;
  rc.range_m = 10.0;
  coph_ideal_channel r(rc);
  r.set_position(0, {0.0, 0.0});
  r.set_position(1, {6.0, 8.0}); // exactly 10 m
  r.set_position(2, {6.0, 8.1});
  r.send(0, 1, packet, 1, 0.0);
  r.send(0, 2, packet, 1, 0.0);
  r.send(0, 3, packet, 1, 0.0);
  EXPECT_EQ(r.poll(1, 0.0).size(), 1u);
  EXPECT_TRUE(r.poll(2, 1.0).empty());
  EXPECT_TRUE(r.poll(3, 1.0).empty());
  EXPECT_EQ(r.statistics().out_of_range, 2);
  EXPECT_THROW(coph_ideal_channel(coph_ideal_channel_config{0, -1.0}), std::invalid_argument);
}
