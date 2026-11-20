/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// C5 (docs/roadmap/NAV-COPH-ROADMAP.md §4.5) against GRL-SNAM's golden, data/coph_golden.json:
// G12, the demos' runner with two CrossingLearner agents for 60 ticks of the crossing, with
// communication on (grl_snam.tools.coph_golden_episode; its episode_rules text states the rules).
// coph_fleet runs the same episode on coph_crossing_world with the golden's sensor noise and its
// packets' bootstrap draws. Every decision is compared exactly; positions and forces at atol 1e-6.

#include <algorithm>
#include <bit>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/coph_fleet.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <map>
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

const pt::ptree &episode() { return golden().get_child("episode"); }

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
coph_shape shape(const pt::ptree &arr) {
  const std::vector<double> v = doubles(arr);
  return v.size() == 5 ? coph_shape::segment(v[0], v[1], v[2], v[3], v[4])
                       : coph_shape::disk(v[0], v[1], v[2]);
}

std::string fixed9(double v) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.9f", v);
  return buf;
}

// The golden world's sensor noise (episode_rules).
double noise(int agent, long long tick, int j) {
  return 0.003 * static_cast<double>(((7 * tick + 13 * j + 5 * agent) % 11) - 5) / 5.0;
}

struct run_result {
  std::unique_ptr<coph_fleet> fleet;
  std::vector<coph_shape> obstacles;
};

run_result run_episode() {
  const pt::ptree &e = episode();
  const pt::ptree &world = e.get_child("world");
  coph_config cfg;
  cfg.controller_mode = "routed";
  cfg.shaping_mode = "progress";
  cfg.replay_objective = "mission";
  cfg.replay_domain = "retained";
  cfg.replay_horizon_mode = "supported_boundary";
  cfg.replay_update_cap = 1024;
  cfg.steps = static_cast<int>(integer(e.get_child("ticks")));
  cfg.validate();

  run_result r;
  coph_crossing_world_config wc;
  wc.cfg = cfg;
  wc.damping = num(world.get_child("damping"));
  for (const pt::ptree *o : items(world.get_child("obstacles")))
    r.obstacles.push_back(shape(*o));
  wc.obstacles = r.obstacles;
  for (const pt::ptree *s : items(world.get_child("world_seeds")))
    wc.seeds.push_back(integer(*s));
  wc.noise = noise;
  const coph_crossing_world w(wc);

  std::map<std::string, std::array<coph_draws, 2>> draws;
  for (const pt::ptree *d : items(e.get_child("draws"))) {
    const auto v = items(*d);
    std::array<coph_draws, 2> both;
    const auto gens = items(*v[1]);
    for (int g = 0; g < 2; ++g)
      for (const pt::ptree *member : items(*gens[g])) {
        std::vector<int> m;
        for (const pt::ptree *i : items(*member))
          m.push_back(static_cast<int>(integer(*i)));
        both[g].push_back(m);
      }
    draws[str(*v[0])] = both;
  }
  coph_draw_source source = [draws](const coph_agent &a, const coph_state &) {
    const coph_learner_state &s = a.state();
    const std::string identity =
        s.agent_id + ":" + fixed9(s.now) + ":" + std::to_string(s.stages.back().id);
    auto it = draws.find(identity);
    if (it == draws.end())
      throw std::runtime_error("no golden draws for " + identity);
    return it->second;
  };

  coph_fleet_config fc;
  fc.cfg = cfg;
  fc.comm = coph_communication_profile();
  fc.senders = {0, 1};
  std::vector<coph_fleet::agent_spec> agents;
  const auto initials = items(world.get_child("initials"));
  const auto goals = items(world.get_child("goals"));
  for (std::size_t i = 0; i < initials.size(); ++i) {
    const std::vector<double> x = doubles(*initials[i]), g = doubles(*goals[i]);
    agents.push_back({{x[0], x[1], x[2], x[3]}, {g[0], g[1]}});
  }
  r.fleet = std::make_unique<coph_fleet>(fc, agents, nullptr, source);
  r.fleet->run(w);
  return r;
}

const run_result &result() {
  static const run_result r = run_episode();
  return r;
}

} // namespace

TEST(CophFleetGolden, SamplesAndOutcomes) {
  const coph_fleet &f = *result().fleet;
  const auto agents = items(episode().get_child("agents"));
  ASSERT_EQ(agents.size(), 2u);
  for (int i = 0; i < 2; ++i) {
    const auto samples = items(agents[i]->get_child("samples"));
    ASSERT_EQ(f.samples(i).size(), samples.size()) << "agent " << i;
    for (std::size_t t = 0; t < samples.size(); ++t) {
      const auto v = items(*samples[t]);
      EXPECT_EQ(f.samples(i)[t].stage, integer(*v[0])) << i << " " << t;
      EXPECT_EQ(f.samples(i)[t].version, integer(*v[1])) << i << " " << t;
      EXPECT_EQ(f.samples(i)[t].done, boolean(*v[2])) << i << " " << t;
    }
    const auto outcome = items(agents[i]->get_child("outcome"));
    EXPECT_EQ(f.success(i), boolean(*outcome[0]));
    EXPECT_EQ(f.hit(i), boolean(*outcome[1]));
  }
}

TEST(CophFleetGolden, Controls) {
  const coph_fleet &f = *result().fleet;
  const auto agents = items(episode().get_child("agents"));
  for (int i = 0; i < 2; ++i) {
    const auto controls = items(agents[i]->get_child("controls"));
    ASSERT_EQ(f.controls(i).size(), controls.size()) << "agent " << i;
    for (std::size_t t = 0; t < controls.size(); ++t) {
      const auto v = items(*controls[t]);
      const coph_fleet::control &c = f.controls(i)[t];
      const std::string where = "agent " + std::to_string(i) + " tick " + std::to_string(t);
      EXPECT_EQ(c.tick, integer(*v[0])) << where;
      EXPECT_EQ(c.meta.reason, str(*v[1])) << where;
      EXPECT_EQ(c.meta.route_reason, str(*v[2])) << where;
      std::vector<int> nodes;
      for (const pt::ptree *n : items(*v[3]))
        nodes.push_back(static_cast<int>(integer(*n)));
      EXPECT_EQ(c.meta.route_nodes, nodes) << where;
      const std::vector<double> wp = doubles(*v[4]);
      ASSERT_EQ(wp.size(), 2u) << where;
      EXPECT_EQ(c.meta.waypoint[0], wp[0]) << where;
      EXPECT_EQ(c.meta.waypoint[1], wp[1]) << where;
      EXPECT_EQ(c.meta.guard_steps, integer(*v[5])) << where;
      EXPECT_EQ(c.meta.route_changed, boolean(*v[6])) << where;
    }
  }
}

TEST(CophFleetGolden, UpdatesAndGeometry) {
  const coph_fleet &f = *result().fleet;
  const auto agents = items(episode().get_child("agents"));
  for (int i = 0; i < 2; ++i) {
    const auto updates = items(agents[i]->get_child("updates"));
    ASSERT_EQ(f.updates(i).size(), updates.size()) << "agent " << i;
    for (std::size_t k = 0; k < updates.size(); ++k) {
      const auto v = items(*updates[k]);
      const coph_fleet::update_record &u = f.updates(i)[k];
      const std::string where = "agent " + std::to_string(i) + " update " + std::to_string(k);
      EXPECT_EQ(u.tick, integer(*v[0])) << where;
      std::vector<std::string> packets;
      for (const pt::ptree *p : items(*v[1]))
        packets.push_back(str(*p));
      EXPECT_EQ(u.packets, packets) << where;
      EXPECT_EQ(u.report.accepted, boolean(*v[2])) << where;
      EXPECT_EQ(u.report.reason, str(*v[3])) << where;
      EXPECT_EQ(u.report.version_after, integer(*v[4])) << where;
      const auto attempts = items(*v[5]);
      ASSERT_EQ(u.report.attempts.size(), attempts.size()) << where;
      for (std::size_t a = 0; a < attempts.size(); ++a) {
        const auto w = items(*attempts[a]);
        EXPECT_EQ(u.report.attempts[a].power, integer(*w[0])) << where;
        EXPECT_EQ(u.report.attempts[a].reason, str(*w[1])) << where;
        EXPECT_EQ(u.report.attempts[a].accepted, boolean(*w[2])) << where;
      }
    }
    const auto geometry = items(agents[i]->get_child("geometry"));
    const auto &mine = f.agents()[i].geometry_events();
    ASSERT_EQ(mine.size(), geometry.size()) << "agent " << i;
    for (std::size_t k = 0; k < geometry.size(); ++k) {
      const auto v = items(*geometry[k]);
      EXPECT_EQ(mine[k].kind, str(*v[0]));
      EXPECT_EQ(mine[k].obstacle, result().obstacles.at(integer(*v[1])));
      EXPECT_EQ(mine[k].source, str(*v[2]));
    }
  }
}

TEST(CophFleetGolden, Events) {
  const coph_fleet &f = *result().fleet;
  const auto events = items(episode().get_child("events"));
  ASSERT_EQ(f.events().size(), events.size());
  for (std::size_t k = 0; k < events.size(); ++k) {
    const auto v = items(*events[k]);
    const coph_fleet::event &e = f.events()[k];
    const std::string kind = str(*v[0]);
    EXPECT_EQ(e.kind, kind) << k;
    if (kind == "activation") {
      EXPECT_EQ(e.sender, integer(*v[1])) << k;
    } else if (kind == "sent") {
      EXPECT_EQ(e.sender, integer(*v[1])) << k;
      EXPECT_EQ(e.receiver, integer(*v[2])) << k;
      EXPECT_EQ(e.packet, str(*v[3])) << k;
    } else if (kind == "delivered") {
      EXPECT_EQ(e.sender, integer(*v[1])) << k;
      EXPECT_EQ(e.receiver, integer(*v[2])) << k;
      EXPECT_EQ(e.packet, str(*v[3])) << k;
      EXPECT_EQ(e.reason, str(*v[4])) << k;
      ASSERT_TRUE(e.version_after.has_value() && e.accepted.has_value()) << k;
      EXPECT_EQ(*e.version_after, integer(*v[5])) << k;
      EXPECT_EQ(*e.accepted, boolean(*v[6])) << k;
    }
  }
}

TEST(CophFleetGolden, PositionsAndForces) {
  const coph_fleet &f = *result().fleet;
  const double atol = episode().get<double>("tier.atol");
  const auto out = items(episode().get_child("out"));
  double worst = 0;
  long long exact = 0, values = 0;
  auto check = [&](double mine, double theirs, const std::string &what) {
    const double d = std::fabs(mine - theirs);
    worst = std::max(worst, d);
    exact += mine == theirs;
    ++values;
    EXPECT_LE(d, atol) << what << ": " << mine << " vs " << theirs;
  };
  for (int i = 0; i < 2; ++i) {
    const auto xs = items(out[i]->get_child("x"));
    ASSERT_EQ(f.samples(i).size(), xs.size());
    for (std::size_t t = 0; t < xs.size(); ++t) {
      const std::vector<double> x = doubles(*xs[t]);
      for (int k = 0; k < 4; ++k)
        check(f.samples(i)[t].x[k], x[k], "x");
    }
    const auto us = items(out[i]->get_child("u"));
    const auto nominal = items(out[i]->get_child("nominal"));
    ASSERT_EQ(f.controls(i).size(), us.size());
    for (std::size_t t = 0; t < us.size(); ++t) {
      const std::vector<double> u = doubles(*us[t]), p = doubles(*nominal[t]);
      for (int k = 0; k < 2; ++k) {
        check(f.controls(i)[t].u[k], u[k], "u");
        check(f.controls(i)[t].meta.proposed[k], p[k], "nominal");
      }
    }
  }
  std::printf("G12: %lld of %lld values bit for bit; the worst difference is %.3g (atol %g)\n",
              exact, values, worst, atol);
}

// ---------------------------------------------------------------------------
// C++ only
// ---------------------------------------------------------------------------

namespace {

coph_config profile(int steps) {
  coph_config cfg;
  cfg.controller_mode = "routed";
  cfg.shaping_mode = "progress";
  cfg.replay_objective = "mission";
  cfg.replay_domain = "retained";
  cfg.replay_horizon_mode = "supported_boundary";
  cfg.replay_update_cap = 1024;
  cfg.steps = steps;
  cfg.validate();
  return cfg;
}

coph_crossing_world crossing(const coph_config &cfg) {
  coph_crossing_world_config wc;
  wc.cfg = cfg;
  wc.obstacles = result().obstacles;
  wc.seeds = {152000, 152001};
  wc.noise = noise;
  return coph_crossing_world(wc);
}

std::vector<coph_fleet::agent_spec> crossers() {
  return {{{0.0, 4.0, 0.0, 0.0}, {12.0, 4.0}}, {{12.0, 4.0, 0.0, 0.0}, {0.0, 4.0}}};
}

} // namespace

TEST(CophFleet, RefusesConfigurationsOutsideTheProfile) {
  coph_config legacy = profile(60);
  legacy.controller_mode = "legacy";
  EXPECT_THROW(coph_agent(legacy, coph_communication_profile(), {0, 4, 0, 0}, {12, 4}, "0",
                          coph_crossing_network()),
               std::invalid_argument);
  coph_communication_config compact = coph_communication_profile();
  compact.prediction_variant = "compact";
  EXPECT_THROW(
      coph_agent(profile(60), compact, {0, 4, 0, 0}, {12, 4}, "0", coph_crossing_network()),
      std::invalid_argument);
  coph_fleet_config fc;
  fc.cfg = legacy;
  fc.comm = coph_communication_profile();
  EXPECT_THROW(coph_fleet(fc, crossers()), std::invalid_argument);
}

TEST(CophFleet, TheEnergyAccountStaysAtRoundOffOverALongRun) {
  // The crossing until both agents are done (at most 2,000 ticks), with the fleet's own draws.
  const coph_config cfg = profile(2000);
  coph_fleet_config fc;
  fc.cfg = cfg;
  fc.comm = coph_communication_profile();
  fc.senders = {0, 1};
  fc.seed = 7;
  coph_fleet f(fc, crossers());
  f.run(crossing(cfg));
  for (int i = 0; i < 2; ++i) {
    const coph_energy &e = f.energy(i);
    std::printf("agent %d: %s after %zu steps; work %.6g, heat %.6g, residual %.3g, shaping "
                "jumps %.6g, version %lld\n",
                i, f.success(i) ? "at its goal" : (f.hit(i) ? "hit" : "timed out"),
                f.controls(i).size(), e.work, e.heat, e.residual, e.shaping_jumps,
                static_cast<long long>(f.agents()[i].state().version));
    EXPECT_EQ(e.steps, static_cast<long long>(f.controls(i).size()));
    EXPECT_GT(e.work, 1.0);
    EXPECT_LE(std::fabs(e.residual), 1e-9 * (e.work + e.heat)) << "agent " << i;
    EXPECT_FALSE(f.hit(i)) << "agent " << i;
  }
}

TEST(CophFleet, PacketsArriveAfterTheChannelsDelay) {
  const coph_config cfg = profile(40);
  coph_fleet_config fc;
  fc.cfg = cfg;
  fc.comm = coph_communication_profile();
  fc.senders = {0, 1};
  coph_ideal_channel_config cc;
  cc.delay_s = 0.3;
  coph_fleet f(fc, crossers(), std::make_shared<coph_ideal_channel>(cc));
  f.run(crossing(cfg));
  std::map<std::string, double> sent;
  int delivered = 0;
  for (const coph_fleet::event &e : f.events()) {
    if (e.kind == "sent")
      sent[e.packet] = e.time;
    if (e.kind == "delivered") {
      ++delivered;
      ASSERT_TRUE(sent.count(e.packet)) << e.packet;
      EXPECT_GE(e.time + 1e-9, sent[e.packet] + 0.3) << e.packet;
      EXPECT_LT(e.time, sent[e.packet] + 0.3 + cfg.dt) << e.packet; // at the next boundary
    }
  }
  EXPECT_GT(delivered, 4);
  // An update at each delivery, between the 8-step ones.
  bool off_cadence = false;
  for (const coph_fleet::update_record &u : f.updates(0))
    off_cadence = off_cadence || (u.tick % cfg.update_every != 0 && !u.packets.empty());
  EXPECT_TRUE(off_cadence);
}
