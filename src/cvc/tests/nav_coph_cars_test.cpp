/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// C5 part 2 (docs/roadmap/NAV-COPH-ROADMAP.md §4.5) against GRL-SNAM's golden,
// data/coph_golden.json: G13, two of the cars controller's learners on an open field under the
// demos' runner, with communication on (grl_snam.tools.coph_golden_cars; its cars_rules text states
// the rules). A car's learner has no network: coph_agent runs CooperativeLearner on Learner and
// routes with the profile's own planner (coph_profile_route). coph_fleet runs the episode on
// coph_crossing_world with no network, the golden's risk patch and sensor noise, and its recorded
// bootstrap draws. Decisions must match bit for bit; positions and forces within the section's
// atol.

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
  static const pt::ptree root = [] {
    pt::ptree r;
    pt::read_json((kData / "coph_golden.json").string(), r);
    return r;
  }();
  return root;
}

const pt::ptree &cars() { return golden().get_child("cars"); }

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

std::string fixed9(double v) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.9f", v);
  return buf;
}

// The golden world's sensor noise (G12's, cars_rules).
double noise(int agent, long long tick, int j) {
  return 0.003 * static_cast<double>(((7 * tick + 13 * j + 5 * agent) % 11) - 5) / 5.0;
}

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

std::vector<coph_vec2> goals() {
  std::vector<coph_vec2> g;
  for (const pt::ptree *p : items(cars().get_child("world.goals"))) {
    const std::vector<double> v = doubles(*p);
    g.push_back({v[0], v[1]});
  }
  return g;
}

std::unique_ptr<coph_fleet> run_cars() {
  const pt::ptree &c = cars();
  const pt::ptree &world = c.get_child("world");
  const coph_config cfg = profile(static_cast<int>(integer(c.get_child("ticks"))));

  coph_crossing_world_config wc;
  wc.cfg = cfg;
  wc.network = coph_network{}; // the open field
  wc.damping = num(world.get_child("damping"));
  for (const pt::ptree *o : items(world.get_child("obstacles"))) {
    const std::vector<double> v = doubles(*o);
    wc.obstacles.push_back(coph_shape::disk(v[0], v[1], v[2]));
  }
  for (const pt::ptree *p : items(world.get_child("patches"))) {
    const std::vector<double> v = doubles(*p);
    wc.patches.push_back({v[0], v[1], v[2], v[3], v[4]});
  }
  for (const pt::ptree *s : items(world.get_child("world_seeds")))
    wc.seeds.push_back(integer(*s));
  wc.noise = noise;
  const coph_crossing_world w(wc);

  // CooperativeLearner's draws only: a car's packet has no terrain layer.
  std::map<std::string, std::array<coph_draws, 2>> draws;
  for (const pt::ptree *d : items(c.get_child("draws"))) {
    const auto v = items(*d);
    const auto gens = items(*v[1]);
    if (gens.size() != 1)
      throw std::runtime_error("G13: expected one generator's draws");
    std::array<coph_draws, 2> both;
    for (const pt::ptree *member : items(*gens[0])) {
      std::vector<int> m;
      for (const pt::ptree *i : items(*member))
        m.push_back(static_cast<int>(integer(*i)));
      both[0].push_back(m);
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
  fc.network = coph_network{}; // cars' learners: the profile's own router
  fc.senders = {0, 1};
  std::vector<coph_fleet::agent_spec> agents;
  const auto initials = items(world.get_child("initials"));
  const std::vector<coph_vec2> g = goals();
  for (std::size_t i = 0; i < initials.size(); ++i) {
    const std::vector<double> x = doubles(*initials[i]);
    agents.push_back({{x[0], x[1], x[2], x[3]}, g[i]});
  }
  auto fleet = std::make_unique<coph_fleet>(fc, agents, nullptr, source);
  fleet->run(w);
  return fleet;
}

const coph_fleet &fleet() {
  static const std::unique_ptr<coph_fleet> f = run_cars();
  return *f;
}

} // namespace

TEST(CophCarsGolden, SamplesAndOutcomes) {
  const coph_fleet &f = fleet();
  const auto agents = items(cars().get_child("agents"));
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

TEST(CophCarsGolden, TheProfilesRouter) {
  const coph_fleet &f = fleet();
  const auto agents = items(cars().get_child("agents"));
  const std::vector<coph_vec2> g = goals();
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
      EXPECT_EQ(c.meta.route_expansions, integer(*v[3])) << where;
      if (is_null(*v[4])) {
        EXPECT_FALSE(c.meta.route_changed) << where;
      } else {
        std::vector<coph_vec2> route;
        for (const pt::ptree *p : items(*v[4])) {
          if (str(*p) == "goal") {
            route.push_back(g[i]);
            continue;
          }
          const auto ij = items(*p);
          route.push_back({static_cast<double>(integer(*ij[0])) * 0.1,
                           static_cast<double>(integer(*ij[1])) * 0.1});
        }
        EXPECT_EQ(c.meta.route, route) << where;
      }
      const std::vector<double> wp = doubles(*v[5]);
      ASSERT_EQ(wp.size(), 2u) << where;
      EXPECT_EQ(c.meta.waypoint[0], wp[0]) << where;
      EXPECT_EQ(c.meta.waypoint[1], wp[1]) << where;
      EXPECT_EQ(c.meta.guard_steps, integer(*v[6])) << where;
      EXPECT_EQ(c.meta.route_changed, boolean(*v[7])) << where;
      EXPECT_EQ(c.meta.stagnation, boolean(*v[8])) << where;
    }
  }
}

TEST(CophCarsGolden, Updates) {
  const coph_fleet &f = fleet();
  const auto agents = items(cars().get_child("agents"));
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
    EXPECT_TRUE(f.agents()[i].geometry_events().empty()) << "a car's learner reports no geometry";
  }
}

TEST(CophCarsGolden, Events) {
  const coph_fleet &f = fleet();
  const auto events = items(cars().get_child("events"));
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

TEST(CophCarsGolden, PositionsAndForces) {
  const coph_fleet &f = fleet();
  const double atol = cars().get<double>("tier.atol");
  const auto out = items(cars().get_child("out"));
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
  std::printf("G13: %lld of %lld values bit for bit; the worst difference is %.3g (atol %g)\n",
              exact, values, worst, atol);
}

// A car's packet is CooperativeLearner's: its JSON has no geometry keys.
TEST(CophCarsGolden, ACarsPacketHasNoGeometry) {
  coph_packet p;
  const std::string maze = p.json();
  p.maze = false;
  const std::string car = p.json();
  EXPECT_NE(maze.find("\"geometry_schema\""), std::string::npos);
  EXPECT_EQ(car.find("\"geometry"), std::string::npos);
  EXPECT_EQ(car.size() + std::string(",\"geometry\":[],\"geometry_schema\":1").size(), maze.size());
}

// The profile's router takes disks only, as the Python does (it unpacks every shape as (x, y, r)).
TEST(CophCarsGolden, TheProfilesRouterRefusesASlab) {
  coph_learner_state s;
  s.cfg = profile(10);
  coph_stage st;
  st.center = {0.0, 0.0};
  s.stages.push_back(st);
  s.geometry = {coph_shape::segment(1.0, 0.0, 0.1, 0.0, 0.5)};
  const coph_atlas atlas = coph_freeze(s, {std::vector<double>()});
  coph_route_context ctx;
  EXPECT_THROW(coph_profile_route({0.0, 0.0}, atlas, ctx, 0), std::invalid_argument);
}
