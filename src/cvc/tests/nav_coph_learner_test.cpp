/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// C3 (docs/roadmap/NAV-COPH-ROADMAP.md §4.3) against GRL-SNAM's golden, data/coph_golden.json:
// G5 (the field fit), G6 (the damping fit), G7 (the replay) and G8 (the update), at three update
// snapshots of a scripted crossing run (grl_snam.tools.coph_golden_learner; its learner_rules text
// states the rules). Each snapshot is rebuilt into a coph_learner_state from the file's inputs
// alone. Values ("out") are compared at the section's tier, rtol 1e-9 and atol 1e-12; decisions
// (reasons, eligibility, handoffs, routes, step counts, accept or reject) exactly.

#include <algorithm>
#include <bit>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/coph_learner.h>
#include <filesystem>
#include <gtest/gtest.h>
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

  void check(double mine, double theirs, const std::string &what) {
    const double allowance = rtol * std::fabs(theirs) + atol;
    const double diff = std::fabs(mine - theirs);
    ++values;
    exact += mine == theirs;
    worst = std::max(worst, diff / allowance);
    EXPECT_LE(diff, allowance) << what << ": " << mine << " vs " << theirs;
  }
  void check(const std::vector<double> &mine, const pt::ptree &theirs, const std::string &what) {
    const std::vector<double> t = doubles(theirs);
    ASSERT_EQ(mine.size(), t.size()) << what;
    for (std::size_t i = 0; i < t.size(); ++i)
      check(mine[i], t[i], what + "[" + std::to_string(i) + "]");
  }
  void report(const char *name) const {
    std::printf("%s: %lld of %lld values bit for bit; the worst is %.3g of the tier's allowance\n",
                name, exact, values, worst);
  }
};

tier_account account() {
  tier_account a;
  a.rtol = learner().get<double>("tier.rtol");
  a.atol = learner().get<double>("tier.atol");
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

// learner_rules' rebuild: the snapshot's state from the script and the snapshot's inputs.
coph_learner_state rebuild(const pt::ptree &snap) {
  const pt::ptree &world = learner().get_child("world");
  const auto script = items(learner().get_child("script"));
  const std::vector<coph_shape> obstacles = [&] {
    std::vector<coph_shape> v;
    for (const pt::ptree *o : items(world.get_child("obstacles")))
      v.push_back(shape(*o));
    return v;
  }();
  const std::string agent = str(world.get_child("agent_id"));
  const double radius = num(world.get_child("sensor_radius"));
  const std::size_t lattice = world.get_child("lattice").size();
  coph_learner_state s;
  s.cfg = profile();
  s.goal = vec2(world.get_child("goal"));
  s.network = coph_crossing_network();
  {
    const auto nodes = items(world.get_child("nodes"));
    EXPECT_EQ(nodes.size(), s.network.nodes.size());
    for (std::size_t k = 0; k < nodes.size(); ++k)
      EXPECT_EQ(vec2(*nodes[k]), s.network.nodes[k]);
  }
  auto tick_state = [&](long long k) { return state4(script[k]->get_child("state")); };
  for (const pt::ptree *st : items(snap.get_child("stages"))) {
    coph_stage stage = make_coph_stage(integer(st->get_child("id")), vec2(st->get_child("center")),
                                       state4(st->get_child("entry")), s.goal, s.cfg);
    EXPECT_EQ(stage.target, vec2(st->get_child("target")));
    stage.weights = doubles(st->get_child("weights"));
    stage.version = integer(st->get_child("version"));
    for (long long i : expand(st->get_child("readings"))) {
      const long long tick = i / static_cast<long long>(lattice),
                      j = i % static_cast<long long>(lattice);
      const std::vector<double> r = doubles(*items(script[tick]->get_child("readings"))[j]);
      stage.readings.push_back({agent + ":" + std::to_string(tick) + ":" + std::to_string(j),
                                tick * s.cfg.dt,
                                {r[0], r[1]},
                                r[2]});
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
      stage.obstacles.push_back(obstacles.at(integer(*o)));
    s.stages.push_back(std::move(stage));
  }
  for (const pt::ptree *o : items(snap.get_child("geometry")))
    s.geometry.push_back(obstacles.at(integer(*o)));
  for (const pt::ptree *o : items(snap.get_child("reported_geometry")))
    s.reported_geometry.push_back(obstacles.at(integer(*o)));
  for (const pt::ptree *a : items(snap.get_child("terrain_anchors"))) {
    const std::vector<double> v = doubles(*a);
    s.terrain_anchors.push_back({v[0], v[1], v[2], v[3], v[4]});
  }
  s.damping = num(snap.get_child("damping"));
  s.policy = vec2(snap.get_child("policy"));
  s.version = integer(snap.get_child("version"));
  return s;
}

struct snapshot_case {
  std::string name;
  const pt::ptree *snap, *fit, *damping, *replay, *update;
};
std::vector<snapshot_case> cases() {
  const auto snaps = items(learner().get_child("snapshots"));
  const auto fits = items(learner().get_child("field_fit"));
  const auto dampings = items(learner().get_child("damping_fit"));
  const auto replays = items(learner().get_child("replay"));
  const auto updates = items(learner().get_child("update"));
  std::vector<snapshot_case> out;
  for (std::size_t i = 0; i < snaps.size(); ++i)
    out.push_back(
        {str(snaps[i]->get_child("name")), snaps[i], fits[i], dampings[i], replays[i], updates[i]});
  return out;
}

std::vector<std::vector<double>> golden_weights(const pt::ptree &fit) {
  std::vector<std::vector<double>> w;
  for (const pt::ptree *st : items(fit.get_child("stages")))
    w.push_back(doubles(st->get_child("out.weights")));
  return w;
}

} // namespace

TEST(CophLearnerGolden, HasThreeSnapshots) {
  ASSERT_EQ(cases().size(), 3u);
  for (const snapshot_case &c : cases())
    EXPECT_EQ(c.snap->get_child("stages").size(), 3u) << c.name;
}

// G5: each stage's normal equations, weights and covariance.
TEST(CophLearnerGolden, FieldFit) {
  tier_account acc = account();
  for (const snapshot_case &c : cases()) {
    const coph_learner_state s = rebuild(*c.snap);
    const std::vector<coph_stage_fit> fit = coph_field_fit(s);
    const auto stages = items(c.fit->get_child("stages"));
    ASSERT_EQ(fit.size(), stages.size());
    for (std::size_t k = 0; k < fit.size(); ++k) {
      const pt::ptree &out = stages[k]->get_child("out");
      const std::string where = c.name + " stage " + std::to_string(k);
      EXPECT_EQ(fit[k].rows, integer(stages[k]->get_child("rows"))) << where;
      acc.check(fit[k].targets, out.get_child("targets"), where + " targets");
      acc.check(fit[k].normal, out.get_child("normal"), where + " normal");
      acc.check(fit[k].rhs, out.get_child("rhs"), where + " rhs");
      acc.check(fit[k].weights, out.get_child("weights"), where + " weights");
      // The inverse's small entries carry the matrix's conditioning (the Python solves by LU,
      // this by Cholesky): compare them against the largest entry.
      const std::vector<double> cov = doubles(out.get_child("covariance"));
      double scale = 0;
      for (double v : cov)
        scale = std::max(scale, std::fabs(v));
      ASSERT_EQ(fit[k].covariance.size(), cov.size());
      for (std::size_t i = 0; i < cov.size(); ++i)
        EXPECT_LE(std::fabs(fit[k].covariance[i] - cov[i]), acc.rtol * scale)
            << where << " covariance";
    }
  }
  acc.report("G5");
}

// G6: the Newton iterates on hyper-duals, the result and the reconciled proposal.
TEST(CophLearnerGolden, DampingFit) {
  tier_account acc = account();
  for (const snapshot_case &c : cases()) {
    const coph_learner_state s = rebuild(*c.snap);
    const coph_damping_fit fit = coph_fit_damping(s);
    const pt::ptree &out = c.damping->get_child("out");
    EXPECT_EQ(fit.transitions, integer(c.damping->get_child("transitions"))) << c.name;
    const auto its = items(out.get_child("iterates"));
    ASSERT_EQ(fit.iterates.size(), its.size()) << c.name;
    for (std::size_t k = 0; k < its.size(); ++k) {
      const coph_damping_iterate &m = fit.iterates[k];
      const pt::ptree &t = *its[k];
      const std::string where = c.name + " iterate " + std::to_string(k);
      acc.check(m.damping, num(t.get_child("damping")), where + " damping");
      acc.check(m.loss, num(t.get_child("loss")), where + " loss");
      acc.check(m.gradient, num(t.get_child("gradient")), where + " gradient");
      acc.check(m.curvature, num(t.get_child("curvature")), where + " curvature");
      acc.check(m.step, num(t.get_child("step")), where + " step");
      acc.check(m.next, num(t.get_child("next")), where + " next");
    }
    acc.check(fit.result, num(out.get_child("result")), c.name + " result");
    acc.check(coph_reconcile_damping(fit.result, s.damping, s.cfg), num(out.get_child("proposal")),
              c.name + " proposal");
  }
  acc.report("G6");
}

// G7: the plan, then the replay at six policies, on the golden's own fitted weights and proposal.
TEST(CophLearnerGolden, Replay) {
  tier_account acc = account();
  for (const snapshot_case &c : cases()) {
    coph_learner_state s = rebuild(*c.snap);
    const coph_atlas atlas = coph_freeze(s, golden_weights(*c.fit));
    const double damping = num(c.damping->get_child("out.proposal"));
    const coph_replay_plan plan = coph_prepare(s, atlas, damping, 12);
    const pt::ptree &g = *c.replay;
    EXPECT_EQ(plan.external_reserve, integer(g.get_child("budget.external_reserve"))) << c.name;
    EXPECT_EQ(plan.baseline_reserved, boolean(g.get_child("budget.baseline_reserved"))) << c.name;
    EXPECT_EQ(plan.pilot_steps, integer(g.get_child("budget.pilot_steps"))) << c.name;
    const auto entries = items(g.get_child("plan"));
    ASSERT_EQ(plan.entries.size(), entries.size()) << c.name;
    for (std::size_t k = 0; k < entries.size(); ++k) {
      const coph_plan_entry &e = plan.entries[k];
      const pt::ptree &t = *entries[k];
      const std::string where = c.name + " plan " + std::to_string(k);
      EXPECT_EQ(e.stage, integer(t.get_child("stage"))) << where;
      EXPECT_EQ(e.eligible, boolean(t.get_child("eligible"))) << where;
      EXPECT_EQ(e.horizon, integer(t.get_child("horizon"))) << where;
      EXPECT_EQ(e.reason, str(t.get_child("reason"))) << where;
      EXPECT_EQ(e.pilot_steps, integer(t.get_child("pilot_steps"))) << where;
      const auto hs = items(t.get_child("handoffs"));
      ASSERT_EQ(e.handoffs.size(), hs.size()) << where;
      for (std::size_t h = 0; h < hs.size(); ++h) {
        const auto v = items(*hs[h]);
        for (int i = 0; i < 3; ++i)
          EXPECT_EQ(e.handoffs[h][i], integer(*v[i])) << where << " handoff " << h;
      }
    }
    int index = 0;
    for (const pt::ptree *p : items(g.get_child("policies"))) {
      const coph_vec2 policy = vec2(p->get_child("policy"));
      const std::string where = c.name + " policy " + std::to_string(index++);
      const coph_replay_result r = coph_replay(s, atlas, plan, policy, damping, true);
      EXPECT_EQ(r.replayed, boolean(p->get_child("replayed"))) << where;
      EXPECT_EQ(r.steps, integer(p->get_child("steps"))) << where;
      const auto el = items(p->get_child("eligibility"));
      ASSERT_EQ(r.eligibility.size(), el.size()) << where;
      for (std::size_t k = 0; k < el.size(); ++k) {
        const auto v = items(*el[k]);
        EXPECT_EQ(r.eligibility[k].stage, integer(*v[0])) << where;
        EXPECT_EQ(r.eligibility[k].eligible, boolean(*v[1])) << where;
        EXPECT_EQ(r.eligibility[k].reason, str(*v[2])) << where;
      }
      const auto ev = items(p->get_child("events"));
      ASSERT_EQ(r.events.size(), ev.size()) << where;
      for (std::size_t k = 0; k < ev.size(); ++k) {
        const auto v = items(*ev[k]);
        EXPECT_EQ(r.events[k].entry, integer(*v[0])) << where;
        EXPECT_EQ(r.events[k].step, integer(*v[1])) << where;
        EXPECT_EQ(r.events[k].from, integer(*v[2])) << where;
        EXPECT_EQ(r.events[k].to, integer(*v[3])) << where;
      }
      const auto ctl = items(p->get_child("controls"));
      ASSERT_EQ(r.controls.size(), ctl.size()) << where;
      for (std::size_t k = 0; k < ctl.size(); ++k) {
        const auto v = items(*ctl[k]);
        const coph_control_record &m = r.controls[k];
        const std::string at = where + " control " + std::to_string(k);
        EXPECT_EQ(m.entry, integer(*v[0])) << at;
        EXPECT_EQ(m.step, integer(*v[1])) << at;
        EXPECT_EQ(m.meta.reason, str(*v[2])) << at;
        EXPECT_EQ(m.meta.route_reason, str(*v[3])) << at;
        EXPECT_EQ(m.meta.guard_steps, integer(*v[4])) << at;
        EXPECT_EQ(m.meta.projection_candidates, integer(*v[5])) << at;
        std::vector<int> nodes;
        for (const pt::ptree *n : items(*v[6]))
          nodes.push_back(static_cast<int>(integer(*n)));
        EXPECT_EQ(m.meta.route_nodes, nodes) << at;
      }
      const auto lengths = items(p->get_child("lengths"));
      ASSERT_EQ(r.paths.size(), lengths.size()) << where;
      const pt::ptree &out = p->get_child("out");
      const auto paths = items(out.get_child("paths"));
      for (std::size_t k = 0; k < lengths.size(); ++k) {
        const auto v = items(*lengths[k]);
        EXPECT_EQ(r.paths[k].first, integer(*v[0])) << where;
        ASSERT_EQ(static_cast<long long>(r.paths[k].second.size()), integer(*v[1])) << where;
        const auto states = items(*paths[k]);
        for (std::size_t i = 0; i < states.size(); ++i)
          acc.check({r.paths[k].second[i].begin(), r.paths[k].second[i].end()}, *states[i],
                    where + " path");
      }
      if (r.replayed) {
        acc.check(r.cost, num(out.get_child("cost")), where + " cost");
        acc.check({r.gradient[0], r.gradient[1]}, out.get_child("gradient"), where + " gradient");
      } else {
        EXPECT_TRUE(is_null(out.get_child("cost"))) << where;
      }
      const auto applied = items(out.get_child("applied"));
      const auto waypoints = items(out.get_child("waypoints"));
      for (std::size_t k = 0; k < r.controls.size(); ++k) {
        acc.check({r.controls[k].meta.applied[0], r.controls[k].meta.applied[1]}, *applied[k],
                  where + " applied");
        acc.check({r.controls[k].meta.waypoint[0], r.controls[k].meta.waypoint[1]}, *waypoints[k],
                  where + " waypoint");
      }
      const auto jumps = items(out.get_child("jumps"));
      ASSERT_EQ(jumps.size(), r.events.size()) << where;
      for (std::size_t k = 0; k < jumps.size(); ++k)
        acc.check(r.events[k].shaping_jump, num(*jumps[k]), where + " jump");

      // Without the gradient, the same replay: the same decisions and the same cost.
      const coph_replay_result plain = coph_replay(s, atlas, plan, policy, damping, false);
      EXPECT_EQ(plain.replayed, r.replayed) << where;
      EXPECT_EQ(plain.steps, r.steps) << where;
      EXPECT_EQ(plain.cost, r.cost) << where;
      EXPECT_EQ(plain.controls.size(), r.controls.size()) << where;
    }
  }
  acc.report("G7");
}

// G8: the whole update from the rebuilt state.
TEST(CophLearnerGolden, Update) {
  tier_account acc = account();
  for (const snapshot_case &c : cases()) {
    coph_learner_state s = rebuild(*c.snap);
    const coph_vec2 policy_before = s.policy;
    const double damping_before = s.damping;
    const coph_update_report r = coph_update(s, state4(c.snap->get_child("x")));
    const pt::ptree &g = *c.update;
    const pt::ptree &out = g.get_child("out");
    const std::string where = c.name;
    const double slack = s.cfg.validation_slack;
    const double field_margin = (r.field_validation_before + slack) - r.field_validation_after;
    const double plant_margin = (r.plant_validation_before + slack) - r.plant_validation_after;
    EXPECT_EQ(field_margin >= 0, boolean(g.get_child("field_valid"))) << where;
    EXPECT_EQ(plant_margin >= 0, boolean(g.get_child("plant_valid"))) << where;
    EXPECT_EQ(r.feasible,
              boolean(g.get_child("damping_feasible")) && boolean(g.get_child("policy_in_box")))
        << where;
    const auto attempts = items(g.get_child("attempts"));
    ASSERT_EQ(r.attempts.size(), attempts.size()) << where;
    for (std::size_t k = 0; k < attempts.size(); ++k) {
      const auto v = items(*attempts[k]);
      EXPECT_EQ(r.attempts[k].power, integer(*v[0])) << where;
      EXPECT_EQ(r.attempts[k].reason, str(*v[1])) << where;
      EXPECT_EQ(r.attempts[k].accepted, boolean(*v[2])) << where;
    }
    EXPECT_EQ(r.exhausted, boolean(g.get_child("exhausted"))) << where;
    EXPECT_EQ(r.policy_independent, boolean(g.get_child("independent"))) << where;
    EXPECT_EQ(r.accepted, boolean(g.get_child("accepted"))) << where;
    EXPECT_EQ(r.reason, str(g.get_child("reason"))) << where;
    EXPECT_EQ(r.version_after, integer(g.get_child("version_after"))) << where;

    acc.check({r.field_validation_before, r.field_validation_after},
              out.get_child("field_validation"), where + " field validation");
    acc.check({r.plant_validation_before, r.plant_validation_after},
              out.get_child("plant_validation"), where + " plant validation");
    acc.check({field_margin, plant_margin}, out.get_child("margins"), where + " margins");
    ASSERT_TRUE(r.replay_before && r.gradient && r.unconstrained && r.bounded) << where;
    acc.check(*r.replay_before, num(out.get_child("replay_before")), where + " replay before");
    acc.check({(*r.gradient)[0], (*r.gradient)[1]}, out.get_child("gradient"), where + " gradient");
    acc.check({(*r.unconstrained)[0], (*r.unconstrained)[1]}, out.get_child("unconstrained"),
              where + " unconstrained");
    acc.check({(*r.bounded)[0], (*r.bounded)[1]}, out.get_child("bounded"), where + " bounded");
    const auto candidates = items(out.get_child("candidates"));
    const auto costs = items(out.get_child("costs"));
    for (std::size_t k = 0; k < r.attempts.size(); ++k) {
      acc.check({r.attempts[k].policy[0], r.attempts[k].policy[1]}, *candidates[k],
                where + " candidate");
      ASSERT_TRUE(r.attempts[k].cost.has_value()) << where;
      acc.check(*r.attempts[k].cost, num(*costs[k]), where + " cost");
    }
    acc.check({r.policy_after[0], r.policy_after[1]}, out.get_child("policy_after"),
              where + " policy after");
    acc.check(r.damping_after, num(out.get_child("damping_after")), where + " damping after");
    acc.check(r.shaping_jump, num(out.get_child("shaping_jump")), where + " shaping jump");

    // The commit, or the rollback.
    if (r.accepted) {
      EXPECT_EQ(s.version, integer(c.snap->get_child("version")) + 1) << where;
      for (std::size_t k = 0; k < s.stages.size(); ++k) {
        EXPECT_EQ(s.stages[k].weights, r.fit[k].weights) << where;
        EXPECT_EQ(s.stages[k].version, s.version) << where;
      }
    } else {
      EXPECT_EQ(s.version, integer(c.snap->get_child("version"))) << where;
      EXPECT_EQ(s.policy, policy_before) << where;
      EXPECT_EQ(s.damping, damping_before) << where;
      const auto st = items(c.snap->get_child("stages"));
      for (std::size_t k = 0; k < s.stages.size(); ++k)
        EXPECT_EQ(s.stages[k].weights, doubles(st[k]->get_child("weights"))) << where;
    }
  }
  acc.report("G8");
}

TEST(CophDual, ProductQuotientAndHyperDual) {
  // f(x, y) = x y / (x + y) at (2, 3): df/dx = y^2 / (x + y)^2, df/dy = x^2 / (x + y)^2.
  const coph_dual2 x = coph_dual2::variable(2.0, 0), y = coph_dual2::variable(3.0, 1);
  const coph_dual2 f = x * y / (x + y);
  EXPECT_DOUBLE_EQ(f.v, 1.2);
  EXPECT_DOUBLE_EQ(f.d[0], 9.0 / 25.0);
  EXPECT_DOUBLE_EQ(f.d[1], 4.0 / 25.0);
  // g(t) = t^3 - 2 t at t = 1.5 on a hyper-dual: g' = 3 t^2 - 2, g'' = 6 t.
  coph_hyper t;
  t.v.v = 1.5;
  t.v.d[0] = 1.0;
  t.d[0].v = 1.0;
  const coph_hyper g = t * t * t - 2.0 * t;
  EXPECT_DOUBLE_EQ(coph_value(g), 1.5 * 1.5 * 1.5 - 3.0);
  EXPECT_DOUBLE_EQ(g.v.d[0], 3 * 1.5 * 1.5 - 2);
  EXPECT_DOUBLE_EQ(g.d[0].v, 3 * 1.5 * 1.5 - 2);
  EXPECT_DOUBLE_EQ(g.d[0].d[0], 6 * 1.5);
}

TEST(CophMazeRoute, TheMiddleRowUntilTheDoorCloses) {
  coph_network net = coph_crossing_network();
  coph_route_context ctx;
  // On the middle row, heading for (12, 4): straight along it, to the first dense route point at
  // least 0.65 away, on the 0.1 m grid. The values are maze_route's own.
  coph_route r = coph_maze_route({3.02, 4.01}, {12.0, 4.0}, net, ctx, 0);
  EXPECT_EQ(r.reason, "maze_graph");
  EXPECT_EQ(r.nodes, (std::vector<int>{4}));
  EXPECT_EQ(r.target, (coph_vec2{38 * 0.1, 40 * 0.1}));
  EXPECT_EQ(*ctx.target, r.target);
  // The middle door closes the middle row (edge 2): the route goes back west, then north.
  ctx.reported_geometry = {coph_shape::segment(9.5, 4.0, 0.1, 0.0, 0.75)};
  r = coph_maze_route({8.0, 4.0}, {12.0, 4.0}, net, ctx, 1);
  EXPECT_EQ(r.reason, "maze_graph");
  EXPECT_EQ(r.blocked, (std::vector<int>{2}));
  EXPECT_EQ(r.nodes, (std::vector<int>{1, 2, 5, 4}));
  EXPECT_EQ(r.target, (coph_vec2{73 * 0.1, 40 * 0.1}));
}
