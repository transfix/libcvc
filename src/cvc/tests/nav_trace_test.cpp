/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A sim_world's input trace (cvc/nav/sim_world_trace.h): a live session on a small kinematic
// world, recorded from the state tree and the host's verbs, replays headlessly to byte-identical
// poses and the same hash; a perturbed trace departs at the step that consumes the perturbation;
// and one trace replayed on another policy departs only for the agent on that policy, from the
// first step its coefficients differ (the GRL-SNAM Laboratory's acceptance test 6).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/clock_follower.h>
#include <cvc/core/input_trace.h>
#include <cvc/core/world_clock.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/drive.h>
#include <cvc/nav/policy_library.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/sim_world_trace.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <string>
#include <vector>

using namespace cvc::nav;

namespace {

constexpr double kS = 0.05; // 1 normalized unit = 20 m
constexpr int R = 64, C = 64;
constexpr double kLo = -32.0, kHi = 32.0; // 1 m cells

std::vector<std::uint8_t> city() {
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  auto block = [&](int r0, int r1, int c0, int c1) {
    for (int r = r0; r < r1; ++r)
      for (int c = c0; c < c1; ++c)
        occ[static_cast<std::size_t>(r) * C + c] = 1;
  };
  block(24, 40, 28, 36); // a building in the middle
  block(8, 14, 40, 46);
  block(48, 56, 16, 22);
  return occ;
}

sim_world::config world_config() {
  sim_world::config c;
  c.rows = R;
  c.cols = C;
  c.min_x = c.min_y = kLo;
  c.max_x = c.max_y = kHi;
  c.scale = kS;
  c.veh.rr = 0.05f;
  c.veh.d_hat = 0.2f;
  c.veh.dt = 0.06f;
  c.veh.vmax = static_cast<float>(8.0 * kS);
  c.veh.L = static_cast<float>(3.3 * kS);
  c.range_m = 25.0;
  c.n_rays = 90;
  c.sense_every = 4;
  return c;
}

sim_world make_world(std::shared_ptr<const coef_mlp> agent2 = nullptr, std::string key = "") {
  const std::vector<std::uint8_t> occ = city();
  const float o[6] = {static_cast<float>(-26 * kS), static_cast<float>(-4 * kS),
                      static_cast<float>(-26 * kS), static_cast<float>(4 * kS),
                      static_cast<float>(-20 * kS), static_cast<float>(20 * kS)};
  const float goal[6] = {static_cast<float>(26 * kS), static_cast<float>(2 * kS),
                         static_cast<float>(26 * kS), static_cast<float>(-6 * kS),
                         static_cast<float>(20 * kS), static_cast<float>(-22 * kS)};
  const float color[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  sim_world w(world_config(), occ.data(), occ.data(), coef_mlp::default_biased(), o, goal, color,
              3);
  if (agent2)
    w.set_policy(1, std::move(agent2), key);
  return w;
}

struct session {
  cvc::input_trace trace;
  std::vector<float> poses; // after every step: every agent's x, y, heading, speed
  long steps = 0;
};

void keep(const sim_world &w, std::vector<float> &poses) {
  for (int i = 0; i < w.size(); ++i) {
    poses.push_back(w.positions_normalized()[2 * i]);
    poses.push_back(w.positions_normalized()[2 * i + 1]);
    poses.push_back(w.headings()[i]);
    poses.push_back(w.speeds_normalized()[i]);
  }
}

// A live session, as a host runs one: uneven frames on a live clock, the UI's writes to nav.* and
// the host's verbs between frames, the world's settings pulled before every step and its
// telemetry published after each frame.
session record_session(sim_world w, const trace_options &o, const std::string &path = "") {
  cvc::app app;
  cvc::world_clock &clock = app.world_clock();
  cvc::world_clock::config cc;
  cc.fixed_dt = 0.01;
  cc.max_steps_per_advance = 100;
  clock.configure(cc);
  cvc::state &st = app.root();
  cvc::input_trace_recorder rec(app, clock);
  w.pull(app, ""); // the knobs, seeded now, start in the header's settings
  rec.start(trace_header(w, o), path);
  cvc::clock_follower nav = cvc::clock_follower::for_step(clock, 0.06);
  session s;
  const double frames[] = {1.0 / 60, 1.0 / 30, 1.0 / 144, 0.05, 1.0 / 60, 0.02, 1.0 / 90};
  for (int f = 0; f < 600; ++f) {
    if (f == 60)
      st("nav.vehicle.vmax").value(std::string("5"));
    if (f == 150)
      EXPECT_TRUE(
          trace_retarget(w, &rec, 2, static_cast<float>(24 * kS), static_cast<float>(20 * kS)));
    if (f == 220)
      trace_add_obstacle(w, &rec, 2, 6, 50, 56);
    if (f == 300)
      st("nav.sep_radius").value(std::string("3"));
    if (f == 330)
      st("nav.vehicle.vmax").value(std::string("7.5"));
    clock.advance(frames[f % 7]);
    for (std::uint64_t n = nav.take(clock); n > 0; --n) {
      w.pull(app, "");
      w.step(1);
      ++s.steps;
      keep(w, s.poses);
    }
    w.publish(app, "");
    rec.check(state_hash(w));
  }
  rec.finish(state_hash(w));
  s.trace = rec.trace();
  return s;
}

trace_options default_options() {
  trace_options o;
  o.world_id = "test:city64";
  o.seed = 7;
  o.policies[0] = policy_ref{}; // the world's own: the built-in default
  return o;
}

std::string temp_path(const char *name) {
  return (std::filesystem::temp_directory_path() / name).string();
}

// A policy whose alpha rises near a wall: one SiLU unit on the clearance feature (phi), far below
// zero (so exactly nothing) more than a few metres out. `gain` is the second build's difference:
// the two builds' coefficients agree until the agent first comes near a wall.
std::string wall_policy(float gain, const char *name) {
  const std::vector<float> w0 = {-200.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  const std::vector<float> w1 = {gain, 0, 0, 0, 0, 0};
  const coef_mlp net = coef_mlp::from_layers(5, 3, {2, 3}, {5, 2}, {1, 0}, {w0, w1},
                                             {{20.0f, 0.0f}, {0, 0, 0}}, {1.0f, 3.0f, 4.0f});
  const std::string path = temp_path(name);
  net.save(path);
  return path;
}

} // namespace

TEST(NavTrace, TheHeaderRebuildsTheWorld) {
  sim_world w = make_world();
  const cvc::input_trace_header h = trace_header(w, default_options());
  EXPECT_EQ(h.dt, static_cast<double>(0.06f));
  ASSERT_EQ(h.agents.size(), 3u);
  EXPECT_EQ(h.agents[1].id, 2u);
  EXPECT_EQ(h.agents[1].policy, "default");
  EXPECT_EQ(h.agents[0].start[0], static_cast<double>(static_cast<float>(-26 * kS)));
  EXPECT_NE(h.world.find("\"kind\":\"cvc.nav.sim_world/v1\",\"complete\":true"), std::string::npos);
  EXPECT_NE(h.world.find("\"prior\":{\"same_as\":\"truth\"}"), std::string::npos);

  policy_library lib;
  sim_world back = world_from_trace(cvc::input_trace_header::from_json(h.to_json()), lib);
  EXPECT_EQ(state_hash(back), state_hash(w));
  EXPECT_EQ(std::memcmp(back.truth(), w.truth(), static_cast<std::size_t>(R) * C), 0);
  for (int t = 0; t < 120; ++t) {
    w.step(1);
    back.step(1);
  }
  EXPECT_EQ(state_hash(back), state_hash(w)) << "the two step alike";
  EXPECT_NE(state_hash(back), state_hash(make_world())) << "and the hash sees them move";
}

TEST(NavTrace, ALiveSessionReplaysToByteIdenticalPoses) {
  const std::string path = temp_path("cvc_nav_trace_session.trace");
  const session live = record_session(make_world(), default_options(), path);
  const cvc::input_trace &t = live.trace;
  ASSERT_GT(live.steps, 200);
  ASSERT_TRUE(t.end.present);

  // The inputs: the UI's writes and the host's verbs, never the world's own telemetry.
  std::vector<std::string> sets, events;
  for (const cvc::input_trace_record &r : t.records) {
    if (r.type == cvc::input_trace_record::kind::set)
      sets.push_back(r.path);
    if (r.type == cvc::input_trace_record::kind::event)
      events.push_back(r.name);
  }
  EXPECT_EQ(sets,
            (std::vector<std::string>{"nav.vehicle.vmax", "nav.sep_radius", "nav.vehicle.vmax"}))
      << "the knobs start in the header; the records hold what changed";
  EXPECT_EQ(t.header.settings.at("nav.range_m"), "25");
  for (const std::string &p : sets) {
    EXPECT_NE(p, "nav.tick");
    EXPECT_NE(p.rfind("nav.settings", 0), 0u) << p;
    EXPECT_NE(p.rfind("nav.agents", 0), 0u) << p;
  }
  EXPECT_EQ(events, (std::vector<std::string>{"nav.retarget", "nav.add_obstacle"}));

  for (const cvc::input_trace &tr : {t, cvc::input_trace::read(path)}) {
    cvc::app app;
    replay_options ro;
    ro.keep_poses = true;
    const replay_result r = replay_trace(app, tr, ro);
    EXPECT_TRUE(r.matched) << describe(r);
    EXPECT_FALSE(r.diverged);
    EXPECT_EQ(r.hash, t.end.hash);
    EXPECT_EQ(r.steps, live.steps);
    EXPECT_EQ(r.ticks, t.end.tick);
    ASSERT_EQ(r.poses.size(), live.poses.size());
    EXPECT_EQ(std::memcmp(r.poses.data(), live.poses.data(), live.poses.size() * sizeof(float)), 0)
        << "byte-identical poses, step for step";
    for (std::size_t k = 0; k < r.pose_ticks.size(); ++k)
      ASSERT_EQ(r.pose_ticks[k], 6 * (k + 1)) << "each step on its own quantum";
  }
  // A step is bit-identical on any thread count (sense_batch and the drive are), so a session
  // recorded on one thread replays on four.
  cvc::app app;
  replay_options four;
  four.num_threads = 4;
  EXPECT_TRUE(replay_trace(app, t, four).matched);
  std::remove(path.c_str());
}

TEST(NavTrace, APerturbedTraceDivergesAtTheStepThatConsumesIt) {
  const session live = record_session(make_world(), default_options());
  const cvc::input_trace &t = live.trace;
  auto first_check_from = [&](std::uint64_t tick) {
    for (const cvc::input_trace_record &r : t.records)
      if (r.type == cvc::input_trace_record::kind::check && r.tick >= tick)
        return r.tick;
    return t.end.tick;
  };
  // `goal_now`: the edit shows in the hash as soon as it is applied (a goal is hashed), not only
  // once a step has consumed it.
  auto bend = [&](const char *what, bool goal_now, auto &&edit) {
    cvc::input_trace bent = t;
    std::uint64_t at = 0;
    for (cvc::input_trace_record &r : bent.records)
      if (edit(r)) {
        at = r.tick;
        break;
      }
    ASSERT_GT(at, 0u) << what;
    cvc::app app;
    replay_options ro;
    ro.keep_poses = true;
    const replay_result r = replay_trace(app, bent, ro);
    EXPECT_FALSE(r.matched) << what;
    EXPECT_TRUE(r.diverged) << what;
    const std::uint64_t step_at = (at / 6 + 1) * 6; // the first step on a quantum after `at`
    EXPECT_EQ(r.first_divergence, first_check_from(goal_now ? at + 1 : step_at)) << what << "\n"
                                                                                 << describe(r);
    // The poses are the live ones up to the step that consumes it, and not from there.
    const std::size_t per = 4 * 3, k_at = step_at / 6 - 1;
    ASSERT_EQ(r.poses.size(), live.poses.size());
    EXPECT_EQ(std::memcmp(r.poses.data(), live.poses.data(), k_at * per * sizeof(float)), 0)
        << what;
    EXPECT_NE(std::memcmp(r.poses.data() + k_at * per, live.poses.data() + k_at * per,
                          per * sizeof(float)),
              0)
        << what << ": the step at tick " << step_at;
  };
  // A different retarget goal.
  bend("the retarget", true, [](cvc::input_trace_record &r) {
    if (r.type != cvc::input_trace_record::kind::event || r.name != "nav.retarget")
      return false;
    r.args = "{\"agent\":2,\"goal\":[1.2,-0.2]}";
    return true;
  });
  // A different top speed, while the agents run at the recorded one.
  bend("the top speed", false, [](cvc::input_trace_record &r) {
    if (r.type != cvc::input_trace_record::kind::set || r.path != "nav.vehicle.vmax" ||
        r.value != "5")
      return false;
    r.value = "3";
    return true;
  });
}

TEST(NavTrace, TwoPolicyBuildsDifferOnlyWhereThePolicyDiffers) {
  // Agent 2 drives a policy of its own, from a file; the replay swaps that file for another
  // build of it.
  const std::string base = wall_policy(0.5f, "cvc_nav_trace_policy.cvcnav");
  const std::string other = wall_policy(1.0f, "cvc_nav_trace_policy2.cvcnav");
  policy_library lib;
  const policy_library::entry e = lib.get(policy_ref{base, ""});
  trace_options o = default_options();
  o.policies[1] = policy_ref{base, ""};
  // No sensing and no crowd: the agents do not interact, so a policy moves only its own agent.
  sim_world w = make_world(e.model, e.key);
  sim_world::config c = w.current_config();
  c.freeze_sense = true;
  w.set_live_knobs(c);
  const session live = record_session(std::move(w), o);
  const cvc::input_trace &t = live.trace;
  EXPECT_EQ(t.header.agents[1].policy, e.key);
  EXPECT_EQ(t.header.agents[0].policy, "default");

  {
    cvc::app app;
    const replay_result same = replay_trace(app, t);
    EXPECT_TRUE(same.matched) << describe(same);
  }
  cvc::app app;
  replay_options ro;
  ro.keep_poses = true;
  ro.policy_files[1] = other;
  const replay_result r = replay_trace(app, t, ro);
  EXPECT_TRUE(r.diverged) << describe(r);
  ASSERT_EQ(r.notes.size(), 1u);
  ASSERT_EQ(r.poses.size(), live.poses.size());

  // The step where the two builds' coefficients first differ, found by running both worlds on
  // the trace side by side.
  long coef_step = -1;
  {
    cvc::app a1, a2;
    policy_library l1, l2;
    sim_world w1 = world_from_trace(t.header, l1);
    sim_world w2 = world_from_trace(t.header, l2, {{1, other}});
    w1.set_capture_drive_telemetry(true);
    w2.set_capture_drive_telemetry(true);
    cvc::input_trace_player p1(a1, a1.world_clock(), t), p2(a2, a2.world_clock(), t);
    p1.on_event([&](const cvc::input_trace_record &ev) { apply_trace_event(w1, ev); });
    p2.on_event([&](const cvc::input_trace_record &ev) { apply_trace_event(w2, ev); });
    p1.rewind();
    p2.rewind();
    cvc::clock_follower f1(6, 0, 0), f2(6, 0, 0);
    long step = 0;
    while (!p1.done() && coef_step < 0) {
      p1.apply_due();
      p2.apply_due();
      p1.next_quantum();
      p2.next_quantum();
      const std::uint64_t n1 = f1.take(a1.world_clock());
      ASSERT_EQ(n1, f2.take(a2.world_clock()));
      for (std::uint64_t k = 0; k < n1 && coef_step < 0; ++k) {
        w1.pull(a1, "");
        w2.pull(a2, "");
        w1.step(1);
        w2.step(1);
        const drive_telemetry *t1 = w1.drive_telemetry_data(), *t2 = w2.drive_telemetry_data();
        for (int i = 0; i < 3; ++i)
          if (t1[i].alpha != t2[i].alpha || t1[i].beta != t2[i].beta ||
              t1[i].gamma != t2[i].gamma) {
            EXPECT_EQ(i, 1) << "only agent 2's coefficients differ";
            coef_step = step;
          }
        ++step;
      }
    }
  }
  ASSERT_GE(coef_step, 0) << "the other build's coefficients differ somewhere";
  EXPECT_GT(coef_step, 10) << "and not from the start: the builds agree away from walls";
  const std::size_t per = 4 * 3;
  long departs = -1; // the first step agent 2's pose differs
  for (long k = 0; k < live.steps; ++k) {
    const float *a = r.poses.data() + k * per, *b = live.poses.data() + k * per;
    // Agents 1 and 3, on the policy both builds share, drive the same whole session.
    ASSERT_EQ(std::memcmp(a, b, 4 * sizeof(float)), 0) << "agent 1, step " << k;
    ASSERT_EQ(std::memcmp(a + 8, b + 8, 4 * sizeof(float)), 0) << "agent 3, step " << k;
    if (departs < 0 && std::memcmp(a + 4, b + 4, 4 * sizeof(float)) != 0)
      departs = k;
  }
  // Agent 2 departs, and never before its coefficients differ (alpha prices the wall only inside
  // the barrier's range, so the pose can follow a few steps later).
  ASSERT_GE(departs, 0) << "agent 2 departs";
  EXPECT_GE(departs, coef_step);
  EXPECT_GE(r.first_divergence, 6u * static_cast<std::uint64_t>(coef_step + 1))
      << "and so does the hash";
  std::remove(base.c_str());
  std::remove(other.c_str());
}

TEST(NavTrace, AWorldV1CannotRebuildIsRefusedOrMarked) {
  sim_world w = make_world();
  trace_options none = default_options();
  none.policies.clear();
  EXPECT_THROW(trace_header(w, none), std::invalid_argument) << "the world's own policy unstated";
  w.step(1);
  try {
    trace_header(w, default_options());
    ADD_FAILURE() << "a stepped world";
  } catch (const std::invalid_argument &e) {
    EXPECT_NE(std::string(e.what()).find("the world has stepped (tick 1)"), std::string::npos)
        << e.what();
  }
  trace_options loose = default_options();
  loose.allow_incomplete = true;
  const cvc::input_trace_header h = trace_header(w, loose);
  EXPECT_NE(h.world.find("\"complete\":false,\"incomplete\":[\"the world has stepped"),
            std::string::npos)
      << h.world;
  policy_library lib;
  EXPECT_THROW(world_from_trace(h, lib), std::runtime_error);
  cvc::input_trace t;
  t.header = h;
  t.header.fixed_dt = 0.01;
  cvc::app app;
  EXPECT_THROW(replay_trace(app, t), std::runtime_error);

  // A host's own event stops the headless replay, saying whose it is.
  sim_world fresh = make_world();
  cvc::input_trace own;
  own.header = trace_header(fresh, default_options());
  own.header.fixed_dt = 0.01;
  own.records.push_back(cvc::input_trace_record::event(3, "host.spawn"));
  own.end.present = true;
  own.end.tick = 12;
  try {
    cvc::app a2;
    replay_trace(a2, own);
    ADD_FAILURE() << "replayed a host's event";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("host.spawn at tick 3 is not nav's"), std::string::npos)
        << e.what();
  }
}

TEST(NavTrace, NavEventsApplyByAgentId) {
  sim_world w = make_world();
  EXPECT_FALSE(trace_retarget(w, nullptr, 9, 0.0f, 0.0f)) << "no agent 9";
  EXPECT_TRUE(trace_set_pose(w, nullptr, 3, 0.1f, 0.2f, 0.5f, 0.25f));
  EXPECT_EQ(w.positions_normalized()[4], 0.1f);
  EXPECT_EQ(w.speeds_normalized()[2], 0.25f);
  sim_world v = make_world();
  EXPECT_TRUE(
      apply_trace_event(v, cvc::input_trace_record::event(
                               0, "nav.set_pose", R"({"agent":3,"pose":[0.1,0.2,0.5,0.25]})")));
  EXPECT_EQ(state_hash(v), state_hash(w));
  // An override is the next step's command, recorded and replayed alike.
  cvc::app app;
  cvc::input_trace_recorder rec(app, app.world_clock());
  rec.start(cvc::input_trace_header());
  EXPECT_TRUE(trace_override(w, &rec, 1, 0.3f, -0.05f));
  EXPECT_FALSE(trace_override(w, &rec, 9, 0.0f, 0.0f));
  const cvc::input_trace t = rec.trace();
  ASSERT_EQ(t.records.size(), 1u);
  EXPECT_EQ(t.records[0].args,
            R"({"agent":1,"command":[0.30000001192092896,-0.05000000074505806]})");
  EXPECT_TRUE(apply_trace_event(v, t.records[0]));
  w.step(1);
  v.step(1);
  EXPECT_EQ(state_hash(v), state_hash(w));
  EXPECT_FALSE(apply_trace_event(v, cvc::input_trace_record::event(0, "other.verb")));
  EXPECT_FALSE(apply_trace_event(v, cvc::input_trace_record::check(0, "x")));
  EXPECT_THROW(apply_trace_event(
                   v, cvc::input_trace_record::event(0, "nav.add_obstacle", R"({"rect":[1,2]})")),
               std::runtime_error);
  EXPECT_THROW(apply_trace_event(v, cvc::input_trace_record::event(0, "nav.retarget",
                                                                   R"({"agent":7,"goal":[0,0]})")),
               std::runtime_error);
  EXPECT_THROW(apply_trace_event(v, cvc::input_trace_record::event(0, "nav.retarget",
                                                                   R"({"agent":1,"goal":[0]})")),
               std::runtime_error);
}
