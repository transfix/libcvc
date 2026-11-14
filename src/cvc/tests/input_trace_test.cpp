/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::input_trace (cvc/core/input_trace.h): the cvc.input_trace/v1 lines round-trip, the
// recorder stamps each watched input with the clock's tick, and the player feeds them back at
// those ticks, so a run replays exactly and a perturbed trace departs where it was perturbed.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/clock_follower.h>
#include <cvc/core/input_trace.h>
#include <cvc/core/world_clock.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

cvc::input_trace sample_trace() {
  cvc::input_trace t;
  cvc::input_trace_header &h = t.header;
  h.seed = 9223372036854775813ull; // past 2^63: a seed survives JSON whole
  h.dt = 0.06;
  h.fixed_dt = 0.01;
  h.phase = 2;
  h.start_tick = 12;
  h.world_id = "grid:\"quoted\"\n";
  h.world = R"({"kind":"test","nested":{"a":[1,2.5,-3e-7],"b":null,"c":true}})";
  cvc::input_trace_agent a;
  a.id = 1;
  a.start = {0.1, -0.2, 3.141592653589793, 0.0};
  a.goal = {0.3f, 0.7f};
  a.policy = "default";
  h.agents.push_back(a);
  a.id = 2;
  a.start = {std::nan(""), 1e-310};
  a.goal = {};
  a.policy = "sha256:" + std::string(64, 'a');
  h.agents.push_back(a);
  h.settings = {{"nav.range_m", "60"}, {"nav.vehicle.vmax", "8.5"}, {"run.note", "tab\there"}};
  h.watch = {"nav", "run"};
  h.meta = R"({"host":"test"})";
  t.records.push_back(cvc::input_trace_record::set(12, "nav.vehicle.vmax", "6"));
  t.records.push_back(
      cvc::input_trace_record::event(30, "nav.retarget", R"({"agent": 2, "goal": [0.5, -0.25]})"));
  t.records.push_back(cvc::input_trace_record::check(30, "sha256:abc"));
  t.records.push_back(cvc::input_trace_record::set(30, "run.note", ""));
  t.end.present = true;
  t.end.tick = 120;
  t.end.hash = "sha256:def";
  return t;
}

} // namespace

TEST(InputTrace, TheLinesRoundTripExactly) {
  const cvc::input_trace t = sample_trace();
  const std::string text = t.str();
  // One JSON object a line: the header, four records, the end.
  EXPECT_EQ(std::count(text.begin(), text.end(), '\n'), 6);
  EXPECT_EQ(text.rfind(R"({"schema":"cvc.input_trace/v1","header":{)", 0), 0u);
  EXPECT_NE(text.find(R"({"tick":12,"set":{"path":"nav.vehicle.vmax","value":"6"}})"),
            std::string::npos);
  EXPECT_NE(
      text.find(
          R"({"tick":30,"event":{"name":"nav.retarget","args":{"agent":2,"goal":[0.5,-0.25]}}})"),
      std::string::npos)
      << "an event's args are kept compact";
  EXPECT_NE(text.find(R"({"end":{"tick":120,"ticks":108,"hash":"sha256:def"}})"),
            std::string::npos);

  const cvc::input_trace back = cvc::input_trace::parse(text);
  EXPECT_EQ(back.str(), text) << "a parsed trace writes the same bytes";
  const cvc::input_trace_header &h = back.header;
  EXPECT_EQ(h.seed, 9223372036854775813ull);
  EXPECT_EQ(h.dt, 0.06);
  EXPECT_EQ(h.fixed_dt, 0.01);
  EXPECT_EQ(h.phase, 2u);
  EXPECT_EQ(h.start_tick, 12u);
  EXPECT_EQ(h.world_id, "grid:\"quoted\"\n");
  ASSERT_EQ(h.agents.size(), 2u);
  EXPECT_EQ(h.agents[0].start[2], 3.141592653589793);
  EXPECT_EQ(h.agents[0].goal[0], static_cast<double>(0.3f)) << "a float survives as itself";
  EXPECT_TRUE(std::isnan(h.agents[1].start[0])) << "NaN is written as null";
  EXPECT_EQ(h.agents[1].start[1], 1e-310) << "a denormal reads back";
  EXPECT_EQ(h.agents[1].policy, "sha256:" + std::string(64, 'a'));
  EXPECT_EQ(h.settings.at("run.note"), "tab\there");
  EXPECT_EQ(h.watch, (std::vector<std::string>{"nav", "run"}));
  EXPECT_EQ(h.meta, R"({"host":"test"})");
  ASSERT_EQ(back.records.size(), 4u);
  EXPECT_EQ(back.records[1].type, cvc::input_trace_record::kind::event);
  EXPECT_EQ(back.records[1].args, R"({"agent":2,"goal":[0.5,-0.25]})");
  EXPECT_EQ(back.records[2].type, cvc::input_trace_record::kind::check);
  EXPECT_EQ(back.records[3].value, "");
  EXPECT_TRUE(back.end.present);
  EXPECT_EQ(back.last_tick(), 120u);

  // The header alone, as a Python host hands it over.
  const cvc::input_trace_header h2 = cvc::input_trace_header::from_json(t.header.to_json());
  EXPECT_EQ(h2.to_json(), t.header.to_json());

  // A trace with no end (a recording that never finished) reads, and stops at its last record.
  cvc::input_trace open = t;
  open.end = cvc::input_trace_end{};
  const cvc::input_trace o2 = cvc::input_trace::parse(open.str());
  EXPECT_FALSE(o2.end.present);
  EXPECT_EQ(o2.last_tick(), 30u);

  // And through a file.
  const std::string path =
      (std::filesystem::temp_directory_path() / "cvc_input_trace_roundtrip.trace").string();
  t.save(path);
  EXPECT_EQ(cvc::input_trace::read(path).str(), text);
  std::remove(path.c_str());

  const std::string info = cvc::describe(t);
  EXPECT_NE(info.find("2 sets, 1 events, 1 checks"), std::string::npos) << info;
  EXPECT_NE(info.find("nav.retarget x1"), std::string::npos) << info;
  EXPECT_NE(info.find("12 .. 120 (108 quanta"), std::string::npos) << info;
}

TEST(InputTrace, ParseNamesWhatIsWrong) {
  const std::string good = sample_trace().str();
  auto fails = [](const std::string &text, const std::string &want) {
    try {
      cvc::input_trace::parse(text);
    } catch (const std::runtime_error &e) {
      EXPECT_NE(std::string(e.what()).find(want), std::string::npos) << e.what();
      return;
    }
    ADD_FAILURE() << "parsed: " << text;
  };
  fails("", "no header");
  fails("{\"schema\":\"cvc.input_trace/v2\",\"header\":{}}\n", "the schema is cvc.input_trace/v2");
  fails("{\"header\":{}}\n", "no schema");
  fails("{\"schema\":\"cvc.input_trace/v1\",\"header\":{}}\nnot json\n", "line 2");
  fails("{\"schema\":\"cvc.input_trace/v1\",\"header\":{}}\n{\"tick\":5,\"set\":{\"path\":\"a\","
        "\"value\":\"1\"}}\n{\"tick\":4,\"check\":\"x\"}\n",
        "tick 4 is before tick 5");
  fails("{\"schema\":\"cvc.input_trace/v1\",\"header\":{}}\n{\"tick\":1,\"poke\":{}}\n",
        "none of set, event, check or end");
  fails(good + "{\"tick\":200,\"check\":\"x\"}\n", "a line after the end");
  fails("{\"schema\":\"cvc.input_trace/v1\",\"header\":{\"seed\":-1}}\n", "header.seed");
  // Blank lines and CRLF line ends are fine.
  std::string crlf;
  for (char c : good) {
    if (c == '\n')
      crlf += "\r\n\r\n";
    else
      crlf += c;
  }
  EXPECT_EQ(cvc::input_trace::parse(crlf).str(), good);
  EXPECT_THROW(cvc::input_trace_record::event(0, "x", "[1]"), std::invalid_argument);
  EXPECT_THROW(cvc::input_trace_record::event(0, "x", "{"), std::invalid_argument);
}

TEST(InputTraceRecorder, StampsWatchedInputsWithTheClocksTick) {
  cvc::app app;
  cvc::world_clock &clock = app.world_clock();
  cvc::state &st = app.root();
  st("nav.range_m").value(std::string("60"));
  st("nav.tick").value(std::string("0"));
  st("nav.tick").property("access", "telemetry");
  st("other.x").value(std::string("1"));

  const std::string path =
      (std::filesystem::temp_directory_path() / "cvc_input_trace_recorder.trace").string();
  cvc::input_trace_recorder rec(app, clock);
  cvc::input_trace_header h;
  h.seed = 3;
  h.dt = 0.03;
  h.world_id = "toy";
  clock.advance_ticks(4);
  rec.start(h, path);
  EXPECT_TRUE(rec.recording());

  st("nav.range_m").value(std::string("80"));      // tick 4
  st("nav.range_m").value(std::string("80"));      // unchanged: nothing
  st("other.x").value(std::string("2"));           // not watched
  st("nav.tick").value(std::string("4"));          // telemetry: an output
  st("nav.agents.1.name").value(std::string("a")); // ignored by default
  st("nav_plant").value(std::string("p"));         // the prefix itself is watched
  st("nav_plantx.y").value(std::string("q"));      // a different key that shares its letters
  clock.advance_ticks(3);
  st("run.mode").value(std::string("go")); // tick 7
  rec.event("host.kick", R"({"dx": 0.5})");
  rec.check("sha256:7");
  clock.advance_ticks(2);
  rec.finish("sha256:9");
  EXPECT_FALSE(rec.recording());
  EXPECT_TRUE(rec.finished());
  st("nav.range_m").value(std::string("90")); // after the end: nothing
  EXPECT_THROW(rec.start(h), std::logic_error);

  const cvc::input_trace t = rec.trace();
  EXPECT_EQ(t.header.fixed_dt, clock.fixed_dt());
  EXPECT_EQ(t.header.start_tick, 4u);
  EXPECT_EQ(t.header.settings.at("nav.range_m"), "60");
  EXPECT_EQ(t.header.settings.count("nav.tick"), 0u) << "telemetry is not a setting";
  EXPECT_EQ(t.header.settings.count("other.x"), 0u);
  EXPECT_EQ(t.header.watch, (std::vector<std::string>{"nav", "nav_plant", "run", "world"}));
  EXPECT_NE(t.header.build.find("\"libcvc\":\""), std::string::npos) << t.header.build;
  ASSERT_EQ(t.records.size(), 5u) << t.str();
  EXPECT_EQ(t.records[0].to_json(), R"({"tick":4,"set":{"path":"nav.range_m","value":"80"}})");
  EXPECT_EQ(t.records[1].to_json(), R"({"tick":4,"set":{"path":"nav_plant","value":"p"}})");
  EXPECT_EQ(t.records[2].to_json(), R"({"tick":7,"set":{"path":"run.mode","value":"go"}})");
  EXPECT_EQ(t.records[3].to_json(), R"({"tick":7,"event":{"name":"host.kick","args":{"dx":0.5}}})");
  EXPECT_EQ(t.records[4].to_json(), R"({"tick":7,"check":"sha256:7"})");
  EXPECT_TRUE(t.end.present);
  EXPECT_EQ(t.end.tick, 9u);
  EXPECT_EQ(t.end.hash, "sha256:9");

  // The file holds the same lines, written as they were recorded.
  std::ifstream f(path, std::ios::binary);
  std::stringstream file;
  file << f.rdbuf();
  EXPECT_EQ(file.str(), t.str());
  std::remove(path.c_str());
}

TEST(InputTraceRecorder, OnSampleRecordsWhatTheHostConsumed) {
  cvc::app app;
  cvc::world_clock &clock = app.world_clock();
  cvc::state &st = app.root();
  cvc::input_trace_recorder::options o;
  o.stamp = cvc::input_trace_recorder::stamping::on_sample;
  o.watch = {"run."};
  o.ignore = {};
  cvc::input_trace_recorder rec(app, clock, o);
  rec.start(cvc::input_trace_header());
  st("run.a").value(std::string("1"));
  clock.advance_ticks(2);
  st("run.a").value(std::string("2")); // two writes before a sample: the last one counts
  st("run.b").value(std::string("x"));
  EXPECT_EQ(rec.size(), 0u);
  rec.sample(); // tick 2
  st("run.b").value(std::string("y"));
  st("run.b").value(std::string("x")); // back to what was recorded: nothing to record
  clock.advance_ticks(1);
  rec.sample();
  st("run.a").value(std::string("3"));
  clock.advance_ticks(1);
  rec.finish("h"); // the last sample, at the end's tick
  const cvc::input_trace t = rec.trace();
  ASSERT_EQ(t.records.size(), 3u) << t.str();
  EXPECT_EQ(t.records[0].to_json(), R"({"tick":2,"set":{"path":"run.a","value":"2"}})");
  EXPECT_EQ(t.records[1].to_json(), R"({"tick":2,"set":{"path":"run.b","value":"x"}})");
  EXPECT_EQ(t.records[2].to_json(), R"({"tick":4,"set":{"path":"run.a","value":"3"}})");
  EXPECT_EQ(t.header.watch, (std::vector<std::string>{"run"}));
}

TEST(InputTraceRecorder, AClockThatGoesBackBreaksTheRecording) {
  cvc::app app;
  cvc::world_clock &clock = app.world_clock();
  cvc::input_trace_recorder rec(app, clock);
  clock.advance_ticks(10);
  rec.start(cvc::input_trace_header());
  clock.reset();
  app.root()("run.x").value(std::string("1"));
  EXPECT_FALSE(rec.recording());
  try {
    rec.finish("h");
    ADD_FAILURE() << "finished";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("went back from tick 10 to 0"), std::string::npos)
        << e.what();
  }
}

namespace {

// A toy simulation on the clock: every 3 quanta, x += v * dt, v read from run.v; the event
// toy.kick moves x at once. Its state hash is x's bits.
struct toy {
  double x = 0.0;
  std::vector<double> trail; // x after every step
  void step(cvc::app &app) {
    const std::string v = app.root()("run.v").value();
    x += (v.empty() ? 0.0 : std::stod(v)) * 0.03;
    trail.push_back(x);
  }
  std::string hash() const {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &x, sizeof bits);
    return std::to_string(bits);
  }
};

cvc::input_trace record_toy(std::vector<double> &trail) {
  cvc::app app;
  cvc::world_clock &clock = app.world_clock();
  cvc::world_clock::config cc;
  cc.fixed_dt = 0.01;
  cc.max_steps_per_advance = 100;
  clock.configure(cc);
  toy sim;
  cvc::input_trace_recorder rec(app, clock);
  cvc::input_trace_header h;
  h.dt = 0.03;
  h.world_id = "toy";
  app.root()("run.v").value(std::string("1"));
  rec.start(h);
  cvc::clock_follower f = cvc::clock_follower::for_step(clock, 0.03);
  // Live frames of uneven length, the inputs between them.
  const double frames[] = {0.016, 0.033, 0.007, 0.05, 0.02, 0.011};
  for (int k = 0; k < 60; ++k) {
    if (k == 10)
      app.root()("run.v").value(std::string("-2"));
    if (k == 25) {
      sim.x += 5.0; // the host's verb, and its record
      rec.event("toy.kick", R"({"dx":5})");
    }
    if (k == 40)
      app.root()("run.v").value(std::string("0.25"));
    clock.advance(frames[k % 6]);
    for (std::uint64_t n = f.take(clock); n > 0; --n)
      sim.step(app);
    rec.check(sim.hash());
  }
  rec.finish(sim.hash());
  trail = sim.trail;
  return rec.trace();
}

std::vector<double> replay_toy(const cvc::input_trace &t, bool *matched, std::uint64_t *first) {
  cvc::app app;
  cvc::world_clock &clock = app.world_clock();
  toy sim;
  cvc::input_trace_player play(app, clock, t);
  play.on_event([&](const cvc::input_trace_record &e) {
    EXPECT_EQ(e.name, "toy.kick");
    sim.x += 5.0;
  });
  play.set_state_hash([&] { return sim.hash(); });
  play.rewind();
  EXPECT_EQ(clock.current_mode(), cvc::world_clock::mode::replay);
  EXPECT_EQ(clock.fixed_dt(), 0.01);
  cvc::clock_follower f(cvc::ticks_per_step(t.header.dt, t.header.fixed_dt), t.header.phase,
                        clock.tick());
  play.run([&](std::uint64_t) {
    for (std::uint64_t n = f.take(clock); n > 0; --n)
      sim.step(app);
  });
  EXPECT_TRUE(play.done());
  *matched = play.matched();
  *first = play.first_divergence();
  return sim.trail;
}

} // namespace

TEST(InputTracePlayer, AReplayReproducesTheRunAndAPerturbationDepartsWhereItWasMade) {
  std::vector<double> live;
  const cvc::input_trace t = record_toy(live);
  ASSERT_GT(live.size(), 40u);
  std::size_t sets = 0, events = 0;
  std::set<std::uint64_t> checked;
  for (const cvc::input_trace_record &r : t.records) {
    sets += r.type == cvc::input_trace_record::kind::set;
    events += r.type == cvc::input_trace_record::kind::event;
    if (r.type == cvc::input_trace_record::kind::check)
      EXPECT_TRUE(checked.insert(r.tick).second) << "one check a tick, at tick " << r.tick;
  }
  EXPECT_EQ(sets, 2u);
  EXPECT_EQ(events, 1u);
  EXPECT_GT(checked.size(), 40u) << "a check a frame that ran a quantum";

  bool matched = false;
  std::uint64_t first = 0;
  const std::vector<double> again = replay_toy(t, &matched, &first);
  EXPECT_TRUE(matched);
  ASSERT_EQ(again.size(), live.size());
  EXPECT_EQ(std::memcmp(again.data(), live.data(), live.size() * sizeof(double)), 0)
      << "byte-identical steps";

  // Through the file format too.
  const std::vector<double> parsed = replay_toy(cvc::input_trace::parse(t.str()), &matched, &first);
  EXPECT_TRUE(matched);
  EXPECT_EQ(parsed, live);

  // Perturb the second set: the replay departs at the first step after its tick, and the first
  // check that disagrees is the first at or after that step.
  cvc::input_trace bent = t;
  std::uint64_t at = 0;
  for (cvc::input_trace_record &r : bent.records)
    if (r.type == cvc::input_trace_record::kind::set && r.value == "-2") {
      r.value = "-2.5";
      at = r.tick;
    }
  ASSERT_GT(at, 0u);
  const std::vector<double> off = replay_toy(bent, &matched, &first);
  EXPECT_FALSE(matched);
  const std::uint64_t step_at = (at / 3 + 1) * 3; // the first step on a quantum after `at`
  std::uint64_t first_check = 0;
  for (const cvc::input_trace_record &r : t.records)
    if (r.type == cvc::input_trace_record::kind::check && r.tick >= step_at) {
      first_check = r.tick;
      break;
    }
  EXPECT_EQ(first, first_check);
  ASSERT_EQ(off.size(), live.size());
  const std::size_t k_at = step_at / 3 - 1; // step k runs on tick 3 (k + 1)
  for (std::size_t k = 0; k < k_at; ++k)
    EXPECT_EQ(off[k], live[k]) << "step " << k << ", before the perturbation";
  EXPECT_NE(off[k_at], live[k_at]) << "the step that consumes it";
}

TEST(InputTracePlayer, StepsQuantumByQuantumForAHostLoop) {
  std::vector<double> live;
  const cvc::input_trace t = record_toy(live);
  cvc::app app;
  cvc::world_clock &clock = app.world_clock();
  toy sim;
  cvc::input_trace_player play(app, clock, t); // no handler: the events come back
  play.rewind();
  EXPECT_EQ(app.root()("run.v").value(), "1") << "the header's settings are written";
  cvc::clock_follower f(3, 0, clock.tick());
  std::size_t events = 0;
  while (!play.done()) {
    for (const cvc::input_trace_record &e : play.apply_due()) {
      ++events;
      EXPECT_EQ(e.name, "toy.kick");
      sim.x += 5.0;
    }
    play.next_quantum();
    for (std::uint64_t n = f.take(clock); n > 0; --n)
      sim.step(app);
    if (!play.expected_hash().empty())
      EXPECT_TRUE(play.verify(sim.hash())) << "tick " << clock.tick();
  }
  EXPECT_EQ(events, 1u);
  EXPECT_EQ(sim.trail, live);
  EXPECT_TRUE(play.matched());
  std::set<std::uint64_t> hashed{t.end.tick}; // a frame with no quantum checks its tick again
  for (const cvc::input_trace_record &r : t.records)
    if (r.type == cvc::input_trace_record::kind::check)
      hashed.insert(r.tick);
  EXPECT_EQ(play.compared(), hashed.size()) << "every check and the end, once a tick";
  EXPECT_THROW(play.next_quantum(), std::logic_error);

  // Rewound, it plays again from the start (a sim without save/restore rebuilds itself first).
  toy fresh;
  play.rewind();
  f.reset(clock.tick());
  play.on_event([&](const cvc::input_trace_record &) { fresh.x += 5.0; });
  play.set_state_hash([&] { return fresh.hash(); });
  play.run([&](std::uint64_t) {
    for (std::uint64_t n = f.take(clock); n > 0; --n)
      fresh.step(app);
  });
  EXPECT_EQ(fresh.trail, live);
  EXPECT_TRUE(play.matched());
}
