/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// physics_replay_test: the native <-> wasm replay gate (the vehicle-physics plan's P7,
// docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md).
//
// A trace is what a physics run did, step by step: the state_hash after every step of a few
// scenes. It is recorded once (`physics_replay_test --record <trace>`) and checked in as
// tests/data/physics_replay_trace.json. Replaying it (`physics_replay_test <trace>`) makes the
// same calls and requires the same hash after every step: natively under ctest on every
// platform, and as wasm-mt under node (ci.yml's physics-wasm job). The first step that differs
// shows where a determinism leak entered.
//
// The scenes cover what a cvc::physics step does beyond the engine:
//   exact    the golden scene (physics_world_test's CrossPlatformGoldenHash): terrain and
//            nine bodies, pushed;
//   humvee   a Humvee through the driver model (steering actuator, Ackermann, engine,
//            gearbox, energy ledger), with a rebase of the floating origin halfway;
//   roboracer  a RoboRacer on raw pedals over terrain with per-cell materials, through a
//            medium that keeps ruts (Bekker sinkage), past a box floating in water (buoyancy
//            and the field's implicit drag).
// Every input is an exactly representable number, so the calls are the same on every
// platform, and what the step does with them must be too. Each run also checks that
// save()/restore() reproduces the roboracer's second half.
//
// A plain main(), not googletest: googletest has no wasm build. Exit codes: 0 when the trace
// matches, 1 on a mismatch or a bad trace, 77 (ctest's skip) when the build has no
// cross-platform-deterministic Jolt.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/geometry/height_field.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/world.h>
#include <cvc/volume/volume.h>
#include <exception>
#include <memory>
#include <string>
#include <vector>

using namespace cvc::physics;

namespace {

constexpr const char *kFormat = "cvc.physics.replay/1";

using hashes = std::vector<std::uint64_t>;

struct scene_run {
  hashes steps;        // the state_hash after each step
  std::string summary; // what happened, for the log
};

std::string hex(std::uint64_t h) {
  char b[24];
  std::snprintf(b, sizeof(b), "%016llx", static_cast<unsigned long long>(h));
  return b;
}

const char *platform() {
#if defined(__EMSCRIPTEN__)
  return "wasm32 (emscripten)";
#elif defined(_MSC_VER) && defined(_M_X64)
  return "x86_64 (msvc)";
#elif defined(_MSC_VER) && defined(_M_ARM64)
  return "arm64 (msvc)";
#elif defined(__APPLE__) && defined(__aarch64__)
  return "arm64 (apple clang)";
#elif defined(__APPLE__) && defined(__x86_64__)
  return "x86_64 (apple clang)";
#elif defined(__clang__) && defined(__x86_64__)
  return "x86_64 (clang)";
#elif defined(__clang__) && defined(__aarch64__)
  return "arm64 (clang)";
#elif defined(__GNUC__) && defined(__x86_64__)
  return "x86_64 (gcc)";
#elif defined(__GNUC__) && defined(__aarch64__)
  return "arm64 (gcc)";
#else
  return "unknown";
#endif
}

cvc::app &ctx() {
  static cvc::app app;
  return app;
}

// A uniform volume over the box [lo, hi]: a field's region and strength.
std::shared_ptr<cvc::volume> uniform(double value, vec3 lo, vec3 hi) {
  auto v = std::make_shared<cvc::volume>(ctx(), cvc::dimension(2, 2, 2), cvc::Float,
                                         cvc::bounding_box(lo.x, lo.y, lo.z, hi.x, hi.y, hi.z));
  v->fill(value);
  return v;
}

// --- the scenes -------------------------------------------------------------------------------

// exact: a height field of exactly representable heights and nine bodies, pushed for 240 steps.
// Its last hash is CrossPlatformGoldenHash's 70b2cc0bd34ea4be.
scene_run exact_scene() {
  world w; // Jolt, cross-platform determinism, 0.01 s
  std::vector<float> heights(17 * 17);
  for (int r = 0; r < 17; ++r)
    for (int c = 0; c < 17; ++c)
      heights[r * 17 + c] = static_cast<float>((r % 5) * 0.125 + (c % 3) * 0.25);
  w.add_body({terrain{std::make_shared<cvc::height_field>(17, 17, -4.0, -4.0, 0.5, 0.5, heights)},
              motion_type::static_});
  std::vector<body_id> bodies;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      body_desc d;
      if ((i + j) % 2)
        d.shape = sphere{0.25};
      else
        d.shape = box{{0.25, 0.125, 0.25}};
      d.position = {i * 1.5 - 1.5, j * 1.5 - 1.5, 2.0 + 0.5 * (i + j)};
      d.linear_velocity = {0.25 * (i - 1), -0.25 * (j - 1), 0.0};
      bodies.push_back(w.add_body(d));
    }
  scene_run run;
  for (int s = 0; s < 240; ++s) {
    w.add_force(bodies[static_cast<size_t>(s) % bodies.size()],
                {(s % 7 - 3) * 2.0, (s % 5 - 2) * 4.0, 0.0});
    w.step();
    run.steps.push_back(w.state_hash());
  }
  run.summary = "nine bodies on terrain, 240 steps";
  return run;
}

// humvee: 500 steps on a flat box (mu 0.9, C_rr 0.015) through the driver model: left, then
// right, then straight; 1.5 m/s^2 for 3.5 s, then -3 m/s^2. At step 250 the floating origin
// moves 8 m along x.
scene_run humvee_scene() {
  world w;
  const material_id m = w.add_material({"ground", 0.9f, 0.0f, 0.015f});
  body_desc g;
  g.shape = box{{150.0, 150.0, 0.5}};
  g.motion = motion_type::static_;
  g.position = {0.0, 0.0, -0.5};
  g.material = m;
  w.add_body(g);
  const vehicle_id car = w.add_vehicle(vehicle_desc::humvee(), {0.0, 0.0, 1.2});
  scene_run run;
  for (int s = 0; s < 500; ++s) {
    if (s == 250)
      w.rebase({8.0, 0.0, 0.0});
    drive_command c;
    c.steer = s < 150 ? 0.0625 : s < 300 ? -0.125 : 0.0;
    c.accel = s < 350 ? 1.5 : -3.0;
    w.drive(car, c);
    w.step();
    run.steps.push_back(w.state_hash());
  }
  const vehicle_state st = w.vehicle_state(car);
  char b[160];
  std::snprintf(b, sizeof(b), "drove %.2f m, %.2f m/s at the end, gear %d, %.0f J of fuel",
                st.energy.distance, st.speed, st.gear, st.energy.source);
  run.summary = b;
  return run;
}

// roboracer's raw pedals, by step.
vehicle_controls pedals(int s) {
  vehicle_controls c;
  if (s < 100) {
    c.throttle = 0.25;
  } else if (s < 200) {
    c.throttle = 0.375;
    c.steer = 0.25;
  } else if (s < 260) {
    c.brake = 0.5;
    c.steer = -0.125;
  } else if (s < 380) {
    c.throttle = 0.5;
    c.steer = -0.25;
  } else if (s < 420) {
    c.hand_brake = 0.5;
  } else {
    c.throttle = 0.25;
    c.reverse = true;
  }
  return c;
}

// roboracer: 500 steps. A 16 m height field of bumps up to 2 cm, in 4 m squares of asphalt
// (mu 0.9, C_rr 0.015) and grass (mu 0.6, C_rr 0.06); a band of mud that keeps ruts across
// 1 m < x < 3 m; a pool with a floating box west of the start.
scene_run roboracer_scene() {
  world w;
  const material_id asphalt = w.add_material({"asphalt", 0.9f, 0.0f, 0.015f});
  const material_id grass = w.add_material({"grass", 0.6f, 0.0f, 0.06f});
  const int n = 65;
  std::vector<float> heights(n * n);
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c)
      heights[r * n + c] = static_cast<float>((r % 4) * 0.00390625 + (c % 6) * 0.001953125);
  std::vector<cvc::height_field::face_material_t> cells((n - 1) * (n - 1));
  for (int r = 0; r < n - 1; ++r)
    for (int c = 0; c < n - 1; ++c)
      cells[r * (n - 1) + c] =
          static_cast<cvc::height_field::face_material_t>((r / 16 + c / 16) % 2);
  w.add_body(
      {terrain{std::make_shared<cvc::height_field>(n, n, -8.0, -8.0, 0.25, 0.25, heights, cells),
               {asphalt, grass}},
       motion_type::static_});
  field mud;
  mud.kind = field_kind::medium;
  mud.volume = uniform(2.0e6, {1.0, -8.0, -0.25}, {3.0, 8.0, 0.03125});
  mud.damping = 2.0e4;
  mud.traction = 0.4;
  mud.ruts = true;
  mud.rut_cell = 0.03125;
  w.add_field(mud);
  field water;
  water.kind = field_kind::fluid;
  water.volume = uniform(1000.0, {-6.0, -6.0, -0.5}, {-4.0, -4.0, 0.5});
  water.damping = 1.0e3;
  w.add_field(water);
  body_desc crate;
  crate.shape = box{{0.125, 0.125, 0.0625}};
  crate.position = {-5.0, -5.0, 1.0};
  crate.density = 500.0;
  crate.allow_sleeping = false; // it bobs on the water, through the field's step, to the end
  const body_id crate_id = w.add_body(crate);

  const vehicle_desc rr = vehicle_desc::roboracer();
  double z0 = 0.0;
  for (const axle_desc &a : rr.axles)
    z0 = std::max(z0, -a.z + a.suspension_max + a.wheel_radius);
  const vehicle_id car = w.add_vehicle(rr, {-1.0, 0.0, 0.03125 + z0});

  scene_run run;
  std::vector<std::uint8_t> half;
  double sunk = 0.0;
  for (int s = 0; s < 500; ++s) {
    if (s == 250)
      half = w.save();
    w.set_controls(car, pedals(s));
    w.step();
    run.steps.push_back(w.state_hash());
    for (const wheel_state &ws : w.vehicle_state(car).wheels)
      sunk = std::max(sunk, ws.sinkage);
  }
  const vehicle_state st = w.vehicle_state(car);
  char b[240];
  std::snprintf(b, sizeof(b),
                "drove %.2f m to (%.2f, %.2f), %.2f m/s at the end, %.1f J from the battery, "
                "%.1f mm into the mud at most; the crate floats at z %.3f",
                st.energy.distance, st.body.position.x, st.body.position.y, st.speed,
                st.energy.source, 1000.0 * sunk, w.state(crate_id).position.z);
  run.summary = b;

  // save()/restore(): the second half again, from the snapshot, must be the same steps.
  w.restore(half);
  for (int s = 250; s < 500; ++s) {
    w.set_controls(car, pedals(s));
    w.step();
    if (w.state_hash() != run.steps[static_cast<size_t>(s)]) {
      run.summary += "; RESTORE DIVERGED at step " + std::to_string(s);
      run.steps.clear(); // fails the run whatever the trace says
      break;
    }
  }
  return run;
}

struct scene {
  const char *name;
  scene_run (*run)();
};

const scene kScenes[] = {
    {"exact", exact_scene},
    {"humvee", humvee_scene},
    {"roboracer", roboracer_scene},
};

// --- record, replay ---------------------------------------------------------------------------

int record(const std::string &path) {
  std::FILE *f = std::fopen(path.c_str(), "w");
  if (!f) {
    std::fprintf(stderr, "physics_replay: cannot write %s\n", path.c_str());
    return 1;
  }
  std::fprintf(f, "{\n  \"format\": \"%s\",\n  \"recorded_on\": \"%s\",\n  \"scenes\": {", kFormat,
               platform());
  bool first = true;
  for (const scene &sc : kScenes) {
    const scene_run a = sc.run(), b = sc.run();
    if (a.steps.empty() || a.steps != b.steps) {
      size_t i = 0;
      while (i < a.steps.size() && i < b.steps.size() && a.steps[i] == b.steps[i])
        ++i;
      std::fprintf(stderr,
                   "physics_replay: %s is not deterministic in-process: two runs part at step "
                   "%zu (%s)\n",
                   sc.name, i + 1, a.summary.c_str());
      std::fclose(f);
      return 1;
    }
    std::fprintf(f, "%s\n    \"%s\": [", first ? "" : ",", sc.name);
    for (size_t i = 0; i < a.steps.size(); ++i)
      std::fprintf(f, "%s\"%s\"", i == 0 ? "" : (i % 6 == 0 ? ",\n      " : ", "),
                   hex(a.steps[i]).c_str());
    std::fprintf(f, "]");
    std::printf("recorded %-10s %4zu steps, last %s: %s\n", sc.name, a.steps.size(),
                hex(a.steps.back()).c_str(), a.summary.c_str());
    first = false;
  }
  std::fprintf(f, "\n  }\n}\n");
  std::fclose(f);
  std::printf("wrote %s on %s\n", path.c_str(), platform());
  return 0;
}

int replay(const std::string &path) {
  boost::property_tree::ptree doc;
  try {
    boost::property_tree::read_json(path, doc);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "physics_replay: cannot read %s: %s\n", path.c_str(), e.what());
    return 1;
  }
  if (doc.get<std::string>("format", "") != kFormat) {
    std::fprintf(stderr, "physics_replay: %s is not a %s trace\n", path.c_str(), kFormat);
    return 1;
  }
  const boost::property_tree::ptree none;
  const boost::property_tree::ptree &scenes = doc.get_child("scenes", none);
  std::printf("replaying %s (recorded on %s) on %s\n", path.c_str(),
              doc.get<std::string>("recorded_on", "?").c_str(), platform());
  int failures = 0;
  size_t total = 0;
  for (const scene &sc : kScenes) {
    const auto it = scenes.find(sc.name);
    if (it == scenes.not_found()) {
      std::printf("FAIL %-10s not in the trace: record it again\n", sc.name);
      ++failures;
      continue;
    }
    hashes want;
    for (const auto &h : it->second)
      want.push_back(std::stoull(h.second.get_value<std::string>(), nullptr, 16));
    const scene_run got = sc.run();
    size_t i = 0;
    while (i < want.size() && i < got.steps.size() && want[i] == got.steps[i])
      ++i;
    if (i == want.size() && i == got.steps.size()) {
      std::printf("ok   %-10s %4zu steps, last %s: %s\n", sc.name, i, hex(got.steps.back()).c_str(),
                  got.summary.c_str());
      total += i;
    } else if (i < want.size() && i < got.steps.size()) {
      std::printf("FAIL %-10s step %zu of %zu differs (trace %s, this build %s); the %zu before "
                  "it match: %s\n",
                  sc.name, i + 1, want.size(), hex(want[i]).c_str(), hex(got.steps[i]).c_str(), i,
                  got.summary.c_str());
      ++failures;
    } else {
      std::printf("FAIL %-10s %zu steps in the trace, %zu in this run: %s\n", sc.name, want.size(),
                  got.steps.size(), got.summary.c_str());
      ++failures;
    }
  }
  for (const auto &kv : scenes) {
    bool known = false;
    for (const scene &sc : kScenes)
      known = known || kv.first == sc.name;
    if (!known) {
      std::printf("FAIL %-10s is in the trace but not in this program\n", kv.first.c_str());
      ++failures;
    }
  }
  if (failures) {
    std::printf("%d scene(s) differ from the trace\n", failures);
    return 1;
  }
  std::printf("all %zu steps match the trace\n", total);
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  const bool rec = argc == 3 && std::strcmp(argv[1], "--record") == 0;
  if (!(rec || argc == 2)) {
    std::fprintf(stderr, "usage: physics_replay_test [--record] <trace.json>\n");
    return 1;
  }
  if (!backend_available(backend::jolt) ||
      !(backend_capabilities(backend::jolt) & cap_determinism_cross_platform)) {
    std::printf("SKIP: this build has no cross-platform-deterministic Jolt backend\n");
    return 77;
  }
  try {
    return rec ? record(argv[2]) : replay(argv[1]);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "physics_replay: %s\n", e.what());
    return 1;
  }
}
