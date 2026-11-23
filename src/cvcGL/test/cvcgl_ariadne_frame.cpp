/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The Ariadne runtime frame's cost (AppRuntime::drain under AriRuntime: GlSceneAdapter's
// sync_visibility, then tick_scene). The scene is shaped like cvcdbg's demo3-ari: 12 `pose`
// vehicles placed from a host's table (fleet.poses, rewritten every frame), each with a driven
// track and a route spine (ribbons from state, the track growing), 6 draped links whose colour and
// opacity are bound, and 8 jam discs; every overlay's `visible:` is bound, 38 of them.
// Headless (tick_scene with no renderer). It times the frames, prints
//   BENCH cvcgl_ariadne_frame frames=<n> mean_us=<n> p50_us=<n> p90_us=<n> vis_us=<n> cpu_us=<n>
// (cpu_us: the thread's CPU time a frame, which a loaded machine's preemption does not inflate)
// and, where cvc::state has a structure epoch, how often the tree's shape moved in those frames
// (each move makes every kept path walk again once: it should hold still in steady frames),
// and checks that what the frame publishes and follows still holds:
//   1. the world steps two physics ticks a 60 Hz frame and publishes physics.stats.*;
//   2. each vehicle's telemetry (entities.<id>.*) is its pose, and entities.count the fleet;
//   3. a bound visible: follows its key, written or deleted and made again;
//   4. a key deleted from the published telemetry is published again the next frame;
//   5. a bound key under a transparent link is read through it once the link is set, and through
//      its new target once the link moves.
// The dynamics need Jolt and YAML; without either it says so and passes.
//
// NOT assert(): built Release, where NDEBUG makes assert() a no-op.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_rules.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/ribbon_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/physics/backend.h>
#include <cvc/state/state.h>
#include <string>
#include <utility>
#include <vector>

using cvc::gl::SceneGraph;
namespace ari = cvc::gl::ariadne;
namespace wv = cvc::ariadne;

static int g_failures = 0;
static void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

namespace {

constexpr int kVehicles = 12, kLinks = 6, kJams = 8;
constexpr double kPi = 3.14159265358979323846;

const char *kDoc = R"(
meta: { name: runtime frame, min_libcvc: 4.0.1 }
requires: [scene.physics]
world:
  vehicles:
    poses: fleet
    archetypes:
      humvee: { vehicle: humvee, controllers: [pose], look: { box: {} } }
scene:
  physics: {}
  world: { source: square, cell: 4 }
  entity_labels: { offset: 2.5 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [240, 240, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - node: track_{i}
      type: ribbon
      repeat: overlays.slots
      source:
        state: overlays.track.{i}
      capacity: 1024
      visible: overlays.track.{i}.on
      casts_shadow: false
    - node: spine_{i}
      type: ribbon
      repeat: overlays.slots
      source:
        state: overlays.spine.{i}
      capacity: 1024
      window:
        from: overlays.spine.{i}.from
        to: overlays.spine.{i}.to
      visible: overlays.spine.{i}.on
      casts_shadow: false
    - node: link_{i}
      type: draped_link
      repeat: overlays.links
      ground: square
      source:
        state: overlays.link.{i}
      width: 1.6
      value: overlays.link.{i}.pdr
      ramp: [[0, [0.90, 0.12, 0.10]], [1, [0.15, 0.85, 0.25]]]
      opacity: overlays.link.{i}.opacity
      visible: overlays.link.{i}.on
      casts_shadow: false
    - node: jam_{i}
      repeat: markers.jams
      source: { sphere: 1 }
      visible: markers.jam.{i}.on
      casts_shadow: false
)";

// The calling thread's CPU time, in seconds (wall time where the platform has no thread clock).
double thread_cpu_s() {
#if defined(CLOCK_THREAD_CPUTIME_ID)
  timespec ts{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0)
    return static_cast<double>(ts.tv_sec) + 1e-9 * static_cast<double>(ts.tv_nsec);
#endif
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

cvc::state &st(cvc::app &app, const std::string &path) { return cvc::state::instance(app)(path); }
std::string val(cvc::app &app, const std::string &path) {
  cvc::state *s = cvc::state::instance(app).findDescendant(path);
  return s ? s->value() : std::string("<none>");
}
std::string num6(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

// Vehicle k at frame f: on a circle, as a host's sim would write it.
wv::entity_pose pose_at(int k, int f) {
  wv::entity_pose p;
  const double a = 2.0 * kPi * k / kVehicles + 0.004 * f;
  const double r = 40.0 + 3.0 * k;
  p.x = r * std::cos(a);
  p.y = r * std::sin(a);
  p.heading = a + 0.5 * kPi;
  p.speed = 0.004 * r * 60.0;
  return p;
}

void write_poses(cvc::app &app, int f) {
  std::vector<std::pair<std::string, wv::entity_pose>> records;
  for (int k = 0; k < kVehicles; ++k)
    records.emplace_back("v" + std::to_string(k), pose_at(k, f));
  st(app, "fleet.poses").value(wv::pack_poses(records));
}

void seed(cvc::app &app) {
  st(app, "overlays.slots").value(std::to_string(kVehicles));
  st(app, "overlays.links").value(std::to_string(kLinks));
  st(app, "markers.jams").value(std::to_string(kJams));
  for (int k = 0; k < kVehicles; ++k) {
    const std::string at = "fleet.v" + std::to_string(k) + ".";
    st(app, at + "archetype").value("humvee");
    st(app, at + "name").value("v" + std::to_string(k));
    const std::string o = std::to_string(k);
    st(app, "overlays.track." + o).value(std::string("0 0 0"));
    st(app, "overlays.track." + o + ".on").value("1");
    std::string spine;
    for (int s = 0; s < 64; ++s) {
      const wv::entity_pose p = pose_at(k, 40 * s);
      spine += num6(p.x) + " " + num6(p.y) + " 0 ";
    }
    st(app, "overlays.spine." + o).value(spine);
    st(app, "overlays.spine." + o + ".from").value("0");
    st(app, "overlays.spine." + o + ".to").value("32");
    st(app, "overlays.spine." + o + ".on").value("1");
  }
  for (int l = 0; l < kLinks; ++l) {
    const std::string o = std::to_string(l);
    st(app, "overlays.link." + o).value("0,0," + std::to_string(10 * l) + ",20");
    st(app, "overlays.link." + o + ".pdr").value("0.5");
    st(app, "overlays.link." + o + ".opacity").value("0.8");
    st(app, "overlays.link." + o + ".on").value("1");
  }
  for (int j = 0; j < kJams; ++j)
    st(app, "markers.jam." + std::to_string(j) + ".on").value(j % 2 ? "1" : "0");
  write_poses(app, 0);
}

} // namespace

int main() {
  std::printf("cvcgl_ariadne_frame\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return 0;
  }
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no Jolt in this build\n");
    return 0;
  }
  cvc::app app;
  ari::register_ribbon_node_types(app);
  cvc::ariadne::register_world_block(app);
  cvc::ariadne::LoadResult doc = cvc::ariadne::load_string(app, kDoc);
  CHECK(doc.ok);
  if (!doc.ok) {
    std::printf("  load: %s\n", doc.error.c_str());
    return 1;
  }
  { // the world: block's vehicles go to the scene, as AppRuntime::load sends them
    cvc::ariadne::world_rules rules = cvc::ariadne::default_world_rules();
    cvc::ariadne::parse_world_block(*cvc::ariadne::find_world_block(doc), rules);
    doc.scene.vehicles = rules.vehicles;
    doc.scene.spawns = rules.spawns;
    doc.scene.vehicle_intents = rules.vehicle_intents;
    doc.scene.vehicle_poses = rules.vehicle_poses;
    doc.scene.vehicle_starts = rules.vehicle_starts;
  }
  seed(app);
  SceneGraph sg(app, "frame");
  sg.setDiagnosticChromeVisible(false);
  std::vector<std::string> warnings;
  ari::RealizedScene rs = ari::realize_scene(sg, doc.scene, "", &warnings);
  for (const std::string &w : warnings)
    std::printf("  warning: %s\n", w.c_str());
  CHECK(rs.world != nullptr && rs.world->vehicles() != nullptr);
  if (!rs.world || !rs.world->vehicles())
    return 1;

  double vis_s = 0.0;
  int frame_no = 0;
  const auto frame = [&]() {
    ++frame_no;
    write_poses(app, frame_no);
    if (frame_no % 6 == 0) // the host's geometry lane: each track grows by a point
      for (int k = 0; k < kVehicles; ++k) {
        cvc::state &t = st(app, "overlays.track." + std::to_string(k));
        const wv::entity_pose p = pose_at(k, frame_no);
        t.value(t.value() + " " + num6(p.x) + " " + num6(p.y) + " 0");
      }
    const auto t0 = std::chrono::steady_clock::now();
    cvc::ariadne::sync_scene_visibility(app, rs.visibility); // GlSceneAdapter::sync_visibility
    const auto t1 = std::chrono::steady_clock::now();
    ari::tick_scene(rs, nullptr, 1.0 / 60.0); // GlSceneAdapter::tick
    vis_s += std::chrono::duration<double>(t1 - t0).count();
  };
  for (int i = 0; i < 120; ++i) // the table's vehicles spawn and settle; the repeats' copies made
    frame();
  std::printf("  %zu visible: bindings, %zu entities\n", rs.visibility.size(),
              rs.world->vehicles()->size());
  CHECK(rs.visibility.size() >= static_cast<std::size_t>(2 * kVehicles + kLinks + kJams));

  // The timed frames.
  constexpr int kFrames = 600;
  std::vector<double> us;
  us.reserve(kFrames);
  vis_s = 0.0;
  const double cpu0 = thread_cpu_s();
#if __has_include(<cvc/state/state_key.h>)
  const std::uint64_t epoch0 = cvc::state::instance(app).structureEpoch();
#endif
  for (int i = 0; i < kFrames; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    frame();
    us.push_back(
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
  }
  const double cpu_us = 1e6 * (thread_cpu_s() - cpu0) / kFrames;
  double mean = 0.0;
  for (double u : us)
    mean += u;
  mean /= kFrames;
  std::vector<double> sorted = us;
  std::sort(sorted.begin(), sorted.end());
  std::printf("BENCH cvcgl_ariadne_frame frames=%d mean_us=%.1f p50_us=%.1f p90_us=%.1f "
              "vis_us=%.1f cpu_us=%.1f\n",
              kFrames, mean, sorted[kFrames / 2], sorted[kFrames * 9 / 10], 1e6 * vis_s / kFrames,
              cpu_us);
#if __has_include(<cvc/state/state_key.h>)
  std::printf("  the tree's shape moved %llu times in those frames\n",
              static_cast<unsigned long long>(cvc::state::instance(app).structureEpoch() - epoch0));
#endif

  // The checks below read the telemetry every frame: written every frame from here (the frames
  // above ran at the default rate, physics.telemetry_hz, where it exists).
  st(app, "physics.telemetry_hz").value("0");

  // 1. Two physics ticks a frame, published.
  wv::SceneWorld &w = *rs.world;
  const std::uint64_t tick = w.world().tick();
  CHECK(val(app, "physics.stats.tick") == std::to_string(tick));
  frame();
  CHECK(w.world().tick() == tick + 2);
  CHECK(val(app, "physics.stats.tick") == std::to_string(tick + 2));
  CHECK(val(app, "physics.stats.bodies") == std::to_string(w.world().body_count()));

  // 2. Each vehicle's telemetry is its pose.
  CHECK(w.vehicles() != nullptr);
  if (!w.vehicles())
    return 1;
  const std::vector<wv::entity_info> ents = w.vehicles()->entities();
  CHECK(ents.size() == static_cast<std::size_t>(kVehicles));
  CHECK(val(app, "entities.count") == std::to_string(kVehicles));
  for (const wv::entity_info &e : ents) {
    const std::string at = "entities." + std::to_string(e.id) + ".";
    CHECK(e.placed && e.posed);
    CHECK(val(app, at + "x") == num6(e.x));
    CHECK(val(app, at + "y") == num6(e.y));
    CHECK(val(app, at + "name") == e.spec.name);
  }

  // 3. visible: follows its key: written, then deleted and made again.
  const std::string track3 = sg.getGraphics("track_3") ? "track_3" : "";
  CHECK(!track3.empty());
  const auto visible = [&](const std::string &node) {
    auto g = sg.getGraphics(node);
    return g ? g->isVisible() : false;
  };
  CHECK(visible("track_3"));
  st(app, "overlays.track.3.on").value("0");
  frame();
  CHECK(!visible("track_3"));
  st(app, "overlays.track.3.on").value("1");
  frame();
  CHECK(visible("track_3"));
  cvc::state::instance(app)("overlays.track.3").removeChild("on"); // unset: the default, shown
  st(app, "overlays.track.4.on").value("0");
  frame();
  CHECK(visible("track_3") && !visible("track_4"));
  st(app, "overlays.track.3.on").value("0"); // made again: followed
  frame();
  CHECK(!visible("track_3"));

  // 4. A deleted telemetry key is published again.
  const std::string first = "entities." + std::to_string(ents.front().id);
  cvc::state::instance(app)(first).removeChild("x");
  CHECK(val(app, first + ".x") == "<none>");
  frame();
  CHECK(val(app, first + ".x") != "<none>");
  cvc::state::instance(app).removeChild("physics"); // the whole stats subtree
  frame();
  CHECK(val(app, "physics.stats.tick") == std::to_string(w.world().tick()));

  // 5. Through a transparent link: markers -> alt (jam 0 shown there), then -> alt2 (hidden).
  CHECK(!visible("jam_0") && visible("jam_1"));
  st(app, "alt.jams").value(std::to_string(kJams));
  st(app, "alt2.jams").value(std::to_string(kJams));
  for (int j = 0; j < kJams; ++j) {
    st(app, "alt.jam." + std::to_string(j) + ".on").value("1");
    st(app, "alt2.jam." + std::to_string(j) + ".on").value("0");
  }
  cvc::state &markers = cvc::state::instance(app)("markers");
  markers.linkTo("alt", cvc::state::link_mode::transparent);
  frame();
  CHECK(visible("jam_0") && visible("jam_1"));
  markers.linkTo("alt2", cvc::state::link_mode::transparent);
  frame();
  CHECK(!visible("jam_0") && !visible("jam_1"));
  markers.clearLink();
  frame();
  CHECK(!visible("jam_0") && visible("jam_1"));

  std::printf("%s (%d failure%s)\n", g_failures ? "FAILED" : "passed", g_failures,
              g_failures == 1 ? "" : "s");
  return g_failures ? 1 : 0;
}
