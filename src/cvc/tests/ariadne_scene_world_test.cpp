// A document's scene world with no renderer (scene_world.h; ARIADNE_WORLDS.md section 14,
// "Headless first"): the physics world and its bodies built from the scene alone, stepped on the
// world clock, publishing physics.stats.*, and run by an AppRuntime with no SceneAdapter; a
// generated_world node run here (its world, keys, Regenerate and colliders); a data node's bodies
// taken from its dataset when it is ready, and swapped when it changes. The same worlds drawn by
// cvcGL are cvcgl_ariadne_physics's test_same_world_headless_and_drawn.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/conference/room_entities.h>
#include <cvc/ariadne/conference/room_intrinsics.h>
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/holon_sync.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/requires.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_boat.h>
#include <cvc/ariadne/scene_generated_world.h>
#include <cvc/ariadne/scene_geometry.h>
#include <cvc/ariadne/scene_water.h>
#include <cvc/ariadne/scene_weather.h>
#include <cvc/ariadne/scene_wind.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/server_document.h>
#include <cvc/ariadne/world_gym.h>
#include <cvc/ariadne/world_rules.h>
#include <cvc/ariadne/world_spawns.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/core/sim_tick.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/world_grid.h>
#include <cvc/ocean/sea_surface.h>
#include <cvc/ocean/shore.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/nav_plant.h> // nav_plant_options
#include <cvc/physics/world.h>
#include <cvc/physics/world_geometry.h> // wet_friction
#include <cvc/state/state.h>
#include <cvc/world/surface.h>
#include <cvc/world/weather.h>
#include <cvc/world/world_geometry.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::ariadne;
namespace fs = std::filesystem;

namespace {
// A document checked against a fresh app's registries: the built-in widget types, blocks and
// decoders only. (A test that registers its own loads against that app instead.)
[[maybe_unused]] cvc::ariadne::LoadResult fresh_load(const std::string &yaml) {
  cvc::app app;
  return cvc::ariadne::load_string(app, yaml);
}
[[maybe_unused]] cvc::ariadne::LoadResult fresh_load_file(const std::string &path) {
  cvc::app app;
  return cvc::ariadne::load_file(app, path);
}
} // namespace

namespace {

#define REQUIRE_YAML()                                                                             \
  do {                                                                                             \
    if (!AppRuntime::have_yaml())                                                                  \
      GTEST_SKIP() << "libcvc built without yaml-cpp";                                             \
  } while (0)

bool jolt() { return cvc::physics::backend_available(cvc::physics::backend::jolt); }

const char *kScene = R"(
scene:
  physics: {}
  nodes:
    - node: terrain
      source:
        heightfield:
          size: 20
          resolution: 41
          layers:
            - { kind: dome, amplitude: 1.0, radius: 6 }
      body: static
    - node: crate
      source: { box: { size: [1, 1, 1] } }
      transform: { position: [2.0, -1.5, 4.0] }
      body: { mass: 10 }
    - node: rig
      type: group
      transform: { position: [0, 0, 1] }
      children:
        - node: ball
          source: { sphere: 0.4 }
          transform: { position: [0, 0, 2] }
          body: { density: 500 }
)";

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

fs::path write_doc(const std::string &name, const std::string &text) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_scene_world_test";
  fs::create_directories(dir);
  const fs::path p = dir / (name + ".ari");
  std::ofstream(p) << text;
  return p;
}

bool any_contains(const std::vector<std::string> &v, const std::string &needle) {
  for (const std::string &s : v)
    if (s.find(needle) != std::string::npos)
      return true;
  return false;
}

SceneClock resolved_clock(const Scene &scene) {
  SceneClock c = scene.clock; // keys under the root prefix: as given
  return c;
}

} // namespace

TEST(AriadneSceneWorld, LocalMatrixIsRotationThenRowScaleThenPosition) {
  SceneNode n;
  n.has_transform = true;
  n.rotation[2] = 90.0; // about z: +x goes to +y
  n.scale[0] = 2.0;
  n.position[0] = 5.0;
  double m[16];
  scene_node_local_matrix(n, m);
  // Rz(90) = [[0,-1,0],[1,0,0],[0,0,1]], row 0 scaled by 2.
  EXPECT_NEAR(m[0], 0.0, 1e-12);
  EXPECT_NEAR(m[1], -2.0, 1e-12);
  EXPECT_NEAR(m[4], 1.0, 1e-12);
  EXPECT_NEAR(m[5], 0.0, 1e-12);
  EXPECT_NEAR(m[10], 1.0, 1e-12);
  EXPECT_NEAR(m[3], 5.0, 1e-12);
  SceneNode plain;
  scene_node_local_matrix(plain, m);
  for (int k = 0; k < 16; ++k)
    EXPECT_EQ(m[k], (k % 5 == 0) ? 1.0 : 0.0);
}

TEST(AriadneSceneWorld, BuildsTheBodiesFromTheSceneAlone) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(kScene);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(scene_has_world(r.scene));
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  if (jolt())
    EXPECT_TRUE(warnings.empty()) << warnings.front();
  ASSERT_EQ(w->bodies().size(), 3u);
  const auto crate = w->body("crate"), ball = w->body("ball"), terrain = w->body("terrain");
  ASSERT_TRUE(crate.valid() && ball.valid() && terrain.valid());
  const cvc::physics::body_state c0 = w->world().state(crate), b0 = w->world().state(ball);
  EXPECT_NEAR(c0.position.x, 2.0, 1e-9);
  EXPECT_NEAR(c0.position.y, -1.5, 1e-9);
  EXPECT_NEAR(c0.position.z, 4.0, 1e-9);
  EXPECT_NEAR(b0.position.z, 3.0, 1e-9); // through its group
  EXPECT_NEAR(w->world().config().fixed_dt, app.world_clock().fixed_dt(), 1e-12);
  for (const SceneWorldBody &b : w->bodies())
    EXPECT_EQ(b.moves, b.node != "terrain");
}

TEST(AriadneSceneWorld, StepsOnTheClockAndPublishesItsStats) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = fresh_load(kScene);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  SceneClockDriver clock(app, resolved_clock(r.scene));
  const auto crate = w->body("crate");
  int captures = 0;
  for (int i = 0; i < 360; ++i) { // six seconds at 60 frames per second
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock(), [&] { ++captures; });
  }
  EXPECT_NEAR(app.world_clock().t(), 6.0, 0.02);
  EXPECT_GT(captures, 0);
  // The crate fell and rests on the dome's flank, well above z = 0 and below where it started.
  const cvc::physics::body_state c = w->world().state(crate);
  EXPECT_LT(c.position.z, 2.0);
  EXPECT_GT(c.position.z, 0.0);
  EXPECT_EQ(val(app, "physics.stats.tick"), std::to_string(w->world().tick()));
  EXPECT_EQ(val(app, "physics.stats.bodies"), "3");
  EXPECT_FALSE(val(app, "physics.stats.step_ms").empty());
  EXPECT_FALSE(val(app, "physics.stats.active_bodies").empty());
  EXPECT_EQ(val(app, "physics.stats.saturated"), "false");
  EXPECT_EQ(val(app, "sim.time").substr(0, 1), "6"); // the clock's own telemetry
}

TEST(AriadneSceneWorld, OnlyAGeometryNodesBodyRunsWithoutARenderer) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(R"(
scene:
  nodes:
    - node: lump
      type: group
      body: { mass: 1 }
    - node: cube
      source: { box: { size: [1, 1, 1] } }
      body: { shape: hull }
)");
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "", &warnings);
  ASSERT_NE(w, nullptr);
  EXPECT_TRUE(any_contains(warnings, "'lump' (group): body:"));
  ASSERT_EQ(w->bodies().size(), 1u);
  EXPECT_EQ(w->bodies()[0].node, "cube");
}

TEST(AriadneSceneWorld, AHeadlessAppRuntimeRunsTheWorld) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, ""); // no SceneAdapter: nothing is drawn
  const std::vector<std::string> warns =
      art.load(write_doc("headless", std::string("requires: [scene.physics]\n") + kScene).string());
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build: scene.physics does not hold";
  EXPECT_FALSE(any_contains(warns, "scene ignored"));
  ASSERT_NE(art.scene_world(), nullptr);
  EXPECT_EQ(art.scene_world()->bodies().size(), 3u);
  // The world steps on real time: drain until a step has run.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (art.scene_world()->world().tick() == 0 && std::chrono::steady_clock::now() < deadline) {
    art.drain();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GT(art.scene_world()->world().tick(), 0u);
  EXPECT_FALSE(val(app, "physics.stats.tick").empty());

  // A document without a world still says its scene is ignored here, and has no world.
  const std::vector<std::string> plain = art.load(
      write_doc("no_world", "scene:\n  nodes:\n    - node: cube\n      source: { box: { size: "
                            "[1, 1, 1] } }\n")
          .string());
  EXPECT_TRUE(any_contains(plain, "scene ignored"));
  EXPECT_EQ(art.scene_world(), nullptr);
}

// sim.step and sim.restart are built in: a step request runs quanta on a paused clock, and a
// restart puts the clock back to tick 0 and rebuilds the world, which then steps exactly as a
// world loaded fresh (the same state hash after the same steps).
TEST(AriadneSceneWorld, SimStepAndRestartAreBuiltIn) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  std::string doc = kScene;
  const std::string physics = "  physics: {}\n";
  doc.replace(doc.find(physics), physics.size(), "  clock: { paused: true }\n" + physics);
  const fs::path path = write_doc("sim_restart", doc);

  cvc::app app;
  AppRuntime art(app, "");
  art.load(path.string());
  ASSERT_NE(art.scene_world(), nullptr);
  art.drain(); // the world anchors on the paused clock
  const cvc::physics::world &w = art.scene_world()->world();
  const std::uint64_t h0 = w.state_hash();
  EXPECT_EQ(w.tick(), 0u);
  art.step_world(30);
  art.drain();
  const std::uint64_t ticks = w.tick();
  ASSERT_GT(ticks, 0u);
  const std::uint64_t h1 = w.state_hash();
  ASSERT_NE(h1, h0);

  EXPECT_TRUE(art.restart_world());
  EXPECT_EQ(app.world_clock().tick(), 0u);
  EXPECT_EQ(art.scene_world()->world().tick(), 0u);
  EXPECT_EQ(art.scene_world()->world().state_hash(), h0); // as the document built it
  EXPECT_EQ(val(app, "sim.restarts"), "1");
  art.drain();
  art.step_world(30);
  art.drain();
  EXPECT_EQ(art.scene_world()->world().tick(), ticks);
  EXPECT_EQ(art.scene_world()->world().state_hash(), h1);

  // A world loaded fresh and stepped the same way is the same world.
  cvc::app app2;
  AppRuntime fresh(app2, "");
  fresh.load(path.string());
  fresh.drain();
  fresh.step_world(30);
  fresh.drain();
  EXPECT_EQ(fresh.scene_world()->world().state_hash(), h1);

  // A document with no scene: nothing to step, and a restart rebuilds nothing.
  cvc::app app3;
  AppRuntime none(app3, "");
  none.load(
      write_doc("no_scene", "windows:\n  - window: W\n    children:\n      - text: hi\n").string());
  none.step_world(3);
  EXPECT_FALSE(none.restart_world());
  EXPECT_EQ(val(app3, "sim.step"), "");
}

TEST(AriadneSceneWorld, ScenePhysicsTokenIsTheEngine) {
  bool found = false;
  for (const CapabilityToken &t : capability_tokens())
    if (std::string(t.name) == "scene.physics")
      found = true;
  EXPECT_TRUE(found);
}

namespace {

// A small generated world (a few hundred metres, a handful of props) and a ball above it.
const char *kGenerated = R"(
scene:
  physics: {}
  nodes:
    - node: world
      type: generated_world
      seed: 5
      half: 150
      trees: 6
      rocks: 3
      buildings: 6
      body: {}
    - node: ball
      source: { sphere: 0.5 }
      transform: { position: [3, -4, 80] }
      body: { density: 400 }
)";

// The walls add_world_geometry puts around a terrain's edge, bodies of the terrain's node.
constexpr std::size_t kEdgeWalls = 4;

std::size_t collide_items(const cvc::world::generated_world &w) {
  std::size_t n = 0;
  const cvc::world::world_geometry g = w.geometry();
  for (const cvc::world::geometry_item &it : g.items())
    if (it.has(cvc::world::role_collide))
      ++n;
  return n;
}

} // namespace

TEST(AriadneSceneWorld, AGeneratedWorldRunsWithoutARenderer) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(kGenerated);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  if (jolt())
    EXPECT_TRUE(warnings.empty()) << warnings.front();
  std::shared_ptr<const cvc::world::generated_world> gw = w->generated_world("world");
  ASSERT_NE(gw, nullptr);
  ASSERT_NE(gw->terrain, nullptr);
  EXPECT_EQ(gw->params.half_m, 150.0);
  // The node's body: is the world's colliders, the terrain first: a static body each.
  const std::size_t items = collide_items(*gw);
  EXPECT_GE(items, 1u);
  EXPECT_EQ(w->bodies_of("world").size(), items + kEdgeWalls);
  EXPECT_EQ(w->world().body_count(), items + kEdgeWalls + 1);
  for (const SceneWorldBody &b : w->bodies())
    EXPECT_EQ(b.moves, b.node == "ball");
  // Of its colliders the terrain (a height field) is ground; its buildings are not.
  std::size_t terrain = 0;
  for (const SceneWorldBody &b : w->bodies())
    if (b.node == "world" && b.ground)
      ++terrain;
  EXPECT_EQ(terrain, 1u);
  // Its keys are seeded and its stats published, as a renderer's node does.
  EXPECT_EQ(val(app, "world.seed"), "5");
  EXPECT_EQ(val(app, "world.half"), "150");
  EXPECT_EQ(val(app, "world.stats.buildings"), std::to_string(gw->building_count));
  EXPECT_FALSE(gw->water.empty()); // the bodies of water, and their line in the panel
  EXPECT_EQ(val(app, "world.stats.water"), cvc::world::water_bodies_summary(gw->water));
  EXPECT_NE(val(app, "world.status").find("fbm terrain"), std::string::npos);
  EXPECT_FALSE(val(app, "world.generators.terrain").empty());

  if (!jolt())
    return;
  // The ball falls onto the generated ground and rests on it.
  SceneClockDriver clock(app, resolved_clock(r.scene));
  for (int i = 0; i < 600; ++i) {
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
  }
  const cvc::physics::body_state b = w->world().state(w->body("ball"));
  const double ground = gw->height(b.position.x, b.position.y);
  EXPECT_NEAR(b.position.z, ground + 0.5, 0.25);
}

// A water node with body: is one fluid field over its ground (scene_water.h), run with no
// renderer: the world flooded to 200 m, a light crate floats at the level on the cascades' sea, a
// dense one sinks; a new wind rebuilds the field and the world still holds one (the old one is
// removed with the node's revision, as a regenerated world's bodies are).
TEST(AriadneSceneWorld, AWaterNodesBodyFloatsWhatIsLighterThanWater) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(R"(
scene:
  physics: {}
  nodes:
    - node: world
      type: generated_world
      seed: 5
      half: 150
      trees: 0
      rocks: 0
      buildings: 0
      water_level: 200
      body: {}
    - node: water
      type: water
      world: world
      level: 200
      wind_speed: 3
      body: {}
    - node: raft
      source: { box: { size: [2, 2, 2] } }
      transform: { position: [5, 5, 204] }
      body: { density: 400 }
    - node: anchor
      source: { box: { size: [1, 1, 1] } }
      transform: { position: [-5, -5, 204] }
      body: { density: 3000 }
)");
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  if (!jolt())
    return;
  EXPECT_TRUE(warnings.empty()) << warnings.front();
  EXPECT_EQ(w->world().field_count(), 1u);
  EXPECT_EQ(val(app, "water.wind_speed"), "3"); // seeded from the node, as the drawn node does
  SceneClockDriver clock(app, resolved_clock(r.scene));
  for (int i = 0; i < 600; ++i) { // 10 s
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
  }
  const cvc::physics::body_state raft = w->world().state(w->body("raft"));
  const cvc::physics::body_state anchor = w->world().state(w->body("anchor"));
  std::printf("[water] after 10 s: raft at z %.3f (level 200), anchor at z %.3f\n", raft.position.z,
              anchor.position.z);
  // At density 400 the raft rides 40 % under: its centre 0.2 m over the water, within the 3 m/s
  // sea's waves (about 0.25 m significant).
  EXPECT_NEAR(raft.position.z, 200.2, 0.6);
  EXPECT_LT(anchor.position.z, 195.0) << "the dense crate sinks";
  // A new wind: the field is rebuilt, and the world still holds one.
  cvc::state::instance(app)("water.wind_speed").value(6.0);
  for (int i = 0; i < 30; ++i) {
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
  }
  EXPECT_EQ(w->world().field_count(), 1u);
  EXPECT_NEAR(w->world().state(w->body("raft")).position.z, 200.2, 1.0);
}

// The physics sea is the drawn one at the coast too (scene_water.h): over a beach the wind sea's
// cascades fade out as the water shoals, and a swell (swell_height) is in it, smaller where it
// breaks in the shallows. No renderer, no physics backend: the runtime's sea read directly.
TEST(AriadneSceneWorld, AWaterNodesSeaFadesAtTheShoreAndCarriesTheSwell) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(R"(
scene:
  nodes:
    - node: water
      type: water
      ground: beach
      level: 0
      wind_speed: 9
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_FALSE(r.scene.nodes.empty());
  const SceneNode &n = r.scene.nodes.front();
  cvc::app app;
  // A beach rising along +x at 1 in 10: 20 m deep at x = -200, the waterline at x = 0.
  const int N = 401;
  std::vector<float> h(std::size_t(N) * N);
  for (int r0 = 0; r0 < N; ++r0)
    for (int c = 0; c < N; ++c)
      h[std::size_t(r0) * N + c] = float(0.1 * (c - 200));
  auto beach = std::make_shared<cvc::height_field>(N, N, -200.0, -200.0, 1.0, 1.0, h);
  WaterRuntime rt(app, n, "", [beach] { return beach; });
  ASSERT_NE(rt.sea(), nullptr);
  ASSERT_NE(rt.shore(), nullptr);
  ASSERT_NE(rt.shore()->grid, nullptr);
  EXPECT_EQ(rt.shore()->layer, nullptr) << "no swell, no shore layer";
  std::printf("[water] the coast in %.0f ms (%d x %d)\n", rt.shore()->build_ms,
              rt.shore()->grid->cols, rt.shore()->grid->rows);
  // The surface's RMS over `span` seconds at (x, 0).
  auto rms = [&](double x, double span) {
    double s = 0.0;
    int k = 0;
    for (double t = 0.0; t < span; t += 0.25, ++k) {
      const double z = rt.sea()->height(x, 0.0, t);
      s += z * z;
    }
    return std::sqrt(s / k);
  };
  const double deep = rms(-150.0, 20.0), shallow = rms(-3.0, 20.0); // 15 m and 0.3 m of water
  std::printf("[water] wind sea RMS: %.3f m in 15 m of water, %.4f m in 0.3 m\n", deep, shallow);
  EXPECT_GT(deep, 0.1);
  EXPECT_LT(shallow, 0.1 * deep) << "the cascades fade out as the water shoals";

  // A 1 m swell on a near-calm sea: the shore layer carries it. The world's clock is not running
  // here, so the sea's wind would never move off 9 m/s (it lags the wind, O13): no lag, no fade.
  cvc::state::instance(app)("water.sea_lag").value(0.0);
  cvc::state::instance(app)("water.sea_fade").value(0.0);
  cvc::state::instance(app)("water.wind_speed").value(1.0);
  cvc::state::instance(app)("water.swell_height").value(1.0);
  cvc::state::instance(app)("water.swell_dir").value(0.0); // toward the beach
  EXPECT_TRUE(rt.tick());
  ASSERT_NE(rt.shore()->layer, nullptr);
  // Over 300 s: the trains' periods are 6 % apart, so they come in sets about 170 s long.
  const double swellDeep = rms(-150.0, 300.0), swellShallow = rms(-5.0, 300.0); // 15 m, 0.5 m
  std::printf("[water] swell RMS: %.3f m in 15 m of water, %.3f m in 0.5 m\n", swellDeep,
              swellShallow);
  EXPECT_GT(swellDeep, 0.15) << "a 1 m swell is about 0.25 m RMS";
  EXPECT_LT(swellDeep, 0.45);
  EXPECT_LT(swellShallow, swellDeep) << "it breaks in the shallows (H under 0.78 d)";
  EXPECT_FALSE(rt.tick()) << "nothing changed: nothing rebuilt";

  // The swash: on the dry sand 3 m up the beach (0.3 m over the level) the surface reaches over
  // the ground as each wave runs up, and stays at the level between them; with `swash: 0` it
  // never leaves the level there.
  auto wetShare = [&](const WaterRuntime &w) {
    int wet = 0, k = 0;
    for (double t = 0.0; t < 300.0; t += 0.25, ++k)
      if (w.sea()->height(3.0, 0.0, t) > 0.3)
        ++wet;
    return double(wet) / k;
  };
  const double wet = wetShare(rt);
  LoadResult dry = fresh_load(R"(
scene:
  nodes:
    - node: water
      type: water
      ground: beach
      level: 0
      swash: 0
)");
  ASSERT_TRUE(dry.ok) << dry.error;
  WaterRuntime noSwash(app, dry.scene.nodes.front(), "", [beach] { return beach; });
  const double wetWithout = wetShare(noSwash);
  std::printf(
      "[water] the sand 0.3 m over the level is under the swash %.0f %% of the time (%.0f %% "
      "with swash: 0)\n",
      100.0 * wet, 100.0 * wetWithout);
  EXPECT_GT(wet, 0.05) << "the swash runs up over it";
  EXPECT_LT(wet, 0.95) << "and drains back";
  EXPECT_EQ(wetWithout, 0.0);
}

// The lee is calm in the physics sea too (O13, E2): a 9 m/s wind blowing toward +x past a round
// island leaves the water just downwind of it calm, so a crate floating there moves less than one
// in the open water upwind; with `lee: 0` the two seas are alike. Then a wind change crossfades the
// sea over sea_fade seconds on the world's clock, and the field is rebuilt once per keyframe.
TEST(AriadneSceneWorld, AWaterNodesLeeIsCalmerThanItsOpenWater) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(R"(
scene:
  nodes:
    - node: water
      type: water
      ground: island
      level: 0
      wind_speed: 9
      wind_dir: 0
      gustiness: 0
)");
  ASSERT_TRUE(r.ok) << r.error;
  const SceneNode &n = r.scene.nodes.front();
  cvc::app app;
  // A round island 120 m in radius rising to 20 m, in water 20 m deep, on 1.2 km of 3 m samples.
  const int N = 401;
  const double cell = 3.0, x0 = -600.0;
  std::vector<float> h(std::size_t(N) * N);
  for (int r0 = 0; r0 < N; ++r0)
    for (int c = 0; c < N; ++c) {
      const double x = x0 + c * cell, y = x0 + r0 * cell, d = std::hypot(x, y);
      h[std::size_t(r0) * N + c] = float(d < 120.0 ? 20.0 * (1.0 - d / 120.0) + 0.5 : -20.0);
    }
  auto island = std::make_shared<cvc::height_field>(N, N, x0, x0, cell, cell, h);
  WaterRuntime rt(app, n, "", [island] { return island; });
  ASSERT_NE(rt.sea(), nullptr);
  ASSERT_NE(rt.keyframes(), nullptr);
  ASSERT_NE(rt.keyframes()->fetch, nullptr) << "a fetch map over the coast";
  const cvc::ocean::fetch_field &F = *rt.keyframes()->fetch;
  std::printf("[water] the fetch map: %d x %d samples of %.1f m in %.1f ms (the keyframe)\n",
              F.cols, F.rows, F.cell, rt.keyframes()->build_ms);
  EXPECT_LE(std::max(F.rows, F.cols), 256) << "a coarse map";
  const double leeX = 160.0, openX = -400.0; // 40 m behind the island; 280 m upwind of it
  auto rms = [](const cvc::ocean::sea_surface &s, double x, double y) {
    double s2 = 0.0;
    int k = 0;
    for (double t = 0.0; t < 30.0; t += 0.25, ++k) {
      const double z = s.height(x, y, t);
      s2 += z * z;
    }
    return std::sqrt(s2 / k);
  };
  const double lee = rms(*rt.sea(), leeX, 0.0), open = rms(*rt.sea(), openX, 0.0);
  std::printf("[water] RMS 40 m behind the island %.3f m, upwind %.3f m\n", lee, open);
  EXPECT_LT(lee, 0.3 * open) << "the lee is calm";

  if (jolt()) {
    // A crate in each, 20 s on the field.
    cvc::physics::world_config c;
    c.backend = cvc::physics::backend::jolt;
    cvc::physics::world w(c);
    w.add_field(*rt.field());
    auto crate = [&](double x) {
      cvc::physics::body_desc d;
      d.shape = cvc::physics::box{{1.0, 1.0, 1.0}};
      d.density = 500.0;
      d.position = {x, 0.0, 0.2};
      d.allow_sleeping = false;
      return w.add_body(d);
    };
    const cvc::physics::body_id a = crate(leeX), b = crate(openX);
    double za = 0.0, zb = 0.0, za2 = 0.0, zb2 = 0.0;
    int k = 0;
    for (int i = 0; i < 1200; ++i) {
      w.step();
      if (i < 180)
        continue; // settle 3 s
      const double ha = w.state(a).position.z, hb = w.state(b).position.z;
      za += ha, zb += hb, za2 += ha * ha, zb2 += hb * hb, ++k;
    }
    const double sa = std::sqrt(std::max(za2 / k - (za / k) * (za / k), 0.0));
    const double sb = std::sqrt(std::max(zb2 / k - (zb / k) * (zb / k), 0.0));
    std::printf("[water] a crate's heave over 17 s: %.3f m RMS in the lee, %.3f m in the open\n",
                sa, sb);
    EXPECT_LT(sa, 0.5 * sb) << "the crate in the lee moves less";
  }

  // lee: 0 puts the waves back behind the island.
  cvc::state::instance(app)("water.lee").value(0.0);
  EXPECT_TRUE(rt.tick());
  EXPECT_EQ(rt.keyframes()->fetch, nullptr);
  const double noLee = rms(*rt.sea(), leeX, 0.0);
  std::printf("[water] without the lee, 40 m behind the island: %.3f m\n", noLee);
  EXPECT_GT(noLee, 3.0 * lee);

  // A wind change on the world's clock (no lag, a 4 s fade): one keyframe, the field rebuilt for
  // it, the sea crossfading; a second change during the fade waits for it to end.
  cvc::state::instance(app)("water.lee").value(1.0);
  cvc::state::instance(app)("water.sea_lag").value(0.0);
  EXPECT_TRUE(rt.tick());
  SceneClockDriver clock(app, resolved_clock(r.scene));
  const std::uint64_t k0 = rt.keyframes()->keyframe;
  cvc::state::instance(app)("water.wind_dir").value(30.0);
  int rebuilds = 0;
  double midWeight = -1.0;
  for (int i = 0; i < 6 * 60; ++i) {
    if (i == 60)
      cvc::state::instance(app)("water.wind_speed").value(12.0);
    clock.drive(1.0 / 60.0);
    if (rt.tick())
      ++rebuilds;
    if (i == 2 * 60)
      midWeight = rt.keyframes()->weight(app.world_clock().t());
  }
  std::printf(
      "[water] two wind changes 1 s apart: %d keyframes, %d rebuilds; 2 s into the fade the "
      "new sea's share is %.2f\n",
      int(rt.keyframes()->keyframe - k0), rebuilds, midWeight);
  EXPECT_EQ(rt.keyframes()->keyframe - k0, 2u) << "the second waited for the first fade";
  EXPECT_EQ(rebuilds, 2);
  EXPECT_GT(midWeight, 0.2);
  EXPECT_LT(midWeight, 0.8);
}

// The water node's field is inertial (field.h): sideways it carries a floating body by the water's
// acceleration and holds it by drag, so a crate on it drifts in the scene's wind at about its
// leeway. The same field damped in every direction, as it was before, holds the crate.
TEST(AriadneSceneWorld, ACrateOnTheWaterDriftsInTheScenesWind) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(R"(
scene:
  wind: { speed: 10, dir: 0, gustiness: 0 }
  nodes:
    - node: water
      type: water
      ground: deep
      level: 0
      wave_amp: 0.1
)");
  ASSERT_TRUE(r.ok) << r.error;
  const SceneNode &n = r.scene.nodes.front();
  cvc::app app;
  seed_scene_wind(app, r.scene, "");
  const int N = 201;
  auto deep = std::make_shared<cvc::height_field>(N, N, -600.0, -600.0, 6.0, 6.0,
                                                  std::vector<float>(std::size_t(N) * N, -20.0f));
  WaterRuntime rt(app, n, "", [deep] { return deep; });
  ASSERT_NE(rt.field(), nullptr);
  EXPECT_TRUE(rt.field()->inertial);
  const std::optional<SceneWind> sw = scene_wind_set(app, "");
  ASSERT_TRUE(sw.has_value());
  if (!jolt())
    return;
  auto run = [&](bool inertial) {
    cvc::physics::world_config c;
    c.backend = cvc::physics::backend::jolt;
    cvc::physics::world w(c);
    cvc::physics::field water = *rt.field();
    water.inertial = inertial;
    w.add_field(water);
    cvc::physics::field air;
    air.kind = cvc::physics::field_kind::flow;
    air.wind = scene_wind_flow(*sw, 0.0);
    w.add_field(air);
    cvc::physics::body_desc d;
    d.shape = cvc::physics::box{{0.5, 0.5, 0.25}};
    d.density = 500.0;
    d.position = {0.0, 0.0, 0.0};
    d.linear_damping = 0.0;
    d.allow_sleeping = false;
    const cvc::physics::body_id b = w.add_body(d);
    w.step(static_cast<std::uint64_t>(30.0 / c.fixed_dt + 0.5)); // 30 s
    return w.state(b).position;
  };
  const cvc::physics::vec3 drifted = run(true), held = run(false);
  std::printf("[water] a crate on the water in a 10 m/s wind, 30 s: %.2f m downwind (%.2f m "
              "across); damped in every direction, %.2f m\n",
              drifted.x, drifted.y, held.x);
  EXPECT_GT(drifted.x, 3.0) << "it drifts downwind, about 0.15 m/s";
  // The waves' spread of directions carries it to and fro across the wind too.
  EXPECT_LT(std::fabs(drifted.y), 0.5 * drifted.x);
  EXPECT_LT(std::fabs(held.x), 0.5) << "the old damping held it";
}

TEST(AriadneSceneWorld, RegenerateMakesANewWorldAndSwapsItsBodies) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(kGenerated);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "", &warnings);
  ASSERT_NE(w, nullptr);
  std::shared_ptr<const cvc::world::generated_world> first = w->generated_world("world");
  ASSERT_NE(first, nullptr);

  // An edit alone only marks the world pending (world.auto is off).
  cvc::state::instance(app)("world.seed").value("11");
  w->sync(&warnings);
  EXPECT_EQ(w->generated_world("world"), first);
  EXPECT_EQ(val(app, "world.pending"), "1");

  // Regenerate makes the new world at once, and the node's bodies are its colliders now.
  cvc::state::instance(app)("world.request").value("regenerate");
  w->sync(&warnings);
  std::shared_ptr<const cvc::world::generated_world> second = w->generated_world("world");
  ASSERT_NE(second, nullptr);
  EXPECT_NE(second, first);
  EXPECT_EQ(second->params.seed, 11u);
  EXPECT_EQ(val(app, "world.request"), "");
  EXPECT_EQ(val(app, "world.pending"), "0");
  EXPECT_EQ(w->bodies_of("world").size(), collide_items(*second) + kEdgeWalls);
  EXPECT_EQ(w->world().body_count(), collide_items(*second) + kEdgeWalls + 1);
  EXPECT_TRUE(w->body("ball").valid());
}

TEST(AriadneSceneWorld, AWorldNodesGridFollowsItsWorld) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(kGenerated);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "", &warnings);
  ASSERT_NE(w, nullptr);
  // The node's world, and the navigator's grid over its terrain.
  std::uint64_t rev = 0;
  ASSERT_NE(w->world_geometry("world", &rev), nullptr);
  EXPECT_EQ(rev, 1u);
  EXPECT_EQ(w->world_geometry("ball"), nullptr); // a node with a body of its own makes no world
  std::shared_ptr<const cvc::nav::world_grid> g = w->grid("world", 5.0, 0.0, &warnings);
  ASSERT_NE(g, nullptr);
  const std::shared_ptr<const cvc::height_field> terrain = w->generated_world("world")->terrain;
  EXPECT_DOUBLE_EQ(g->min_x, terrain->x0());
  EXPECT_DOUBLE_EQ(g->cell(), 5.0);
  EXPECT_GT(g->occupied_count(), static_cast<std::size_t>(2 * (g->rows + g->cols) - 4));
  // Made once per world, cell and half.
  EXPECT_EQ(w->grid("world", 5.0), g);
  std::shared_ptr<const cvc::nav::world_grid> cropped = w->grid("world", 5.0, 60.0);
  ASSERT_NE(cropped, nullptr);
  EXPECT_NE(cropped, g);
  EXPECT_EQ(cropped->rows, 25);
  EXPECT_EQ(w->grid("ball", 5.0), nullptr);
  // A new world is a new grid.
  cvc::state::instance(app)("world.seed").value("11");
  cvc::state::instance(app)("world.request").value("regenerate");
  w->sync(&warnings);
  w->world_geometry("world", &rev);
  EXPECT_EQ(rev, 2u);
  std::shared_ptr<const cvc::nav::world_grid> g2 = w->grid("world", 5.0);
  ASSERT_NE(g2, nullptr);
  EXPECT_NE(g2, g);
}

TEST(AriadneSceneWorld, ADataNodesBodiesComeWithItsDataset) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(R"(
scene:
  physics: {}
  nodes:
    - node: city
      source: { data: city }
      body: {}
)");
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "", &warnings);
  ASSERT_NE(w, nullptr);
  EXPECT_TRUE(w->bodies_of("city").empty()); // nothing loaded yet
  w->sync(&warnings);
  EXPECT_TRUE(w->bodies_of("city").empty());

  // The dataset is ready: its terrain becomes the node's body.
  cvc::state &root = cvc::state::instance(app);
  root("data.city.terrain")
      .data(std::shared_ptr<const cvc::height_field>(
          std::make_shared<cvc::height_field>(3, 3, -10.0, -10.0, 10.0, 10.0)));
  root("data.city.rev").value("1");
  root("data.city.status").value("ready");
  w->sync(&warnings);
  ASSERT_EQ(w->bodies_of("city").size(), 1u + kEdgeWalls);
  const auto first = w->body("city");
  w->sync(&warnings); // the same rev: nothing changes
  EXPECT_EQ(w->body("city"), first);

  // A reload (a new rev) swaps it; the old body goes.
  root("data.city.terrain")
      .data(std::shared_ptr<const cvc::height_field>(
          std::make_shared<cvc::height_field>(5, 5, -10.0, -10.0, 5.0, 5.0)));
  root("data.city.rev").value("2");
  w->sync(&warnings);
  ASSERT_EQ(w->bodies_of("city").size(), 1u + kEdgeWalls);
  EXPECT_EQ(w->world().body_count(), 1u + kEdgeWalls);
  EXPECT_FALSE(w->world().contains(first));

  // Without the physics: block a data node's body warns, as cvcGL does.
  LoadResult bare = load_string(app, R"(
scene:
  nodes:
    - node: city
      source: { data: city }
      body: {}
)");
  ASSERT_TRUE(bare.ok) << bare.error;
  std::vector<std::string> bare_warnings;
  EXPECT_EQ(SceneWorld::from_scene(app, bare.scene, "", "", &bare_warnings), nullptr);
  EXPECT_TRUE(any_contains(bare_warnings, "needs the scene's physics: block"));
}

namespace {

// Three worlds a scene can be, and a ball that falls on whichever is on.
const char *kWorlds = R"(
scene:
  physics: {}
  world: { source: flat, cell: 4 }
  nodes:
    - node: land
      type: generated_world
      world: generated
      seed: 5
      half: 150
      trees: 4
      rocks: 2
      buildings: 4
      body: {}
    - node: hill
      world: terrain
      source:
        heightfield:
          size: 40
          resolution: 21
          layers:
            - { kind: dome, amplitude: 2.0, radius: 10 }
      body: static
    - node: square
      world: flat
      source: { box: { size: [60, 60, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - node: ball
      source: { sphere: 0.5 }
      transform: { position: [0, 0, 5] }
      body: { density: 400 }
)";

} // namespace

TEST(AriadneSceneWorld, AWorldNodesVisibilityFollowsItsWorld) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(kWorlds);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(r.scene.world.present);
  EXPECT_EQ(r.scene.world.source, "flat");
  EXPECT_DOUBLE_EQ(r.scene.world.cell, 4.0);
  ASSERT_EQ(r.scene.nodes.size(), 4u);
  EXPECT_EQ(r.scene.nodes[0].world, "generated");
  EXPECT_EQ(r.scene.nodes[0].visible_bind, "scene.world.on.land");
  EXPECT_FALSE(r.scene.nodes[0].visible_default);     // not the world on at the start
  EXPECT_TRUE(r.scene.nodes[2].visible_default);      // the flat one is
  EXPECT_TRUE(r.scene.nodes[3].visible_bind.empty()); // the ball is in every world
  EXPECT_TRUE(scene_has_world(r.scene));
}

TEST(AriadneSceneWorld, TheScenesWorldsAreOnOneAtATime) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(kWorlds);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  // The block's world is on; the others' bodies are out.
  EXPECT_EQ(w->world_on(), "flat");
  EXPECT_EQ(val(app, "scene.world.source"), "flat");
  EXPECT_EQ(val(app, "scene.world.sources"), "generated\nterrain\nflat");
  EXPECT_EQ(val(app, "scene.world.kind"), "flat");
  EXPECT_EQ(val(app, "scene.world.node"), "square");
  EXPECT_EQ(val(app, "scene.world.ready"), "1");
  EXPECT_EQ(val(app, "scene.world.rev"), "1");
  EXPECT_EQ(val(app, "scene.world.on.square"), "1");
  EXPECT_EQ(val(app, "scene.world.on.land"), "0");
  EXPECT_EQ(val(app, "scene.world.on.hill"), "0");
  EXPECT_EQ(w->bodies_of("square").size(), 1u);
  EXPECT_TRUE(w->bodies_of("land").empty());
  EXPECT_TRUE(w->bodies_of("hill").empty());
  EXPECT_TRUE(w->body("ball").valid());
  // The flat ground's grid: its 60 m footprint at 4 m.
  EXPECT_EQ(val(app, "scene.world.grid.rows"), "16");
  EXPECT_NE(val(app, "scene.world.grid.summary").find("16 x 16 at 4 m"), std::string::npos)
      << val(app, "scene.world.grid.summary");

  // The key switches the world: the generated one's colliders come in, the square's go.
  cvc::state::instance(app)("scene.world.source").value("generated");
  w->sync(&warnings);
  EXPECT_EQ(w->world_on(), "generated");
  const std::shared_ptr<const cvc::world::generated_world> gw = w->generated_world("land");
  ASSERT_NE(gw, nullptr);
  EXPECT_EQ(w->bodies_of("land").size(), collide_items(*gw) + kEdgeWalls);
  EXPECT_TRUE(w->bodies_of("square").empty());
  EXPECT_EQ(val(app, "scene.world.kind"), "generated");
  EXPECT_EQ(val(app, "scene.world.rev"), "2");
  EXPECT_EQ(val(app, "scene.world.on.land"), "1");
  EXPECT_EQ(val(app, "scene.world.on.square"), "0");
  EXPECT_EQ(w->world().body_count(), collide_items(*gw) + kEdgeWalls + 1);
  // A new world from the one on is a new revision.
  cvc::state::instance(app)("world.request").value("randomize");
  w->sync(&warnings);
  EXPECT_EQ(val(app, "scene.world.rev"), "3");
  EXPECT_EQ(w->bodies_of("land").size(), collide_items(*w->generated_world("land")) + kEdgeWalls);

  // set_world: the terrain, its grid its field's.
  EXPECT_TRUE(w->set_world("terrain", &warnings));
  EXPECT_EQ(val(app, "scene.world.source"), "terrain");
  EXPECT_EQ(w->bodies_of("hill").size(), 1u);
  EXPECT_TRUE(w->bodies_of("land").empty());
  EXPECT_EQ(val(app, "scene.world.grid.rows"), "11"); // 40 m at 4 m
  EXPECT_EQ(w->world().body_count(), 2u);
  // A name no node has: warned once, and the world on stays.
  warnings.clear();
  EXPECT_FALSE(w->set_world("nosuch", &warnings));
  cvc::state::instance(app)("scene.world.source").value("nosuch");
  w->sync(&warnings);
  w->sync(&warnings);
  EXPECT_EQ(w->world_on(), "terrain");
  std::size_t said = 0;
  for (const std::string &s : warnings)
    said += s.find("no world called 'nosuch'") != std::string::npos ? 1 : 0;
  EXPECT_EQ(said, 2u); // set_world's, and the key's once
  // Restart keeps the world on, and only it.
  w->restart(&warnings);
  EXPECT_EQ(w->bodies_of("hill").size(), 1u);
  EXPECT_TRUE(w->bodies_of("square").empty());
  EXPECT_TRUE(w->bodies_of("land").empty());
  EXPECT_EQ(w->world().body_count(), 2u);
}

TEST(AriadneSceneWorld, PhysicsOffHoldsTheWorldStill) {
  cvc::app app;
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_string(app, kScene);
  ASSERT_TRUE(r.ok) << r.error;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(val(app, "physics.enabled"), "1"); // seeded on
  SceneClockDriver clock(app, resolved_clock(r.scene));
  const auto run = [&](int frames) {
    for (int i = 0; i < frames; ++i) {
      clock.drive(1.0 / 60.0);
      w->follow(app.world_clock());
    }
  };
  run(30);
  const std::uint64_t t1 = w->world().tick();
  const cvc::physics::body_state c1 = w->world().state(w->body("crate"));
  EXPECT_GT(t1, 0u);
  // Off: the clock runs on, the world holds still.
  cvc::state::instance(app)("physics.enabled").value("0");
  run(60);
  EXPECT_EQ(w->world().tick(), t1);
  EXPECT_EQ(w->world().state(w->body("crate")).position.z, c1.position.z);
  // On again: it carries on from there, without catching up the second it was off.
  cvc::state::instance(app)("physics.enabled").value("1");
  run(1);
  EXPECT_GT(w->world().tick(), t1);
  EXPECT_LE(w->world().tick(), t1 + 4);
  // The stats: a history of step times, and the bodies by node.
  run(20);
  const std::string history = val(app, "physics.stats.step_ms_history");
  EXPECT_GE(std::count(history.begin(), history.end(), ','), 10);
  const std::string nodes = val(app, "physics.stats.nodes");
  EXPECT_NE(nodes.find("terrain  1  0"), std::string::npos) << nodes;
  EXPECT_NE(nodes.find("crate  1  "), std::string::npos) << nodes;
  EXPECT_NE(nodes.find("ball  1  "), std::string::npos) << nodes;
}

// ---------------------------------------------------------------------------------------------
// The world's vehicles (ARIADNE_WORLDS.md section 14, L2): archetypes from the world: block,
// spawned as Jolt vehicles into the scene's own physics world.

namespace {

// Two worlds, a crate, and a world: block with three vehicles.
const char *kVehicles = R"(
world:
  vehicles:
    archetypes:
      humvee:
        vehicle: humvee
        ranges: { mass: [2000, 3500], max_speed: [5, 20] }
        controllers: [nav@1, none]
      racer:
        vehicle: { preset: roboracer, max_speed: 6 }
        look: { file: racer.glb, scale: 2, yaw: 90, z: -0.05 }
        controllers: [none]
      walker:
        plant: kinematic
        vehicle: humvee
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [120, 120, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - node: field
      world: field
      source: { box: { size: [80, 80, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - node: crate
      source: { box: { size: [1, 1, 1] } }
      transform: { position: [40, 40, 2] }
      body: { mass: 10 }
)";

// A loaded scene with its world: block's vehicles, as AppRuntime gives it.
LoadResult load_vehicles(const char *text) {
  cvc::app app;
  register_world_block(app);
  LoadResult r = load_string(app, text);
  if (r.ok)
    if (const Value *world = find_world_block(r)) {
      world_rules rules = default_world_rules();
      const std::string e = parse_world_block(*world, rules);
      EXPECT_EQ(e, "");
      r.scene.vehicles = rules.vehicles;
    }
  return r;
}

double planar(const entity_info &a, double x, double y) { return std::hypot(a.x - x, a.y - y); }

} // namespace

TEST(AriadneWorldVehicles, TheWorldBlockDeclaresVehicles) {
  REQUIRE_YAML();
  LoadResult r = load_vehicles(kVehicles);
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.vehicles.size(), 3u);
  const vehicle_archetype &h = r.scene.vehicles[0];
  EXPECT_EQ(h.id, "humvee");
  EXPECT_EQ(h.plant, "jolt");
  EXPECT_EQ(h.look.kind, ""); // the default: the Humvee model when installed, else the box
  EXPECT_EQ(h.ranges.at("mass"), (std::pair<double, double>{2000, 3500}));
  EXPECT_EQ(h.controllers, (std::vector<std::string>{"nav@1", "none"}));
  const vehicle_archetype &rr = r.scene.vehicles[1];
  EXPECT_EQ(rr.preset, "roboracer");
  EXPECT_EQ(rr.overrides.at("max_speed"), 6.0);
  EXPECT_EQ(rr.look.kind, "file");
  EXPECT_EQ(rr.look.path, "racer.glb");
  EXPECT_EQ(rr.look.yaw, 90.0);
  EXPECT_EQ(r.scene.vehicles[2].plant, "kinematic");

  // The avatars are as they were: the vehicles are a list of their own.
  world_rules rules = default_world_rules();
  EXPECT_EQ(parse_world_block(*find_world_block(r), rules), "");
  EXPECT_EQ(rules.avatar().id, "person");
  ASSERT_NE(rules.find_vehicle("racer"), nullptr);

  // What is wrong is said.
  const auto error = [](const std::string &archetype) {
    cvc::app app; // its own, with the world: block
    register_world_block(app);
    LoadResult bad =
        load_string(app, "world:\n  vehicles:\n    archetypes:\n      v: " + archetype +
                             "\nscene:\n  nodes: []\n");
    world_rules x = default_world_rules();
    const Value *block = find_world_block(bad);
    return block ? parse_world_block(*block, x) : std::string("no world: block");
  };
  EXPECT_NE(error("{ vehicle: tank }").find("not a preset"), std::string::npos);
  EXPECT_NE(error("{ plant: hover }").find("plant 'hover'"), std::string::npos);
  EXPECT_NE(error("{ controllers: [walker@1] }").find("nav@1, none, external or pose"),
            std::string::npos);
  EXPECT_NE(error("{ plant: jolt:roboracer }").find("only dynamic, dynamic4w and dynamicboat"),
            std::string::npos);
  EXPECT_NE(error("{ plant: dynamic4w:roboracer, vehicle: humvee }").find("names preset"),
            std::string::npos);
  EXPECT_EQ(error("{ plant: dynamic4w:roboracer }"), "");
  EXPECT_NE(error("{ ranges: { mass: [10, 5] } }").find("ranges: mass"), std::string::npos);
  EXPECT_NE(error("{ look: { file: '' } }").find("names nothing"), std::string::npos);
  EXPECT_EQ(error("{ vehicle: roboracer }"), "");
}

namespace {

// An environment variable set for a scope, and put back after.
struct EnvFor {
  std::string name, old;
  bool had = false;
  EnvFor(const char *n, const std::string &value) : name(n) {
    if (const char *v = std::getenv(n)) {
      had = true;
      old = v;
    }
    set(value);
  }
  ~EnvFor() { set(had ? old : std::string()); }
  void set(const std::string &v) {
#ifdef _WIN32
    _putenv_s(name.c_str(), v.c_str());
#else
    if (v.empty())
      unsetenv(name.c_str());
    else
      setenv(name.c_str(), v.c_str(), 1);
#endif
  }
};

} // namespace

TEST(AriadneWorldVehicles, LooksAreFoundWhereTheyAre) {
  cvc::app app;
  // A prefix with the vehicle-humvee package's model where cvcpkg puts it.
  const fs::path prefix = fs::temp_directory_path() / "ariadne_world_vehicles_prefix";
  fs::create_directories(prefix / "share" / "cvc-scenes" / "shared");
  std::ofstream(prefix / "share" / "cvc-scenes" / "shared" / "Humvee.glb") << "glb";
  EnvFor bundle("CVC_NAV_BUNDLE", "");
  EnvFor env("CVC_PREFIX", prefix.string());
  const std::string humvee = find_vehicle_model("humvee");
  EXPECT_NE(humvee.find("Humvee.glb"), std::string::npos) << humvee;
  EXPECT_EQ(find_vehicle_model("no_such_vehicle.glb"), "");
  // A name without an extension is its .glb (the vehicle-tactical-boat package's TacticalBoat).
  std::ofstream(prefix / "share" / "cvc-scenes" / "shared" / "TacticalBoat.glb") << "glb";
  const std::string boat = find_vehicle_model("TacticalBoat");
  EXPECT_NE(boat.find("TacticalBoat.glb"), std::string::npos) << boat;
  EXPECT_EQ(find_vehicle_model("TacticalBoat.glb"), boat);

  std::vector<vehicle_archetype> v(6);
  v[0].id = "humvee"; // no look: the model, fitted
  v[1].id = "racer";  // no look and no model: its box
  v[1].preset = "roboracer";
  v[2].id = "tank"; // a model that is not installed: its box, and a warning
  v[2].look.kind = "model";
  v[2].look.path = "tank";
  v[3].id = "beside"; // a file beside the document
  v[3].look.kind = "file";
  v[3].look.path = "car.glb";
  v[4].id = "asset"; // a document asset's file
  v[4].look.kind = "asset";
  v[4].look.path = "car";
  v[5].id = "patrol"; // no look, the tactical boat preset: its model, fitted
  v[5].plant = "dynamicboat";
  v[5].preset = "tactical";
  SceneAsset car;
  car.id = "car";
  car.source.source_file = "models/car.glb";
  car.source.base_dir = (prefix / "doc").string();
  std::vector<std::string> warnings;
  resolve_vehicle_looks(v, (prefix / "doc").string(), {car}, &warnings);
  EXPECT_EQ(v[0].look.kind, "file");
  EXPECT_EQ(v[0].look.path, humvee);
  EXPECT_TRUE(v[0].look.fit);
  EXPECT_EQ(v[1].look.kind, "box");
  EXPECT_EQ(v[5].look.kind, "file");
  EXPECT_EQ(v[5].look.path, boat);
  EXPECT_TRUE(v[5].look.fit);
  EXPECT_EQ(v[2].look.kind, "box");
  EXPECT_TRUE(any_contains(warnings, "model 'tank' is not installed"));
  EXPECT_EQ(fs::path(v[3].look.path), prefix / "doc" / "car.glb");
  EXPECT_FALSE(v[3].look.fit);
  EXPECT_EQ(v[4].look.kind, "file");
  EXPECT_EQ(fs::path(v[4].look.path), prefix / "doc" / "models" / "car.glb");

  // `model:` in the world: block.
  REQUIRE_YAML();
  register_world_block(app);
  LoadResult r = load_string(app, "world:\n  vehicles:\n    archetypes:\n      h: { look: { model: "
                                  "humvee, yaw: 180 } }\nscene:\n  nodes: []\n");
  world_rules rules = default_world_rules();
  ASSERT_NE(find_world_block(r), nullptr);
  EXPECT_EQ(parse_world_block(*find_world_block(r), rules), "");
  EXPECT_EQ(rules.vehicles[0].look.kind, "model");
  EXPECT_EQ(rules.vehicles[0].look.path, "humvee");
  EXPECT_TRUE(rules.vehicles[0].look.fit);
  EXPECT_EQ(rules.vehicles[0].look.yaw, 180.0);
}

// A look from a document's data: entry: `look: { data: NAME, file: FILE }` parses to a data look,
// fitted unless fit: false, and resolve_vehicle_looks copies the entry's source and path: into it,
// so the look is whole without the document (a client draws it from the same package); the loader
// gives the scene the entries, their relative sources made absolute.
TEST(AriadneWorldVehicles, LooksComeFromADataPackage) {
  REQUIRE_YAML();
  cvc::app app;
  register_world_block(app);
  LoadResult r = load_string(app, R"(
data:
  humvee: { uri: cvcpkg://cy-pca/vehicle-humvee, path: share/cvc-scenes/shared, load: lazy }
  local: { uri: models/pkg, load: lazy }
world:
  vehicles:
    archetypes:
      h: { vehicle: humvee, look: { data: humvee, file: Humvee.glb } }
      flat: { look: { file: car.glb, data: local, fit: false } }
      lost: { look: { data: no_such_entry, file: x.glb } }
scene:
  nodes: []
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.data.size(), 2u);
  EXPECT_EQ(r.scene.data[0].name, "humvee");
  EXPECT_EQ(r.scene.data[0].uri, "cvcpkg://cy-pca/vehicle-humvee");
  EXPECT_EQ(r.scene.data[0].path, "share/cvc-scenes/shared");
  world_rules rules = default_world_rules();
  ASSERT_NE(find_world_block(r), nullptr);
  EXPECT_EQ(parse_world_block(*find_world_block(r), rules), "");
  ASSERT_EQ(rules.vehicles.size(), 3u);
  EXPECT_EQ(rules.vehicles[0].look.kind, "data");
  EXPECT_EQ(rules.vehicles[0].look.data, "humvee");
  EXPECT_EQ(rules.vehicles[0].look.path, "Humvee.glb");
  EXPECT_TRUE(rules.vehicles[0].look.fit);
  EXPECT_EQ(rules.vehicles[1].look.kind, "data") << "data: and file: in either order";
  EXPECT_FALSE(rules.vehicles[1].look.fit);
  std::vector<std::string> warnings;
  resolve_vehicle_looks(rules.vehicles, "", r.scene.assets, &warnings, r.scene.data);
  EXPECT_EQ(rules.vehicles[0].look.uri, "cvcpkg://cy-pca/vehicle-humvee");
  EXPECT_EQ(rules.vehicles[0].look.subpath, "share/cvc-scenes/shared");
  EXPECT_EQ(rules.vehicles[0].look.path, "Humvee.glb");
  EXPECT_EQ(rules.vehicles[2].look.kind, "box");
  EXPECT_TRUE(any_contains(warnings, "no data: entry 'no_such_entry'"));
  // Resolved once, it travels whole: resolving it again (a host that got it resolved) keeps it.
  std::vector<vehicle_archetype> again = {rules.vehicles[0]};
  resolve_vehicle_looks(again, "", {}, nullptr, {});
  EXPECT_EQ(again[0].look.uri, "cvcpkg://cy-pca/vehicle-humvee");
  EXPECT_EQ(again[0].look.kind, "data");
  // data: needs file:.
  LoadResult bad =
      load_string(app, "world:\n  vehicles:\n    archetypes:\n      h: { look: { data: "
                       "humvee } }\nscene:\n  nodes: []\n");
  world_rules br = default_world_rules();
  ASSERT_NE(find_world_block(bad), nullptr);
  EXPECT_NE(parse_world_block(*find_world_block(bad), br).find("data: needs file:"),
            std::string::npos);
  // A document on disk: its relative data sources are made absolute against its directory.
  const fs::path dir = fs::temp_directory_path() / "ariadne_data_looks";
  fs::create_directories(dir);
  const fs::path doc = dir / "doc.ari";
  std::ofstream(doc) << "data:\n  local: { uri: models/pkg }\nscene:\n  nodes: []\n";
  LoadResult onfile = load_file(app, doc.string());
  ASSERT_TRUE(onfile.ok) << onfile.error;
  ASSERT_EQ(onfile.scene.data.size(), 1u);
  // Both sides canonical: the temp directory can come as a short name (C:\Users\RUNNER~1\...)
  // that the loader resolves to the long one.
  EXPECT_EQ(fs::weakly_canonical(fs::path(onfile.scene.data[0].uri)),
            fs::weakly_canonical(dir / "models" / "pkg"));
  std::error_code ec;
  fs::remove_all(dir, ec);
}

TEST(AriadneWorldVehicles, ArchetypeStatsAndSettings) {
  vehicle_archetype a;
  a.id = "humvee";
  a.ranges["mass"] = {2000.0, 3500.0};
  a.ranges["max_speed"] = {5.0, 20.0};
  const cvc::physics::vehicle_desc d = vehicle_desc_of(a);
  // The preset's mass, its top speed held to the range.
  EXPECT_DOUBLE_EQ(d.mass, cvc::physics::vehicle_desc::humvee().mass);
  EXPECT_DOUBLE_EQ(d.max_speed, 20.0);
  // A spawn's settings, held to the ranges.
  EXPECT_DOUBLE_EQ(vehicle_desc_of(a, 9000.0, 1.0).mass, 3500.0);
  EXPECT_DOUBLE_EQ(vehicle_desc_of(a, 9000.0, 1.0).max_speed, 5.0);
  EXPECT_DOUBLE_EQ(vehicle_desc_of(a, 2500.0, 12.0).mass, 2500.0);
  const vehicle_stats s = vehicle_stats_of(cvc::physics::vehicle_desc::humvee());
  EXPECT_NEAR(s.length, 4.84, 1e-9);
  EXPECT_NEAR(s.wheelbase, 3.30, 1e-9);
  EXPECT_GT(s.power_kw, 50.0);
  EXPECT_LT(s.power_kw, 400.0);
  a.preset = "tank";
  EXPECT_THROW(vehicle_desc_of(a), std::invalid_argument);
}

TEST(AriadneWorldVehicles, VehiclesDriveInTheScenesPhysicsWorld) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_vehicles(kVehicles);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  WorldVehicles *v = w->vehicles();
  ASSERT_NE(v, nullptr);
  // The catalog, in the document's order, and the panel's keys seeded.
  EXPECT_EQ(val(app, "vehicles.archetypes.names"), "humvee\nracer\nwalker");
  EXPECT_EQ(val(app, "vehicles.archetypes.humvee.length"), "4.84");
  EXPECT_EQ(val(app, "vehicles.archetypes.racer.look.kind"), "file");
  EXPECT_EQ(val(app, "vehicles.archetypes.humvee.ranges.mass.hi"), "3500");
  EXPECT_EQ(val(app, "vehicles.selected"), "humvee");
  EXPECT_NE(val(app, "vehicles.selection.summary").find("4.84 x 2.18"), std::string::npos)
      << val(app, "vehicles.selection.summary");

  w->sync(&warnings); // the ground: the square's grid
  EXPECT_EQ(v->status(), "ready");
  const std::size_t bodies = w->world().body_count();
  entity_spec driver;
  driver.archetype = "humvee";
  driver.name = "Ada";
  driver.team = 2;
  driver.metadata["note"] = "first";
  driver.placed = true;
  driver.x = -30.0;
  driver.y = -30.0;
  std::string why;
  const std::uint64_t a = v->spawn(driver, &why);
  ASSERT_NE(a, 0u) << why;
  entity_spec parked;
  parked.archetype = "racer"; // its first controller: none
  parked.placed = true;
  parked.x = 20.0;
  parked.y = -20.0;
  const std::uint64_t b = v->spawn(parked, &why);
  ASSERT_NE(b, 0u) << why;
  EXPECT_EQ(w->world().vehicle_count(), 2u);
  EXPECT_EQ(w->world().body_count(), bodies + 2); // each chassis is a body in the scene's world
  const std::vector<entity_info> start = v->entities();
  ASSERT_EQ(start.size(), 2u);
  EXPECT_TRUE(start[0].placed);
  EXPECT_EQ(start[0].spec.controller, "nav@1");
  EXPECT_EQ(start[1].spec.controller, "none");
  EXPECT_EQ(start[1].spec.name, "racer 2");
  EXPECT_DOUBLE_EQ(start[1].spec.max_speed, 6.0);

  // The world clock runs them with the scene's bodies: nav@1 drives off, none holds still on the
  // ground.
  SceneClockDriver clock(app, resolved_clock(r.scene));
  for (int i = 0; i < 360; ++i) { // 6 s
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
  }
  const std::vector<entity_info> now = v->entities();
  EXPECT_GT(planar(now[0], start[0].x, start[0].y), 5.0) << now[0].x << "," << now[0].y;
  EXPECT_GT(now[0].distance, 5.0);
  EXPECT_GT(now[0].energy_wh, 0.0);
  EXPECT_GT(now[0].wheel_load, 1000.0); // its weight on the square
  EXPECT_LT(planar(now[1], start[1].x, start[1].y), 0.5);
  EXPECT_GT(now[1].z, 0.0);
  EXPECT_LT(now[1].z, 1.0); // a roboracer standing on the square, not through it
  // Their keys and the telemetry.
  EXPECT_EQ(val(app, "entities.ids"), "1,2");
  EXPECT_EQ(val(app, "entities.count"), "2");
  EXPECT_EQ(val(app, "entities.1.name"), "Ada");
  EXPECT_EQ(val(app, "entities.1.team"), "2");
  EXPECT_EQ(val(app, "entities.1.metadata.note"), "first");
  EXPECT_EQ(val(app, "entities.1.controller"), "nav@1");
  EXPECT_EQ(val(app, "entities.2.archetype"), "racer");
  EXPECT_FALSE(val(app, "entities.1.speed").empty());
  const std::string lines = val(app, "physics.stats.vehicles");
  EXPECT_NE(lines.find("1  Ada  "), std::string::npos) << lines;
  EXPECT_NE(lines.find("2  racer 2  "), std::string::npos) << lines;

  // Removed: its vehicle leaves the world and its keys go.
  EXPECT_TRUE(v->despawn(b));
  EXPECT_EQ(w->world().vehicle_count(), 1u);
  clock.drive(1.0 / 60.0);
  w->follow(app.world_clock());
  EXPECT_EQ(val(app, "entities.ids"), "1");
  EXPECT_EQ(cvc::state::instance(app).findDescendant("entities.2"), nullptr);
}

TEST(AriadneWorldVehicles, SpawnsAndRemovesFromTheKeys) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_vehicles(kVehicles);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  cvc::state &st = cvc::state::instance(app);
  w->sync(&warnings);
  // The settings: a fraction of the archetype's range, shown as the vehicle a spawn would make.
  st("vehicles.spawn.mass_t").value("1");
  w->sync(&warnings);
  EXPECT_EQ(val(app, "vehicles.selection.mass"), "3500");
  EXPECT_EQ(val(app, "vehicles.selection.mass_text"), "3500 kg (2000-3500 kg)");
  // A spawn: the panel's keys, then a new request.
  st("vehicles.spawn.name").value("Bo");
  st("vehicles.spawn.team").value("3");
  st("vehicles.spawn.color").value("1,0,0");
  st("vehicles.spawn.color2").value("0,0,1");
  st("vehicles.spawn.metadata").value("call=red one, squad=a");
  st("vehicles.spawn.controller").value("none");
  st("vehicles.spawn.at").value("10,5,90");
  st("vehicles.spawn.request").value("1");
  w->sync(&warnings);
  EXPECT_EQ(val(app, "vehicles.spawn.status"), "spawned 1: Bo")
      << val(app, "vehicles.spawn.status");
  ASSERT_EQ(w->vehicles()->size(), 1u);
  const entity_info e = w->vehicles()->entities()[0];
  EXPECT_EQ(e.spec.team, 3);
  EXPECT_EQ(e.spec.color[0], 1.0f);
  EXPECT_EQ(e.spec.color2[2], 1.0f);
  EXPECT_EQ(e.spec.metadata.at("call"), "red one");
  EXPECT_EQ(e.spec.metadata.at("squad"), "a");
  EXPECT_DOUBLE_EQ(e.spec.mass, 3500.0);
  EXPECT_NEAR(e.yaw, 3.14159265 / 2.0, 0.05);
  // The same request again is not another spawn; a refused one says why.
  w->sync(&warnings);
  EXPECT_EQ(w->vehicles()->size(), 1u);
  st("vehicles.selected").value("racer");
  st("vehicles.spawn.controller").value("nav@1");
  st("vehicles.spawn.request").value("2");
  w->sync(&warnings);
  EXPECT_EQ(val(app, "vehicles.spawn.status"), "vehicle 'racer' does not allow controller 'nav@1'");
  // A plant that is not one is refused; a kinematic one (nav's bicycle) spawns, its chassis box a
  // kinematic body in the physics world (not a vehicle there).
  st("vehicles.selected").value("walker");
  st("vehicles.spawn.controller").value("");
  st("vehicles.spawn.plant").value("hover");
  st("vehicles.spawn.request").value("3");
  w->sync(&warnings);
  EXPECT_NE(val(app, "vehicles.spawn.status").find("no plant 'hover'"), std::string::npos);
  EXPECT_EQ(w->vehicles()->size(), 1u);
  st("vehicles.spawn.plant").value("");
  st("vehicles.spawn.at").value("20,-20");
  st("vehicles.spawn.request").value("4");
  w->sync(&warnings);
  EXPECT_EQ(val(app, "vehicles.spawn.status"), "spawned 2: Bo"); // a kinematic plant
  EXPECT_EQ(w->vehicles()->size(), 2u);
  EXPECT_EQ(w->world().vehicle_count(), 1u);
  EXPECT_EQ(w->vehicles()->entities()[1].plant, "kinematic");
  EXPECT_EQ(w->vehicles()->entities()[1].spec.plant, "kinematic");
  EXPECT_TRUE(w->vehicles()->entities()[1].chassis.valid());
  // Removed by id, then a clear.
  st("vehicles.selected").value("humvee");
  st("vehicles.spawn.at").value("");
  st("vehicles.spawn.request").value("5");
  w->sync(&warnings);
  EXPECT_EQ(w->vehicles()->size(), 3u);
  st("vehicles.remove.id").value("1");
  st("vehicles.remove.request").value("1");
  w->sync(&warnings);
  EXPECT_EQ(w->vehicles()->size(), 2u);
  EXPECT_EQ(w->world().vehicle_count(), 1u); // the humvee; the walker's box is not a vehicle
  st("vehicles.clear.request").value("1");
  w->sync(&warnings);
  EXPECT_EQ(w->vehicles()->size(), 0u);
  EXPECT_EQ(w->world().vehicle_count(), 0u);
}

TEST(AriadneWorldVehicles, RestartAndASwitchPlaceThemAgain) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_vehicles(kVehicles);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  WorldVehicles *v = w->vehicles();
  w->sync(&warnings);
  entity_spec s;
  s.archetype = "humvee";
  s.placed = true;
  s.x = -20.0;
  s.y = 10.0;
  ASSERT_NE(v->spawn(s), 0u);
  const entity_info start = v->entities()[0];
  SceneClockDriver clock(app, resolved_clock(r.scene));
  for (int i = 0; i < 240; ++i) {
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
  }
  EXPECT_GT(planar(v->entities()[0], start.x, start.y), 3.0);
  // Restart: a fresh physics world, the vehicle back where it started, at rest.
  const std::uint64_t rev = v->revision();
  w->restart(&warnings);
  EXPECT_GT(v->revision(), rev);
  EXPECT_EQ(w->world().vehicle_count(), 1u);
  const entity_info back = v->entities()[0];
  EXPECT_TRUE(back.placed);
  EXPECT_LT(planar(back, start.x, start.y), 0.5);
  EXPECT_NEAR(back.speed, 0.0, 1e-6);
  // Another world: the vehicle on its ground (its start is free there too).
  EXPECT_TRUE(w->set_world("field", &warnings));
  w->sync(&warnings);
  EXPECT_EQ(w->world().vehicle_count(), 1u);
  EXPECT_TRUE(v->entities()[0].placed);
  EXPECT_LT(planar(v->entities()[0], start.x, start.y), 0.5);
}

TEST(AriadneWorldVehicles, AHeadlessAppRuntimeTakesTheDocumentsVehicles) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "");
  const std::vector<std::string> warns = art.load(
      write_doc("vehicles", std::string("requires: [scene.physics]\n") + kVehicles).string());
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  ASSERT_NE(art.scene_world(), nullptr);
  ASSERT_NE(art.scene_world()->vehicles(), nullptr);
  EXPECT_EQ(val(app, "vehicles.archetypes.names"), "humvee\nracer\nwalker");
  // A file look is a path beside the document.
  const std::string path = val(app, "vehicles.archetypes.racer.look.path");
  EXPECT_NE(path.find("ariadne_scene_world_test"), std::string::npos) << path;
  EXPECT_NE(path.find("racer.glb"), std::string::npos) << path;
  cvc::state::instance(app)("vehicles.spawn.request").value("1");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (art.scene_world()->vehicles()->size() == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    art.drain();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(art.scene_world()->vehicles()->size(), 1u) << val(app, "vehicles.spawn.status");
  EXPECT_FALSE(any_contains(warns, "world:"));
}

// Vehicles steer around each other (WorldVehicles::set_avoid, on by default): two driving head on
// with their goals swapped pass with more room than they do without it, on one plant and across
// two (the other plant's vehicles are the navigator's foreign agents).
namespace {
const char *kAvoid = R"(
world:
  vehicles:
    archetypes:
      car: { plant: kinematic, vehicle: { preset: humvee, max_speed: 6 }, controllers: [nav@1] }
      bike: { plant: dynamic, vehicle: { preset: humvee, max_speed: 6 }, controllers: [nav@1] }
scene:
  physics: {}
  world: { source: square, cell: 1 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [120, 120, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)";

// Two vehicles driving at each other from 50 m apart, 0.5 m off one line: the closest they come,
// and where each ends (the car drives west, the second east).
struct pass_result {
  double closest = 1e9, car_x = 0.0, second_x = 0.0;
};
pass_result closest_pass(const char *second, bool avoid) {
  LoadResult r = load_vehicles(kAvoid);
  EXPECT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  pass_result out;
  if (!w)
    return out;
  WorldVehicles *v = w->vehicles();
  v->set_avoid(avoid);
  v->set_wander(false);
  w->sync(&warnings);
  // The second faces +x from the west; the car faces it from the east.
  entity_spec a;
  a.archetype = "car";
  a.placed = true;
  a.x = 25.0;
  a.y = 0.5;
  a.yaw = 3.14159265358979323846;
  entity_spec b = a;
  b.archetype = second;
  b.x = -25.0;
  b.y = 0.0;
  b.yaw = 0.0;
  const std::uint64_t ia = v->spawn(a), ib = v->spawn(b);
  EXPECT_NE(ia, 0u);
  EXPECT_NE(ib, 0u);
  v->set_goal(ia, -30.0, 0.5);
  v->set_goal(ib, 30.0, 0.0);
  SceneClockDriver clock(app, resolved_clock(r.scene));
  for (int i = 0; i < 900; ++i) {
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
    const std::vector<entity_info> e = v->entities();
    if (e.size() == 2 && e[0].placed && e[1].placed) {
      out.closest = std::min(out.closest, std::hypot(e[0].x - e[1].x, e[0].y - e[1].y));
      out.car_x = e[0].x;
      out.second_x = e[1].x;
    }
  }
  return out;
}
} // namespace

TEST(AriadneWorldVehicles, VehiclesSteerAroundEachOtherOnOneAndManyPlants) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  for (const char *second : {"car", "bike"}) {
    const pass_result off = closest_pass(second, false), on = closest_pass(second, true);
    std::printf("  head on, car and %s: without avoiding closest %.2f m, ends at x %.1f and %.1f; "
                "with, %.2f m, %.1f and %.1f\n",
                second, off.closest, off.car_x, off.second_x, on.closest, on.car_x, on.second_x);
    // With it, each swings right and drives on past the other, with room between them.
    EXPECT_LT(on.car_x, -20.0) << "car and " << second << ": the car got past";
    EXPECT_GT(on.second_x, 20.0) << "car and " << second << ": the " << second << " got past";
    EXPECT_GT(on.closest, 2.0) << "car and " << second;
    // Without, they meet: two kinematic boxes drive through each other, and a kinematic box
    // shoves a dynamic chassis back the way it came.
    if (std::string(second) == "car")
      EXPECT_LT(off.closest, 1.0);
    else
      EXPECT_LT(off.second_x, 0.0) << "the bike never got past the car";
  }
  // The key turns it off and on, and says which.
  LoadResult r = load_vehicles(kAvoid);
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  WorldVehicles *v = w->vehicles();
  EXPECT_TRUE(v->avoid());
  EXPECT_EQ(val(app, "vehicles.avoid"), "1");
  cvc::state::instance(app)("vehicles.avoid").value("0");
  v->poll();
  EXPECT_FALSE(v->avoid());
}

// What the navigators know (WorldVehicles::set_sense, the vehicles.sense key): the world's grid
// whole (known, the default), or nothing at first and what their vehicles' rays find (fog).
TEST(AriadneWorldVehicles, UnderFogTheNavigatorsSenseTheWorld) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const char *doc = R"(
world:
  vehicles:
    archetypes:
      car: { plant: kinematic, vehicle: { preset: humvee, max_speed: 6 }, controllers: [nav@1] }
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [120, 120, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - { node: b1, source: { box: { size: [10, 10, 6] } }, transform: { position: [-20, 0, 3] },
        body: static }
    - { node: b2, source: { box: { size: [10, 10, 6] } }, transform: { position: [20, 0, 3] },
        body: static }
    - { node: b3, source: { box: { size: [10, 10, 6] } }, transform: { position: [0, 20, 3] },
        body: static }
)";
  const auto blocked = [](const std::vector<std::uint8_t> &occ) {
    std::size_t n = 0;
    for (std::uint8_t c : occ)
      n += c ? 1 : 0;
    return n;
  };
  LoadResult r = load_vehicles(doc);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  WorldVehicles *v = w->vehicles();
  EXPECT_EQ(v->sense(), "known");
  EXPECT_EQ(val(app, "vehicles.sense"), "known");
  w->sync(&warnings);
  for (int i = 0; i < 3; ++i) {
    entity_spec e;
    e.archetype = "car";
    ASSERT_NE(v->spawn(e), 0u);
  }
  SceneClockDriver clock(app, resolved_clock(r.scene));
  const auto run = [&](int frames) {
    for (int k = 0; k < frames; ++k) {
      clock.drive(1.0 / 60.0);
      w->follow(app.world_clock());
    }
  };
  run(10);
  std::vector<std::uint8_t> known;
  int rows = 0, cols = 0;
  ASSERT_TRUE(v->belief("kinematic", known, rows, cols));
  ASSERT_EQ(known.size(), static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols));
  const std::size_t truth = blocked(known);
  EXPECT_GT(truth, 0u) << "known: the grid, blocks and all";

  EXPECT_FALSE(v->set_sense("xray"));
  ASSERT_TRUE(v->set_sense("fog"));
  EXPECT_EQ(val(app, "vehicles.sense"), "fog");
  run(1);
  std::vector<std::uint8_t> fog;
  ASSERT_TRUE(v->belief("kinematic", fog, rows, cols));
  const std::size_t first = blocked(fog);
  EXPECT_LT(first, truth) << "fog: it starts knowing less than the grid";
  run(600);
  ASSERT_TRUE(v->belief("kinematic", fog, rows, cols));
  const std::size_t later = blocked(fog);
  EXPECT_GT(later, first) << "its rays found blocks";
  EXPECT_LE(later, truth + truth / 4) << "and what it found is (about) what is there";
  std::printf("  fog: %zu blocked cells at first, %zu after 10 s; the grid has %zu\n", first, later,
              truth);
  // The key sets it back.
  cvc::state::instance(app)("vehicles.sense").value("known");
  v->poll();
  EXPECT_EQ(v->sense(), "known");
}

// ---------------------------------------------------------------------------------------------
// Entity intents (ARIADNE_WORLDS.md section 14, S1): an entity steered from outside, by its keys,
// a table of intents, a C++ (or Python) setter and a server document's published namespace.

namespace {

const char *kIntents = R"(
world:
  vehicles:
    intents: /drive
    archetypes:
      humvee:
        vehicle: humvee
        controllers: [nav@1, none, external]
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [160, 160, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)";

LoadResult load_intents() {
  cvc::app app;
  register_world_block(app);
  LoadResult r = load_string(app, kIntents);
  if (r.ok)
    if (const Value *world = find_world_block(r)) {
      world_rules rules = default_world_rules();
      EXPECT_EQ(parse_world_block(*world, rules), "");
      r.scene.vehicles = rules.vehicles;
      r.scene.vehicle_intents = rules.vehicle_intents;
    }
  return r;
}

// One entity of `controller` at (x, y) facing +x.
std::uint64_t spawn_at(WorldVehicles &v, const std::string &controller, double x, double y,
                       const std::string &name = std::string()) {
  entity_spec s;
  s.archetype = "humvee";
  s.controller = controller;
  s.name = name;
  s.placed = true;
  s.x = x;
  s.y = y;
  s.yaw = 0.0;
  std::string why;
  const std::uint64_t id = v.spawn(s, &why);
  EXPECT_NE(id, 0u) << why;
  return id;
}

entity_info entity(const WorldVehicles &v, std::uint64_t id) {
  for (const entity_info &e : v.entities())
    if (e.id == id)
      return e;
  ADD_FAILURE() << "no entity " << id;
  return entity_info{};
}

void run(cvc::app &app, SceneWorld &w, SceneClockDriver &clock, int frames) {
  for (int i = 0; i < frames; ++i) {
    clock.drive(1.0 / 60.0);
    w.follow(app.world_clock());
  }
}

} // namespace

// world: vehicles: start: (the vehicles a world starts with): parsed in order, its archetypes
// checked against archetypes: wherever that is, and spawned when the world's vehicles are made.
TEST(AriadneWorldVehicles, TheWorldStartsWithItsStartVehicles) {
  REQUIRE_YAML();
  const auto parse = [](const std::string &start, world_rules &x) {
    cvc::app app;
    register_world_block(app);
    LoadResult r = load_string(app, "world:\n  vehicles:\n    start: " + start +
                                        "\n    archetypes:\n      v: { plant: kinematic, "
                                        "vehicle: humvee }\nscene:\n  nodes: []\n");
    x = default_world_rules();
    const Value *block = find_world_block(r);
    return block ? parse_world_block(*block, x) : std::string("no world: block");
  };
  world_rules x;
  ASSERT_EQ(parse("[{archetype: v, count: 3, name: car, team: 2}, {archetype: v, at: [4, 5], "
                  "heading: 90, controller: none}]",
                  x),
            "");
  ASSERT_EQ(x.vehicle_starts.size(), 2u);
  EXPECT_EQ(x.vehicle_starts[0].count, 3);
  EXPECT_EQ(x.vehicle_starts[0].name, "car");
  EXPECT_EQ(x.vehicle_starts[0].team, 2);
  EXPECT_FALSE(x.vehicle_starts[0].placed);
  EXPECT_TRUE(x.vehicle_starts[1].placed);
  EXPECT_DOUBLE_EQ(x.vehicle_starts[1].x, 4.0);
  EXPECT_TRUE(x.vehicle_starts[1].has_heading);
  EXPECT_EQ(x.vehicle_starts[1].controller, "none");
  EXPECT_NE(parse("[{archetype: w}]", x).find("names archetype 'w'"), std::string::npos);
  EXPECT_NE(parse("[{count: 2}]", x).find("names no archetype"), std::string::npos);
  EXPECT_NE(parse("[{archetype: v, count: 0}]", x).find("count: expects a whole number"),
            std::string::npos);
  EXPECT_NE(parse("[{archetype: v, count: 257}]", x).find("count: expects a whole number"),
            std::string::npos);
  EXPECT_NE(parse("[{archetype: v, plant: hover}]", x).find("plant: 'hover' is not one"),
            std::string::npos);
  EXPECT_NE(parse("{archetype: v}", x).find("start: expects a list"), std::string::npos);

  // A headless runtime spawns them with the world, before anything asks.
  cvc::app app;
  AppRuntime art(app, "");
  art.load(write_doc("start", R"(requires: [scene.physics]
world:
  vehicles:
    archetypes:
      v: { plant: kinematic, vehicle: humvee, controllers: [nav@1, none] }
    start:
      - { archetype: v, at: [10, -10], heading: 90, controller: none }
      - { archetype: v, count: 3, name: car }
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [120, 120, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)")
               .string());
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  ASSERT_NE(art.scene_world(), nullptr);
  WorldVehicles *v = art.scene_world()->vehicles();
  ASSERT_NE(v, nullptr);
  art.drain();
  const std::vector<entity_info> e = v->entities();
  ASSERT_EQ(e.size(), 4u);
  EXPECT_EQ(e[0].spec.name, "v 1");
  EXPECT_EQ(e[0].spec.controller, "none");
  // at: is the kinematic bicycle's rear axle; the table has its box's centre, ahead of it along
  // the heading (+y) by the axle's offset.
  EXPECT_NEAR(e[0].x, 10.0, 0.5);
  EXPECT_GT(e[0].y, -10.5);
  EXPECT_LT(e[0].y, -7.5);
  EXPECT_EQ(e[1].spec.name, "car 1");
  EXPECT_EQ(e[3].spec.name, "car 3");
}

// A fleet is a named start: entry (fleet:): up to 4096 vehicles, with its layout, seed, rally
// points, colours and keys; an unnamed entry stays at 256 and its fleet keys are ignored.
TEST(AriadneWorldVehicles, AFleetIsANamedStartEntry) {
  REQUIRE_YAML();
  std::vector<std::string> warnings;
  const auto parse = [&](const std::string &start, world_rules &x) {
    cvc::app app;
    register_world_block(app);
    LoadResult r = load_string(app, "world:\n  vehicles:\n    start: " + start +
                                        "\n    archetypes:\n      v: { plant: kinematic, "
                                        "vehicle: humvee }\nscene:\n  nodes: []\n");
    x = default_world_rules();
    warnings.clear();
    const Value *block = find_world_block(r);
    return block ? parse_world_block(*block, x, &warnings) : std::string("no world: block");
  };
  world_rules x;
  ASSERT_EQ(parse("[{fleet: city, archetype: v, count: 800, spawn: opposed, seed: 7, rally: "
                  "{count: 16, assign: latitude}, colors: hsv, keys: summary, plant: kinematic}]",
                  x),
            "");
  ASSERT_EQ(x.vehicle_starts.size(), 1u);
  const vehicle_start &f = x.vehicle_starts[0];
  EXPECT_EQ(f.fleet, "city");
  EXPECT_EQ(f.count, 800);
  EXPECT_EQ(f.layout, "opposed");
  EXPECT_EQ(f.seed, 7u);
  EXPECT_EQ(f.rally, 16);
  EXPECT_EQ(f.assign, "latitude");
  EXPECT_EQ(f.colors, "hsv");
  EXPECT_EQ(f.keys, "summary");
  EXPECT_TRUE(warnings.empty());
  ASSERT_EQ(parse("[{fleet: c, archetype: v, rally: 3}]", x), ""); // rally: T is rally: {count: T}
  EXPECT_EQ(x.vehicle_starts[0].rally, 3);
  ASSERT_EQ(parse("[{archetype: v, count: 2, rally: 4}]", x), "");
  EXPECT_EQ(x.vehicle_starts[0].rally, 0);
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_NE(warnings[0].find("rally: is a fleet's"), std::string::npos);
  const auto error = [&](const std::string &start, const std::string &says) {
    world_rules y;
    const std::string e = parse(start, y);
    EXPECT_NE(e.find(says), std::string::npos) << start << ": " << e;
  };
  error("[{archetype: v, count: 300}]", "name a fleet: for more");
  error("[{fleet: c, archetype: v, count: 4097}]", "count: expects a whole number");
  error("[{fleet: \"two words\", archetype: v}]", "is not a state name");
  error("[{fleet: c, archetype: v}, {fleet: c, archetype: v}]", "is named twice");
  error("[{fleet: c, archetype: v, at: [1, 2]}]", "laid out by spawn:, not at:");
  error("[{fleet: c, archetype: v, controller: pose}]", "controller: pose places one");
  error("[{fleet: c, archetype: v, spawn: ring}]", "spawn: 'ring' is not one");
  error("[{fleet: c, archetype: v, rally: 0}]", "rally: expects a count");
  error("[{fleet: c, archetype: v, rally: {count: 65}}]", "rally: expects a count");
  error("[{fleet: c, archetype: v, rally: {count: 2, assign: longitude}}]", "assign: 'longitude'");
  error("[{fleet: c, archetype: v, colors: rainbow}]", "colors: 'rainbow' is not one");
  error("[{fleet: c, archetype: v, keys: some}]", "keys: 'some' is not one");
  error("[{fleet: c, archetype: v, seed: -1}]", "seed: expects a whole number");
}

namespace {

const char *kFleetDoc = R"(requires: [scene.physics]
world:
  vehicles:
    archetypes:
      v: { plant: kinematic, vehicle: humvee, controllers: [nav@1, none] }
    start:
      - { fleet: city, archetype: v, count: 64, spawn: opposed, seed: 7, rally: { count: 4 } }
      - { archetype: v, name: solo, controller: none, at: [0, 0] }
scene:
  clock: { paused: true }
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [400, 400, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)";

double fleet_mean_x(const std::vector<entity_info> &es) {
  double sum = 0.0;
  int n = 0;
  for (const entity_info &e : es)
    if (e.fleet == "city") {
      sum += e.x;
      ++n;
    }
  return n ? sum / n : 0.0;
}

} // namespace

// A fleet is laid out as the nav demos' swarms are: starts in the west 30 % of the grid, goals in
// the east 30 %, teams by the goals' latitude, each team's goal held at its rally point and its
// colour the team's. The rally points are keys (a write moves the team), a restart puts the fleet
// back where it started and a reseed lays it out anew; a summary fleet writes no per-entity keys
// but for the vehicles vehicles.watch names. The layout is its seed's: the same in another runtime.
TEST(AriadneWorldVehicles, AFleetIsLaidOutInTeamsAtRallyPoints) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const fs::path path = write_doc("fleet_rally", kFleetDoc);
  cvc::app app;
  AppRuntime art(app, "");
  art.load(path.string());
  ASSERT_NE(art.scene_world(), nullptr);
  WorldVehicles *v = art.scene_world()->vehicles();
  ASSERT_NE(v, nullptr);
  art.drain();
  ASSERT_EQ(v->fleets(), std::vector<std::string>{"city"});
  const std::vector<entity_info> es = v->entities();
  ASSERT_EQ(es.size(), 65u);

  std::array<std::array<double, 2>, 4> rally{};
  for (int t = 0; t < 4; ++t) {
    ASSERT_TRUE(v->rally("city", t, rally[t][0], rally[t][1]));
    EXPECT_GT(rally[t][0], 200.0 * 0.4 - 4.0) << t; // in the east 30 % of [-200, 200]
    if (t > 0)
      EXPECT_GT(rally[t][1], rally[t - 1][1]) << t; // one per latitude band, south to north
  }
  EXPECT_FALSE(v->rally("city", 4, rally[0][0], rally[0][1]));
  std::array<int, 4> per_team{};
  std::array<std::array<float, 3>, 4> color{};
  for (int k = 0; k < 64; ++k) {
    const entity_info &e = es[static_cast<std::size_t>(k)];
    EXPECT_EQ(e.spec.name, "city " + std::to_string(k + 1));
    EXPECT_EQ(e.fleet, "city");
    ASSERT_GE(e.rally, 0);
    ASSERT_LT(e.rally, 4);
    EXPECT_EQ(e.spec.team, e.rally);
    EXPECT_LT(e.x, -200.0 * 0.4 + 4.0) << k; // in the west 30 % (the chassis ahead of its axle)
    EXPECT_DOUBLE_EQ(e.goal_x, rally[e.rally][0]);
    EXPECT_DOUBLE_EQ(e.goal_y, rally[e.rally][1]);
    if (per_team[e.rally]++ == 0)
      color[e.rally] = e.spec.color;
    EXPECT_EQ(e.spec.color, color[e.rally]) << k; // the team's colour
  }
  for (int t = 0; t < 4; ++t) {
    EXPECT_GT(per_team[t], 0) << t;
    if (t > 0)
      EXPECT_NE(color[t], color[t - 1]);
  }
  EXPECT_EQ(es[64].spec.name, "solo");
  EXPECT_EQ(es[64].fleet, "");
  EXPECT_EQ(es[64].rally, -1);

  // Its keys, and a summary fleet's entities: only the unnamed entry's have keys.
  art.step_world(6);
  art.drain();
  cvc::state &st = cvc::state::instance(app);
  EXPECT_EQ(val(app, "vehicles.fleets.names"), "city");
  EXPECT_EQ(val(app, "vehicles.fleets.city.count"), "64");
  EXPECT_EQ(val(app, "vehicles.fleets.city.placed"), "64");
  EXPECT_EQ(val(app, "vehicles.fleets.city.teams"), "4");
  EXPECT_EQ(val(app, "vehicles.fleets.city.seed"), "7");
  EXPECT_EQ(val(app, "vehicles.fleets.city.rally.count"), "4");
  EXPECT_NE(val(app, "vehicles.fleets.city.rally.3.color"), "");
  EXPECT_NE(val(app, "vehicles.fleets.city.status").find("laid out: 64 v opposed"),
            std::string::npos);
  const std::string solo = std::to_string(es[64].id), first = std::to_string(es[0].id);
  EXPECT_EQ(val(app, "entities.ids"), solo);
  EXPECT_EQ(val(app, "entities.count"), "65");
  EXPECT_EQ(st.findDescendant("entities." + first), nullptr);
  EXPECT_NE(val(app, "entities.summary").find("fleet city  64 v (keys: summary)"),
            std::string::npos);
  st("vehicles.watch").value(first); // a camera follows it: it has keys
  art.step_world(6);
  art.drain();
  EXPECT_EQ(val(app, "entities.ids"), first + "," + solo);
  EXPECT_NE(val(app, "entities." + first + ".x"), "");
  EXPECT_EQ(val(app, "entities." + first + ".team"), std::to_string(es[0].rally));
  st("vehicles.watch").value("");
  art.step_world(6);
  art.drain();
  EXPECT_EQ(st.findDescendant("entities." + first), nullptr);

  // A rally point's keys move its team; they drive toward it.
  const double x0 = fleet_mean_x(v->entities());
  st("vehicles.fleets.city.rally.0.x").value("150");
  st("vehicles.fleets.city.rally.0.y").value("-20");
  art.step_world(6);
  art.drain();
  double rx = 0.0, ry = 0.0;
  ASSERT_TRUE(v->rally("city", 0, rx, ry));
  EXPECT_DOUBLE_EQ(rx, 150.0);
  EXPECT_DOUBLE_EQ(ry, -20.0);
  for (const entity_info &e : v->entities())
    if (e.fleet == "city" && e.rally == 0) {
      EXPECT_DOUBLE_EQ(e.goal_x, 150.0);
      EXPECT_DOUBLE_EQ(e.goal_y, -20.0);
    } else if (e.fleet == "city") {
      EXPECT_DOUBLE_EQ(e.goal_x, rally[e.rally][0]);
    }
  EXPECT_TRUE(v->move_rally("city", 1, 1e6, 0.0)); // held to the grid
  ASSERT_TRUE(v->rally("city", 1, rx, ry));
  EXPECT_LE(rx, 201.0);
  EXPECT_FALSE(v->move_rally("city", 9, 0.0, 0.0));
  EXPECT_FALSE(v->move_rally("town", 0, 0.0, 0.0));
  art.step_world(static_cast<int>(std::lround(2.4 / cvc::kSimQuantum))); // 2.4 s
  art.drain();
  EXPECT_GT(fleet_mean_x(v->entities()), x0 + 5.0); // under way, east

  // A restart: every vehicle back at its start, the rally points as laid out.
  ASSERT_TRUE(v->restart_fleet("city", false));
  std::vector<entity_info> again = v->entities();
  ASSERT_EQ(again.size(), 65u);
  for (std::size_t k = 0; k < 64; ++k) {
    EXPECT_NEAR(again[k].x, es[k].x, 1e-3) << k;
    EXPECT_NEAR(again[k].y, es[k].y, 1e-3) << k;
    EXPECT_EQ(again[k].rally, es[k].rally) << k;
  }
  ASSERT_TRUE(v->rally("city", 0, rx, ry));
  EXPECT_DOUBLE_EQ(rx, rally[0][0]);
  // A reseed by its key: another layout.
  st("vehicles.fleets.city.reseed.request").value("1");
  art.step_world(6);
  art.drain();
  EXPECT_EQ(val(app, "vehicles.fleets.city.seed"), "8");
  again = v->entities();
  int moved = 0;
  for (std::size_t k = 0; k < 64; ++k)
    moved += std::hypot(again[k].x - es[k].x, again[k].y - es[k].y) > 1.0 ? 1 : 0;
  EXPECT_GT(moved, 48);
  EXPECT_FALSE(v->restart_fleet("town", false));

  // The same seed in another runtime: the same layout.
  cvc::app app2;
  AppRuntime art2(app2, "");
  art2.load(path.string());
  ASSERT_NE(art2.scene_world(), nullptr);
  art2.drain();
  const std::vector<entity_info> es2 = art2.scene_world()->vehicles()->entities();
  ASSERT_EQ(es2.size(), es.size());
  for (std::size_t k = 0; k < es.size(); ++k) {
    EXPECT_EQ(es2[k].x, es[k].x) << k;
    EXPECT_EQ(es2[k].y, es[k].y) << k;
    EXPECT_EQ(es2[k].goal_x, es[k].goal_x) << k;
  }
}

// keys: all keeps a fleet's per-entity keys; colors: spawn keeps the spawn's colour; a fleet with
// no rally points holds each vehicle at its own goal (it does not wander from it).
TEST(AriadneWorldVehicles, AFleetWithoutRallyPointsHoldsItsOwnGoals) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  std::string doc = kFleetDoc;
  const std::string entry =
      "{ fleet: city, archetype: v, count: 64, spawn: opposed, seed: 7, rally: { count: 4 } }";
  doc.replace(doc.find(entry), entry.size(),
              "{ fleet: city, archetype: v, count: 12, seed: 3, keys: all, colors: spawn }");
  cvc::app app;
  AppRuntime art(app, "");
  art.load(write_doc("fleet_own_goals", doc).string());
  ASSERT_NE(art.scene_world(), nullptr);
  WorldVehicles *v = art.scene_world()->vehicles();
  art.drain();
  const std::vector<entity_info> es = v->entities();
  ASSERT_EQ(es.size(), 13u);
  int spread = 0;
  for (std::size_t k = 0; k < 12; ++k) {
    EXPECT_EQ(es[k].rally, -1);
    EXPECT_EQ(es[k].spec.team, 0);
    EXPECT_EQ(es[k].spec.color, entity_spec{}.color); // the spawn's
    spread += std::abs(es[k].x) > 80.0 ? 0 : 1;       // scattered: anywhere free
  }
  EXPECT_GT(spread, 0);
  double rx = 0.0, ry = 0.0;
  EXPECT_FALSE(v->rally("city", 0, rx, ry));
  art.step_world(6);
  art.drain();
  EXPECT_EQ(val(app, "vehicles.fleets.city.teams"), "0");
  EXPECT_NE(val(app, "entities." + std::to_string(es[0].id) + ".x"), ""); // keys: all
  int listed = 1;
  for (char c : val(app, "entities.ids"))
    listed += c == ',' ? 1 : 0;
  EXPECT_EQ(listed, 13);
  // Held: after a long drive every goal is still the one laid out.
  art.step_world(1200);
  art.drain();
  const std::vector<entity_info> later = v->entities();
  for (std::size_t k = 0; k < 12; ++k) {
    EXPECT_DOUBLE_EQ(later[k].goal_x, es[k].goal_x) << k;
    EXPECT_DOUBLE_EQ(later[k].goal_y, es[k].goal_y) << k;
  }
}

TEST(AriadneWorldVehicles, TheWorldBlockTakesIntentsAndTheExternalController) {
  REQUIRE_YAML();
  LoadResult r = load_intents();
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.scene.vehicle_intents, "/drive");
  ASSERT_EQ(r.scene.vehicles.size(), 1u);
  EXPECT_EQ(r.scene.vehicles[0].controllers.back(), "external");
  const auto error = [](const std::string &intents) {
    cvc::app app;
    register_world_block(app);
    LoadResult bad = load_string(app, "world:\n  vehicles:\n    intents: " + intents +
                                          "\n    archetypes:\n      v: { vehicle: humvee }\n"
                                          "scene:\n  nodes: []\n");
    world_rules x = default_world_rules();
    const Value *block = find_world_block(bad);
    return block ? parse_world_block(*block, x) : std::string("no world: block");
  };
  EXPECT_EQ(error("drive.cars"), "");
  EXPECT_NE(error("\"a..b\"").find("intents: expects a state path"), std::string::npos);
  EXPECT_NE(error("\"two words\"").find("intents: expects a state path"), std::string::npos);
  entity_intent::mode m = entity_intent::mode::off;
  EXPECT_TRUE(parse_intent_mode("blend", m));
  EXPECT_EQ(m, entity_intent::mode::blend);
  EXPECT_FALSE(parse_intent_mode("steer", m));
  EXPECT_STREQ(intent_mode_name(entity_intent::mode::replace), "replace");
}

TEST(AriadneWorldVehicles, AnIntentDrivesAnExternalEntity) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_intents();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  WorldVehicles &v = *w->vehicles();
  EXPECT_EQ(v.intents_key(), "drive");
  w->sync(&warnings);
  const std::uint64_t id = spawn_at(v, "external", -40.0, 0.0);
  SceneClockDriver clock(app, resolved_clock(r.scene));

  // No intent: it holds still where it was put.
  run(app, *w, clock, 120);
  const entity_info start = entity(v, id);
  EXPECT_LT(planar(start, -40.0, 0.0), 3.0); // the chassis centre, ahead of the point
  EXPECT_LT(std::fabs(start.speed), 0.05);
  EXPECT_EQ(val(app, "entities.1.command.by"), "hold");

  // The setter: straight ahead at 2 m/s^2.
  entity_intent go;
  go.command = entity_intent::mode::replace;
  go.accel = 2.0;
  EXPECT_TRUE(v.set_intent(id, go));
  EXPECT_EQ(v.intent(id), go);
  EXPECT_FALSE(v.set_intent(99, go)) << "not an entity";
  entity_intent bad = go;
  bad.weight = 2.0;
  EXPECT_FALSE(v.set_intent(id, bad));
  run(app, *w, clock, 180); // 3 s
  const entity_info moved = entity(v, id);
  EXPECT_GT(moved.x - start.x, 4.0) << moved.x;
  EXPECT_LT(std::fabs(moved.y - start.y), 0.5) << moved.y;
  EXPECT_GT(moved.speed, 2.0);
  EXPECT_EQ(moved.intent, go);
  EXPECT_NEAR(moved.command_accel, 2.0, 1e-3);
  EXPECT_EQ(val(app, "entities.1.command.by"), "intent");

  // Its keys: turn left. A change of a key replaces the setter's intent.
  cvc::state &st = cvc::state::instance(app);
  st("entities.1.intent.steer").value("0.3");
  st("entities.1.intent.accel").value("0.5");
  run(app, *w, clock, 120);
  const entity_info turned = entity(v, id);
  EXPECT_EQ(v.intent(id).command, entity_intent::mode::replace) << "a steer with no mode replaces";
  EXPECT_NEAR(v.intent(id).steer, 0.3, 1e-12);
  EXPECT_GT(turned.yaw, moved.yaw + 0.3) << turned.yaw;
  EXPECT_NEAR(turned.command_steer, 0.3, 1e-6);

  // Off: it brakes to a stop and holds there.
  st("entities.1.intent.mode").value("off");
  run(app, *w, clock, 240);
  const entity_info held = entity(v, id);
  EXPECT_LT(std::fabs(held.speed), 0.05) << held.speed;
  run(app, *w, clock, 60);
  EXPECT_LT(planar(entity(v, id), held.x, held.y), 0.05);

  // The intents table, by name: the entity's own keys have not changed, so the table's does.
  const std::uint64_t named = spawn_at(v, "external", 30.0, -40.0, "lead");
  run(app, *w, clock, 30);
  st("drive.lead.intent.accel").value("1.5");
  run(app, *w, clock, 120);
  EXPECT_DOUBLE_EQ(v.intent(named).accel, 1.5);
  EXPECT_GT(entity(v, named).speed, 1.0);
  // Removed: its intent goes with it.
  EXPECT_TRUE(v.despawn(named));
  EXPECT_FALSE(v.set_intent(named, go));
  EXPECT_EQ(v.intent(named), entity_intent{});
}

TEST(AriadneWorldVehicles, ABlendIsTheWeightedCommandAndAForcePushesTheDrive) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_intents();
  ASSERT_TRUE(r.ok) << r.error;
  // Three worlds alike, but for the intent on their one nav@1 entity: none, a replace, a blend.
  struct one {
    cvc::app app;
    std::unique_ptr<SceneWorld> w;
    std::unique_ptr<SceneClockDriver> clock;
    std::uint64_t id = 0;
  };
  const auto make = [&](one &o) {
    std::vector<std::string> warnings;
    o.w = SceneWorld::from_scene(o.app, r.scene, "", "physics", &warnings);
    ASSERT_NE(o.w, nullptr);
    o.w->sync(&warnings);
    o.id = spawn_at(*o.w->vehicles(), "nav@1", -40.0, -10.0);
    o.clock = std::make_unique<SceneClockDriver>(o.app, resolved_clock(r.scene));
  };
  one plain, mine, half;
  make(plain);
  make(mine);
  make(half);
  entity_intent in;
  in.command = entity_intent::mode::replace;
  in.steer = 0.2;
  in.accel = 1.0;
  ASSERT_TRUE(mine.w->vehicles()->set_intent(mine.id, in));
  in.command = entity_intent::mode::blend;
  in.weight = 0.25;
  ASSERT_TRUE(half.w->vehicles()->set_intent(half.id, in));
  for (one *o : {&plain, &mine, &half})
    run(o->app, *o->w, *o->clock, 2); // the clock anchors, then the first navigator tick
  const entity_info a = entity(*plain.w->vehicles(), plain.id);
  const entity_info b = entity(*mine.w->vehicles(), mine.id);
  const entity_info c = entity(*half.w->vehicles(), half.id);
  EXPECT_NEAR(b.command_steer, 0.2, 1e-6);
  EXPECT_NEAR(c.command_steer, 0.75 * a.command_steer + 0.25 * 0.2, 1e-5)
      << a.command_steer << " " << c.command_steer;
  EXPECT_NEAR(c.command_accel, 0.75 * a.command_accel + 0.25 * 1.0, 1e-4)
      << a.command_accel << " " << c.command_accel;
  EXPECT_EQ(val(half.app, "entities.1.command.by"), "blend");
  EXPECT_EQ(val(plain.app, "entities.1.command.by"), "nav@1");

  // A force north on the drive: the same nav@1 run ends north of the plain one, and the plain
  // drive is untouched by a force of zero.
  one pushed, zero;
  make(pushed);
  make(zero);
  entity_intent north;
  north.force_y = 3.0;
  ASSERT_TRUE(pushed.w->vehicles()->set_intent(pushed.id, north));
  ASSERT_TRUE(zero.w->vehicles()->set_intent(zero.id, entity_intent{}));
  run(plain.app, *plain.w, *plain.clock, 238);
  run(pushed.app, *pushed.w, *pushed.clock, 240);
  run(zero.app, *zero.w, *zero.clock, 240);
  const entity_info p = entity(*plain.w->vehicles(), plain.id);
  const entity_info q = entity(*pushed.w->vehicles(), pushed.id);
  EXPECT_GT(q.y, p.y + 0.5) << p.x << "," << p.y << " vs " << q.x << "," << q.y;
  EXPECT_EQ(zero.w->world().state_hash(), plain.w->world().state_hash());
}

// Intents may be set from another thread while the world steps: a writer keeps replacing a's
// intent (and blending b's) as the frames run, and the last one set wins. The frames start once
// the writer has landed its first intent, and go on past 120 until another lands while the world
// steps. A thread may start late, or be descheduled: on a busy machine all 120 frames (a few tens
// of milliseconds) can pass before it runs. Then none of its intents reach a, which sits still,
// and only the last 6 frames move it (0.108 m/s, what Windows CI read on #1318).
TEST(AriadneWorldVehicles, IntentsMaybeSetFromAnotherThread) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_intents();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  const std::uint64_t a = spawn_at(v, "external", -40.0, 20.0);
  const std::uint64_t b = spawn_at(v, "nav@1", 20.0, 20.0);
  SceneClockDriver clock(app, resolved_clock(r.scene));
  std::atomic<bool> done{false};
  std::atomic<int> landed{0}; // the writer's intents a took
  std::thread writer([&] {
    for (int i = 0; !done.load(); ++i) {
      entity_intent in;
      in.command = entity_intent::mode::replace;
      in.accel = 1.0 + (i % 3);
      in.force_x = (i % 5) * 0.5;
      if (v.set_intent(a, in))
        landed.fetch_add(1);
      in.command = entity_intent::mode::blend;
      v.set_intent(b, in);
      (void)v.intent(a);
    }
  });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (landed.load() == 0 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::yield();
  const int before = landed.load();
  run(app, *w, clock, 120);
  // The writer may be descheduled for all 120 frames: step on, a frame at a time, until one of its
  // intents lands while the world steps (10 s at most).
  const auto more = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (landed.load() == before && std::chrono::steady_clock::now() < more)
    run(app, *w, clock, 1);
  const int during = landed.load() - before;
  done = true;
  writer.join();
  const std::vector<entity_info> all = v.entities();
  const bool there =
      std::any_of(all.begin(), all.end(), [a](const entity_info &e) { return e.id == a; });
  const double moving = there ? entity(v, a).speed : -1.0;
  ASSERT_GT(before, 0) << "the writer landed no intent in 10 s";
  EXPECT_GT(during, 0) << "no intent landed while the frames ran";
  entity_intent last;
  last.command = entity_intent::mode::replace;
  last.accel = 2.5;
  ASSERT_TRUE(v.set_intent(a, last));
  run(app, *w, clock, 6);
  EXPECT_EQ(entity(v, a).intent, last);
  EXPECT_GT(entity(v, a).speed, 0.5)
      << "after the 120 threaded frames a's speed was " << moving << " m/s, " << during
      << " intents landed during them, and a " << (there ? "existed" : "was missing");
}

TEST(AriadneWorldVehicles, AServerDocumentSteersThroughItsPublishedNamespace) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_intents();
  ASSERT_TRUE(r.ok) << r.error;
  // The relay's document publishes `drive`; the world here reads it as its intents table (a
  // client's world reads the same keys once the relay replicates them).
  cvc::app app;
  const fs::path doc = write_doc("drive_server", R"(meta: { name: driver, min_libcvc: 3.7.5 }
server:
  publish: [drive]
on_tick: |
  (if (> (int (state-get "sim.tick")) 240)
    (state-set "drive.lead.intent.accel" "2")
    (state-set "drive.lead.intent.accel" "0"))
)");
  ServerDocument server(app, "garage");
  ASSERT_EQ(server.read(doc.string()), "");
  for (const std::string &warning : server.load(doc.string()))
    ADD_FAILURE() << "warning: " << warning;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  const std::uint64_t id = spawn_at(v, "external", -40.0, -40.0, "lead");
  SceneClockDriver clock(app, resolved_clock(r.scene));
  std::uint64_t tick = 0;
  const auto steps = [&](int frames) {
    for (int i = 0; i < frames; ++i) {
      tick += 2; // the relay's 120 Hz under a 60 Hz frame
      server.step(tick, tick / 120.0);
      clock.drive(1.0 / 60.0);
      w->follow(app.world_clock());
    }
  };
  steps(60);
  EXPECT_EQ(val(app, "drive.lead.intent.accel"), "0");
  EXPECT_LT(std::fabs(entity(v, id).speed), 0.1);
  steps(180);
  EXPECT_EQ(val(app, "drive.lead.intent.accel"), "2");
  EXPECT_DOUBLE_EQ(v.intent(id).accel, 2.0);
  EXPECT_GT(entity(v, id).speed, 2.0);
}

// ---------------------------------------------------------------------------------------------
// Entity poses (the `pose` controller): an entity an outside simulation places, by its keys, a
// packed value, a table of poses, a C++ (or Python) setter and a server document's published
// namespace. No plant integrates it; its chassis is a kinematic box at each pose.

namespace {

const char *kPoses = R"(
world:
  vehicles:
    poses: /fleet
    archetypes:
      humvee:
        vehicle: humvee
        controllers: [pose, nav@1]
      racer:
        plant: kinematic
        vehicle: roboracer
        controllers: [nav@1]
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [160, 160, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - node: crate
      source: { box: { size: [1, 1, 1] } }
      transform: { position: [40, 30, 0.5] }
      body: { mass: 50 }
)";

LoadResult load_poses() {
  cvc::app app;
  register_world_block(app);
  LoadResult r = load_string(app, kPoses);
  if (r.ok)
    if (const Value *world = find_world_block(r)) {
      world_rules rules = default_world_rules();
      EXPECT_EQ(parse_world_block(*world, rules), "");
      r.scene.vehicles = rules.vehicles;
      r.scene.vehicle_poses = rules.vehicle_poses;
    }
  return r;
}

// A `pose` humvee, with no pose yet (or at a spawn's place when `at` is given).
std::uint64_t spawn_posed(WorldVehicles &v, const std::string &name = std::string()) {
  entity_spec s;
  s.archetype = "humvee";
  s.controller = "pose";
  s.name = name;
  std::string why;
  const std::uint64_t id = v.spawn(s, &why);
  EXPECT_NE(id, 0u) << why;
  return id;
}

// The scripted track: where entity `id`'s keys put it at frame k (exactly representable numbers).
entity_pose track(int k) {
  entity_pose p;
  p.x = -20.0 + 0.25 * k;
  p.y = 10.0 - 0.125 * k;
  p.heading = -0.5 + 0.0078125 * k;
  p.speed = 15.0;
  p.steer = 0.0625 * ((k / 30) % 2 ? 1 : -1);
  return p;
}

void write_track(cvc::state &st, std::uint64_t id, const entity_pose &p) {
  const std::string at = "entities." + std::to_string(id) + ".pose.";
  char buf[32];
  const auto put = [&](const char *leaf, double v) {
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    st(at + leaf).value(std::string(buf));
  };
  put("x", p.x);
  put("y", p.y);
  put("heading", p.heading);
  put("speed", p.speed);
  put("steer", p.steer);
}

// The chassis's height a kinematic box stands at on the square's top (z = 0): its wheels' radius
// and half its height.
double stand_height() {
  const cvc::physics::vehicle_desc d = cvc::physics::vehicle_desc::humvee();
  return d.half_extents.z + d.axles.front().wheel_radius;
}

} // namespace

TEST(AriadneWorldVehicles, TheWorldBlockTakesPosesAndThePoseController) {
  REQUIRE_YAML();
  LoadResult r = load_poses();
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.scene.vehicle_poses, "/fleet");
  ASSERT_EQ(r.scene.vehicles.size(), 2u);
  EXPECT_EQ(r.scene.vehicles[0].controllers.front(), "pose");
  EXPECT_TRUE(vehicle_controller_known("pose"));
  const auto error = [](const std::string &poses) {
    cvc::app app;
    register_world_block(app);
    LoadResult bad = load_string(app, "world:\n  vehicles:\n    poses: " + poses +
                                          "\n    archetypes:\n      v: { vehicle: humvee }\n"
                                          "scene:\n  nodes: []\n");
    world_rules x = default_world_rules();
    const Value *block = find_world_block(bad);
    return block ? parse_world_block(*block, x) : std::string("no world: block");
  };
  EXPECT_EQ(error("fleet.cars"), "");
  EXPECT_NE(error("\"a..b\"").find("poses: expects a state path"), std::string::npos);

  // The packed records: written and read back exactly, a bad record skipped.
  entity_pose a;
  a.x = 1.0 / 3.0;
  a.y = -2.5e7;
  a.heading = 3.0;
  a.speed = 7.25;
  a.time = 0.06;
  entity_pose b;
  b.x = 4.0;
  b.y = 5.0;
  b.z = 1.5;
  b.pitch = 0.1;
  b.roll = -0.2;
  b.steer = 0.3;
  b.rear_axle = true;
  const std::string text = pack_poses({{"1", a}, {"lead", b}});
  std::vector<std::pair<std::string, entity_pose>> back;
  EXPECT_TRUE(parse_poses(text, back));
  ASSERT_EQ(back.size(), 2u);
  EXPECT_EQ(back[0].first, "1");
  EXPECT_EQ(back[0].second.x, a.x);
  EXPECT_EQ(back[0].second.y, a.y);
  EXPECT_TRUE(std::isnan(back[0].second.z));
  EXPECT_EQ(back[0].second.time, 0.06);
  EXPECT_FALSE(back[0].second.rear_axle);
  EXPECT_EQ(back[1].first, "lead");
  EXPECT_EQ(back[1].second.z, 1.5);
  EXPECT_EQ(back[1].second.roll, -0.2);
  EXPECT_TRUE(back[1].second.rear_axle);
  EXPECT_TRUE(std::isnan(back[1].second.time));
  EXPECT_FALSE(
      parse_poses(text + "\nbad 1 2\n2 1 2 nan 0 0 0 0 0 nan;3 inf 0 0 0 0 0 0 0 nan", back));
  ASSERT_EQ(back.size(), 3u) << "the good ones, the bad two left";
  EXPECT_EQ(back[2].first, "2");
}

// Poses on the keys, each frame, with no plant integrating the entity: it is exactly where they
// say, standing on the ground, its chassis a kinematic body there; its wheels steer and roll.
TEST(AriadneWorldVehicles, APoseEntityFollowsAScriptedTrackExactly) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_poses();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  EXPECT_EQ(v.poses_key(), "fleet");
  const std::uint64_t id = spawn_posed(v, "lead");
  SceneClockDriver clock(app, resolved_clock(r.scene));
  const std::size_t bodies = w->world().body_count();

  // No pose yet: it waits, out of the world.
  run(app, *w, clock, 10);
  EXPECT_FALSE(entity(v, id).placed);
  EXPECT_FALSE(entity(v, id).posed);
  EXPECT_EQ(val(app, "entities.1.controller"), "pose");

  cvc::state &st = cvc::state::instance(app);
  const double h = stand_height();
  const std::uint64_t t0 = w->world().tick(); // its chassis goes in at the next step
  for (int k = 0; k < 240; ++k) {
    write_track(st, id, track(k));
    run(app, *w, clock, 1);
    const entity_info e = entity(v, id);
    const entity_pose want = track(k);
    ASSERT_TRUE(e.placed && e.posed) << "frame " << k;
    // Exactly the pose, bit for bit: x, y, the heading, the speed and the steer.
    EXPECT_EQ(e.x, want.x) << "frame " << k;
    EXPECT_EQ(e.y, want.y) << "frame " << k;
    EXPECT_EQ(e.yaw, want.heading) << "frame " << k;
    EXPECT_EQ(e.speed, want.speed);
    EXPECT_EQ(e.steer, want.steer);
    EXPECT_EQ(e.z, h) << "standing on the square's top";
    // Its chassis is a body there (as the physics world keeps it, in floats).
    const cvc::physics::body_state b = w->world().state(e.chassis);
    EXPECT_NEAR(b.position.x, want.x, 1e-4);
    EXPECT_NEAR(b.position.y, want.y, 1e-4);
    EXPECT_NEAR(b.position.z, h, 1e-4);
  }
  EXPECT_EQ(w->world().body_count(), bodies + 1) << "one body: its chassis";
  EXPECT_EQ(val(app, "entities.1.command.by"), "pose");
  // No plant has it, and none was made for it.
  const std::vector<entity_plant> plants = v.plants();
  ASSERT_EQ(plants.size(), 1u);
  EXPECT_EQ(plants[0].model, "pose");
  // Its wheels: the front ones steered by the pose's steer, every one rolled at its speed.
  const std::vector<entity_wheel> wheels = v.wheels(id);
  ASSERT_EQ(wheels.size(), 4u);
  EXPECT_DOUBLE_EQ(wheels[0].steer, track(239).steer);
  EXPECT_DOUBLE_EQ(wheels[3].steer, 0.0);
  const double r0 = cvc::physics::vehicle_desc::humvee().axles.front().wheel_radius;
  // 15 m/s, every step after the one that put it in
  const double rolled =
      15.0 * static_cast<double>(w->world().tick() - t0 - 1) * w->world().config().fixed_dt;
  EXPECT_NEAR(std::cos(wheels[0].spin), std::cos(rolled / r0), 1e-6);
  EXPECT_NEAR(std::sin(wheels[0].spin), std::sin(rolled / r0), 1e-6);
  EXPECT_NEAR(wheels[0].z, -h + r0, 1e-9) << "on the ground under it";

  // A matrix: its translation and rotation (here pitched nose down and rolled).
  const double p = 0.1, rl = -0.05, hd = 0.75;
  const double cp = std::cos(p), sp = std::sin(p), cr = std::cos(rl), sr = std::sin(rl),
               ch = std::cos(hd), sh = std::sin(hd);
  // R = Rz(hd) Ry(p) Rx(rl), row-major, with (5, -6, 1.75) its translation.
  const double m[16] = {ch * cp,
                        ch * sp * sr - sh * cr,
                        ch * sp * cr + sh * sr,
                        5.0,
                        sh * cp,
                        sh * sp * sr + ch * cr,
                        sh * sp * cr - ch * sr,
                        -6.0,
                        -sp,
                        cp * sr,
                        cp * cr,
                        1.75,
                        0.0,
                        0.0,
                        0.0,
                        1.0};
  std::string text;
  for (int i = 0; i < 16; ++i) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%s%.17g", i ? "," : "", m[i]);
    text += buf;
  }
  st("entities.1.pose.matrix").value(text);
  run(app, *w, clock, 30);
  const entity_info e = entity(v, id);
  EXPECT_NEAR(e.x, 5.0, 1e-12);
  EXPECT_NEAR(e.y, -6.0, 1e-12);
  EXPECT_NEAR(e.z, 1.75, 1e-12);
  EXPECT_NEAR(e.yaw, hd, 1e-12);
  EXPECT_NEAR(e.pose.pitch, p, 1e-12);
  EXPECT_NEAR(e.pose.roll, rl, 1e-12);
  const cvc::physics::quat q = w->world().state(e.chassis).rotation;
  // Its nose (the body's +x) points along the matrix's first column.
  const double nx = 1.0 - 2.0 * (q.y * q.y + q.z * q.z), ny = 2.0 * (q.x * q.y + q.w * q.z),
               nz = 2.0 * (q.x * q.z - q.w * q.y);
  EXPECT_NEAR(nx, m[0], 1e-5);
  EXPECT_NEAR(ny, m[4], 1e-5);
  EXPECT_NEAR(nz, m[8], 1e-5);

  // A rear axle's point: its centre is ahead of it along the heading.
  st("entities.1.pose.matrix").value("");
  st("entities.1.pose.x").value("0");
  st("entities.1.pose.y").value("0");
  st("entities.1.pose.heading").value("1.5707963267948966");
  st("entities.1.pose.ref").value("rear");
  run(app, *w, clock, 30);
  const entity_info rear = entity(v, id);
  double axle = 0.0;
  for (const cvc::physics::axle_desc &a : cvc::physics::vehicle_desc::humvee().axles)
    axle = std::min(axle, a.x);
  EXPECT_NEAR(rear.x, 0.0, 1e-9);
  EXPECT_NEAR(rear.y, -axle, 1e-9);
  entity_pose given;
  ASSERT_TRUE(v.pose(id, given));
  EXPECT_TRUE(given.rear_axle) << "pose() is what was given";
  EXPECT_EQ(given.x, 0.0);

  // The state after the track, pinned on every platform (a cross-platform-deterministic Jolt, as
  // physics_world_test's golden scene): no plant, so the poses and the crate decide it.
  const std::uint64_t hash = w->world().state_hash();
  std::printf("[pose-track] state hash after %llu ticks: %016llx\n",
              static_cast<unsigned long long>(w->world().tick()),
              static_cast<unsigned long long>(hash));
  EXPECT_EQ(w->world().tick(), 618u); // 1/120 s steps (cvc::kSimQuantum)
  if (cvc::physics::backend_capabilities(cvc::physics::backend::jolt) &
      cvc::physics::cap_determinism_cross_platform)
    EXPECT_EQ(hash, 0xe6d183c87bbc907eull) << std::hex << "state hash " << hash;
}

// Timed poses (set_pose's): drawn between the last two, a span behind; held when the next is late;
// a jump, a time gone back or the first one taken at once; the same time again changes nothing.
TEST(AriadneWorldVehicles, TimedPosesAreDrawnBetweenTheLastTwo) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_poses();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  const std::uint64_t id = spawn_posed(v);
  const double dt = w->world().config().fixed_dt;
  const auto step = [&](int n) {
    for (int i = 0; i < n; ++i) {
      v.before_step();
      w->world().step();
      v.after_step();
    }
  };
  const auto at = [&](double x, double t, double heading = 0.0) {
    entity_pose p;
    p.x = x;
    p.y = 2.0;
    p.heading = heading;
    p.speed = 10.0;
    p.time = t;
    return p;
  };
  EXPECT_FALSE(v.set_pose(id + 1, at(0.0, 0.0))) << "not an entity";
  entity_pose bad = at(0.0, 0.0);
  bad.y = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(v.set_pose(id, bad));
  ASSERT_TRUE(v.set_pose(id, at(0.0, 0.0, 3.0)));
  step(1);
  EXPECT_EQ(entity(v, id).x, 0.0) << "the first: there at once";
  // The next, six physics steps on (span): over the next six steps of world time it moves there,
  // the heading the short way round (3 rad to -3 rad is 0.28 rad, not 6).
  const double span = 6 * dt;
  ASSERT_TRUE(v.set_pose(id, at(0.6, span, -3.0)));
  const double arc = std::remainder(-3.0 - 3.0, 2.0 * 3.14159265358979323846);
  for (int k = 1; k <= 6; ++k) {
    step(1);
    const entity_info e = entity(v, id);
    const double s = std::min(1.0, k * dt / span);
    EXPECT_NEAR(e.x, 0.6 * s, 1e-12) << "step " << k;
    EXPECT_NEAR(std::remainder(e.yaw - (3.0 + s * arc), 2.0 * 3.14159265358979323846), 0.0, 1e-12);
    EXPECT_NEAR(e.pose.time, span * s, 1e-12);
  }
  EXPECT_EQ(entity(v, id).x, 0.6) << "there exactly, at the span's end";
  step(5);
  EXPECT_EQ(entity(v, id).x, 0.6) << "the next is late: it holds";
  // The same time again: the same pose, nothing changes.
  ASSERT_TRUE(v.set_pose(id, at(5.0, span)));
  step(2);
  EXPECT_EQ(entity(v, id).x, 0.6);
  // A jump (farther than pose_jump): at once, its chassis teleported.
  ASSERT_TRUE(v.set_pose(id, at(50.0, 2 * span)));
  step(1);
  EXPECT_EQ(entity(v, id).x, 50.0);
  EXPECT_NEAR(w->world().state(entity(v, id).chassis).position.x, 50.0, 1e-4);
  // A time gone back (the outside simulation restarted): at once.
  ASSERT_TRUE(v.set_pose(id, at(49.0, 0.0)));
  step(1);
  EXPECT_EQ(entity(v, id).x, 49.0);
  // An untimed one: at the end of the next step.
  entity_pose still = at(48.0, std::numeric_limits<double>::quiet_NaN());
  ASSERT_TRUE(v.set_pose(id, still));
  step(1);
  EXPECT_EQ(entity(v, id).x, 48.0);
  EXPECT_THROW(v.set_pose_jump(0.0), std::invalid_argument);
  v.set_pose_jump(100.0);
  EXPECT_EQ(v.pose_jump(), 100.0);
}

// Many at once: set_poses, the packed `entities.poses` value, and set_poses from another thread
// while the world steps.
TEST(AriadneWorldVehicles, ManyPosesAtOnceAndFromAnotherThread) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_poses();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  std::vector<std::uint64_t> ids;
  for (int i = 0; i < 8; ++i)
    ids.push_back(spawn_posed(v));
  SceneClockDriver clock(app, resolved_clock(r.scene));
  std::vector<entity_pose_update> batch;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    entity_pose p;
    p.x = 8.0 * static_cast<double>(i) - 30.0;
    p.y = -10.0;
    batch.push_back({ids[i], p});
  }
  batch.push_back({999, entity_pose{}}); // not an entity: left
  EXPECT_EQ(v.set_poses(batch), ids.size());
  run(app, *w, clock, 2);
  for (std::size_t i = 0; i < ids.size(); ++i)
    EXPECT_EQ(entity(v, ids[i]).x, 8.0 * static_cast<double>(i) - 30.0);

  // The packed value: one write for all of them.
  std::vector<std::pair<std::string, entity_pose>> records;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    entity_pose p;
    p.x = 1.5 * static_cast<double>(i);
    p.y = 20.0;
    p.heading = 0.25;
    records.emplace_back(std::to_string(ids[i]), p);
  }
  cvc::state::instance(app)("entities.poses").value(pack_poses(records));
  run(app, *w, clock, 2);
  for (std::size_t i = 0; i < ids.size(); ++i) {
    const entity_info e = entity(v, ids[i]);
    EXPECT_EQ(e.x, 1.5 * static_cast<double>(i));
    EXPECT_EQ(e.y, 20.0);
    EXPECT_EQ(e.yaw, 0.25);
  }
  // Unchanged, the packed value does not undo a setter's pose.
  ASSERT_TRUE(v.set_pose(ids[0], batch[0].pose));
  run(app, *w, clock, 2);
  EXPECT_EQ(entity(v, ids[0]).x, -30.0);

  // From another thread, while frames run: no tearing, and the last one holds.
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (int k = 0; k <= 400; ++k) {
      std::vector<entity_pose_update> b;
      for (std::size_t i = 0; i < ids.size(); ++i) {
        entity_pose p;
        p.x = static_cast<double>(k) * 0.05;
        p.y = static_cast<double>(i) * 6.0;
        b.push_back({ids[i], p});
      }
      v.set_poses(b);
      std::this_thread::yield();
    }
    done = true;
  });
  while (!done)
    run(app, *w, clock, 1);
  writer.join();
  run(app, *w, clock, 2);
  for (std::size_t i = 0; i < ids.size(); ++i) {
    EXPECT_EQ(entity(v, ids[i]).x, 20.0);
    EXPECT_EQ(entity(v, ids[i]).y, static_cast<double>(i) * 6.0);
  }
}

// The poses table (world: vehicles: poses:): its entries that name an archetype spawn their
// entities, its packed value places them by entry, an entry removed removes its entity; a pose
// entity is a body the others meet: it pushes a crate, and a world restart puts it back where its
// last pose is.
TEST(AriadneWorldVehicles, APosesTableSpawnsAndPlacesItsEntities) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_poses();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  SceneClockDriver clock(app, resolved_clock(r.scene));
  cvc::state &st = cvc::state::instance(app);
  st("fleet.alpha.archetype").value("humvee");
  st("fleet.alpha.name").value("Alpha 1");
  st("fleet.alpha.team").value("2");
  st("fleet.alpha.color").value("0.1,0.2,0.3");
  st("fleet.bravo.archetype").value("humvee");
  st("fleet.nope.archetype").value("racer"); // no pose controller: refused, once
  run(app, *w, clock, 2);
  ASSERT_EQ(v.size(), 2u) << v.status();
  EXPECT_NE(v.status().find("'nope'"), std::string::npos) << v.status();
  std::vector<entity_info> es = v.entities();
  EXPECT_EQ(es[0].spec.name, "Alpha 1");
  EXPECT_EQ(es[0].spec.team, 2);
  EXPECT_FLOAT_EQ(es[0].spec.color[2], 0.3f);
  EXPECT_EQ(es[0].spec.controller, "pose");
  EXPECT_EQ(es[1].spec.name, "bravo");
  EXPECT_FALSE(es[0].placed) << "no pose yet";

  // The packed value, by entry; then one's own keys, by entry too.
  entity_pose a, b;
  a.x = 36.0;
  a.y = 30.0;
  a.speed = 4.0;
  b.x = -40.0;
  b.y = -40.0;
  b.heading = 1.0;
  st("fleet.poses").value(pack_poses({{"alpha", a}, {"bravo", b}}));
  run(app, *w, clock, 2);
  es = v.entities();
  EXPECT_EQ(es[0].x, 36.0);
  EXPECT_EQ(es[1].yaw, 1.0);
  st("fleet.bravo.pose.x").value("-41");
  st("fleet.bravo.pose.y").value("-40");
  run(app, *w, clock, 2);
  EXPECT_EQ(entity(v, es[1].id).x, -41.0);

  // Alpha drives into the crate (at x 40) at 4 m/s, and pushes it.
  const std::vector<cvc::physics::body_id> crate = w->bodies_of("crate");
  ASSERT_EQ(crate.size(), 1u);
  const double crate_x = w->world().state(crate[0]).position.x;
  for (int k = 1; k <= 120; ++k) {
    a.x = 36.0 + 4.0 * k / 60.0;
    st("fleet.poses").value(pack_poses({{"alpha", a}, {"bravo", b}}));
    run(app, *w, clock, 1);
  }
  EXPECT_GT(w->world().state(crate[0]).position.x, crate_x + 2.0) << "the crate was pushed";

  // A restart: the world made afresh, each pose entity back in at its last pose.
  const std::uint64_t alpha = es[0].id;
  w->restart(&warnings);
  run(app, *w, clock, 2);
  EXPECT_TRUE(entity(v, alpha).placed);
  EXPECT_EQ(entity(v, alpha).x, a.x);

  // Its entry gone: it goes; a new archetype there: another in its place.
  st("fleet").removeChild("bravo");
  run(app, *w, clock, 2);
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v.entities()[0].spec.name, "Alpha 1");
  st("fleet.alpha.name").value("Alpha One");
  run(app, *w, clock, 2);
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v.entities()[0].spec.name, "Alpha One");
  EXPECT_NE(v.entities()[0].id, alpha);
}

// The pose document arigl runs offscreen (ctest arigl_pose_entities,
// data/ariadne_pose_entities.ari), run headless: its three pose entities (two from start:, one from
// its table) in the world with the ground and nothing else, and at tick 60 the state hash arigl
// prints, pinned in both.
TEST(AriadneWorldVehicles, APoseDocumentsHashIsPinned) {
  REQUIRE_YAML();
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "ariadne_pose_entities.ari";
  if (!fs::exists(path))
    GTEST_SKIP() << "no source tree beside this test: " << path.string();
  cvc::app app;
  AppRuntime art(app, "");
  art.load(path.string());
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  art.drain();
  SceneWorld *w = art.scene_world();
  ASSERT_NE(w, nullptr);
  ASSERT_NE(w->vehicles(), nullptr);
  cvc::world_clock::config cc;
  cc.fixed_dt = w->world().config().fixed_dt;
  cvc::world_clock clock(cc);
  w->follow(clock); // anchors, and spawns the table's entry
  while (w->world().tick() < 60) {
    clock.advance_ticks(1);
    w->follow(clock);
  }
  ASSERT_EQ(w->world().tick(), 60u);
  WorldVehicles &v = *w->vehicles();
  ASSERT_EQ(v.size(), 3u) << v.status();
  for (const entity_info &e : v.entities())
    EXPECT_TRUE(e.placed && e.posed) << e.spec.name;
  const entity_info third = v.entities()[2];
  EXPECT_EQ(third.spec.name, "third");
  EXPECT_EQ(third.x, 12.0);
  EXPECT_EQ(third.pose.pitch, 0.0625);
  EXPECT_EQ(w->world().body_count(), 4u) << "the ground and three chassis";
  EXPECT_EQ(w->world().vehicle_count(), 0u);
  EXPECT_EQ(val(app, "physics.stats.tick"), "60");
  const std::uint64_t hash = w->world().state_hash();
  char hex[24];
  std::snprintf(hex, sizeof(hex), "%016llx", static_cast<unsigned long long>(hash));
  EXPECT_EQ(val(app, "physics.stats.hash"), hex) << "published while physics.hash is 1";
  std::printf("[pose-document] state hash at tick 60: %s\n", hex);
  if (cvc::physics::backend_capabilities(cvc::physics::backend::jolt) &
      cvc::physics::cap_determinism_cross_platform)
    EXPECT_EQ(hash, 0xcbe4b2596cd56330ull) << "state hash " << hex;
}

// A server document places entities through its published namespace: its program writes the
// table (an entry and its pose each tick); the world here reads it, as a relay client's does once
// the relay replicates it.
TEST(AriadneWorldVehicles, AServerDocumentPlacesThroughItsPublishedNamespace) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_poses();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  const fs::path doc = write_doc("pose_server", R"(meta: { name: placer, min_libcvc: 3.7.5 }
server:
  publish: [fleet]
init: |
  (state-set "fleet.lead.archetype" "humvee")
on_tick: |
  (state-set "fleet.lead.pose.x" (* 0.05 (int (state-get "sim.tick"))))
  (state-set "fleet.lead.pose.y" "-12")
  (state-set "fleet.lead.pose.t" (/ (int (state-get "sim.tick")) 120.0))
)");
  ServerDocument server(app, "garage");
  ASSERT_EQ(server.read(doc.string()), "");
  for (const std::string &warning : server.load(doc.string()))
    ADD_FAILURE() << "warning: " << warning;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  SceneClockDriver clock(app, resolved_clock(r.scene));
  std::uint64_t tick = 0;
  const auto steps = [&](int frames) {
    for (int i = 0; i < frames; ++i) {
      tick += 2; // the relay's 120 Hz under a 60 Hz frame
      server.step(tick, tick / 120.0);
      clock.drive(1.0 / 60.0);
      w->follow(app.world_clock());
    }
  };
  steps(120);
  ASSERT_EQ(v.size(), 1u) << v.status();
  const entity_info e = v.entities()[0];
  EXPECT_EQ(e.spec.name, "lead");
  EXPECT_TRUE(e.placed);
  // Its timed poses, drawn a tick behind: within one step's travel of the server's last.
  EXPECT_NEAR(e.x, 0.05 * static_cast<double>(tick), 0.05 * 2.0 + 1e-9);
  EXPECT_EQ(e.y, -12.0);
}

// ---------------------------------------------------------------------------------------------
// Every plant in a scene world (ARIADNE_WORLDS.md section 14, L2b): the kinematic bicycle and the
// dynamic models beside Jolt, each stepped on the world's clock, against its buildings.

namespace {

const char *kPlants = R"(
world:
  vehicles:
    archetypes:
      jolt: { plant: jolt, vehicle: humvee, controllers: [nav@1, external] }
      bike: { plant: dynamic, vehicle: humvee, controllers: [nav@1, external] }
      four: { plant: "dynamic4w:humvee", controllers: [nav@1, external] }
      walker: { plant: kinematic, vehicle: humvee, controllers: [nav@1, external] }
scene:
  physics: {}
  world: { source: yard, cell: 2 }
  nodes:
    - node: yard
      world: yard
      source:
        heightfield:
          size: 200
          resolution: 101
          layers:
            - { kind: dome, amplitude: 1.5, radius: 25 }
      body: static
    - node: wall
      source: { box: { size: [2, 60, 6] } }
      transform: { position: [60, 0, 3] }
      body: static
    - node: crate
      source: { box: { size: [1, 1, 1] } }
      transform: { position: [-20, -60, 0.6] }
      body: { mass: 200 }
)";

LoadResult load_plants() {
  cvc::app app;
  register_world_block(app);
  LoadResult r = load_string(app, kPlants);
  if (r.ok)
    if (const Value *world = find_world_block(r)) {
      world_rules rules = default_world_rules();
      EXPECT_EQ(parse_world_block(*world, rules), "");
      r.scene.vehicles = rules.vehicles;
    }
  return r;
}

std::uint64_t spawn_one(WorldVehicles &v, const std::string &archetype,
                        const std::string &controller, double x, double y, double yaw = 0.0) {
  entity_spec s;
  s.archetype = archetype;
  s.controller = controller;
  s.placed = true;
  s.x = x;
  s.y = y;
  s.yaw = yaw;
  std::string why;
  const std::uint64_t id = v.spawn(s, &why);
  EXPECT_NE(id, 0u) << archetype << ": " << why;
  return id;
}

entity_info entity_of(const WorldVehicles &v, std::uint64_t id) {
  for (const entity_info &e : v.entities())
    if (e.id == id)
      return e;
  ADD_FAILURE() << "no entity " << id;
  return entity_info{};
}

void drive(cvc::app &app, SceneWorld &w, SceneClockDriver &clock, int frames) {
  for (int i = 0; i < frames; ++i) {
    clock.drive(1.0 / 60.0);
    w.follow(app.world_clock());
  }
}

} // namespace

TEST(AriadneWorldVehicles, EveryPlantDrivesInTheScenesWorld) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_plants();
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.vehicles.size(), 4u);
  EXPECT_EQ(r.scene.vehicles[2].plant, "dynamic4w");
  EXPECT_EQ(r.scene.vehicles[2].preset, "humvee");
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  WorldVehicles &v = *w->vehicles();
  w->sync(&warnings);
  ASSERT_EQ(v.status(), "ready");
  EXPECT_EQ(val(app, "vehicles.archetypes.four.plant"), "dynamic4w");
  const std::size_t bodies = w->world().body_count();
  const std::array<std::string, 4> kinds{"jolt", "bike", "four", "walker"};
  std::array<std::uint64_t, 4> ids{};
  for (std::size_t k = 0; k < 4; ++k)
    ids[k] = spawn_one(v, kinds[k], "nav@1", -50.0 + 25.0 * k, 40.0, -1.5);
  EXPECT_EQ(w->world().body_count(), bodies + 4) << "each one's chassis is a body in the world";
  std::array<entity_info, 4> start;
  for (std::size_t k = 0; k < 4; ++k)
    start[k] = entity_of(v, ids[k]);
  EXPECT_EQ(start[1].plant, "dynamic");
  EXPECT_EQ(start[3].plant, "kinematic");
  SceneClockDriver clock(app, resolved_clock(r.scene));
  drive(app, *w, clock, 480); // 8 s
  for (std::size_t k = 0; k < 4; ++k) {
    const entity_info e = entity_of(v, ids[k]);
    EXPECT_GT(planar(e, start[k].x, start[k].y), 5.0) << kinds[k] << " " << e.x << "," << e.y;
    EXPECT_GT(e.distance, 5.0) << kinds[k];
    EXPECT_GT(e.z, e.ground_z) << kinds[k] << ": above the ground";
    EXPECT_LT(e.z, e.ground_z + 3.0) << kinds[k] << ": on it, not flying";
    if (kinds[k] != "walker")
      EXPECT_GT(e.energy_wh, 0.0) << kinds[k];
  }
  const std::string lines = val(app, "physics.stats.vehicles");
  for (const char *plant : {"  jolt  ", "  dynamic  ", "  dynamic4w  ", "  kinematic  "})
    EXPECT_NE(lines.find(plant), std::string::npos) << plant << " in\n" << lines;

  // The four-wheeler's wheels as drawn: on the ground, below its box, the front ones steered.
  const std::uint64_t turning = spawn_one(v, "four", "external", -60.0, -30.0, 0.0);
  entity_intent turn;
  turn.command = entity_intent::mode::replace;
  turn.steer = 0.3;
  turn.accel = 0.5;
  ASSERT_TRUE(v.set_intent(turning, turn));
  drive(app, *w, clock, 120);
  const std::vector<entity_wheel> wheels = v.wheels(turning);
  ASSERT_EQ(wheels.size(), 4u);
  for (std::size_t k = 0; k < 4; ++k) {
    EXPECT_TRUE(wheels[k].down) << k;
    EXPECT_LT(wheels[k].z, 0.0) << k << ": below the box's centre";
    EXPECT_EQ(wheels[k].x > 0.0, k < 2) << k << ": front, then rear";
    EXPECT_EQ(wheels[k].y > 0.0, k % 2 == 0) << k << ": left, then right";
  }
  EXPECT_GT(wheels[0].steer, 0.2);
  EXPECT_GT(wheels[1].steer, 0.2);
  EXPECT_DOUBLE_EQ(wheels[2].steer, 0.0);
  EXPECT_GT(wheels[0].steer, wheels[1].steer) << "Ackermann: the inner wheel turns more";
  EXPECT_EQ(v.wheels(ids[0]).size(), 4u);
  EXPECT_TRUE(v.wheels(999).empty());
  EXPECT_TRUE(v.despawn(turning));

  // Removed: their bodies go.
  for (const std::uint64_t id : ids)
    EXPECT_TRUE(v.despawn(id));
  EXPECT_EQ(w->world().body_count(), bodies);
}

TEST(AriadneWorldVehicles, TheDynamicPlantsMeetTheBuildings) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_plants();
  ASSERT_TRUE(r.ok) << r.error;
  // Each driven at the wall at x = 59 (its face) from x = 30, on its own: the wall stops it.
  for (const char *kind : {"jolt", "bike", "four"}) {
    cvc::app app;
    std::vector<std::string> warnings;
    std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
    ASSERT_NE(w, nullptr);
    w->sync(&warnings);
    WorldVehicles &v = *w->vehicles();
    const std::uint64_t id = spawn_one(v, kind, "external", 30.0, 0.0, 0.0);
    entity_intent go;
    go.command = entity_intent::mode::replace;
    go.accel = 2.5;
    ASSERT_TRUE(v.set_intent(id, go));
    SceneClockDriver clock(app, resolved_clock(r.scene));
    drive(app, *w, clock, 600); // 10 s: far enough to pass it twice over
    const entity_info e = entity_of(v, id);
    EXPECT_LT(e.x, 59.0) << kind << ": stopped by the wall, not through it";
    EXPECT_GT(e.x, 45.0) << kind << ": it got there";
    EXPECT_LT(std::fabs(e.speed), 2.0) << kind;
  }
  // The kinematic one pushes what it meets: the crate in its way moves.
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  const cvc::physics::body_id crate = w->body("crate");
  SceneClockDriver clock(app, resolved_clock(r.scene));
  drive(app, *w, clock, 60); // the crate settles
  const double crate_x = w->world().state(crate).position.x;
  const std::uint64_t id = spawn_one(v, "walker", "external", -35.0, -60.0, 0.0);
  entity_intent go;
  go.command = entity_intent::mode::replace;
  go.accel = 2.0;
  ASSERT_TRUE(v.set_intent(id, go));
  drive(app, *w, clock, 360);
  EXPECT_GT(entity_of(v, id).x, -20.0) << "it drove through the crate's place";
  EXPECT_GT(w->world().state(crate).position.x, crate_x + 2.0) << "and pushed it along";
}

TEST(AriadneWorldVehicles, ARestartKeepsTheScenesBodiesWithDynamicPlants) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_plants();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  const std::size_t bodies = w->world().body_count();
  // Driven by their intents (nav@1 draws new goals after a restart, so it does not replay).
  std::vector<std::uint64_t> ids;
  for (const char *kind : {"bike", "four", "walker", "jolt"}) {
    ids.push_back(spawn_one(v, kind, "external", -40.0 + 20.0 * ids.size(), -20.0, 1.2));
    entity_intent go;
    go.command = entity_intent::mode::replace;
    go.steer = 0.1;
    go.accel = 1.5;
    EXPECT_TRUE(v.set_intent(ids.back(), go));
  }
  SceneClockDriver clock(app, resolved_clock(r.scene));
  drive(app, *w, clock, 180);
  // Restart: a fresh world with the scene's bodies made again under the same ids, and each
  // entity back at its start. The old vehicles' bodies are not removed from the new world.
  w->restart(&warnings);
  EXPECT_EQ(w->world().body_count(), bodies + ids.size());
  EXPECT_TRUE(w->body("crate").valid());
  EXPECT_TRUE(w->body("wall").valid());
  for (std::size_t k = 0; k < ids.size(); ++k) {
    const entity_info e = entity_of(v, ids[k]);
    EXPECT_TRUE(e.placed);
    EXPECT_NEAR(e.speed, 0.0, 1e-6);
  }
  // It steps again as a new world: the same run twice over gives the same state.
  drive(app, *w, clock, 120);
  const std::uint64_t once = w->world().state_hash();
  w->restart(&warnings);
  drive(app, *w, clock, 120);
  EXPECT_EQ(w->world().state_hash(), once);
}

// ---------------------------------------------------------------------------------------------
// Where the vehicles start (ARIADNE_WORLDS.md section 14, L3): spawn points, their policy, a
// minimap's picks, and points suggested along the roads.

namespace {

const char *kSpawns = R"(
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee, controllers: [none, nav@1] }
      racer: { vehicle: roboracer, controllers: [none] }
  spawns:
    policy: round_robin
    scope: team
    points:
      - { at: [-40, -40], heading: 90, team: 0, world: square }
      - { at: [40, -40], heading: 90, team: 0, world: square }
      - { at: [0, 40], heading: 180, team: 1, archetype: racer, world: square }
      - { at: [0, 0], world: field }
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [120, 120, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - node: field
      world: field
      source: { box: { size: [80, 80, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)";

LoadResult load_spawns(const char *text) {
  cvc::app app;
  register_world_block(app);
  LoadResult r = load_string(app, text);
  if (r.ok)
    if (const Value *world = find_world_block(r)) {
      world_rules rules = default_world_rules();
      EXPECT_EQ(parse_world_block(*world, rules), "");
      r.scene.vehicles = rules.vehicles;
      r.scene.spawns = rules.spawns;
    }
  return r;
}

// A pick on the minimap, as a viewport writes it.
void click(cvc::app &app, double x, double y) {
  cvc::state &st = cvc::state::instance(app);
  st("spawns.pick.x").value(std::to_string(x));
  st("spawns.pick.y").value(std::to_string(y));
  st("spawns.pick.hit").value("1");
  st("spawns.pick.phase").value("down");
  const std::string seq = st("spawns.pick.seq").value();
  st("spawns.pick.seq").value(std::to_string(seq.empty() ? 1 : std::stoi(seq) + 1));
}

} // namespace

TEST(AriadneWorldSpawns, TheWorldBlockDeclaresSpawnPoints) {
  REQUIRE_YAML();
  LoadResult r = load_spawns(kSpawns);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.scene.spawns.policy, "round_robin");
  EXPECT_EQ(r.scene.spawns.scope, "team");
  ASSERT_EQ(r.scene.spawns.points.size(), 4u);
  EXPECT_EQ(r.scene.spawns.points[0].x, -40.0);
  EXPECT_EQ(r.scene.spawns.points[0].heading, 90.0);
  EXPECT_EQ(r.scene.spawns.points[2].team, 1);
  EXPECT_EQ(r.scene.spawns.points[2].archetype, "racer");
  EXPECT_EQ(r.scene.spawns.points[3].world, "field");
  EXPECT_EQ(r.scene.spawns.points[3].team, -1);
  const auto error = [](const std::string &spawns) {
    cvc::app app; // its own, with the world: block
    register_world_block(app);
    LoadResult bad =
        load_string(app, "world:\n  vehicles:\n    archetypes:\n      v: { vehicle: humvee "
                         "}\n  spawns: " +
                             spawns + "\nscene:\n  nodes: []\n");
    world_rules x = default_world_rules();
    const Value *block = find_world_block(bad);
    return block ? parse_world_block(*block, x) : std::string("no world: block");
  };
  EXPECT_NE(error("{ policy: closest }").find("policy 'closest'"), std::string::npos);
  EXPECT_NE(error("{ scope: colour }").find("scope 'colour'"), std::string::npos);
  EXPECT_NE(error("{ points: [ { heading: 3 } ] }").find("has no at:"), std::string::npos);
  EXPECT_NE(error("{ points: [ { at: [1, 2], archetype: tank } ] }").find("vehicle 'tank'"),
            std::string::npos);
  EXPECT_EQ(error("{ policy: nearest_free, scope: all, points: [ { at: [1, 2] } ] }"), "");
}

TEST(AriadneWorldSpawns, PointsBelongToTheirWorldAndTheMinimapEditsThem) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_spawns(kSpawns);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  ASSERT_NE(w->vehicles(), nullptr);
  w->sync(&warnings);
  WorldSpawns &sp = w->vehicles()->spawns();
  // The square's three; the field's is not in the table.
  EXPECT_EQ(sp.points().size(), 3u);
  EXPECT_EQ(val(app, "spawns.ids"), "1,2,3");
  EXPECT_EQ(val(app, "spawns.world"), "square");
  EXPECT_EQ(val(app, "spawns.points.3.archetype"), "racer");
  EXPECT_EQ(val(app, "spawns.policy"), "round_robin");
  EXPECT_EQ(val(app, "spawns.tool"), "add");

  // A click adds a point facing the heading, tagged as the tag keys say.
  cvc::state &st = cvc::state::instance(app);
  st("spawns.heading").value("270");
  st("spawns.tag.team").value("2");
  click(app, 20.0, 20.0);
  w->sync(&warnings);
  ASSERT_EQ(sp.points().size(), 4u);
  const WorldSpawns::entry added = sp.points().back();
  EXPECT_DOUBLE_EQ(added.point.x, 20.0);
  EXPECT_DOUBLE_EQ(added.point.heading, 270.0);
  EXPECT_EQ(added.point.team, 2);
  EXPECT_EQ(added.point.world, "square");
  EXPECT_EQ(val(app, "spawns.points.5.team"), "2");
  // Refused: off the world, and too near another.
  click(app, 500.0, 0.0);
  w->sync(&warnings);
  EXPECT_NE(val(app, "spawns.status").find("off the world"), std::string::npos)
      << val(app, "spawns.status");
  click(app, 22.0, 21.0);
  w->sync(&warnings);
  EXPECT_NE(val(app, "spawns.status").find("within"), std::string::npos);
  EXPECT_EQ(sp.points().size(), 4u);
  // Tag the nearest, then remove it.
  st("spawns.tool").value("tag");
  st("spawns.tag.team").value("-1");
  st("spawns.tag.archetype").value("humvee");
  click(app, 21.0, 19.0);
  w->sync(&warnings);
  EXPECT_EQ(sp.points().back().point.team, -1);
  EXPECT_EQ(sp.points().back().point.archetype, "humvee");
  st("spawns.tool").value("remove");
  click(app, 19.0, 20.0);
  w->sync(&warnings);
  EXPECT_EQ(sp.points().size(), 3u);
  EXPECT_EQ(cvc::state::instance(app).findDescendant("spawns.points.5"), nullptr);

  // Another world: its own points.
  EXPECT_TRUE(w->set_world("field", &warnings));
  w->sync(&warnings);
  ASSERT_EQ(sp.points().size(), 1u);
  EXPECT_EQ(sp.points()[0].point.world, "field");
  EXPECT_EQ(val(app, "spawns.ids"), "4");
  st("spawns.clear.request").value("1");
  w->sync(&warnings);
  EXPECT_TRUE(sp.points().empty());
  EXPECT_TRUE(w->set_world("square", &warnings));
  w->sync(&warnings);
  EXPECT_EQ(sp.points().size(), 3u); // the square's kept
}

TEST(AriadneWorldSpawns, ThePolicyPicksTheSpawnsPoint) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_spawns(kSpawns);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  const auto start_of = [&](int team, const std::string &archetype) {
    entity_spec s;
    s.archetype = archetype;
    s.team = team;
    std::string why;
    const std::uint64_t id = v.spawn(s, &why);
    EXPECT_NE(id, 0u) << why;
    for (const entity_info &e : v.entities())
      if (e.id == id)
        return std::array<double, 2>{e.x, e.y};
    return std::array<double, 2>{1e9, 1e9};
  };
  const auto near_point = [](const std::array<double, 2> &at, double x, double y) {
    return std::hypot(at[0] - x, at[1] - y) < 4.0; // the chassis centre, ahead of the point
  };
  // round_robin per team: team 0 takes its two points in turn, the third spawn moves aside.
  const auto a = start_of(0, "humvee");
  const auto b = start_of(0, "humvee");
  EXPECT_TRUE(near_point(a, -40, -40)) << a[0] << "," << a[1];
  EXPECT_TRUE(near_point(b, 40, -40)) << b[0] << "," << b[1];
  const auto c = start_of(0, "humvee");
  EXPECT_GT(std::hypot(c[0] - a[0], c[1] - a[1]), 3.0); // not on the first one's vehicle
  // The racer point is team 1's, for racers only; a team 1 humvee has no point: a free one.
  const auto d = start_of(1, "racer");
  EXPECT_TRUE(near_point(d, 0, 40)) << d[0] << "," << d[1];
  const auto e = start_of(1, "humvee");
  EXPECT_FALSE(near_point(e, 0, 40));
  // fixed: always the first that fits; nearest_free: the first no vehicle is near.
  v.clear();
  cvc::state &st = cvc::state::instance(app);
  st("spawns.policy").value("nearest_free");
  w->sync(&warnings);
  const auto f = start_of(0, "humvee");
  const auto g = start_of(0, "humvee");
  EXPECT_TRUE(near_point(f, -40, -40));
  EXPECT_TRUE(near_point(g, 40, -40)) << g[0] << "," << g[1];
  v.clear();
  st("spawns.policy").value("random");
  w->sync(&warnings);
  for (int i = 0; i < 4; ++i) {
    const auto h = start_of(0, "humvee");
    v.clear();
    EXPECT_TRUE(near_point(h, -40, -40) || near_point(h, 40, -40)) << h[0] << "," << h[1];
  }
}

TEST(AriadneWorldSpawns, PointsAreSuggestedAlongTheRoads) {
  REQUIRE_YAML();
  LoadResult r = load_spawns(R"(
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee }
scene:
  physics: {}
  world: { source: city, cell: 4 }
  nodes:
    - node: land
      type: generated_world
      world: city
      seed: 5
      half: 150
      trees: 4
      rocks: 2
      buildings: 12
      body: {}
)");
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  const std::shared_ptr<const cvc::world::generated_world> gw = w->generated_world("land");
  ASSERT_NE(gw, nullptr);
  if (gw->streets.empty())
    GTEST_SKIP() << "this world has no streets";
  cvc::state &st = cvc::state::instance(app);
  st("spawns.suggest.count").value("8");
  st("spawns.suggest.spacing").value("40");
  st("spawns.suggest.request").value("1");
  w->sync(&warnings);
  WorldSpawns &sp = w->vehicles()->spawns();
  ASSERT_FALSE(sp.suggestions().empty()) << sp.status();
  EXPECT_LE(sp.suggestions().size(), 8u);
  for (const spawn_point &p : sp.suggestions()) {
    int seg = -1;
    EXPECT_LT(gw->streets.distance_to_road(p.x, p.y, &seg), 12.0) << p.x << "," << p.y;
  }
  EXPECT_EQ(val(app, "spawns.suggestions.count"), std::to_string(sp.suggestions().size()));
  // The same world suggests the same points.
  const std::vector<spawn_point> first = sp.suggestions();
  st("spawns.suggest.request").value("2");
  w->sync(&warnings);
  EXPECT_EQ(sp.suggestions(), first);
  // Accepted: they are points.
  st("spawns.accept.request").value("1");
  w->sync(&warnings);
  EXPECT_TRUE(sp.suggestions().empty());
  EXPECT_GE(sp.points().size(), 1u);
  EXPECT_LE(sp.points().size(), first.size());
}

// --- boats, and spawns by medium (ARIADNE_WORLDS.md §14; OCEAN-AND-WEATHER-ROADMAP.md D4) -----

namespace {

// A 300 m square of flat ground at 0 m with a bowl 6 m deep at its centre (radius 40 m), and the
// water node's water at -1 m over it: a round lake about 107 m across, 5 m deep in the middle.
// Points: two untagged in the lake, two untagged ashore, a boat's ashore and a car's in the lake.
const char *kLake = R"(
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee, controllers: [none, nav@1] }
      launch: { plant: dynamicboat:launch, controllers: [none, nav@1] }
  spawns:
    policy: fixed
    scope: all
    points:
      - { at: [0, 0] }
      - { at: [-30, 10] }
      - { at: [80, 80] }
      - { at: [-90, 60] }
      - { at: [100, -100], archetype: launch }
      - { at: [10, -20], archetype: humvee }
scene:
  physics: {}
  world: { source: lakeside, cell: 2 }
  nodes:
    - node: terrain
      world: lakeside
      source:
        heightfield:
          size: 300
          resolution: 151
          layers:
            - { kind: dome, amplitude: -6, radius: 40 }
      body: static
    - node: water
      type: water
      ground: terrain
      level: -1
      wind_speed: 0.5
      wave_amp: 0.1
      body: {}
)";

// The lake's depth at (x, y): the water at -1 m over the bowl.
double lake_depth(double x, double y) {
  return -1.0 + 6.0 * std::exp(-(x * x + y * y) / (40.0 * 40.0));
}

std::array<double, 2> spawned_at(WorldVehicles &v, const std::string &archetype, std::string *why,
                                 bool placed = false, double x = 0.0, double y = 0.0) {
  entity_spec s;
  s.archetype = archetype;
  s.controller = "none";
  s.placed = placed;
  s.x = x;
  s.y = y;
  const std::uint64_t id = v.spawn(s, why);
  if (!id)
    return {1e9, 1e9};
  for (const entity_info &e : v.entities())
    if (e.id == id)
      return {e.x, e.y};
  return {1e9, 1e9};
}

} // namespace

// The lake world's grid carries the water's depth (the water node's level over the terrain), and
// the archetypes' needs come from their plants: a humvee on land, a launch on water with its draft
// (0.2 m: its propeller's, under the keel's 0.15 m) and half its length. The points say what they
// are free for.
TEST(AriadneWorldBoats, TheGridCarriesTheWaterAndThePlantsSayTheirMedium) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_spawns(kLake);
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.vehicles.size(), 2u);
  EXPECT_EQ(r.scene.vehicles[1].plant, "dynamicboat");
  EXPECT_EQ(r.scene.vehicles[1].preset, "launch");
  EXPECT_DOUBLE_EQ(r.scene.spawns.ford_depth, 0.3);
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  const auto g = w->grid("terrain", 2.0, 0.0);
  ASSERT_NE(g, nullptr);
  ASSERT_TRUE(g->has_water());
  EXPECT_DOUBLE_EQ(g->water_level, -1.0);
  EXPECT_NEAR(g->depth_at(0.0, 0.0), 5.0, 0.01);
  EXPECT_LT(g->depth_at(80.0, 80.0), 0.0);
  WorldSpawns &sp = w->vehicles()->spawns();
  EXPECT_EQ(sp.need_of("humvee").medium, cvc::nav::plant_medium::land);
  const spawn_need boat = sp.need_of("launch");
  EXPECT_EQ(boat.medium, cvc::nav::plant_medium::water);
  EXPECT_NEAR(boat.draft, 0.2, 1e-9);
  EXPECT_NEAR(boat.half_length, 2.0, 1e-9);
  EXPECT_EQ(val(app, "vehicles.archetypes.launch.medium"), "water");
  const std::vector<WorldSpawns::entry> pts = sp.points();
  ASSERT_EQ(pts.size(), 6u);
  const char *medium[6] = {"water", "water", "land", "land", "", ""};
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(pts[i].medium, medium[i]) << "point " << pts[i].id;
    EXPECT_EQ(pts[i].free, i < 4) << "point " << pts[i].id;
  }
  EXPECT_EQ(val(app, "spawns.points.1.medium"), "water");
  EXPECT_EQ(val(app, "spawns.points.5.free"), "0") << "a boat's point ashore";
  // Edits: an untagged point in the lake is taken (a boat's), a car's in the lake is refused.
  spawn_point p;
  p.x = 20.0;
  p.y = 20.0;
  EXPECT_NE(sp.add(p), 0);
  std::string why;
  p.x = -20.0;
  p.archetype = "humvee";
  EXPECT_EQ(sp.add(p, &why), 0);
  EXPECT_NE(why.find("humvee's ground"), std::string::npos) << why;
}

// The owner's ask: cars never start in water deeper than a ford, over many spawns and every
// policy; boats start only on water deep enough for their draft and open for half their length
// round them. The untagged points serve each archetype where its medium is; the tagged points
// off their medium serve nothing.
TEST(AriadneWorldBoats, SpawnsKeepCarsOutOfTheLakeAndBoatsInIt) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_spawns(kLake);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  WorldSpawns &sp = v.spawns();
  const WorldSpawns::start_ground boats = sp.ground_for("launch");
  cvc::state &st = cvc::state::instance(app);
  int cars = 0, launches = 0;
  for (const char *policy : {"fixed", "random", "round_robin", "nearest_free"}) {
    st("spawns.policy").value(std::string(policy));
    w->sync(&warnings);
    for (int i = 0; i < 12; ++i) {
      std::string why;
      const std::array<double, 2> c = spawned_at(v, "humvee", &why);
      ASSERT_LT(c[0], 1e8) << policy << ": " << why;
      EXPECT_LE(lake_depth(c[0], c[1]), 0.3) << policy << ": a humvee at " << c[0] << ", " << c[1];
      ++cars;
    }
    for (int i = 0; i < 6; ++i) {
      std::string why;
      const std::array<double, 2> b = spawned_at(v, "launch", &why);
      ASSERT_LT(b[0], 1e8) << policy << ": " << why;
      EXPECT_GE(lake_depth(b[0], b[1]), 0.5) << policy << ": a launch at " << b[0] << ", " << b[1];
      EXPECT_TRUE(boats.free_at(b[0], b[1])) << policy << ": " << b[0] << ", " << b[1];
      ++launches;
    }
    // fixed takes the first point that fits each: the boat the lake's centre, the car (80, 80).
    if (std::string(policy) == "fixed") {
      bool centre = false, ashore = false;
      for (const entity_info &e : v.entities()) {
        centre = centre || (e.spec.archetype == "launch" && std::hypot(e.x, e.y) < 3.0);
        ashore =
            ashore || (e.spec.archetype == "humvee" && std::hypot(e.x - 80.0, e.y - 80.0) < 4.0);
      }
      EXPECT_TRUE(centre) << "an untagged point in the lake serves the launch";
      EXPECT_TRUE(ashore) << "an untagged point ashore serves the humvee";
    }
    v.clear();
  }
  std::printf("[boats] %d humvees and %d launches over four policies, each on its medium\n", cars,
              launches);

  // No points: the free_points fallback draws each on its medium too.
  sp.clear();
  EXPECT_TRUE(sp.points().empty());
  for (int i = 0; i < 8; ++i) {
    std::string why;
    const std::array<double, 2> c = spawned_at(v, "humvee", &why);
    const std::array<double, 2> b = spawned_at(v, "launch", &why);
    ASSERT_LT(c[0], 1e8) << why;
    ASSERT_LT(b[0], 1e8) << why;
    EXPECT_LE(lake_depth(c[0], c[1]), 0.3) << c[0] << ", " << c[1];
    EXPECT_GE(lake_depth(b[0], b[1]), 0.5) << b[0] << ", " << b[1];
    EXPECT_TRUE(boats.free_at(b[0], b[1]));
  }
}

// A car placed at a point in the lake is moved, not refused: to a free point ashore, as a start
// that is blocked has always moved (a regenerated world can flood a start a document gave). A
// boat placed ashore is moved into the lake the same way.
TEST(AriadneWorldBoats, AStartOffItsMediumIsMovedToIt) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_spawns(kLake);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  std::string why;
  const std::array<double, 2> car = spawned_at(v, "humvee", &why, true, 0.0, 0.0);
  ASSERT_LT(car[0], 1e8) << why;
  EXPECT_LE(lake_depth(car[0], car[1]), 0.3) << car[0] << ", " << car[1];
  EXPECT_GT(std::hypot(car[0], car[1]), 45.0);
  const std::array<double, 2> boat = spawned_at(v, "launch", &why, true, 80.0, 80.0);
  ASSERT_LT(boat[0], 1e8) << why;
  EXPECT_GE(lake_depth(boat[0], boat[1]), 0.5) << boat[0] << ", " << boat[1];
  // A place on its medium is kept.
  const std::array<double, 2> kept = spawned_at(v, "launch", &why, true, -20.0, -20.0);
  EXPECT_NEAR(kept[0], -20.0, 0.5);
  EXPECT_NEAR(kept[1], -20.0, 0.5);
}

// A boat's suggestions are over the water, free for it; a land vehicle's are along the roads, and
// this world has none.
TEST(AriadneWorldBoats, ABoatsSuggestionsAreOverTheWater) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_spawns(kLake);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldSpawns &sp = w->vehicles()->spawns();
  cvc::state &st = cvc::state::instance(app);
  st("spawns.tag.archetype").value(std::string("launch"));
  const std::vector<spawn_point> s = sp.suggest(6, 20.0);
  ASSERT_FALSE(s.empty()) << sp.status();
  EXPECT_LE(s.size(), 6u);
  for (const spawn_point &p : s)
    EXPECT_TRUE(sp.ground_for("launch").free_at(p.x, p.y)) << p.x << ", " << p.y;
  EXPECT_NE(sp.status().find("over the water"), std::string::npos) << sp.status();
  st("spawns.tag.archetype").value(std::string("humvee"));
  EXPECT_TRUE(sp.suggest(6, 20.0).empty());
  // water_spawn_points gives every body of water with room a point: two ponds, one each.
  cvc::nav::world_grid g = cvc::nav::flat_grid(-50.0, -20.0, 50.0, 20.0, 1.0, 0.0f);
  for (int r2 = 0; r2 < g.rows; ++r2)
    for (int c = 0; c < g.cols; ++c) {
      const double x = g.min_x + c, y = g.min_y + r2;
      const bool pond = std::hypot(x + 30.0, y) < 10.0 || std::hypot(x - 30.0, y) < 6.0;
      g.height[static_cast<std::size_t>(r2) * g.cols + c] = pond ? -2.0f : 1.0f;
    }
  cvc::nav::set_water(g, 0.0);
  const cvc::nav::world_grid water = cvc::nav::medium_grid(g, cvc::nav::plant_medium::water, 0.5);
  const std::vector<spawn_point> two = water_spawn_points(water, 2, 50.0, 2.0);
  ASSERT_EQ(two.size(), 2u);
  EXPECT_NE(two[0].x < 0.0, two[1].x < 0.0) << "one in each pond";
}

// The roadmap's D4 exit as a world vehicle: a launch on nav@1 starts placed in the lake and the
// navigator drives it to goal after goal it draws on the water, never into water shallower than
// its draft nor onto the shore (no contact with the terrain).
TEST(AriadneWorldBoats, NavDrivesABoatAcrossTheLake) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_spawns(kLake);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  WorldVehicles &v = *w->vehicles();
  entity_spec s;
  s.archetype = "launch";
  s.controller = "nav@1";
  s.placed = true;
  s.x = -35.0;
  s.y = 0.0;
  s.yaw = 0.0;
  std::string why;
  const std::uint64_t id = v.spawn(s, &why);
  ASSERT_NE(id, 0u) << why;
  const auto me = [&]() {
    for (const entity_info &e : v.entities())
      if (e.id == id)
        return e;
    return entity_info{};
  };
  entity_info e = me();
  ASSERT_TRUE(e.placed);
  EXPECT_NEAR(e.x, -35.0, 0.5);
  EXPECT_NEAR(e.z, -1.0 + 0.15, 0.05) << "afloat at its draft";
  SceneClockDriver clock(app, resolved_clock(r.scene));
  double goal_x = e.goal_x, goal_y = e.goal_y, min_depth = 1e9, travelled = 0.0;
  double px = e.x, py = e.y;
  int goals = 0, frames = 0;
  const double first = std::hypot(goal_x - e.x, goal_y - e.y);
  for (; frames < 60 * 240 && goals < 2; ++frames) { // up to 4 minutes
    const int i = frames;
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
    if (i % 6)
      continue;
    e = me();
    min_depth = std::min(min_depth, lake_depth(e.x, e.y));
    travelled += std::hypot(e.x - px, e.y - py);
    px = e.x;
    py = e.y;
    if (e.goal_x != goal_x || e.goal_y != goal_y) { // reached: nav@1 drew the next
      ++goals;
      goal_x = e.goal_x;
      goal_y = e.goal_y;
    }
  }
  std::printf("[boats] the launch reached %d goals in %.1f s (the first %.1f m off), travelling "
              "%.1f m; the shallowest water under it %.2f m\n",
              goals, frames / 60.0, first, travelled, min_depth);
  EXPECT_GE(goals, 2) << "nav did not drive it to its goals";
  EXPECT_GT(min_depth, 0.2) << "it reached water shallower than its draft";
  EXPECT_GT(travelled, 40.0);
}

// A world with water and no water node (a generated world flooded to 200 m): the runtime adds
// still water at the world's level for its boats once it has the grid, and a jonboat floats on it.
TEST(AriadneWorldBoats, WithoutAWaterNodeTheBoatsGetStillWater) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_spawns(R"(
world:
  vehicles:
    archetypes:
      jonboat: { plant: dynamicboat, vehicle: jonboat, controllers: [none] }
scene:
  physics: {}
  world: { source: sea, cell: 4 }
  nodes:
    - node: world
      type: generated_world
      world: sea
      seed: 5
      half: 150
      trees: 0
      rocks: 0
      buildings: 0
      water_level: 200
      body: {}
)");
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(w->world().field_count(), 0u) << "no water node";
  w->sync(&warnings);
  EXPECT_EQ(w->world().field_count(), 1u) << "the still water the runtime added";
  WorldVehicles &v = *w->vehicles();
  std::string why;
  const std::array<double, 2> b = spawned_at(v, "jonboat", &why);
  ASSERT_LT(b[0], 1e8) << why;
  EXPECT_EQ(w->world().field_count(), 1u);
  SceneClockDriver clock(app, resolved_clock(r.scene));
  for (int i = 0; i < 300; ++i) {
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
  }
  for (const entity_info &e : v.entities())
    EXPECT_NEAR(e.z, 200.0 + 0.25 - 350.0 / (3.6 * 1.4 * 1000.0), 0.05) << "afloat";
}

// The world: block takes a boat archetype and the spawns' knobs, and refuses what it cannot run.
TEST(AriadneWorldBoats, TheWorldBlockTakesBoatsAndTheSpawnKnobs) {
  REQUIRE_YAML();
  const auto parse = [](const std::string &block, world_rules &out) {
    cvc::app app;
    register_world_block(app);
    LoadResult lr = load_string(app, block + "\nscene:\n  nodes: []\n");
    const Value *b = find_world_block(lr);
    out = default_world_rules();
    return b ? parse_world_block(*b, out) : std::string("no world: block");
  };
  world_rules rules;
  EXPECT_EQ(parse(R"(world:
  vehicles:
    archetypes:
      a: { plant: dynamicboat, vehicle: jonboat }
      b: { plant: "dynamicboat:launch", vehicle: { mass: 1000, max_speed: 3 } }
  spawns: { ford_depth: 0.5, keel_clearance: 0.2, shore_clearance: 1.5 })",
                  rules),
            "");
  ASSERT_EQ(rules.vehicles.size(), 2u);
  EXPECT_EQ(rules.vehicles[1].preset, "launch");
  EXPECT_DOUBLE_EQ(rules.spawns.ford_depth, 0.5);
  EXPECT_DOUBLE_EQ(rules.spawns.keel_clearance, 0.2);
  EXPECT_DOUBLE_EQ(rules.spawns.shore_clearance, 1.5);
  EXPECT_TRUE(vehicle_plant_floats("dynamicboat"));
  EXPECT_FALSE(vehicle_plant_floats("dynamic"));
  EXPECT_NE(parse("world:\n  vehicles:\n    archetypes:\n      a: { plant: dynamicboat, vehicle: "
                  "humvee }",
                  rules)
                .find("boat preset"),
            std::string::npos);
  EXPECT_NE(parse("world:\n  vehicles:\n    archetypes:\n      a: { plant: jolt, vehicle: launch }",
                  rules)
                .find("is not a preset"),
            std::string::npos);
  EXPECT_NE(parse("world:\n  vehicles:\n    archetypes:\n      a: { plant: dynamicboat, vehicle: "
                  "{ preset: launch, max_steer: 0.3 } }",
                  rules)
                .find("not a boat's"),
            std::string::npos);
  EXPECT_NE(parse("world:\n  spawns: { ford_depth: -1 }", rules).find("zero or more"),
            std::string::npos);
  // The boat's vehicle as the runtime reads one: its hull's box, mass and envelope, no wheels.
  vehicle_archetype a;
  a.id = "b";
  a.plant = "dynamicboat";
  a.preset = "launch";
  a.ranges["mass"] = {800.0, 1000.0};
  const cvc::physics::vehicle_desc d = vehicle_desc_of(a, 2000.0);
  EXPECT_DOUBLE_EQ(d.mass, 1000.0) << "held to the range";
  EXPECT_DOUBLE_EQ(d.half_extents.x, 2.0);
  EXPECT_TRUE(d.axles.empty());
  EXPECT_DOUBLE_EQ(boat_desc_of(a).boat.hull.mass, 900.0);
  a.preset = "humvee";
  EXPECT_THROW(boat_desc_of(a), std::invalid_argument);
}

// --- the boat node (OCEAN-AND-WEATHER-ROADMAP.md D6) -------------------------------------------

namespace {

// The lake of kLake's world, with a launch as a boat node at its west side.
const char *kBoatNode = R"(
scene:
  physics: {}
  nodes:
    - node: terrain
      source:
        heightfield:
          size: 300
          resolution: 151
          layers:
            - { kind: dome, amplitude: -6, radius: 40 }
      body: static
    - node: water
      type: water
      ground: terrain
      level: -1
      wind_speed: 0.5
      wave_amp: 0.1
      body: {}
    - node: skiff
      type: boat
      water: water
      preset: launch
      transform: { position: [-30, 0, 0] }
      goal: [30, 10]
)";

} // namespace

// The node puts its boat in the world at its draft once the water's grid is ready; its keys drive
// it (a helm: it moves and turns); drive: nav takes it to its goal across the lake without water
// under it shallower than its draft; a restart puts it back where it started.
TEST(AriadneBoatNode, FloatsSteersFromItsKeysAndNavTakesItToItsGoal) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = fresh_load(kBoatNode);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  EXPECT_TRUE(warnings.empty()) << warnings.front();
  ASSERT_EQ(w->boats().size(), 1u);
  BoatRuntime *b = w->boat("skiff");
  ASSERT_NE(b, nullptr);
  ASSERT_TRUE(b->placed()) << b->status();
  EXPECT_EQ(val(app, "skiff.placed"), "1");
  EXPECT_EQ(val(app, "skiff.drive"), "keys");
  EXPECT_NEAR(b->draft(), 0.2, 1e-9);
  cvc::physics::boat_state s = b->state();
  EXPECT_NEAR(s.body.position.x, -30.0, 1e-6);
  EXPECT_NEAR(s.body.position.z, -1.0 + 0.15, 0.01) << "afloat at its draft";

  SceneClockDriver clock(app, resolved_clock(r.scene));
  const auto run = [&](double seconds) {
    for (int i = 0; i < static_cast<int>(seconds * 60.0); ++i) {
      clock.drive(1.0 / 60.0);
      w->follow(app.world_clock());
    }
  };
  run(2.0);
  s = b->state();
  EXPECT_NEAR(s.body.position.z, -1.0 + 0.15, 0.05) << "still afloat, at rest";
  EXPECT_LT(std::fabs(s.speed), 0.05);
  // The helm: full ahead and half rudder (left).
  cvc::state &st = cvc::state::instance(app);
  st("skiff.throttle").value(1.0);
  st("skiff.rudder").value(0.5);
  const double h0 = s.heading;
  run(12.0);
  s = b->state();
  std::printf("[boat node] on the helm for 12 s: at (%.1f, %.1f), %.2f m/s, turned %.2f rad\n",
              s.body.position.x, s.body.position.y, s.speed, s.heading - h0);
  EXPECT_GT(s.speed, 1.5);
  EXPECT_GT(std::remainder(s.heading - h0, 2.0 * 3.14159265358979), 0.5) << "turned left";
  EXPECT_GT(std::hypot(s.body.position.x + 30.0, s.body.position.y), 5.0);
  EXPECT_NE(val(app, "skiff.speed"), "");

  // drive: nav, to its goal.
  st("skiff.drive").value(std::string("nav"));
  double shallowest = 1e9;
  int frames = 0;
  for (; frames < 60 * 120 && val(app, "skiff.reached") != "1"; ++frames) {
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
    const cvc::physics::vec3 p = b->state().body.position;
    shallowest = std::min(shallowest, lake_depth(p.x, p.y));
  }
  s = b->state();
  std::printf("[boat node] nav took it to (%.1f, %.1f) in %.1f s; the shallowest water under it "
              "%.2f m\n",
              s.body.position.x, s.body.position.y, frames / 60.0, shallowest);
  EXPECT_EQ(val(app, "skiff.reached"), "1");
  EXPECT_LT(std::hypot(s.body.position.x - 30.0, s.body.position.y - 10.0), 8.0);
  EXPECT_GT(shallowest, 0.2);

  // A restart: the boat back where it started, at rest.
  w->restart(&warnings);
  w->sync(&warnings);
  ASSERT_TRUE(b->placed());
  s = b->state();
  EXPECT_NEAR(s.body.position.x, -30.0, 1e-6);
  EXPECT_NEAR(s.body.position.y, 0.0, 1e-6);
}

// terrain_lab.ari's launch (D6): run headless, it floats on the lake by the crates, on the water
// node's 9 m/s sea with its 0.8 m swell, and nav takes it to its goal 45 m across the lake, clear
// of the crates (the navigator's grid has the lake's shores, not its floating bodies).
TEST(AriadneBoatNode, NavTakesTerrainLabsLaunchAcrossTheLake) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const fs::path doc = fs::path(CVC_EXAMPLES_DIR) / "ariadne" / "documents" / "terrain_lab.ari";
  LoadResult r = fresh_load_file(doc.string());
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  BoatRuntime *b = w->boat("launch");
  ASSERT_NE(b, nullptr);
  ASSERT_TRUE(b->placed()) << b->status();
  const auto g = w->grid("world", 2.0, 0.0);
  ASSERT_NE(g, nullptr);
  EXPECT_GT(g->depth_at(0.0, 170.0), 5.0);
  EXPECT_GT(g->depth_at(40.0, 150.0), 5.0);
  cvc::physics::boat_state s = b->state();
  EXPECT_NEAR(s.body.position.x, 0.0, 1e-6);
  EXPECT_NEAR(s.body.position.z, 0.15, 1.0) << "on the sea's surface, at its draft";
  cvc::state::instance(app)("launch.drive").value(std::string("nav"));
  SceneClockDriver clock(app, resolved_clock(r.scene));
  double shallowest = 1e9;
  int frames = 0;
  for (; frames < 60 * 150 && val(app, "launch.reached") != "1"; ++frames) {
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
    const cvc::physics::vec3 p = b->state().body.position;
    shallowest = std::min(shallowest, g->depth_at(p.x, p.y));
  }
  s = b->state();
  std::printf("[boat node] terrain_lab's launch reached (%.1f, %.1f) in %.1f s; the shallowest "
              "water under it %.1f m\n",
              s.body.position.x, s.body.position.y, frames / 60.0, shallowest);
  EXPECT_EQ(val(app, "launch.reached"), "1");
  EXPECT_GT(shallowest, 1.0);
}

// ---------------------------------------------------------------------------------------------
// C5 (D6): with the water node's `wake_push`, the waves the floating bodies make push the others
// (cvc::physics::field_wake), inside the physics step.

namespace {

// kBoatNode's lake with two launches: `runner` at its west side, heading east, and `moored` 22 m
// on and 5 m to the north of its line, broadside to it (heading north). wake_push as given.
std::string two_boats(bool wake_push) {
  return std::string(R"(
scene:
  physics: {}
  nodes:
    - node: terrain
      source:
        heightfield:
          size: 300
          resolution: 151
          layers:
            - { kind: dome, amplitude: -6, radius: 40 }
      body: static
    - node: water
      type: water
      ground: terrain
      level: -1
      wind_speed: 0.5
      wave_amp: 0.1
      wake_push: )") +
         (wake_push ? "1" : "0") + R"(
      body: {}
    - node: runner
      type: boat
      water: water
      preset: launch
      transform: { position: [-28, 0, 0] }
    - node: moored
      type: boat
      water: water
      preset: launch
      transform: { position: [-6, 5, 0], rotation: [0, 0, 90] }
)";
}

// The water node's field's wake, from the world's fields (an invalid id without one).
cvc::physics::field_id wake_field(const SceneWorld &w) {
  for (const cvc::physics::field_snapshot &f : w.world().snapshot().fields)
    if (f.desc.wake.on())
      return f.id;
  return {};
}

} // namespace

// The runner, at full throttle, passes the moored launch 5 m off. With wake_push its wake rocks
// and pushes the moored one (the physics' wake has both boats as its sources); without it the
// moored launch sits as it would alone, since the runner's wake was only ever drawn.
TEST(AriadneBoatNode, WakePushLetsOneBoatsWakeMoveAnother) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  struct sample {
    cvc::physics::vec3 p;
    double tilt = 0.0;
  };
  struct result {
    std::vector<sample> moored;
    cvc::physics::vec3 runner;
    std::size_t sources = 0;
    bool wake = false;
  };
  const auto run = [&](bool push) {
    result out;
    LoadResult r = fresh_load(two_boats(push));
    EXPECT_TRUE(r.ok) << r.error;
    cvc::app app;
    std::vector<std::string> warnings;
    std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
    if (!w) {
      ADD_FAILURE() << "no world";
      return out;
    }
    w->sync(&warnings);
    EXPECT_TRUE(warnings.empty()) << warnings.front();
    BoatRuntime *runner = w->boat("runner"), *moored = w->boat("moored");
    if (!runner || !moored || !runner->placed() || !moored->placed()) {
      ADD_FAILURE() << "the boats are not afloat";
      return out;
    }
    cvc::state::instance(app)("runner.throttle").value(1.0);
    SceneClockDriver clock(app, resolved_clock(r.scene));
    for (int i = 0; i < 60 * 18; ++i) {
      clock.drive(1.0 / 60.0);
      w->follow(app.world_clock());
      const cvc::physics::boat_state s = moored->state();
      const cvc::physics::quat &q = s.body.rotation;
      out.moored.push_back(
          {s.body.position, std::acos(std::min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))});
    }
    out.runner = runner->state().body.position;
    const cvc::physics::field_id f = wake_field(*w);
    out.wake = f.valid();
    if (f.valid())
      out.sources = w->world().field_wake_info(f).sources.size();
    return out;
  };
  const result off = run(false), on = run(true);
  ASSERT_EQ(off.moored.size(), on.moored.size());
  ASSERT_FALSE(off.moored.empty());
  EXPECT_FALSE(off.wake);
  EXPECT_TRUE(on.wake);
  EXPECT_EQ(on.sources, 2u) << "both boats make waves";
  // How far each run's moored launch strays from where it was after the first second, and how far
  // the two runs' moored launches are apart.
  const auto stray = [](const result &res, double &heave, double &drift, double &tilt) {
    const cvc::physics::vec3 p0 = res.moored[60].p;
    const double tilt0 = res.moored[60].tilt;
    heave = drift = tilt = 0.0;
    for (std::size_t i = 60; i < res.moored.size(); ++i) {
      const sample &s = res.moored[i];
      heave = std::max(heave, std::fabs(s.p.z - p0.z));
      drift = std::max(drift, std::hypot(s.p.x - p0.x, s.p.y - p0.y));
      tilt = std::max(tilt, std::fabs(s.tilt - tilt0));
    }
  };
  double off_heave, off_drift, off_tilt, on_heave, on_drift, on_tilt;
  stray(off, off_heave, off_drift, off_tilt);
  stray(on, on_heave, on_drift, on_tilt);
  double apart = 0.0;
  for (std::size_t i = 0; i < on.moored.size(); ++i) {
    const cvc::physics::vec3 a = on.moored[i].p, b = off.moored[i].p;
    apart = std::max(apart, std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                                      (a.z - b.z) * (a.z - b.z)));
  }
  std::printf("[wake push] the runner ends at (%.1f, %.1f); the moored launch over 17 s: without "
              "wake_push heave %.4f m, drift %.4f m, tilt %.4f rad; with it heave %.4f m, drift "
              "%.4f m, tilt %.4f rad; the two runs %.4f m apart at most\n",
              on.runner.x, on.runner.y, off_heave, off_drift, off_tilt, on_heave, on_drift, on_tilt,
              apart);
  EXPECT_GT(on.runner.x, -6.0) << "the runner went past the moored launch";
  EXPECT_LT(off_heave, 0.01);
  EXPECT_LT(off_tilt, 0.01);
  EXPECT_GT(on_heave, 3.0 * off_heave + 0.01) << "its wake heaves the moored launch";
  EXPECT_GT(on_tilt, 3.0 * off_tilt + 0.01) << "and rolls it";
  EXPECT_GT(apart, 0.02);
}

// The wake is stepped inside the physics step on the world clock's ticks, so it is the same
// whatever the frames: one run takes a frame per clock quantum, another frames of 0.137 s, which
// the clock caps at 8 quanta each (world_clock::max_steps_per_advance), and wherever the second is
// looked at, the world (its bodies, the boats and the wake's waves, all in its state hash) is the
// first's. TheWakesPushIsTheSameAtUnevenFrames, below, takes frames of uneven length.
TEST(AriadneBoatNode, TheWakesPushIsTheSameAtAnyFramePacing) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  struct result {
    std::map<std::uint64_t, std::uint64_t> hashes; // tick -> state hash
    std::size_t sources = 0;
  };
  const auto run = [&](double frame_dt, double seconds, int every) {
    result out;
    LoadResult r = fresh_load(two_boats(true));
    EXPECT_TRUE(r.ok) << r.error;
    cvc::app app;
    std::vector<std::string> warnings;
    std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
    if (!w) {
      ADD_FAILURE() << "no world";
      return out;
    }
    w->sync(&warnings);
    cvc::state::instance(app)("runner.throttle").value(1.0);
    cvc::state::instance(app)("runner.rudder").value(0.3);
    SceneClockDriver clock(app, resolved_clock(r.scene));
    int frame = 0;
    while (app.world_clock().t() < seconds) {
      clock.drive(frame_dt);
      w->follow(app.world_clock());
      if (++frame % every == 0)
        out.hashes[w->world().tick()] = w->world().state_hash();
    }
    const cvc::physics::field_id f = wake_field(*w);
    if (f.valid())
      out.sources = w->world().field_wake_info(f).sources.size();
    return out;
  };
  cvc::app probe;
  const result fine = run(probe.world_clock().fixed_dt(), 12.0, 1);
  const result coarse = run(0.137, 12.0, 1);
  EXPECT_EQ(fine.sources, 2u);
  EXPECT_EQ(coarse.sources, 2u);
  int compared = 0;
  for (const auto &[tick, hash] : coarse.hashes) {
    const auto it = fine.hashes.find(tick);
    if (it == fine.hashes.end())
      continue;
    ++compared;
    EXPECT_EQ(hash, it->second) << "at tick " << tick;
  }
  std::printf("[wake push] %d ticks compared between frames of a quantum and of 0.137 s "
              "(8 quanta)\n",
              compared);
  EXPECT_GT(compared, 5);
}

// The wake at uneven frame pacing that the clock does not cap (it takes at most 8 quanta a frame):
// frames of 1, 3, 2, 5 and 7 quanta in turn, none a multiple of the waves' 4-step period (1/30 s at
// the clock's 120 Hz), against a frame per quantum, have the same state hash at every tick they
// share.
TEST(AriadneBoatNode, TheWakesPushIsTheSameAtUnevenFrames) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const auto run = [&](const std::vector<int> &frames, double seconds) {
    std::map<std::uint64_t, std::uint64_t> hashes;
    LoadResult r = fresh_load(two_boats(true));
    EXPECT_TRUE(r.ok) << r.error;
    cvc::app app;
    std::vector<std::string> warnings;
    std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
    if (!w) {
      ADD_FAILURE() << "no world";
      return hashes;
    }
    w->sync(&warnings);
    cvc::state::instance(app)("runner.throttle").value(1.0);
    cvc::state::instance(app)("runner.rudder").value(0.3);
    SceneClockDriver clock(app, resolved_clock(r.scene));
    const double quantum = app.world_clock().fixed_dt();
    for (std::size_t f = 0; app.world_clock().t() < seconds; ++f) {
      clock.drive(frames[f % frames.size()] * quantum);
      w->follow(app.world_clock());
      hashes[w->world().tick()] = w->world().state_hash();
    }
    return hashes;
  };
  const auto fine = run({1}, 10.0);
  const auto uneven = run({1, 3, 2, 5, 7}, 10.0);
  int compared = 0;
  std::set<std::uint64_t> gaps;
  std::uint64_t last = 0;
  for (const auto &[tick, hash] : uneven) {
    if (tick > last) // a frame that ran no step (the first, the anchor) is not a gap
      gaps.insert(tick - last);
    last = tick;
    const auto it = fine.find(tick);
    if (it == fine.end())
      continue;
    ++compared;
    EXPECT_EQ(hash, it->second) << "at tick " << tick;
  }
  std::string seen;
  for (const std::uint64_t g : gaps)
    seen += " " + std::to_string(g);
  std::printf("[wake push] %d ticks compared; the uneven frames advanced by%s ticks\n", compared,
              seen.c_str());
  EXPECT_GT(gaps.size(), 3u) << "the frames were uneven";
  EXPECT_GT(compared, 100);
}

// A new keyframe of the sea (the wind changed) sets the water's field in place when it has a wake:
// the field keeps its id and the wake its waves, read the same just before and just after the
// swap.
TEST(AriadneBoatNode, TheWakesWavesSurviveANewSea) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = fresh_load(two_boats(true));
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  cvc::state &st = cvc::state::instance(app);
  st("runner.throttle").value(1.0);
  SceneClockDriver clock(app, resolved_clock(r.scene));
  for (int i = 0; i < 60 * 6; ++i) { // the runner under way, its wake in the window
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
  }
  const cvc::physics::field_id id = wake_field(*w);
  ASSERT_TRUE(id.valid());
  const auto surface_of = [&]() -> const void * {
    for (const cvc::physics::field_snapshot &f : w->world().snapshot().fields)
      if (f.id == id)
        return f.desc.surface.get();
    return nullptr;
  };
  // The wind rises, the sea follows at once (no lag, a short crossfade): a new keyframe.
  st("water.sea_lag").value(0.0);
  st("water.sea_fade").value(0.5);
  st("water.wind_speed").value(8.0);
  int swaps = 0;
  for (int i = 0; i < 60 * 4 && swaps == 0; ++i) {
    clock.drive(1.0 / 60.0);
    const void *before = surface_of();
    const cvc::physics::wake_info info0 = w->world().field_wake_info(id);
    const cvc::physics::vec3 p = w->boat("runner")->state().body.position;
    const double h0 = w->world().field_wake_height(id, p.x - 6.0, p.y);
    w->sync(&warnings); // the swap, if the sea took a keyframe, before any step
    if (surface_of() != before) {
      ++swaps;
      const cvc::physics::wake_info info1 = w->world().field_wake_info(id);
      EXPECT_TRUE(wake_field(*w) == id) << "the same field, set in place";
      EXPECT_EQ(info1.windows, info0.windows);
      EXPECT_EQ(info1.sources, info0.sources);
      EXPECT_EQ(w->world().field_wake_height(id, p.x - 6.0, p.y), h0);
      std::printf("[wake push] a new sea at %.2f s: the wake's %d states and its height %.4f m "
                  "behind the runner kept\n",
                  app.world_clock().t(), info1.windows, h0);
      EXPECT_NE(h0, 0.0);
    }
    w->follow(app.world_clock());
  }
  EXPECT_EQ(swaps, 1) << "the wind change made a new sea";
}

// ---------------------------------------------------------------------------------------------
// The scene's one wind (scene_wind.h, O14): `scene: wind:` seeds wind.speed / wind.dir /
// wind.gustiness, and the physics pushes every body in the air with it.

namespace {

// kPlants with a 30 m/s wind toward +y (dir 90): a gale across the yard.
LoadResult load_plants_in_wind(bool wind) {
  std::string doc = kPlants;
  if (wind) {
    const std::string at = "scene:\n  physics: {}\n";
    const std::size_t p = doc.find(at);
    EXPECT_NE(p, std::string::npos);
    doc.insert(p + at.size(), "  wind: { speed: 30, dir: 90, gustiness: 0 }\n");
  }
  cvc::app app;
  register_world_block(app);
  LoadResult r = load_string(app, doc);
  if (r.ok)
    if (const Value *world = find_world_block(r)) {
      world_rules rules = default_world_rules();
      EXPECT_EQ(parse_world_block(*world, rules), "");
      r.scene.vehicles = rules.vehicles;
    }
  return r;
}

} // namespace

// Every plant feels the wind: a jolt vehicle through its aero drag (physics reads the wind field),
// the dynamic models and the kinematic bicycle through their drive (WorldVehicles::set_wind). Each
// one's push points downwind, and each one's path over 8 s bends from the one it takes in still
// air (its navigator steering back against the push: the four-wheeler, heaviest for its area,
// least).
TEST(AriadneWorldVehicles, TheSceneWindPushesEveryPlant) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const std::array<std::string, 4> kinds{"jolt", "bike", "four", "walker"};
  struct run {
    cvc::app app;
    std::unique_ptr<SceneWorld> w;
    std::array<std::uint64_t, 4> ids{};
  };
  auto start = [&](run &r, bool wind) {
    LoadResult doc = load_plants_in_wind(wind);
    ASSERT_TRUE(doc.ok) << doc.error;
    EXPECT_EQ(doc.scene.wind.present, wind);
    std::vector<std::string> warnings;
    r.w = SceneWorld::from_scene(r.app, doc.scene, "", "physics", &warnings);
    ASSERT_NE(r.w, nullptr);
    r.w->sync(&warnings);
    EXPECT_EQ(r.w->wind_field().valid(), wind) << "the wind is a flow field in the world";
    for (std::size_t k = 0; k < 4; ++k)
      r.ids[k] = spawn_one(*r.w->vehicles(), kinds[k], "nav@1", -50.0 + 25.0 * k, 40.0, -1.5);
    SceneClockDriver clock(r.app, resolved_clock(doc.scene));
    drive(r.app, *r.w, clock, 480); // 8 s
  };
  run still, blown;
  start(still, false);
  start(blown, true);
  for (std::size_t k = 0; k < 4; ++k) {
    const entity_info a = entity_of(*still.w->vehicles(), still.ids[k]);
    const entity_info b = entity_of(*blown.w->vehicles(), blown.ids[k]);
    const double bend = std::hypot(b.x - a.x, b.y - a.y);
    std::printf("[wind] %s (%s): the wind's push (%.0f, %.0f) N; its path bent %.2f m in 8 s\n",
                kinds[k].c_str(), b.plant.c_str(), b.wind_fx, b.wind_fy, bend);
    EXPECT_EQ(a.wind_fx, 0.0) << kinds[k] << ": no push in still air";
    EXPECT_EQ(a.wind_fy, 0.0) << kinds[k];
    EXPECT_GT(b.wind_fy, 100.0) << kinds[k] << ": pushed downwind (+y)";
    EXPECT_GT(bend, 0.05) << kinds[k] << ": its path bends in the wind";
  }
}

// A body in the air is carried downwind, and turns with wind.dir: a light ball (1 kg, 1 m across, a
// beach ball that falls at some 4 m/s) dropped from 40 m rides the wind as it falls; the water
// follows the same wind (its own keys left alone).
TEST(AriadneSceneWorld, TheSceneWindCarriesWhatFallsAndTurnsWithItsKey) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(R"(
scene:
  physics: {}
  wind: { speed: 12, dir: 0, gustiness: 0 }
  nodes:
    - node: floor
      source: { box: { size: [400, 400, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - node: ball
      source: { sphere: { radius: 0.5 } }
      transform: { position: [0, 0, 40] }
      body: { mass: 1 }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_TRUE(r.scene.wind.present);
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(val(app, "wind.speed"), "12");
  if (!jolt())
    return;
  ASSERT_TRUE(w->wind_field().valid());
  SceneClockDriver clock(app, resolved_clock(r.scene));
  for (int i = 0; i < 240; ++i) { // 4 s: it falls, carried downwind
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
  }
  const cvc::physics::body_state carried = w->world().state(w->body("ball"));
  std::printf("[wind] a 1 kg ball dropped from 40 m in a 12 m/s wind, 4 s on: (%.2f, %.2f, %.2f)\n",
              carried.position.x, carried.position.y, carried.position.z);
  EXPECT_GT(carried.position.x, 1.0) << "carried downwind (+x)";
  EXPECT_NEAR(carried.position.y, 0.0, 0.05);
  // One key turns it: toward +y now; the ball, lifted back up, is carried that way.
  cvc::state::instance(app)("wind.dir").value(90.0);
  cvc::physics::body_state up = carried;
  up.position = {0.0, 0.0, 40.0};
  up.linear_velocity = {0.0, 0.0, 0.0};
  w->world().set_state(w->body("ball"), up);
  for (int i = 0; i < 240; ++i) {
    clock.drive(1.0 / 60.0);
    w->follow(app.world_clock());
  }
  const cvc::physics::body_state turned = w->world().state(w->body("ball"));
  std::printf("[wind] after wind.dir 90, 4 s on: (%.2f, %.2f, %.2f)\n", turned.position.x,
              turned.position.y, turned.position.z);
  EXPECT_GT(turned.position.y, 1.0) << "carried toward +y";
  // No wind: the field leaves the world.
  cvc::state::instance(app)("wind.speed").value(std::string());
  w->sync(&warnings);
  EXPECT_FALSE(w->wind_field().valid());
}

// The water follows the scene's wind (scene_water_spec through scene_wind): its spectrum's wind is
// the scene's, its own water.wind_* stay as they were, and with wind.speed cleared it takes them
// again.
TEST(AriadneSceneWorld, TheWaterFollowsTheScenesWind) {
  REQUIRE_YAML();
  LoadResult r = fresh_load(R"(
scene:
  wind: { speed: 11, dir: 45, gustiness: 0.4 }
  nodes:
    - node: water
      type: water
      ground: beach
      level: 0
      wind_speed: 4
      wind_dir: 200
      sea_lag: 0
      sea_fade: 0
)");
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  seed_scene_wind(app, r.scene, "");
  const SceneNode &n = r.scene.nodes.front();
  SceneWaterSpec s = scene_water_spec(app, n, "");
  EXPECT_EQ(s.wind_speed, 11.0);
  EXPECT_EQ(s.wind_dir, 45.0);
  EXPECT_EQ(s.gustiness, 0.4);
  EXPECT_EQ(val(app, "water.wind_speed"), "4") << "its own key stays";
  const int N = 101;
  auto flat = std::make_shared<cvc::height_field>(N, N, -100.0, -100.0, 2.0, 2.0,
                                                  std::vector<float>(std::size_t(N) * N, -10.0f));
  WaterRuntime rt(app, n, "", [flat] { return flat; });
  ASSERT_NE(rt.keyframes(), nullptr);
  EXPECT_NEAR(rt.keyframes()->to.dir_rad, 45.0 * 3.14159265358979323846 / 180.0, 1e-12);
  cvc::state::instance(app)("wind.dir").value(-30.0);
  EXPECT_TRUE(rt.tick());
  EXPECT_NEAR(rt.keyframes()->to.dir_rad, -30.0 * 3.14159265358979323846 / 180.0, 1e-12)
      << "one key turns the sea";
  cvc::state::instance(app)("wind.speed").value(std::string());
  s = scene_water_spec(app, n, "");
  EXPECT_EQ(s.wind_speed, 4.0);
  EXPECT_EQ(s.wind_dir, 200.0);
}

// The gym (ARIADNE_WORLDS.md section 14, L4): a world document driven by reset, step and observe.

namespace {

const char *kGymVehicles = R"(
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee, controllers: [nav@1, none] }
)";

const char *kGymSquare = R"(
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [120, 120, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)";

// Two spawn points facing +x, and a wall 30 m ahead of the first.
const char *kGymSpawns = R"(  spawns:
    policy: round_robin
    scope: all
    points:
      - { at: [0, 0], heading: 0 }
      - { at: [0, 30], heading: 0 }
)";

const char *kGymWall = R"(    - node: wall
      source: { box: { size: [2, 80, 4] } }
      transform: { position: [30, 0, 2] }
      body: static
)";

std::string gym_doc(bool spawns, bool wall) {
  return std::string("requires: [scene.physics]\n") + kGymVehicles + (spawns ? kGymSpawns : "") +
         kGymSquare + (wall ? kGymWall : "");
}

gym_options agents(int n) {
  gym_options o;
  o.count = n;
  return o;
}

const double kNaN = std::numeric_limits<double>::quiet_NaN();

// A dynamic plant's ground on a flat world, as a twin of it samples it: its grip (a double, as
// the plant holds it) and no height.
struct flat_twin_ground {
  double mu = 0.9;
  template <class S> void at(S, S, S &m, S &h) const {
    m = S(mu);
    h = S(0);
  }
};

} // namespace

TEST(AriadneWorldGym, ResetStepAndObserve) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  WorldGym gym(write_doc("gym", gym_doc(true, true)).string(), agents(2));
  EXPECT_NEAR(gym.dt(), cvc::kSimTick, 1e-12);                      // libcvc's one simulation tick
  EXPECT_THROW(gym.step({0.0, 0.0}, {0.0, 0.0}), std::logic_error); // no episode yet

  const gym_observation o = gym.reset(1);
  ASSERT_EQ(o.n, 2u);
  EXPECT_EQ(gym.agents(), 2u);
  EXPECT_EQ(o.episode, 1u);
  EXPECT_EQ(o.step, 0u);
  // At the spawn points, round-robin, facing +x (the chassis's centre ahead of the point, which is
  // the rear axle's); the body and envelope of a Humvee.
  EXPECT_NEAR(o.x[0], 0.0, 2.5);
  EXPECT_NEAR(o.y[0], 0.0, 1.0);
  EXPECT_NEAR(o.y[1], 30.0, 1.0);
  EXPECT_NEAR(o.yaw[0], 0.0, 0.05);
  EXPECT_GT(o.length[0], 4.0);
  EXPECT_GT(o.max_steer[0], 0.1);
  EXPECT_GT(o.max_speed[0], 1.0);
  // The rays: 16 around the heading; the one straight ahead (k = 8) meets the grid's edge ~60 m on.
  ASSERT_EQ(o.ray_angles.size(), 16u);
  ASSERT_EQ(o.ranges.size(), 32u);
  EXPECT_NEAR(o.ray_angles[8], 0.0, 1e-12);
  EXPECT_GT(o.ranges[8], 50.0f);
  EXPECT_LE(o.ranges[8], 60.0f);
  EXPECT_EQ(val(gym.app(), "gym.episode"), "1");

  EXPECT_THROW(gym.step({0.0}, {0.0}), std::invalid_argument); // one value per agent

  // A step is one navigator tick of physics steps on the paused clock.
  const std::uint64_t t0 = gym.world().world().tick();
  gym_step s = gym.step({0.0, 0.0}, {2.0, 2.0});
  EXPECT_EQ(gym.world().world().tick() - t0,
            static_cast<std::uint64_t>(gym.vehicles().steps_per_tick()));
  EXPECT_EQ(s.observation.step, 1u);
  EXPECT_NEAR(s.observation.time, gym.dt(), 1e-12);
  ASSERT_EQ(s.reward.size(), 2u);
  // Straight ahead under throttle: the first agent gains speed along +x; the second, left to
  // nav@1 (NaN), drives to its own goal.
  for (int i = 0; i < 60; ++i)
    s = gym.step({0.0, kNaN}, {2.0, kNaN});
  EXPECT_GT(s.observation.speed[0], 2.0);
  EXPECT_GT(s.observation.x[0], 3.0);
  EXPECT_NEAR(s.observation.y[0], 0.0, 1.5);
  EXPECT_EQ(val(gym.app(), "gym.step"), "61");
  // Nothing else steps it: the world holds still between steps.
  const std::uint64_t held = gym.world().world().tick();
  gym.runtime().drain();
  EXPECT_EQ(gym.world().world().tick(), held);
}

TEST(AriadneWorldGym, OneSeedAndOneListOfCommandsGiveOneWorld) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const std::string path = write_doc("gym_drawn", gym_doc(false, false)).string();
  const auto run = [&](WorldGym &gym, std::uint64_t seed) {
    gym.reset(seed);
    gym_step s;
    for (int i = 0; i < 40; ++i)
      s = gym.step({0.2, -0.1}, {1.5, 1.0});
    return s.observation;
  };
  WorldGym a(path, agents(2)), b(path, agents(2));
  const gym_observation a1 = run(a, 11);
  const gym_observation b1 = run(b, 11);
  for (std::size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(a1.x[i], b1.x[i]);
    EXPECT_EQ(a1.y[i], b1.y[i]);
    EXPECT_EQ(a1.yaw[i], b1.yaw[i]);
  }
  // A reset with the seed again is the episode again, in the same gym.
  const gym_observation a2 = run(a, 11);
  EXPECT_EQ(a2.x[0], a1.x[0]);
  EXPECT_EQ(a2.y[1], a1.y[1]);
  EXPECT_EQ(a2.episode, 2u);
  EXPECT_EQ(a2.id[0], a1.id[0]); // ids start again at each reset
  // Another seed: other starts.
  const gym_observation s1 = a.reset(11);
  const gym_observation s2 = a.reset(12);
  EXPECT_GT(std::hypot(s1.x[0] - s2.x[0], s1.y[0] - s2.y[0]), 1.0);
}

TEST(AriadneWorldGym, TheEpisodeEndsTheRewardIsYoursAndTheLogKeepsIt) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  gym_options o;
  o.max_steps = 5;
  WorldGym gym(write_doc("gym_end", gym_doc(true, true)).string(), o);
  gym.reset(3);
  int calls = 0;
  gym.set_reward([&](const gym_observation &before, const std::vector<double> &steer,
                     const std::vector<double> &, const gym_observation &after) {
    ++calls;
    EXPECT_EQ(after.step, before.step + 1);
    return std::vector<double>(after.n, steer[0]);
  });
  gym_step s;
  for (int i = 0; i < 5; ++i) {
    s = gym.step({0.25}, {1.0});
    EXPECT_EQ(s.truncated, i == 4);
    EXPECT_FALSE(s.terminated);
  }
  EXPECT_EQ(calls, 5);
  EXPECT_DOUBLE_EQ(s.reward[0], 0.25);
  EXPECT_EQ(val(gym.app(), "gym.done"), "1");
  EXPECT_EQ(val(gym.app(), "gym.return"), "1.25");

  // The log: a row per agent at the reset and at each step, written as CSV.
  const gym_log &log = gym.log();
  EXPECT_EQ(log.size(), 6u);
  EXPECT_EQ(log.step.front(), 0u);
  EXPECT_TRUE(std::isnan(log.steer_cmd.front()));
  EXPECT_DOUBLE_EQ(log.steer_cmd.back(), 0.25);
  const fs::path csv = fs::temp_directory_path() / "ariadne_world_gym_log.csv";
  ASSERT_TRUE(gym.save_log(csv.string()));
  {
    std::ifstream in(csv);
    std::string header, line;
    std::getline(in, header);
    EXPECT_EQ(header.rfind("episode,step,id,time,x,y", 0), 0u) << header;
    int rows = 0;
    while (std::getline(in, line))
      ++rows;
    EXPECT_EQ(rows, 6);
  }
  fs::remove(csv);
  gym.clear_log();
  EXPECT_EQ(gym.log().size(), 0u);

  // A reward of the wrong size is refused.
  gym.set_reward([](const gym_observation &, const std::vector<double> &,
                    const std::vector<double> &,
                    const gym_observation &) { return std::vector<double>{}; });
  EXPECT_THROW(gym.step({0.0}, {0.0}), std::runtime_error);
  gym.set_reward(nullptr); // the progress reward again
  EXPECT_NO_THROW(gym.step({0.0}, {0.0}));
}

TEST(AriadneWorldGym, TheNavigatorDrivesWhatTheCommandsLeaveAndAWallIsAContact) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  WorldGym gym(write_doc("gym_nav", gym_doc(true, true)).string(), agents(2));
  const gym_observation o = gym.reset(4);
  // The first agent, full throttle at the wall 30 m ahead; the second, left to nav@1, given a goal
  // ahead and to its left, past the wall's end.
  ASSERT_TRUE(gym.set_goal(o.id[1], 20.0, 50.0));
  EXPECT_DOUBLE_EQ(gym.observe().goal_x[1], 20.0); // in the observation at once
  const double d0 = std::hypot(20.0 - o.x[1], 50.0 - o.y[1]);
  int hits = 0;
  double best = d0;
  for (int i = 0; i < 240; ++i) {
    const gym_step s = gym.step({0.0, kNaN}, {3.0, kNaN});
    hits += s.observation.collided[0];
    best = std::min(best, s.observation.goal_dist[1]);
    EXPECT_DOUBLE_EQ(s.observation.goal_x[1], 20.0); // held: no wandering
  }
  EXPECT_GT(hits, 0) << "no contact with the wall";
  EXPECT_LT(gym.observe().x[0], 29.0); // stopped by it
  EXPECT_LT(best, d0 - 10.0) << "nav@1 made no way to its goal";
}

// The world has an edge (the square ends at 60 m) and nothing walls it in the physics: a vehicle
// driven off it is out of the world, and the episode ends.
TEST(AriadneWorldGym, AnAgentOffTheWorldEndsTheEpisode) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  gym_options o;
  entity_spec s;
  s.archetype = "humvee";
  s.controller = "none";
  s.placed = true;
  s.x = 40.0;
  s.y = 0.0;
  s.yaw = 0.0;
  o.agents = {s};
  WorldGym gym(write_doc("gym_edge", gym_doc(false, false)).string(), o);
  gym.reset(0);
  EXPECT_FALSE(gym.observe().out[0]);
  gym_step st;
  int steps = 0;
  for (; steps < 400; ++steps) {
    st = gym.step({0.0}, {3.0});
    if (st.terminated)
      break;
  }
  ASSERT_TRUE(st.terminated) << "never left the world: x " << st.observation.x[0];
  EXPECT_TRUE(st.observation.out[0]);
  EXPECT_GT(st.observation.x[0], 55.0);
  EXPECT_FALSE(st.truncated);
}

// Every plant runs in the scene's physics world: a Jolt vehicle, nav's bicycle as a kinematic box,
// the dynamic single-track and four-wheel models as their chassis boxes. Each drives on the
// commands, the same way twice.
TEST(AriadneWorldGym, EveryPlantDrivesInTheWorld) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  gym_options o;
  const char *plants[] = {"jolt", "kinematic", "dynamic", "dynamic4w"};
  for (int k = 0; k < 4; ++k) {
    entity_spec s;
    s.archetype = "humvee";
    s.controller = "none";
    s.plant = plants[k];
    s.placed = true;
    s.x = -30.0;
    s.y = -30.0 + 20.0 * k;
    s.yaw = 0.0;
    o.agents.push_back(s);
  }
  const std::string path = write_doc("gym_plants", gym_doc(false, false)).string();
  const auto run = [&](WorldGym &gym) {
    gym.reset(2);
    gym_step st;
    for (int i = 0; i < 60; ++i)
      st = gym.step({0.0, 0.0, 0.0, 0.0}, {2.0, 2.0, 2.0, 2.0});
    return st.observation;
  };
  WorldGym a(path, o), b(path, o);
  const gym_observation oa = run(a), ob = run(b);
  ASSERT_EQ(oa.n, 4u);
  EXPECT_EQ(a.world().world().vehicle_count(), 1u); // the Jolt one; the others are boxes there
  for (int k = 0; k < 4; ++k) {
    EXPECT_GT(oa.x[k], -30.0 + 3.0) << plants[k] << " drove ahead";
    EXPECT_NEAR(oa.y[k], -30.0 + 20.0 * k, 1.5) << plants[k] << " kept straight";
    EXPECT_GT(oa.speed[k], 1.0) << plants[k];
    EXPECT_FALSE(oa.collided[k]) << plants[k];
    EXPECT_EQ(oa.x[k], ob.x[k]) << plants[k] << ": the same twice";
    EXPECT_EQ(oa.y[k], ob.y[k]) << plants[k];
  }
}

// A world with vehicles runs on the simulation hosts' clock: the 1/120 s quantum, physics at
// 120 Hz, real time catching up 0.25 s a frame. The gym's tick is libcvc's one simulation tick,
// seven physics steps; gym_options::tick sets another, held to whole physics steps.
TEST(AriadneWorldGym, TheTickIsLibcvcsOneSimulationTick) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const std::string path = write_doc("gym_tick", gym_doc(false, false)).string();
  WorldGym gym(path, agents(1));
  EXPECT_EQ(gym.app().world_clock().fixed_dt(), cvc::kSimQuantum);
  EXPECT_EQ(gym.app().world_clock().current_config().max_steps_per_advance, 30);
  EXPECT_TRUE(gym.app().world_clock().current_config().cap_scales_with_rate);
  EXPECT_EQ(gym.world().world().config().fixed_dt, cvc::kSimQuantum);
  const gym_observation o0 = gym.reset(1);
  EXPECT_NEAR(gym.dt(), cvc::kSimTick, 1e-12);
  EXPECT_EQ(gym.vehicles().steps_per_tick(), cvc::kSimTickQuanta);
  const gym_step st = gym.step({kNaN}, {kNaN});
  EXPECT_NEAR(st.observation.time - o0.time, cvc::kSimTick, 1e-9);

  gym_options o = agents(1);
  o.tick = 0.05;
  WorldGym five(path, o);
  five.reset(1);
  EXPECT_NEAR(five.dt(), 0.05, 1e-12);
  EXPECT_EQ(five.vehicles().steps_per_tick(), 6);

  o.tick = 0.055; // not whole physics steps: held to the nearest, and the status says so
  WorldGym odd(path, o);
  odd.reset(1);
  EXPECT_EQ(odd.vehicles().steps_per_tick(),
            static_cast<int>(std::lround(0.055 / cvc::kSimQuantum)));
  EXPECT_NE(odd.vehicles().status().find("not a whole number"), std::string::npos)
      << odd.vehicles().status();
  EXPECT_THROW(gym.vehicles().set_tick(0.0), std::invalid_argument);
}

// What a differentiable twin of the world reads: the navigator's own grid, its signed distance and
// gradient, the ground's grip, and the navigator's settings.
TEST(AriadneWorldGym, TheFieldsAreTheNavigatorsGrid) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  WorldGym gym(write_doc("gym_fields", gym_doc(false, true)).string(), agents(1));
  EXPECT_THROW(gym.fields(), std::logic_error); // no episode yet
  EXPECT_THROW(gym.plants(), std::logic_error);
  gym.reset(3);
  const gym_fields f = gym.fields();
  const std::shared_ptr<const cvc::nav::world_grid> g = gym.vehicles().grid();
  ASSERT_TRUE(g);
  ASSERT_EQ(f.rows, g->rows);
  ASSERT_EQ(f.cols, g->cols);
  EXPECT_EQ(f.cell, g->cell());
  EXPECT_EQ(f.min_x, g->min_x);
  EXPECT_EQ(f.min_y, g->min_y);
  const std::size_t cells = static_cast<std::size_t>(f.rows) * static_cast<std::size_t>(f.cols);
  ASSERT_EQ(f.occupancy, g->occupancy);
  ASSERT_EQ(f.height.size(), cells);
  ASSERT_EQ(f.mu.size(), cells);
  EXPECT_EQ(f.mu.front(), 0.9);
  // The navigator's own signed distance, bit for bit: + free, - blocked.
  ASSERT_EQ(f.sdf, cvc::nav::signed_distance(*g));
  ASSERT_EQ(f.sdf_dx.size(), cells);
  ASSERT_EQ(f.sdf_dy.size(), cells);
  std::size_t free = 0, blocked = 0;
  for (std::size_t k = 0; k < cells; ++k) {
    if (f.occupancy[k]) {
      ++blocked;
      EXPECT_LE(f.sdf[k], 0.0f) << k;
    } else {
      ++free;
      EXPECT_GT(f.sdf[k], 0.0f) << k;
    }
  }
  EXPECT_GT(free, 0u);
  EXPECT_GT(blocked, 0u);
  // The gradient points away from the square's walls: east of the west wall, north of the south.
  const int r = f.rows / 2, c = f.cols / 2;
  const auto at = [&](const std::vector<float> &v, int rr, int cc) {
    return v[static_cast<std::size_t>(rr) * static_cast<std::size_t>(f.cols) +
             static_cast<std::size_t>(cc)];
  };
  EXPECT_GT(at(f.sdf_dx, r, 3), 0.5f);
  EXPECT_LT(at(f.sdf_dx, r, f.cols - 4), -0.5f);
  EXPECT_GT(at(f.sdf_dy, 3, c), 0.5f);
  EXPECT_LT(at(f.sdf_dy, f.rows - 4, c), -0.5f);
  // The navigator: nav's normalized units per metre, its tick, the arrival radius.
  EXPECT_TRUE(f.navigator.ready);
  EXPECT_EQ(f.navigator.scale, 0.05);
  EXPECT_NEAR(f.navigator.tick, gym.dt(), 1e-6);
  EXPECT_GT(f.navigator.reach, 0.0);
  EXPECT_GT(f.navigator.vmax, 0.0f);
}

// Each agent's plant as a model: on dynamic4w the plant's own four-wheel model, its parameters and
// state, and what it took of the command in each substep, so that replaying the model from the
// state before a tick gives the state after it, to the last bit. The other plants give theirs.
TEST(AriadneWorldGym, ThePlantsAreModelsATwinReplays) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  gym_options o;
  const char *plants[] = {"jolt", "kinematic", "dynamic", "dynamic4w"};
  const char *models[] = {"dyn4", "bike", "dyn_bike", "dyn4"};
  for (int k = 0; k < 4; ++k) {
    entity_spec sp;
    sp.archetype = "humvee";
    sp.controller = "none";
    sp.plant = plants[k];
    sp.placed = true;
    sp.x = -30.0;
    sp.y = -30.0 + 20.0 * k;
    sp.yaw = 0.0;
    o.agents.push_back(sp);
  }
  WorldGym gym(write_doc("gym_twin", gym_doc(false, false)).string(), o);
  gym.reset(4);
  gym_step st;
  for (int i = 0; i < 20; ++i)
    st = gym.step({0.05, 0.05, 0.05, 0.05}, {2.0, 2.0, 2.0, 2.0});
  const std::vector<entity_plant> before = gym.plants();
  st = gym.step({0.08, 0.08, 0.08, 0.08}, {1.5, 1.5, 1.5, 1.5});
  const std::vector<entity_plant> after = gym.plants();
  ASSERT_EQ(after.size(), 4u);
  for (int k = 0; k < 4; ++k) {
    const entity_plant &p = after[static_cast<std::size_t>(k)];
    EXPECT_EQ(p.id, st.observation.id[static_cast<std::size_t>(k)]) << plants[k];
    EXPECT_EQ(p.plant, plants[k]);
    EXPECT_TRUE(p.placed) << plants[k];
    EXPECT_EQ(p.model, models[k]) << plants[k];
    EXPECT_EQ(p.exact, k != 0) << plants[k];
    EXPECT_GE(p.substeps, 1) << plants[k];
    EXPECT_GT(p.limits.hdt, 0.0f) << plants[k];
  }
  // Jolt: the four-wheel model's numbers for a Humvee, its heading and speed from the chassis.
  const entity_plant &j = after[0];
  EXPECT_GT(j.dyn4.m, 1000.0f);
  EXPECT_GT(j.dyn4.lf + j.dyn4.lr, 2.5f);
  EXPECT_NEAR(j.dyn4_state.psi, st.observation.yaw[0], 1e-9);
  EXPECT_NEAR(j.dyn4_state.vx, st.observation.speed[0], 0.5);
  EXPECT_TRUE(j.applied_accel.empty());
  // nav's bicycle: the rear axle and heading the navigator holds, in metres.
  const entity_plant &b = after[1];
  EXPECT_NEAR(b.bike_state.psi, st.observation.yaw[1], 1e-6);
  EXPECT_NEAR(b.bike_state.vx, st.observation.speed[1], 1e-4);
  EXPECT_NEAR(b.bike_state.x, st.observation.x[1] + b.rear_x * std::cos(b.bike_state.psi), 1e-3);
  // The dynamic single-track model: its parameters and a value per model step.
  const entity_plant &d = after[2];
  EXPECT_GT(d.bike.m, 1000.0f);
  EXPECT_EQ(d.applied_accel.size(), d.applied_grade.size());
  EXPECT_EQ(static_cast<int>(d.applied_accel.size()), d.substeps);
  // The four-wheel model: replayed from the state before the tick with the command it took.
  const entity_plant &w0 = before[3], &w = after[3];
  ASSERT_EQ(static_cast<int>(w.applied_accel.size()), w.substeps);
  EXPECT_EQ(w.collisions_tick, 0);
  EXPECT_NEAR(w.steer_command, 0.08, 1e-6);
  EXPECT_GT(w.accel_command, 0.0);
  const flat_twin_ground ground{w.mu};
  cvc::nav::diff::dyn4_state<double> s = w0.dyn4_state, next{};
  for (int k = 0; k < w.substeps; ++k) {
    cvc::nav::diff::dyn4_ph_books<double> books{};
    cvc::nav::diff::dyn4_plant_books<double>(w.dyn4, w.limits, ground, s, w.steer_command,
                                             w.applied_accel[static_cast<std::size_t>(k)], next,
                                             books);
    s = next;
  }
  for (int i = 0; i < cvc::nav::diff::dyn4_state<double>::size; ++i)
    EXPECT_EQ(s[i], w.dyn4_state[i]) << "state " << i;
  EXPECT_GT(w.dyn4_state.vx, 1.0);
}

// What a twin replays of the drive: each agent's coefficients, carrot and the external force it
// summed (an intent's), reported after a tick; capturing it changes nothing the drive does.
TEST(AriadneWorldGym, TheDriveATwinReplaysAndTheForceItSummed) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  gym_options o;
  for (int k = 0; k < 2; ++k) {
    entity_spec sp;
    sp.archetype = "humvee";
    sp.controller = "nav@1";
    sp.plant = "dynamic4w";
    sp.placed = true;
    sp.x = -30.0;
    sp.y = -10.0 + 20.0 * k;
    sp.yaw = 0.0;
    o.agents.push_back(sp);
  }
  const std::string path = write_doc("gym_drive", gym_doc(false, false)).string();
  gym_options off = o;
  off.drive_telemetry = false;
  WorldGym a(path, o), b(path, off);
  const std::vector<double> nan(2, std::numeric_limits<double>::quiet_NaN());
  for (WorldGym *g : {&a, &b}) {
    g->reset(9);
    EXPECT_FALSE(g->plants()[0].drive.known);
    entity_intent in;
    in.force_x = 0.5;
    in.force_y = 1.5;
    ASSERT_TRUE(g->vehicles().set_intent(g->observe().id[0], in));
  }
  EXPECT_TRUE(a.vehicles().drive_telemetry());
  EXPECT_FALSE(b.vehicles().drive_telemetry());
  for (int i = 0; i < 30; ++i) {
    const gym_step sa = a.step(nan, nan), sb = b.step(nan, nan);
    for (std::size_t k = 0; k < 2; ++k) { // the same drive, captured or not
      EXPECT_EQ(sa.observation.x[k], sb.observation.x[k]) << "tick " << i;
      EXPECT_EQ(sa.observation.y[k], sb.observation.y[k]) << "tick " << i;
    }
  }
  const std::vector<entity_plant> pa = a.plants(), pb = b.plants();
  ASSERT_EQ(pa.size(), 2u);
  EXPECT_FALSE(pb[0].drive.known);
  for (const entity_plant &p : pa) {
    EXPECT_TRUE(p.drive.known);
    EXPECT_GT(p.drive.beta, 0.0);
    EXPECT_GT(p.drive.gamma, 0.0);
    EXPECT_TRUE(std::isfinite(p.drive.goal_x) && std::isfinite(p.drive.goal_y));
  }
  EXPECT_NEAR(pa[0].drive.force_x, 0.5, 1e-6);
  EXPECT_NEAR(pa[0].drive.force_y, 1.5, 1e-6);
  EXPECT_EQ(pa[1].drive.force_x, 0.0);
  EXPECT_EQ(pa[1].drive.force_y, 0.0);
  // The carrot is ahead of the agent, toward its goal.
  const gym_observation &ob = a.observe();
  EXPECT_LT(std::hypot(pa[1].drive.goal_x - ob.goal_x[1], pa[1].drive.goal_y - ob.goal_y[1]),
            std::hypot(ob.x[1] - ob.goal_x[1], ob.y[1] - ob.goal_y[1]));
}

// The limits the drive planned each command with on the plant (plants()' plan): the vehicle's
// own, as nav_plant_options sizes them from its body; and the park brake the plan announces.
TEST(AriadneWorldGym, ThePlanTheDriveCommandedThePlantWith) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  gym_options o;
  for (const char *plant : {"dynamic4w", "kinematic"}) {
    entity_spec sp;
    sp.archetype = "humvee";
    sp.controller = "nav@1";
    sp.plant = plant;
    sp.placed = true;
    sp.x = -30.0;
    sp.y = -10.0 + 20.0 * static_cast<double>(o.agents.size());
    sp.yaw = 0.0;
    o.agents.push_back(sp);
  }
  WorldGym g(write_doc("gym_plan", gym_doc(false, false)).string(), o);
  const gym_observation o0 = g.reset(3);
  ASSERT_TRUE(g.set_goal(o0.id[0], o0.x[0] + 25.0, o0.y[0]));
  std::vector<entity_plant> ps = g.plants();
  ASSERT_EQ(ps.size(), 2u);
  EXPECT_FALSE(ps[1].plan.known); // nav's bicycle has no plant arm
  const entity_plan &p = ps[0].plan;
  ASSERT_TRUE(p.known); // from the start
  const cvc::physics::vehicle_desc &d = ps[0].desc;
  const cvc::physics::nav_plant_options po; // the plant's defaults
  EXPECT_NEAR(p.latency, po.latency, 1e-6);
  EXPECT_NEAR(p.a_brake, po.brake_margin * d.max_decel, 1e-4);
  EXPECT_NEAR(p.a_max, po.accel_margin * d.max_accel, 1e-4);
  EXPECT_NEAR(p.vmax, d.max_speed, 1e-4);
  EXPECT_GT(p.a_lat, 0.0);
  EXPECT_LE(p.a_lat, po.lateral_g * 9.81 + 1e-3);
  EXPECT_NEAR(p.wheelbase, d.wheelbase(), 1e-5);
  EXPECT_NEAR(p.max_steer, d.max_steer, 1e-6);
  EXPECT_NEAR(p.steer_rate, d.max_steer_rate, 1e-6);
  EXPECT_TRUE(p.footprint);
  EXPECT_GT(p.body_rr, 0.5 * d.half_extents.y);
  for (std::size_t b = 1; b < p.body_offsets.size(); ++b)
    EXPECT_GT(p.body_offsets[b], p.body_offsets[b - 1]); // tail to nose
  EXPECT_FALSE(p.parked);
  EXPECT_FALSE(p.modulated);

  // Its steering now is the plant's actuator, as the model's state holds it.
  const std::vector<double> nan(2, kNaN);
  for (int i = 0; i < 5; ++i)
    g.step(nan, nan);
  ps = g.plants();
  EXPECT_DOUBLE_EQ(ps[0].plan.steer_now,
                   static_cast<double>(static_cast<float>(ps[0].dyn4_state.delta)));

  // Driven to its goal, it parks: the next tick brakes to a stop at most at a_brake.
  bool parked = false;
  for (int i = 0; i < 400 && !parked; ++i) {
    g.step(nan, nan);
    parked = g.plants()[0].plan.parked;
  }
  ASSERT_TRUE(parked);
  const double v = g.plants()[0].dyn4_state.vx, dt = g.fields().navigator.dt;
  g.step(nan, nan);
  const entity_plant q = g.plants()[0];
  EXPECT_NEAR(q.accel_command, std::clamp(-v / dt, -q.plan.a_brake, q.plan.a_max), 1e-3);
}

// nav@1 comes to rest at its goal (WorldVehicles turns on sim_world's stop_at_goal and
// park_hold): three agents sent 35-95 m, on dynamic4w and on nav's bicycle, arrive one by one, and
// the episode ends with each at rest within its reach and staying there. Without it each met its
// goal at speed and stopped past it (the bicycle drove on), so they were never all there at once.
TEST(AriadneWorldGym, NavAtOneComesToRestAtItsGoalAndTheEpisodeEnds) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  gym_options o;
  for (const char *plant : {"dynamic4w", "dynamic4w", "kinematic"}) {
    entity_spec sp;
    sp.archetype = "humvee";
    sp.controller = "nav@1";
    sp.plant = plant;
    sp.placed = true;
    sp.x = -50.0;
    sp.y = -30.0 + 30.0 * static_cast<double>(o.agents.size());
    sp.yaw = 0.0;
    o.agents.push_back(sp);
  }
  WorldGym g(write_doc("gym_arrive", gym_doc(false, false)).string(), o);
  const gym_observation o0 = g.reset(4);
  EXPECT_TRUE(g.fields().navigator.stop_at_goal);
  EXPECT_TRUE(g.fields().navigator.park_hold);
  const double gx[3] = {-15.0, 15.0, 45.0};
  for (std::size_t k = 0; k < 3; ++k)
    ASSERT_TRUE(g.set_goal(o0.id[k], gx[k], o0.y[k]));
  const std::vector<double> nan(3, kNaN);
  int t = 0;
  bool done = false;
  for (; t < 1500 && !done; ++t)
    done = g.step(nan, nan).terminated;
  ASSERT_TRUE(done) << "every agent at its goal at once";
  for (int k = 0; k < 100; ++k)
    g.step(nan, nan);
  const gym_observation &ob = g.observe();
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_TRUE(ob.reached[k]) << "agent " << k << " stays at its goal, " << ob.goal_dist[k]
                               << " m from it";
    EXPECT_LT(std::fabs(ob.speed[k]), 0.05) << "agent " << k << " at rest";
  }
}

// Vehicles on different plants meet in the one physics world. Nav's bicycle is a kinematic box,
// which no building stops: it touches what is blocked in the grid.
TEST(AriadneWorldGym, VehiclesOnOtherPlantsAndTheBicycleTouch) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  gym_options o;
  entity_spec bike;
  bike.archetype = "humvee";
  bike.controller = "none";
  bike.plant = "kinematic";
  bike.placed = true;
  bike.x = 0.0;
  bike.y = 0.0;
  bike.yaw = 0.0;
  entity_spec parked = bike;
  parked.plant = "jolt";
  parked.x = 20.0;
  o.agents = {bike, parked};
  WorldGym gym(write_doc("gym_touch", gym_doc(false, false)).string(), o);
  gym.reset(0);
  bool touched = false;
  for (int i = 0; i < 200 && !touched; ++i) {
    const gym_step st = gym.step({0.0, kNaN}, {2.0, kNaN});
    touched = st.observation.hit_vehicle[0] && st.observation.hit_vehicle[1];
  }
  EXPECT_TRUE(touched) << "the bicycle drove through the parked Humvee untouched";
  // The bicycle on to the grid's wall at the square's edge.
  gym_options edge;
  bike.y = 30.0;
  edge.agents = {bike};
  WorldGym walled(write_doc("gym_bike_wall", gym_doc(false, false)).string(), edge);
  walled.reset(0);
  bool hit = false;
  for (int i = 0; i < 400 && !hit; ++i)
    hit = walled.step({0.0}, {3.0}).observation.hit_obstacle[0];
  EXPECT_TRUE(hit) << "the bicycle met no wall: x " << walled.observe().x[0];
}

// With regenerate each reset makes the generated world from its seed: another seed, another
// world; the seed again, the world again.
TEST(AriadneWorldGym, EachEpisodeCanHaveAWorldOfItsOwn) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const std::string doc = std::string("requires: [scene.physics]\n") + kGymVehicles +
                          R"(scene:
  physics: {}
  world: { source: land, cell: 4 }
  nodes:
    - node: land
      type: generated_world
      world: land
      seed: 5
      half: 100
      trees: 6
      rocks: 3
      buildings: 8
      body: {}
)";
  gym_options o;
  o.regenerate = true;
  WorldGym gym(write_doc("gym_regen", doc).string(), o);
  const auto occupancy = [&] { return gym.vehicles().grid()->occupancy; };
  const auto heights = [&] { return gym.vehicles().grid()->height; };
  gym.reset(11);
  EXPECT_EQ(val(gym.app(), "world.seed"), "11");
  const std::vector<float> h11 = heights();
  const std::vector<std::uint8_t> o11 = occupancy();
  gym.reset(12);
  EXPECT_EQ(val(gym.app(), "world.seed"), "12");
  EXPECT_NE(heights(), h11) << "another seed made the same world";
  gym.reset(11);
  EXPECT_EQ(heights(), h11);
  EXPECT_EQ(occupancy(), o11);
  for (int i = 0; i < 20; ++i)
    gym.step({kNaN}, {kNaN});
  EXPECT_FALSE(gym.observe().out[0]);
}

TEST(AriadneWorldGym, ADocumentWithoutVehiclesOrAWorldIsRefused) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  try {
    WorldGym gym(
        write_doc("gym_none", std::string("requires: [scene.physics]\n") + kGymSquare).string());
    FAIL() << "a document without vehicles loaded";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("no vehicles"), std::string::npos) << e.what();
  }
  try {
    WorldGym gym(write_doc("gym_nothing", "windows:\n  - window: W\n").string());
    FAIL() << "a document without a world loaded";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("runs no world"), std::string::npos) << e.what();
  }
}

// ---------------------------------------------------------------------------------------------
// A generated world's ground under its vehicles: its collider is where the navigator's grid says
// the ground is, and a vehicle started on a slope starts on it, not partly inside it.

namespace {

// world_gym.ari's generated world: hills, and buildings along its streets.
const char *kGymLand = R"(
scene:
  physics: {}
  world: { source: land, cell: 2 }
  nodes:
    - node: land
      type: generated_world
      world: land
      seed: 5
      half: 150
      trees: 60
      rocks: 20
      buildings: 24
      body: {}
)";

} // namespace

TEST(AriadneSceneWorld, AGeneratedWorldsColliderIsUnderEveryFreeCell) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  cvc::app app;
  LoadResult r = load_string(app, kGymLand);
  ASSERT_TRUE(r.ok) << r.error;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "", &warnings);
  ASSERT_NE(w, nullptr);
  ASSERT_GT(w->generated_world("land")->building_count, 0);
  std::shared_ptr<const cvc::nav::world_grid> g = w->grid("land", 2.0, 0.0, &warnings);
  ASSERT_NE(g, nullptr);
  // Straight down at every free sample (the buildings' too are next to them): the ground, where
  // the grid has it.
  int free = 0, missed = 0, off = 0;
  for (int row = 0; row < g->rows; ++row)
    for (int c = 0; c < g->cols; ++c) {
      const std::size_t i = static_cast<std::size_t>(row) * g->cols + c;
      if (g->occupancy[i])
        continue;
      ++free;
      const double x = g->min_x + c * g->cell(), y = g->min_y + row * g->cell();
      const std::optional<cvc::physics::ray_hit> hit =
          w->world().raycast({x, y, 500.0}, {x, y, -500.0});
      if (!hit)
        ++missed;
      else if (std::fabs(hit->point.z - g->height[i]) > 0.1)
        ++off;
    }
  EXPECT_GT(free, g->rows * g->cols / 2);
  EXPECT_EQ(missed, 0);
  EXPECT_EQ(off, 0);
}

TEST(AriadneWorldGym, AVehicleStartedOnASlopeStaysOnTheGround) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const std::string path =
      write_doc("gym_land", std::string("requires: [scene.physics]\n") + kGymVehicles + kGymLand)
          .string();
  // Free places on the world's slopes (1 in 3 to 1 in 1), apart, each facing up its slope or across
  // it. A vehicle placed level at one sample of the ground starts with its uphill end in the
  // ground, and on the steeper ones its body too: it falls through the terrain from the first step
  // (world_gym.ari's generated world, episode seed 1).
  std::vector<std::array<double, 3>> spots; // x, y, heading
  {
    cvc::app app;
    LoadResult r = load_string(app, kGymLand);
    ASSERT_TRUE(r.ok) << r.error;
    std::vector<std::string> warnings;
    std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "", &warnings);
    ASSERT_NE(w, nullptr);
    std::shared_ptr<const cvc::world::generated_world> gw = w->generated_world("land");
    std::shared_ptr<const cvc::nav::world_grid> g = w->grid("land", 2.0);
    ASSERT_NE(g, nullptr);
    for (double y = -120.0; y <= 120.0 && spots.size() < 12; y += 6.0)
      for (double x = -120.0; x <= 120.0 && spots.size() < 12; x += 6.0) {
        bool clear = true;
        for (double dy = -6.0; dy <= 6.0 && clear; dy += 2.0)
          for (double dx = -6.0; dx <= 6.0 && clear; dx += 2.0)
            clear = !g->occupied(x + dx, y + dy);
        bool apart = true;
        for (const auto &s : spots)
          apart = apart && std::hypot(s[0] - x, s[1] - y) > 25.0;
        if (!clear || !apart)
          continue;
        const double gx = (gw->height(x + 2.5, y) - gw->height(x - 2.5, y)) / 5.0,
                     gy = (gw->height(x, y + 2.5) - gw->height(x, y - 2.5)) / 5.0;
        const double slope = std::hypot(gx, gy);
        if (slope < 0.33 || slope > 1.0)
          continue;
        const double up = std::atan2(gy, gx);
        spots.push_back({x, y, spots.size() % 2 ? up + 0.5 * 3.14159265358979 : up});
      }
  }
  ASSERT_GE(spots.size(), 4u);
  gym_options o;
  for (const auto &s : spots) {
    entity_spec e;
    e.archetype = "humvee";
    e.controller = "none";
    e.placed = true;
    e.x = s[0];
    e.y = s[1];
    e.yaw = s[2];
    o.agents.push_back(e);
  }
  WorldGym gym(path, o);
  const gym_observation o0 = gym.reset(1);
  ASSERT_EQ(o0.n, spots.size());
  std::shared_ptr<const cvc::world::generated_world> land = gym.world().generated_world("land");
  // Each starts above the ground under it, tilted onto its slope.
  for (std::size_t i = 0; i < spots.size(); ++i) {
    EXPECT_GT(o0.z[i], land->height(o0.x[i], o0.y[i]) + 0.5)
        << "agent " << i << " starts in the ground at " << spots[i][0] << ", " << spots[i][1];
    EXPECT_NEAR(o0.yaw[i], std::remainder(spots[i][2], 2.0 * 3.14159265358979), 0.05);
  }
  const std::vector<double> none(spots.size(), kNaN); // each parks where it is
  gym_step st;
  for (int i = 0; i < 60; ++i) // 3 s
    st = gym.step(none, none);
  // And is on it 3 s later (the steepest slide a few metres down theirs, parked).
  for (std::size_t i = 0; i < spots.size(); ++i) {
    const double ground = land->height(st.observation.x[i], st.observation.y[i]);
    EXPECT_FALSE(st.observation.out[i])
        << "agent " << i << " at " << spots[i][0] << ", " << spots[i][1];
    EXPECT_GT(st.observation.z[i], ground) << "agent " << i << " fell into the ground";
    EXPECT_LT(st.observation.z[i], ground + 2.5) << "agent " << i;
  }
}

// A generated world's terrain has walls at its edge: a vehicle driven flat out at the edge stops
// against it, a contact with an obstacle, and stays in the world. Without them it drives off the
// terrain and out (world_gym.ari's generated world, episode seed 1: a vehicle slid down the
// slippery underwater slope at the west edge and off it).
TEST(AriadneWorldGym, AGeneratedWorldsEdgeHoldsAVehicleIn) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const std::string path = write_doc("gym_land_edge", std::string("requires: [scene.physics]\n") +
                                                          kGymVehicles + kGymLand)
                               .string();
  // Free places 12-30 m from each edge, apart, each facing its edge.
  std::vector<std::array<double, 3>> spots; // x, y, heading
  {
    cvc::app app;
    LoadResult r = load_string(app, kGymLand);
    ASSERT_TRUE(r.ok) << r.error;
    std::vector<std::string> warnings;
    std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "", &warnings);
    ASSERT_NE(w, nullptr);
    std::shared_ptr<const cvc::nav::world_grid> g = w->grid("land", 2.0);
    ASSERT_NE(g, nullptr);
    const double pi = 3.14159265358979;
    const std::array<std::array<double, 3>, 4> edges = {
        {{-1, 0, pi}, {1, 0, 0}, {0, -1, -0.5 * pi}, {0, 1, 0.5 * pi}}};
    for (const auto &e : edges) {
      bool found = false;
      for (double in = 12.0; in <= 30.0 && !found; in += 2.0)
        for (double along = -100.0; along <= 100.0 && !found; along += 4.0) {
          const double x = e[0] != 0 ? e[0] * (150.0 - in) : along,
                       y = e[1] != 0 ? e[1] * (150.0 - in) : along;
          bool clear = true;
          for (double dy = -6.0; dy <= 6.0 && clear; dy += 2.0)
            for (double dx = -6.0; dx <= 6.0 && clear; dx += 2.0)
              clear = !g->occupied(x + dx, y + dy);
          if (clear) {
            spots.push_back({x, y, e[2]});
            found = true;
          }
        }
    }
  }
  ASSERT_GE(spots.size(), 2u);
  gym_options o;
  for (const auto &s : spots) {
    entity_spec e;
    e.archetype = "humvee";
    e.controller = "none";
    e.placed = true;
    e.x = s[0];
    e.y = s[1];
    e.yaw = s[2];
    o.agents.push_back(e);
  }
  WorldGym gym(path, o);
  gym.reset(1);
  const std::vector<double> straight(spots.size(), 0.0), flat_out(spots.size(), 3.0);
  std::vector<int> hit(spots.size(), 0);
  std::vector<double> reach(spots.size(), 0.0); // the farthest each got toward its edge
  gym_step st;
  for (int i = 0; i < 400; ++i) { // 20 s
    st = gym.step(straight, flat_out);
    for (std::size_t k = 0; k < spots.size(); ++k) {
      hit[k] += st.observation.hit_obstacle[k];
      reach[k] = std::max(reach[k],
                          std::max(std::abs(st.observation.x[k]), std::abs(st.observation.y[k])));
      ASSERT_FALSE(st.observation.out[k])
          << "agent " << k << " from " << spots[k][0] << ", " << spots[k][1]
          << " left the world at " << st.observation.x[k] << ", " << st.observation.y[k];
    }
    if (st.terminated || st.truncated)
      break;
  }
  // Some stall on a steep slope short of their edge; the rest drive into the wall and stop there.
  int stopped = 0;
  for (std::size_t k = 0; k < spots.size(); ++k) {
    EXPECT_LT(reach[k], 150.0) << "agent " << k;
    if (hit[k] > 0) {
      ++stopped;
      EXPECT_GT(reach[k], 145.0) << "agent " << k << " hit something short of its edge";
    }
  }
  EXPECT_GE(stopped, 1) << "no agent met a wall";
}

// A generated_world node's `edge:` picks what is at the world's edge: `shore` lines it with a beach
// and the sea and leaves out the walls, so the node's bodies are its colliders alone.
TEST(AriadneSceneWorld, AGeneratedWorldsEdgeCanBeAShore) {
  REQUIRE_YAML();
  std::string doc = kGymLand;
  const std::string at = "      body: {}\n";
  ASSERT_NE(doc.find(at), std::string::npos);
  doc.insert(doc.find(at), "      edge: shore\n");
  cvc::app app;
  LoadResult r = load_string(app, doc);
  ASSERT_TRUE(r.ok) << r.error;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "", &warnings);
  ASSERT_NE(w, nullptr);
  std::shared_ptr<const cvc::world::generated_world> gw = w->generated_world("land");
  ASSERT_NE(gw, nullptr);
  EXPECT_EQ(gw->params.edge, int(cvc::world::world_edge::shore));
  EXPECT_EQ(w->bodies_of("land").size(), collide_items(*gw));         // no walls
  EXPECT_LT(gw->height(-149.9, 0.0), gw->params.water_level_m - 5.0); // the sea at the edge
}

// The world's edge is a live setting (world.edge, the World panel's Edge), seeded from the node's
// `edge:`: an edit waits for Regenerate like the others, and the new world has that edge, its
// walls going with it.
TEST(AriadneSceneWorld, TheWorldsEdgeIsALiveSetting) {
  REQUIRE_YAML();
  cvc::app app;
  LoadResult r = load_string(app, kGenerated);
  ASSERT_TRUE(r.ok) << r.error;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "", &warnings);
  ASSERT_NE(w, nullptr);
  std::shared_ptr<const cvc::world::generated_world> first = w->generated_world("world");
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(val(app, "world.edge"), "0"); // the node's default: walls
  const std::size_t items = collide_items(*first);
  EXPECT_EQ(w->bodies_of("world").size(), items + kEdgeWalls);

  cvc::state::instance(app)("world.edge").value("1"); // shore
  w->sync(&warnings);
  EXPECT_EQ(w->generated_world("world"), first);
  EXPECT_EQ(val(app, "world.pending"), "1");
  cvc::state::instance(app)("world.request").value("regenerate");
  w->sync(&warnings);
  std::shared_ptr<const cvc::world::generated_world> shore = w->generated_world("world");
  ASSERT_NE(shore, nullptr);
  EXPECT_NE(shore, first);
  EXPECT_EQ(shore->params.edge, int(cvc::world::world_edge::shore));
  EXPECT_EQ(shore->params.seed, first->params.seed);              // the same world but its edge
  EXPECT_EQ(w->bodies_of("world").size(), collide_items(*shore)); // no walls

  cvc::state::instance(app)("world.edge").value("0"); // back to walls
  cvc::state::instance(app)("world.request").value("regenerate");
  w->sync(&warnings);
  ASSERT_NE(w->generated_world("world"), nullptr);
  EXPECT_EQ(w->generated_world("world")->params.edge, int(cvc::world::world_edge::wall));
  EXPECT_EQ(w->bodies_of("world").size(), collide_items(*w->generated_world("world")) + kEdgeWalls);
}

// O20 (F6): the rain's friction is the scene's world's (WeatherFriction), set inside the physics
// step on the world clock's ticks. So a world without a renderer applies it, and it changes at the
// same clock tick whatever the frames: one run takes a frame per clock quantum (every step seen),
// another uneven 0.137 s frames, and wherever the second is looked at it has the first's friction.
// weather.friction 0 gives the surfaces their dry friction back at the next step.
TEST(AriadneSceneWeather, TheRainsFrictionIsSetInTheStepOnTheClocksTicks) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const cvc::world::surface_registry &surfaces = cvc::world::surface_registry::builtin();
  const cvc::world::surface_class &asphalt = surfaces[surfaces.by_name("asphalt")];
  const float dry = static_cast<float>(asphalt.mu);
  const float soaked = static_cast<float>(cvc::physics::wet_friction(asphalt, 1.0));
  ASSERT_LT(soaked, dry);
  // A full storm born over the terrain at t = 0; the weather runs 50 times the world's time, so its
  // rain soaks the ground (2 weather seconds) well inside the first half second.
  cvc::world::weather_event e;
  e.t = 0.0;
  e.x = 0.0;
  e.y = 0.0;
  e.radius = 400.0;
  e.peak = 1.0;
  e.life = 3000.0;
  const std::string events = cvc::world::format_weather_events({e});
  struct sample {
    std::uint64_t tick;
    float friction;
  };
  struct run_result {
    std::vector<sample> samples;
    double wetness = -1.0;
    std::string published;
    float off = 0.0f; // the friction a frame after weather.friction went to 0
  };
  const auto run = [&](double frame_dt, double seconds) {
    LoadResult r = fresh_load(kScene);
    EXPECT_TRUE(r.ok) << r.error;
    SceneNode wn;
    wn.id = "weather";
    wn.type = "weather";
    wn.props.kind = Value::Kind::Map;
    for (const auto &[k, v] :
         std::vector<std::pair<const char *, std::string>>{{"chance", "0"},
                                                           {"area", "100"},
                                                           {"soak", "2"},
                                                           {"dry", "600"},
                                                           {"time_scale", "50"},
                                                           {"events", events}}) {
      Value s;
      s.kind = Value::Kind::Scalar;
      s.scalar = v;
      wn.props.entries.emplace_back(k, s);
    }
    r.scene.nodes.push_back(wn);
    cvc::app app;
    std::vector<std::string> warnings;
    std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
    run_result out;
    if (!w || w->weathers().size() != 1) {
      ADD_FAILURE() << "no world, or not one weather runtime";
      return out;
    }
    const cvc::physics::material_id m =
        w->world().add_material({"asphalt", dry, 0.0f, asphalt.rolling_resistance});
    SceneClockDriver clock(app, resolved_clock(r.scene));
    while (app.world_clock().t() < seconds) {
      clock.drive(frame_dt);
      w->follow(app.world_clock());
      out.samples.push_back({app.world_clock().tick(), w->world().get_material(m).friction});
    }
    out.wetness = w->weathers()[0]->applied();
    out.published = val(app, "weather.wetness");
    cvc::state::instance(app)("weather.friction").value(0.0);
    clock.drive(frame_dt);
    w->follow(app.world_clock());
    out.off = w->world().get_material(m).friction;
    return out;
  };
  cvc::app probe;
  const double quantum = probe.world_clock().fixed_dt();
  const run_result fine = run(quantum, 3.0);
  const run_result coarse = run(0.137, 3.0);
  ASSERT_FALSE(fine.samples.empty());
  ASSERT_FALSE(coarse.samples.empty());

  // The rain lowered the friction, with no renderer, to the wet ground's at the wetness it applied
  // (the storm, steered on, has left the point and the ground has begun to dry), and said so.
  EXPECT_GT(fine.wetness, 0.5);
  EXPECT_LE(fine.wetness, 1.0);
  EXPECT_EQ(fine.samples.back().friction,
            static_cast<float>(cvc::physics::wet_friction(asphalt, fine.wetness)));
  EXPECT_LT(fine.samples.back().friction, dry);
  EXPECT_GE(fine.samples.back().friction, soaked);
  EXPECT_EQ(std::stod(fine.published), fine.wetness);
  EXPECT_EQ(coarse.wetness, fine.wetness);
  // It changed on the clock: on the first step of a half second (none before it), never between.
  std::uint64_t first = 0;
  for (const sample &s : fine.samples)
    if (s.friction != dry) {
      first = s.tick;
      break;
    }
  ASSERT_GT(first, 0u);
  const double t_first = static_cast<double>(first) * quantum;
  EXPECT_NEAR(std::fmod(t_first, WeatherFriction::kWetPeriod), 0.0, 0.05)
      << "the friction first changed at t = " << t_first << " s";
  // The coarse frames saw the same friction at every tick they were looked at.
  std::map<std::uint64_t, float> by_tick;
  for (const sample &s : fine.samples)
    by_tick[s.tick] = s.friction;
  int compared = 0;
  for (const sample &s : coarse.samples) {
    const auto it = by_tick.find(s.tick);
    if (it == by_tick.end())
      continue;
    ++compared;
    EXPECT_EQ(s.friction, it->second) << "at tick " << s.tick;
  }
  EXPECT_GT(compared, 10);
  // Off: dry again at the next step.
  EXPECT_EQ(fine.off, dry);
  EXPECT_EQ(coarse.off, dry);
}

// O20 (F4, F6's gusts): the storms' gust fronts are in the scene's wind. A light ball (1 kg, 1 m
// across) floats with gravity off, 10 m up, in a 6 m/s wind toward +x, with a weather node whose
// one authored storm (400 m across, at its peak) is centred at (cx, cy) at t = 0 (the ball starts
// at the origin) and runs at `rate` weather seconds a world second. The scene's wind steers it
// (scene_weather_steering): toward +x at the wind's speed at the deck's base (300 m with no sky).
namespace {

const char *kStormAir = R"(
scene:
  physics: { gravity: [0, 0, 0] }
  wind: { speed: 6, dir: 0, gustiness: 0 }
  nodes:
    - node: ball
      source: { sphere: { radius: 0.5 } }
      transform: { position: [0, 0, 10] }
      body: { mass: 1 }
)";

LoadResult storm_scene(double cx, double cy, double rate, bool storm = true,
                       const char *doc = kStormAir) {
  LoadResult r = fresh_load(doc);
  EXPECT_TRUE(r.ok) << r.error;
  std::string events;
  if (storm) {
    // Born 1500 weather seconds before t = 0 (mature then), where the steering has carried it to
    // (cx, cy) by t = 0.
    const double steer = 6.0 * cvc::algo::wind_profile(kWeatherDeckBase);
    cvc::world::weather_event e;
    e.t = 0.0;
    e.tau = -1500.0;
    e.x = cx - steer * 1500.0;
    e.y = cy;
    e.radius = 400.0;
    e.peak = 1.0;
    e.life = 3000.0;
    events = cvc::world::format_weather_events({e});
  }
  SceneNode wn;
  wn.id = "weather";
  wn.type = "weather";
  wn.props.kind = Value::Kind::Map;
  for (const auto &[k, v] :
       std::vector<std::pair<const char *, std::string>>{{"chance", "0"},
                                                         {"area", "100"},
                                                         {"time_scale", std::to_string(rate)},
                                                         {"events", events}}) {
    Value sv;
    sv.kind = Value::Kind::Scalar;
    sv.scalar = v;
    wn.props.entries.emplace_back(k, sv);
  }
  r.scene.nodes.push_back(wn);
  return r;
}

struct storm_run {
  std::vector<std::pair<std::uint64_t, cvc::physics::body_state>> frames; // the ball at each frame
  std::vector<std::uint64_t> hashes;                                      // the world's, each frame
  cvc::physics::body_state end;
  std::string front; // wind.front at the end
  bool storm_flow = false;
  double offset = -1.0; // the storm air's clock offset (StormWindFlow::clock_offset)
  // At the end, the front at (-300, 0): the air read at the world's field time (what a body
  // outside the world passes, mujoco_robot::set_wind), the weathers at the clock's time (the storms
  // drawn), and the air read at the clock's time (the doubled offset a caller must not pass).
  double front_field = 0.0, front_clock = 0.0, front_doubled = 0.0;
  std::uint64_t keyframes = 0; // the water's sea keyframes, when the scene has a water node
};

// Each frame drives the clock, then the world follows it. The world anchors on the clock at its
// first follow: at t = 0, as a drawn scene's first frame (its clock driven 0 s), unless `late`.
storm_run run_storm(const LoadResult &r, double frame_dt, double seconds, bool late = false,
                    const char *body = "ball") {
  storm_run out;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  if (!w) {
    ADD_FAILURE() << "no world";
    return out;
  }
  SceneClockDriver clock(app, resolved_clock(r.scene));
  if (!late)
    w->follow(app.world_clock());
  while (app.world_clock().t() < seconds) {
    clock.drive(frame_dt);
    w->follow(app.world_clock());
    out.frames.emplace_back(app.world_clock().tick(), w->world().state(w->body(body)));
    out.hashes.push_back(w->world().state_hash());
  }
  out.end = w->world().state(w->body(body));
  if (w->wind_field().valid())
    if (const auto *storm =
            dynamic_cast<const StormWindFlow *>(w->world().get_field(w->wind_field()).wind.get())) {
      out.storm_flow = true;
      out.offset = storm->clock_offset();
      double fx, fy;
      storm->front(-300.0, 0.0, w->world().field_time(), fx, fy);
      out.front_field = fx;
      storm->front(-300.0, 0.0, app.world_clock().t(), fx, fy);
      out.front_doubled = fx;
      for (const cvc::world::weather &m : storm->weathers()) {
        m.gust_front(m.at(app.world_clock().t()), -300.0, 0.0, fx, fy);
        out.front_clock += fx;
      }
    }
  if (const WaterRuntime *water = w->water("water"))
    if (water->keyframes())
      out.keyframes = water->keyframes()->keyframe;
  cvc::state *f = cvc::state::instance(app).findDescendant("wind.front");
  out.front = f ? f->value() : std::string(); // never written: no front
  return out;
}

} // namespace

// A light body is pushed by a storm's gust front (F6: gusts push light bodies): ahead of the storm,
// at its front's edge, the 6 m/s wind is 18 m/s and the ball goes with it; beside the storm the
// front blows across the wind, so the ball is carried off to the side. With the storm out of reach,
// or no storm, the ball rides the 6 m/s wind alone. The world's air is a StormWindFlow then.
TEST(AriadneSceneWeather, AStormsGustFrontPushesALightBody) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const double s = 3.0;
  const storm_run ahead = run_storm(storm_scene(-600.0, 0.0, 1.0), 1.0 / 60.0, s);
  const storm_run beside = run_storm(storm_scene(0.0, -600.0, 1.0), 1.0 / 60.0, s);
  const storm_run away = run_storm(storm_scene(-9000.0, 0.0, 1.0), 1.0 / 60.0, s);
  const storm_run none = run_storm(storm_scene(0.0, 0.0, 1.0, false), 1.0 / 60.0, s);
  std::printf("[storm] a 1 kg ball in a 6 m/s wind, %.0f s on: ahead of a storm (%.2f, %.2f) m at "
              "(%.2f, %.2f) m/s; beside it (%.2f, %.2f) m at (%.2f, %.2f) m/s; out of reach "
              "(%.2f, %.2f) m; no storm (%.2f, %.2f) m\n",
              s, ahead.end.position.x, ahead.end.position.y, ahead.end.linear_velocity.x,
              ahead.end.linear_velocity.y, beside.end.position.x, beside.end.position.y,
              beside.end.linear_velocity.x, beside.end.linear_velocity.y, away.end.position.x,
              away.end.position.y, none.end.position.x, none.end.position.y);
  EXPECT_TRUE(ahead.storm_flow);
  EXPECT_TRUE(none.storm_flow); // a weather node: the air can carry its storms
  // Ahead: the air is 18 m/s along the wind; the ball nearly at it.
  EXPECT_GT(ahead.end.linear_velocity.x, 15.0);
  EXPECT_LT(ahead.end.linear_velocity.x, 18.5);
  EXPECT_NEAR(ahead.end.linear_velocity.y, 0.0, 0.05);
  // Beside: across the wind, out of the storm (+y): the wind there veers.
  EXPECT_GT(beside.end.linear_velocity.y, 4.0);
  EXPECT_GT(std::atan2(beside.end.linear_velocity.y, beside.end.linear_velocity.x), 0.6);
  // Out of reach, and with no storm, the 6 m/s wind alone, the same to the bit.
  EXPECT_LT(away.end.linear_velocity.x, 6.01);
  EXPECT_GT(away.end.linear_velocity.x, 5.0);
  EXPECT_EQ(away.end.position.x, none.end.position.x);
  EXPECT_EQ(away.end.position.y, none.end.position.y);
  EXPECT_GT(ahead.end.position.x, 2.0 * none.end.position.x);
  // The wind the water and the trees read is the front at the weather's centre (the ball's start):
  // published as wind.front, and none without a storm near.
  double fx = 0.0, fy = 0.0;
  ASSERT_EQ(std::sscanf(ahead.front.c_str(), "%lf %lf", &fx, &fy), 2) << ahead.front;
  EXPECT_GT(fx, 11.0);
  EXPECT_NEAR(fy, 0.0, 0.01);
  EXPECT_EQ(away.front, "");
  EXPECT_EQ(none.front, "");
}

// The gust front is in the physics step on the clock's time, not a frame's: with the storm walking
// at 146 m/s (a time-lapse of 15), its front sweeps up to the ball in a few seconds, and one run
// with a frame per clock quantum and one with uneven 0.137 s frames see the ball, and the whole
// world's state hash, the same at every tick both looked at.
TEST(AriadneSceneWeather, TheGustFrontIsInTheStepOnTheClocksTime) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const LoadResult r = storm_scene(-1200.0, 0.0, 15.0);
  cvc::app probe;
  const storm_run fine = run_storm(r, probe.world_clock().fixed_dt(), 6.0);
  const storm_run coarse = run_storm(r, 0.137, 6.0);
  ASSERT_FALSE(fine.frames.empty());
  ASSERT_FALSE(coarse.frames.empty());
  // The front reached the ball: the wind it rode rose from 6 m/s.
  double fastest = 0.0;
  for (const auto &[tick, b] : fine.frames)
    fastest = std::max(fastest, b.linear_velocity.x);
  std::printf("[storm] the ball's fastest as the front swept up: %.2f m/s; %zu and %zu frames\n",
              fastest, fine.frames.size(), coarse.frames.size());
  EXPECT_GT(fastest, 12.0);
  std::map<std::uint64_t, std::size_t> by_tick;
  for (std::size_t k = 0; k < fine.frames.size(); ++k)
    by_tick[fine.frames[k].first] = k;
  int compared = 0;
  for (std::size_t k = 0; k < coarse.frames.size(); ++k) {
    const auto it = by_tick.find(coarse.frames[k].first);
    if (it == by_tick.end())
      continue;
    ++compared;
    const cvc::physics::body_state &a = fine.frames[it->second].second;
    const cvc::physics::body_state &b = coarse.frames[k].second;
    EXPECT_EQ(a.position.x, b.position.x) << "at tick " << it->first;
    EXPECT_EQ(a.position.y, b.position.y) << "at tick " << it->first;
    EXPECT_EQ(a.linear_velocity.x, b.linear_velocity.x) << "at tick " << it->first;
    EXPECT_EQ(fine.hashes[it->second], coarse.hashes[k]) << "at tick " << it->first;
  }
  EXPECT_GT(compared, 10);
  // A world that anchored on the clock late (its first follow after a 0.137 s frame) reads the
  // storms at the clock's time all the same: its air's offset is the clock's time there.
  const storm_run late = run_storm(r, 0.137, 0.5, true);
  EXPECT_EQ(fine.offset, 0.0);
  EXPECT_NEAR(late.offset, late.frames.front().first * probe.world_clock().fixed_dt(), 1e-12);
  EXPECT_GT(late.offset, 0.05);
  // What a body outside the world reads (a MuJoCo robot: the air at the world's field time) is the
  // storms at the clock's time; the air read at the clock's time would add the offset twice.
  std::printf("[storm] a late world's front at (-300, 0): %.6f m/s read at its field time, %.6f at "
              "the clock's (the storms drawn), %.6f with the offset twice\n",
              late.front_field, late.front_clock, late.front_doubled);
  EXPECT_GT(late.front_clock, 1.0);
  EXPECT_NEAR(late.front_field, late.front_clock, 1e-9 * std::fabs(late.front_clock));
  EXPECT_GT(std::fabs(late.front_doubled - late.front_clock), 1e-4);
}

// The water reads the storm's front through its sea's lag (E4, F4): its spec's wind is the scene's
// with the front at the weather's centre in it, while the gusts' pattern drifts with the wind as
// authored; the sea's lagged wind then builds toward it over sea_lag seconds. A scene without a
// weather node never writes wind.front, and its water reads the wind as authored.
TEST(AriadneSceneWeather, TheWaterReadsTheGustFrontThroughItsLag) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build"; // the scene's world, which writes the front
  cvc::app app;
  LoadResult r = storm_scene(-600.0, 0.0, 1.0);
  SceneNode water;
  water.id = "water";
  water.type = "water";
  water.props.kind = Value::Kind::Map;
  seed_scene_wind(app, r.scene, "");
  // Without the front: the wind as authored.
  const SceneWaterSpec calm = scene_water_spec(app, water, "");
  EXPECT_EQ(calm.wind_speed, 6.0);
  EXPECT_EQ(calm.wind_dir, 0.0);
  // The scene's world writes the front at the weather's centre (its area's: the origin).
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "", &warnings);
  ASSERT_NE(w, nullptr);
  w->sync(&warnings);
  const std::optional<SceneWind> now = scene_wind_now(app, "");
  ASSERT_TRUE(now.has_value());
  std::printf("[storm] wind.front '%s': the wind %.2f m/s toward %.2f deg, as authored %.2f m/s\n",
              val(app, "wind.front").c_str(), now->mean_speed(), now->mean_dir(), now->speed);
  EXPECT_GT(now->mean_speed(), 17.0); // 6 + the front's 12
  EXPECT_NEAR(now->mean_dir(), 0.0, 0.1);
  EXPECT_EQ(now->speed, 6.0);
  EXPECT_EQ(scene_wind_set(app, "")->front_x, 0.0); // the steering's and the clouds': as authored
  const SceneWaterSpec stormy = scene_water_spec(app, water, "");
  EXPECT_EQ(stormy.wind_speed, now->mean_speed());
  EXPECT_EQ(stormy.gust_speed, 6.0);
  EXPECT_EQ(stormy.gust_dir, 0.0);
  // The sea builds toward it over its lag (20 s): under half way after 6 s, most of the way by 60.
  SceneWaterSeaClock sea;
  SceneWaterSpec lagged = stormy; // its default lag (20 s) and crossfade (4 s)
  lagged.lee = false;
  lagged.wind_speed = 6.0;
  sea.advance(0.0, lagged, nullptr);
  lagged.wind_speed = stormy.wind_speed;
  double at6 = 0.0, build_ms = 0.0, most_ms = 0.0;
  const std::uint64_t first = sea.sea()->keyframe;
  for (int k = 1; k <= 120; ++k) {
    const std::uint64_t had = sea.sea()->keyframe;
    sea.advance(0.5 * k, lagged, nullptr);
    if (sea.sea()->keyframe != had) { // what the front costs the sea: a keyframe's h0
      build_ms += sea.sea()->build_ms;
      most_ms = std::max(most_ms, sea.sea()->build_ms);
    }
    if (k == 12)
      at6 = sea.sea()->to.speed_mps;
  }
  std::printf("[storm] the sea's wind: %.2f m/s at 6 s, %.2f m/s at 60 s; %llu keyframes, %.0f ms "
              "in all, %.0f ms at most\n",
              at6, sea.sea()->to.speed_mps,
              static_cast<unsigned long long>(sea.sea()->keyframe - first), build_ms, most_ms);
  EXPECT_GT(at6, 6.5);
  EXPECT_LT(at6, 0.5 * (6.0 + stormy.wind_speed));
  EXPECT_GT(sea.sea()->to.speed_mps, 0.85 * stormy.wind_speed);
  // No weather node: no wind.front, and the water reads the wind as authored.
  cvc::app plain;
  LoadResult pr = fresh_load(kStormAir);
  ASSERT_TRUE(pr.ok) << pr.error;
  std::unique_ptr<SceneWorld> pw = SceneWorld::from_scene(plain, pr.scene, "", "", &warnings);
  ASSERT_NE(pw, nullptr);
  pw->sync(&warnings);
  EXPECT_EQ(cvc::state::instance(plain).findDescendant("wind.front"), nullptr);
  EXPECT_EQ(scene_water_spec(plain, water, "").wind_speed, 6.0);
  ASSERT_TRUE(pw->wind_field().valid());
  EXPECT_EQ(dynamic_cast<const StormWindFlow *>(pw->world().get_field(pw->wind_field()).wind.get()),
            nullptr);
}

// What the storms cost the physics a step: the weather once (its cells at the step's time, cached),
// then O(cells) a body. Timed against the plain wind on 100 bodies, with 32 events in the weather
// (as WorldWeather.AFrameIsCheap times a frame), and where no front reaches the storm wind is the
// plain wind's velocity bit for bit.
TEST(AriadneSceneWeather, TheStormWindIsCheap) {
  cvc::world::weather_params p;
  p.chance = 0.0;
  cvc::world::weather model(p);
  std::vector<cvc::world::weather_event> ev;
  for (int k = 0; k < 32; ++k) {
    cvc::world::weather_event e;
    e.t = 30.0 * k;
    if (k % 4 == 0) {
      e.x = -600.0 + 10.0 * k;
      e.y = 0.0;
      e.radius = 400.0;
      e.peak = 1.0;
      e.life = 3000.0;
    } else {
      e.type = cvc::world::weather_event::kind::steer;
      e.speed = 3.0;
      e.az = 90.0 + k;
    }
    ev.push_back(e);
  }
  model.set_events(ev);
  const cvc::algo::wind_state w{6.0, 0.3, 0.25};
  const StormWindFlow storm(w, 0.0, {model});
  const cvc::physics::wind_flow plain(w, 0.0);
  const int bodies = 100, steps = 2000;
  const auto time = [&](const cvc::physics::flow_velocity &f) {
    double sum = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int s = 0; s < steps; ++s) {
      const double t = 600.0 + s / 120.0;
      for (int b = 0; b < bodies; ++b)
        sum += f.reference({-500.0 + 10.0 * b, 3.0 * b, 2.0}, t).x;
    }
    const double us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    EXPECT_TRUE(std::isfinite(sum));
    return us / steps;
  };
  const double us_plain = time(plain), us_storm = time(storm);
  std::printf("[storm] a step's wind on %d bodies: %.1f us plain, %.1f us with the storms (%zu "
              "cells live)\n",
              bodies, us_plain, us_storm, model.at(600.0).cells.size());
  EXPECT_LT(us_storm, 2000.0); // loose: a slow, shared machine
  // Far from every front: the plain wind's velocity to the bit.
  for (double t : {0.0, 600.0, 1234.5}) {
    const cvc::physics::vec3 p0{9.0e4, -7.0e4, 12.0};
    const cvc::physics::vec3 a = storm.velocity(p0, t), b = plain.velocity(p0, t);
    EXPECT_EQ(a.x, b.x);
    EXPECT_EQ(a.y, b.y);
    EXPECT_EQ(a.z, b.z);
  }
  // The digest names the weather: another event, another digest.
  std::vector<cvc::world::weather_event> more = ev;
  more.push_back(ev.front());
  more.back().t = 2000.0;
  cvc::world::weather other = model;
  other.set_events(more);
  EXPECT_NE(StormWindFlow(w, 0.0, {other}).digest(), storm.digest());
  EXPECT_EQ(StormWindFlow(w, 0.0, {model}).digest(), storm.digest());
  EXPECT_NE(storm.digest(), plain.digest());
}

// A crate floating on a lake (1 x 1 x 0.5 m at half the water's density) in a 6 m/s wind, with the
// storm of storm_scene time-lapsed by 15 so its front reaches the lake's centre in a few seconds.
const char *kStormLake = R"(
scene:
  physics: {}
  wind: { speed: 6, dir: 0, gustiness: 0 }
  nodes:
    - node: terrain
      source:
        heightfield:
          size: 400
          resolution: 81
          layers:
            - { kind: dome, amplitude: -30, radius: 150 }
      body: static
    - node: water
      type: water
      ground: terrain
      level: 0
      wave_amp: 0.3
      body: {}
    - node: ball
      source: { box: { size: [1, 1, 0.5] } }
      transform: { position: [0, 0, 0] }
      body: { density: 500 }
)";

// A 10 kg crate, 1 m a side (friction 0.3: 29 N), on a floor in a 6 m/s wind (9 N on it), so
// it comes to rest and sleeps; a storm's front reaches it 10 s in (18 m/s there: 84 N).
const char *kStormFloor = R"(
scene:
  physics: { materials: { slick: { friction: 0.3 } } }
  wind: { speed: 6, dir: 0, gustiness: 0 }
  nodes:
    - node: floor
      source: { box: { size: [600, 600, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - node: ball
      source: { box: { size: [1, 1, 1] } }
      transform: { position: [0, 0, 0.5] }
      body: { mass: 10, material: slick }
)";

// In a scene with water, weather and wind, a passing front moves the water's wind on its own, and
// the physics sea's keyframes, and the field the bodies float in, are timed on the clock: the sea
// steps once a clock period and each keyframe's field goes in at a step the clock names. So a run
// with a frame per clock quantum and one with 0.137 s frames float the crate alike, and the whole
// world's state hash agrees, at every tick both looked at, across the front's arrival.
TEST(AriadneSceneWeather, TheFrontsSeaFloatsBodiesAlikeAtEveryFrameRate) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const LoadResult r = storm_scene(-1200.0, 0.0, 15.0, true, kStormLake);
  cvc::app probe;
  const storm_run fine = run_storm(r, probe.world_clock().fixed_dt(), 9.0);
  const storm_run coarse = run_storm(r, 0.137, 9.0);
  ASSERT_FALSE(fine.frames.empty());
  ASSERT_FALSE(coarse.frames.empty());
  std::printf("[storm] the lake: %zu sea keyframes (%zu at 0.137 s frames); the crate ends at "
              "(%.3f, %.3f, %.3f); %zu and %zu frames\n",
              fine.keyframes, coarse.keyframes, fine.end.position.x, fine.end.position.y,
              fine.end.position.z, fine.frames.size(), coarse.frames.size());
  EXPECT_GE(fine.keyframes, 2u) << "the front grew the sea";
  EXPECT_EQ(fine.keyframes, coarse.keyframes);
  std::map<std::uint64_t, std::size_t> by_tick;
  for (std::size_t k = 0; k < fine.frames.size(); ++k)
    by_tick[fine.frames[k].first] = k;
  int compared = 0, differ = 0;
  for (std::size_t k = 0; k < coarse.frames.size(); ++k) {
    const auto it = by_tick.find(coarse.frames[k].first);
    if (it == by_tick.end())
      continue;
    ++compared;
    const cvc::physics::body_state &a = fine.frames[it->second].second;
    const cvc::physics::body_state &b = coarse.frames[k].second;
    if (a.position.x != b.position.x || a.position.z != b.position.z ||
        a.linear_velocity.z != b.linear_velocity.z || fine.hashes[it->second] != coarse.hashes[k])
      ++differ;
  }
  EXPECT_GT(compared, 30);
  EXPECT_EQ(differ, 0) << "of " << compared << " ticks both looked at";
}

// A body asleep where a front arrives is woken, so the front pushes it (F6): the crate rests and
// sleeps in the 6 m/s wind, then the storm's front (time-lapsed by 15) reaches it.
TEST(AriadneSceneWeather, AFrontWakesASleepingBody) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const LoadResult r = storm_scene(-2400.0, 0.0, 15.0, true, kStormFloor);
  const storm_run run = run_storm(r, 1.0 / 60.0, 14.0);
  ASSERT_FALSE(run.frames.empty());
  // Where it rested, asleep, before the front came (by 9 s).
  cvc::app probe;
  const double dt = probe.world_clock().fixed_dt();
  double rest_x = 0.0;
  bool asleep = false;
  for (const auto &[tick, b] : run.frames)
    if (static_cast<double>(tick) * dt < 9.0) {
      rest_x = b.position.x;
      asleep = !b.active;
    }
  std::printf("[storm] a 10 kg crate: asleep at 9 s %s, at x %.3f; after the front, x %.3f\n",
              asleep ? "yes" : "no", rest_x, run.end.position.x);
  EXPECT_TRUE(asleep) << "the crate came to rest and slept in the 6 m/s wind";
  EXPECT_GT(run.end.position.x - rest_x, 0.5) << "the front woke it and pushed it downwind";
}

namespace {

// storm_scene's lake with a short crossfade (sea_fade 0.2 s, under the clock's 0.5 s period) and a
// short lag (sea_lag 1 s), so the front's wind would keyframe the sea in periods running.
const char *kStormLakeShortFade = R"(
scene:
  physics: {}
  wind: { speed: 6, dir: 0, gustiness: 0 }
  nodes:
    - node: terrain
      source:
        heightfield:
          size: 400
          resolution: 81
          layers:
            - { kind: dome, amplitude: -30, radius: 150 }
      body: static
    - node: water
      type: water
      ground: terrain
      level: 0
      wave_amp: 0.3
      sea_fade: 0.2
      sea_lag: 1
      body: {}
    - node: ball
      source: { box: { size: [1, 1, 0.5] } }
      transform: { position: [0, 0, 0] }
      body: { density: 500 }
)";

// The renderer's order: the world built from the bodies a host holds (SceneWorld::build), the
// water's runtime its own, ticked after the world follows each frame (cvcGL's water node ticks it
// after tick_scene's follow), against from_scene's, ticked in the world's sync before the steps.
storm_run run_drawn_order(const LoadResult &r, double frame_dt, double seconds) {
  storm_run out;
  cvc::app app;
  std::vector<std::string> warnings;
  seed_scene_wind(app, r.scene, "");
  std::map<const SceneNode *, std::array<double, 16>> matrices;
  scene_world_matrices(r.scene, matrices);
  std::vector<SceneBodyInput> inputs;
  std::map<std::string, std::shared_ptr<const cvc::height_field>> fields;
  std::unique_ptr<WaterRuntime> water;
  for (const SceneNode &n : r.scene.nodes) {
    if (!n.body.present)
      continue;
    SceneBodyInput in;
    in.spec = &n;
    in.world = matrices[&n];
    if (n.type == "water") {
      std::shared_ptr<const cvc::height_field> hf = fields.at(n.props.str("ground"));
      water = std::make_unique<WaterRuntime>(app, n, "", [hf] { return hf; });
      in.colliders = water->colliders();
    } else {
      cvc::geometry g;
      std::shared_ptr<const cvc::height_field> field;
      if (!read_scene_node_geometry(app, n, g, field, &warnings)) {
        ADD_FAILURE() << "no geometry for " << n.id;
        return out;
      }
      in.geometry = std::make_shared<const cvc::geometry>(std::move(g));
      if (field)
        fields[n.id] = field;
    }
    inputs.push_back(std::move(in));
  }
  std::unique_ptr<SceneWorld> w =
      SceneWorld::build(app, r.scene.physics, inputs, fields, "physics", &warnings);
  if (!w || !water) {
    ADD_FAILURE() << "no world or no water";
    return out;
  }
  w->bind_wind(r.scene, "");
  w->bind_weather(r.scene, "");
  SceneClockDriver clock(app, resolved_clock(r.scene));
  w->follow(app.world_clock());
  water->tick();
  while (app.world_clock().t() < seconds) {
    clock.drive(frame_dt);
    w->follow(app.world_clock());
    water->tick(); // after the steps, as a renderer's water node does
    out.frames.emplace_back(app.world_clock().tick(), w->world().state(w->body("ball")));
    out.hashes.push_back(w->world().state_hash());
  }
  out.end = w->world().state(w->body("ball"));
  if (water->keyframes())
    out.keyframes = water->keyframes()->keyframe;
  return out;
}

// How many of the ticks both runs looked at have a different body (position, vertical velocity).
int differing(const storm_run &a, const storm_run &b, int *compared) {
  std::map<std::uint64_t, std::size_t> by_tick;
  for (std::size_t k = 0; k < a.frames.size(); ++k)
    by_tick[a.frames[k].first] = k;
  int differ = 0;
  *compared = 0;
  for (std::size_t k = 0; k < b.frames.size(); ++k) {
    const auto it = by_tick.find(b.frames[k].first);
    if (it == by_tick.end())
      continue;
    ++*compared;
    const cvc::physics::body_state &x = a.frames[it->second].second, &y = b.frames[k].second;
    if (x.position.x != y.position.x || x.position.z != y.position.z ||
        x.linear_velocity.z != y.linear_velocity.z)
      ++differ;
  }
  return differ;
}

} // namespace

// A short crossfade (0.2 s) would let the sea keyframe in two periods running. The world's own
// water runtime ticks before the steps and a renderer's after them, so one would replace a
// keyframe's field before its step and the other put it in first. While the scene has storms the
// crossfade is at least two periods, so the renderer's order and the headless world float the
// crate alike, and their state hashes agree.
TEST(AriadneSceneWeather, ARendersSeaAndAHeadlessOneFloatAlikeWithAShortCrossfade) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const LoadResult r = storm_scene(-1200.0, 0.0, 15.0, true, kStormLakeShortFade);
  // Frames of 0.137 s (8 clock quanta, as the clock caps a frame), so a frame passes a period's
  // start with steps to spare past it: where the two orders part.
  const storm_run headless = run_storm(r, 0.137, 9.0);
  const storm_run drawn = run_drawn_order(r, 0.137, 9.0);
  int compared = 0;
  const int differ = differing(headless, drawn, &compared);
  int hashes_differ = 0;
  for (std::size_t k = 0; k < headless.frames.size() && k < drawn.frames.size(); ++k)
    if (headless.frames[k].first == drawn.frames[k].first && headless.hashes[k] != drawn.hashes[k])
      ++hashes_differ;
  std::printf("[storm] a 0.2 s crossfade: %llu sea keyframes headless, %llu in a renderer's order; "
              "the crate differs at %d of %d ticks, the state hash at %d\n",
              static_cast<unsigned long long>(headless.keyframes),
              static_cast<unsigned long long>(drawn.keyframes), differ, compared, hashes_differ);
  EXPECT_GE(headless.keyframes, 3u);
  EXPECT_EQ(headless.keyframes, drawn.keyframes);
  EXPECT_GT(compared, 100);
  EXPECT_EQ(differ, 0);
  EXPECT_EQ(hashes_differ, 0);
}

// The storm wind's peak is a live key, weather.gust_front (the Weather panel's Storm wind). A
// change reaches the physics at the start of the clock's next period after the scene's world reads
// it: so a run with a frame per clock quantum and one with 0.137 s frames switch at the same step
// and agree on the ball and the whole world's state hash at every tick both looked at; and 0 turns
// the fronts off, the air where a front blew the plain wind again.
TEST(AriadneSceneWeather, TheStormWindsPeakIsALiveKeySwitchedOnTheClock) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const LoadResult r = storm_scene(-600.0, 0.0, 1.0);
  cvc::app probe;
  const double q = probe.world_clock().fixed_dt();
  struct gust_run {
    storm_run run;
    std::vector<double> gust; // the weather's applied peak at each frame
    std::uint64_t read_tick = 0;
    bool storm_air_end = true;
    cvc::physics::vec3 air_end, plain_end;
  };
  const auto run = [&](double frame_dt, double to) {
    gust_run out;
    cvc::app app;
    std::vector<std::string> warnings;
    std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
    if (!w || w->weathers().size() != 1) {
      ADD_FAILURE() << "no world, or not one weather";
      return out;
    }
    EXPECT_EQ(val(app, "weather.gust_front"), "12"); // seeded from the prop's default
    SceneClockDriver clock(app, resolved_clock(r.scene));
    w->follow(app.world_clock());
    bool set = false;
    while (app.world_clock().t() < 4.0) {
      clock.drive(frame_dt);
      if (set && out.read_tick == 0)
        out.read_tick = app.world_clock().tick(); // the sync about to read it
      w->follow(app.world_clock());
      out.run.frames.emplace_back(app.world_clock().tick(), w->world().state(w->body("ball")));
      out.run.hashes.push_back(w->world().state_hash());
      out.gust.push_back(w->weathers()[0]->gust_front());
      if (!set && app.world_clock().t() >= 2.21) { // mid-frame for the 0.137 s run
        cvc::state::instance(app)("weather.gust_front").value(to);
        set = true;
      }
    }
    const cvc::physics::body_state b = w->world().state(w->body("ball"));
    out.storm_air_end = dynamic_cast<const StormWindFlow *>(w->wind().get()) != nullptr;
    out.air_end = w->wind()->velocity(b.position, w->world().field_time());
    out.plain_end = cvc::physics::wind_flow(cvc::algo::wind_state{6.0, 0.0, 0.0}, 0.0)
                        .velocity(b.position, w->world().field_time());
    return out;
  };
  const gust_run fine = run(q, 24.0), coarse = run(0.137, 24.0);
  ASSERT_FALSE(fine.gust.empty());
  ASSERT_FALSE(coarse.gust.empty());
  // Where it switched: the first step past the start of the period after the read.
  const std::uint64_t ticks = scene_clock_period_ticks(kSceneWindFrontPeriod, q);
  const std::uint64_t boundary = (fine.read_tick / ticks + 1) * ticks;
  std::uint64_t switched = 0;
  for (std::size_t k = 0; k < fine.gust.size(); ++k)
    if (fine.gust[k] == 24.0) {
      switched = fine.run.frames[k].first;
      break;
    }
  int compared = 0;
  const int differ = differing(fine.run, coarse.run, &compared);
  std::map<std::uint64_t, std::uint64_t> fine_hash;
  for (std::size_t k = 0; k < fine.run.frames.size(); ++k)
    fine_hash[fine.run.frames[k].first] = fine.run.hashes[k];
  int hashes_differ = 0;
  for (std::size_t k = 0; k < coarse.run.frames.size(); ++k)
    if (const auto it = fine_hash.find(coarse.run.frames[k].first);
        it != fine_hash.end() && it->second != coarse.run.hashes[k])
      ++hashes_differ;
  std::printf("[storm] weather.gust_front 12 -> 24 read at tick %llu (%llu at 0.137 s frames): in "
              "at tick %llu (the period's start %llu); the ball ends at %.2f m/s; %d of %d ticks "
              "differ, %d hashes\n",
              static_cast<unsigned long long>(fine.read_tick),
              static_cast<unsigned long long>(coarse.read_tick),
              static_cast<unsigned long long>(switched), static_cast<unsigned long long>(boundary),
              fine.run.frames.back().second.linear_velocity.x, differ, compared, hashes_differ);
  EXPECT_GT(switched, boundary);
  EXPECT_LE(switched, boundary + 2);
  EXPECT_GT(compared, 30);
  EXPECT_EQ(differ, 0);
  EXPECT_EQ(hashes_differ, 0);
  EXPECT_GT(fine.run.frames.back().second.linear_velocity.x, 22.0); // 6 + 24 where it is
  // 0: no fronts; the air where one blew is the plain wind.
  const gust_run off = run(q, 0.0);
  std::printf("[storm] weather.gust_front 0: the air at the ball %.4f m/s, the plain wind %.4f\n",
              off.air_end.x, off.plain_end.x);
  EXPECT_FALSE(off.storm_air_end);
  EXPECT_EQ(off.air_end.x, off.plain_end.x);
  EXPECT_EQ(off.air_end.y, off.plain_end.y);
  EXPECT_LT(off.run.frames.back().second.linear_velocity.x, 8.0); // slowing to the 6 m/s wind
}

// The water steps on the clock's periods only with storms that can blow: a scene wind and a weather
// node. Without a wind (no fronts) it steps each frame, as before. And the storms steer at the
// sky's base as the drawn node reads it: its <sky>.base key first (a --set or a kept key), else the
// document's cloud_sky base (none here: 300 m).
TEST(AriadneSceneWeather, TheSeaStepsOnTheClockOnlyWithAWindAndSteersAtTheSkysKey) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  for (const bool wind : {true, false}) {
    std::string doc = kStormLake;
    if (!wind) {
      const std::string line = "  wind: { speed: 6, dir: 0, gustiness: 0 }\n";
      ASSERT_NE(doc.find(line), std::string::npos);
      doc.erase(doc.find(line), line.size());
    }
    const LoadResult r = storm_scene(-1200.0, 0.0, 15.0, true, doc.c_str());
    cvc::app app;
    std::vector<std::string> warnings;
    std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "", &warnings);
    ASSERT_NE(w, nullptr);
    w->sync(&warnings);
    ASSERT_NE(w->water("water"), nullptr);
    EXPECT_EQ(w->water("water")->colliders()->clock_period, wind ? kSceneWindFrontPeriod : 0.0)
        << (wind ? "with" : "without") << " a scene wind";
  }
  // The sky's key, set before the scene loads, steers the headless storms.
  cvc::app app;
  cvc::state::instance(app)("sky.base").value(600.0);
  const LoadResult r = storm_scene(-1200.0, 0.0, 1.0);
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "", &warnings);
  ASSERT_NE(w, nullptr);
  ASSERT_EQ(w->weathers().size(), 1u);
  const double steer = w->weathers()[0]->model().params().steer_speed;
  std::printf("[storm] steered at sky.base 600: %.4f m/s (at 300: %.4f)\n", steer,
              6.0 * cvc::algo::wind_profile(300.0));
  EXPECT_DOUBLE_EQ(steer, 6.0 * cvc::algo::wind_profile(600.0));
}

// ----------------
// One world (ARIADNE_WORLDS.md section 18.2): a server holon runs a scene's world
// ----------------
namespace {

holon_config one_world_holon() {
  holon_config c;
  c.physics = true;
  return c;
}

// A car driven at 2 m/s^2 from (-40, 0) along +x for `seconds`, the world run by a server holon,
// with an avatar standing at (-25, 0) or none. Its final state, and the avatar's.
struct shove_run {
  entity_info car;
  cvc::physics::body_state avatar;
};
shove_run shove(bool with_avatar, double seconds) {
  LoadResult r = load_intents();
  EXPECT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  EXPECT_NE(w, nullptr);
  WorldVehicles &v = *w->vehicles();
  w->sync(&warnings);
  const std::uint64_t id = spawn_at(v, "external", -40.0, 0.0);
  entity_intent go;
  go.command = entity_intent::mode::replace;
  go.accel = 2.0;
  EXPECT_TRUE(v.set_intent(id, go));
  holon h(holon_role::server, one_world_holon());
  std::string why;
  EXPECT_TRUE(h.attach_world(w.get(), &why)) << why;
  cvc::physics::body_state feet;
  feet.position = {-25.0, 0.0, 0.0};
  if (with_avatar)
    h.add_participant("walker", false, feet);
  const int frames = static_cast<int>(std::lround(seconds * 60.0));
  for (int i = 0; i < frames; ++i)
    h.advance(1.0 / 60.0);
  shove_run out;
  out.car = entity(v, id);
  if (with_avatar)
    out.avatar = h.state("walker");
  return out;
}

} // namespace

// FLEETS_AND_NIGHT_ROADMAP.md FL0: what a fleet of world vehicles costs, headless, before fleets
// exist: N agents spawned by the gym and stepped one navigator tick at a time (nav@1 drives every
// one), on an open 400 m square, and on a city bundle when CVC_FLEET_BENCH_BUNDLE names one. Prints
// the spawn time and the mean tick, the map known and under fog. A benchmark, not a check: it runs
// only with CVC_FLEET_BENCH=1.
namespace {

std::string fleet_bench_doc(const std::string &bundle) {
  std::string doc = std::string("requires: [scene.physics]\n") + kGymVehicles;
  if (bundle.empty())
    return doc + R"(
scene:
  physics: {}
  world: { source: square, cell: 4 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [400, 400, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)";
  return doc + "data:\n  city: { uri: \"" + bundle + R"(", as: city_bundle }
scene:
  physics: {}
  world: { source: city, cell: 31.25 }
  nodes:
    - { node: city, source: { data: city }, world: city, body: {} }
)";
}

void fleet_bench(const std::string &doc, const std::string &plant, int n, const std::string &sense,
                 int ticks, bool avoid = true) {
  gym_options o;
  o.count = n;
  o.rays = 1;
  o.max_steps = 0;
  o.wander = true;
  o.drive_telemetry = false;
  o.set["vehicles.sense"] = sense;
  for (int i = 0; i < n; ++i) {
    entity_spec s;
    s.plant = plant;
    o.agents.push_back(s);
  }
  using clk = std::chrono::steady_clock;
  const auto t0 = clk::now();
  WorldGym gym(doc, o);
  gym.reset(1);
  const double spawn_ms = std::chrono::duration<double, std::milli>(clk::now() - t0).count();
  gym.vehicles().set_avoid(avoid);
  const std::vector<double> none(gym.agents(), kNaN);
  for (int i = 0; i < 10; ++i)
    gym.step(none, none);
  const auto t1 = clk::now();
  for (int i = 0; i < ticks; ++i)
    gym.step(none, none);
  const double tick_ms = std::chrono::duration<double, std::milli>(clk::now() - t1).count() / ticks;
  // Where a tick goes: the physics world's steps (the last 120 steps' wall time, six a tick), and
  // one publish of the entities' keys (timed here alone).
  double phys = 0.0;
  int nphys = 0;
  {
    std::stringstream in(
        cvc::ariadne::read_or<std::string>(gym.app(), "physics.stats.step_ms_history", ""));
    std::string v;
    while (std::getline(in, v, ','))
      if (!v.empty()) {
        phys += std::atof(v.c_str());
        ++nphys;
      }
  }
  const double phys_tick = nphys ? 6.0 * phys / nphys : 0.0;
  const auto t2 = clk::now();
  gym.vehicles().publish();
  const double pub_ms = std::chrono::duration<double, std::milli>(clk::now() - t2).count();
  // A tick is 0.06 s of world time: at 1x a tick under 60 ms keeps up, at the demos' 3.6x under
  // 16.7 ms.
  std::printf("[fleet] %-9s n=%4d sense=%-5s avoid=%d spawn %7.1f ms  tick %7.2f ms (%5.1fx)  "
              "physics %6.2f  publish %6.2f  rest %6.2f ms\n",
              plant.c_str(), n, sense.c_str(), avoid ? 1 : 0, spawn_ms, tick_ms,
              tick_ms > 0.0 ? 60.0 / tick_ms : 0.0, phys_tick, pub_ms,
              tick_ms - phys_tick - pub_ms);
}

// A fleet declared in the document (start: fleet:) on a paused clock: the load (spawn and layout),
// six steps, and one publish, with its per-entity keys (keys: all) or its summary alone.
void fleet_doc_bench(const std::string &bundle, int n, const std::string &keys, int ticks) {
  std::string doc = fleet_bench_doc(bundle);
  const std::string arch = "      humvee: { vehicle: humvee, controllers: [nav@1, none] }\n";
  doc.insert(doc.find(arch) + arch.size(),
             "    start:\n      - { fleet: city, archetype: humvee, plant: kinematic, count: " +
                 std::to_string(n) +
                 ", spawn: opposed, seed: 7, rally: { count: 16 }, keys: " + keys + " }\n");
  doc.insert(doc.find("scene:\n") + 7, "  clock: { paused: true }\n");
  const fs::path path = write_doc("fleet_doc_bench", doc);
  using clk = std::chrono::steady_clock;
  cvc::app app;
  AppRuntime art(app, "");
  const auto t0 = clk::now();
  art.load(path.string());
  art.drain();
  const double load_ms = std::chrono::duration<double, std::milli>(clk::now() - t0).count();
  if (!art.scene_world() || !art.scene_world()->vehicles()) {
    std::printf("[fleet doc] no world\n");
    return;
  }
  WorldVehicles &v = *art.scene_world()->vehicles();
  for (int i = 0; i < 10; ++i) {
    art.step_world(6);
    art.drain();
  }
  const auto t1 = clk::now();
  for (int i = 0; i < ticks; ++i) {
    art.step_world(6);
    art.drain();
  }
  const double tick_ms = std::chrono::duration<double, std::milli>(clk::now() - t1).count() / ticks;
  const auto t2 = clk::now();
  v.publish();
  const double pub_ms = std::chrono::duration<double, std::milli>(clk::now() - t2).count();
  std::printf("[fleet doc] kinematic n=%4d keys=%-7s load %7.1f ms  6 steps %7.2f ms  publish "
              "%6.2f ms  (%zu entities, %s)\n",
              n, keys.c_str(), load_ms, tick_ms, pub_ms, v.size(),
              cvc::ariadne::read_or<std::string>(app, "vehicles.fleets.city.status", "").c_str());
}

} // namespace

// A car and an avatar in one world: the car, driven into a standing avatar, shoves it down the
// road (the walker can't hold a 70 kg body against a Humvee), and the avatar takes some of the
// car's momentum, so the car ends a little behind the same run without it. Both bodies are in the
// scene's physics world, stepped on the holon's clock; the scene's square is what the avatar
// stands on (no room statics were given).
TEST(AriadneOneWorld, ACarShovesAnAvatarAndTheAvatarSlowsTheCar) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  // Before the car arrives (about 3.6 s): the avatar stands on the scene's square, where it was.
  const shove_run early = shove(true, 2.0);
  EXPECT_NEAR(early.avatar.position.z, 0.0, 0.05) << "on the square";
  EXPECT_LT(std::hypot(early.avatar.position.x + 25.0, early.avatar.position.y), 0.05);
  const shove_run hit = shove(true, 6.0);
  const shove_run clear = shove(false, 6.0);
  std::printf("  car at 6 s: x %.3f, %.3f m/s with the avatar in its way, x %.3f, %.3f m/s "
              "without; the avatar ends at (%.2f, %.2f), %.2f m/s\n",
              hit.car.x, hit.car.speed, clear.car.x, clear.car.speed, hit.avatar.position.x,
              hit.avatar.position.y,
              std::hypot(hit.avatar.linear_velocity.x, hit.avatar.linear_velocity.y));
  EXPECT_GT(hit.avatar.position.x, -25.0 + 2.0) << "shoved down the road";
  EXPECT_GT(clear.car.x, -10.0) << "the car drove past where the avatar stood";
  EXPECT_LT(hit.car.x, clear.car.x) << "the avatar took some of the car's momentum";
}

// What attach_world takes: a server holon with physics and a world on its quantum. A client, a
// holon without physics, or a world on another step is refused and the holon is as it was;
// detaching puts the avatars back in a world of the holon's own, where they stood.
TEST(AriadneOneWorld, AttachTakesAServerWorldOnTheHolonsQuantum) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_intents();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  std::string why;
  holon client(holon_role::client, one_world_holon());
  EXPECT_FALSE(client.attach_world(w.get(), &why));
  EXPECT_NE(why.find("client"), std::string::npos) << why;
  holon kinematic(holon_role::server, holon_config{});
  EXPECT_FALSE(kinematic.attach_world(w.get(), &why));
  EXPECT_NE(why.find("physics"), std::string::npos) << why;
  holon_config slow = one_world_holon();
  slow.clock.fixed_dt = 1.0 / 60.0;
  holon other(holon_role::server, slow);
  EXPECT_FALSE(other.attach_world(w.get(), &why));
  EXPECT_NE(why.find("quantum"), std::string::npos) << why;
  EXPECT_EQ(other.attached_world(), nullptr);

  holon h(holon_role::server, one_world_holon());
  cvc::physics::body_state feet;
  feet.position = {3.0, 4.0, 0.0};
  h.add_participant("a", false, feet);
  const std::size_t before = w->world().body_count();
  ASSERT_TRUE(h.attach_world(w.get(), &why)) << why;
  EXPECT_EQ(h.attached_world(), w.get());
  EXPECT_EQ(w->world().body_count(), before + 1) << "the avatar is a body in the scene's world";
  for (int i = 0; i < 60; ++i)
    h.advance(1.0 / 60.0);
  const cvc::physics::body_state there = h.state("a");
  EXPECT_NEAR(there.position.z, 0.0, 0.05) << "standing on the scene's square";
  ASSERT_TRUE(h.attach_world(nullptr, &why)) << why;
  EXPECT_EQ(h.attached_world(), nullptr);
  EXPECT_EQ(w->world().body_count(), before) << "out of the scene's world again";
  EXPECT_LT(std::hypot(h.state("a").position.x - there.position.x,
                       h.state("a").position.y - there.position.y),
            1e-9)
      << "back in its own world where it stood";
}

namespace {
// A navigated car driving west past an avatar standing in its lane, the world run by a server
// holon: the closest the car's centre came to the avatar, where the car ended, and how far the
// avatar was moved.
struct mover_pass {
  double closest = 1e9, car_x = 0.0, avatar_moved = 0.0;
};
mover_pass pass_avatar(bool avoid) {
  LoadResult r = load_vehicles(kAvoid);
  EXPECT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  mover_pass out;
  if (!w)
    return out;
  WorldVehicles *v = w->vehicles();
  v->set_avoid(avoid);
  v->set_wander(false);
  w->sync(&warnings);
  entity_spec a;
  a.archetype = "car";
  a.placed = true;
  a.x = 25.0;
  a.y = 0.5;
  a.yaw = 3.14159265358979323846;
  const std::uint64_t id = v->spawn(a);
  EXPECT_NE(id, 0u);
  v->set_goal(id, -30.0, 0.5);
  holon h(holon_role::server, one_world_holon());
  std::string why;
  EXPECT_TRUE(h.attach_world(w.get(), &why)) << why;
  cvc::physics::body_state feet;
  feet.position = {0.0, 0.5, 0.0};
  h.add_participant("walker", false, feet);
  for (int i = 0; i < 900; ++i) {
    h.advance(1.0 / 60.0);
    const entity_info e = entity(*v, id);
    const cvc::physics::body_state s = h.state("walker");
    if (e.placed) {
      out.closest = std::min(out.closest, std::hypot(e.x - s.position.x, e.y - s.position.y));
      out.car_x = e.x;
    }
  }
  const cvc::physics::body_state end = h.state("walker");
  out.avatar_moved = std::hypot(end.position.x, end.position.y - 0.5);
  return out;
}
} // namespace

// The navigators steer around the avatars (section 18.4): a car on nav@1 swings past a person
// standing in its lane and drives on to its goal, where without avoiding it drives into them.
TEST(AriadneOneWorld, ANavigatedCarSteersAroundAnAvatar) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  const mover_pass on = pass_avatar(true), off = pass_avatar(false);
  std::printf("  past an avatar: avoiding, closest %.2f m, the car ends at x %.1f, the avatar "
              "moved %.2f m; not avoiding, %.2f m, x %.1f, moved %.2f m\n",
              on.closest, on.car_x, on.avatar_moved, off.closest, off.car_x, off.avatar_moved);
  EXPECT_LT(on.car_x, -20.0) << "the car got past";
  EXPECT_GT(on.closest, 2.0) << "with room to spare";
  EXPECT_LT(on.avatar_moved, 0.1) << "and never touched the avatar";
  EXPECT_GT(off.avatar_moved, on.avatar_moved + 0.5) << "without avoiding, it drove into them";
}

// Generic entities (section 18.4): a participant gets into a vehicle and drives it with its
// intent's drive axes; its person leaves the world and it is where the vehicle is; the entity
// table and the room's entities say who drives what; getting out puts the person beside the
// vehicle and gives the vehicle back to its controller. Refused: a client, an unknown entity,
// one someone else drives.
TEST(AriadneOneWorld, AParticipantGetsIntoAVehicleAndDrivesIt) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_intents();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  WorldVehicles &v = *w->vehicles();
  w->sync(&warnings);
  const std::uint64_t id = spawn_at(v, "external", -40.0, 0.0);
  holon h(holon_role::server, one_world_holon());
  std::string why;
  ASSERT_TRUE(h.attach_world(w.get(), &why)) << why;
  cvc::physics::body_state feet;
  feet.position = {-38.0, 6.0, 0.0};
  h.add_participant("ann", false, feet);
  h.add_participant("bob", false, cvc::physics::body_state{});
  for (int i = 0; i < 30; ++i)
    h.advance(1.0 / 60.0);
  const std::size_t bodies = w->world().body_count();
  const entity_info parked = entity(v, id);

  holon client(holon_role::client, one_world_holon());
  EXPECT_FALSE(client.set_driver("ann", id, &why));
  EXPECT_FALSE(h.set_driver("ann", 99, &why));
  EXPECT_FALSE(h.set_driver("nobody", id, &why));
  ASSERT_TRUE(h.set_driver("ann", id, &why)) << why;
  EXPECT_EQ(h.drives("ann"), id);
  EXPECT_EQ(v.driver(id), "ann");
  EXPECT_FALSE(h.set_driver("bob", id, &why)) << "someone else drives it";
  EXPECT_NE(why.find("ann"), std::string::npos) << why;
  EXPECT_EQ(w->world().body_count(), bodies - 1) << "ann's person left the world";
  const std::vector<conference::room_entity> wire = conference::room_entities_of(v);
  ASSERT_EQ(wire.size(), 1u);
  EXPECT_EQ(wire[0].driver, "ann") << "the room's entities say who drives it";

  // Ann's drive axes drive it: full throttle straight ahead for 3 s.
  avatar_intent go;
  go.steer = 0.0;
  go.throttle = 1.0;
  h.set_intent("ann", go);
  for (int i = 0; i < 180; ++i)
    h.advance(1.0 / 60.0);
  const entity_info driven = entity(v, id);
  EXPECT_GT(driven.x - parked.x, 4.0) << "it drove";
  EXPECT_GT(driven.speed, 2.0);
  const cvc::physics::body_state ann = h.state("ann");
  EXPECT_LT(std::hypot(ann.position.x - driven.x, ann.position.y - driven.y), 0.5)
      << "ann is where the vehicle is";

  bool saw_person = false, saw_vehicle = false;
  for (const holon::entity &e : h.entities()) {
    if (e.id == "p:ann") {
      saw_person = true;
      EXPECT_EQ(e.kind, "person");
      EXPECT_FALSE(e.in_world);
    }
    if (e.id == "e" + std::to_string(id)) {
      saw_vehicle = true;
      EXPECT_EQ(e.kind, "humvee");
      EXPECT_EQ(e.driver, "participant:ann");
      EXPECT_EQ(e.owner, "ann");
    }
  }
  EXPECT_TRUE(saw_person && saw_vehicle);

  // Out: beside the vehicle, standing, and the vehicle back to its controller (external: with no
  // intent it brakes to a stop).
  ASSERT_TRUE(h.set_driver("ann", 0, &why)) << why;
  EXPECT_EQ(h.drives("ann"), 0u);
  EXPECT_EQ(v.driver(id), "");
  EXPECT_EQ(w->world().body_count(), bodies);
  for (int i = 0; i < 120; ++i)
    h.advance(1.0 / 60.0);
  const entity_info after = entity(v, id);
  const cvc::physics::body_state out = h.state("ann");
  const double d = std::hypot(out.position.x - after.x, out.position.y - after.y);
  std::printf("  drove %.1f m to %.2f m/s; out at %.2f m from the vehicle, which went on to %.1f m "
              "at %.2f m/s\n",
              driven.x - parked.x, driven.speed, d, after.x - parked.x, after.speed);
  EXPECT_GT(d, 1.0) << "beside it, not in it";
  EXPECT_LT(d, 6.0);
  EXPECT_NEAR(out.position.z, 0.0, 0.1) << "standing on the square";
  EXPECT_LT(after.speed, 0.5) << "its controller (external) brought it to rest";
}

// The drive axes are shares of what the vehicle can do (section 18.6), so one scheme drives any
// vehicle: steer of its lock; throttle of its acceleration; a brake toward a stop and never past
// it; reverse, a car braking while it still rolls forward and backing up from there, a boat's
// propeller astern; a boat has no brake.
TEST(AriadneOneWorld, TheDriveAxesAreSharesOfTheVehiclesEnvelope) {
  holon::drive_envelope car;
  car.max_steer = 0.5;
  car.max_accel = 3.0;
  car.max_decel = 7.0;
  holon::drive_envelope boat = car;
  boat.floats = true;
  boat.max_decel = 1.5; // its propeller astern
  const double dt = 7.0 / 120.0;
  const auto cmd = [&](const holon::drive_envelope &e, double steer, double throttle, double brake,
                       double speed) {
    avatar_intent in;
    in.steer = steer;
    in.throttle = throttle;
    in.brake = brake;
    return holon::drive_command(in, e, speed, dt);
  };
  EXPECT_DOUBLE_EQ(cmd(car, 1.0, 0.0, 0.0, 0.0).steer, 0.5);
  EXPECT_DOUBLE_EQ(cmd(car, -0.5, 0.0, 0.0, 0.0).steer, -0.25);
  EXPECT_DOUBLE_EQ(cmd(car, 0.0, 1.0, 0.0, 5.0).accel, 3.0);
  EXPECT_DOUBLE_EQ(cmd(car, 0.0, 0.5, 0.0, 5.0).accel, 1.5);
  // The brake: its share of max_decel at speed, only what stops it near a stop, nothing at rest.
  EXPECT_DOUBLE_EQ(cmd(car, 0.0, 0.0, 1.0, 10.0).accel, -7.0);
  EXPECT_DOUBLE_EQ(cmd(car, 0.0, 0.0, 0.5, 10.0).accel, -3.5);
  EXPECT_NEAR(cmd(car, 0.0, 0.0, 1.0, 0.1).accel, -0.1 / dt, 1e-12);
  EXPECT_DOUBLE_EQ(cmd(car, 0.0, 0.0, 1.0, 0.0).accel, 0.0) << "never past a stop";
  EXPECT_NEAR(cmd(car, 0.0, 0.0, 1.0, -0.2).accel, 0.2 / dt, 1e-12) << "rolling back, it stops too";
  EXPECT_DOUBLE_EQ(cmd(car, 0.0, 1.0, 1.0, 10.0).accel, -7.0) << "the brake wins";
  // Reverse: braking while it rolls forward, then backing up.
  EXPECT_DOUBLE_EQ(cmd(car, 0.0, -1.0, 0.0, 5.0).accel, -7.0);
  EXPECT_DOUBLE_EQ(cmd(car, 0.0, -1.0, 0.0, 0.2).accel, -3.0);
  EXPECT_DOUBLE_EQ(cmd(car, 0.0, -1.0, 0.0, -2.0).accel, -3.0);
  // A boat: no brake; reverse is the propeller astern, whatever its speed.
  EXPECT_DOUBLE_EQ(cmd(boat, 0.0, 0.0, 1.0, 5.0).accel, 0.0);
  EXPECT_DOUBLE_EQ(cmd(boat, 0.0, -1.0, 0.0, 5.0).accel, -1.5);
  EXPECT_DOUBLE_EQ(cmd(boat, 0.0, -1.0, 0.0, 0.0).accel, -1.5);
  // None is none: 0.
  const holon::drive_cmd none = holon::drive_command(avatar_intent{}, car, 3.0, dt);
  EXPECT_DOUBLE_EQ(none.steer, 0.0);
  EXPECT_DOUBLE_EQ(none.accel, 0.0);
}

namespace {
// kIntents' square, with a drivable humvee (reach 3 m) and a truck no one may drive.
const char *kDrivable = R"(
world:
  vehicles:
    archetypes:
      humvee:
        vehicle: humvee
        controllers: [nav@1, none, external]
        drivable: true
      truck:
        vehicle: humvee
        controllers: [external]
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [160, 160, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)";

LoadResult load_drivable() {
  cvc::app app;
  register_world_block(app);
  LoadResult r = load_string(app, kDrivable);
  if (r.ok)
    if (const Value *world = find_world_block(r)) {
      world_rules rules = default_world_rules();
      EXPECT_EQ(parse_world_block(*world, rules), "");
      r.scene.vehicles = rules.vehicles;
    }
  return r;
}

std::uint64_t spawn_kind(WorldVehicles &v, const std::string &archetype, double x, double y) {
  entity_spec s;
  s.archetype = archetype;
  s.controller = "external";
  s.placed = true;
  s.x = x;
  s.y = y;
  s.yaw = 0.0;
  std::string why;
  const std::uint64_t id = v.spawn(s, &why);
  EXPECT_NE(id, 0u) << why;
  return id;
}
} // namespace

// A driven car (section 18.6): full throttle, then the brake brings it to a stop and holds it
// there (the brake never reverses); reverse backs it up.
TEST(AriadneOneWorld, ACarBrakesToAStopAndOnlyReverseBacksItUp) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_drivable();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  WorldVehicles &v = *w->vehicles();
  w->sync(&warnings);
  const std::uint64_t id = spawn_kind(v, "humvee", -40.0, 0.0);
  holon h(holon_role::server, one_world_holon());
  std::string why;
  ASSERT_TRUE(h.attach_world(w.get(), &why)) << why;
  cvc::physics::body_state feet;
  feet.position = {-40.0, 3.0, 0.0};
  h.add_participant("ann", false, feet);
  for (int i = 0; i < 30; ++i)
    h.advance(1.0 / 60.0);
  ASSERT_TRUE(h.get_in("ann", id, &why)) << why;
  const auto drive = [&](double throttle, double brake, double seconds) {
    avatar_intent in;
    in.steer = 0.0;
    in.throttle = throttle;
    in.brake = brake;
    h.set_intent("ann", in);
    for (int i = 0; i < static_cast<int>(seconds * 60.0); ++i)
      h.advance(1.0 / 60.0);
    return entity(v, id);
  };
  const entity_info going = drive(1.0, 0.0, 3.0);
  EXPECT_GT(going.speed, 3.0);
  const entity_info braked = drive(0.0, 1.0, 3.0);
  EXPECT_LT(std::fabs(braked.speed), 0.2) << "the brake stopped it";
  const entity_info held = drive(0.0, 1.0, 2.0);
  EXPECT_GT(held.x, braked.x - 0.1) << "and held it: the brake never backs up";
  const entity_info backed = drive(-1.0, 0.0, 3.0);
  std::printf("  braked from %.2f m/s to %.2f; held %.2f m; reverse: %.2f m/s, %.1f m back\n",
              going.speed, braked.speed, held.x - braked.x, backed.speed, held.x - backed.x);
  EXPECT_LT(backed.speed, -0.5) << "reverse backs it up";
  EXPECT_LT(backed.x, held.x - 1.0);
}

// Getting in is the world's to rule on (section 18.6): a participant asks in the room tree
// (request_drive) and the relay's server_pump answers. A vehicle too far away, or one whose
// archetype is not drivable, is refused with why; a drivable one within reach lets them in, and
// the room's entities say which may be driven.
TEST(AriadneOneWorld, GettingInIsAskedForInTheRoomAndRuledOn) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_drivable();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  WorldVehicles &v = *w->vehicles();
  w->sync(&warnings);
  const std::uint64_t far_off = spawn_kind(v, "humvee", -40.0, 0.0);
  const std::uint64_t truck = spawn_kind(v, "truck", 4.0, 8.0);
  const std::uint64_t close =
      spawn_kind(v, "humvee", 6.5, 0.0); // its rear axle: the box from ~6.1 m
  holon h(holon_role::server, one_world_holon());
  std::string why;
  ASSERT_TRUE(h.attach_world(w.get(), &why)) << why;
  const std::string room = "garage";
  conference::room_join(app, room, "ann"); // the first joiner stands at (4, 0)
  const auto pump = [&](int frames) {
    for (int i = 0; i < frames; ++i)
      server_pump(h, app, room, 1.0 / 60.0);
  };
  pump(30);
  const auto ask = [&](std::uint64_t entity) {
    const std::uint64_t seq = request_drive(app, room, "ann", entity);
    pump(1);
    const drive_status st = read_drive_status(app, room, "ann");
    EXPECT_EQ(st.seq, seq);
    return st;
  };
  drive_status st = ask(far_off);
  EXPECT_FALSE(st.ok);
  EXPECT_NE(st.why.find("m away"), std::string::npos) << st.why;
  st = ask(truck);
  EXPECT_FALSE(st.ok);
  EXPECT_NE(st.why.find("not drivable"), std::string::npos) << st.why;
  EXPECT_EQ(read_driving(app, room, "ann"), 0u);
  st = ask(close);
  EXPECT_TRUE(st.ok) << st.why;
  EXPECT_EQ(h.drives("ann"), close);
  EXPECT_EQ(read_driving(app, room, "ann"), close);
  st = ask(0);
  EXPECT_TRUE(st.ok);
  EXPECT_EQ(read_driving(app, room, "ann"), 0u);

  bool saw_humvee = false, saw_truck = false;
  for (const conference::room_entity &e : conference::room_entities_of(v)) {
    if (e.id == close) {
      saw_humvee = true;
      EXPECT_TRUE(e.drivable);
      EXPECT_DOUBLE_EQ(e.reach, 3.0);
      EXPECT_EQ(e.controls, "car");
    }
    if (e.id == truck) {
      saw_truck = true;
      EXPECT_FALSE(e.drivable);
    }
  }
  EXPECT_TRUE(saw_humvee && saw_truck);
}

// An avatar that is a vehicle (section 18.6, OW6c): a participant asks for an archetype that is
// one, and the server spawns that vehicle where its person stands and seats it there for good; it
// drives it, can't get out or into another, and asking for a person puts it back on its feet
// beside it while the vehicle goes. Where the first archetype is a vehicle, a newcomer is one
// without asking, and its vehicle goes when it leaves.
TEST(AriadneOneWorld, AnAvatarCanBeAVehicle) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_drivable();
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  WorldVehicles &v = *w->vehicles();
  w->sync(&warnings);
  const std::uint64_t other = spawn_kind(v, "humvee", -40.0, 0.0);
  holon h(holon_role::server, one_world_holon());
  std::string why;
  ASSERT_TRUE(h.attach_world(w.get(), &why)) << why;
  avatar_archetype person, racer;
  person.id = "person";
  racer.id = "racer";
  racer.vehicle = "humvee";
  h.set_archetypes({person, racer});
  const std::string room = "garage";
  conference::room_join(app, room, "ann"); // the first joiner stands at (4, 0)
  const auto pump = [&](int frames) {
    for (int i = 0; i < frames; ++i)
      server_pump(h, app, room, 1.0 / 60.0);
  };
  const auto entity = [&](std::uint64_t id, entity_info &out) {
    for (const entity_info &e : v.entities())
      if (e.id == id) {
        out = e;
        return true;
      }
    return false;
  };
  pump(30);
  EXPECT_EQ(h.embodies("ann"), 0u) << "a person until it asks";
  EXPECT_FALSE(read_archetype_status(app, room, "ann").answered) << "and nothing is said";
  const cvc::physics::vec3 stood = h.state("ann").position;

  request_archetype(app, room, "ann", "racer");
  pump(1);
  archetype_status st = read_archetype_status(app, room, "ann");
  EXPECT_TRUE(st.answered && st.ok) << st.why;
  EXPECT_EQ(st.archetype, "racer");
  const std::uint64_t body = h.embodies("ann");
  ASSERT_NE(body, 0u);
  EXPECT_EQ(h.drives("ann"), body);
  EXPECT_EQ(read_driving(app, room, "ann"), body);
  EXPECT_EQ(h.archetype_of("ann"), "racer");
  entity_info e;
  ASSERT_TRUE(entity(body, e));
  EXPECT_EQ(e.spec.archetype, "humvee");
  EXPECT_LT(std::hypot(e.x - stood.x, e.y - stood.y), 3.0) << "spawned where it stood";
  bool tagged = false;
  for (const conference::room_entity &re : conference::room_entities_of(v))
    tagged = tagged || (re.id == body && re.avatar == "ann");
  EXPECT_TRUE(tagged) << "the room's entities say whose body it is";

  // It drives it.
  avatar_intent go;
  go.throttle = 1.0;
  write_intent(app, room, "ann", go);
  pump(90);
  ASSERT_TRUE(entity(body, e));
  EXPECT_GT(std::hypot(e.x - stood.x, e.y - stood.y), 2.0) << "under way";
  go.throttle = 0.0;
  go.brake = 1.0;
  write_intent(app, room, "ann", go);

  // No getting out, or into another.
  std::uint64_t seq = request_drive(app, room, "ann", 0);
  pump(1);
  drive_status ds = read_drive_status(app, room, "ann");
  EXPECT_EQ(ds.seq, seq);
  EXPECT_FALSE(ds.ok);
  EXPECT_NE(ds.why.find("you are your vehicle"), std::string::npos) << ds.why;
  seq = request_drive(app, room, "ann", other);
  pump(1);
  ds = read_drive_status(app, room, "ann");
  EXPECT_FALSE(ds.ok);
  EXPECT_EQ(h.drives("ann"), body);

  // An archetype the world has not: refused, and it stays as it is.
  request_archetype(app, room, "ann", "dragon");
  pump(1);
  st = read_archetype_status(app, room, "ann");
  EXPECT_FALSE(st.ok);
  EXPECT_NE(st.why.find("no avatar archetype 'dragon'"), std::string::npos) << st.why;
  EXPECT_EQ(h.embodies("ann"), body);

  // A person again: on its feet beside the vehicle, which goes.
  ASSERT_TRUE(entity(body, e));
  request_archetype(app, room, "ann", "person");
  pump(1);
  st = read_archetype_status(app, room, "ann");
  EXPECT_TRUE(st.ok) << st.why;
  EXPECT_EQ(h.embodies("ann"), 0u);
  EXPECT_EQ(h.drives("ann"), 0u);
  EXPECT_EQ(read_driving(app, room, "ann"), 0u);
  EXPECT_FALSE(entity(body, e)) << "its vehicle went";
  const cvc::physics::vec3 feet = h.state("ann").position;
  EXPECT_LT(std::hypot(feet.x - e.x, feet.y - e.y), 4.0) << "beside where it was";

  // A world whose first archetype is a vehicle: a newcomer is one without asking.
  h.set_archetypes({racer, person});
  conference::room_join(app, room, "bob");
  pump(2);
  const std::uint64_t bobs = h.embodies("bob");
  ASSERT_NE(bobs, 0u);
  EXPECT_EQ(read_driving(app, room, "bob"), bobs);
  st = read_archetype_status(app, room, "bob");
  EXPECT_TRUE(st.answered && st.ok) << st.why;
  EXPECT_EQ(h.embodies("ann"), 0u) << "ann asked to be a person";
  seq = request_drive(app, room, "ann", bobs);
  pump(1);
  EXPECT_FALSE(read_drive_status(app, room, "ann").ok) << "no one gets into bob";
  conference::room_leave(app, room, "bob");
  pump(2);
  EXPECT_FALSE(entity(bobs, e)) << "bob's vehicle goes with bob";
}

TEST(AriadneWorldGym, FleetBench) {
  REQUIRE_YAML();
  if (!std::getenv("CVC_FLEET_BENCH"))
    GTEST_SKIP() << "a benchmark: set CVC_FLEET_BENCH=1";
  const char *bundle = std::getenv("CVC_FLEET_BENCH_BUNDLE");
  const std::string doc = write_doc("fleet_bench", fleet_bench_doc(bundle ? bundle : "")).string();
  for (const char *sense : {"known", "fog"})
    for (const int n : {100, 400, 800})
      fleet_bench(doc, "kinematic", n, sense, 100);
  fleet_bench(doc, "kinematic", 800, "known", 100, /*avoid=*/false);
  if (jolt())
    for (const int n : {60, 120, 240})
      fleet_bench(doc, "jolt", n, "known", 100);
  for (const int n : {800, 2000})
    for (const char *keys : {"all", "summary"})
      fleet_doc_bench(bundle ? bundle : "", n, keys, 50);
}

TEST(AriadneWorldGym, KinematicBoxesTouchWhenTheyOverlap) {
  REQUIRE_YAML();
  // Two kinematic boxes make no contact in the physics world: entities() finds their overlaps
  // (in a grid as wide as the largest box). Two placed nose to tail overlapping, a third alone, and
  // a pair side by side across a cell's edge.
  gym_options o;
  o.max_steps = 0;
  const auto at = [](double x, double y, double yaw) {
    entity_spec s;
    s.plant = "kinematic";
    s.controller = "none";
    s.placed = true;
    s.x = x;
    s.y = y;
    s.yaw = yaw;
    return s;
  };
  o.agents = {at(0.0, 0.0, 0.0), at(3.0, 0.5, 0.3), at(40.0, 40.0, 0.0), at(-20.0, 9.9, 0.0),
              at(-20.0, 11.6, 0.0)};
  WorldGym gym(write_doc("kinematic_touch", gym_doc(false, false)).string(), o);
  gym.reset(1);
  const std::vector<entity_info> es = gym.vehicles().entities();
  ASSERT_EQ(es.size(), 5u);
  EXPECT_TRUE(es[0].vehicle_contact);
  EXPECT_TRUE(es[1].vehicle_contact);
  EXPECT_FALSE(es[2].vehicle_contact);
  EXPECT_TRUE(es[3].vehicle_contact); // 1.7 m apart, side by side: a Humvee is 2.2 m wide
  EXPECT_TRUE(es[4].vehicle_contact);
}

namespace {
// kDrivable's square with the humvee on the kinematic plant, as traffic.ari's trucks are.
const char *kKinematicDrivable = R"(
world:
  vehicles:
    archetypes:
      humvee:
        vehicle: humvee
        plant: kinematic
        controllers: [nav@1, none, external]
        drivable: true
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [160, 160, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)";

// The command the relay sets after a step for a driver's next navigator tick (holon's drive).
void command_after(cvc::ariadne::kinematic_vehicle &m, const cvc::ariadne::kinematic_drive &k,
                   const avatar_intent &in, double dt) {
  holon::drive_envelope env;
  env.max_steer = k.max_steer;
  env.max_accel = k.max_accel;
  env.max_decel = k.max_decel;
  const double speed = m.vx * std::cos(m.box[3]) + m.vy * std::sin(m.box[3]);
  const holon::drive_cmd c =
      holon::drive_command(in, env, speed, static_cast<double>(k.tick_steps) * dt);
  m.steer_cmd = c.steer;
  m.accel_cmd = c.accel;
}
} // namespace

// The vehicle a client drives, run as the relay's world runs it (ARIADNE_WORLDS.md section 18.5,
// OW5b): on the kinematic plant, the room's entities carry its numbers and the clock ticks its
// navigator ticks on (WorldVehicles::kinematics), and a model stepped on the same drive axes at
// the same ticks (kinematic_vehicle_step, the command after each step) stays on the relay's
// vehicle through throttle, a turn, the brake and reverse.
TEST(AriadneOneWorld, AClientRunsTheVehicleItDrivesAsTheWorldDoes) {
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  cvc::app app;
  register_world_block(app);
  LoadResult r = load_string(app, kKinematicDrivable);
  ASSERT_TRUE(r.ok) << r.error;
  {
    const Value *world = find_world_block(r);
    ASSERT_NE(world, nullptr);
    world_rules rules = default_world_rules();
    ASSERT_EQ(parse_world_block(*world, rules), "");
    r.scene.vehicles = rules.vehicles;
  }
  std::vector<std::string> warnings;
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", "physics", &warnings);
  ASSERT_NE(w, nullptr);
  WorldVehicles &v = *w->vehicles();
  w->sync(&warnings);
  const std::uint64_t id = spawn_kind(v, "humvee", -40.0, 0.0);
  holon h(holon_role::server, one_world_holon());
  std::string why;
  ASSERT_TRUE(h.attach_world(w.get(), &why)) << why;
  cvc::physics::body_state feet;
  feet.position = {-40.0, 3.0, 0.0};
  h.add_participant("ann", false, feet);
  for (int i = 0; i < 30; ++i)
    h.advance(1.0 / 60.0);
  ASSERT_TRUE(h.get_in("ann", id, &why)) << why;
  const double dt = h.clock().fixed_dt();
  for (int i = 0; i < 20; ++i) // the driver's command set, the navigator ticked on a clock tick
    h.advance(dt);

  cvc::ariadne::kinematic_drive k;
  ASSERT_TRUE(v.kinematics(id, k));
  EXPECT_TRUE(k.valid());
  EXPECT_EQ(k.tick_steps, v.steps_per_tick());
  bool carried = false;
  for (const conference::room_entity &e : conference::room_entities_of(v))
    if (e.id == id) {
      carried = true;
      EXPECT_EQ(e.drive, k) << "the room's entities carry it";
    }
  EXPECT_TRUE(carried);

  // From where the relay has it (at rest: exact), on the same drive axes at each tick.
  cvc::ariadne::kinematic_vehicle m = cvc::ariadne::kinematic_vehicle_at(k, h.state("ann"),
                                                                         h.tick(), dt);
  const auto axes = [](int step) {
    avatar_intent in;
    in.steer = 0.0;
    in.throttle = 0.0;
    in.brake = 0.0;
    if (step < 240) {
      in.throttle = 1.0;
    } else if (step < 480) {
      in.throttle = 1.0;
      in.steer = 0.5;
    } else if (step < 600) {
      in.throttle = 0.5;
      in.steer = -1.0;
    } else if (step < 840) {
      in.brake = 1.0;
    } else {
      in.throttle = -1.0;
    }
    return in;
  };
  double worst = 0.0, top = 0.0;
  const cvc::physics::vec3 start = h.state("ann").position;
  for (int step = 0; step < 1080; ++step) {
    const avatar_intent in = axes(step);
    h.set_intent("ann", in);
    const std::uint64_t before = h.tick();
    h.advance(dt);
    for (std::uint64_t t = before + 1; t <= h.tick(); ++t) {
      cvc::ariadne::kinematic_vehicle_step(m, k, t, dt);
      command_after(m, k, in, dt);
    }
    const cvc::physics::body_state relay = h.state("ann");
    worst = std::max(worst, std::hypot(m.box[0] - relay.position.x, m.box[1] - relay.position.y));
    top = std::max(top, std::hypot(relay.linear_velocity.x, relay.linear_velocity.y));
  }
  const cvc::physics::body_state end = h.state("ann");
  std::printf("  the model on the relay's vehicle: %.6f m at worst over 9 s (top %.2f m/s, "
              "%.1f m from the start, speed at the end %.2f m/s)\n",
              worst, top, std::hypot(end.position.x - start.x, end.position.y - start.y), m.speed);
  EXPECT_GT(top, 5.0) << "it drove";
  EXPECT_LT(m.speed, -0.5) << "and backed up at the end";
  EXPECT_LT(worst, 0.01) << "step for step";
}
