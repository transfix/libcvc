// A document's scene world with no renderer (scene_world.h; ARIADNE_WORLDS.md section 14,
// "Headless first"): the physics world and its bodies built from the scene alone, stepped on the
// world clock, publishing physics.stats.*, and run by an AppRuntime with no SceneAdapter; a
// generated_world node run here (its world, keys, Regenerate and colliders); a data node's bodies
// taken from its dataset when it is ready, and swapped when it changes. The same worlds drawn by
// cvcGL are cvcgl_ariadne_physics's test_same_world_headless_and_drawn.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/requires.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_generated_world.h>
#include <cvc/ariadne/scene_geometry.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_rules.h>
#include <cvc/ariadne/world_spawns.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/world_grid.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
#include <cvc/world/world_geometry.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::ariadne;
namespace fs = std::filesystem;

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
  LoadResult r = load_string(kScene);
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
  LoadResult r = load_string(kScene);
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
  LoadResult r = load_string(R"(
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
  LoadResult r = load_string(kGenerated);
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
  EXPECT_EQ(w->bodies_of("world").size(), items);
  EXPECT_EQ(w->world().body_count(), items + 1);
  for (const SceneWorldBody &b : w->bodies())
    EXPECT_EQ(b.moves, b.node == "ball");
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

TEST(AriadneSceneWorld, RegenerateMakesANewWorldAndSwapsItsBodies) {
  REQUIRE_YAML();
  LoadResult r = load_string(kGenerated);
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
  EXPECT_EQ(w->bodies_of("world").size(), collide_items(*second));
  EXPECT_EQ(w->world().body_count(), collide_items(*second) + 1);
  EXPECT_TRUE(w->body("ball").valid());
}

TEST(AriadneSceneWorld, AWorldNodesGridFollowsItsWorld) {
  REQUIRE_YAML();
  LoadResult r = load_string(kGenerated);
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
  LoadResult r = load_string(R"(
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
  ASSERT_EQ(w->bodies_of("city").size(), 1u);
  const auto first = w->body("city");
  w->sync(&warnings); // the same rev: nothing changes
  EXPECT_EQ(w->body("city"), first);

  // A reload (a new rev) swaps it; the old body goes.
  root("data.city.terrain")
      .data(std::shared_ptr<const cvc::height_field>(
          std::make_shared<cvc::height_field>(5, 5, -10.0, -10.0, 5.0, 5.0)));
  root("data.city.rev").value("2");
  w->sync(&warnings);
  ASSERT_EQ(w->bodies_of("city").size(), 1u);
  EXPECT_EQ(w->world().body_count(), 1u);
  EXPECT_FALSE(w->world().contains(first));

  // Without the physics: block a data node's body warns, as cvcGL does.
  LoadResult bare = load_string(R"(
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
  LoadResult r = load_string(kWorlds);
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
  LoadResult r = load_string(kWorlds);
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
  EXPECT_EQ(w->bodies_of("land").size(), collide_items(*gw));
  EXPECT_TRUE(w->bodies_of("square").empty());
  EXPECT_EQ(val(app, "scene.world.kind"), "generated");
  EXPECT_EQ(val(app, "scene.world.rev"), "2");
  EXPECT_EQ(val(app, "scene.world.on.land"), "1");
  EXPECT_EQ(val(app, "scene.world.on.square"), "0");
  EXPECT_EQ(w->world().body_count(), collide_items(*gw) + 1);
  // A new world from the one on is a new revision.
  cvc::state::instance(app)("world.request").value("randomize");
  w->sync(&warnings);
  EXPECT_EQ(val(app, "scene.world.rev"), "3");
  EXPECT_EQ(w->bodies_of("land").size(), collide_items(*w->generated_world("land")));

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
  REQUIRE_YAML();
  if (!jolt())
    GTEST_SKIP() << "no Jolt in this build";
  LoadResult r = load_string(kScene);
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
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
  register_world_block();
  LoadResult r = load_string(text);
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
    LoadResult bad = load_string("world:\n  vehicles:\n    archetypes:\n      v: " + archetype +
                                 "\nscene:\n  nodes: []\n");
    world_rules x = default_world_rules();
    const Value *block = find_world_block(bad);
    return block ? parse_world_block(*block, x) : std::string("no world: block");
  };
  EXPECT_NE(error("{ vehicle: tank }").find("not a preset"), std::string::npos);
  EXPECT_NE(error("{ plant: hover }").find("plant 'hover'"), std::string::npos);
  EXPECT_NE(error("{ controllers: [walker@1] }").find("nav@1 or none"), std::string::npos);
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
  // A prefix with the vehicle-humvee package's model where cvcpkg puts it.
  const fs::path prefix = fs::temp_directory_path() / "ariadne_world_vehicles_prefix";
  fs::create_directories(prefix / "share" / "cvc-scenes" / "shared");
  std::ofstream(prefix / "share" / "cvc-scenes" / "shared" / "Humvee.glb") << "glb";
  EnvFor bundle("CVC_NAV_BUNDLE", "");
  EnvFor env("CVC_PREFIX", prefix.string());
  const std::string humvee = find_vehicle_model("humvee");
  EXPECT_NE(humvee.find("Humvee.glb"), std::string::npos) << humvee;
  EXPECT_EQ(find_vehicle_model("no_such_vehicle.glb"), "");

  std::vector<vehicle_archetype> v(5);
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
  EXPECT_EQ(v[2].look.kind, "box");
  EXPECT_TRUE(any_contains(warnings, "model 'tank' is not installed"));
  EXPECT_EQ(fs::path(v[3].look.path), prefix / "doc" / "car.glb");
  EXPECT_FALSE(v[3].look.fit);
  EXPECT_EQ(v[4].look.kind, "file");
  EXPECT_EQ(fs::path(v[4].look.path), prefix / "doc" / "models" / "car.glb");

  // `model:` in the world: block.
  REQUIRE_YAML();
  register_world_block();
  LoadResult r = load_string("world:\n  vehicles:\n    archetypes:\n      h: { look: { model: "
                             "humvee, yaw: 180 } }\nscene:\n  nodes: []\n");
  world_rules rules = default_world_rules();
  ASSERT_NE(find_world_block(r), nullptr);
  EXPECT_EQ(parse_world_block(*find_world_block(r), rules), "");
  EXPECT_EQ(rules.vehicles[0].look.kind, "model");
  EXPECT_EQ(rules.vehicles[0].look.path, "humvee");
  EXPECT_TRUE(rules.vehicles[0].look.fit);
  EXPECT_EQ(rules.vehicles[0].look.yaw, 180.0);
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
  st("vehicles.selected").value("walker");
  st("vehicles.spawn.controller").value("");
  st("vehicles.spawn.request").value("3");
  w->sync(&warnings);
  EXPECT_NE(val(app, "vehicles.spawn.status").find("plant kinematic does not run"),
            std::string::npos);
  EXPECT_EQ(w->vehicles()->size(), 1u);
  // Removed by id, then a clear.
  st("vehicles.selected").value("humvee");
  st("vehicles.spawn.at").value("");
  st("vehicles.spawn.request").value("4");
  w->sync(&warnings);
  EXPECT_EQ(w->vehicles()->size(), 2u);
  st("vehicles.remove.id").value("1");
  st("vehicles.remove.request").value("1");
  w->sync(&warnings);
  EXPECT_EQ(w->vehicles()->size(), 1u);
  EXPECT_EQ(w->world().vehicle_count(), 1u);
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
  register_world_block();
  LoadResult r = load_string(text);
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
    LoadResult bad = load_string("world:\n  vehicles:\n    archetypes:\n      v: { vehicle: humvee "
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
