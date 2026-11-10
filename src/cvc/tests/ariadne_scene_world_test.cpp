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
