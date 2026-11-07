// A document's scene world with no renderer (scene_world.h; ARIADNE_WORLDS.md section 14,
// "Headless first"): the physics world and its bodies built from the scene alone, stepped on the
// world clock, publishing physics.stats.*, and run by an AppRuntime with no SceneAdapter. The same
// world drawn by cvcGL is cvcgl_ariadne_physics's test_same_world_headless_and_drawn.

#include <chrono>
#include <cmath>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/requires.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_geometry.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/core/app.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
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
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "physics", &warnings);
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
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "physics", &warnings);
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
  std::unique_ptr<SceneWorld> w = SceneWorld::from_scene(app, r.scene, "", &warnings);
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

TEST(AriadneSceneWorld, ScenePhysicsTokenIsTheEngine) {
  bool found = false;
  for (const CapabilityToken &t : capability_tokens())
    if (std::string(t.name) == "scene.physics")
      found = true;
  EXPECT_TRUE(found);
}
