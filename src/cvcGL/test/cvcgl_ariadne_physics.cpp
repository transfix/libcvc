// Ariadne physics: and body: (WORLD-GEOMETRY-ROADMAP.md, G5). A scene spec puts a crate on a
// heightfield and it lands where the renderer draws the ground. Headless. These checks pin that:
//   1. a heightfield node's cvc::height_field is kept (RealizedScene::height_fields) and is the
//      mesh the node draws;
//   2. each body starts at its node's realized world pose, a nested node's included;
//   3. ticking the scene steps the world on app.world_clock() (driven as by `clock: {}`), and a
//      moving body's node is drawn at the body's pose;
//   4. at rest, the crate sits half its height above the DRAWN mesh (measured on the node's own
//      triangles), and the ball on the dome's peak;
//   5. the null backend places bodies and never moves them; problems warn, never throw;
//   6. the same document's world built headless (core libcvc's SceneWorld, no SceneGraph) and drawn
//      here reaches the same state, bit for bit (physics::world::state_hash), and both publish
//      physics.stats.*;
//   7. so does a generated world's: a generated_world node's body: (its terrain and buildings, made
//      in core) with a ball falling onto it, run headless and drawn;
//   8. sim.restart rebuilds a drawn world; and of a scene's worlds (`world:` on its nodes) only the
//      world on is drawn and has bodies, and writing scene.world.source switches both.
// The dynamics need Jolt and YAML; without either, those checks are skipped.
//
// NOT assert(): built Release, where NDEBUG makes assert() a no-op.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene_boat.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_rules.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/LabelLayer.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/vehicle_view.h>
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <system_error>
#include <vector>
#include <vtkMatrix4x4.h>

using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
namespace ari = cvc::gl::ariadne;

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

static int g_failures = 0;
static void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

namespace {

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

const char *kScene = R"(
meta: { min_libcvc: "0.0.0" }
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

// The height of the DRAWN mesh at (x, y): the triangle of `g` under the point, interpolated, and
// that triangle's upward normal's z. What the renderer shows, computed without cvc::height_field.
bool drawn_height(const cvc::geometry &g, double x, double y, double &z, double &nz) {
  const auto &p = g.const_points();
  for (const auto &t : g.const_tris()) {
    const auto &a = p[t[0]], &b = p[t[1]], &c = p[t[2]];
    const double d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1]);
    if (std::fabs(d) < 1e-12)
      continue;
    const double l0 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / d;
    const double l1 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / d;
    const double l2 = 1.0 - l0 - l1;
    if (l0 < -1e-9 || l1 < -1e-9 || l2 < -1e-9)
      continue;
    z = l0 * a[2] + l1 * b[2] + l2 * c[2];
    const double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
    const double vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
    const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, n3 = ux * vy - uy * vx;
    nz = std::fabs(n3) / std::sqrt(nx * nx + ny * ny + n3 * n3);
    return true;
  }
  return false;
}

double translation(const cvc::gl::GraphicsNode &node, int k) {
  return node.getWorldTransform()->GetElement(k, 3);
}

void test_crate_lands_on_the_drawn_ground() {
  std::printf("test_crate_lands_on_the_drawn_ground\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  cvc::ariadne::LoadResult r = fresh_load(kScene);
  CHECK(r.ok);
  if (!r.ok) {
    std::printf("  %s\n", r.error.c_str());
    return;
  }
  cvc::app app;
  SceneGraph sg(app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, sg.getStatePrefix(), &warnings);
  for (const auto &w : warnings)
    std::printf("  warning: %s\n", w.c_str());
  const bool jolt = cvc::physics::backend_available(cvc::physics::backend::jolt);
  CHECK(warnings.empty() || !jolt);

  // 1. The terrain's field is the mesh its node draws.
  auto terrain = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("terrain"));
  CHECK(terrain && terrain->getGeometry());
  CHECK(out.height_fields.count("terrain") == 1);
  if (!terrain || !terrain->getGeometry() || !out.height_fields.count("terrain"))
    return;
  const cvc::height_field &field = *out.height_fields.at("terrain");
  const cvc::geometry &drawn = *terrain->getGeometry();
  CHECK(field.rows() == 41 && field.cols() == 41);
  CHECK(drawn.num_points() == field.num_points() && drawn.num_tris() == field.num_tris());
  CHECK(near(drawn.const_points()[field.vertex_of(20, 20)][2], 1.0, 1e-6)); // the dome's peak

  // 2. Bodies at their nodes' world poses; the ball's through its group.
  CHECK(out.physics != nullptr);
  CHECK(out.bodies.size() == 3);
  if (!out.physics || out.bodies.size() != 3)
    return;
  cvc::physics::world &w = *out.physics;
  const auto crate_id = out.body("crate"), ball_id = out.body("ball");
  CHECK(crate_id.valid() && ball_id.valid() && out.body("terrain").valid());
  const auto c0 = w.state(crate_id), b0 = w.state(ball_id);
  CHECK(near(c0.position.x, 2.0, 1e-6) && near(c0.position.y, -1.5, 1e-6) &&
        near(c0.position.z, 4.0, 1e-6));
  CHECK(near(b0.position.x, 0.0, 1e-6) && near(b0.position.z, 3.0, 1e-6));
  CHECK(near(w.config().fixed_dt, app.world_clock().fixed_dt(), 1e-12)); // one quantum per step
  if (!jolt) {
    std::printf("  skipped the dynamics: no Jolt in this build\n");
    return;
  }

  // 3. Six seconds at 60 frames per second.
  for (int i = 0; i < 360; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  CHECK(near(app.world_clock().t(), 6.0, 0.02));
  auto crate = sg.getGraphics("crate");
  CHECK(crate != nullptr);
  if (!crate)
    return;
  const auto c = w.state(crate_id);
  CHECK(near(translation(*crate, 0), c.position.x, 1e-6));
  CHECK(near(translation(*crate, 1), c.position.y, 1e-6));
  CHECK(near(translation(*crate, 2), c.position.z, 1e-6));

  // 4. At rest, half its height above the drawn ground under its centre (a facet tilted by
  // acos(nz) puts the centre 0.5 / nz above it).
  double ground = 0.0, nz = 1.0;
  CHECK(drawn_height(drawn, c.position.x, c.position.y, ground, nz));
  std::printf("  crate at (%.3f, %.3f, %.4f); drawn ground %.4f, facet tilt %.1f deg\n",
              c.position.x, c.position.y, c.position.z, ground, std::acos(nz) * 57.29578);
  CHECK(near(c.position.z - ground, 0.5 / nz, 0.015));
  const double v = std::sqrt(c.linear_velocity.x * c.linear_velocity.x +
                             c.linear_velocity.y * c.linear_velocity.y +
                             c.linear_velocity.z * c.linear_velocity.z);
  CHECK(v < 0.02);
  // The ball rests on the peak, drawn where its body is, through its group.
  const auto b = w.state(ball_id);
  CHECK(near(b.position.z, 1.0 + 0.4, 0.01) && near(b.position.x, 0.0, 0.01));
  auto ball = sg.getGraphics("rig")->findChildByName("ball");
  CHECK(ball != nullptr);
  if (ball)
    CHECK(near(translation(*ball, 2), b.position.z, 1e-6));
}

void test_null_backend_places_and_holds() {
  std::printf("test_null_backend_places_and_holds\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  std::string spec = kScene;
  spec.replace(spec.find("physics: {}"), 11, "physics: { backend: null }");
  cvc::ariadne::LoadResult r = fresh_load(spec);
  CHECK(r.ok);
  cvc::app app;
  SceneGraph sg(app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, sg.getStatePrefix(), &warnings);
  CHECK(warnings.empty());
  CHECK(out.physics && out.physics->backend() == cvc::physics::backend::null);
  for (int i = 0; i < 60; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  auto crate = sg.getGraphics("crate");
  CHECK(crate && near(translation(*crate, 2), 4.0, 1e-9));
}

void test_problems_warn() {
  std::printf("test_problems_warn\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  cvc::ariadne::LoadResult r = fresh_load(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  physics: { fixed_dt: 0.01, materials: { ice: { friction: 0.1 } } }
  nodes:
    - node: hill
      source: { heightfield: { size: 4, resolution: 5 } }
      body: dynamic
    - node: folder
      type: group
      body: dynamic
    - node: crate
      source: { box: 1 }
      transform: { position: [0, 0, 3] }
      body: { material: lava, motion: floating }
)");
  CHECK(r.ok);
  cvc::app app;
  SceneGraph sg(app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, sg.getStatePrefix(), &warnings);
  const auto said = [&](const char *what) {
    for (const auto &w : warnings)
      if (w.find(what) != std::string::npos)
        return true;
    return false;
  };
  for (const auto &w : warnings)
    std::printf("  warning: %s\n", w.c_str());
  CHECK(said("fixed_dt"));              // 0.01 s is not a whole number of 1/120 s quanta
  CHECK(said("a terrain is static"));   // body: dynamic on a heightfield
  CHECK(said("needs a geometry node")); // body: on a group
  CHECK(said("no physics material"));   // material: lava
  CHECK(said("motion 'floating'"));     // not dynamic, kinematic or static
  CHECK(out.physics != nullptr);
  CHECK(out.body("hill").valid() && out.body("crate").valid() && !out.body("folder").valid());
  if (out.physics)
    CHECK(near(out.physics->config().fixed_dt, 1.0 / 120.0, 1e-12)); // rounded to one quantum
}

// A scene with every body shape a document builds from data: a terrain, a box and a sphere (one
// nested under a rotated, scaled group), a box primitive's hull, and a mesh.
const char *kSameWorld = R"(
meta: { min_libcvc: "0.0.0" }
scene:
  physics: { materials: { rubber: { friction: 0.9, restitution: 0.6 } } }
  nodes:
    - node: terrain
      source:
        heightfield:
          size: 24
          resolution: 49
          layers:
            - { kind: dome, amplitude: 1.5, radius: 5 }
            - { kind: wave, amplitude: 0.2, wavelength: 6, direction: [1, 0.5] }
      body: static
    - node: crate
      source: { box: { size: [1, 0.6, 0.8] } }
      transform: { position: [2.0, -1.5, 4.0], rotation: [10, 20, 30] }
      body: { mass: 10, material: rubber }
    - node: rig
      type: group
      transform: { position: [-1, 1, 1], rotation: [0, 0, 45], scale: [1.5, 1.5, 1.5] }
      children:
        - node: ball
          source: { sphere: 0.4 }
          transform: { position: [0.5, 0, 2] }
          body: { density: 500, velocity: [1, 0, 0] }
        - node: block
          source: { box: { size: [0.5, 0.5, 0.5] } }
          transform: { position: [-0.5, 0.5, 3] }
          body: { shape: hull }
    - node: ramp
      source: { box: { size: [3, 1, 0.2] } }
      transform: { position: [0, -4, 1.5], rotation: [15, 0, 0] }
      body: { motion: static, shape: mesh }
)";

void test_same_world_headless_and_drawn() {
  std::printf("test_same_world_headless_and_drawn\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  cvc::ariadne::LoadResult r = fresh_load(kSameWorld);
  CHECK(r.ok);
  if (!r.ok)
    return;
  constexpr int kFrames = 240; // four seconds at 60 frames per second

  // Drawn: realized over a SceneGraph, stepped by tick_scene.
  cvc::app drawn_app;
  ari::register_world_node_types(drawn_app);
  SceneGraph sg(drawn_app);
  std::vector<std::string> drawn_warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &drawn_warnings);
  for (const auto &w : drawn_warnings)
    std::printf("  drawn warning: %s\n", w.c_str());
  CHECK(out.world != nullptr && out.physics == &out.world->world());
  if (!out.physics)
    return;
  for (int i = 0; i < kFrames; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);

  // Headless: the scene alone, stepped on its clock.
  cvc::app headless_app;
  std::vector<std::string> headless_warnings;
  std::unique_ptr<cvc::ariadne::SceneWorld> world = cvc::ariadne::SceneWorld::from_scene(
      headless_app, r.scene, "", "physics", &headless_warnings);
  for (const auto &w : headless_warnings)
    std::printf("  headless warning: %s\n", w.c_str());
  CHECK(world != nullptr);
  if (!world)
    return;
  CHECK(headless_warnings == drawn_warnings);
  cvc::ariadne::SceneClockDriver clock(headless_app, r.scene.clock);
  for (int i = 0; i < kFrames; ++i) {
    clock.drive(1.0 / 60.0);
    world->follow(headless_app.world_clock());
  }

  CHECK(world->bodies().size() == out.bodies.size());
  CHECK(world->bodies().size() == 5);
  CHECK(world->world().tick() == out.physics->tick());
  CHECK(world->world().tick() > 0);
  const std::uint64_t drawn_hash = out.physics->state_hash();
  const std::uint64_t headless_hash = world->world().state_hash();
  if (drawn_hash != headless_hash)
    std::printf("  drawn %016llx, headless %016llx\n", static_cast<unsigned long long>(drawn_hash),
                static_cast<unsigned long long>(headless_hash));
  CHECK(drawn_hash == headless_hash);
  // The bodies moved (it is not two worlds frozen alike).
  const bool jolt = cvc::physics::backend_available(cvc::physics::backend::jolt);
  if (jolt) {
    const auto b = world->world().state(world->body("ball"));
    CHECK(b.position.z < 2.5 * 1.5 + 1.0);
  }
  // Both publish the world's telemetry.
  CHECK(cvc::state::instance(drawn_app)("physics.stats.tick").value() ==
        std::to_string(out.physics->tick()));
  CHECK(cvc::state::instance(headless_app)("physics.stats.bodies").value() == "5");
}

const char *kGeneratedWorld = R"(
meta: { min_libcvc: "0.0.0" }
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
      texture: 256
      body: {}
    - node: ball
      source: { sphere: 0.5 }
      transform: { position: [3, -4, 80] }
      body: { density: 400 }
)";

void test_same_generated_world_headless_and_drawn() {
  std::printf("test_same_generated_world_headless_and_drawn\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  cvc::ariadne::LoadResult r = fresh_load(kGeneratedWorld);
  CHECK(r.ok);
  if (!r.ok)
    return;
  constexpr int kFrames = 300; // five seconds: the ball falls and lands

  cvc::app drawn_app;
  ari::register_world_node_types(drawn_app);
  SceneGraph sg(drawn_app);
  std::vector<std::string> drawn_warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &drawn_warnings);
  for (const auto &w : drawn_warnings)
    std::printf("  drawn warning: %s\n", w.c_str());
  CHECK(out.world != nullptr);
  if (!out.world)
    return;
  for (int i = 0; i < kFrames; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);

  cvc::app headless_app;
  std::vector<std::string> headless_warnings;
  std::unique_ptr<cvc::ariadne::SceneWorld> world = cvc::ariadne::SceneWorld::from_scene(
      headless_app, r.scene, "", "physics", &headless_warnings);
  for (const auto &w : headless_warnings)
    std::printf("  headless warning: %s\n", w.c_str());
  CHECK(world != nullptr);
  if (!world)
    return;
  CHECK(headless_warnings == drawn_warnings);
  CHECK(world->generated_world("world") != nullptr);
  CHECK(out.world->generated_world("world") == nullptr); // the drawn node runs it there
  cvc::ariadne::SceneClockDriver clock(headless_app, r.scene.clock);
  for (int i = 0; i < kFrames; ++i) {
    clock.drive(1.0 / 60.0);
    world->follow(headless_app.world_clock());
  }

  // The same colliders (the terrain, and the buildings when the city has any) and the ball.
  CHECK(world->bodies_of("world").size() >= 1);
  CHECK(world->bodies_of("world").size() == out.world->bodies_of("world").size());
  CHECK(world->world().body_count() == out.physics->body_count());
  CHECK(world->world().tick() == out.physics->tick());
  const std::uint64_t drawn_hash = out.physics->state_hash();
  const std::uint64_t headless_hash = world->world().state_hash();
  if (drawn_hash != headless_hash)
    std::printf("  drawn %016llx, headless %016llx\n", static_cast<unsigned long long>(drawn_hash),
                static_cast<unsigned long long>(headless_hash));
  CHECK(drawn_hash == headless_hash);
  // Both made the same world and say so (the drawn node's keys are under the SceneGraph's prefix).
  for (const char *k : {"world.stats.buildings", "world.stats.props", "world.status"}) {
    const std::string drawn =
        cvc::state::instance(drawn_app)(cvc::ariadne::resolve_bind(sg.getStatePrefix(), k)).value();
    const std::string headless = cvc::state::instance(headless_app)(k).value();
    if (drawn != headless)
      std::printf("  %s: drawn '%s', headless '%s'\n", k, drawn.c_str(), headless.c_str());
    CHECK(!drawn.empty() && drawn == headless);
  }
  if (cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    const auto b = world->world().state(world->body("ball"));
    CHECK(b.position.z < 80.0); // it fell
  }
}

} // namespace

// sim.restart in a drawn scene: restart_scene_world rebuilds its world (the same physics::world)
// and binds its moving nodes again; with the clock back at tick 0 it steps exactly as before.
void test_restart_drawn_world() {
  std::printf("test_restart_drawn_world\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  cvc::ariadne::LoadResult r = fresh_load(kSameWorld);
  CHECK(r.ok);
  if (!r.ok)
    return;
  cvc::app app;
  SceneGraph sg(app);
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", nullptr);
  CHECK(out.world != nullptr);
  if (!out.world)
    return;
  const std::uint64_t h0 = out.physics->state_hash();
  for (int i = 0; i < 120; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  const std::uint64_t ticks = out.physics->tick();
  const std::uint64_t h1 = out.physics->state_hash();
  CHECK(ticks > 0);
  cvc::physics::world *const before = out.physics;
  app.world_clock().reset();
  std::vector<std::string> warnings;
  CHECK(ari::restart_scene_world(out, &warnings));
  CHECK(warnings.empty());
  CHECK(out.physics == before); // the same world object, rebuilt
  CHECK(out.physics->tick() == 0);
  CHECK(out.physics->state_hash() == h0);
  CHECK(out.physics_binding != nullptr);
  CHECK(out.bodies.size() == out.world->bodies().size());
  for (int i = 0; i < 120; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  CHECK(out.physics->tick() == ticks);
  CHECK(out.physics->state_hash() == h1);
  // A scene with no world has nothing to restart.
  ari::RealizedScene empty;
  CHECK(!ari::restart_scene_world(empty));
}

// A drawn scene's worlds (`world:` on its nodes): only the world on is drawn and has bodies, and
// writing scene.world.source switches both at the next tick.
void test_worlds_drawn() {
  std::printf("test_worlds_drawn\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  cvc::ariadne::LoadResult r = fresh_load(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  physics: {}
  world: { source: terrain, cell: 2 }
  nodes:
    - node: hill
      world: terrain
      source:
        heightfield:
          size: 20
          resolution: 21
          layers:
            - { kind: dome, amplitude: 1.0, radius: 6 }
      body: static
    - node: square
      world: flat
      source: { box: { size: [30, 30, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - node: ball
      source: { sphere: 0.4 }
      transform: { position: [0, 0, 4] }
      body: { density: 500 }
)");
  CHECK(r.ok);
  if (!r.ok)
    return;
  cvc::app app;
  SceneGraph sg(app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &warnings);
  CHECK(out.world != nullptr);
  if (!out.world)
    return;
  const auto shown = [&](const char *id) {
    auto it = out.body_nodes.find(id);
    if (it == out.body_nodes.end() || !it->second)
      return std::string("?");
    return cvc::state::instance(app)(it->second->stateName("visible")).value();
  };
  ari::tick_scene(out, nullptr, 1.0 / 60.0);
  CHECK(out.world->world_on() == "terrain");
  CHECK(shown("hill") == "1");
  CHECK(shown("square") == "0");
  CHECK(out.world->bodies_of("hill").size() == 1);
  CHECK(out.world->bodies_of("square").empty());
  CHECK(cvc::state::instance(app)("scene.world.grid.rows").value() == "11");
  // Switch: the square is drawn and collides; the hill is neither.
  cvc::state::instance(app)("scene.world.source").value("flat");
  ari::tick_scene(out, nullptr, 1.0 / 60.0);
  cvc::ariadne::sync_scene_visibility(app,
                                      out.visibility); // as a host does each frame (AriRuntime)
  CHECK(out.world->world_on() == "flat");
  CHECK(shown("hill") == "0");
  CHECK(shown("square") == "1");
  CHECK(out.world->bodies_of("square").size() == 1);
  CHECK(out.world->bodies_of("hill").empty());
  CHECK(cvc::state::instance(app)("scene.world.rev").value() == "2");
}

// The world's vehicles drawn (L2): each spawned entity a node bound to its chassis, gone when it
// is removed; and a vehicle_preview node draws the selected archetype on its own scene.
void test_vehicles_drawn() {
  cvc::app app;
  std::printf("test_vehicles_drawn\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no Jolt in this build\n");
    return;
  }
  cvc::ariadne::register_world_block(app);
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, R"(
meta: { min_libcvc: "0.0.0" }
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee }
      racer: { vehicle: roboracer, controllers: [none] }
  spawns:
    points:
      - { at: [-20, 10], heading: 90, team: 1 }
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [80, 80, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)");
  CHECK(r.ok);
  if (!r.ok)
    return;
  cvc::ariadne::world_rules rules = cvc::ariadne::default_world_rules();
  CHECK(cvc::ariadne::parse_world_block(*cvc::ariadne::find_world_block(r), rules).empty());
  r.scene.vehicles = rules.vehicles;
  r.scene.spawns = rules.spawns;
  SceneGraph sg(app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &warnings);
  CHECK(out.world != nullptr && out.world->vehicles() != nullptr);
  if (!out.world || !out.world->vehicles())
    return;
  cvc::state &st = cvc::state::instance(app);
  ari::tick_scene(out, nullptr, 1.0 / 60.0);
  st("vehicles.spawn.at").value("5,-3,0");
  st("vehicles.spawn.color").value("1,0,0");
  st("vehicles.spawn.request").value("1");
  for (int i = 0; i < 30; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  CHECK(out.world->vehicles()->size() == 1);
  std::shared_ptr<cvc::gl::GraphicsNode> node = sg.getGraphics("vehicle_1");
  CHECK(node != nullptr);
  CHECK(out.vehicle_markers.size() == 1);
  if (node && !out.vehicle_markers.empty()) {
    CHECK(out.physics_binding->bound(out.vehicle_markers[0].second));
    // Drawn where its chassis is.
    const cvc::ariadne::entity_info e = out.world->vehicles()->entities()[0];
    const vtkMatrix4x4 *m = node->getTransform();
    CHECK(near(m->GetElement(0, 3), e.x, 0.5));
    CHECK(near(m->GetElement(1, 3), e.y, 0.5));
  }
  // Removed: its node goes.
  st("vehicles.remove.id").value("1");
  st("vehicles.remove.request").value("1");
  for (int i = 0; i < 3; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  CHECK(sg.getGraphics("vehicle_1") == nullptr);
  CHECK(out.vehicle_markers.empty());

  // One on nav's bicycle: its chassis a kinematic box in the physics world (not a vehicle there),
  // drawn on it.
  st("vehicles.spawn.plant").value("kinematic");
  st("vehicles.spawn.controller").value("none");
  st("vehicles.spawn.at").value("-10,15,0");
  st("vehicles.spawn.request").value("2");
  for (int i = 0; i < 30; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  CHECK(out.world->vehicles()->size() == 1);
  CHECK(out.world->world().vehicle_count() == 0);
  CHECK(out.vehicle_markers.size() == 1);
  std::shared_ptr<cvc::gl::GraphicsNode> bike = sg.getGraphics("vehicle_2");
  CHECK(bike != nullptr);
  if (bike && !out.vehicle_markers.empty()) {
    CHECK(out.vehicle_markers[0].second.valid());
    CHECK(out.physics_binding->bound(out.vehicle_markers[0].second));
    const cvc::ariadne::entity_info e = out.world->vehicles()->entities()[0];
    CHECK(e.spec.plant == "kinematic");
    const vtkMatrix4x4 *m = bike->getTransform();
    CHECK(near(m->GetElement(0, 3), e.x, 0.5));
    CHECK(near(m->GetElement(1, 3), e.y, 0.5));
    CHECK(near(e.x, -10.0, 3.0) && near(e.y, 15.0, 3.0));
  }
  st("vehicles.clear.request").value("1");
  for (int i = 0; i < 3; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  CHECK(out.vehicle_markers.empty());
  st("vehicles.spawn.plant").value("");
  st("vehicles.spawn.controller").value("nav@1");

  // The spawn points: a marker each, tagged spawns, and one for a point the minimap adds.
  CHECK(sg.getGraphics("spawn_point_1") != nullptr);
  CHECK(out.spawn_markers.size() == 1);
  st("spawns.pick.x").value("20");
  st("spawns.pick.y").value("20");
  st("spawns.pick.hit").value("1");
  st("spawns.pick.phase").value("down");
  st("spawns.pick.seq").value("1");
  for (int i = 0; i < 3; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  CHECK(sg.getGraphics("spawn_point_2") != nullptr);
  CHECK(out.spawn_markers.size() == 2);

  // The preview: the selected archetype on a scene of its own, rebuilt when the pick changes.
  ari::register_vehicle_preview_node(app);
  cvc::ariadne::LoadResult studio = cvc::ariadne::load_string(app, R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: preview
      type: vehicle_preview
      spin: 0
)");
  CHECK(studio.ok);
  if (!studio.ok)
    return;
  SceneGraph sg2(app);
  ari::RealizedScene shown = ari::realize_scene(sg2, studio.scene, "", &warnings);
  ari::tick_scene(shown, nullptr, 1.0 / 60.0);
  std::shared_ptr<cvc::gl::GraphicsNode> first = sg2.getGraphics("preview__vehicle");
  CHECK(first != nullptr);
  st("vehicles.selected").value("racer");
  ari::tick_scene(shown, nullptr, 1.0 / 60.0);
  std::shared_ptr<cvc::gl::GraphicsNode> second = sg2.getGraphics("preview__vehicle");
  CHECK(second != nullptr && second != first);
  if (second) // the racer stands on the set: its centre half its height and a wheel up
    CHECK(second->getTransform()->GetElement(2, 3) < 0.3);
}

// Entity intents (S1), scripted on the keys at fixed frames: an external entity driven by its
// intent, a nav@1 one with a force on its drive and then a blend, run headless (SceneWorld on its
// clock) and drawn (realize_scene + tick_scene, as arigl --offscreen steps it), reach the same
// state, bit for bit.
const char *kIntentWorld = R"(
meta: { min_libcvc: "0.0.0" }
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee, controllers: [nav@1, external] }
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

// The script both runs follow: what is written at which frame.
void intent_script(cvc::state &st, int frame) {
  switch (frame) {
  case 1:
    st("vehicles.spawn.controller").value("external");
    st("vehicles.spawn.at").value("-30,0,0");
    st("vehicles.spawn.request").value("1");
    break;
  case 2:
    st("vehicles.spawn.controller").value("nav@1");
    st("vehicles.spawn.at").value("20,-20,90");
    st("vehicles.spawn.request").value("2");
    break;
  case 30:
    st("entities.1.intent.accel").value("2");
    st("entities.2.force.x").value("-2");
    st("entities.2.force.y").value("1.5");
    break;
  case 120:
    st("entities.1.intent.steer").value("0.25");
    st("entities.2.intent.mode").value("blend");
    st("entities.2.intent.steer").value("-0.2");
    st("entities.2.intent.accel").value("0.5");
    st("entities.2.intent.weight").value("0.4");
    break;
  case 200:
    st("entities.1.intent.mode").value("off");
    break;
  default:
    break;
  }
}

void test_vehicle_intents_headless_and_drawn() {
  std::printf("test_vehicle_intents_headless_and_drawn\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no Jolt in this build\n");
    return;
  }
  const auto load = [](cvc::app &app) {
    cvc::ariadne::register_world_block(app);
    cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, kIntentWorld);
    if (r.ok) {
      cvc::ariadne::world_rules rules = cvc::ariadne::default_world_rules();
      CHECK(cvc::ariadne::parse_world_block(*cvc::ariadne::find_world_block(r), rules).empty());
      r.scene.vehicles = rules.vehicles;
    }
    return r;
  };
  constexpr int kFrames = 300; // five seconds at 60 frames per second

  cvc::app drawn_app;
  cvc::ariadne::LoadResult r = load(drawn_app);
  CHECK(r.ok);
  if (!r.ok)
    return;
  SceneGraph sg(drawn_app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &warnings);
  CHECK(out.world != nullptr && out.world->vehicles() != nullptr);
  if (!out.world || !out.world->vehicles())
    return;
  cvc::state &dst = cvc::state::instance(drawn_app);
  for (int i = 0; i < kFrames; ++i) {
    intent_script(dst, i);
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  }

  cvc::app headless_app;
  std::unique_ptr<cvc::ariadne::SceneWorld> world =
      cvc::ariadne::SceneWorld::from_scene(headless_app, r.scene, "", "physics", &warnings);
  CHECK(world != nullptr && world->vehicles() != nullptr);
  if (!world || !world->vehicles())
    return;
  cvc::state &hst = cvc::state::instance(headless_app);
  cvc::ariadne::SceneClockDriver clock(headless_app, r.scene.clock);
  for (int i = 0; i < kFrames; ++i) {
    intent_script(hst, i);
    clock.drive(1.0 / 60.0);
    world->follow(headless_app.world_clock());
  }

  CHECK(world->vehicles()->size() == 2 && out.world->vehicles()->size() == 2);
  CHECK(world->world().tick() == out.physics->tick());
  const std::uint64_t drawn_hash = out.physics->state_hash();
  const std::uint64_t headless_hash = world->world().state_hash();
  if (drawn_hash != headless_hash)
    std::printf("  drawn %016llx, headless %016llx\n", static_cast<unsigned long long>(drawn_hash),
                static_cast<unsigned long long>(headless_hash));
  CHECK(drawn_hash == headless_hash);
  // The intents did their work: the external entity drove off and stopped, the nav@1 one blends.
  if (world->vehicles()->size() == 2) {
    const cvc::ariadne::entity_info e = world->vehicles()->entities()[0];
    CHECK(e.x > -25.0);
    CHECK(std::fabs(e.speed) < 2.0);
    CHECK(hst("entities.2.command.by").value() == "blend");
    CHECK(dst("entities.2.command.by").value() == "blend");
  }
}

// Entity poses (the `pose` controller), scripted on the keys at fixed frames: an entity an outside
// simulation places, with no plant integrating it, first untimed (where its keys say at the next
// step) then timed (drawn between the last two), run headless (SceneWorld on its clock) and drawn
// (realize_scene + tick_scene, as arigl --offscreen steps it): the same state, bit for bit, and
// the same pinned hash; the entity exactly where the track says; its drawing on its chassis, with
// its label, and its box look's wheels steered and rolled by the pose's steer and speed.
const char *kPoseWorld = R"(
meta: { min_libcvc: "0.0.0" }
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee, controllers: [pose], look: { box: {} } }
scene:
  physics: {}
  world: { source: square, cell: 2 }
  entity_labels: { offset: 2.5 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [120, 120, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)";

// Where the script puts entity 1 at frame k (untimed before frame 120, timed after, 1/60 s
// apart).
std::array<double, 5> pose_track(int k) {
  return {-30.0 + 0.125 * k, 5.0 + 0.0625 * k, 0.25 - 0.00390625 * k, 7.5,
          k % 60 < 30 ? 0.125 : -0.125};
}

void pose_script(cvc::state &st, int frame) {
  if (frame == 1) {
    st("vehicles.spawn.controller").value("pose");
    st("vehicles.spawn.request").value("1");
  }
  if (frame < 2 || frame > 240)
    return;
  const std::array<double, 5> p = pose_track(frame);
  char buf[32];
  const char *const leaves[5] = {"x", "y", "heading", "speed", "steer"};
  for (int i = 0; i < 5; ++i) {
    std::snprintf(buf, sizeof(buf), "%.17g", p[static_cast<std::size_t>(i)]);
    st(std::string("entities.1.pose.") + leaves[i]).value(std::string(buf));
  }
  if (frame >= 120) {
    std::snprintf(buf, sizeof(buf), "%.17g", frame / 60.0);
    st("entities.1.pose.t").value(std::string(buf));
  }
}

void test_vehicle_poses_headless_and_drawn() {
  std::printf("test_vehicle_poses_headless_and_drawn\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no Jolt in this build\n");
    return;
  }
  const auto load = [](cvc::app &app) {
    cvc::ariadne::register_world_block(app);
    cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, kPoseWorld);
    if (r.ok) {
      cvc::ariadne::world_rules rules = cvc::ariadne::default_world_rules();
      CHECK(cvc::ariadne::parse_world_block(*cvc::ariadne::find_world_block(r), rules).empty());
      r.scene.vehicles = rules.vehicles;
    }
    return r;
  };
  constexpr int kFrames = 300; // five seconds at 60 frames per second

  cvc::app drawn_app;
  cvc::ariadne::LoadResult r = load(drawn_app);
  CHECK(r.ok);
  if (!r.ok)
    return;
  SceneGraph sg(drawn_app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &warnings);
  CHECK(out.world != nullptr && out.world->vehicles() != nullptr);
  if (!out.world || !out.world->vehicles())
    return;
  cvc::state &dst = cvc::state::instance(drawn_app);
  int exact = 0, untimed = 0;
  for (int i = 0; i < kFrames; ++i) {
    pose_script(dst, i);
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
    // Untimed, it is exactly where the keys put it, from the frame they were written in.
    if (i >= 2 && i < 120 && out.world->vehicles()->size() == 1) {
      const cvc::ariadne::entity_info e = out.world->vehicles()->entities()[0];
      const std::array<double, 5> p = pose_track(i);
      ++untimed;
      if (e.x == p[0] && e.y == p[1] && e.yaw == p[2] && e.speed == p[3] && e.steer == p[4])
        ++exact;
    }
  }
  CHECK(untimed == 118);
  CHECK(exact == untimed);

  cvc::app headless_app;
  std::unique_ptr<cvc::ariadne::SceneWorld> world =
      cvc::ariadne::SceneWorld::from_scene(headless_app, r.scene, "", "physics", &warnings);
  CHECK(world != nullptr && world->vehicles() != nullptr);
  if (!world || !world->vehicles())
    return;
  cvc::state &hst = cvc::state::instance(headless_app);
  cvc::ariadne::SceneClockDriver clock(headless_app, r.scene.clock);
  for (int i = 0; i < kFrames; ++i) {
    pose_script(hst, i);
    clock.drive(1.0 / 60.0);
    world->follow(headless_app.world_clock());
  }

  CHECK(world->vehicles()->size() == 1 && out.world->vehicles()->size() == 1);
  CHECK(world->world().tick() == out.physics->tick());
  CHECK(out.physics->vehicle_count() == 0); // no plant: no Jolt vehicle, a kinematic box
  const std::uint64_t drawn_hash = out.physics->state_hash();
  const std::uint64_t headless_hash = world->world().state_hash();
  std::printf("  drawn %016llx, headless %016llx after %llu ticks\n",
              static_cast<unsigned long long>(drawn_hash),
              static_cast<unsigned long long>(headless_hash),
              static_cast<unsigned long long>(world->world().tick()));
  CHECK(drawn_hash == headless_hash);
  if (cvc::physics::backend_capabilities(cvc::physics::backend::jolt) &
      cvc::physics::cap_determinism_cross_platform)
    CHECK(headless_hash == 0x696bba66a89b43ecull); // on every platform: no plant decides it
  if (world->vehicles()->size() != 1 || out.world->vehicles()->size() != 1)
    return;
  const cvc::ariadne::entity_info h = world->vehicles()->entities()[0];
  const cvc::ariadne::entity_info d = out.world->vehicles()->entities()[0];
  // The track's last pose, held since frame 240 (timed, so reached a span after it came).
  const std::array<double, 5> last = pose_track(240);
  CHECK(h.x == last[0] && h.y == last[1] && h.yaw == last[2]);
  CHECK(d.x == h.x && d.y == h.y && d.z == h.z && d.yaw == h.yaw);
  CHECK(hst("entities.1.command.by").value() == "pose");
  // Drawn on its chassis, labelled with its name.
  std::shared_ptr<cvc::gl::GraphicsNode> node = sg.getGraphics("vehicle_1");
  CHECK(node != nullptr);
  CHECK(out.vehicle_markers.size() == 1);
  if (!node || out.vehicle_markers.empty())
    return;
  CHECK(out.physics_binding->bound(out.vehicle_markers[0].second));
  const vtkMatrix4x4 *m = node->getTransform();
  CHECK(near(m->GetElement(0, 3), d.x, 1e-3));
  CHECK(near(m->GetElement(1, 3), d.y, 1e-3));
  CHECK(near(m->GetElement(2, 3), d.z, 1e-3));
  const std::shared_ptr<cvc::gl::LabelLayerNode> layer = cvc::gl::LabelLayerNode::of(sg);
  const std::vector<cvc::gl::LabelLayerNode::Id> labels = layer->labelsOf(node.get());
  CHECK(labels.size() == 1 && layer->text(labels[0]) == d.spec.name);
  // Its front left wheel: steered by the pose's steer, rolled by its speed (Rz(steer) Ry(spin)).
  std::shared_ptr<cvc::gl::GraphicsNode> wheel = node->findChildByName("vehicle_1_wheel0");
  CHECK(wheel != nullptr);
  const std::vector<cvc::ariadne::entity_wheel> wheels = out.world->vehicles()->wheels(1);
  CHECK(wheels.size() == 4);
  if (wheel && wheels.size() == 4) {
    const vtkMatrix4x4 *w = wheel->getTransform();
    CHECK(wheels[0].spin != 0.0);
    CHECK(near(wheels[0].steer, last[4], 1e-12));
    CHECK(near(w->GetElement(2, 0), -std::sin(wheels[0].spin), 1e-6));
    CHECK(near(w->GetElement(1, 0), std::sin(last[4]) * std::cos(wheels[0].spin), 1e-6));
  }
}

// Every plant in a scene world (L2b): a Jolt Humvee, the dynamic single track, the dynamic four
// wheels and the kinematic bicycle, spawned on the keys and run by nav@1 on a hill, reach the same
// state headless and drawn, bit for bit; and the drawn four-wheeler's wheels are posed where its
// plant has them.
const char *kPlantWorld = R"(
meta: { min_libcvc: "0.0.0" }
world:
  vehicles:
    archetypes:
      jolt: { plant: jolt, vehicle: humvee, look: { box: {} } }
      bike: { plant: dynamic, vehicle: humvee, look: { box: {} } }
      four: { plant: dynamic4w, vehicle: humvee, look: { box: {} } }
      walker: { plant: kinematic, vehicle: humvee, look: { box: {} } }
scene:
  physics: {}
  world: { source: hill, cell: 2 }
  nodes:
    - node: hill
      world: hill
      source:
        heightfield:
          size: 160
          resolution: 81
          layers:
            - { kind: dome, amplitude: 2.0, radius: 30 }
      body: static
)";

void plant_script(cvc::state &st, int frame) {
  static const char *const kinds[4] = {"jolt", "bike", "four", "walker"};
  static const char *const at[4] = {"-40,-30,30", "-30,30,-30", "30,-30,150", "30,30,-150"};
  if (frame >= 1 && frame <= 4) {
    st("vehicles.selected").value(kinds[frame - 1]);
    st("vehicles.spawn.at").value(at[frame - 1]);
    st("vehicles.spawn.request").value(std::to_string(frame));
  }
}

void test_every_plant_headless_and_drawn() {
  std::printf("test_every_plant_headless_and_drawn\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no Jolt in this build\n");
    return;
  }
  const auto load = [](cvc::app &app) {
    cvc::ariadne::register_world_block(app);
    cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, kPlantWorld);
    if (r.ok) {
      cvc::ariadne::world_rules rules = cvc::ariadne::default_world_rules();
      CHECK(cvc::ariadne::parse_world_block(*cvc::ariadne::find_world_block(r), rules).empty());
      r.scene.vehicles = rules.vehicles;
    }
    return r;
  };
  constexpr int kFrames = 360; // six seconds at 60 frames per second

  cvc::app drawn_app;
  cvc::ariadne::LoadResult r = load(drawn_app);
  CHECK(r.ok);
  if (!r.ok)
    return;
  SceneGraph sg(drawn_app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &warnings);
  CHECK(out.world != nullptr && out.world->vehicles() != nullptr);
  if (!out.world || !out.world->vehicles())
    return;
  cvc::state &dst = cvc::state::instance(drawn_app);
  for (int i = 0; i < kFrames; ++i) {
    plant_script(dst, i);
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  }

  cvc::app headless_app;
  std::unique_ptr<cvc::ariadne::SceneWorld> world =
      cvc::ariadne::SceneWorld::from_scene(headless_app, r.scene, "", "physics", &warnings);
  CHECK(world != nullptr && world->vehicles() != nullptr);
  if (!world || !world->vehicles())
    return;
  cvc::state &hst = cvc::state::instance(headless_app);
  cvc::ariadne::SceneClockDriver clock(headless_app, r.scene.clock);
  for (int i = 0; i < kFrames; ++i) {
    plant_script(hst, i);
    clock.drive(1.0 / 60.0);
    world->follow(headless_app.world_clock());
  }

  CHECK(world->vehicles()->size() == 4 && out.world->vehicles()->size() == 4);
  CHECK(world->world().tick() == out.physics->tick());
  const std::uint64_t drawn_hash = out.physics->state_hash();
  const std::uint64_t headless_hash = world->world().state_hash();
  if (drawn_hash != headless_hash)
    std::printf("  drawn %016llx, headless %016llx\n", static_cast<unsigned long long>(drawn_hash),
                static_cast<unsigned long long>(headless_hash));
  CHECK(drawn_hash == headless_hash);
  // Each drove, and each is drawn on its chassis.
  for (const cvc::ariadne::entity_info &e : world->vehicles()->entities()) {
    std::printf("  %s: %.1f m at %.1f m/s\n", e.plant.c_str(), e.distance, e.speed);
    CHECK(e.distance > 3.0);
  }
  CHECK(out.vehicle_markers.size() == 4);
  // The drawn four-wheeler's front-left wheel where its plant has it, not where the box look put
  // it.
  const std::vector<cvc::ariadne::entity_wheel> wheels = out.world->vehicles()->wheels(3);
  std::shared_ptr<cvc::gl::GraphicsNode> four = sg.getGraphics("vehicle_3");
  std::shared_ptr<cvc::gl::GraphicsNode> fl =
      four ? four->findChildByName("vehicle_3_wheel0") : nullptr;
  CHECK(fl != nullptr && wheels.size() == 4);
  if (fl && wheels.size() == 4) {
    const vtkMatrix4x4 *m = fl->getTransform();
    CHECK(near(m->GetElement(0, 3), wheels[0].x, 1e-6));
    CHECK(near(m->GetElement(1, 3), wheels[0].y, 1e-6));
    CHECK(near(m->GetElement(2, 3), wheels[0].z, 1e-6));
  }
}

// The boat node (OCEAN-AND-WEATHER-ROADMAP.md D6) and the boats drawn (D5): a launch on a lake (a
// bowl under a water node at -1 m) drawn with its hull at its boat's pose once the boat is in the
// world, its helm keys moving it; the water's shaders told to keep out of its hull; and a boat
// world vehicle drawn with its hull, not the chassis box stand-in.
void test_boats_drawn() {
  cvc::app app;
  std::printf("test_boats_drawn\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no Jolt in this build\n");
    return;
  }
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  cvc::ariadne::register_world_block(app);
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, R"(
meta: { min_libcvc: "0.0.0" }
world:
  vehicles:
    archetypes:
      launch: { plant: dynamicboat:launch, controllers: [none] }
scene:
  physics: {}
  world: { source: lake, cell: 2 }
  nodes:
    - node: terrain
      world: lake
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
)");
  CHECK(r.ok);
  if (!r.ok)
    return;
  cvc::ariadne::world_rules rules = cvc::ariadne::default_world_rules();
  CHECK(cvc::ariadne::parse_world_block(*cvc::ariadne::find_world_block(r), rules).empty());
  r.scene.vehicles = rules.vehicles;
  r.scene.spawns = rules.spawns;
  SceneGraph sg(app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &warnings);
  for (const std::string &w : warnings)
    std::printf("  warning: %s\n", w.c_str());
  CHECK(out.world != nullptr);
  if (!out.world)
    return;
  std::shared_ptr<cvc::gl::GraphicsNode> boat = sg.getGraphics("skiff");
  CHECK(boat != nullptr);
  auto hull =
      boat ? std::dynamic_pointer_cast<cvc::gl::GeometryNode>(boat->findChildByName("skiff_hull"))
           : nullptr;
  CHECK(hull != nullptr && hull->getGeometry() && hull->getGeometry()->num_points() == 484);
  for (int i = 0; i < 30; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  cvc::ariadne::BoatRuntime *b = out.world->boat("skiff");
  CHECK(b != nullptr && b->placed());
  if (!b || !b->placed() || !boat)
    return;
  CHECK(boat->isVisible());
  CHECK(out.physics_binding->bound(b->hull()));
  const cvc::physics::boat_state s0 = b->state();
  const vtkMatrix4x4 *m = boat->getTransform();
  CHECK(near(m->GetElement(0, 3), s0.body.position.x, 0.05));
  CHECK(near(m->GetElement(2, 3), -1.0 + 0.15, 0.1)); // afloat at its draft
  // Its helm: ahead and left.
  cvc::state &st = cvc::state::instance(app);
  st("skiff.throttle").value(1.0);
  st("skiff.rudder").value(0.5);
  for (int i = 0; i < 300; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  const cvc::physics::boat_state s1 = b->state();
  std::printf(
      "  the helm moved it %.1f m and turned it %.2f rad\n",
      std::hypot(s1.body.position.x - s0.body.position.x, s1.body.position.y - s0.body.position.y),
      s1.heading - s0.heading);
  CHECK(std::hypot(s1.body.position.x - s0.body.position.x,
                   s1.body.position.y - s0.body.position.y) > 5.0);
  CHECK(s1.heading - s0.heading > 0.1);
  CHECK(near(boat->getTransform()->GetElement(0, 3), s1.body.position.x, 0.5));
  // The water keeps out of its hull: its half length and beam, about where it is.
  std::shared_ptr<cvc::gl::GraphicsNode> water = sg.getGraphics("water");
  auto far = water ? std::dynamic_pointer_cast<cvc::gl::GeometryNode>(water->findChildByName("far"))
                   : nullptr;
  std::array<float, 3> p{}, e{};
  CHECK(far && far->shaderUniform3f("uHullP0", p) && far->shaderUniform3f("uHullE0", e));
  CHECK(near(e[0], 2.0, 1e-6) && near(e[1], 0.75, 1e-6));
  CHECK(near(p[0], s1.body.position.x, 0.5) && near(p[1], s1.body.position.y, 0.5));
  // between its bottom and its sheer, its waterline (the level, -1 m) in between
  float bottom = 0.0f;
  CHECK(far && far->shaderUniformf("uHullB0", bottom));
  CHECK(near(e[2] - bottom, b->sheer() - b->bottom(), 0.05) && bottom < -1.0f && e[2] > -1.0f);
  std::array<float, 3> e1{};
  CHECK(far && far->shaderUniform3f("uHullE1", e1) && e1[0] == 0.0f); // no second boat yet
  // A boat world vehicle: drawn with its hull, and the water keeps out of it too.
  st("vehicles.selected").value(std::string("launch"));
  st("vehicles.spawn.controller").value(std::string("none"));
  st("vehicles.spawn.at").value("20,0,0");
  st("vehicles.spawn.request").value("1");
  for (int i = 0; i < 30; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  CHECK(out.world->vehicles() && out.world->vehicles()->size() == 1);
  std::shared_ptr<cvc::gl::GraphicsNode> v = sg.getGraphics("vehicle_1");
  CHECK(v != nullptr && v->findChildByName("vehicle_1_hull") != nullptr);
  CHECK(v != nullptr && v->findChildByName("vehicle_1_wheel0") == nullptr); // no box look's wheels
  CHECK(far && far->shaderUniform3f("uHullE1", e1) && near(e1[0], 2.0, 1e-6));
}

// ── Vehicle models fitted (fit_vehicle_geometry) ────────────────────────────────────────────

// A closed box from (x0, y0, z0) to (x1, y1, z1), its triangles wound outward, added to `g`.
void add_box(cvc::geometry &g, double x0, double y0, double z0, double x1, double y1, double z1) {
  using idx = cvc::geometry::index_t;
  auto &pts = g.points();
  auto &tris = g.tris();
  const idx b = static_cast<idx>(pts.size());
  for (int k = 0; k < 8; ++k)
    pts.push_back({k & 1 ? x1 : x0, k & 2 ? y1 : y0, k & 4 ? z1 : z0});
  const idx q[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4},
                       {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
  for (const auto &f : q) {
    tris.push_back({b + f[0], b + f[1], b + f[2]});
    tris.push_back({b + f[0], b + f[2], b + f[3]});
  }
}

// A boat as a Y-up file has it (a glTF's frame), standing on y = 0: an 8 x 1.2 x 2 m hull along x
// and a cabin 1.8 m tall over its +x half, so its bow, the end with fewer points, is at -x. Its
// beam (z, 2 m) is its shortest extent, not its height (y, 3 m). `lift` raises it off y = 0.
cvc::geometry y_up_boat(cvc::app &app, double lift = 0.0) {
  cvc::geometry g(app);
  add_box(g, -4.0, lift, -1.0, 4.0, lift + 1.2, 1.0);
  add_box(g, 0.5, lift + 1.2, -0.8, 3.5, lift + 3.0, 0.8);
  return g;
}

struct bounds {
  double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
};
bounds bounds_of(const cvc::geometry &g) {
  bounds b;
  for (const auto &v : g.const_points())
    for (int k = 0; k < 3; ++k) {
      b.lo[k] = std::min(b.lo[k], static_cast<double>(v[k]));
      b.hi[k] = std::max(b.hi[k], static_cast<double>(v[k]));
    }
  return b;
}

// The volume a closed mesh's triangles enclose, signed: + when they are wound outward.
double signed_volume(const cvc::geometry &g) {
  const auto &p = g.const_points();
  double v = 0.0;
  for (const auto &t : g.const_tris()) {
    const auto &a = p[t[0]], &b = p[t[1]], &c = p[t[2]];
    v += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
         a[2] * (b[0] * c[1] - b[1] * c[0]);
  }
  return v / 6.0;
}

// The highest z of the points on one side of x = 0 (+1 the +x side, -1 the -x side).
double top_on(const cvc::geometry &g, int side) {
  double top = -1e30;
  for (const auto &v : g.const_points())
    if (v[0] * side > 0.0)
      top = std::max(top, static_cast<double>(v[2]));
  return top;
}

// The fitter's frame: up the axis a file stands on (Y-up or Z-up), else its shortest extent; its
// length along x, scaled, its nose at +x; never mirrored; and the axes a caller names.
void test_fit_vehicle_geometry() {
  cvc::app app;
  std::printf("test_fit_vehicle_geometry\n");
  const double L = 7.8, sc = L / 8.0;
  const double vol = signed_volume(y_up_boat(app));
  CHECK(near(vol, 8.0 * 1.2 * 2.0 + 3.0 * 1.8 * 1.6, 1e-9)); // the boxes are wound outward
  // Y-up, standing on y = 0: y is up, though z is its shortest extent; the bow (-x) to +x.
  {
    const cvc::geometry f = ari::fit_vehicle_geometry(y_up_boat(app), L);
    const bounds b = bounds_of(f);
    CHECK(near(b.lo[0], -0.5 * L, 1e-5) && near(b.hi[0], 0.5 * L, 1e-5));
    CHECK(near(b.lo[1], -sc, 1e-5) && near(b.hi[1], sc, 1e-5));
    CHECK(near(b.lo[2], 0.0, 1e-5) && near(b.hi[2], 3.0 * sc, 1e-5));
    CHECK(near(top_on(f, -1), 3.0 * sc, 1e-5) && near(top_on(f, 1), 1.2 * sc, 1e-5)); // cabin aft
    CHECK(near(signed_volume(f), vol * sc * sc * sc, 1e-3)); // a rotation: still wound outward
  }
  // The same boat as a Z-up file: the same fit.
  {
    const cvc::geometry y = y_up_boat(app);
    cvc::geometry z(app);
    for (const auto &v : y.const_points())
      z.points().push_back({v[0], -v[2], v[1]}); // a rotation about x
    z.tris() = y.const_tris();
    const cvc::geometry f = ari::fit_vehicle_geometry(z, L);
    const bounds b = bounds_of(f);
    CHECK(near(b.lo[2], 0.0, 1e-5) && near(b.hi[2], 3.0 * sc, 1e-5));
    CHECK(near(top_on(f, -1), 3.0 * sc, 1e-5) && near(top_on(f, 1), 1.2 * sc, 1e-5));
    CHECK(near(signed_volume(f), vol * sc * sc * sc, 1e-3));
  }
  // Off its ground plane: its shortest extent (z, the beam) is up, as before.
  {
    const cvc::geometry f = ari::fit_vehicle_geometry(y_up_boat(app, -1.5), L);
    const bounds b = bounds_of(f);
    CHECK(near(b.lo[2], 0.0, 1e-5) && near(b.hi[2], 2.0 * sc, 1e-5));
    CHECK(signed_volume(f) > 0.0);
  }
  // Named axes win: y up and x forward off its ground plane, and the nose either way.
  for (const int nose : {1, -1}) {
    ari::vehicle_fit_axes axes;
    axes.up = 1;
    axes.forward = 0;
    axes.nose = nose;
    const cvc::geometry f = ari::fit_vehicle_geometry(y_up_boat(app, -1.5), L, axes);
    const bounds b = bounds_of(f);
    CHECK(near(b.lo[2], 0.0, 1e-5) && near(b.hi[2], 3.0 * sc, 1e-5));
    CHECK(near(top_on(f, nose), 3.0 * sc, 1e-5)); // the cabin's end (+x in the file) at `nose`
    CHECK(near(signed_volume(f), vol * sc * sc * sc, 1e-3));
  }
  bool threw = false;
  try {
    ari::fit_vehicle_geometry(cvc::geometry(app), L);
  } catch (const std::runtime_error &) {
    threw = true;
  }
  CHECK(threw);
}

// The models the packages install, when they are there: TacticalBoat (vehicle-tactical-boat, a
// Y-up glTF) stands on its keel with its bow (the end whose bottom rises) at +x; the Humvee (a
// Z-up glTF) keeps its shortest extent up.
void test_installed_vehicle_models_fit() {
  cvc::app app;
  std::printf("test_installed_vehicle_models_fit\n");
  const std::string boat = cvc::ariadne::find_vehicle_model("TacticalBoat.glb");
  if (boat.empty()) {
    std::printf("  TacticalBoat.glb: skipped (vehicle-tactical-boat not installed)\n");
  } else {
    try {
      const cvc::geometry f = ari::fit_vehicle_geometry(cvc::read_model(app, boat).merged(), 7.8);
      const bounds b = bounds_of(f);
      std::printf("  TacticalBoat.glb: %.2f x %.2f x %.2f m\n", b.hi[0] - b.lo[0],
                  b.hi[1] - b.lo[1], b.hi[2] - b.lo[2]);
      CHECK(near(b.hi[0] - b.lo[0], 7.8, 1e-3));
      CHECK(b.hi[2] - b.lo[2] > b.hi[1] - b.lo[1]); // its height (3 m) up, not its beam
      // The bow: the bottom rises toward it (the forefoot), not toward the flat transom.
      const auto rise = [&](int side) {
        std::vector<double> z;
        for (const auto &v : f.const_points())
          if (v[0] * side > 0.3 * 7.8)
            z.push_back(v[2]);
        std::sort(z.begin(), z.end());
        const std::size_t n = std::max<std::size_t>(1, z.size() / 5);
        double sum = 0.0;
        for (std::size_t i = 0; i < n && i < z.size(); ++i)
          sum += z[i];
        return sum / static_cast<double>(n);
      };
      std::printf("  its bottom's rise: %.2f m at +x, %.2f m at -x\n", rise(1), rise(-1));
      CHECK(rise(1) > rise(-1));
    } catch (const std::exception &e) {
      std::printf("  TacticalBoat.glb: %s\n", e.what());
      CHECK(false);
    }
  }
  const std::string humvee = cvc::ariadne::find_vehicle_model("humvee");
  if (humvee.empty()) {
    std::printf("  Humvee.glb: skipped (not installed)\n");
  } else {
    try {
      const cvc::geometry f =
          ari::fit_vehicle_geometry(cvc::read_model(app, humvee).merged(), 4.84);
      const bounds b = bounds_of(f);
      std::printf("  Humvee.glb: %.2f x %.2f x %.2f m\n", b.hi[0] - b.lo[0], b.hi[1] - b.lo[1],
                  b.hi[2] - b.lo[2]);
      CHECK(near(b.hi[0] - b.lo[0], 4.84, 1e-3));
      CHECK(b.hi[2] - b.lo[2] < b.hi[1] - b.lo[1]);
    } catch (const std::exception &e) {
      std::printf("  Humvee.glb: %s\n", e.what());
      CHECK(false);
    }
  }
}

// A boat node's model: `model:` a file (a Y-up OBJ written here) is drawn in place of the hull,
// fitted to the boat's length overall with its keel on the hull's bottom; the water keeps out of
// the box inside the hull at its waterline (BoatRuntime::cutout); a `model:` not found warns and
// draws the hull; and the tactical preset draws TacticalBoat when it is installed, else its hull
// without a warning.
void test_boat_models_drawn() {
  cvc::app app;
  std::printf("test_boat_models_drawn\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no Jolt in this build\n");
    return;
  }
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path dir = fs::temp_directory_path(ec) / "cvcgl_ariadne_boat_models";
  fs::create_directories(dir, ec);
  const fs::path obj = dir / "y_up_boat.obj";
  {
    const cvc::geometry g = y_up_boat(app);
    std::ofstream out(obj);
    for (const auto &v : g.const_points())
      out << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
    for (const auto &t : g.const_tris())
      out << "f " << t[0] + 1 << " " << t[1] + 1 << " " << t[2] + 1 << "\n";
  }
  bool reads_obj = false;
  try {
    reads_obj = cvc::read_model(app, obj.string()).merged().num_tris() == 24;
  } catch (const std::exception &) {
  }
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  const std::string doc = R"(
meta: { min_libcvc: "0.0.0" }
scene:
  physics: {}
  world: { source: lake, cell: 2 }
  nodes:
    - node: terrain
      world: lake
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
      body: {}
    - node: patrol
      type: boat
      water: water
      preset: tactical
      model: ")" + obj.generic_string() +
                          R"("
      transform: { position: [-20, 0, 0] }
    - node: stray
      type: boat
      water: water
      preset: jonboat
      model: no_such_boat
      transform: { position: [15, 0, 0] }
    - node: tactical
      type: boat
      water: water
      preset: tactical
      transform: { position: [0, -15, 0] }
)";
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, doc);
  CHECK(r.ok);
  if (!r.ok)
    return;
  SceneGraph sg(app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &warnings);
  int stray_warnings = 0, tactical_warnings = 0;
  for (const std::string &w : warnings) {
    std::printf("  warning: %s\n", w.c_str());
    stray_warnings += w.find("no model 'no_such_boat'") != std::string::npos;
    tactical_warnings += w.find("TacticalBoat") != std::string::npos;
  }
  CHECK(stray_warnings == 1);
  CHECK(tactical_warnings == 0);
  const auto child = [&](const std::string &node,
                         const std::string &name) -> std::shared_ptr<GeometryNode> {
    std::shared_ptr<cvc::gl::GraphicsNode> g = sg.getGraphics(node);
    return g ? std::dynamic_pointer_cast<GeometryNode>(g->findChildByName(name)) : nullptr;
  };
  if (reads_obj) {
    const std::shared_ptr<GeometryNode> m = child("patrol", "patrol_model");
    CHECK(m != nullptr && child("patrol", "patrol_hull") == nullptr);
    if (m && m->getGeometry()) {
      const bounds b = bounds_of(*m->getGeometry());
      const double sc = 7.8 / 8.0;
      CHECK(near(b.lo[0], -3.9, 1e-4) && near(b.hi[0], 3.9, 1e-4));
      CHECK(near(b.lo[2], 0.0, 1e-4) && near(b.hi[2], 3.0 * sc, 1e-4)); // y up in the file
      // Its keel on the tactical hull's keel (its lowest station point, body z).
      CHECK(near(m->getTransform()->GetElement(2, 3), -0.5545, 1e-4));
    }
  } else {
    std::printf("  OBJ not read in this build: the patrol boat's model unchecked\n");
    CHECK(child("patrol", "patrol_hull") != nullptr);
  }
  CHECK(child("stray", "stray_hull") != nullptr && child("stray", "stray_model") == nullptr);
  const bool installed = !cvc::ariadne::find_vehicle_model("TacticalBoat.glb").empty();
  std::printf("  TacticalBoat.glb %s\n", installed ? "installed: the tactical boat drawn as it"
                                                   : "not installed: the tactical boat's hull");
  CHECK(child("tactical", installed ? "tactical_model" : "tactical_hull") != nullptr);
  // The model rides the boat's hull body as the hull did.
  for (int i = 0; i < 30; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  cvc::ariadne::BoatRuntime *b = out.world ? out.world->boat("patrol") : nullptr;
  CHECK(b != nullptr && b->placed());
  std::shared_ptr<cvc::gl::GraphicsNode> patrol = sg.getGraphics("patrol");
  if (b && b->placed() && patrol) {
    CHECK(patrol->isVisible());
    CHECK(near(patrol->getTransform()->GetElement(0, 3), b->state().body.position.x, 0.05));
  }
  // The water's shaders keep out of the tactical hull's cut-out (inside its V hull at its
  // waterline, centred aft of its origin), not its bounding box.
  if (b && b->placed()) {
    const cvc::physics::waterline_box &c = b->cutout();
    std::printf("  the tactical hull's water cut-out: centre %+.3f m, half length %.3f m, half "
                "beam %.3f m (its box: %.3f, %.3f)\n",
                c.x, c.half_length, c.half_beam, b->half_length(), b->half_beam());
    CHECK(c.half_beam < b->half_beam() && c.half_length < b->half_length());
    std::shared_ptr<cvc::gl::GraphicsNode> water = sg.getGraphics("water");
    auto far =
        water ? std::dynamic_pointer_cast<GeometryNode>(water->findChildByName("far")) : nullptr;
    bool found = false;
    for (const char *k : {"uHullE0", "uHullE1", "uHullE2", "uHullE3"}) {
      std::array<float, 3> e{};
      if (far && far->shaderUniform3f(k, e) && near(e[1], c.half_beam, 1e-4) &&
          near(e[0], c.half_length, 1e-4))
        found = true;
    }
    CHECK(found);
  }
  fs::remove_all(dir, ec);
}

// A boat's model from a document data: entry's package (here a local directory, as a cvcpkg://
// package unpacks): drawn as its hull while the package loads, then the model in its place, fitted
// on the hull's keel; a package that cannot load leaves the hull. vehicle_model_state says how far
// a data look has got: pending, then ready, or failed.
void test_boat_model_from_data() {
  cvc::app app;
  std::printf("test_boat_model_from_data\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no Jolt in this build\n");
    return;
  }
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path dir = fs::temp_directory_path(ec) / "cvcgl_ariadne_boat_data";
  const fs::path shared = dir / "pkg" / "share" / "cvc-scenes" / "shared";
  fs::create_directories(shared, ec);
  {
    const cvc::geometry g = y_up_boat(app);
    std::ofstream out(shared / "y_up_boat.obj");
    for (const auto &v : g.const_points())
      out << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
    for (const auto &t : g.const_tris())
      out << "f " << t[0] + 1 << " " << t[1] + 1 << " " << t[2] + 1 << "\n";
  }
  if (!cvc::can_read_model_from_memory(".obj")) {
    std::printf("  skipped: no model reader for .obj from memory in this build\n");
    fs::remove_all(dir, ec);
    return;
  }
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  const std::string doc = R"(
meta: { min_libcvc: "0.0.0" }
data:
  boatpkg: { uri: ")" + (dir / "pkg").generic_string() +
                          R"(", path: share/cvc-scenes/shared, load: lazy }
  gone: { uri: ")" + (dir / "no_such_pkg").generic_string() +
                          R"(", path: share/cvc-scenes/shared, load: lazy }
scene:
  physics: {}
  world: { source: lake, cell: 2 }
  nodes:
    - node: terrain
      world: lake
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
      body: {}
    - node: patrol
      type: boat
      water: water
      preset: tactical
      model: { data: boatpkg, file: y_up_boat.obj }
      transform: { position: [-20, 0, 0] }
    - node: lost
      type: boat
      water: water
      preset: jonboat
      model: { data: gone, file: y_up_boat.obj }
      transform: { position: [15, 0, 0] }
)";
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, doc);
  CHECK(r.ok);
  if (!r.ok)
    return;
  CHECK(r.scene.data.size() == 2);
  SceneGraph sg(app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &warnings);
  for (const std::string &w : warnings)
    std::printf("  warning: %s\n", w.c_str());
  const auto child = [&](const std::string &node,
                         const std::string &name) -> std::shared_ptr<GeometryNode> {
    std::shared_ptr<cvc::gl::GraphicsNode> g = sg.getGraphics(node);
    return g ? std::dynamic_pointer_cast<GeometryNode>(g->findChildByName(name)) : nullptr;
  };
  // Either already loaded (the model) or still loading (the hull for now), never both.
  CHECK((child("patrol", "patrol_model") != nullptr) !=
        (child("patrol", "patrol_hull") != nullptr));
  // The package loads on the store's worker: wait for it, then the next frame swaps the model in.
  cvc::ariadne::DataSpec spec;
  spec.uri = (dir / "pkg").generic_string();
  spec.path = "share/cvc-scenes/shared";
  const std::shared_ptr<cvc::ariadne::DataStore> store = cvc::ariadne::DataStore::of(app);
  CHECK(store->wait(store->load(spec), 30.0));
  cvc::ariadne::DataSpec gone = spec;
  gone.uri = (dir / "no_such_pkg").generic_string();
  CHECK(!store->wait(store->load(gone), 30.0)); // failed, not ready
  for (int i = 0; i < 5; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  const std::shared_ptr<GeometryNode> m = child("patrol", "patrol_model");
  CHECK(m != nullptr && child("patrol", "patrol_hull") == nullptr);
  if (m && m->getGeometry()) {
    const bounds b = bounds_of(*m->getGeometry());
    CHECK(near(b.lo[0], -3.9, 1e-4) && near(b.hi[0], 3.9, 1e-4)); // the boat's length overall
    CHECK(near(b.lo[2], 0.0, 1e-4) && near(b.hi[2], 3.0 * 7.8 / 8.0, 1e-4)); // y up in the file
  }
  CHECK(child("lost", "lost_hull") != nullptr && child("lost", "lost_model") == nullptr);
  // The state a view polls.
  cvc::ariadne::vehicle_look look;
  look.kind = "data";
  look.uri = spec.uri;
  look.subpath = spec.path;
  look.path = "y_up_boat.obj";
  look.fit = true;
  CHECK(ari::vehicle_model_state(app, *out.vehicle_models, look, 7.8) ==
        ari::vehicle_model_status::ready);
  cvc::ariadne::vehicle_look lost = look;
  lost.uri = gone.uri;
  std::vector<std::string> why;
  CHECK(ari::vehicle_model_state(app, *ari::make_vehicle_model_cache(), lost, 7.8, &why) ==
        ari::vehicle_model_status::failed);
  CHECK(!why.empty());
  for (const std::string &w : why)
    std::printf("  failed as it should: %s\n", w.c_str());
  cvc::ariadne::vehicle_look plain;
  plain.kind = "box";
  CHECK(ari::vehicle_model_state(app, *out.vehicle_models, plain, 4.0) ==
        ari::vehicle_model_status::failed);
  fs::remove_all(dir, ec);
}

// A geometry node's `model:` (a crate drawn as its model, floating as its box): the model fitted
// into the box the node's own geometry spans, put in from a scene tick, so the node's body is made
// from its box even when the model is ready at realize (a file); from a data: package once it has
// loaded; a data: entry the document lacks keeps the box, with a warning.
void test_node_model_from_data() {
  cvc::app app;
  std::printf("test_node_model_from_data\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no Jolt in this build\n");
    return;
  }
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path dir = fs::temp_directory_path(ec) / "cvcgl_ariadne_node_model";
  const fs::path shared = dir / "pkg" / "share" / "cvc-scenes" / "shared";
  fs::create_directories(shared, ec);
  {
    // An octahedron, 4 x 2 x 1, well away from the origin: Z up, as an OBJ is taken.
    std::ofstream out(shared / "crate.obj");
    out << "v 12 10 10\nv 8 10 10\nv 10 11 10\nv 10 9 10\nv 10 10 10.5\nv 10 10 9.5\n"
           "f 1 3 5\nf 3 2 5\nf 2 4 5\nf 4 1 5\nf 3 1 6\nf 2 3 6\nf 4 2 6\nf 1 4 6\n";
  }
  if (!cvc::can_read_model_from_memory(".obj")) {
    std::printf("  skipped: no model reader for .obj from memory in this build\n");
    fs::remove_all(dir, ec);
    return;
  }
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  const std::string doc = R"(
meta: { min_libcvc: "0.0.0" }
data:
  crates: { uri: ")" + (dir / "pkg").generic_string() +
                          R"(", path: share/cvc-scenes/shared, load: lazy }
scene:
  physics: {}
  nodes:
    - node: packed
      source: { box: { size: [3, 2, 1.5] } }
      transform: { position: [0, 0, 5] }
      material: { color: [0.6, 0.4, 0.2] }
      model: { data: crates, file: crate.obj }
      body: { density: 450 }
    - node: local
      source: { box: { size: [2, 2, 2] } }
      transform: { position: [6, 0, 5] }
      model: ")" + (shared / "crate.obj").generic_string() +
                          R"("
      body: { density: 450 }
    - node: lost
      source: { box: { size: [1, 1, 1] } }
      transform: { position: [-6, 0, 5] }
      model: { data: no_such_entry, file: crate.obj }
      body: { density: 450 }
)";
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, doc);
  CHECK(r.ok);
  if (!r.ok)
    return;
  SceneGraph sg(app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &warnings);
  bool lost_warned = false;
  for (const std::string &w : warnings) {
    std::printf("  warning: %s\n", w.c_str());
    lost_warned = lost_warned || (w.find("'lost'") != std::string::npos &&
                                  w.find("no data: entry 'no_such_entry'") != std::string::npos);
  }
  CHECK(lost_warned);
  const auto node = [&](const std::string &id) {
    return std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(id));
  };
  const auto tris = [&](const std::string &id) -> std::size_t {
    const std::shared_ptr<GeometryNode> g = node(id);
    return g && g->getGeometry() ? g->getGeometry()->num_tris() : 0;
  };
  // Each still its box at realize, the local file's model ready though it is, and a body each.
  const std::size_t box_tris = tris("local");
  CHECK(box_tris > 8);
  CHECK(tris("packed") == box_tris && tris("lost") == box_tris);
  bounds packed_box, local_box;
  if (node("packed") && node("packed")->getGeometry())
    packed_box = bounds_of(*node("packed")->getGeometry());
  if (node("local") && node("local")->getGeometry())
    local_box = bounds_of(*node("local")->getGeometry());
  int bodies = 0;
  for (const auto &[id, body] : out.bodies)
    bodies += id == "packed" || id == "local" || id == "lost";
  CHECK(bodies == 3);
  // The package loads on the store's worker; then a frame puts each model in.
  cvc::ariadne::DataSpec spec;
  spec.uri = (dir / "pkg").generic_string();
  spec.path = "share/cvc-scenes/shared";
  const std::shared_ptr<cvc::ariadne::DataStore> store = cvc::ariadne::DataStore::of(app);
  CHECK(store->wait(store->load(spec), 30.0));
  for (int i = 0; i < 3; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  CHECK(tris("packed") == 8 && tris("local") == 8); // the octahedron
  CHECK(tris("lost") == box_tris);                  // its box
  // Fitted into the box on each axis: the model's 4 x 2 x 1 spans the box's 3 x 2 x 1.5.
  for (const auto &[id, box] : {std::pair{"packed", packed_box}, std::pair{"local", local_box}}) {
    const std::shared_ptr<GeometryNode> g = node(id);
    if (!g || !g->getGeometry())
      continue;
    const bounds b = bounds_of(*g->getGeometry());
    for (int k = 0; k < 3; ++k)
      CHECK(near(b.lo[k], box.lo[k], 1e-6) && near(b.hi[k], box.hi[k], 1e-6));
  }
  CHECK(near(packed_box.hi[0] - packed_box.lo[0], 3.0, 1e-6) &&
        near(packed_box.hi[2] - packed_box.lo[2], 1.5, 1e-6));
  // The state a view polls: a box look has no model.
  cvc::ariadne::vehicle_look plain;
  plain.kind = "box";
  CHECK(ari::boxed_model_state(app, *ari::make_vehicle_model_cache(), plain, {0, 0, 0},
                               {1, 1, 1}) == ari::vehicle_model_status::failed);
  fs::remove_all(dir, ec);
}

int main() {
  test_crate_lands_on_the_drawn_ground();
  test_null_backend_places_and_holds();
  test_problems_warn();
  test_same_world_headless_and_drawn();
  test_same_generated_world_headless_and_drawn();
  test_restart_drawn_world();
  test_worlds_drawn();
  test_vehicles_drawn();
  test_vehicle_intents_headless_and_drawn();
  test_vehicle_poses_headless_and_drawn();
  test_every_plant_headless_and_drawn();
  test_boats_drawn();
  test_fit_vehicle_geometry();
  test_installed_vehicle_models_fit();
  test_boat_models_drawn();
  test_boat_model_from_data();
  test_node_model_from_data();
  if (g_failures) {
    std::printf("cvcgl_ariadne_physics: %d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("cvcgl_ariadne_physics: OK\n");
  return 0;
}
