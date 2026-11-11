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
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_rules.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/vehicle_view.h>
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkMatrix4x4.h>

using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
namespace ari = cvc::gl::ariadne;

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
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(kScene);
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
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(spec);
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
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(R"(
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
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(kSameWorld);
  CHECK(r.ok);
  if (!r.ok)
    return;
  constexpr int kFrames = 240; // four seconds at 60 frames per second

  // Drawn: realized over a SceneGraph, stepped by tick_scene.
  cvc::app drawn_app;
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
  ari::register_world_node_types();
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(kGeneratedWorld);
  CHECK(r.ok);
  if (!r.ok)
    return;
  constexpr int kFrames = 300; // five seconds: the ball falls and lands

  cvc::app drawn_app;
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
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(kSameWorld);
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
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(R"(
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
  std::printf("test_vehicles_drawn\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no Jolt in this build\n");
    return;
  }
  cvc::ariadne::register_world_block();
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(R"(
meta: { min_libcvc: "0.0.0" }
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee }
      racer: { vehicle: roboracer, controllers: [none] }
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
  cvc::app app;
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

  // The preview: the selected archetype on a scene of its own, rebuilt when the pick changes.
  ari::register_vehicle_preview_node();
  cvc::ariadne::LoadResult studio = cvc::ariadne::load_string(R"(
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

int main() {
  test_crate_lands_on_the_drawn_ground();
  test_null_backend_places_and_holds();
  test_problems_warn();
  test_same_world_headless_and_drawn();
  test_same_generated_world_headless_and_drawn();
  test_restart_drawn_world();
  test_worlds_drawn();
  test_vehicles_drawn();
  if (g_failures) {
    std::printf("cvcgl_ariadne_physics: %d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("cvcgl_ariadne_physics: OK\n");
  return 0;
}
