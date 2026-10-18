// Ariadne physics: and body: (WORLD-GEOMETRY-ROADMAP.md, G5). A scene spec puts a crate on a
// heightfield and it lands where the renderer draws the ground. Headless. These checks pin that:
//   1. a heightfield node's cvc::height_field is kept (RealizedScene::height_fields) and is the
//      mesh the node draws;
//   2. each body starts at its node's realized world pose, a nested node's included;
//   3. ticking the scene steps the world on app.world_clock() (driven as by `clock: {}`), and a
//      moving body's node is drawn at the body's pose;
//   4. at rest, the crate sits half its height above the DRAWN mesh (measured on the node's own
//      triangles), and the ball on the dome's peak;
//   5. the null backend places bodies and never moves them; problems warn, never throw.
// The dynamics need Jolt and YAML; without either, those checks are skipped.
//
// NOT assert(): built Release, where NDEBUG makes assert() a no-op.
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/physics/world.h>
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

} // namespace

int main() {
  test_crate_lands_on_the_drawn_ground();
  test_null_backend_places_and_holds();
  test_problems_warn();
  if (g_failures) {
    std::printf("cvcgl_ariadne_physics: %d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("cvcgl_ariadne_physics: OK\n");
  return 0;
}
