// cvc::gl::physics_scene_binding -- a physics world's bodies drawn in a scene
// graph between the world's last two states. Headless: a SceneGraph with no
// renderer, pumped by hand. These checks pin that:
//   1. a bound node's local transform is exactly the interpolated pose (times
//      the visual offset when there is one);
//   2. present() from another thread is posted to the owner thread, never
//      applied where it was made;
//   3. a group sink gets its bodies' poses in group order, without bodies
//      removed from the world;
//   4. a body removed from the world, or a node destroyed with its scene entry,
//      is unbound and the rest keep drawing;
//   5. present(clock, alpha) draws at the world's fraction, not the clock's,
//      for a world that steps once every several quanta;
//   6. a Jolt crate falls in the scene as it falls in the world.
//
// NOT assert(): built Release, where NDEBUG makes assert() a no-op.
#include <array>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/physics_scene_binding.h>
#include <cvc/physics/world.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <vtkMatrix4x4.h>

using cvc::gl::GraphicsNode;
using cvc::gl::physics_scene_binding;
using cvc::gl::SceneGraph;
namespace phys = cvc::physics;

static int g_failures = 0;
static void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

namespace {

using Mat = std::array<double, 16>;

cvc::geometry tri(double s = 0.5) {
  cvc::geometry g;
  const double xyz[3][3] = {{-s, -s, 0.0}, {s, -s, 0.0}, {0.0, s, 0.0}};
  for (auto &v : xyz) {
    cvc::geometry::point_t p;
    p[0] = v[0];
    p[1] = v[1];
    p[2] = v[2];
    g.points().push_back(p);
  }
  cvc::geometry::tri_t t;
  t[0] = 0;
  t[1] = 1;
  t[2] = 2;
  g.tris().push_back(t);
  return g;
}

phys::world_config null_config(double fixed_dt = 0.01) {
  phys::world_config c;
  c.backend = phys::backend::null;
  c.fixed_dt = fixed_dt;
  return c;
}

cvc::world_clock clock_at(double fixed_dt = 0.01) {
  cvc::world_clock::config cc;
  cc.fixed_dt = fixed_dt;
  return cvc::world_clock(cc);
}

// A kinematic box moving at v along x and spinning about z: the null backend
// integrates it exactly, and its rotation exercises the matrix's rotation part.
phys::body_id mover(phys::world &w, double x0, double v, double spin = 0.0) {
  phys::body_desc d;
  d.shape = phys::box{{0.1, 0.1, 0.1}};
  d.motion = phys::motion_type::kinematic;
  d.position = {x0, 0.0, 0.0};
  d.linear_velocity = {v, 0.0, 0.0};
  d.angular_velocity = {0.0, 0.0, spin};
  return w.add_body(d);
}

Mat expected(const phys::pose_tracker &t, phys::body_id id, double alpha) {
  Mat m{};
  const auto p = t.sample_one(id, alpha);
  if (p)
    phys::pose_matrix(*p, m.data());
  return m;
}

Mat times(const Mat &a, const Mat &b) {
  Mat c{};
  for (int r = 0; r < 4; ++r)
    for (int k = 0; k < 4; ++k)
      for (int col = 0; col < 4; ++col)
        c[r * 4 + col] += a[r * 4 + k] * b[k * 4 + col];
  return c;
}

bool equals(const vtkMatrix4x4 *a, const Mat &m) {
  if (!a)
    return false;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      if (a->GetElement(i, j) != m[i * 4 + j])
        return false;
  return true;
}

bool near(const vtkMatrix4x4 *a, const Mat &m, double tol = 1e-12) {
  if (!a)
    return false;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      if (std::fabs(a->GetElement(i, j) - m[i * 4 + j]) > tol)
        return false;
  return true;
}

// ---------------------------------------------------------------------------

void test_node_takes_the_interpolated_pose() {
  std::printf("test_node_takes_the_interpolated_pose\n");
  cvc::app app;
  SceneGraph sg(app);
  phys::world w(null_config());
  auto clock = clock_at();
  const phys::body_id a = mover(w, 0.0, 1.0, 0.7);
  const phys::body_id b = mover(w, 3.0, -1.0);
  auto na = sg.addGraphics("a", tri());
  auto nb = sg.addGraphics("b", tri());
  sg.processEvents();

  physics_scene_binding view(w);
  view.bind(a, na);
  // b's mesh is modelled with its base at z = 0 while the body's origin is its
  // centre: the offset lowers the visual by the half height.
  const Mat lower = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -0.1, 0, 0, 0, 1};
  view.bind(b, nb, lower.data());
  CHECK(view.node_count() == 2);

  view.follow(clock); // anchors
  clock.advance_ticks(5);
  CHECK(view.follow(clock) == 5);
  view.present(0.25);
  CHECK(view.last_posed() == 2);
  CHECK(equals(na->getTransform(), expected(view.tracker(), a, 0.25)));
  CHECK(near(nb->getTransform(), times(expected(view.tracker(), b, 0.25), lower)));
  // Between ticks 4 and 5: x = 0.0425 for a.
  CHECK(std::fabs(na->getTransform()->GetElement(0, 3) - 0.0425) < 1e-12);

  const Mat a_last = expected(view.tracker(), a, 0.25);
  view.unbind(a);
  CHECK(!view.bound(a));
  CHECK(!view.tracker().tracking(a)); // its only user left
  view.present(0.75);
  CHECK(view.last_posed() == 1);
  CHECK(equals(na->getTransform(), a_last)); // no longer posed: keeps its last pose

  bool threw = false;
  try {
    view.bind(phys::body_id{}, na);
  } catch (const cvc::physics_error &) {
    threw = true;
  }
  CHECK(threw);
}

void test_present_off_the_owner_thread_is_posted() {
  std::printf("test_present_off_the_owner_thread_is_posted\n");
  cvc::app app;
  SceneGraph sg(app);
  phys::world w(null_config());
  auto clock = clock_at();
  const phys::body_id a = mover(w, 1.0, 1.0);
  auto na = sg.addGraphics("a", tri());
  sg.processEvents();
  physics_scene_binding view(w);
  view.bind(a, na);
  view.follow(clock);
  clock.advance_ticks(3);
  view.follow(clock);

  std::thread t([&] { view.present(0.5); });
  t.join();
  CHECK(na->getTransform()->GetElement(0, 3) == 0.0); // not applied on the other thread
  sg.processEvents();
  CHECK(equals(na->getTransform(), expected(view.tracker(), a, 0.5)));
}

void test_groups_and_removed_bodies() {
  std::printf("test_groups_and_removed_bodies\n");
  cvc::app app;
  SceneGraph sg(app);
  phys::world w(null_config());
  auto clock = clock_at();
  std::vector<phys::body_id> ids;
  for (int i = 0; i < 4; ++i)
    ids.push_back(mover(w, 10.0 * i, 1.0));
  physics_scene_binding view(w);
  std::vector<std::vector<phys::pose_tracker::sample>> calls;
  // Reverse order, to show the sink gets the group's order, not the world's.
  const std::vector<phys::body_id> order = {ids[3], ids[1], ids[2], ids[0]};
  const auto g = view.bind_group(order, [&](const auto &p) { calls.push_back(p); });
  auto n1 = sg.addGraphics("one", tri());
  sg.processEvents();
  view.bind(ids[1], n1); // in a group and bound to a node at once
  CHECK(view.group_count() == 1);

  view.follow(clock);
  clock.advance_ticks(2);
  view.follow(clock);
  view.present(0.5);
  CHECK(calls.size() == 1);
  CHECK(calls.back().size() == 4);
  bool in_order = calls.back().size() == 4;
  for (std::size_t i = 0; in_order && i < 4; ++i)
    in_order = calls.back()[i].id == order[i];
  CHECK(in_order);
  CHECK(std::fabs(calls.back()[0].pose.position.x - 30.015) < 1e-12);

  // Remove a grouped body and the node-bound one: the sink skips the first, the
  // node is unbound and keeps its last pose.
  const Mat last = expected(view.tracker(), ids[1], 0.5);
  w.remove_body(ids[2]);
  w.remove_body(ids[1]);
  clock.advance_ticks(1);
  view.follow(clock);
  view.present(0.5);
  CHECK(calls.back().size() == 2);
  CHECK(calls.back()[0].id == ids[3] && calls.back()[1].id == ids[0]);
  CHECK(!view.bound(ids[1]));
  CHECK(view.node_count() == 0);
  CHECK(equals(n1->getTransform(), last));

  view.unbind_group(g);
  view.present(0.5);
  CHECK(calls.size() == 2); // no sink after unbind_group
  CHECK(view.tracker().size() == 0);

  bool threw = false;
  try {
    view.bind_group({ids[0], phys::body_id{}}, [](const auto &) {});
  } catch (const cvc::physics_error &) {
    threw = true;
  }
  CHECK(threw);
  CHECK(view.tracker().size() == 0); // the half-made group left nothing tracked
}

void test_destroyed_node_is_unbound() {
  std::printf("test_destroyed_node_is_unbound\n");
  cvc::app app;
  SceneGraph sg(app);
  phys::world w(null_config());
  auto clock = clock_at();
  const phys::body_id a = mover(w, 0.0, 1.0);
  const phys::body_id b = mover(w, 1.0, 1.0);
  physics_scene_binding view(w);
  {
    auto na = sg.addGraphics("a", tri());
    view.bind(a, na);
  }
  auto nb = sg.addGraphics("b", tri());
  view.bind(b, nb);
  sg.processEvents();
  sg.removeGraphics("a");
  sg.processEvents();
  view.follow(clock);
  clock.advance_ticks(1);
  view.follow(clock);
  view.present(0.5);
  CHECK(!view.bound(a));
  CHECK(view.bound(b));
  CHECK(view.last_posed() == 1);
  CHECK(equals(nb->getTransform(), expected(view.tracker(), b, 0.5)));
}

void test_clock_alpha_is_the_worlds_fraction() {
  std::printf("test_clock_alpha_is_the_worlds_fraction\n");
  cvc::app app;
  SceneGraph sg(app);
  phys::world w(null_config(0.04)); // a step every four quanta
  auto clock = clock_at(0.01);
  const phys::body_id a = mover(w, 0.0, 1.0);
  auto na = sg.addGraphics("a", tri());
  sg.processEvents();
  physics_scene_binding view(w);
  view.bind(a, na);
  view.follow(clock);
  clock.advance_ticks(10); // steps at ticks 4 and 8; tick 10 is halfway to 12
  CHECK(view.follow(clock) == 2);
  view.present(clock, 0.0);
  // Halfway between x = 0.04 and x = 0.08.
  CHECK(std::fabs(na->getTransform()->GetElement(0, 3) - 0.06) < 1e-12);
}

void test_jolt_crate_falls_in_the_scene() {
  std::printf("test_jolt_crate_falls_in_the_scene\n");
  if (!phys::backend_available(phys::backend::jolt)) {
    std::printf("  SKIP: this libcvc was built without Jolt\n");
    return;
  }
  cvc::app app;
  SceneGraph sg(app);
  phys::world_config c;
  c.backend = phys::backend::jolt;
  phys::world w(c);
  auto clock = clock_at();
  phys::body_desc ground;
  ground.shape = phys::box{{5.0, 5.0, 0.5}};
  ground.motion = phys::motion_type::static_;
  ground.position = {0.0, 0.0, -0.5};
  w.add_body(ground);
  phys::body_desc crate;
  crate.shape = phys::box{{0.2, 0.2, 0.2}};
  crate.position = {0.0, 0.0, 2.0};
  const phys::body_id id = w.add_body(crate);
  auto node = sg.addGraphics("crate", tri());
  sg.processEvents();
  physics_scene_binding view(w);
  view.bind(id, node);
  view.follow(clock);
  double last_z = 1e9;
  bool fell = true;
  for (int f = 0; f < 40; ++f) {
    clock.advance_ticks(2);
    view.follow(clock);
    view.present(0.5);
    const double z = node->getTransform()->GetElement(2, 3);
    if (f < 20 && !(z < last_z))
      fell = false;
    last_z = z;
  }
  CHECK(fell);
  // After 0.8 s from 2 m the crate has landed: resting on the ground at its half height.
  clock.advance_ticks(200);
  view.follow(clock);
  view.present(0.0);
  CHECK(std::fabs(node->getTransform()->GetElement(2, 3) - 0.2) < 0.02);
  CHECK(std::fabs(node->getTransform()->GetElement(2, 3) - w.state(id).position.z) < 1e-3);
}

} // namespace

int main() {
  test_node_takes_the_interpolated_pose();
  test_present_off_the_owner_thread_is_posted();
  test_groups_and_removed_bodies();
  test_destroyed_node_is_unbound();
  test_clock_alpha_is_the_worlds_fraction();
  test_jolt_crate_falls_in_the_scene();

  if (g_failures) {
    std::printf("cvcgl_physics_scene_binding: %d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("cvcgl_physics_scene_binding: OK\n");
  return 0;
}
