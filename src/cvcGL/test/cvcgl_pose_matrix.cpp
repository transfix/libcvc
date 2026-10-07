// GraphicsNode::setPoseMatrix -- the per-frame pose path for many moving nodes.
//
// setTransform(const double[16]) is the general-purpose matrix setter, and it is
// expensive per call in ways that only show up when a scene moves dozens of
// nodes every frame:
//   * it formats the matrix to 17 digits and writes the state tree inline;
//   * the "matrix" handler re-parses that echo and runs the whole transform
//     cascade a SECOND time;
//   * each of those two cascades fires transformChanged, and the scene used to
//     answer every one with its own world-bounds walk over all graphics.
// So N moving vehicles cost 2N bounds walks per frame on the render thread.
//
// setPoseMatrix applies the matrix on the scene's owner thread with no string
// round trip, publishes it through the scene's state_publisher (formatted on
// the flushing thread, coalesced), and the scene folds every move between two
// pumps into ONE bounds walk in processEvents(). These checks pin that:
//   1. the node's local, world and actor matrices equal the input EXACTLY;
//   2. 100 poses between two pumps cost exactly one bounds walk, and the grid
//      still grows to enclose a node moved out of it;
//   3. a call from another thread is marshalled to the owner thread (never
//      applied where it was made), latest-wins, and an owner-thread call
//      supersedes a parked one;
//   4. the pose reaches the state tree without being re-applied when it comes
//      back, and an external write still moves the node;
//   5. the visible result is identical to setTransform for the same matrix --
//      structurally, through a parent, and in rendered pixels (that last part
//      SKIPs when the build cannot rasterise);
//   6. state ends on the node's current matrix when setPoseMatrix is mixed
//      with setTransform, resetTransform or an external write, when a flush is
//      already writing the older pose, and when the publisher shed a pose of
//      a node that then stays still;
//   7. the echo guard drops only this scene's publisher's writes, so a mirror
//      scene bound to the same paths follows published poses;
//   8. the bounds walk is coalesced in every case it claims: an off-thread
//      pose needs one pump, N adds cost one walk, an add plus a move in one
//      frame cost one walk, and a move after a consumed render request is
//      still drawn.
//
// NOT assert(): built Release, where NDEBUG makes assert() a no-op.
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/GridNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/state_publisher.h>
#include <cvc/state/state.h>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <vtkLinearTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkProp3D.h>
#include <vtkTransform.h>

using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

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

cvc::geometry tri(double s = 5.0) {
  cvc::geometry g;
  const double xyz[3][3] = {{-s, -s, 0.0}, {s, -s, 0.0}, {0.0, 1.2 * s, 0.0}};
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

// A rigid pose with awkward doubles in every slot (no exact binary fractions in
// the rotation), so "equal" below really means bit-for-bit, not "close".
Mat pose(double zDeg, double xDeg, double tx, double ty, double tz) {
  auto t = vtkSmartPointer<vtkTransform>::New();
  t->PostMultiply();
  t->RotateX(xDeg);
  t->RotateZ(zDeg);
  t->Translate(tx, ty, tz);
  Mat m;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      m[i * 4 + j] = t->GetMatrix()->GetElement(i, j);
  return m;
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

bool equals(const vtkMatrix4x4 *a, const vtkMatrix4x4 *b) {
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      if (a->GetElement(i, j) != b->GetElement(i, j))
        return false;
  return true;
}

vtkProp3D *actorOf(const std::shared_ptr<GraphicsNode> &n) {
  return vtkProp3D::SafeDownCast(n->prop());
}

// The matrix the actor actually draws with: its user transform (the node's world
// matrix, handed over by applyTransformToVTK).
vtkMatrix4x4 *actorUserMatrix(const std::shared_ptr<GraphicsNode> &n) {
  vtkProp3D *a = actorOf(n);
  return (a && a->GetUserTransform()) ? a->GetUserTransform()->GetMatrix() : nullptr;
}

Mat parseMatrix(const std::string &s) {
  Mat m{};
  std::istringstream iss(s);
  char comma;
  for (int i = 0; i < 16; ++i) {
    if (i > 0)
      iss >> comma;
    iss >> m[i];
  }
  return m;
}

std::string stateMatrix(cvc::app &app, const std::shared_ptr<GraphicsNode> &n) {
  return cvc::state::instance(app)(n->getState("matrix").fullName()).value();
}

void offOwnerThread(const std::function<void()> &f) {
  std::thread t(f);
  t.join();
}

bool encloses(const cvc::bounding_box &outer, const cvc::bounding_box &inner) {
  for (int i = 0; i < 3; ++i)
    if (outer[i] > inner[i] || outer[i + 3] < inner[i + 3])
      return false;
  return true;
}

// ---------------------------------------------------------------------------

void test_matrices_equal_the_input_exactly() {
  std::printf("test_matrices_equal_the_input_exactly\n");
  cvc::app app;
  SceneGraph sg(app);
  auto n = sg.addGraphics("v", tri());
  sg.processEvents();

  const Mat m = pose(37.0, 12.0, 1234.5678, -98.7654321, 0.3);
  n->setPoseMatrix(m.data());

  CHECK(equals(n->getTransform(), m));
  CHECK(equals(n->getWorldTransform(), m));
  CHECK(equals(actorUserMatrix(n), m));
  // And the full matrix VTK composes for drawing (position/orientation/scale
  // are untouched, so it is the user transform with nothing folded in).
  vtkProp3D *a = actorOf(n);
  CHECK(a && equals(a->GetMatrix(), m));
}

void test_hundred_poses_cost_one_bounds_walk() {
  std::printf("test_hundred_poses_cost_one_bounds_walk\n");
  cvc::app app;
  SceneGraph sg(app);
  std::vector<std::shared_ptr<GraphicsNode>> nodes;
  for (int i = 0; i < 100; ++i)
    nodes.push_back(sg.addGraphics("v" + std::to_string(i), tri(1.0)));
  sg.processEvents(); // registration's own (authoritative) walk lands here

  const std::uint64_t w0 = sg.boundsWalkCount();
  for (int i = 0; i < 100; ++i) {
    // The last node leaves the current world box, so the grid has to grow.
    const double x = (i == 99) ? 500.0 : 0.25 * i;
    const Mat m = pose(3.0 * i, 0.0, x, -0.5 * i, 0.0);
    nodes[i]->setPoseMatrix(m.data());
  }
  CHECK(sg.boundsWalkCount() == w0); // nothing walked inline, per call
  sg.processEvents();
  const std::uint64_t walks = sg.boundsWalkCount() - w0;
  std::printf("  100 poses -> %llu bounds walk(s)\n", static_cast<unsigned long long>(walks));
  CHECK(walks == 1);

  // Nothing moved since: the next pump walks nothing.
  sg.processEvents();
  CHECK(sg.boundsWalkCount() - w0 == 1);

  // Coalescing must not cost correctness: the grid followed the node that left.
  const cvc::bounding_box grid = sg.getGridNode()->bounds();
  CHECK(grid[3] >= 500.0);
  CHECK(encloses(grid, sg.computeGraphicsBounds()));
}

// The coalescing is the scene's, not setPoseMatrix's: the general setters fold
// into the same single walk (setTransform used to post two per call).
void test_set_transform_moves_are_coalesced_too() {
  std::printf("test_set_transform_moves_are_coalesced_too\n");
  cvc::app app;
  SceneGraph sg(app);
  std::vector<std::shared_ptr<GraphicsNode>> nodes;
  for (int i = 0; i < 100; ++i)
    nodes.push_back(sg.addGraphics("v" + std::to_string(i), tri(1.0)));
  sg.processEvents();

  const std::uint64_t w0 = sg.boundsWalkCount();
  for (int i = 0; i < 100; ++i) {
    const Mat m = pose(1.0 * i, 0.0, 0.1 * i, 0.0, 0.0);
    nodes[i]->setTransform(m.data());
    nodes[i]->setPosition(0.1 * i, 1.0, 0.0);
  }
  sg.processEvents();
  CHECK(sg.boundsWalkCount() - w0 == 1);
}

void test_off_thread_call_is_marshalled_latest_wins() {
  std::printf("test_off_thread_call_is_marshalled_latest_wins\n");
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop(); // no flush thread racing the counts below
  auto n = sg.addGraphics("v", tri());
  sg.processEvents();

  int fired = 0;
  std::thread::id applyThread;
  boost::signals2::scoped_connection c = n->transformChanged.connect([&](GraphicsNode *) {
    ++fired;
    applyThread = std::this_thread::get_id();
  });

  std::vector<Mat> poses;
  for (int k = 1; k <= 50; ++k)
    poses.push_back(pose(2.0 * k, 0.5 * k, 10.0 + k, -k, 0.25 * k));

  offOwnerThread([&] {
    for (const Mat &m : poses)
      n->setPoseMatrix(m.data());
  });

  // Made on another thread, so NOT applied there: the node, its actor and the
  // world matrix are all untouched until the owner pumps.
  vtkSmartPointer<vtkMatrix4x4> identity = vtkSmartPointer<vtkMatrix4x4>::New();
  CHECK(equals(n->getTransform(), identity));
  CHECK(equals(actorUserMatrix(n), identity));
  CHECK(fired == 0);

  sg.processEvents();

  // Latest wins: 50 calls parked one pose, applied once, on the owner thread.
  CHECK(equals(n->getTransform(), poses.back()));
  CHECK(equals(actorUserMatrix(n), poses.back()));
  CHECK(fired == 1);
  CHECK(applyThread == std::this_thread::get_id());

  // A further pump has nothing left to apply.
  sg.processEvents();
  CHECK(fired == 1);
}

// Concurrent callers on different threads are unordered, but one thread's
// calls are not: a pose parked by a worker and then superseded by the owner
// must not land on top of the owner's newer pose at the next pump.
void test_owner_call_supersedes_a_parked_pose() {
  std::printf("test_owner_call_supersedes_a_parked_pose\n");
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();
  auto n = sg.addGraphics("v", tri());
  sg.processEvents();

  const Mat stale = pose(10.0, 0.0, 1.0, 2.0, 3.0);
  const Mat fresh = pose(-20.0, 5.0, -4.0, 5.0, -6.0);
  offOwnerThread([&] { n->setPoseMatrix(stale.data()); });
  n->setPoseMatrix(fresh.data()); // owner thread: applied now
  CHECK(equals(n->getTransform(), fresh));
  sg.processEvents();
  CHECK(equals(n->getTransform(), fresh));
}

void test_pose_reaches_state_without_reapplying() {
  std::printf("test_pose_reaches_state_without_reapplying\n");
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop(); // every flush below is driven by hand
  auto n = sg.addGraphics("v", tri());
  sg.processEvents();

  int fired = 0;
  boost::signals2::scoped_connection c =
      n->transformChanged.connect([&fired](GraphicsNode *) { ++fired; });

  const Mat m = pose(33.0, 7.0, 0.1, 0.2, 0.3);
  n->setPoseMatrix(m.data());
  CHECK(fired == 1);
  CHECK(sg.publisher().pending() == 1); // published, not written inline

  // Flushed from off the owner thread (the worker's shape) and from the owner
  // thread (a host draining by hand): either way the echo is the node's own and
  // must not run the cascade again.
  offOwnerThread([&] { sg.publisher().flush(); });
  sg.processEvents();
  CHECK(fired == 1);
  CHECK(parseMatrix(stateMatrix(app, n)) == m); // exact round trip through the tree

  const Mat m2 = pose(-3.0, 1.0, 9.0, 8.0, 7.0);
  n->setPoseMatrix(m2.data());
  sg.publisher().flush();
  sg.processEvents();
  CHECK(fired == 2);
  CHECK(parseMatrix(stateMatrix(app, n)) == m2);
  CHECK(equals(n->getTransform(), m2));

  // A write from outside the node is not an echo and still poses it.
  const std::string path = n->getState("matrix").fullName();
  offOwnerThread([&] {
    cvc::state::instance(app)(path).value(std::string("1,0,0,7,0,1,0,8,0,0,1,9,0,0,0,1"));
  });
  sg.processEvents();
  CHECK(n->getTransform()->GetElement(0, 3) == 7.0);
  CHECK(n->getTransform()->GetElement(1, 3) == 8.0);
  CHECK(n->getTransform()->GetElement(2, 3) == 9.0);
}

void test_identical_pose_is_a_no_op() {
  std::printf("test_identical_pose_is_a_no_op\n");
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();
  auto n = sg.addGraphics("v", tri());
  sg.processEvents();

  const Mat m = pose(15.0, 0.0, 4.0, 5.0, 6.0);
  n->setPoseMatrix(m.data());
  sg.processEvents();
  sg.publisher().flush();

  int fired = 0;
  boost::signals2::scoped_connection c =
      n->transformChanged.connect([&fired](GraphicsNode *) { ++fired; });
  const vtkMTimeType actorTime = actorOf(n)->GetMTime();
  const vtkMTimeType userTime = actorOf(n)->GetUserTransform()->GetMTime();
  const std::uint64_t w0 = sg.boundsWalkCount();

  // A parked vehicle re-sent every frame: no cascade, no actor Modified (which
  // would make the shadow baker re-bake), no publish, no bounds walk.
  for (int k = 0; k < 10; ++k)
    n->setPoseMatrix(m.data());
  sg.processEvents();
  CHECK(fired == 0);
  CHECK(actorOf(n)->GetMTime() == actorTime);
  CHECK(actorOf(n)->GetUserTransform()->GetMTime() == userTime);
  CHECK(sg.publisher().pending() == 0);
  CHECK(sg.boundsWalkCount() == w0);
}

// A node with no scene has no owner thread and no publisher: the pose applies
// where it is made and is written to state directly -- once, not echoed back
// into a second cascade.
void test_detached_node() {
  std::printf("test_detached_node\n");
  cvc::app app;
  auto n = std::make_shared<GeometryNode>(app, "posetest.detached", "detached");
  int fired = 0;
  boost::signals2::scoped_connection c =
      n->transformChanged.connect([&fired](GraphicsNode *) { ++fired; });
  const Mat m = pose(-71.0, 3.0, -1.5, 2.5, -3.5);
  n->setPoseMatrix(m.data());
  CHECK(equals(n->getTransform(), m));
  CHECK(fired == 1);
  CHECK(parseMatrix(cvc::state::instance(app)(n->getState("matrix").fullName()).value()) == m);
}

// A pose waiting in the publisher must not be written over a newer matrix that
// setTransform or resetTransform put on the node directly. If it were, the node
// would show the newer matrix while state kept the older one. A node that then
// stays still would never correct that, because re-sending its current pose is
// a no-op.
void test_set_transform_supersedes_a_queued_pose() {
  std::printf("test_set_transform_supersedes_a_queued_pose\n");
  cvc::app app;
  SceneGraph sg(app);
  auto &pub = sg.publisher();
  pub.stop();
  auto n = sg.addGraphics("v", tri());
  sg.processEvents();
  pub.flush();

  const Mat a = pose(25.0, 5.0, 10.0, 0.5, -1.0);
  const Mat b = pose(-40.0, 0.0, 20.0, -0.75, 2.0);

  // A node that never used setPoseMatrix keeps setTransform exactly as it was:
  // a direct write, nothing queued.
  n->setTransform(b.data());
  CHECK(pub.pending() == 0);
  CHECK(parseMatrix(stateMatrix(app, n)) == b);

  n->setPoseMatrix(a.data()); // queued
  n->setTransform(b.data());  // newer, written directly
  pub.flush();
  sg.processEvents();
  CHECK(equals(n->getTransform(), b));
  CHECK(parseMatrix(stateMatrix(app, n)) == b);

  // The vtkMatrix4x4 overload.
  n->setPoseMatrix(a.data());
  auto vb = vtkSmartPointer<vtkMatrix4x4>::New();
  vb->DeepCopy(b.data());
  n->setTransform(vb);
  pub.flush();
  sg.processEvents();
  CHECK(equals(n->getTransform(), b));
  CHECK(parseMatrix(stateMatrix(app, n)) == b);

  // resetTransform.
  n->setPoseMatrix(a.data());
  n->resetTransform();
  pub.flush();
  sg.processEvents();
  vtkSmartPointer<vtkMatrix4x4> identity = vtkSmartPointer<vtkMatrix4x4>::New();
  CHECK(equals(n->getTransform(), identity));
  CHECK(stateMatrix(app, n) == "1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1");
}

// The harder ordering. A flush has already taken the queued pose but has not
// written it yet, and setTransform runs in that gap. This is a worker flush
// racing the render thread. Removing the queued entry cannot fix it, because
// the flush already holds it. Queueing the newer matrix fixes it: the next
// flush writes it after the stale one.
//
// The gap is reached deterministically. Listeners on pad paths published in
// the same batch run inside the flush, and the first one to fire calls
// setTransform. Whether that happens before or after the pose is written
// depends on hash order, so the test reports which case ran. It checks the
// result in both cases.
void test_pose_being_flushed_is_superseded() {
  std::printf("test_pose_being_flushed_is_superseded\n");
  cvc::app app;
  SceneGraph sg(app);
  auto &pub = sg.publisher();
  pub.stop();
  auto n = sg.addGraphics("v", tri());
  sg.processEvents();
  pub.flush();

  const Mat a = pose(25.0, 5.0, 10.0, 0.5, -1.0);
  const Mat b = pose(-40.0, 0.0, 20.0, -0.75, 2.0);
  const std::string path = n->getState("matrix").fullName();
  const std::string textA = cvc::gl::state_publisher::format_matrix(a.data());

  bool fired = false, beforeA = false;
  std::vector<std::string> pads;
  std::vector<boost::signals2::scoped_connection> conns;
  for (int k = 0; k < 32; ++k) {
    pads.push_back("posetest.flushgap.pad" + std::to_string(k));
    cvc::state::instance(app)(pads.back()).value(std::string("0"));
    conns.emplace_back(cvc::state::instance(app)(pads.back()).valueChanged.connect([&]() {
      if (fired)
        return;
      fired = true;
      beforeA = cvc::state::instance(app)(path).value() != textA;
      n->setTransform(b.data());
    }));
  }

  n->setPoseMatrix(a.data());
  for (const std::string &p : pads)
    pub.publish(p, "1");
  pub.flush();
  pub.flush(); // what was queued during the first flush
  sg.processEvents();
  std::printf("  setTransform ran %s the queued pose was written\n", beforeA ? "BEFORE" : "after");
  CHECK(fired);
  CHECK(equals(n->getTransform(), b));
  CHECK(parseMatrix(stateMatrix(app, n)) == b);
}

// The same applies to a write from outside the node. A script that moves a node
// while one of the node's poses is queued must not be undone by the next flush.
void test_external_write_supersedes_a_queued_pose() {
  std::printf("test_external_write_supersedes_a_queued_pose\n");
  cvc::app app;
  SceneGraph sg(app);
  auto &pub = sg.publisher();
  pub.stop();
  auto n = sg.addGraphics("v", tri());
  sg.processEvents();

  const Mat a = pose(25.0, 5.0, 10.0, 0.5, -1.0);
  n->setPoseMatrix(a.data()); // queued
  const std::string path = n->getState("matrix").fullName();
  offOwnerThread([&] {
    cvc::state::instance(app)(path).value(std::string("1,0,0,7,0,1,0,8,0,0,1,9,0,0,0,1"));
  });
  sg.processEvents(); // the node follows the script
  pub.flush();        // ... and the queued pose must not overwrite it
  sg.processEvents();
  CHECK(n->getTransform()->GetElement(0, 3) == 7.0);
  CHECK(n->getTransform()->GetElement(1, 3) == 8.0);
  CHECK(n->getTransform()->GetElement(2, 3) == 9.0);
  Mat cur;
  std::copy(n->getTransform()->GetData(), n->getTransform()->GetData() + 16, cur.begin());
  CHECK(parseMatrix(stateMatrix(app, n)) == cur);
}

// The publisher sheds values past its path cap, which is safe only because a
// moving node republishes next frame. A parked node re-sends an unchanged pose,
// which change detection skips. So once the publisher has shed anything since a
// node's last publish, the node publishes the unchanged pose again (state only,
// no cascade) instead of leaving state stale indefinitely.
void test_shed_pose_is_published_again_when_parked() {
  std::printf("test_shed_pose_is_published_again_when_parked\n");
  cvc::app app;
  SceneGraph sg(app);
  auto &pub = sg.publisher();
  pub.stop();
  auto a = sg.addGraphics("a", tri());
  auto b = sg.addGraphics("b", tri());
  sg.processEvents();
  pub.flush();

  const Mat pa = pose(12.0, 0.0, 1.0, 2.0, 3.0);
  const Mat pb = pose(-12.0, 0.0, -1.0, -2.0, -3.0);
  pub.set_max_pending(1); // room for one of the two
  a->setPoseMatrix(pa.data());
  b->setPoseMatrix(pb.data());
  CHECK(pub.dropped() == 1);
  pub.set_max_pending(8192);
  pub.flush();
  const bool aStale = parseMatrix(stateMatrix(app, a)) != pa;
  const bool bStale = parseMatrix(stateMatrix(app, b)) != pb;
  CHECK(aStale != bStale); // exactly one pose was shed

  int fired = 0;
  boost::signals2::scoped_connection ca =
      a->transformChanged.connect([&fired](GraphicsNode *) { ++fired; });
  boost::signals2::scoped_connection cb =
      b->transformChanged.connect([&fired](GraphicsNode *) { ++fired; });

  // Both vehicles are parked and re-send their poses.
  a->setPoseMatrix(pa.data());
  b->setPoseMatrix(pb.data());
  pub.flush();
  sg.processEvents();
  CHECK(fired == 0); // nothing moved
  CHECK(parseMatrix(stateMatrix(app, a)) == pa);
  CHECK(parseMatrix(stateMatrix(app, b)) == pb);

  // With nothing shed since, a parked pose is free again.
  a->setPoseMatrix(pa.data());
  b->setPoseMatrix(pb.data());
  CHECK(pub.pending() == 0);
}

// A second scene built with the same state prefix binds its nodes to the same
// paths. For example, a mirror view. The echo guard must drop only this scene's
// own publisher's writes, so the mirror follows a published pose just as it
// follows setTransform.
void test_mirror_scene_follows_published_poses() {
  std::printf("test_mirror_scene_follows_published_poses\n");
  cvc::app app;
  SceneGraph src(app, "posemirror");
  SceneGraph mir(app, "posemirror");
  src.publisher().stop();
  mir.publisher().stop();
  auto a = src.addGraphics("v", tri());
  auto b = mir.addGraphics("v", tri());
  src.processEvents();
  mir.processEvents();
  CHECK(a->getState("matrix").fullName() == b->getState("matrix").fullName());

  int aFired = 0;
  boost::signals2::scoped_connection c =
      a->transformChanged.connect([&aFired](GraphicsNode *) { ++aFired; });

  const Mat m1 = pose(30.0, 10.0, 4.0, -5.0, 6.0);
  a->setPoseMatrix(m1.data());
  src.publisher().flush();
  src.processEvents();
  mir.processEvents();
  CHECK(equals(b->getTransform(), m1)); // the mirror follows
  CHECK(aFired == 1);                   // and the source still drops its own echo

  // Flushed on another thread, as the worker does. The mirror's move is
  // marshalled to its owner thread.
  const Mat m2 = pose(-60.0, 0.0, -7.0, 8.0, -9.0);
  a->setPoseMatrix(m2.data());
  offOwnerThread([&] { src.publisher().flush(); });
  src.processEvents();
  mir.processEvents();
  CHECK(equals(b->getTransform(), m2));
  CHECK(aFired == 2);

  // The published "position" key uses the same guard.
  a->setPosition(1.5, -2.5, 3.5);
  src.publisher().flush();
  src.processEvents();
  mir.processEvents();
  CHECK(b->getTransform()->GetElement(0, 3) == 1.5);
  CHECK(b->getTransform()->GetElement(1, 3) == -2.5);
  CHECK(b->getTransform()->GetElement(2, 3) == 3.5);
  CHECK(aFired == 3);
}

// --- world-bounds coalescing -------------------------------------------------

// A pose made on another thread is applied while processEvents drains the
// queue. The walk runs after the drain, so ONE pump both applies the pose and
// grows the grid to it. A walk run before the drain would miss the pose until
// the next pump.
void test_off_thread_pose_out_of_the_grid_needs_one_pump() {
  std::printf("test_off_thread_pose_out_of_the_grid_needs_one_pump\n");
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();
  auto n = sg.addGraphics("v", tri());
  sg.processEvents();

  const std::uint64_t w0 = sg.boundsWalkCount();
  const Mat far = pose(20.0, 0.0, 800.0, -600.0, 50.0);
  offOwnerThread([&] { n->setPoseMatrix(far.data()); });
  CHECK(sg.boundsWalkCount() == w0);
  sg.processEvents();
  CHECK(sg.boundsWalkCount() - w0 == 1);
  CHECK(equals(n->getTransform(), far));
  const cvc::bounding_box grid = sg.getGridNode()->bounds();
  CHECK(grid[3] >= 800.0 && grid[1] <= -600.0);
  CHECK(encloses(grid, sg.computeGraphicsBounds()));
}

// Each add or remove asks for an authoritative recompute. One queued recompute
// covers all of them, so building a scene of N nodes costs one walk, not N.
void test_adding_nodes_costs_one_walk() {
  std::printf("test_adding_nodes_costs_one_walk\n");
  cvc::app app;
  SceneGraph sg(app);
  sg.processEvents();

  const std::uint64_t w0 = sg.boundsWalkCount();
  for (int i = 0; i < 50; ++i)
    sg.addGraphics("v" + std::to_string(i), tri(1.0 + 4.0 * i));
  CHECK(sg.boundsWalkCount() == w0); // nothing walked inline
  sg.processEvents();
  std::printf("  50 adds -> %llu bounds walk(s)\n",
              static_cast<unsigned long long>(sg.boundsWalkCount() - w0));
  CHECK(sg.boundsWalkCount() - w0 == 1);
  CHECK(encloses(sg.getGridNode()->bounds(), sg.computeGraphicsBounds()));
}

// An add and a move in the same frame. The recompute the add queued already
// sees the move, so the grow-only walk the move flagged is cleared rather than
// run again.
void test_add_and_move_in_one_frame_costs_one_walk() {
  std::printf("test_add_and_move_in_one_frame_costs_one_walk\n");
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();
  sg.processEvents();

  const std::uint64_t w0 = sg.boundsWalkCount();
  auto n = sg.addGraphics("v", tri());
  const Mat far = pose(0.0, 0.0, -900.0, 300.0, 0.0);
  n->setPoseMatrix(far.data());
  sg.processEvents();
  CHECK(sg.boundsWalkCount() - w0 == 1);
  const cvc::bounding_box grid = sg.getGridNode()->bounds();
  CHECK(grid[0] <= -900.0);
  CHECK(encloses(grid, sg.computeGraphicsBounds()));
}

// Only the first move since a pump requests a render. A host may consume that
// request (draw a frame) before the pump, and a node can move after that frame.
// The pump must then request another render, or the later move is not drawn
// until something else asks for a frame.
void test_move_after_a_consumed_render_request_is_drawn() {
  std::printf("test_move_after_a_consumed_render_request_is_drawn\n");
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();
  auto a = sg.addGraphics("a", tri());
  auto b = sg.addGraphics("b", tri());
  const Mat wideA = pose(0.0, 0.0, -20.0, 0.0, 0.0);
  const Mat wideB = pose(0.0, 0.0, 20.0, 0.0, 0.0);
  a->setPoseMatrix(wideA.data());
  b->setPoseMatrix(wideB.data());
  sg.processEvents(); // the grid now spans both, so the moves below stay inside it
  sg.checkAndResetRenderNeeded();
  const cvc::bounding_box grid0 = sg.getGridNode()->bounds();

  // Moves inside the grid: the walk will not grow it, so it will not ask for
  // a frame itself.
  const Mat pa = pose(0.0, 0.0, -15.0, 0.0, 0.0);
  const Mat pb = pose(0.0, 0.0, 15.0, 0.0, 0.0);
  a->setPoseMatrix(pa.data());
  CHECK(sg.checkAndResetRenderNeeded()); // the host draws a frame
  b->setPoseMatrix(pb.data());           // the scene is already flagged
  const std::uint64_t w0 = sg.boundsWalkCount();
  sg.processEvents();
  CHECK(sg.boundsWalkCount() - w0 == 1);
  const cvc::bounding_box grid1 = sg.getGridNode()->bounds();
  for (int i = 0; i < 6; ++i)
    CHECK(grid1[i] == grid0[i]);         // the grid did not grow ...
  CHECK(sg.checkAndResetRenderNeeded()); // ... yet b's move gets drawn too
}

// Structural parity with setTransform: same local, world, actor matrices and
// world box, including through a parent whose pose its child inherits.
void test_matches_set_transform() {
  std::printf("test_matches_set_transform\n");
  cvc::app app;
  SceneGraph sg(app);
  auto a = sg.addGraphics("a", tri());
  auto b = sg.addGraphics("b", tri());
  auto ca = a->addGraphicsChild<GeometryNode>("ca");
  auto cb = b->addGraphicsChild<GeometryNode>("cb");
  ca->setGeometry(tri(1.0));
  cb->setGeometry(tri(1.0));
  ca->setPosition(1.0, 2.0, 3.0);
  cb->setPosition(1.0, 2.0, 3.0);
  sg.processEvents();

  const Mat m = pose(123.0, -45.0, 17.25, -0.125, 3.0);
  a->setTransform(m.data());
  b->setPoseMatrix(m.data());
  sg.processEvents();

  CHECK(equals(a->getTransform(), b->getTransform()));
  CHECK(equals(a->getWorldTransform(), b->getWorldTransform()));
  CHECK(equals(actorUserMatrix(a), actorUserMatrix(b)));
  CHECK(equals(actorOf(a)->GetMatrix(), actorOf(b)->GetMatrix()));
  CHECK(equals(ca->getWorldTransform(), cb->getWorldTransform()));
  CHECK(equals(actorUserMatrix(ca), actorUserMatrix(cb)));
  const cvc::bounding_box wa = a->getCombinedWorldBoundingBox();
  const cvc::bounding_box wb = b->getCombinedWorldBoundingBox();
  for (int i = 0; i < 6; ++i)
    CHECK(wa[i] == wb[i]);

  // The child composes parent x child exactly as the cascade always has.
  auto expect = vtkSmartPointer<vtkMatrix4x4>::New();
  vtkMatrix4x4::Multiply4x4(b->getTransform(), cb->getTransform(), expect);
  CHECK(equals(cb->getWorldTransform(), expect));
}

// Pixel parity: the same matrix through either setter draws the same frame.
std::vector<unsigned char> renderPosed(bool viaPoseMatrix, bool posed) {
  cvc::app app;
  SceneGraph sg(app, viaPoseMatrix ? "posepx_b" : "posepx_a");
  sg.setDiagnosticChromeVisible(false);
  auto n = sg.addGraphics("v", tri());
  if (posed) {
    const Mat m = pose(30.0, 0.0, 3.0, -2.0, 0.0);
    if (viaPoseMatrix)
      n->setPoseMatrix(m.data());
    else
      n->setTransform(m.data());
  }
  SceneRenderer sr(sg, 64, 64, /*offscreen=*/true);
  sr.setCamera(0, 0, 60, 0, 0, 0, 0, 1, 0, 40, 1, 200);
  sr.render();
  return sr.frameRGB();
}

void test_render_matches_set_transform() {
  std::printf("test_render_matches_set_transform\n");
  const std::vector<unsigned char> viaSet = renderPosed(false, true);
  if (viaSet.empty()) {
    std::printf("  SKIP: no rasteriser in this build/environment\n");
    return;
  }
  const std::vector<unsigned char> viaPose = renderPosed(true, true);
  const std::vector<unsigned char> unposed = renderPosed(true, false);
  CHECK(viaPose == viaSet);
  CHECK(viaPose != unposed); // the pose is actually visible, not a blank frame twice
}

} // namespace

int main() {
  test_matrices_equal_the_input_exactly();
  test_hundred_poses_cost_one_bounds_walk();
  test_set_transform_moves_are_coalesced_too();
  test_off_thread_call_is_marshalled_latest_wins();
  test_owner_call_supersedes_a_parked_pose();
  test_pose_reaches_state_without_reapplying();
  test_identical_pose_is_a_no_op();
  test_detached_node();
  test_set_transform_supersedes_a_queued_pose();
  test_pose_being_flushed_is_superseded();
  test_external_write_supersedes_a_queued_pose();
  test_shed_pose_is_published_again_when_parked();
  test_mirror_scene_follows_published_poses();
  test_off_thread_pose_out_of_the_grid_needs_one_pump();
  test_adding_nodes_costs_one_walk();
  test_add_and_move_in_one_frame_costs_one_walk();
  test_move_after_a_consumed_render_request_is_drawn();
  test_matches_set_transform();
  test_render_matches_set_transform();

  if (g_failures) {
    std::printf("cvcgl_pose_matrix: %d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("cvcgl_pose_matrix: OK\n");
  return 0;
}
