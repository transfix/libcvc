// A node's own pose, arriving back late from the publisher, must not rewind it.
//
// setPosition() applies the pose to m_transform immediately and PUBLISHES the
// string form through the scene's state_publisher, which coalesces and flushes
// on its own thread. That flush writes the state tree, which fires the node's
// handleStateChanged("position") — on the flusher's thread, so the work is
// marshalled onto the scene's owner thread and runs later, whenever someone
// pumps processEvents().
//
// The echo guard (posStr == m_echoPosition) was evaluated THERE, at drain time,
// against the node's LATEST published value. That is a single slot and the
// callback outlived it. Pose a node twice with a flush landing in between and
// the drain compares the FIRST pose — still what the tree holds, because the
// second publish has not flushed yet — against the SECOND pose, now the echo.
// Mismatch. So it concludes somebody else moved the node and writes the OLD
// pose back over the new one, dragging the whole subtree with it.
//
// bindings/pycvc/test_pycvc_gl.py::test_group_nodes_compose_transforms caught
// this about one run in six: move a group twice and its children snapped back
// to the group's PREVIOUS position.
//
// Reproduced below with no timing whatsoever. The publisher's worker is stopped
// and flush() is driven by hand from a non-owner thread, which is the only
// thing the worker ever contributed here: a state write arriving from off the
// owner thread, so the echo check is deferred instead of running inline.
//
// These tests assert(), and cvcpkg builds them Release -- where NDEBUG makes
// assert() expand to nothing and every check below would pass vacuously.
// Undefine it before <cassert> so the assertions actually run.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/state_publisher.h>
#include <functional>
#include <memory>
#include <string>
#include <thread>

using cvc::gl::GraphicsNode;
using cvc::gl::SceneGraph;

namespace {

cvc::geometry dot() {
  cvc::geometry g;
  cvc::geometry::point_t p;
  p[0] = p[1] = p[2] = 0;
  g.points().push_back(p);
  return g;
}

double worldX(const std::shared_ptr<GraphicsNode> &n) {
  return n->getWorldTransform()->GetElement(0, 3);
}

// Run `f` on a thread that is NOT the scene's owner, and wait for it. This is
// what makes the echo check deferred: SceneNode::runOnMainThread only queues
// when the state write reaches the node from off the owner thread.
void offOwnerThread(const std::function<void()> &f) {
  std::thread t(f);
  t.join();
}

// The regression. Two poses, with a flush of the first landing in between; the
// second pose must survive the drain.
void test_a_stale_flush_does_not_rewind_the_node() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop(); // no cadence: this test drives every flush by hand

  auto group = sg.addGraphics("group");
  auto leaf = group->createChild<cvc::gl::GeometryNode>("leaf", dot());

  group->setPosition(10.0, 0.0, 0.0);
  leaf->setPosition(0.0, 0.0, 2.0);
  // setPosition composes the subtree synchronously; no pump needed to see it.
  assert(worldX(leaf) == 10.0 && "the first pose must compose down to the leaf");

  // The flush lands while that first pose is still the newest thing the node
  // published, and queues a callback for the owner thread. Nothing drains it
  // yet -- that gap is the whole bug, and in the Python failure it is simply
  // wherever the 30 Hz worker happened to fall between two pumps.
  offOwnerThread([&] { sg.publisher().flush(); }); // tree now holds "10,0,0"

  // Move the group again. m_transform is -4 from this line on; the publish is
  // queued and the tree still says "10,0,0".
  group->setPosition(-4.0, 0.0, 0.0);
  assert(worldX(group) == -4.0);

  // NOW drain the callback the first flush queued. It carries the node's own
  // older pose, and used to be applied as if a script had written it.
  sg.processEvents();

  assert(worldX(group) == -4.0 && "a node's own stale pose must not rewind it");
  assert(worldX(leaf) == -4.0 && "and must not drag the subtree back either");
  std::printf("  ok: a late flush of an older pose does not rewind the node\n");
}

// The same shape one step further along: the stale callback is drained after
// the newer value has ALSO reached the tree. This one always worked (the drain
// re-reads the key, so it sees the new value and matches the echo), and is here
// so the fix is not free to break it.
void test_drain_after_both_flushes_is_a_no_op() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();

  auto n = sg.addGraphics("both", dot());
  n->setPosition(1.0, 0.0, 0.0);
  offOwnerThread([&] { sg.publisher().flush(); });
  n->setPosition(2.0, 0.0, 0.0);
  offOwnerThread([&] { sg.publisher().flush(); });
  sg.processEvents();

  assert(worldX(n) == 2.0);
  std::printf("  ok: draining both flushes leaves the node on the newer pose\n");
}

// The guard must only swallow the node's OWN echo. A write someone else makes
// to the position key still has to move the node — that is the scripting
// surface, and silently ignoring it would be a worse bug than the one above.
// Written from off the owner thread so it takes the same deferred path.
void test_an_external_write_still_moves_the_node() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();

  auto n = sg.addGraphics("ext", dot());
  n->setPosition(1.0, 0.0, 0.0);
  offOwnerThread([&] { sg.publisher().flush(); });
  sg.processEvents();
  assert(worldX(n) == 1.0);

  const std::string path = n->getState("position").fullName();
  offOwnerThread([&] { cvc::state::instance(app)(path).value(std::string("7,0,0")); });
  sg.processEvents();

  assert(worldX(n) == 7.0 && "a state write from outside the node must still pose it");
  std::printf("  ok: an external position write still moves the node\n");
}

// An external write landing while the node has a publish in flight must not be
// mistaken for the node's own echo either — the tree disagrees with the node in
// both directions here, and only the publisher's own writes may be dropped.
void test_external_write_while_a_publish_is_queued() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();

  auto n = sg.addGraphics("ext2", dot());
  n->setPosition(3.0, 0.0, 0.0); // queued, not flushed
  const std::string path = n->getState("position").fullName();
  offOwnerThread([&] { cvc::state::instance(app)(path).value(std::string("9,0,0")); });
  sg.processEvents();

  assert(worldX(n) == 9.0 && "an outside write is not the node's echo, queued publish or not");
  std::printf("  ok: an external write is honoured while a publish is still queued\n");
}

} // namespace

int main() {
  test_a_stale_flush_does_not_rewind_the_node();
  test_drain_after_both_flushes_is_a_no_op();
  test_an_external_write_still_moves_the_node();
  test_external_write_while_a_publish_is_queued();
  std::printf("cvcGL pose echo: OK\n");
  return 0;
}
