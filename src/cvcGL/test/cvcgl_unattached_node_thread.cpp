// A node with no scene must not run its handlers on a thread other than its own.
//
// SceneNode::runOnMainThread() marshals off-owner-thread work through the
// scene's event queue, but a node with no scene has no queue, so it ran the work
// inline on whatever thread called. That is the right thing for the thread that
// is building the node, and the wrong thing for anyone else. The state
// publisher's worker is someone else: it flushes pose writes on its own thread,
// and SceneNode disables instance threading, so the handlers run there.
//
// The window is real. GraphicsNode::addGraphicsChild() constructs the node --
// which connects it to its state path -- before it attaches it to the scene. If
// a pose for that path is still queued in the publisher (an earlier node of the
// same name, posed and then removed: StageLighting::setWash rebuilding its wash
// lights does exactly this), the worker's flush lands in the new node's handler
// in that window, and the handler ran on the worker thread while the owner was
// still inside addLight(). cvcgl_stage_caster_truth segfaulted on it about one
// run in twenty with NVIDIA's swap throttle off, with GraphicsNode::
// updateOwnClipWorld() on the publisher's thread and BBoxNode::
// createCoordinateLabels() on the main one.
//
// What is asserted here needs no race window: off-thread work for a node with
// no scene must not reach the node until its own thread (or the scene owner
// that attaches it) runs it.
//
// These tests assert(), and cvcpkg builds them Release -- where NDEBUG makes
// assert() expand to nothing and every check below would pass vacuously.
// Undefine it before <cassert> so the assertions actually run.
#undef NDEBUG
#include <boost/signals2.hpp>
#include <cassert>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/LightNode.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/state_publisher.h>
#include <cvc/state/state.h>
#include <functional>
#include <memory>
#include <string>
#include <thread>

using cvc::gl::GraphicsNode;
using cvc::gl::LightNode;
using cvc::gl::SceneGraph;

namespace {

// Run `f` on a thread that is not the node's, and wait for it.
void offThread(const std::function<void()> &f) {
  std::thread t(f);
  t.join();
}

void attach(SceneGraph &sg, const std::shared_ptr<GraphicsNode> &node) {
  sg.getGraphicsRoot()->addGraphicsChild(node);
}

// An unattached node: a write from another thread waits for the attach, and the
// attach (on the scene's owner thread) applies it.
void test_off_thread_write_waits_for_the_attach() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop(); // this test drives every flush by hand

  auto lamp = std::make_shared<LightNode>(app, "unattached.lamp", "lamp");
  lamp->setIntensity(1.0);
  assert(lamp->intensity() == 1.0);

  const std::string path = lamp->getState("intensity").fullName();
  offThread([&] { cvc::state::instance(app)(path).value(2.5); });
  assert(lamp->intensity() == 1.0 &&
         "an unattached node must not apply state on the writer's thread");

  attach(sg, lamp);
  assert(lamp->intensity() == 2.5 && "the owner thread applies it when it attaches the node");
  std::printf("  ok: an off-thread write to an unattached node waits for the attach\n");
}

// The crash path. A pose queued in the publisher for a path whose node has been
// removed, then flushed on the publisher's thread while a new node on the same
// path is constructed but not yet attached.
void test_a_stale_flush_does_not_run_on_the_flusher() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();

  auto first = sg.addLight("wash");
  first->setPosition(1.0, 2.0, 3.0); // attached: queued in the publisher, not yet written
  const std::string path = first->getState().fullName();
  sg.removeGraphics("wash");
  first.reset();

  // addLight()'s window, held open: constructed (so connected to state), no scene.
  auto again = std::make_shared<LightNode>(app, path, "wash");
  int fired = 0;
  std::thread::id appliedOn;
  boost::signals2::scoped_connection c = again->transformChanged.connect([&](GraphicsNode *) {
    ++fired;
    appliedOn = std::this_thread::get_id();
  });

  offThread([&] { sg.publisher().flush(); });
  assert(fired == 0 && "the stale pose must not be applied on the publisher's thread");

  attach(sg, again);
  assert(fired > 0 && appliedOn == std::this_thread::get_id() &&
         "the attach applies it, on the owner thread");
  std::printf("  ok: a stale publisher flush waits for the new node's attach\n");
}

// A node that is never attached is drained by its own thread: the next work it
// does itself runs what arrived from elsewhere first, then its own.
void test_the_home_thread_drains_in_order() {
  cvc::app app;
  auto lamp = std::make_shared<LightNode>(app, "homedrain.lamp", "lamp");
  lamp->setIntensity(1.0);

  const std::string intensity = lamp->getState("intensity").fullName();
  offThread([&] { cvc::state::instance(app)(intensity).value(2.5); });
  assert(lamp->intensity() == 1.0);

  // Any write on the node's own thread runs inline, after what was held.
  cvc::state::instance(app)(lamp->getState("color_r").fullName()).value(0.25);
  assert(lamp->intensity() == 2.5 && "the held write is applied by the node's own thread");
  std::printf("  ok: the node's own thread drains held work before its own\n");
}

// The single-threaded path is untouched: on the node's own thread, with no
// scene, a write still applies inline.
void test_own_thread_write_still_applies_inline() {
  cvc::app app;
  auto lamp = std::make_shared<LightNode>(app, "inline.lamp", "lamp");
  cvc::state::instance(app)(lamp->getState("intensity").fullName()).value(4.5);
  assert(lamp->intensity() == 4.5 && "an own-thread write to an unattached node applies inline");
  std::printf("  ok: an own-thread write to an unattached node still applies inline\n");
}

// A node that outlives its scene (a script holding it) has no scene again, and
// belongs to the thread that attached it. Another thread's writes wait for it.
void test_a_node_outliving_its_scene_keeps_off_thread_writes_off_the_writer() {
  cvc::app app;
  std::shared_ptr<LightNode> lamp;
  {
    SceneGraph sg(app);
    sg.publisher().stop();
    lamp = sg.addLight("kept");
    lamp->setIntensity(1.0);
    sg.processEvents();
  }
  assert(!lamp->getSceneGraph());

  const std::string path = lamp->getState("intensity").fullName();
  offThread([&] { cvc::state::instance(app)(path).value(3.0); });
  assert(lamp->intensity() == 1.0 &&
         "a node with a dead scene must not apply state on the writer's thread");

  cvc::state::instance(app)(lamp->getState("color_r").fullName()).value(0.5);
  assert(lamp->intensity() == 3.0 && "and the thread it belongs to applies it");
  std::printf("  ok: a node outliving its scene keeps off-thread writes off the writer\n");
}

} // namespace

int main() {
  test_off_thread_write_waits_for_the_attach();
  test_a_stale_flush_does_not_run_on_the_flusher();
  test_the_home_thread_drains_in_order();
  test_own_thread_write_still_applies_inline();
  test_a_node_outliving_its_scene_keeps_off_thread_writes_off_the_writer();
  std::printf("cvcGL unattached node threading: OK\n");
  return 0;
}
