// A LightNode must only apply state on its scene's owner thread.
//
// LightNode::handleStateChanged did its own work — readAllFromState() into
// m_impl, then notifyScene() — on whatever thread wrote the state. Only the
// notifyScene() tail was marshalled. But the handler is reachable off the owner
// thread: a pose write goes through the scene's state_publisher, whose worker
// flushes on its own thread, and that flush fires childChanged; SceneNode
// disables instance threading, so the handler runs on the flusher's thread.
//
// So m_impl was being rewritten while the owner thread could be reading the
// same fields in SceneGraph::applyLights() — which drops and recreates every
// vtkLight and re-bakes a shadow map per caster. Two threads inside VTK at
// once. cvcgl_stage_caster_truth segfaulted on it roughly one run in twelve,
// always with LightNode::handleStateChanged on the publisher's thread and
// BBoxNode::createBBox() on the main one.
//
// The invariant asserted here is the one that closes it, and it is observable
// without a sanitizer or a race window: a light's state must not reach m_impl
// until the owner thread pumps. light->intensity() reads m_impl directly, so it
// says precisely when the apply happened.
//
// These tests assert(), and cvcpkg builds them Release -- where NDEBUG makes
// assert() expand to nothing and every check below would pass vacuously.
// Undefine it before <cassert> so the assertions actually run.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/LightNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/state_publisher.h>
#include <functional>
#include <string>
#include <thread>

using cvc::gl::LightNode;
using cvc::gl::SceneGraph;

namespace {

// Run `f` on a thread that is NOT the scene's owner, and wait for it.
void offOwnerThread(const std::function<void()> &f) {
  std::thread t(f);
  t.join();
}

// A state write from off the owner thread must wait for the pump.
void test_off_thread_write_waits_for_the_pump() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop(); // this test drives every flush by hand

  auto light = sg.addLight("sun");
  light->setIntensity(1.0);
  sg.processEvents();
  assert(light->intensity() == 1.0);

  const std::string path = light->getState("intensity").fullName();
  offOwnerThread([&] { cvc::state::instance(app)(path).value(2.5); });

  assert(light->intensity() == 1.0 && "light state must not reach m_impl off the owner thread");
  sg.processEvents();
  assert(light->intensity() == 2.5 && "and must reach it once the owner thread pumps");
  std::printf("  ok: an off-thread state write is applied only on the pump\n");
}

// The crash path itself: the write arrives from inside a publisher flush, on
// the publisher's thread. Publishing a light key by hand reproduces exactly
// what the worker does, with none of its timing.
void test_a_publisher_flush_does_not_apply_on_its_own_thread() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();

  auto light = sg.addLight("key");
  light->setIntensity(1.0);
  sg.processEvents();
  assert(light->intensity() == 1.0);

  sg.publisher().publish(light->getState("intensity").fullName(), "3.5");
  offOwnerThread([&] { sg.publisher().flush(); });

  assert(light->intensity() == 1.0 && "a flush must not apply light state on the flusher's thread");
  sg.processEvents();
  assert(light->intensity() == 3.5);
  std::printf("  ok: a publisher flush defers the light apply to the owner thread\n");
}

// The single-threaded path must be untouched: on the owner thread the handler
// still runs inline, with no pump needed. A host that writes light state and
// reads it straight back has to keep working.
void test_owner_thread_write_still_applies_inline() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();

  auto light = sg.addLight("fill");
  const std::string path = light->getState("intensity").fullName();
  cvc::state::instance(app)(path).value(4.5); // owner thread — no pump below

  assert(light->intensity() == 4.5 && "an owner-thread write must still apply inline");
  std::printf("  ok: an owner-thread write still applies inline, no pump needed\n");
}

// Several lights, several off-thread writes, one pump: every light must end up
// holding its own value. Guards against the deferred applies being coalesced,
// dropped, or landing on the wrong node.
void test_many_lights_each_keep_their_own_value() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();

  std::vector<std::shared_ptr<LightNode>> lights;
  for (int i = 0; i < 6; ++i)
    lights.push_back(sg.addLight("l" + std::to_string(i)));
  sg.processEvents();

  offOwnerThread([&] {
    for (int i = 0; i < 6; ++i)
      cvc::state::instance(app)(lights[i]->getState("intensity").fullName()).value(0.5 * (i + 1));
  });
  sg.processEvents();

  for (int i = 0; i < 6; ++i)
    assert(lights[i]->intensity() == 0.5 * (i + 1) && "each light keeps its own deferred value");
  std::printf("  ok: six lights each keep their own value across one pump\n");
}

} // namespace

int main() {
  test_off_thread_write_waits_for_the_pump();
  test_a_publisher_flush_does_not_apply_on_its_own_thread();
  test_owner_thread_write_still_applies_inline();
  test_many_lights_each_keep_their_own_value();
  std::printf("cvcGL light state threading: OK\n");
  return 0;
}
