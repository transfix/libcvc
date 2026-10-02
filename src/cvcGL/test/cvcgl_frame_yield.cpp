// FrameYield (cvc/gl/FrameYield.h): who yields to the browser once per frame in a
// WebAssembly build -- VTK inside every render, or the app's loop. Natively there
// is no in-render yield, so App must be a NO-OP: the window's DoubleBuffer stays
// on (native double buffering is real, and turning it off would tear) and the
// effective mode reads back Vtk. Pins that, the facade forwarding, the nested
// aliases + CVC_GL_HAS_FRAME_YIELD(_LOCK) that consumers feature-detect with, the
// one-way lock (lockFrameYield: an -sASYNCIFY_IGNORE_INDIRECT=1 app's App for
// good), and that a closed renderer refuses the calls like every other one.
//
// The wasm side (SetDoubleBuffer(0), ?frameyield=, Module.cvcglFrameYieldLocked,
// the StartEvent watchdog) needs a browser;
// docs/CVCGL_WASM.md lists its acceptance checks. Offscreen and headless: nothing
// here renders.
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/FrameYield.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ViewportManager.h>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vtkRenderWindow.h>

#ifndef CVC_GL_HAS_FRAME_YIELD
#error "cvc/gl/FrameYield.h must define CVC_GL_HAS_FRAME_YIELD"
#endif
#ifndef CVC_GL_HAS_FRAME_YIELD_LOCK
#error "cvc/gl/FrameYield.h must define CVC_GL_HAS_FRAME_YIELD_LOCK"
#endif

using cvc::gl::FrameYield;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::ViewportManager;

static int fails = 0;
static void check(bool ok, const std::string &w) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", w.c_str());
  if (!ok)
    ++fails;
}

// The detection a consumer that must build against older cvcGL too writes (the
// docs/CVCGL_WASM.md feature-detection snippet): the nested alias makes the call
// well-formed only where it exists.
template <class V> bool app_yields(V &v) {
  if constexpr (requires { v.setFrameYield(V::FrameYield::App); }) {
    v.setFrameYield(V::FrameYield::App);
    return true;
  } else {
    return false;
  }
}
// ... and the lock, for an app linked with -sASYNCIFY_IGNORE_INDIRECT=1.
template <class V> bool app_locks(V &v) {
  if constexpr (requires { v.lockFrameYield(); }) {
    v.lockFrameYield();
    return true;
  } else {
    return false;
  }
}
struct OldRenderer {}; // a cvcGL without FrameYield
static_assert(std::is_same_v<SceneRenderer::FrameYield, FrameYield>);
static_assert(std::is_same_v<ViewportManager::FrameYield, FrameYield>);

int main() {
  std::printf("cvcgl_frame_yield\n");
  cvc::app app;

  {
    SceneGraph sg(app, "fy_sr");
    SceneRenderer view(sg, 64, 48, /*offscreen=*/true, "main");
    vtkRenderWindow *rw = view.renderWindow();
    const int db0 = rw->GetDoubleBuffer();
    check(view.frameYield() == FrameYield::Vtk, "SceneRenderer: default is Vtk");
    check(app_yields(view), "SceneRenderer: requires-detection finds setFrameYield");
    check(rw->GetDoubleBuffer() == db0,
          "SceneRenderer: App leaves DoubleBuffer untouched natively");
    check(rw->GetDoubleBuffer() == 1, "SceneRenderer: DoubleBuffer stays 1 natively");
    check(view.frameYield() == FrameYield::Vtk,
          "SceneRenderer: effective mode stays Vtk natively (no in-render yield to remove)");
    check(view.viewportManager().frameYield() == view.frameYield(),
          "SceneRenderer forwards to its ViewportManager");
    view.setFrameYield(FrameYield::Vtk);
    check(rw->GetDoubleBuffer() == db0 && view.frameYield() == FrameYield::Vtk,
          "SceneRenderer: back to Vtk changes nothing natively");
    // An app's own SetDoubleBuffer (the escape hatch) is never overridden by Vtk mode.
    rw->SetDoubleBuffer(0);
    view.setFrameYield(FrameYield::Vtk);
    check(rw->GetDoubleBuffer() == 0,
          "SceneRenderer: Vtk does not reset an app's own DoubleBuffer");
    rw->SetDoubleBuffer(db0);
    // The lock: recorded, one-way, and still a no-op on the window natively.
    check(!view.frameYieldLocked(), "SceneRenderer: not locked by default");
    check(app_locks(view), "SceneRenderer: requires-detection finds lockFrameYield");
    check(view.frameYieldLocked() && view.viewportManager().frameYieldLocked(),
          "SceneRenderer: lockFrameYield locks (and forwards to its ViewportManager)");
    view.setFrameYield(FrameYield::Vtk); // refused (logged): the lock is one-way
    view.lockFrameYield();               // idempotent
    check(view.frameYieldLocked(), "SceneRenderer: setFrameYield(Vtk) does not unlock");
    check(rw->GetDoubleBuffer() == db0 && view.frameYield() == FrameYield::Vtk,
          "SceneRenderer: locked App leaves DoubleBuffer untouched natively (effective Vtk)");
    view.close();
    bool threw = false;
    try {
      view.setFrameYield(FrameYield::App);
    } catch (const std::runtime_error &) {
      threw = true;
    }
    check(threw, "SceneRenderer: setFrameYield on a closed renderer throws");
    threw = false;
    try {
      view.lockFrameYield();
    } catch (const std::runtime_error &) {
      threw = true;
    }
    check(threw, "SceneRenderer: lockFrameYield on a closed renderer throws");
  }

  {
    SceneGraph sg(app, "fy_vm");
    ViewportManager vm(sg, 64, 48, /*offscreen=*/true, "main");
    vtkRenderWindow *rw = vm.renderWindow();
    check(vm.frameYield() == FrameYield::Vtk, "ViewportManager: default is Vtk");
    check(app_yields(vm), "ViewportManager: requires-detection finds setFrameYield");
    check(rw->GetDoubleBuffer() == 1, "ViewportManager: App leaves DoubleBuffer 1 natively");
    check(vm.frameYield() == FrameYield::Vtk, "ViewportManager: effective mode stays Vtk natively");
    check(!vm.frameYieldLocked(), "ViewportManager: not locked by default");
    check(app_locks(vm) && vm.frameYieldLocked(), "ViewportManager: lockFrameYield locks");
    check(rw->GetDoubleBuffer() == 1, "ViewportManager: locked App leaves DoubleBuffer 1 natively");
  }

  OldRenderer old;
  check(!app_yields(old), "requires-detection falls back on a renderer without FrameYield");
  check(!app_locks(old), "requires-detection falls back on a renderer without the lock");

  std::printf("%s (%d failure%s)\n", fails ? "FAILED" : "OK", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
