// TouchGestures — pinch-zoom and two-finger pan for cvcGL on touch devices.
// See the header for WHY this bypasses VTK's multi-touch path entirely.
//
// Settings are cvc::state, like CameraController/FpsHud. The DOM-side gesture
// accumulator is stored ON THE CANVAS ELEMENT, not on window, so two viewers on
// one page do not share one.

#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/TouchGestures.h>
#include <stdexcept>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>

// Install capture-phase listeners for THIS viewer's canvas. The accumulator is
// a property of the canvas element (el.__cvcglTouch), so it is per-viewer.
// Listeners handle 2+ finger gestures only and stopPropagation so VTK's own
// (broken) multi-touch handlers never see them; single touches pass straight
// through, which is what keeps taps on ImGui widgets and 1-finger drag working.
EM_JS(void, cvcgl_touch_install, (const char *sel), {
  var s = UTF8ToString(sel);
  var el = document.querySelector(s) || document.getElementById('canvas') ||
           document.querySelector('canvas');
  if (!el || el.__cvcglTouch)
    return;
  var st = {scale : 1.0, panX : 0.0, panY : 0.0, active : false, d0 : 0, cx : 0, cy : 0};
  el.__cvcglTouch = st;
  var target = el.parentElement || el;

  function dist(t) {
    var dx = t[0].clientX - t[1].clientX, dy = t[0].clientY - t[1].clientY;
    return Math.sqrt(dx * dx + dy * dy);
  }
  function mid(t) {
    return [ (t[0].clientX + t[1].clientX) / 2, (t[0].clientY + t[1].clientY) / 2 ];
  }
  function start(e) {
    if (e.touches.length < 2)
      return; // single touch: leave it to VTK (taps, ImGui, 1-finger drag)
    e.stopPropagation();
    e.preventDefault();
    var m = mid(e.touches);
    st.active = true;
    st.d0 = dist(e.touches);
    st.cx = m[0];
    st.cy = m[1];
  }
  function move(e) {
    if (e.touches.length < 2) {
      st.active = false;
      return;
    }
    e.stopPropagation();
    e.preventDefault();
    var d = dist(e.touches), m = mid(e.touches);
    if (st.active && st.d0 > 1.0) {
      st.scale *= (d / st.d0);   // pinch, accumulated multiplicatively
      st.panX += (m[0] - st.cx); // two-finger drag, accumulated in pixels
      st.panY += (m[1] - st.cy);
    }
    st.d0 = d;
    st.cx = m[0];
    st.cy = m[1];
    st.active = true;
  }
  function end(e) {
    if (e.touches.length < 2)
      st.active = false;
    if (e.touches.length >= 2) {
      e.stopPropagation();
      e.preventDefault();
    }
  }
  // Non-passive: we must be allowed to preventDefault, or the browser's own
  // page pinch-zoom fights the camera.
  var opt = {capture : true, passive : false};
  target.addEventListener('touchstart', start, opt);
  target.addEventListener('touchmove', move, opt);
  target.addEventListener('touchend', end, opt);
  target.addEventListener('touchcancel', end, opt);
});

// Drain this canvas's accumulators into out[3] = {scale, panX, panY}, resetting
// them. One call per frame keeps the JS/wasm boundary crossings at one.
EM_JS(void, cvcgl_touch_take, (const char *sel, double *out), {
  var s = UTF8ToString(sel);
  var el = document.querySelector(s) || document.getElementById('canvas') ||
           document.querySelector('canvas');
  var st = el && el.__cvcglTouch;
  setValue(out, st ? st.scale : 1.0, 'double');
  setValue(out + 8, st ? st.panX : 0.0, 'double');
  setValue(out + 16, st ? st.panY : 0.0, 'double');
  if (st) {
    st.scale = 1.0;
    st.panX = 0.0;
    st.panY = 0.0;
  }
});
#endif // __EMSCRIPTEN__

namespace cvc {
namespace gl {

struct TouchGestures::Impl {
  CameraController *cam = nullptr;
  std::string canvas = "#canvas";
  // mirrored from state
  bool enabled = true;
  bool panEnabled = true;
  bool invertPinch = false;
  double pinchSteps = 4.0; // wheel-steps per doubling of the pinch distance
};

std::string TouchGestures::viewerStatePath(const std::string &scenePrefix,
                                           const std::string &viewerName) {
  return scenePrefix + ".viewers." + viewerName + ".touch";
}

TouchGestures::TouchGestures(cvc::app &ctx, const std::string &statePath, CameraController *cam,
                             const std::string &canvasSelector)
    : cvc::state_object<TouchGestures>(ctx, statePath), m_impl(std::make_unique<Impl>()) {
  // Synchronous reactions on the calling thread, like CameraController: this is
  // driven from the render thread anyway.
  this->setInstanceThreading(false);
  m_impl->cam = cam;
  if (!canvasSelector.empty())
    m_impl->canvas = canvasSelector;
  seedState();
#ifdef __EMSCRIPTEN__
  cvcgl_touch_install(m_impl->canvas.c_str());
#endif
}

TouchGestures::TouchGestures(SceneRenderer &viewer, CameraController &cam)
    : TouchGestures(viewer.scene().appContext(),
                    viewerStatePath(viewer.scene().getStatePrefix(), viewer.name()), &cam,
                    "#canvas") {}

TouchGestures::~TouchGestures() = default;

void TouchGestures::seedState() {
  Impl &s = *m_impl;
  getState("enabled").value(s.enabled ? 1 : 0);
  getState("pinch_steps").value(s.pinchSteps);
  getState("pan_enabled").value(s.panEnabled ? 1 : 0);
  getState("invert_pinch").value(s.invertPinch ? 1 : 0);
}

void TouchGestures::readAllFromState() {
  Impl &s = *m_impl;
  try {
    s.enabled = getState("enabled").value<int>() != 0;
    s.pinchSteps = getState("pinch_steps").value<double>();
    s.panEnabled = getState("pan_enabled").value<int>() != 0;
    s.invertPinch = getState("invert_pinch").value<int>() != 0;
  } catch (const std::exception &) {
    // partially-initialised state: keep what we have
  }
}

void TouchGestures::handleStateChanged(const std::string &) { readAllFromState(); }

void TouchGestures::setEnabled(bool on) { getState("enabled").value(on ? 1 : 0); }
bool TouchGestures::enabled() const { return m_impl->enabled; }
void TouchGestures::setPinchSteps(double v) { getState("pinch_steps").value(v); }
double TouchGestures::pinchSteps() const { return m_impl->pinchSteps; }
void TouchGestures::setPanEnabled(bool on) { getState("pan_enabled").value(on ? 1 : 0); }
bool TouchGestures::panEnabled() const { return m_impl->panEnabled; }
void TouchGestures::setInvertPinch(bool on) { getState("invert_pinch").value(on ? 1 : 0); }

bool TouchGestures::supported() {
#ifdef __EMSCRIPTEN__
  return true;
#else
  return false;
#endif
}

void TouchGestures::update() {
#ifdef __EMSCRIPTEN__
  Impl &s = *m_impl;
  if (!s.enabled || !s.cam)
    return;
  double g[3] = {1.0, 0.0, 0.0};
  cvcgl_touch_take(s.canvas.c_str(), g);
  const double scale = g[0], panX = g[1], panY = g[2];

  // Pinch -> zoom. mouseWheel() already means "zoom" in every camera mode
  // (parallel scale in Map, orbit distance in Orbit, fly speed in Fly), so the
  // gesture maps onto the one control that is always correct. log2 keeps the
  // response proportional: doubling the finger distance is a fixed number of
  // steps regardless of where the pinch started.
  if (scale > 0.0 && std::abs(scale - 1.0) > 1e-4) {
    double steps = s.pinchSteps * std::log2(scale);
    if (s.invertPinch)
      steps = -steps;
    s.cam->mouseWheel(steps);
  }

  // Two-finger drag -> the same motion a mouse drag makes: pan on a 2-D map,
  // orbit in 3-D. beginDrag/endDrag bracket it because Map-mode panning only
  // applies while the controller considers itself dragging.
  if (s.panEnabled && (std::abs(panX) > 0.5 || std::abs(panY) > 0.5)) {
    s.cam->beginDrag();
    s.cam->mouseLook(static_cast<int>(panX), static_cast<int>(panY));
    s.cam->endDrag();
  }
#endif
}

} // namespace gl
} // namespace cvc
