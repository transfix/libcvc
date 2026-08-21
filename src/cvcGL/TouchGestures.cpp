// TouchGestures — pinch-zoom and two-finger pan for cvcGL on touch devices.
// See the header for WHY this bypasses VTK's multi-touch path entirely.

#include <cmath>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/TouchGestures.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>

// The DOM half. Listeners go on the canvas's PARENT in the CAPTURE phase so we
// see a multi-touch gesture before VTK's own (broken) handlers on the canvas,
// and stopPropagation keeps it that way. Anything that is not a 2+ finger
// gesture is passed straight through untouched — that is what keeps single-tap
// on ImGui widgets and one-finger drag working.
//
// The handlers only ACCUMULATE into a plain object; C++ drains it each frame.
// Nothing here calls into wasm, so there is no re-entrancy against VTK's queue.
EM_JS(void, cvcgl_touch_install, (), {
  if (window.__cvcglTouch)
    return;
  var st = {
    scale : 1.0,
    panX : 0.0,
    panY : 0.0,
    active : false,
    d0 : 0,
    cx : 0,
    cy : 0,
    installed : false
  };
  window.__cvcglTouch = st;
  var canvas = document.getElementById('canvas') || document.querySelector('canvas');
  if (!canvas)
    return;
  var target = canvas.parentElement || canvas;

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
    if (e.touches.length >= 1 && e.touches.length < 2)
      return; // let VTK own the remaining finger
    if (e.touches.length >= 2) {
      e.stopPropagation();
      e.preventDefault();
    }
  }
  // Non-passive: we must be allowed to preventDefault (browser pinch-zoom of
  // the PAGE would otherwise fight the camera).
  var opt = {capture : true, passive : false};
  target.addEventListener('touchstart', start, opt);
  target.addEventListener('touchmove', move, opt);
  target.addEventListener('touchend', end, opt);
  target.addEventListener('touchcancel', end, opt);
  st.installed = true;
});

// Drain the accumulators (and reset them) in one call.
EM_JS(double, cvcgl_touch_take_scale, (), {
  var st = window.__cvcglTouch;
  if (!st)
    return 1.0;
  var s = st.scale;
  st.scale = 1.0;
  return s;
});
EM_JS(double, cvcgl_touch_take_pan_x, (), {
  var st = window.__cvcglTouch;
  if (!st)
    return 0.0;
  var v = st.panX;
  st.panX = 0.0;
  return v;
});
EM_JS(double, cvcgl_touch_take_pan_y, (), {
  var st = window.__cvcglTouch;
  if (!st)
    return 0.0;
  var v = st.panY;
  st.panY = 0.0;
  return v;
});
EM_JS(int, cvcgl_touch_installed, (),
      { return (window.__cvcglTouch && window.__cvcglTouch.installed) ? 1 : 0; });
#endif // __EMSCRIPTEN__

namespace cvc {
namespace gl {

struct TouchGestures::Impl {
  CameraController *cam = nullptr;
  bool enabled = true;
  double pinchSteps = 4.0; // wheel-steps per doubling of the pinch distance
};

TouchGestures::TouchGestures(SceneRenderer &viewer, CameraController &cam) : m_impl(new Impl) {
  (void)viewer;
  m_impl->cam = &cam;
#ifdef __EMSCRIPTEN__
  cvcgl_touch_install();
#endif
}

TouchGestures::~TouchGestures() = default;

void TouchGestures::setEnabled(bool on) { m_impl->enabled = on; }
bool TouchGestures::enabled() const { return m_impl->enabled; }
void TouchGestures::setPinchScale(double stepsPerDoubling) {
  m_impl->pinchSteps = stepsPerDoubling;
}

bool TouchGestures::supported() {
#ifdef __EMSCRIPTEN__
  return true;
#else
  return false;
#endif
}

void TouchGestures::update() {
#ifdef __EMSCRIPTEN__
  if (!m_impl->enabled || !m_impl->cam)
    return;
  const double scale = cvcgl_touch_take_scale();
  const double panX = cvcgl_touch_take_pan_x();
  const double panY = cvcgl_touch_take_pan_y();

  // Pinch -> zoom. mouseWheel() already means "zoom" in every camera mode
  // (parallel scale in Map, orbit distance in Orbit, fly speed in Fly), so the
  // gesture maps onto the one control that is always correct. log2 keeps the
  // response proportional: doubling the finger distance is a fixed number of
  // steps regardless of where the pinch started.
  if (scale > 0.0 && std::abs(scale - 1.0) > 1e-4) {
    const double steps = m_impl->pinchSteps * std::log2(scale);
    m_impl->cam->mouseWheel(steps);
  }

  // Two-finger drag -> the same motion a mouse drag makes: pan on a 2-D map,
  // orbit in 3-D. beginDrag/endDrag bracket it because Map-mode panning only
  // applies while the controller considers itself dragging.
  if (std::abs(panX) > 0.5 || std::abs(panY) > 0.5) {
    m_impl->cam->beginDrag();
    m_impl->cam->mouseLook(static_cast<int>(panX), static_cast<int>(panY));
    m_impl->cam->endDrag();
  }
#endif
}

} // namespace gl
} // namespace cvc
