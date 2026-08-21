#ifndef CVC_GL_TOUCH_GESTURES_H
#define CVC_GL_TOUCH_GESTURES_H

#include <cvc/core/state_object.h>
#include <memory>
#include <string>

class SceneRenderer; // cvcGL (global namespace)

namespace cvc {
class app;

namespace gl {

class CameraController;

// ---------------
// TouchGestures
// ---------------
// Phone/tablet camera control: PINCH to zoom and two-finger drag to pan/turn,
// on top of a cvcGL viewer that has a CameraController.
//
//     cvc::gl::CameraController cam(view);
//     cvc::gl::TouchGestures touch(view, cam);   // that is the whole setup
//     while (...) { ...; touch.update(); view.render(); }
//
// EVERYTHING IS cvc::state, exactly like CameraController and FpsHud: the
// settings live under "<scene prefix>.viewers.<viewer name>.touch" and are
// two-way bound. Set "...touch.pinch_steps" from a script, a config file or a
// replicated peer and the gesture response follows; the setters below write the
// same state. Keys: enabled, pinch_steps, pan_enabled, invert_pinch.
//
// WHY THIS EXISTS (VTK's own multi-touch does not work in the browser):
// VTK's gesture recognizer is complete and enabled by default, and the wasm
// interactor does listen for touch events — but it feeds them wrong in three
// independent ways:
//   1. vtkWebAssemblyRenderWindowInteractor::ProcessEvent loops
//      SetEventInformation over every touch and then calls LeftButtonPressEvent
//      ONCE, so when two fingers land in the same touchstart (how people
//      actually pinch) PointersDownCount only reaches 1 and RecognizeGesture —
//      which requires >1 — never runs. The camera orbits instead of zooming.
//   2. it uses the touch ARRAY INDEX as VTK's pointer slot instead of
//      Touch.identifier, so slots shift and leak when fingers lift out of order
//      (VTK ships GetPointerIndexForContact() for exactly this; the Android
//      interactor uses it, the wasm one does not).
//   3. vtkInteractorStyle::OnPinch/OnPan/OnRotate are empty virtuals that only
//      vtkInteractorStyleMultiTouchCamera overrides.
//
// Rather than fight that, this class handles MULTI-TOUCH ONLY, in the capture
// phase, and stops those events before VTK sees them. SINGLE touches are left
// completely alone: VTK already synthesizes them into mouse events, so taps on
// ImGui widgets and one-finger drag keep working exactly as they do today. Dear
// ImGui is single-touch by design, so it can never claim a pinch — the two
// layers cannot fight over the same gesture.
//
// Gestures are accumulated by the DOM listeners and applied in update(), which
// you call once per frame: VTK drains its own event queue inside
// requestAnimationFrame, so applying camera changes at frame time keeps this in
// step with everything else. The accumulator lives ON THE CANVAS ELEMENT, so
// two viewers on one page each get their own — there is no global.
//
// Native builds: this compiles to a no-op shell (there is no browser canvas to
// listen to). Desktop touchscreens go through VTK's normal gesture path, for
// which CvcCameraInteractorStyle implements OnStartPinch/OnPinch/OnPan.
class TouchGestures : public cvc::state_object<TouchGestures> {
public:
  // Canonical: state at "<viewer.scene prefix>.viewers.<viewer.name>.touch".
  TouchGestures(SceneRenderer &viewer, CameraController &cam);
  // Low-level: explicit app context + full state path (headless / custom).
  TouchGestures(cvc::app &ctx, const std::string &statePath, CameraController *cam,
                const std::string &canvasSelector);
  ~TouchGestures();

  TouchGestures(const TouchGestures &) = delete;
  TouchGestures &operator=(const TouchGestures &) = delete;

  // The canonical state path for a viewer's touch settings.
  static std::string viewerStatePath(const std::string &scenePrefix, const std::string &viewerName);

  // Apply any gesture accumulated since the last call. Call once per frame.
  void update();

  // ---- settings (mirrored to state "…touch.*") ----
  void setEnabled(bool on); // state "enabled"
  bool enabled() const;
  // Wheel-steps per doubling of the pinch distance (state "pinch_steps").
  void setPinchSteps(double stepsPerDoubling);
  double pinchSteps() const;
  // Two-finger drag drives the camera's drag motion (state "pan_enabled").
  void setPanEnabled(bool on);
  bool panEnabled() const;
  // Reverse the pinch direction (state "invert_pinch").
  void setInvertPinch(bool on);

  // True on a build where touch handling is actually installed (wasm).
  static bool supported();

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void seedState();
  void readAllFromState();

  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_TOUCH_GESTURES_H
