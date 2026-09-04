#ifndef CVC_GL_SCREEN_TEXT_HUD_H
#define CVC_GL_SCREEN_TEXT_HUD_H

#include <cvc/core/state_object.h>
#include <memory>
#include <string>

class vtkRenderer; // VTK (global namespace)

namespace cvc {
namespace gl {
class SceneRenderer;

// --------------
// ScreenTextHud
// --------------
// A screen-space text overlay — title cards, story captions, status lines —
// drawn by VTK itself (a vtkTextActor 2-D overlay) so it appears wherever the
// scene renders: the native window, an offscreen capture, or the WebAssembly
// canvas.
//
// EVERYTHING IS cvc::state, like FpsHud and CameraController: the text and every
// setting live under "<scene prefix>.viewers.<viewer>.hud.<name>" and are
// two-way bound, so a caption can be set (or read) from a script, a config file
// or a replicated peer.
//
// This used to be deliberately unbound, on the grounds that captions are driven
// by the host frame loop and a state write per change would be overhead. That
// concern was real but the fix is narrower than dropping binding: every write
// here is CHANGE-GATED, so calling setText() with the same string sixty times a
// second — which is exactly how a caption table is evaluated — costs nothing
// after the first. Only an actual change reaches the state tree.
//
// Several instances coexist on one viewer (a title card + a caption band + a
// status line), each owning its own actor, so each needs its own NAME to get its
// own state node. Pass one when you construct more than one per viewer.
//
// Position is in NORMALIZED VIEWPORT coordinates ([0,1] x [0,1], origin bottom
// left), so placement survives window resizes and capture sizes. Text is
// horizontally centered on the anchor by default (a caption band look); switch
// to left-justified for a corner status line. An empty text hides the actor.
//
// Call from the render/frame thread (the same thread that drives
// SceneRenderer::render / processUIEvents), like every other per-frame call.
class ScreenTextHud : public cvc::state_object<ScreenTextHud> {
public:
  // Adds the overlay to the viewer's renderer. Defaults: centered at
  // (0.5, 0.06) — a lower-third caption band — font 18, warm white, shadowed
  // for readability on any scene, hidden until text is set.
  // `name` distinguishes several overlays on one viewer; it is the last element
  // of the state path. Two overlays sharing a name would share state nodes.
  explicit ScreenTextHud(SceneRenderer &viewer, const std::string &name = "text");
  // Low-level: explicit app context + full state path (headless / custom).
  ScreenTextHud(cvc::app &ctx, const std::string &statePath, SceneRenderer *viewer);
  ~ScreenTextHud(); // removes the actor from the renderer

  // The canonical state path for one named overlay on a viewer.
  static std::string viewerStatePath(const std::string &scenePrefix, const std::string &viewerName,
                                     const std::string &name);

  ScreenTextHud(const ScreenTextHud &) = delete;
  ScreenTextHud &operator=(const ScreenTextHud &) = delete;

  // Every setter is CHANGE-GATED (see above) and writes through to state; every
  // getter reads back the same setting whether it arrived through the setter or
  // through state. That is what makes the binding two-way in a way you can
  // actually observe: a caption pushed in by a script, a config file or a
  // replicated peer is only reachable from C++ through these, and a test that
  // reads state back out of state proves nothing about whether the overlay ever
  // saw the change. Position and color are read through out-parameters, like
  // StageLighting::stage.
  void setText(const std::string &text); // "" hides the overlay
  const std::string &text() const;
  void setPosition(double nx, double ny);
  void position(double &nx, double &ny) const;
  void setFontSize(int points);
  int fontSize() const;
  void setColor(double r, double g, double b);
  void color(double &r, double &g, double &b) const;
  void setOpacity(double alpha);
  double opacity() const;
  void setCentered(bool centered); // false = left-justified (status-line style)
  bool centered() const;
  void setVisible(bool on); // AND'ed with "has text"
  bool visible() const;     // the caller's intent, NOT whether the actor draws

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void seedState();
  void readAllFromState();
  void applyToActor();

  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_SCREEN_TEXT_HUD_H
