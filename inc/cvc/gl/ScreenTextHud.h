#ifndef CVC_GL_SCREEN_TEXT_HUD_H
#define CVC_GL_SCREEN_TEXT_HUD_H

#include <memory>
#include <string>

class vtkRenderer;   // VTK (global namespace)
class SceneRenderer; // cvcGL (global namespace)

namespace cvc {
namespace gl {

// --------------
// ScreenTextHud
// --------------
// A screen-space text overlay — title cards, story captions, status lines —
// drawn by VTK itself (a vtkTextActor 2-D overlay) so it appears wherever the
// scene renders: the native window, an offscreen capture, or the WebAssembly
// canvas.
//
// Deliberately LEAN and DIRECT, unlike FpsHud: no state binding, no observers.
// Captions are driven by the host frame loop (a caption table evaluated against
// sim time), so a state round-trip per change would be overhead with no
// scriptability payoff. FpsHud remains the state-bound overlay pattern; this is
// the "just draw the words" one. Several instances coexist on one viewer (a
// title card + a caption band + a status line), each owning its own actor.
//
// Position is in NORMALIZED VIEWPORT coordinates ([0,1] x [0,1], origin bottom
// left), so placement survives window resizes and capture sizes. Text is
// horizontally centered on the anchor by default (a caption band look); switch
// to left-justified for a corner status line. An empty text hides the actor.
//
// Call from the render/frame thread (the same thread that drives
// SceneRenderer::render / processUIEvents), like every other per-frame call.
class ScreenTextHud {
public:
  // Adds the overlay to the viewer's renderer. Defaults: centered at
  // (0.5, 0.06) — a lower-third caption band — font 18, warm white, shadowed
  // for readability on any scene, hidden until text is set.
  explicit ScreenTextHud(SceneRenderer &viewer);
  ~ScreenTextHud(); // removes the actor from the renderer

  ScreenTextHud(const ScreenTextHud &) = delete;
  ScreenTextHud &operator=(const ScreenTextHud &) = delete;

  void setText(const std::string &text); // "" hides the overlay
  void setPosition(double nx, double ny);
  void setFontSize(int points);
  void setColor(double r, double g, double b);
  void setOpacity(double alpha);
  void setCentered(bool centered); // false = left-justified (status-line style)
  void setVisible(bool on);        // AND'ed with "has text"

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_SCREEN_TEXT_HUD_H
