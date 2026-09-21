/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_VIEWPORT_MANAGER_H__
#define __CVC_GL_VIEWPORT_MANAGER_H__

#include <memory>
#include <string>
#include <vector>

class vtkRenderWindow; // VTK (global namespace)

namespace cvc {
class app;
namespace gl {
class SceneGraph;
class Viewport;

// ----------------
// ViewportManager
// ----------------
// The N-viewport superset of SceneRenderer: it owns ONE vtkRenderWindow (ONE GL
// context — mandatory under WASM's single canvas) and composites N Viewports as
// LAYERED vtkRenderers in that window, differentiated by SetViewport rect and
// SetLayer, drawn by a single window->Render(). This is the picture-in-picture
// path (the DBG minimap, generalised): alternate views of one scene, or entirely
// separate scenes, all in one app instance and one context.
//
// It is constructed like a SceneRenderer (a main scene + size + offscreen flag)
// and auto-creates a full-screen PRIMARY viewport over that scene, so the common
// single-view case is unchanged. Extra viewports are added over other scenes
// (addSceneViewport) or as alternate views (addMirrorViewport, a follow-up).
//
// The app the viewports' cameras live in is taken from the main scene.
class ViewportManager {
public:
  ViewportManager(SceneGraph &mainScene, int width = 1024, int height = 768, bool offscreen = true,
                  const std::string &name = "main");
  ~ViewportManager();

  ViewportManager(const ViewportManager &) = delete;
  ViewportManager &operator=(const ViewportManager &) = delete;

  // The auto-created full-screen viewport over the main scene (layer 0).
  Viewport &primary();
  Viewport &viewport(const std::string &name);
  bool hasViewport(const std::string &name) const;
  std::vector<std::string> viewportNames() const;

  // Add a viewport drawing a DIFFERENT SceneGraph (a clean, independent renderer
  // via scene.setRenderer). `region` is normalized [x0,y0,x1,y1]; `layer` >= 1
  // for an inset drawn over the primary. Throws if `name` is taken or `scene` is
  // already drawn by another viewport (a scene attaches to ONE renderer — the
  // loud-guard for the silent "second setRenderer blanks the first view" trap).
  Viewport &addSceneViewport(const std::string &name, SceneGraph &scene, const double region[4],
                             int layer = 1);

  // Add a MIRROR viewport: an alternate view (its own camera) of the scene that
  // `sourceViewport` already draws — the true minimap. It does NOT attach the
  // scene a second time (which would blank the source); instead render() copies
  // the source renderer's props into it each frame, so it always shows the live
  // scene from its own camera. `region` is normalized [x0,y0,x1,y1]; `layer`
  // defaults to 2 (above a layer-1 inset). Throws if `name` is taken or
  // `sourceViewport` does not exist.
  Viewport &addMirrorViewport(const std::string &name, const std::string &sourceViewport,
                              const double region[4], int layer = 2);

  // Composite every visible viewport in one pass (drains each scene's pending
  // graphics events first, like SceneRenderer::render).
  void render();
  void resize(int width, int height);
  void resetCamera(); // primary()
  void processUIEvents();
  bool windowClosed() const;

  // Tick EVERY viewport's CameraController once per rendered frame: held-key fly
  // motion (frame-rate independent via dt), Track smoothing, Map refit, throttled
  // pose->state mirroring. render() does NOT do this (parity with SceneRenderer's
  // host-driven loop) — a host that wants live cameras calls this before render().
  // Ticks all viewports, not just the active one, so a background Track/minimap
  // camera still follows.
  void updateCameras(double dtSeconds);

  // ---- input routing --------------------------------------------------------
  // The picture-in-picture input problem: one window, one interactor, N cameras.
  // Onscreen, an internal interactor style feeds these route*() methods; they are
  // also PUBLIC and interactor-free so the routing logic is unit-testable offscreen
  // (drive them with synthetic display-pixel coordinates). All positions are in
  // display pixels, VTK's bottom-left origin (what vtkRenderWindowInteractor::
  // GetEventPosition returns) — do NOT pass top-left/DOM coordinates without
  // flipping y, or pitch and vertical pan invert.
  enum class MouseButton { Left, Middle, Right };

  // The routing DECISION: the TOPMOST visible, input-enabled viewport whose
  // renderer contains display-pixel (x, y), iterating in descending layer (a
  // later-added viewport wins on a tie); nullptr over the gutter. Layer- and
  // visibility-aware, and returns null on a miss — unlike VTK's FindPokedRenderer,
  // which ignores SetLayer/SetDraw and silently falls back to the primary.
  Viewport *viewportAt(int x, int y) const;

  // A button event. On down: hit-test, latch that viewport for the whole drag,
  // make it the active (keyboard) viewport (focus-follows-click), and feed
  // Left->beginDrag / Middle->beginPan (Right reserved, no-op). On up: feed the
  // matching end on the LATCHED viewport, releasing the latch when no button
  // remains held — so a drag that wanders out of its viewport keeps steering the
  // camera it started on.
  void routeMouseButton(MouseButton button, bool down, int x, int y);
  // Pointer motion. During a drag: feed mouseLook(dx, dy) (deltas vs the last
  // position) to the latched viewport. Otherwise route to the viewport under the
  // cursor (Fly free-look), resetting the delta baseline when the hovered viewport
  // changes so no cross-boundary jump is fed.
  void routeMouseMove(int x, int y);
  // Wheel goes to the viewport under the cursor (not the latched/active one):
  // mouseWheel(steps), steps = +1 forward / -1 backward. No-op over the gutter.
  void routeMouseWheel(int x, int y, double steps);
  // Keyboard has no cursor, so it targets the ACTIVE viewport. "Escape" releases
  // that viewport's pointer capture and is not forwarded as a key (matching the
  // single-view style); every other keySym is a VTK key sym passed verbatim.
  void routeKey(const std::string &keySym, bool down);

  // The viewport keyboard events go to (the last clicked). Resolves the name in
  // cvc::state at "<main scene prefix>.active_viewport"; falls back to the primary
  // if unset/unknown, so it is never null after construction.
  Viewport *activeViewport() const;
  // Focus a viewport programmatically (no click). Writes the active_viewport state
  // key and calls releaseHeldKeys() on the outgoing viewport so a key held across
  // the handoff does not stick. Ignores an unknown name.
  void setActiveViewport(const std::string &name);

  // Whole-window capture (all viewports + layers), rows bottom-up, RGB.
  void writePNG(const std::string &path);
  std::vector<unsigned char> frameRGB();
  int frameWidth() const;
  int frameHeight() const;

  vtkRenderWindow *renderWindow() const; // escape hatch, parity with SceneRenderer

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_VIEWPORT_MANAGER_H__
