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
