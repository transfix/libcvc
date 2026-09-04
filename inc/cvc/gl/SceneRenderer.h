/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_SCENE_RENDERER_H__
#define __CVC_GL_SCENE_RENDERER_H__

#include <memory>
#include <string>
#include <vector>

class vtkRenderer;     // VTK (global namespace)
class vtkRenderWindow; // VTK (global namespace)

namespace cvc {
namespace gl {

class SceneGraph;

// ---------------
// SceneRenderer
// ---------------
// A render target that stays open across frames.
//
// The pre-existing one-shot helpers (pycvc's render_png, and anything shaped
// like it) build a vtkRenderer and a vtkRenderWindow, attach the whole scene,
// draw, and then Finalize() the GL context again. That is right for a single
// screenshot and wrong for a sequence: every frame pays context creation, a
// full re-attach of every node's actor, and teardown, so an animation spends
// most of its wall clock building and destroying GL state rather than drawing.
// It also re-frames the camera on each call, which is why a scripted camera
// could not be held steady from one frame to the next.
//
// SceneRenderer keeps the window and renderer alive. The scene is attached
// ONCE, at construction; each frame only drains queued scene events and
// redraws, so the per-frame cost is the draw itself. One class covers both
// consumers we care about:
//
//   * offscreen — capture a sequence to PNGs, or hand back raw RGB to pipe
//     straight into an encoder with no per-frame PNG round trip;
//   * onscreen  — a non-blocking window a caller can drive from its own loop
//     (unlike SceneGraph-level blocking `show`-style helpers, which own the
//     loop and therefore cannot be stepped by a simulation).
//
// That is deliberate: a live preview and a recording should be the same code
// path, or they drift apart and the video stops matching what was on screen.
//
// NOT a singleton and not a global. A SceneRenderer holds a reference to the
// SceneGraph it draws and owns its own VTK objects; nothing is looked up from
// a global registry — consistent with the rest of cvcGL, where the app handle
// is injected rather than discovered.
//
// ONE RENDERER PER SCENE AT A TIME, THOUGH. SceneGraph::setRenderer is a
// single attachment: constructing a second SceneRenderer over the same scene
// moves every node's actor to it and leaves the first drawing an empty frame
// — silently, since nothing errors and the first renderer keeps working, it
// just has nothing left in it (measured: mean luma 81.8 -> 0.0). For several
// views of one scene, re-aim ONE renderer between shots with setCamera; that
// is cheap, and it is what the camera persisting across frames is for. Two
// renderers are only independent when they draw two different scenes.
//
// Threading: like the rest of cvcGL, a renderer belongs to the thread that
// drives it. Construct and render on the same thread the SceneGraph pumps.
//
class SceneRenderer {
public:
  // `offscreen == false` needs a display and produces a real window. `name`
  // identifies this viewer in the state graph: a CameraController attached here
  // roots its state at "<scene prefix>.viewers.<name>.camera", so it is obvious
  // which viewer a camera belongs to.
  SceneRenderer(SceneGraph &scene, int width = 1024, int height = 768, bool offscreen = true,
                const std::string &name = "main");
  ~SceneRenderer();

  SceneRenderer(const SceneRenderer &) = delete;
  SceneRenderer &operator=(const SceneRenderer &) = delete;

  // Drain queued scene events and draw one frame.
  void render();

  // render() + encode a PNG at `path`.
  void writePNG(const std::string &path);

  // render() + the raw framebuffer as RGB bytes (3 per pixel, rows bottom-up,
  // VTK's native order). Sized frameWidth() * frameHeight() * 3.
  std::vector<unsigned char> frameRGB();

  int frameWidth() const;
  int frameHeight() const;

  // Camera. Auto-framing is a one-time convenience; an explicit camera is what
  // a scripted sequence wants, and here it persists across frames instead of
  // being reset by every draw.
  void resetCamera();
  void setCamera(double eyeX, double eyeY, double eyeZ, double focalX, double focalY, double focalZ,
                 double upX, double upY, double upZ, double viewAngle = 30.0, double clipNear = 1.0,
                 double clipFar = 1e5);

  void setBackground(double r, double g, double b);
  void resize(int width, int height);

  // Onscreen only: service window/interactor events without blocking, so a
  // playback loop stays responsive. No-op offscreen.
  void processUIEvents();

  // True once the user has closed an onscreen window; always false offscreen.
  bool windowClosed() const;

  // The renderer and window this draws through, for callers that need to
  // reach VTK directly: lights, a 2-D HUD overlay, a gradient background, a
  // scalar bar, a second camera pass. In Python these arrive as live
  // vtkmodules objects, so the scene is scriptable rather than opaque.
  vtkRenderer *renderer() const;
  vtkRenderWindow *renderWindow() const;

  // The scene this draws, and this viewer's name in the state graph.
  SceneGraph &scene() const;
  const std::string &name() const;

  // Detach from the scene and release the GL context. Idempotent; the
  // destructor calls it. Exposed so a caller can decide WHEN the context dies
  // rather than leaving it to static teardown at process exit, which segfaults
  // on some offscreen backends.
  void close();
  bool isClosed() const;

private:
  struct impl;
  std::unique_ptr<impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_SCENE_RENDERER_H__
