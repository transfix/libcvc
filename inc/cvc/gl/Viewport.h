/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_VIEWPORT_H__
#define __CVC_GL_VIEWPORT_H__

#include <memory>
#include <string>

class vtkRenderer; // VTK (global namespace)

namespace cvc {
class app;
namespace gl {
class SceneGraph;
class CameraController;
class ViewportManager;

// -----------
// Viewport
// -----------
// One rectangle of a ViewportManager's single window: its OWN vtkRenderer,
// CameraController and the SceneGraph it draws. It never opens a GL context —
// it is a vtkRenderer inside the manager's one vtkRenderWindow, placed by a
// NORMALIZED region and a layer, composited with the others in one render.
//
// Two flavours (both first-class): a SCENE viewport draws its own distinct
// SceneGraph (a clean, independent renderer); a MIRROR viewport is an alternate
// view of a scene another viewport already draws (the DBG-minimap / PiP case).
// This header is the shared surface; the manager's addSceneViewport /
// addMirrorViewport decide the flavour.
//
// Each viewport owns a CameraController at its own state path
// ("<scene prefix>.viewers.<name>.camera"), so a PiP inset aims / follows
// independently of the main view. The controller is built with the low-level
// (app, path) ctor and driven through update()/event feeds — it does NOT attach
// to the window interactor (the manager's one input router owns that).
class Viewport {
public:
  ~Viewport();
  Viewport(const Viewport &) = delete;
  Viewport &operator=(const Viewport &) = delete;

  const std::string &name() const;

  // Placement. region is normalized [x0, y0, x1, y1] in VTK's y-up window space;
  // {0,0,1,1} is the whole window. Stored normalized, so it survives a resize and
  // is re-applied to the renderer; move it every frame to track a draggable inset.
  void setRegion(double x0, double y0, double x1, double y1);
  void region(double out[4]) const;

  // Composite layer: 0 is the base (opaque) layer; >0 is an overlay drawn on top.
  // The manager keeps the window's layer count >= max(layer)+1.
  void setLayer(int layer);
  int layer() const;

  // Cheap show/hide: a hidden viewport is skipped in the composite (its renderer
  // is left in the window but not drawn), so a PiP can be toggled per frame.
  void setVisible(bool on);
  bool visible() const;

  // Background colour + whether this viewport CLEARS its rect. An opaque overlay
  // panel clears (so it reads as a solid inset); a translucent overlay must NOT
  // clear, or it wipes the base view under it.
  void setBackground(double r, double g, double b, bool opaque = true);

  CameraController &camera(); // this viewport's own controller / vtkCamera
  SceneGraph &scene() const;  // the scene it draws (its own, or a mirror source)
  vtkRenderer *renderer() const;

  bool isMirror() const;
  // For a mirror viewport, the source viewport's renderer whose props it echoes
  // each frame (with its own camera); nullptr for a scene viewport. The manager
  // reads this in render() to keep the mirror in sync with the live scene.
  vtkRenderer *mirrorSource() const;

private:
  friend class ViewportManager; // the manager constructs and owns Viewports
  Viewport(cvc::app &app, SceneGraph &scene, const std::string &cameraStatePath,
           const std::string &name, bool mirror);
  void setMirrorSource(vtkRenderer *src); // manager-only, at addMirrorViewport

  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_VIEWPORT_H__
