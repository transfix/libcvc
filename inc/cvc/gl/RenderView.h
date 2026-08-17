/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_RENDERVIEW_H__
#define __CVC_GL_RENDERVIEW_H__

#include <cstdint>
#include <memory>
#include <set>
#include <string>

#include <vtkSmartPointer.h>

class vtkRenderer;
class vtkProp;

namespace cvc {
namespace gl {

class Node;
class Shape;

// ---------------------------------------------------------------------------
// cvc::gl::RenderView — one window's worth of rendering.
// ---------------------------------------------------------------------------
// The unit the old design was missing. SceneNode holds a single vtkRenderer*,
// so a graph belongs to one window and nothing can differ between windows;
// here the renderer lives in the VIEW, the graph knows nothing about it, and a
// shape keeps one prop per view it has been traversed for.
//
// A view carries its own visibility mask, so a graph annotated with
// VisibilityMask nodes shows different content per window with no graph edits:
// give the overview window mask 0x2 and tag the annotations 0x2.
//
// The view does NOT own the render window or the camera — the host does
// (volrover3's VTKRenderWidget, a SceneRenderer, an offscreen capture). It owns
// the renderer and the bookkeeping of which props it has been handed, so that
// removing a view cleanly detaches every shape's instance for it.
class RenderView {
public:
  explicit RenderView(vtkRenderer *renderer, std::string name = {});
  ~RenderView();

  RenderView(const RenderView &) = delete;
  RenderView &operator=(const RenderView &) = delete;

  vtkRenderer *renderer() const { return m_renderer; }
  const std::string &name() const { return m_name; }

  // Only shapes whose accumulated mask intersects this are drawn here.
  std::uint32_t visibilityMask() const { return m_mask; }
  void setVisibilityMask(std::uint32_t mask) { m_mask = mask; }

  // Called by Shape when it creates/destroys its prop for this view. Keeps the
  // renderer's prop collection in step without the shape knowing about VTK's
  // AddViewProp/RemoveViewProp ordering rules.
  void attachProp(vtkProp *prop);
  void detachProp(vtkProp *prop);

  // Register shapes that hold an instance for this view, so the view can tell
  // them to let go when it is destroyed. Without this a shape would keep a prop
  // for a renderer that no longer exists.
  void trackShape(Shape *shape);
  void untrackShape(Shape *shape);

  // Traverse `root` for this view and reconcile every prop. One call per frame
  // per view; the graph is shared, the result is not.
  void render(Node &root);

private:
  vtkRenderer *m_renderer = nullptr;
  std::string m_name;
  std::uint32_t m_mask = 0xFFFFFFFFu;
  std::set<Shape *> m_shapes;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_RENDERVIEW_H__
