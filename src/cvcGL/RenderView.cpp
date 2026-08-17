/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <vtkProp.h>
#include <vtkRenderer.h>

#include <cvc/gl/RenderView.h>
#include <cvc/gl/nodes.h>
#include <cvc/gl/traversal.h>

namespace cvc {
namespace gl {

RenderView::RenderView(vtkRenderer *renderer, std::string name)
    : m_renderer(renderer), m_name(std::move(name)) {}

RenderView::~RenderView() {
  // Tell every shape holding an instance for us to drop it. Without this a
  // shape outliving the view would keep a prop bound to a dead renderer, and
  // the next traversal for a *different* view would still see it in the map.
  auto shapes = m_shapes; // releaseView() calls back into untrackShape()
  for (Shape *s : shapes)
    if (s)
      s->releaseView(this);
  m_shapes.clear();
}

void RenderView::attachProp(vtkProp *prop) {
  if (m_renderer && prop)
    m_renderer->AddViewProp(prop);
}

void RenderView::detachProp(vtkProp *prop) {
  if (m_renderer && prop)
    m_renderer->RemoveViewProp(prop);
}

void RenderView::trackShape(Shape *shape) {
  if (shape)
    m_shapes.insert(shape);
}

void RenderView::untrackShape(Shape *shape) { m_shapes.erase(shape); }

void RenderView::render(Node &root) {
  RenderAction action(*this);
  action.apply(root);
}

} // namespace gl
} // namespace cvc
