/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/gl/nodes.h>
#include <cvc/gl/traversal.h>
#include <stdexcept>

namespace cvc {
namespace gl {

TransformElement::TransformElement() : matrix(vtkSmartPointer<vtkMatrix4x4>::New()) {
  matrix->Identity();
}

TraversalState::TraversalState() { m_stack.emplace_back(); }

void TraversalState::push() {
  // Deep-copy the matrix rather than sharing the smart pointer: the whole point
  // of a push is that what happens above cannot be seen below the pop.
  StateFrame copy = m_stack.back();
  auto m = vtkSmartPointer<vtkMatrix4x4>::New();
  m->DeepCopy(m_stack.back().transform.matrix);
  copy.transform.matrix = m;
  m_stack.push_back(std::move(copy));
}

void TraversalState::pop() {
  // The base frame is the traversal's own; popping it would leave no state at
  // all, which can only mean a Separator's push/pop got unbalanced.
  if (m_stack.size() <= 1)
    throw std::logic_error("cvc::gl::TraversalState::pop: unbalanced push/pop");
  m_stack.pop_back();
}

StateFrame &TraversalState::top() { return m_stack.back(); }
const StateFrame &TraversalState::top() const { return m_stack.back(); }

void TraversalState::multiplyTransform(vtkMatrix4x4 *m) {
  if (!m)
    return;
  auto out = vtkSmartPointer<vtkMatrix4x4>::New();
  // current * m — Inventor's convention, so a child's transform is expressed in
  // its parent's frame.
  vtkMatrix4x4::Multiply4x4(m_stack.back().transform.matrix, m, out);
  m_stack.back().transform.matrix = out;
}

void Action::apply(Node &root) {
  m_state = TraversalState();
  m_visitedShapes = 0;
  root.traverse(*this);
}

void BoundingBoxAction::extend(const double b[6], vtkMatrix4x4 *toWorld) {
  // Transform all eight corners and take the extent of the result. Transforming
  // only min/max is wrong the moment there is a rotation in the state.
  for (int i = 0; i < 8; ++i) {
    double p[4] = {b[(i & 1) ? 3 : 0], b[(i & 2) ? 4 : 1], b[(i & 4) ? 5 : 2], 1.0};
    double w[4] = {p[0], p[1], p[2], 1.0};
    if (toWorld)
      toWorld->MultiplyPoint(p, w);
    if (m_empty) {
      m_bounds[0] = m_bounds[3] = w[0];
      m_bounds[1] = m_bounds[4] = w[1];
      m_bounds[2] = m_bounds[5] = w[2];
      m_empty = false;
    } else {
      m_bounds[0] = std::min(m_bounds[0], w[0]);
      m_bounds[1] = std::min(m_bounds[1], w[1]);
      m_bounds[2] = std::min(m_bounds[2], w[2]);
      m_bounds[3] = std::max(m_bounds[3], w[0]);
      m_bounds[4] = std::max(m_bounds[4], w[1]);
      m_bounds[5] = std::max(m_bounds[5], w[2]);
    }
  }
}

} // namespace gl
} // namespace cvc
