/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_TRAVERSAL_H__
#define __CVC_GL_TRAVERSAL_H__

#include <cstdint>
#include <string>
#include <vector>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

namespace cvc {
namespace gl {

class Node;
class RenderView;

// ---------------------------------------------------------------------------
// cvc::gl traversal — an explicit render traversal, in the OpenInventor shape.
// ---------------------------------------------------------------------------
// The existing SceneNode/GraphicsNode tree renders by OWNERSHIP: every node
// builds one vtkProp, holds one vtkRenderer*, and pushes its transform down to
// its descendants whenever it changes. That model has two limits this one is
// built to remove:
//
//   * ONE VIEW. A node holds a single renderer pointer, so a graph cannot feed
//     two windows, and nothing can differ per view — not visibility, not draw
//     style, not level of detail.
//   * EAGER CASCADES. Setting a transform walks the whole subtree immediately
//     (and serialises the matrix into the state tree on every call), so posing
//     a root costs time proportional to everything under it, whether or not any
//     of it is visible.
//
// Here, state is carried by the TRAVERSAL instead. An Action walks the graph
// carrying a TraversalState; property nodes modify that state as they are
// visited; shape nodes read it and bring their prop FOR THIS VIEW up to date.
// Nothing is pushed eagerly and nothing is per-view in the graph itself, so one
// graph can be traversed once per view, differently each time.
//
// WHAT IS DELIBERATELY NOT INVENTOR: SoGLRenderAction emits GL as it walks.
// This does not — VTK stays the rasteriser, because VTK owns the GPU volume
// raycasting that VolumeNode is built on, plus culling and depth ordering, and
// re-implementing that to be faithful to Inventor would be a large step
// backwards. RenderAction is therefore a SYNC action: it reconciles each
// shape's per-view vtkProp with the accumulated state, and VTK draws. Every
// other Inventor property holds — ordering, separators, the state stack.
//
// ORDER-DEPENDENT STATE (the Inventor semantic, not parent/child inheritance):
// a property node affects every node visited AFTER it until the enclosing
// Separator pops. So
//
//     Separator
//       Material(red)      <- applies to both shapes below
//       Sphere
//       Transform(t)       <- applies to Cube only, NOT to Sphere
//       Cube
//
// A Group does NOT push state, so a property inside it leaks to the group's
// following siblings; a Separator does push, so it does not. That is the whole
// difference between the two, and it is why Separator is the one you reach for
// by default.

// ── state elements ─────────────────────────────────────────────────────────
// One struct per kind of state the traversal carries. Adding a new kind of
// inheritable property means adding an element and the accessors for it, not
// touching every node.

struct TransformElement {
  // The accumulated object-to-world matrix at this point in the traversal.
  vtkSmartPointer<vtkMatrix4x4> matrix;
  TransformElement();
};

struct MaterialElement {
  double color[3] = {0.8, 0.8, 0.8};
  double opacity = 1.0;
  double specular = 0.0;
  double specularPower = 10.0;
  double ambient = 0.15;
  double diffuse = 0.85;
  bool overrideVertexColors = false; // when false a mesh's own colours win
};

enum class DrawStyle { Filled, Lines, Points, Invisible };

struct DrawStyleElement {
  DrawStyle style = DrawStyle::Filled;
  double lineWidth = 1.0;
  double pointSize = 1.0;
};

// Per-view selective visibility, as Inventor's SoSwitch/mask idea: a shape is
// drawn in a view only if the view's mask and the shape's mask intersect. This
// is the cheap way to say "annotations in the overview window only".
struct VisibilityElement {
  std::uint32_t mask = 0xFFFFFFFFu;
};

// One frame of the state stack. Separator pushes a copy and pops it; Group does
// not, which is what makes state order-dependent within a group.
struct StateFrame {
  TransformElement transform;
  MaterialElement material;
  DrawStyleElement drawStyle;
  VisibilityElement visibility;
};

// ── the traversal state ────────────────────────────────────────────────────
class TraversalState {
public:
  TraversalState();

  void push(); // Separator entry
  void pop();  // Separator exit
  std::size_t depth() const { return m_stack.size(); }

  StateFrame &top();
  const StateFrame &top() const;

  // Post-multiply the current matrix (the Inventor convention: a Transform
  // node composes onto whatever is already accumulated).
  void multiplyTransform(vtkMatrix4x4 *m);
  vtkMatrix4x4 *transform() { return top().transform.matrix; }

  const MaterialElement &material() const { return top().material; }
  MaterialElement &material() { return top().material; }
  const DrawStyleElement &drawStyle() const { return top().drawStyle; }
  DrawStyleElement &drawStyle() { return top().drawStyle; }
  std::uint32_t visibilityMask() const { return top().visibility.mask; }
  void setVisibilityMask(std::uint32_t m) { top().visibility.mask = m; }

private:
  std::vector<StateFrame> m_stack;
};

// ── actions ────────────────────────────────────────────────────────────────
// An Action is a traversal with a purpose. Nodes do not know about actions
// beyond their type, which is what lets a second action (bounds, picking,
// search) reuse the same graph without every node growing a second method.
class Action {
public:
  enum class Kind { Render, BoundingBox, Custom };

  explicit Action(Kind kind) : m_kind(kind) {}
  virtual ~Action() = default;

  Action(const Action &) = delete;
  Action &operator=(const Action &) = delete;

  Kind kind() const { return m_kind; }
  TraversalState &state() { return m_state; }
  const TraversalState &state() const { return m_state; }

  // Walk `root`. Resets the state first, so an action is re-appliable.
  void apply(Node &root);

  // Count of shape nodes the last apply() actually reached (skipped ones —
  // invisible, masked out — are not counted). Cheap traversal telemetry.
  std::size_t visitedShapes() const { return m_visitedShapes; }
  void countShape() { ++m_visitedShapes; }

private:
  Kind m_kind;
  TraversalState m_state;
  std::size_t m_visitedShapes = 0;
};

// Reconciles each visited shape's prop FOR ONE VIEW with the accumulated state.
class RenderAction : public Action {
public:
  explicit RenderAction(RenderView &view) : Action(Kind::Render), m_view(view) {}
  RenderView &view() { return m_view; }

private:
  RenderView &m_view;
};

// Accumulates a world-space bounding box over the shapes it reaches. Proof that
// the action abstraction is not render-only, and what a "frame the scene" needs
// once bounds are no longer cached eagerly on every node.
class BoundingBoxAction : public Action {
public:
  BoundingBoxAction() : Action(Kind::BoundingBox) {}

  void extend(const double bounds[6], vtkMatrix4x4 *toWorld);
  bool empty() const { return m_empty; }
  // (minx, miny, minz, maxx, maxy, maxz); all zero when empty.
  const double *bounds() const { return m_bounds; }

private:
  double m_bounds[6] = {0, 0, 0, 0, 0, 0};
  bool m_empty = true;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_TRAVERSAL_H__
