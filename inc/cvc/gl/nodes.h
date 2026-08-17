/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_NODES_H__
#define __CVC_GL_NODES_H__

#include <cvc/gl/traversal.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

class vtkProp;
class vtkActor;
class vtkPolyData;

namespace cvc {
class geometry;
class volume;
} // namespace cvc

namespace cvc {
namespace gl {

class RenderView;

// ---------------------------------------------------------------------------
// The node taxonomy (see traversal.h for the model)
// ---------------------------------------------------------------------------
// Split along Inventor's line, which is the split the existing GraphicsNode
// tree does not have: PROPERTY nodes carry state and draw nothing, SHAPE nodes
// draw and carry no state, GROUP nodes hold children. That separation is what
// makes order-dependent state expressible at all — with transform and geometry
// fused into one object, "this transform applies to the next three shapes" has
// nowhere to live.

class Node {
public:
  explicit Node(std::string name = {}) : m_name(std::move(name)) {}
  virtual ~Node() = default;

  Node(const Node &) = delete;
  Node &operator=(const Node &) = delete;

  const std::string &name() const { return m_name; }
  void setName(std::string n) { m_name = std::move(n); }

  // Visit this node under `action`. The one method every action goes through.
  virtual void traverse(Action &action) = 0;

private:
  std::string m_name;
};

// ── group nodes ────────────────────────────────────────────────────────────

// Children in order, state NOT pushed: a property node inside a Group leaks
// out to whatever follows the Group. That is Inventor's SoGroup, and it is the
// sharp edge of order-dependent state — reach for Separator unless you want
// the leak.
class Group : public Node {
public:
  using Node::Node;

  void addChild(std::shared_ptr<Node> child);
  void insertChild(std::size_t index, std::shared_ptr<Node> child);
  bool removeChild(const Node *child);
  void removeAllChildren();

  std::size_t numChildren() const { return m_children.size(); }
  const std::vector<std::shared_ptr<Node>> &children() const { return m_children; }
  std::shared_ptr<Node> findChild(const std::string &name) const;

  void traverse(Action &action) override;

protected:
  // Traverse children in order without touching the state stack.
  void traverseChildren(Action &action);

private:
  std::vector<std::shared_ptr<Node>> m_children;
};

// Children in order with the state PUSHED around them, so nothing inside
// escapes. The default container.
class Separator : public Group {
public:
  using Group::Group;
  void traverse(Action &action) override;
};

// ── property nodes ─────────────────────────────────────────────────────────

// Composes onto the current matrix; affects everything visited after it, until
// the enclosing Separator pops.
class Transform : public Node {
public:
  using Node::Node;

  void setMatrix(const double m[16]); // row-major
  void setMatrix(vtkMatrix4x4 *m);
  void setTranslation(double x, double y, double z);
  void setScale(double s);
  void setRotation(double angleRadians, double ax, double ay, double az);
  void identity();

  vtkMatrix4x4 *matrix() { return m_matrix; }
  void traverse(Action &action) override;

private:
  vtkSmartPointer<vtkMatrix4x4> m_matrix = [] {
    auto m = vtkSmartPointer<vtkMatrix4x4>::New();
    m->Identity();
    return m;
  }();
};

class Material : public Node {
public:
  using Node::Node;

  void setColor(double r, double g, double b);
  void setOpacity(double a);
  void setSpecular(double v, double power);
  // When true this material overrides a mesh's per-vertex colours.
  void setOverrideVertexColors(bool on) { m_element.overrideVertexColors = on; }
  const MaterialElement &element() const { return m_element; }

  void traverse(Action &action) override;

private:
  MaterialElement m_element;
};

class DrawStyleNode : public Node {
public:
  using Node::Node;

  void setStyle(DrawStyle s) { m_element.style = s; }
  void setLineWidth(double w) { m_element.lineWidth = w; }
  void setPointSize(double p) { m_element.pointSize = p; }
  const DrawStyleElement &element() const { return m_element; }

  void traverse(Action &action) override;

private:
  DrawStyleElement m_element;
};

// Restricts everything visited after it to views whose mask intersects this
// one — "annotations only in the overview window", per view, no graph edits.
class VisibilityMask : public Node {
public:
  using Node::Node;
  void setMask(std::uint32_t m) { m_mask = m; }
  std::uint32_t mask() const { return m_mask; }
  void traverse(Action &action) override;

private:
  std::uint32_t m_mask = 0xFFFFFFFFu;
};

// Inventor's SoSwitch: traverse one child, or none (-1), or all (-3).
class Switch : public Group {
public:
  static constexpr int None = -1;
  static constexpr int All = -3;

  using Group::Group;
  void setWhichChild(int which) { m_which = which; }
  int whichChild() const { return m_which; }
  void traverse(Action &action) override;

private:
  int m_which = All;
};

// ── shape nodes ────────────────────────────────────────────────────────────

// A drawable. Holds NO renderer and NO single prop: it keeps one prop PER VIEW,
// created on demand the first time a RenderAction for that view reaches it.
// That map is what lets one graph feed N windows, and what lets a shape be
// filled in one view and wireframe in another.
class Shape : public Node {
public:
  using Node::Node;
  ~Shape() override;

  void traverse(Action &action) override;

  // Drop this shape's prop for `view` (called when a view goes away).
  void releaseView(RenderView *view);
  std::size_t numViewInstances() const { return m_props.size(); }

protected:
  // Build the prop for a view. Called once per view, lazily.
  virtual vtkSmartPointer<vtkProp> createProp(RenderView &view) = 0;
  // Apply the accumulated state to this view's prop.
  virtual void applyState(vtkProp *prop, const StateFrame &frame);
  // Object-space bounds, for BoundingBoxAction. Return false if unknown.
  virtual bool localBounds(double out[6]) const = 0;

  vtkProp *propFor(RenderView &view);

private:
  std::map<RenderView *, vtkSmartPointer<vtkProp>> m_props;
};

// A triangle/line mesh. Owns its polydata once; every view's actor shares the
// same mapper input, so N views cost N actors, not N copies of the geometry.
class GeometryShape : public Shape {
public:
  using Shape::Shape;

  void setGeometry(const cvc::geometry &geom);
  bool hasGeometry() const { return m_polyData != nullptr; }

protected:
  vtkSmartPointer<vtkProp> createProp(RenderView &view) override;
  void applyState(vtkProp *prop, const StateFrame &frame) override;
  bool localBounds(double out[6]) const override;

private:
  vtkSmartPointer<vtkPolyData> m_polyData;
  bool m_hasVertexColors = false;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_NODES_H__
