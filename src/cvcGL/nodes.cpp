/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/RenderView.h>
#include <cvc/gl/nodes.h>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>

namespace cvc {
namespace gl {

// ── Group ──────────────────────────────────────────────────────────────────
void Group::addChild(std::shared_ptr<Node> child) {
  if (child)
    m_children.push_back(std::move(child));
}

void Group::insertChild(std::size_t index, std::shared_ptr<Node> child) {
  if (!child)
    return;
  index = std::min(index, m_children.size());
  m_children.insert(m_children.begin() + static_cast<std::ptrdiff_t>(index), std::move(child));
}

bool Group::removeChild(const Node *child) {
  auto it = std::find_if(m_children.begin(), m_children.end(),
                         [child](const std::shared_ptr<Node> &c) { return c.get() == child; });
  if (it == m_children.end())
    return false;
  m_children.erase(it);
  return true;
}

void Group::removeAllChildren() { m_children.clear(); }

std::shared_ptr<Node> Group::findChild(const std::string &name) const {
  for (const auto &c : m_children)
    if (c && c->name() == name)
      return c;
  return nullptr;
}

void Group::traverseChildren(Action &action) {
  // By index, not by iterator: a shape's traversal can legitimately mutate the
  // graph (a LOD swap, a lazily built subtree), and an invalidated iterator
  // would be a crash rather than a visible mistake.
  for (std::size_t i = 0; i < m_children.size(); ++i) {
    if (auto child = m_children[i])
      child->traverse(action);
  }
}

// No push: state set inside a Group escapes to whatever follows it. Inventor's
// SoGroup, and the reason Separator exists.
void Group::traverse(Action &action) { traverseChildren(action); }

// ── Separator ──────────────────────────────────────────────────────────────
void Separator::traverse(Action &action) {
  action.state().push();
  try {
    traverseChildren(action);
  } catch (...) {
    action.state().pop(); // never leave the stack unbalanced
    throw;
  }
  action.state().pop();
}

// ── Switch ─────────────────────────────────────────────────────────────────
void Switch::traverse(Action &action) {
  if (m_which == None)
    return;
  if (m_which == All) {
    traverseChildren(action);
    return;
  }
  if (m_which >= 0 && static_cast<std::size_t>(m_which) < numChildren()) {
    if (auto child = children()[static_cast<std::size_t>(m_which)])
      child->traverse(action);
  }
}

// ── Transform ──────────────────────────────────────────────────────────────
void Transform::setMatrix(const double m[16]) {
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      m_matrix->SetElement(i, j, m[i * 4 + j]);
}

void Transform::setMatrix(vtkMatrix4x4 *m) {
  if (m)
    m_matrix->DeepCopy(m);
}

void Transform::identity() { m_matrix->Identity(); }

void Transform::setTranslation(double x, double y, double z) {
  m_matrix->Identity();
  m_matrix->SetElement(0, 3, x);
  m_matrix->SetElement(1, 3, y);
  m_matrix->SetElement(2, 3, z);
}

void Transform::setScale(double s) {
  m_matrix->Identity();
  m_matrix->SetElement(0, 0, s);
  m_matrix->SetElement(1, 1, s);
  m_matrix->SetElement(2, 2, s);
}

void Transform::setRotation(double angle, double ax, double ay, double az) {
  const double n = std::sqrt(ax * ax + ay * ay + az * az);
  m_matrix->Identity();
  if (n <= 0.0)
    return;
  ax /= n;
  ay /= n;
  az /= n;
  const double c = std::cos(angle), s = std::sin(angle), k = 1.0 - c;
  m_matrix->SetElement(0, 0, c + k * ax * ax);
  m_matrix->SetElement(0, 1, k * ax * ay - s * az);
  m_matrix->SetElement(0, 2, k * ax * az + s * ay);
  m_matrix->SetElement(1, 0, k * ax * ay + s * az);
  m_matrix->SetElement(1, 1, c + k * ay * ay);
  m_matrix->SetElement(1, 2, k * ay * az - s * ax);
  m_matrix->SetElement(2, 0, k * ax * az - s * ay);
  m_matrix->SetElement(2, 1, k * ay * az + s * ax);
  m_matrix->SetElement(2, 2, c + k * az * az);
}

void Transform::traverse(Action &action) { action.state().multiplyTransform(m_matrix); }

// ── Material / DrawStyle / VisibilityMask ──────────────────────────────────
void Material::setColor(double r, double g, double b) {
  m_element.color[0] = r;
  m_element.color[1] = g;
  m_element.color[2] = b;
}
void Material::setOpacity(double a) { m_element.opacity = a; }
void Material::setSpecular(double v, double power) {
  m_element.specular = v;
  m_element.specularPower = power;
}
void Material::traverse(Action &action) { action.state().material() = m_element; }

void DrawStyleNode::traverse(Action &action) { action.state().drawStyle() = m_element; }

void VisibilityMask::traverse(Action &action) {
  // Intersect rather than replace, so nesting narrows and never re-widens.
  action.state().setVisibilityMask(action.state().visibilityMask() & m_mask);
}

// ── Shape ──────────────────────────────────────────────────────────────────
Shape::~Shape() {
  for (auto &kv : m_props) {
    if (kv.first) {
      kv.first->detachProp(kv.second);
      kv.first->untrackShape(this);
    }
  }
}

void Shape::releaseView(RenderView *view) {
  auto it = m_props.find(view);
  if (it == m_props.end())
    return;
  if (view)
    view->detachProp(it->second);
  m_props.erase(it);
}

vtkProp *Shape::propFor(RenderView &view) {
  auto it = m_props.find(&view);
  if (it != m_props.end())
    return it->second;
  auto prop = createProp(view);
  if (!prop)
    return nullptr;
  m_props[&view] = prop;
  view.attachProp(prop);
  view.trackShape(this); // so the view can make us let go when it dies
  return prop;
}

void Shape::applyState(vtkProp *, const StateFrame &) {}

void Shape::traverse(Action &action) {
  const StateFrame &frame = action.state().top();

  if (action.kind() == Action::Kind::BoundingBox) {
    double b[6];
    if (localBounds(b)) {
      action.countShape();
      static_cast<BoundingBoxAction &>(action).extend(b, frame.transform.matrix);
    }
    return;
  }

  if (action.kind() != Action::Kind::Render)
    return;

  auto &render = static_cast<RenderAction &>(action);
  RenderView &view = render.view();

  // Per-view culling, decided by the traversal rather than by the graph: the
  // same shape can be drawn in one window and skipped in another.
  const bool visible = frame.drawStyle.style != DrawStyle::Invisible &&
                       (frame.visibility.mask & view.visibilityMask()) != 0u;

  vtkProp *prop = propFor(view);
  if (!prop)
    return;
  if (!visible) {
    prop->SetVisibility(0);
    return;
  }
  prop->SetVisibility(1);
  action.countShape();
  applyState(prop, frame);
}

// ── GeometryShape ──────────────────────────────────────────────────────────
void GeometryShape::setGeometry(const cvc::geometry &geom) {
  auto poly = vtkSmartPointer<vtkPolyData>::New();

  auto points = vtkSmartPointer<vtkPoints>::New();
  const auto &pts = geom.const_points();
  points->SetNumberOfPoints(static_cast<vtkIdType>(pts.size()));
  for (std::size_t i = 0; i < pts.size(); ++i)
    points->SetPoint(static_cast<vtkIdType>(i), pts[i][0], pts[i][1], pts[i][2]);
  poly->SetPoints(points);

  const auto &tris = geom.const_tris();
  if (!tris.empty()) {
    auto cells = vtkSmartPointer<vtkCellArray>::New();
    for (const auto &t : tris) {
      vtkIdType ids[3] = {static_cast<vtkIdType>(t[0]), static_cast<vtkIdType>(t[1]),
                          static_cast<vtkIdType>(t[2])};
      cells->InsertNextCell(3, ids);
    }
    poly->SetPolys(cells);
  }

  const auto &lines = geom.const_lines();
  if (!lines.empty()) {
    auto cells = vtkSmartPointer<vtkCellArray>::New();
    for (const auto &l : lines) {
      vtkIdType ids[2] = {static_cast<vtkIdType>(l[0]), static_cast<vtkIdType>(l[1])};
      cells->InsertNextCell(2, ids);
    }
    poly->SetLines(cells);
  }

  const auto &colors = geom.const_colors();
  m_hasVertexColors = colors.size() == pts.size() && !colors.empty();
  if (m_hasVertexColors) {
    auto rgb = vtkSmartPointer<vtkUnsignedCharArray>::New();
    rgb->SetNumberOfComponents(3);
    rgb->SetName("Colors");
    rgb->SetNumberOfTuples(static_cast<vtkIdType>(colors.size()));
    for (std::size_t i = 0; i < colors.size(); ++i) {
      unsigned char c[3];
      for (int k = 0; k < 3; ++k) {
        double v = colors[i][k] * 255.0;
        c[k] = static_cast<unsigned char>(v < 0 ? 0 : (v > 255 ? 255 : v));
      }
      rgb->SetTypedTuple(static_cast<vtkIdType>(i), c);
    }
    poly->GetPointData()->SetScalars(rgb);
  }

  m_polyData = poly;

  // Every view's mapper points at the same polydata, so N views cost N actors
  // and one copy of the geometry.
  // (Existing per-view actors pick the change up through the mapper.)
}

vtkSmartPointer<vtkProp> GeometryShape::createProp(RenderView &) {
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  if (m_polyData)
    mapper->SetInputData(m_polyData);
  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  return actor;
}

void GeometryShape::applyState(vtkProp *prop, const StateFrame &frame) {
  auto *actor = vtkActor::SafeDownCast(prop);
  if (!actor)
    return;

  actor->SetUserMatrix(frame.transform.matrix);

  auto *p = actor->GetProperty();
  const auto &m = frame.material;
  p->SetColor(m.color[0], m.color[1], m.color[2]);
  p->SetOpacity(m.opacity);
  p->SetSpecular(m.specular);
  p->SetSpecularPower(m.specularPower);
  p->SetAmbient(m.ambient);
  p->SetDiffuse(m.diffuse);

  // A mesh's own colours win unless the material explicitly overrides them.
  if (auto *mapper = actor->GetMapper())
    mapper->SetScalarVisibility(m_hasVertexColors && !m.overrideVertexColors);

  switch (frame.drawStyle.style) {
  case DrawStyle::Filled:
    p->SetRepresentationToSurface();
    break;
  case DrawStyle::Lines:
    p->SetRepresentationToWireframe();
    break;
  case DrawStyle::Points:
    p->SetRepresentationToPoints();
    break;
  case DrawStyle::Invisible:
    break; // handled by the visibility flag in Shape::traverse
  }
  p->SetLineWidth(frame.drawStyle.lineWidth);
  p->SetPointSize(frame.drawStyle.pointSize);
}

bool GeometryShape::localBounds(double out[6]) const {
  if (!m_polyData || m_polyData->GetNumberOfPoints() == 0)
    return false;
  m_polyData->GetBounds(out);
  return true;
}

} // namespace gl
} // namespace cvc
