/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// MoleculeNode (cvc/gl/MoleculeNode.h): a molecule's view, its spheres as impostors.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/MoleculeNode.h>
#include <cvc/mol/molecule.h>
#include <cvc/mol/view.h>
#include <limits>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkOpenGLSphereMapper.h>
#include <vtkPlaneCollection.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkShaderProgram.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>

namespace cvc {
namespace gl {

// VTK's impostor sphere mapper, with two things added: its bounds take the spheres' radii in (the
// base's are the centres' alone, so the camera's clipping range could cut the outermost spheres),
// and it notes when its shader program failed to build, which is what "impostors can't run here"
// means.
class ImpostorSphereMapper : public vtkOpenGLSphereMapper {
public:
  static ImpostorSphereMapper *New();
  vtkTypeMacro(ImpostorSphereMapper, vtkOpenGLSphereMapper);

  double MaxRadius = 0.0;
  bool Failed = false;

  double *GetBounds() override {
    double *b = this->Superclass::GetBounds();
    if (b && b[0] <= b[1]) {
      for (int i = 0; i < 3; ++i) {
        Padded[2 * i] = b[2 * i] - MaxRadius;
        Padded[2 * i + 1] = b[2 * i + 1] + MaxRadius;
      }
      return Padded;
    }
    return b;
  }
  void GetBounds(double bounds[6]) override {
    const double *b = this->GetBounds();
    for (int i = 0; i < 6; ++i)
      bounds[i] = b[i];
  }

protected:
  ImpostorSphereMapper() = default;
  ~ImpostorSphereMapper() override = default;

  void RenderPieceDraw(vtkRenderer *ren, vtkActor *act) override {
    this->Superclass::RenderPieceDraw(ren, act);
    vtkPolyData *in = this->GetInput();
    // With points to draw, a program that would not compile or link leaves the helper the draw
    // bound with none.
    if (in && in->GetNumberOfPoints() > 0) {
      const vtkOpenGLHelper *bo = this->LastBoundBO;
      if (!bo || !bo->Program || !bo->Program->GetCompiled())
        Failed = true;
    }
  }

private:
  double Padded[6] = {0, 0, 0, 0, 0, 0};
  ImpostorSphereMapper(const ImpostorSphereMapper &) = delete;
  void operator=(const ImpostorSphereMapper &) = delete;
};
vtkStandardNewMacro(ImpostorSphereMapper);

struct MoleculeNode::impostor {
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  vtkSmartPointer<ImpostorSphereMapper> mapper = vtkSmartPointer<ImpostorSphereMapper>::New();
  vtkSmartPointer<vtkPolyData> points = vtkSmartPointer<vtkPolyData>::New();
  std::size_t count = 0;
};

MoleculeNode::MoleculeNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
    : GraphicsNode(ctx, statePath, name), m_imp(std::make_unique<impostor>()) {
  impostor &d = *m_imp;
  d.mapper->SetInputData(d.points);
  d.mapper->SetScaleArray("radius");
  d.mapper->SetScalarModeToUsePointData();
  d.mapper->SetColorModeToDirectScalars();
  d.mapper->ScalarVisibilityOn();
  d.actor->SetMapper(d.mapper);
  // Picked exactly by pickRay: the cell picker would only meet the spheres' centres.
  d.actor->PickableOff();
  vtkProperty *p = d.actor->GetProperty();
  p->SetAmbient(m_ambient);
  p->SetDiffuse(m_diffuse);
  p->SetSpecular(m_specular);
  p->SetSpecularPower(m_specularPower);
  // The shadow pass draws casters with its own depth shader, which knows nothing of impostors.
  setCastsShadow(false);
}

MoleculeNode::~MoleculeNode() = default;

vtkProp *MoleculeNode::getProp() { return m_imp->actor; }
vtkActor *MoleculeNode::impostorActor() const { return m_imp->actor; }
std::size_t MoleculeNode::impostorCount() const { return m_imp->count; }
bool MoleculeNode::impostorsFailed() const { return m_imp->mapper->Failed; }

void MoleculeNode::applyTransformToVTK() { applyWorldTransformToProps({m_imp->actor}); }

void MoleculeNode::applyClipPlanes(vtkPlaneCollection *planes) {
  m_imp->mapper->SetClippingPlanes(planes);
}

void MoleculeNode::ensureParts() {
  if (m_sticks)
    return;
  const auto part = [this](const char *suffix) {
    std::shared_ptr<GeometryNode> g =
        createChild<GeometryNode>(getName() + suffix, cvc::geometry());
    if (g) {
      g->setUseSingleColor(false);
      g->setAmbient(m_ambient);
      g->setDiffuse(m_diffuse);
      g->setSpecular(m_specular);
      g->setSpecularPower(m_specularPower);
      g->setVisible(false);
    }
    return g;
  };
  m_spheresMesh = part("_spheres");
  m_sticks = part("_sticks");
  m_cartoon = part("_cartoon");
}

void MoleculeNode::setLighting(double ambient, double diffuse, double specular,
                               double specularPower) {
  m_ambient = ambient;
  m_diffuse = diffuse;
  m_specular = specular;
  m_specularPower = specularPower;
  vtkProperty *p = m_imp->actor->GetProperty();
  p->SetAmbient(ambient);
  p->SetDiffuse(diffuse);
  p->SetSpecular(specular);
  p->SetSpecularPower(specularPower);
  for (const std::shared_ptr<GeometryNode> &g : {m_spheresMesh, m_sticks, m_cartoon})
    if (g) {
      g->setAmbient(ambient);
      g->setDiffuse(diffuse);
      g->setSpecular(specular);
      g->setSpecularPower(specularPower);
    }
}

void MoleculeNode::setView(std::shared_ptr<const cvc::mol::molecule> mol,
                           std::shared_ptr<const cvc::mol::molecule_view> view) {
  ensureParts();
  m_mol = std::move(mol);
  m_view = std::move(view);
  impostor &d = *m_imp;
  // The impostors' points: a centre, a radius and a colour each.
  vtkNew<vtkPoints> pts;
  vtkNew<vtkFloatArray> radius;
  radius->SetName("radius");
  vtkNew<vtkUnsignedCharArray> colors;
  colors->SetName("colors");
  colors->SetNumberOfComponents(3);
  double largest = 0.0;
  const std::size_t n = m_view ? m_view->radii.size() : 0;
  pts->SetDataTypeToFloat();
  pts->SetNumberOfPoints(static_cast<vtkIdType>(n));
  radius->SetNumberOfTuples(static_cast<vtkIdType>(n));
  colors->SetNumberOfTuples(static_cast<vtkIdType>(n));
  for (std::size_t i = 0; i < n; ++i) {
    const vtkIdType id = static_cast<vtkIdType>(i);
    pts->SetPoint(id, m_view->centers[3 * i], m_view->centers[3 * i + 1],
                  m_view->centers[3 * i + 2]);
    radius->SetValue(id, m_view->radii[i]);
    const cvc::mol::rgb8 &c = m_view->colors[i];
    colors->SetTypedTuple(id, c.data());
    largest = std::max(largest, static_cast<double>(m_view->radii[i]));
  }
  // A vertex cell a sphere: VTK's mapper takes its bounds from the cells, and a prop with none is
  // culled.
  vtkNew<vtkCellArray> verts;
  verts->AllocateExact(static_cast<vtkIdType>(n), static_cast<vtkIdType>(n));
  for (std::size_t i = 0; i < n; ++i) {
    const vtkIdType id = static_cast<vtkIdType>(i);
    verts->InsertNextCell(1, &id);
  }
  vtkNew<vtkPolyData> poly;
  poly->SetPoints(pts);
  poly->SetVerts(verts);
  poly->GetPointData()->AddArray(radius);
  poly->GetPointData()->SetScalars(colors);
  d.points = poly;
  d.mapper->SetInputData(d.points);
  d.mapper->MaxRadius = largest;
  d.mapper->Modified();
  d.count = n;
  if (m_view) {
    m_sticks->setGeometry(m_view->sticks);
    m_cartoon->setGeometry(m_view->cartoon);
    m_spheresMesh->setGeometry(m_view->spheres);
  } else {
    m_sticks->setGeometry(cvc::geometry());
    m_cartoon->setGeometry(cvc::geometry());
    m_spheresMesh->setGeometry(cvc::geometry());
  }
  showParts();
  ownBoundsChanged();
}

void MoleculeNode::setSpherePath(SpherePath path) {
  if (path == m_path)
    return;
  m_path = path;
  ensureParts();
  showParts();
}

void MoleculeNode::showParts() {
  const bool spheres = m_view && !m_view->radii.empty();
  m_imp->actor->SetVisibility(spheres && m_path == SpherePath::Impostor ? 1 : 0);
  if (m_spheresMesh)
    m_spheresMesh->setVisible(spheres && m_path == SpherePath::Mesh && m_view &&
                              !m_view->spheres.points().empty());
  if (m_sticks)
    m_sticks->setVisible(m_view && !m_view->sticks.points().empty());
  if (m_cartoon)
    m_cartoon->setVisible(m_view && !m_view->cartoon.points().empty());
}

cvc::bounding_box MoleculeNode::getBoundingBox() const {
  if (!m_view || (m_view->bounds[0] == 0 && m_view->bounds[3] == 0 && m_view->bounds[1] == 0 &&
                  m_view->bounds[4] == 0 && m_view->bounds[2] == 0 && m_view->bounds[5] == 0))
    return cvc::bounding_box();
  const auto &b = m_view->bounds;
  return cvc::bounding_box(b[0], b[1], b[2], b[3], b[4], b[5]);
}

int MoleculeNode::pickRay(const double origin[3], const double direction[3], double *world,
                          double *t) const {
  const bool shown = m_imp->actor->GetVisibility() || (m_spheresMesh && m_spheresMesh->isVisible());
  if (!m_view || m_view->radii.empty() || !isVisibleInHierarchy() || !shown)
    return -1;
  // The ray in the node's own frame: its origin, and a second point along it.
  double o[3], q[3],
      far_[3] = {origin[0] + direction[0], origin[1] + direction[1], origin[2] + direction[2]};
  worldToLocal(origin, o);
  worldToLocal(far_, q);
  const double d[3] = {q[0] - o[0], q[1] - o[1], q[2] - o[2]};
  double local_hit[3];
  const int atom = cvc::mol::pick_sphere(*m_view, o, d, nullptr, local_hit);
  if (atom < 0)
    return -1;
  double w[3];
  localToWorld(local_hit, w);
  if (world)
    std::copy(w, w + 3, world);
  if (t)
    *t = std::sqrt((w[0] - origin[0]) * (w[0] - origin[0]) +
                   (w[1] - origin[1]) * (w[1] - origin[1]) +
                   (w[2] - origin[2]) * (w[2] - origin[2]));
  return atom;
}

int MoleculeNode::atomNear(const double world[3]) const {
  if (!m_view || !m_mol)
    return -1;
  double local[3];
  worldToLocal(world, local);
  return cvc::mol::nearest_drawn_atom(*m_mol, *m_view, local);
}

std::vector<std::pair<std::string, std::string>> MoleculeNode::atomFields(int atom) const {
  if (!m_mol || atom < 0 || static_cast<std::size_t>(atom) >= m_mol->atoms.size())
    return {};
  const cvc::mol::atom &a = m_mol->atoms[static_cast<std::size_t>(atom)];
  char b[64];
  std::snprintf(b, sizeof b, "%.6g", a.b_factor);
  std::string structure;
  const int s = m_mol->structure_of(a);
  if (s >= 0)
    switch (m_mol->structures[static_cast<std::size_t>(s)].type) {
    case cvc::mol::secondary_structure::kind::helix:
      structure = "helix";
      break;
    case cvc::mol::secondary_structure::kind::sheet:
      structure = "sheet";
      break;
    case cvc::mol::secondary_structure::kind::turn:
      structure = "turn";
      break;
    }
  return {{"atom", std::to_string(atom)},
          {"serial", std::to_string(a.serial)},
          {"name", a.name},
          {"element", a.element},
          {"residue", a.residue},
          {"residue_seq", std::to_string(a.residue_seq)},
          {"chain", std::string(1, a.chain)},
          {"model", std::to_string(a.model)},
          {"b_factor", b},
          {"hetero", a.hetero ? "1" : "0"},
          {"structure", structure}};
}

} // namespace gl
} // namespace cvc
