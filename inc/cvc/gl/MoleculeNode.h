/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef CVC_GL_MOLECULE_NODE_H
#define CVC_GL_MOLECULE_NODE_H

// MoleculeNode: a molecule drawn from a cvc::mol::molecule_view (cvc/mol/view.h), the view the
// `molecule` scene node type builds (cvc/gl/ariadne/molecule_nodes.h).
//
// What it draws:
//   - the view's spheres as IMPOSTORS: each a screen-facing quad whose fragments are shaded and
//     depth-written as the sphere's surface (VTK's vtkOpenGLSphereMapper), so a protein's
//     thousands of atoms cost one point each. This is the node's own actor;
//   - or, where impostors can't run (their shader would not build in this GL context, or the
//     document asks for it), the same spheres TESSELLATED (the view's `spheres` mesh), in a child;
//   - the bonds' cylinders (`sticks`) and the cartoon's tube (`cartoon`), each in a child
//     GeometryNode with per-vertex colours.
//
// The parts move, hide and clip with the node. The node keeps the molecule and the view it shows,
// shared and read-only: one copy, whoever else holds it.
//
// PICKING. pickRay() meets the spheres exactly (a ray against each sphere, in the node's own
// frame), which the scene's cell picker can't do for impostors, and atomNear() names the atom a
// pick on the sticks or the cartoon landed nearest. The scene's pick (scene_views.h) uses both,
// and reports the atom's fields (atomFields) with the hit.
//
// Owner thread, like the scene graph.

#include <cvc/gl/GraphicsNode.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class vtkActor;

namespace cvc {
namespace mol {
struct molecule;
struct molecule_view;
} // namespace mol
namespace gl {

class GeometryNode;
class ImpostorSphereMapper;

class MoleculeNode : public GraphicsNode {
public:
  enum class SpherePath { Impostor, Mesh };

  MoleculeNode(cvc::app &ctx, const std::string &statePath, const std::string &name = "molecule");
  ~MoleculeNode() override;

  // Show `view` of `mol` (both null: nothing). A view built with sphere_mesh carries the
  // tessellated spheres the Mesh path draws; without them the Mesh path draws no spheres.
  void setView(std::shared_ptr<const cvc::mol::molecule> mol,
               std::shared_ptr<const cvc::mol::molecule_view> view);
  const std::shared_ptr<const cvc::mol::molecule> &molecule() const { return m_mol; }
  const std::shared_ptr<const cvc::mol::molecule_view> &view() const { return m_view; }

  // Which way the spheres are drawn.
  void setSpherePath(SpherePath path);
  SpherePath spherePath() const { return m_path; }
  // Whether a draw found that impostors can't run here: their shader program failed to build.
  // The node then draws nothing for them until it is set to the Mesh path.
  bool impostorsFailed() const;

  // Material: ambient, diffuse and specular for every part.
  void setLighting(double ambient, double diffuse, double specular, double specularPower);

  // The atom a WORLD ray (origin, direction) meets first among the spheres shown, or -1. `world`
  // (three doubles) is where, and `t` the distance from the origin, when not null.
  int pickRay(const double origin[3], const double direction[3], double *world = nullptr,
              double *t = nullptr) const;
  // The drawn atom nearest a WORLD point, or -1 when nothing is drawn.
  int atomNear(const double world[3]) const;
  // What a pick on atom `atom` reports: atom (its index), serial, name, element, residue,
  // residue_seq, chain, model, b_factor, hetero and structure (helix, sheet, turn or "").
  std::vector<std::pair<std::string, std::string>> atomFields(int atom) const;

  // The parts, for tests and for the views' tags (null until the first setView).
  std::shared_ptr<GeometryNode> spheresMesh() const { return m_spheresMesh; }
  std::shared_ptr<GeometryNode> sticks() const { return m_sticks; }
  std::shared_ptr<GeometryNode> cartoon() const { return m_cartoon; }
  // The impostor actor (the node's own prop), and how many spheres it holds.
  vtkActor *impostorActor() const;
  std::size_t impostorCount() const;

  cvc::bounding_box getBoundingBox() const override;
  int maxClipPlanes() const override { return 6; }

protected:
  vtkProp *getProp() override;
  void applyTransformToVTK() override;
  void applyClipPlanes(vtkPlaneCollection *planes) override;

private:
  void ensureParts();
  void showParts();

  std::shared_ptr<const cvc::mol::molecule> m_mol;
  std::shared_ptr<const cvc::mol::molecule_view> m_view;
  SpherePath m_path = SpherePath::Impostor;
  struct impostor;
  std::unique_ptr<impostor> m_imp;
  std::shared_ptr<GeometryNode> m_spheresMesh, m_sticks, m_cartoon;
  double m_ambient = 0.25, m_diffuse = 0.8, m_specular = 0.25, m_specularPower = 24.0;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_MOLECULE_NODE_H
