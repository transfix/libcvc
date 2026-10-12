/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// cvcgl_tet_surface -- GeometryNode's TETS render mode. With libigl
// (cvc::mesh_ops_available()) a tet mesh draws as its boundary surface: on an
// n^3 cube of tets that is 12 n^2 outward-facing triangles on the cube's faces,
// over ALL the mesh's points so a per-vertex field still lines up, surviving
// rebuilds; tets the boundary extraction rejects fall back to the edges. A
// curvature color_by on a tet mesh is the boundary's; on bare points, none.
// Without libigl the mode keeps drawing the 6 edges of every tet. Headless:
// the polydata is read through SceneNode::prop() -> vtkActor -> mapper input.
// Explicit checks + non-zero return (assert() is a no-op under Release).

#include <array>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/mesh_ops.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkDataArray.h>
#include <vtkIdList.h>
#include <vtkMapper.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>

using cvc::gl::GeometryNode;
using cvc::gl::GeometryRenderMode;
using cvc::gl::SceneGraph;

namespace {
int g_fail = 0;
void check(bool ok, const char *msg) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
  if (!ok)
    ++g_fail;
}

// The cube [0,n]^3 split into n^3 unit cubes of 6 tets each (Kuhn: one tet per
// axis order, all sharing the cube's main diagonal, so neighbouring cubes'
// faces are split the same way). (n+1)^3 points, 6 n^3 tets of mixed
// orientation; every boundary square is two triangles: 12 n^2 in all.
cvc::geometry tetCube(cvc::app &app, int n) {
  cvc::geometry g(app);
  g.set_geometry_type(cvc::geometry::VOLUME_TET);
  const int m = n + 1;
  auto id = [m](int i, int j, int k) {
    return static_cast<cvc::geometry::index_t>(i + m * (j + m * k));
  };
  auto &p = g.points();
  for (int k = 0; k < m; ++k)
    for (int j = 0; j < m; ++j)
      for (int i = 0; i < m; ++i)
        p.push_back({double(i), double(j), double(k)});
  static const int orders[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2},
                                   {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  auto &t = g.tets();
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i)
        for (const auto &o : orders) {
          int c[3] = {i, j, k};
          cvc::geometry::tet_t tet;
          tet[0] = id(c[0], c[1], c[2]);
          for (int s = 0; s < 3; ++s) {
            ++c[o[s]];
            tet[s + 1] = id(c[0], c[1], c[2]);
          }
          t.push_back(tet);
        }
  return g;
}

vtkActor *actorOf(GeometryNode &n) { return vtkActor::SafeDownCast(n.prop()); }

vtkPolyData *polyOf(GeometryNode &n) {
  vtkActor *a = actorOf(n);
  return a ? vtkPolyData::SafeDownCast(a->GetMapper()->GetInput()) : nullptr;
}

// Every polygon is a triangle on a face of the cube [0,n]^3 whose winding
// faces out of the cube.
bool outwardOnCubeFaces(vtkPolyData *pd, int n) {
  const double c = 0.5 * n;
  vtkCellArray *polys = pd->GetPolys();
  vtkSmartPointer<vtkIdList> ids = vtkSmartPointer<vtkIdList>::New();
  polys->InitTraversal();
  while (polys->GetNextCell(ids)) {
    if (ids->GetNumberOfIds() != 3)
      return false;
    double a[3], b[3], d[3];
    pd->GetPoint(ids->GetId(0), a);
    pd->GetPoint(ids->GetId(1), b);
    pd->GetPoint(ids->GetId(2), d);
    bool onFace = false;
    for (int ax = 0; ax < 3; ++ax)
      for (double f : {0.0, double(n)})
        onFace = onFace || (a[ax] == f && b[ax] == f && d[ax] == f);
    const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const double v[3] = {d[0] - a[0], d[1] - a[1], d[2] - a[2]};
    const double nrm[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                           u[0] * v[1] - u[1] * v[0]};
    const double out[3] = {(a[0] + b[0] + d[0]) / 3 - c, (a[1] + b[1] + d[1]) / 3 - c,
                           (a[2] + b[2] + d[2]) / 3 - c};
    if (!onFace || nrm[0] * out[0] + nrm[1] * out[1] + nrm[2] * out[2] <= 0)
      return false;
  }
  return true;
}

int representation(GeometryNode &n) {
  vtkActor *a = actorOf(n);
  return a ? a->GetProperty()->GetRepresentation() : -1;
}

void testTetCube(cvc::app &app, int n) {
  std::printf("TETS mode on a %d^3 tet cube (libigl %s)\n", n,
              cvc::mesh_ops_available() ? "available" : "unavailable");
  SceneGraph sg(app, "tettest");
  const cvc::geometry g = tetCube(app, n);
  const vtkIdType numPts = (n + 1) * (n + 1) * (n + 1), numTets = 6 * n * n * n;
  auto node = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("cube", g));
  vtkPolyData *pd = node ? polyOf(*node) : nullptr;
  if (!pd) {
    check(false, "the tet cube is a GeometryNode with a polydata");
    return;
  }
  check(node->getRenderMode() == GeometryRenderMode::TETS, "a VOLUME_TET mesh picks TETS mode");
  check(pd->GetNumberOfPoints() == numPts, "the polydata keeps every point of the tet mesh");

  if (cvc::mesh_ops_available()) {
    check(pd->GetNumberOfPolys() == 12 * n * n && pd->GetNumberOfLines() == 0,
          "TETS draws the boundary surface: 12 n^2 triangles, no edges");
    check(outwardOnCubeFaces(pd, n), "every triangle is on a cube face and faces outward");
    check(representation(*node) == VTK_SURFACE, "the boundary draws as a surface");
    vtkDataArray *normals = pd->GetPointData()->GetNormals();
    check(normals && normals->GetNumberOfTuples() == numPts, "point normals for every point");
  } else {
    check(pd->GetNumberOfLines() == 6 * numTets && pd->GetNumberOfPolys() == 0,
          "without libigl TETS draws the 6 edges of every tet");
    check(representation(*node) == VTK_WIREFRAME, "the edge fallback draws as a wireframe");
  }

  // A per-vertex field over the tet mesh's points maps 1:1 onto what is drawn.
  std::vector<double> x(static_cast<size_t>(numPts));
  for (size_t i = 0; i < x.size(); ++i)
    x[i] = g.points()[i][0];
  node->setScalarField(x, cvc::colormap_kind::GRAY, 0.0, n);
  auto *colors = vtkUnsignedCharArray::SafeDownCast(pd->GetPointData()->GetScalars());
  const std::vector<unsigned char> want =
      cvc::colormap_rgb(x, cvc::colormap_kind::GRAY, 0.0, double(n));
  check(colors && colors->GetNumberOfTuples() == numPts &&
            std::vector<unsigned char>(colors->GetPointer(0), colors->GetPointer(0) + 3 * numPts) ==
                want,
        "a scalar field on the tet mesh colours the drawn surface per point");

  // Rebuilds keep the mode's cells (the boundary is cached per geometry).
  const vtkIdType polys = pd->GetNumberOfPolys(), lines = pd->GetNumberOfLines();
  node->setUseSingleColor(true);
  check(pd->GetNumberOfPolys() == polys && pd->GetNumberOfLines() == lines,
        "use_single_color rebuild: the same cells");
  node->setUseSingleColor(false);
  node->setRenderMode(GeometryRenderMode::POINTS);
  check(pd->GetNumberOfVerts() == numPts, "POINTS mode on the tet mesh");
  node->setRenderMode(GeometryRenderMode::TETS);
  check(pd->GetNumberOfPolys() == polys && pd->GetNumberOfLines() == lines,
        "back to TETS: the same cells");

  // A new geometry is not served the old boundary.
  node->setGeometry(tetCube(app, 1));
  if (cvc::mesh_ops_available())
    check(pd->GetNumberOfPolys() == 12 && pd->GetNumberOfPoints() == 8,
          "setGeometry(a 1^3 tet cube): its own 12-triangle boundary");
  else
    check(pd->GetNumberOfLines() == 36, "setGeometry(a 1^3 tet cube): its 36 tet edges");
}

// color_by a curvature kind on a mesh without surface triangles. A tet mesh is
// measured on its boundary (what TETS draws) -- a field that varies over the
// cube's corners, edges and faces, not the all-zero one an empty triangle list
// gives -- and its interior points get NaN (the colormap's NaN colour). Bare
// points and lines have no surface at all: no field, hasScalarField() false.
void testCurvatureWithoutTris(cvc::app &app) {
  std::printf("color_by=mean without surface triangles (libigl %s)\n",
              cvc::mesh_ops_available() ? "available" : "unavailable");
  SceneGraph sg(app, "tetcurv");
  const int n = 3;
  const cvc::geometry g = tetCube(app, n);
  auto node = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("cube", g));
  vtkPolyData *pd = node ? polyOf(*node) : nullptr;
  if (!pd) {
    check(false, "the tet cube is a GeometryNode with a polydata");
    return;
  }
  node->setUseSingleColor(false);
  node->getState("color_by").value(std::string("mean"));
  if (cvc::mesh_ops_available()) {
    const std::pair<double, double> r = node->scalarRange();
    check(node->hasScalarField() && std::isfinite(r.first) && std::isfinite(r.second),
          "a tet mesh has a curvature field (its boundary's) with a finite range");
    auto *colors = vtkUnsignedCharArray::SafeDownCast(pd->GetPointData()->GetScalars());
    const std::vector<unsigned char> nanRgb =
        cvc::colormap_rgb(std::vector<double>(1, std::numeric_limits<double>::quiet_NaN()),
                          cvc::colormap_kind::VIRIDIS, 0.0, 1.0);
    std::set<std::array<unsigned char, 3>> boundaryColours;
    bool interiorNaN = true;
    if (colors && colors->GetNumberOfComponents() == 3 &&
        colors->GetNumberOfTuples() == static_cast<vtkIdType>(g.num_points())) {
      for (size_t i = 0; i < g.num_points(); ++i) {
        const auto &p = g.const_points()[i];
        bool onBoundary = false;
        for (int ax = 0; ax < 3; ++ax)
          onBoundary = onBoundary || p[ax] == 0.0 || p[ax] == double(n);
        const unsigned char *c = colors->GetPointer(static_cast<vtkIdType>(3 * i));
        if (onBoundary)
          boundaryColours.insert(std::array<unsigned char, 3>{{c[0], c[1], c[2]}});
        else
          interiorNaN = interiorNaN && nanRgb.size() == 3 && c[0] == nanRgb[0] &&
                        c[1] == nanRgb[1] && c[2] == nanRgb[2];
      }
    } else {
      interiorNaN = false;
    }
    check(boundaryColours.size() > 1,
          "the boundary's curvature varies over the cube (not a constant all-zero field)");
    check(interiorNaN, "interior points, on no boundary triangle, get the NaN colour");
  } else {
    check(!node->hasScalarField(), "without libigl a tet mesh has no curvature field");
  }

  cvc::geometry bare(app);
  for (int i = 0; i < 10; ++i)
    bare.points().push_back({double(i), 0.5 * i, 0.0});
  bare.lines().push_back({0, 1});
  bare.lines().push_back({1, 2});
  auto pts = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("bare", bare));
  if (!pts) {
    check(false, "the bare points are a GeometryNode");
    return;
  }
  pts->setUseSingleColor(false);
  pts->getState("color_by").value(std::string("mean"));
  check(!pts->hasScalarField() && std::isnan(pts->scalarRange().first),
        "points and lines: color_by=mean gives no field (not an all-zero one)");
}

// Tets the boundary extraction rejects (an index past the last point) fall
// back to the edge wireframe instead of throwing out of setGeometry.
void testRejectedTets(cvc::app &app) {
  std::printf("TETS mode on tets the boundary extraction rejects\n");
  SceneGraph sg(app, "tetbad");
  cvc::geometry g = tetCube(app, 1);
  g.tets().push_back({0, 1, 2, 99}); // 8 points: 99 is out of range
  auto node = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("bad", g));
  vtkPolyData *pd = node ? polyOf(*node) : nullptr;
  check(pd && pd->GetNumberOfPolys() == 0 && pd->GetNumberOfLines() == 6 * 7 &&
            representation(*node) == VTK_WIREFRAME,
        "an invalid tet mesh draws the tet edges, without throwing");
}
} // namespace

int main() {
  cvc::app app;
  testTetCube(app, 3);
  testCurvatureWithoutTris(app);
  testRejectedTets(app);
  std::printf("%s: cvcgl_tet_surface (%d check%s failed)\n", g_fail == 0 ? "PASS" : "FAIL", g_fail,
              g_fail == 1 ? "" : "s");
  return g_fail == 0 ? 0 : 1;
}
