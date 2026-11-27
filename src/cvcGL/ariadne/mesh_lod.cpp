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
*/

#include <algorithm>
#include <cstdint>
#include <cvc/gl/ariadne/mesh_lod.h>
#include <vector>
#include <vtkCellArray.h>
#include <vtkCleanPolyData.h>
#include <vtkIdList.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkQuadricDecimation.h>

namespace cvc {
namespace gl {
namespace ariadne {

// A mesh decimated to about (1 - reduction) of its triangles by VTK's quadric error metric, which
// keeps a silhouette: the coarser vehicle rungs. False when it collapsed.
bool decimate_mesh(const std::vector<double> &verts, const std::vector<std::uint32_t> &tris,
                   double reduction, std::vector<double> &ov, std::vector<std::uint32_t> &ot) {
  const vtkIdType nv = static_cast<vtkIdType>(verts.size() / 3);
  vtkNew<vtkPoints> pts;
  pts->SetNumberOfPoints(nv);
  for (vtkIdType i = 0; i < nv; ++i)
    pts->SetPoint(i, verts[3 * i], verts[3 * i + 1], verts[3 * i + 2]);
  vtkNew<vtkCellArray> cells;
  for (std::size_t k = 0; k + 2 < tris.size(); k += 3) {
    const vtkIdType tri[3] = {static_cast<vtkIdType>(tris[k]), static_cast<vtkIdType>(tris[k + 1]),
                              static_cast<vtkIdType>(tris[k + 2])};
    cells->InsertNextCell(3, tri);
  }
  vtkNew<vtkPolyData> pd;
  pd->SetPoints(pts);
  pd->SetPolys(cells);
  // Welded first: a model's mesh is split at its texture seams, and the decimator collapses edges
  // only within a connected surface, so a split one barely coarsens (the coarse rungs carry no
  // texture coordinates).
  vtkNew<vtkCleanPolyData> weld;
  weld->SetInputData(pd);
  weld->PointMergingOn();
  weld->SetTolerance(0.0);
  vtkNew<vtkQuadricDecimation> dec;
  dec->SetInputConnection(weld->GetOutputPort());
  dec->SetTargetReduction(reduction);
  dec->Update();
  vtkPolyData *out = dec->GetOutput();
  ov.clear();
  ot.clear();
  if (!out || !out->GetPoints() || out->GetNumberOfPolys() < 4)
    return false;
  // Only the points its triangles use: the filter keeps every input point, so without this a rung
  // of a twentieth of the triangles would stream nearly as many vertices as the full model (the
  // nav demos' rungs did), and every vertex is uploaded every frame.
  vtkPoints *op = out->GetPoints();
  std::vector<std::int64_t> remap(static_cast<std::size_t>(op->GetNumberOfPoints()), -1);
  vtkCellArray *polys = out->GetPolys();
  polys->InitTraversal();
  vtkNew<vtkIdList> ids;
  while (polys->GetNextCell(ids)) {
    if (ids->GetNumberOfIds() != 3)
      continue;
    for (int j = 0; j < 3; ++j) {
      const vtkIdType id = ids->GetId(j);
      if (remap[id] < 0) {
        remap[id] = static_cast<std::int64_t>(ov.size() / 3);
        double p[3];
        op->GetPoint(id, p);
        ov.insert(ov.end(), {p[0], p[1], p[2]});
      }
      ot.push_back(static_cast<std::uint32_t>(remap[id]));
    }
  }
  return !ot.empty();
}

bool decimate_mesh_by_role(const std::vector<double> &verts, const std::vector<std::uint32_t> &tris,
                           const std::vector<unsigned char> &role,
                           const std::vector<unsigned char> &own, double reduction,
                           std::vector<double> &ov, std::vector<std::uint32_t> &ot,
                           std::vector<unsigned char> &orole, std::vector<unsigned char> &oown) {
  std::vector<unsigned char> roles; // in the order their triangles first come
  for (std::size_t t = 0; t + 2 < tris.size(); t += 3) {
    const unsigned char q = role[tris[t]];
    if (std::find(roles.begin(), roles.end(), q) == roles.end())
      roles.push_back(q);
  }
  std::vector<double> rv;
  std::vector<std::uint32_t> rt;
  std::vector<unsigned char> rrole, rown;
  bool coarser = false;
  for (const unsigned char q : roles) {
    std::vector<std::int64_t> at(verts.size() / 3, -1);
    std::vector<std::uint32_t> from; // each of the part's points, its point in the mesh
    std::vector<double> pv;
    std::vector<std::uint32_t> pt;
    for (std::size_t t = 0; t + 2 < tris.size(); t += 3) {
      if (role[tris[t]] != q)
        continue;
      for (int c = 0; c < 3; ++c) {
        const std::uint32_t v = tris[t + c];
        if (at[v] < 0) {
          at[v] = static_cast<std::int64_t>(from.size());
          from.push_back(v);
          pv.insert(pv.end(), {verts[3 * v], verts[3 * v + 1], verts[3 * v + 2]});
        }
        pt.push_back(static_cast<std::uint32_t>(at[v]));
      }
    }
    std::vector<double> dv;
    std::vector<std::uint32_t> dt;
    const bool ok = decimate_mesh(pv, pt, reduction, dv, dt);
    coarser = coarser || ok;
    const std::vector<double> &uv = ok ? dv : pv;
    const std::vector<std::uint32_t> &ut = ok ? dt : pt;
    const auto base = static_cast<std::uint32_t>(rv.size() / 3);
    rv.insert(rv.end(), uv.begin(), uv.end());
    for (const std::uint32_t i : ut)
      rt.push_back(base + i);
    const std::size_t n = uv.size() / 3;
    rrole.insert(rrole.end(), n, q);
    for (std::size_t i = 0; i < n; ++i) {
      const std::uint32_t v = from[ok ? 0 : i];
      rown.insert(rown.end(), {own[3 * v], own[3 * v + 1], own[3 * v + 2]});
    }
  }
  if (!coarser)
    return false;
  ov = std::move(rv);
  ot = std::move(rt);
  orole = std::move(rrole);
  oown = std::move(rown);
  return true;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
