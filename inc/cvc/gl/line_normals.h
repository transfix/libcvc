/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef CVC_GL_LINE_NORMALS_H
#define CVC_GL_LINE_NORMALS_H

#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

namespace cvc {
namespace gl {

// Give a LINE (or point) polydata a constant normal array.
//
// VTK's polydata mapper decides between its LIT and UNLIT shader templates on
// the mere presence of point normals, and the shadow-map pass splices its
// snippet into the lit template only. So a single normal-less line primitive
// anywhere in the scene fails to compile its shader the moment shadows are
// switched on -- "'vertexVC' : undeclared identifier" -- and that actor then
// draws nothing. Every SceneGraph builds a GridNode by default, so without this
// enabling shadows breaks on a scene that has not been given any geometry yet.
//
// vtkPolyDataNormals cannot be used here: it discards lines from its output.
// A constant normal is the honest answer anyway, since a line has no surface
// orientation for one to be derived from; +Z merely makes the shading uniform.
inline void ensureLineNormals(vtkPolyData *polyData) {
  if (!polyData || !polyData->GetPoints())
    return;
  if (polyData->GetPointData()->GetNormals())
    return;
  const vtkIdType numPts = polyData->GetNumberOfPoints();
  if (numPts == 0)
    return;

  vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
  normals->SetNumberOfComponents(3);
  normals->SetNumberOfTuples(numPts);
  for (vtkIdType i = 0; i < numPts; ++i)
    normals->SetTuple3(i, 0.0, 0.0, 1.0);
  polyData->GetPointData()->SetNormals(normals);
}

} // namespace gl
} // namespace cvc

#endif
