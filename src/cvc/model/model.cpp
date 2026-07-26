/*
  Copyright 2024 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

#include <cvc/model/model.h>

namespace cvc {

// ---------------
// model::merged
// ---------------
// Purpose:
//   Concatenate every mesh into a single geometry. cvc::geometry::merge already
//   appends the per-vertex attribute arrays and offsets the appended
//   line/tri/quad indices by the running vertex count, so a fold over merge()
//   produces the correct flattened index buffer.
geometry model::merged() const {
  geometry out;
  for (std::vector<mesh>::const_iterator i = meshes.begin(); i != meshes.end(); ++i)
    out.merge(i->geom);
  return out;
}

// ---------------
// model::extents
// ---------------
// Purpose:
//   Union of every mesh's bounding box. generic_bounding_box::operator+ treats a
//   null (zero-volume) box as the identity, so an empty model yields a null box.
bounding_box model::extents() const {
  bounding_box bbox;
  for (std::vector<mesh>::const_iterator i = meshes.begin(); i != meshes.end(); ++i)
    bbox = bbox + i->geom.extents();
  return bbox;
}

} // namespace cvc
