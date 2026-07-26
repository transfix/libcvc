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
  // geometry::merge appends the per-vertex attribute arrays (normals/colors/uvs/
  // tangents) by raw insert, WITHOUT padding them to the vertex count — it only
  // offsets the appended index buffers. So merging meshes with heterogeneous
  // attribute presence (e.g. one textured mesh carrying UVs+tangents and one
  // untextured mesh carrying neither, which is routine for a multi-material
  // OBJ/glTF) would leave a merged array shorter than points() and misaligned
  // relative to the correctly-offset triangle indices — a consumer indexing
  // uvs[vertexIndex] then reads out of bounds or the wrong vertex. Normalize
  // first: if ANY mesh carries an attribute, pad the meshes that lack it to full
  // length with a neutral default, so every per-vertex array in the result stays
  // exactly num_points() long and index-aligned.
  bool any_normals = false, any_colors = false, any_uvs = false, any_tangents = false;
  for (std::vector<mesh>::const_iterator i = meshes.begin(); i != meshes.end(); ++i) {
    any_normals = any_normals || !i->geom.const_normals().empty();
    any_colors = any_colors || !i->geom.const_colors().empty();
    any_uvs = any_uvs || !i->geom.const_uvs().empty();
    any_tangents = any_tangents || !i->geom.const_tangents().empty();
  }

  geometry::normal_t def_n;
  def_n[0] = 0;
  def_n[1] = 0;
  def_n[2] = 1;
  geometry::color_t def_c;
  def_c[0] = 1;
  def_c[1] = 1;
  def_c[2] = 1;
  geometry::uv_t def_uv;
  def_uv[0] = 0;
  def_uv[1] = 0;
  geometry::tangent_t def_t;
  def_t[0] = 1;
  def_t[1] = 0;
  def_t[2] = 0;
  def_t[3] = 1;

  geometry out;
  for (std::vector<mesh>::const_iterator i = meshes.begin(); i != meshes.end(); ++i) {
    geometry g = i->geom; // COW copy; the padding below detaches only what it touches
    const std::size_t nv = static_cast<std::size_t>(g.num_points());
    if (any_normals && g.const_normals().empty())
      g.normals().assign(nv, def_n);
    if (any_colors && g.const_colors().empty())
      g.colors().assign(nv, def_c);
    if (any_uvs && g.const_uvs().empty())
      g.uvs().assign(nv, def_uv);
    if (any_tangents && g.const_tangents().empty())
      g.tangents().assign(nv, def_t);
    out.merge(g);
  }
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
