/*
  Copyright 2024 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

#ifndef __CVC_MODEL_H__
#define __CVC_MODEL_H__

// cvc::model — a small scene value type (Phase-3 mesh/model surface). A single
// mesh file (OBJ/glTF/FBX/...) can carry several meshes plus their materials and
// textures, so a model bundles a vector of meshes (each a cvc::geometry + a
// material index) and a vector of cvc::material. Textures decode BELOW the VTK
// line via cvc::image, so cvcGL, pycvc, and the mesh loaders consume already
// decoded pixels. Like cvc::geometry / cvc::image this is a plain, copyable
// value type; the reference-counted sharing lives inside geometry/image.

#include <boost/array.hpp>
#include <boost/cstdint.hpp>
#include <cvc/geometry/geometry.h>
#include <cvc/image/image.h>
#include <cvc/volume/bounding_box.h>
#include <string>
#include <vector>

namespace cvc {

// --------
// material
// --------
// Purpose:
//   A minimal PBR-ish material description. Values are normalized to the glTF
//   metallic/roughness model; OBJ/other importers map their fields onto it
//   (Kd -> base_color rgb, d/opacity -> base_color a, etc.). base_color is a
//   multiplier applied on top of base_color_texture when one is present.
// ---- Change History ----
// 2024 -- Joe R. -- Creation (Phase-3 model surface).
struct material {
  std::string name;
  boost::array<double, 4> base_color = {
      {1, 1, 1, 1}}; // RGBA multiplier (glTF baseColorFactor / obj Kd+d)
  double metallic = 0.0;
  double roughness = 1.0;
  boost::array<double, 3> emissive = {{0, 0, 0}};
  std::string
      base_color_texture_path; // path exactly as referenced in the file (may be relative or empty)
  image base_color_texture;    // the LOADED texture (image.empty() if none / unresolved)

  bool has_base_color_texture() const { return !base_color_texture.empty(); }
};

// -----
// model
// -----
// Purpose:
//   A scene: several meshes and the materials they reference. Each mesh pairs a
//   cvc::geometry with an index into materials (-1 when the mesh has no material).
// ---- Change History ----
// 2024 -- Joe R. -- Creation (Phase-3 model surface).
struct model {
  struct mesh {
    geometry geom;
    int material = -1; // index into model::materials, or -1 for none
    std::string name;
  };

  std::vector<mesh> meshes;
  std::vector<material> materials;

  bool empty() const { return meshes.empty(); }

  // ---------------
  // model::merged
  // ---------------
  // Purpose:
  //   Concatenate every mesh into a single cvc::geometry, offsetting each mesh's
  //   triangle (and line/quad) indices by the running vertex count so the merged
  //   index buffer is correct. Per-vertex attributes (normals/colors/uvs/tangents)
  //   are appended in the same order. Material assignment is not preserved.
  geometry merged() const;

  // ---------------
  // model::extents
  // ---------------
  // Purpose:
  //   Union of every mesh's bounding box (reuses geometry's bbox facility).
  bounding_box extents() const;

  boost::uint64_t num_meshes() const { return meshes.size(); }
};

} // namespace cvc

#endif // __CVC_MODEL_H__
