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
//   A file's scene graph (glTF, FBX, DAE, ...) comes along as `nodes`: each node
//   places the meshes it draws by its transform and every ancestor's, and a mesh
//   several nodes draw is drawn once for each.
// ---- Change History ----
// 2024 -- Joe R. -- Creation (Phase-3 model surface).
// 2026 -- Joe R. -- The file's node tree (nodes, instances(), placed()).
struct model {
  struct mesh {
    geometry geom;
    int material = -1; // index into model::materials, or -1 for none
    std::string name;
  };

  // A 4x4 affine transform, row-major: element (r, c) is [r * 4 + c], so the
  // translation is [3], [7], [11] and the last row is 0 0 0 1. It maps a point p
  // (a column) to M p.
  typedef boost::array<double, 16> transform_t;
  static transform_t identity_transform();
  static bool is_identity(const transform_t &t);
  // a * b: b applied first.
  static transform_t compose(const transform_t &a, const transform_t &b);

  // A node of the file's scene graph: its name, its transform relative to its
  // parent, the meshes it draws (indices into meshes) and its children (indices
  // into nodes).
  struct node {
    std::string name;
    transform_t transform = identity_transform();
    std::vector<int> meshes;
    std::vector<int> children;
  };

  // One drawn copy of a mesh: which mesh, the node that draws it (-1 when the
  // model has no nodes), and its transform into the model's space (the node's
  // own composed with every ancestor's).
  struct instance {
    int mesh = -1;
    int node = -1;
    transform_t transform = identity_transform();
  };

  std::vector<mesh> meshes;
  std::vector<material> materials;

  // The scene graph, nodes[0] its root. Empty: every mesh is drawn once, as it
  // is (a model built in code, or read from a format without a scene graph).
  // With nodes, only the meshes a node draws are drawn.
  std::vector<node> nodes;

  // The physical length, in canonical SI metres, of ONE unit of this model's
  // local coordinate system -- i.e. what the vertex coordinates are authored in.
  // 1.0 means the mesh is already in metres (the safe default, and what every
  // importer produces today). An importer that knows better stamps it: glTF is
  // metres (1.0), FBX is usually centimetres (0.01), and a CAD part authored in
  // millimetres is 0.001. This is DISTINCT from any render/scene scale; it is a
  // property of the file, and it is what lets model dimensions be mapped to the
  // world unit base (see cvc::world_units) so a span or a picked coordinate can
  // be reported in real metres/kilometres/miles rather than raw file numbers.
  double metres_per_source_unit = 1.0;

  bool empty() const { return meshes.empty(); }

  // ------------------
  // model::instances
  // ------------------
  // Purpose:
  //   Every drawn copy of a mesh, in the order the scene graph lists them: depth
  //   first from nodes[0], a node's meshes before its children. A node that two
  //   parents list is drawn under each. With no nodes, each mesh once, in order,
  //   untransformed. A mesh or child index out of range is skipped, and so is a
  //   child that is its own ancestor (a cycle).
  std::vector<instance> instances() const;

  // ---------------
  // model::placed
  // ---------------
  // Purpose:
  //   `g` moved by `t`: its points transformed, its normals by the inverse
  //   transpose and made unit, its tangents turned and made unit, and when `t`
  //   mirrors (a negative determinant) its triangles and quads rewound and its
  //   tangents' handedness flipped, so faces keep facing out. The identity
  //   returns `g` unchanged (shared).
  static geometry placed(const geometry &g, const transform_t &t);

  // ---------------
  // model::merged
  // ---------------
  // Purpose:
  //   Every instance's mesh, placed by its transform, concatenated into a single
  //   cvc::geometry, offsetting each triangle (and line/quad) index by the running
  //   vertex count so the merged index buffer is correct. Per-vertex attributes
  //   (normals/colors/uvs/tangents) are appended in the same order. Material
  //   assignment is not preserved.
  geometry merged() const;

  // ---------------
  // model::extents
  // ---------------
  // Purpose:
  //   Union of every instance's bounding box, placed, in the model's own
  //   (authoring) coordinate space.
  bounding_box extents() const;

  // ---------------
  // model::extents_metres
  // ---------------
  // Purpose:
  //   extents() scaled by metres_per_source_unit, i.e. the model's footprint in
  //   canonical SI metres regardless of what units the file was authored in.
  //   This is the bridge from a model's declared dimensions to the world unit
  //   base; feed the corners to cvc::world_units to display them in the active
  //   regime. A null (empty) box is returned unchanged.
  bounding_box extents_metres() const;

  boost::uint64_t num_meshes() const { return meshes.size(); }
};

} // namespace cvc

#endif // __CVC_MODEL_H__
