/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The geometry a scene node's `source:` declares, built from data alone: the plane, box and sphere
// primitives, a procedural heightfield (the world's own cvc::height_field), or a mesh file / URI
// with its `fit:` applied. No renderer: a headless host builds a scene's physics bodies from these
// (scene_world.h), and cvcGL draws the same meshes, so a body and the mesh a window shows are one
// surface.

#ifndef __CVC_ARIADNE_SCENE_GEOMETRY_H__
#define __CVC_ARIADNE_SCENE_GEOMETRY_H__

#include <cvc/geometry/geometry.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {

class height_field;

namespace ariadne {

struct SceneNode;
struct SceneHeightfield;

// A flat quad over [-size/2, size/2]^2 at z = 0 with up normals: `source: { plane: { size } }`.
cvc::geometry make_plane_geometry(float size);
// A box centred on the origin with edge lengths `size`, a normal per face: `source: { box }`.
cvc::geometry make_box_geometry(const float size[3]);
// A UV sphere about the origin: `source: { sphere: { radius } }`.
cvc::geometry make_sphere_geometry(float radius);

// A procedural heightfield (`source: { heightfield: {...} }`): a res x res grid over
// [-size/2, size/2]^2 displaced in z by its generator's terrain (`dem_path`: a resolved file for a
// `file` generator) and its dome / wave layers, with UVs and band colours on the mesh. Throws
// cvc::geometry_error for a size that is not > 0, std::invalid_argument for an unknown generator.
std::shared_ptr<cvc::height_field> make_height_field(const SceneHeightfield &hf,
                                                     const std::string &dem_path = std::string());

// `fit:` -- stand a mesh on the ground: turn the dataset's up axis (`x`, `y` or `z`) to +z, a half
// turn about x when `flip`, centre it in XY, sit its base on z = 0 and scale its tallest extent to
// `height`. Returns a fresh geometry (points, normals, colours, tris).
cvc::geometry fit_to_ground(const cvc::geometry &raw, char up, bool flip, float height);
// That rotation alone: points and normals turned, everything else copied.
cvc::geometry turn_up(const cvc::geometry &raw, char up, bool flip);
void turn_up_point(char up, bool flip, const double in[3], double out[3]);
// A node's `fit: { up }` as one of 'x', 'y', 'z' (SceneNode::fit_up_y wins: it is the `y` case).
char fit_up_axis(const SceneNode &node);

// The mesh a node's `source:` declares (a heightfield, a primitive, or a file / URI read and
// fitted), and for a heightfield its grid in `field` (left empty otherwise). A file is read with
// ctx's geometry handlers. Warns, naming the node, and returns false when there is no usable
// source.
bool read_scene_node_geometry(cvc::app &ctx, const SceneNode &node, cvc::geometry &geom,
                              std::shared_ptr<const cvc::height_field> &field,
                              std::vector<std::string> *warnings);

// A node's local transform (`transform: { position, rotation, scale }`) as a row-major 4x4: the
// rotation Rz * Ry * Rx (degrees, about the parent's axes), each row then scaled (the scene graph's
// setScale), the position last. Identity without a transform.
void scene_node_local_matrix(const SceneNode &node, double out[16]);

// a * b, row-major 4x4.
void multiply_matrix4(const double a[16], const double b[16], double out[16]);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_GEOMETRY_H__
