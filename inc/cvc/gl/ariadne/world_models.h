/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

// world_models.h — a generated world's theme models (cvc::world::generated_world::models), drawn.
//
// Each model asset (cvc/world/model_assets.h) is read once into a LOD ladder in the world frame:
// rung k is the asset's LOD k glb (read with the app's model handlers, merged, turned from glTF's
// +Y up to the world's +Z up by model_to_world) with its base-colour texture, and its switch error
// the asset's world_error_m. Every placed copy is a LodGraphicsNode posed by its instance's
// transform, whose rungs draw from the first copy's VTK arrays and texture
// (GeometryNode::shareArraysFrom, shareTexture), so a hundred palms hold one palm's buffers; each
// chooses its own rung from its distance (LodGraphicsNode::select), so only the copies near the
// camera draw at full resolution.

#ifndef CVC_GL_ARIADNE_WORLD_MODELS_H
#define CVC_GL_ARIADNE_WORLD_MODELS_H

#include <cvc/geometry/geometry.h>
#include <cvc/image/image.h>
#include <cvc/lod/pyramid.h>
#include <cvc/world/generated_world.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace gl {
class GeometryNode;
class GraphicsNode;
class LodGraphicsNode;
namespace ariadne {

// One asset, ready to draw.
struct model_mesh {
  std::string name;
  cvc::lod::mesh_pyramid pyr;       // rungs in the world frame, world_error_m from the asset
  std::vector<cvc::image> textures; // per rung: its base colour (empty: none)
  // Per rung, the first node drawing it: the others share its arrays and texture. Frame thread.
  std::vector<std::weak_ptr<GeometryNode>> donors;
};

// A mesh in glTF's frame (+Y up, front +Z) turned into the world's (+Z up, front +Y), points and
// normals (cvc::world::model_to_world).
cvc::geometry model_frame_to_world(cvc::geometry g);

// The asset's ladder, read from its files: its own directory, else the app's catalog's entry of
// its name (a world read from a bundle knows its assets by name). Cached on the app by file: every
// world of the app that draws the asset shares one. A rung whose file cannot be read ends the
// ladder there; null (and `error` set) when LOD 0 cannot be read. Thread-safe: a world's job reads
// its models off the frame thread.
std::shared_ptr<model_mesh> load_model_mesh(cvc::app &app, const cvc::world::model_asset &asset,
                                            std::string *error = nullptr);

// The row-major 4x4 transform of an instance: scale, yaw about +Z, then translate.
void model_instance_matrix(const cvc::world::generated_world::model_instance &m, double out[16]);

// An instance as a LOD ladder under `parent`, hidden, posed by its transform, its rungs' errors
// scaled with it, drawing from `mesh`'s shared arrays and textures. `soft` is the world's look.
// Frame thread.
std::shared_ptr<LodGraphicsNode>
make_model_instance(GraphicsNode &parent, const std::string &name, model_mesh &mesh,
                    const cvc::world::generated_world::model_instance &m, bool soft);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_WORLD_MODELS_H
