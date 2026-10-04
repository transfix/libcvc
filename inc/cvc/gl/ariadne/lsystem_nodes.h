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

// lsystem_nodes.h — custom Ariadne scene node types that render the cvc::lsys generators.
//
// The cvcGL side of the lsystem library: it turns the GL-free field/mesh generators in cvc::lsys
// (forest.h / cloud.h / water.h) into live GraphicsNodes a `.ari` scene can declare. These node
// types used to live inside the lsystem_forest demo's own .cpp; promoting them here means ANY
// Ariadne host — including the generic ariadne_hello shell — can render an L-system forest, sea or
// sky from a pure `.ari`, with the shading supplied declaratively by the `shader:` surface.
//
// register_lsystem_node_types() installs them (idempotent, process-global, before realize_scene):
//   - "forest_trees" — a scattered forest (cvc::lsys::grow_forest): a wood GeometryNode (per-vertex
//     colour) + a needle-LINES GeometryNode, with a per-frame wind re-pose tick (repose_forest,
//     driven by the scene's world clock). Honours a `shader: { preset: bark }` on the node.
//
// (wave_sea / cloud_sky follow.) These are registered for a host by register_cvcgl_extensions()
// (extensions.h), which ariadne_hello calls.

#ifndef CVC_GL_ARIADNE_LSYSTEM_NODES_H
#define CVC_GL_ARIADNE_LSYSTEM_NODES_H

namespace cvc {
namespace gl {
namespace ariadne {

// Register the L-system scene node types (forest_trees, …) with the scene-node registry. Idempotent
// — re-registering replaces, and types already present are harmless to re-add. Call once at host
// setup, before realize_scene.
void register_lsystem_node_types();

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_LSYSTEM_NODES_H
