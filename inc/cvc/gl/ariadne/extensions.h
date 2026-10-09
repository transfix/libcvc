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

// extensions.h — the single hook by which cvcGL contributes its functionality to Ariadne.
//
// The Ariadne registries (scene node types, shader presets, program-lane intrinsics, …) are
// process-global and populated by explicit host calls. Rather than make every host wire each one,
// cvcGL bundles everything it offers behind ONE call a host makes at setup. The generic
// ariadne_hello shell calls it, so a pure `.ari` document can use cvcGL's extended node types and
// shader effects with no bespoke host C++ — which is exactly what lets a demo collapse into a
// self-contained `.ari`.
//
// Today it installs the default shader presets (terrain_bump / bark), the L-system scene node
// types (forest_trees / …) and, on a CVC_ENABLE_LIBIGL build, the geometry-processing and
// finite-element node types (mesh_lab / fe_lab, mesh_nodes.h). It takes the app because app-scoped
// extensions (the L-system program intrinsics) register against the app's runtime; the
// process-global ones ignore it. ariadne_hello and AriRuntime (so pycvc hosts) both call it.

#ifndef CVC_GL_ARIADNE_EXTENSIONS_H
#define CVC_GL_ARIADNE_EXTENSIONS_H

namespace cvc {
class app;
namespace gl {
namespace ariadne {

// Register all of cvcGL's Ariadne extensions for `app`. Idempotent; call once at host setup, before
// loading/realizing a document. A host that wants only a subset can call the individual registrars
// (register_default_shader_presets / register_lsystem_node_types / register_mesh_node_types)
// instead.
void register_cvcgl_extensions(cvc::app &app);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_EXTENSIONS_H
