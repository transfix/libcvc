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

// extensions.cpp — the cvcGL → Ariadne extension bundle (see extensions.h).

#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/lsystem_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h> // register_default_shader_presets

namespace cvc {
namespace gl {
namespace ariadne {

void register_cvcgl_extensions(cvc::app &app) {
  register_default_shader_presets(); // "terrain_bump", "bark"
  register_lsystem_node_types();     // "forest_trees", …
  // App-scoped extensions (the L-system program intrinsics) will register here against app's
  // runtime; the registrations above are process-global and do not need it.
  (void)app;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
