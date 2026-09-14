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

// recipes.h — the built-in recipe library.
//
// Each recipe is an embedded `.lsys` source. Trees/shrub derive in parallel mode
// (turtle alphabet); rocks and building shells derive in sequential mode (scope
// alphabet). `;(k)` sets the material role (see cvc::lsys::role): trunk=0,
// branch=1, foliage=2, rock=3, ground=4, water=5, wall_concrete=6, wall_brick=7,
// wall_drywall=8, glass=9, metal=10, wood_solid=11.

#ifndef CVC_LSYS_RECIPES_H
#define CVC_LSYS_RECIPES_H

#include <cvc/lsys/grammar.h>
#include <string>
#include <vector>

namespace cvc {
namespace lsys {

// Names of all built-in recipes (sorted, stable).
std::vector<std::string> recipe_names();

bool has_recipe(const std::string &name);

// Raw `.lsys` source of a recipe (empty if unknown).
std::string recipe_source(const std::string &name);

// Parse a built-in recipe into a ruleset. Throws std::out_of_range if unknown.
ruleset load_recipe(const std::string &name);

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_RECIPES_H
