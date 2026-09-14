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

// context.h — context-provider interface (roadmap §5.1 escape hatch #3).
//
// The default (string-neighbour, bracket- and #ignore-aware) context match is
// implemented directly inside derive(); this interface is the extension point
// for a future SPATIAL context (BVH occlusion / snap-line queries, [TOP94]/
// [ENV96]). derive_options carries a nullable pointer; a null provider means
// "use the built-in string context".

#ifndef CVC_LSYS_CONTEXT_H
#define CVC_LSYS_CONTEXT_H

#include <cvc/lsys/module.h>

namespace cvc {
namespace lsys {

class context_provider {
public:
  virtual ~context_provider() = default;

  // Answer a spatial query ?P(x,y,z): is this position inside/allowed? The
  // built-in string context ignores this; spatial providers override it.
  virtual bool query_point(double x, double y, double z) const {
    (void)x;
    (void)y;
    (void)z;
    return true;
  }
};

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_CONTEXT_H
