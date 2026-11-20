/*
  Copyright 2008-2012, 2026 The University of Texas at Austin

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

// Vendored from VolumeRover's ContourTiler (John Edwards, UT Austin; transfix/volrover
// src/ContourTiler at 1cf0fbd7) and ported onto libcvc's tiler kernel and cvc::planar
// (VOLROVER-LIBRARY-PORTS-ROADMAP.md §3).

#ifndef __RIGHT_ENUM_H__
#define __RIGHT_ENUM_H__

#include "common.h"

namespace cvc::tiler {

/// Example:
///
/// struct color_def
/// {
///     enum type
///     {
///         red, green, blue
///     };
/// };
///
/// typedef Right_enum<color_def> color; // use int as enum holder data type (in common platforms)
/// typedef Right_enum<color_def, unsigned char> color; // use uchar as enum holder
///
/// // usage:
/// void f(color p)
/// {
///     p = color::green;
/// }
///
/// int main()
/// {
///     color p = color::red;
///     f(p);
/// }

template <typename def, typename inner = typename def::type> struct Right_enum : def {
  typedef typename def::type type;
  //   typedef typename inner inner;
  inner v;
  Right_enum(type v) : v(static_cast<type>(v)) {}
  operator inner() const { return v; }
};

} // namespace cvc::tiler

#endif
