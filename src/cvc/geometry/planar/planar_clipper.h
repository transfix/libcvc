/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::planar's Clipper2 conversions, shared by clipper.cpp and wavefront.cpp. Never installed:
// no public header names a Clipper2 type.

#ifndef CVC_GEOMETRY_PLANAR_CLIPPER_H
#define CVC_GEOMETRY_PLANAR_CLIPPER_H

#include <clipper2/clipper.h>
#include <cvc/geometry/planar.h>

namespace cvc {
namespace planar {
namespace detail {

// A set's rings, each polygon normalised (outer counterclockwise, holes clockwise), so Clipper2's
// non-zero rule reads the set as the union of its polygons.
Clipper2Lib::Paths64 to_paths(const polygon_set &s);

// Clipper2's tree of outer rings and holes as normalised polygons; an island inside a hole is a
// polygon of its own.
polygon_set from_tree(const Clipper2Lib::PolyTree64 &tree);

// One mitred offset pass of normalised paths.
polygon_set mitred_offset(const Clipper2Lib::Paths64 &paths, double delta, double miter_limit);

} // namespace detail
} // namespace planar
} // namespace cvc

#endif
