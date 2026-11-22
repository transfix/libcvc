/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// pocket_tunnel with Tight Cocone's marking given instead of computed, internal to libcvc: for
// the legacy A/B (pocket_tunnel_test, tools/volrover-goldens), which runs the port's own stages
// (S_max, the mouths, the output) on the legacy library's marking and compares them exactly.
//
// Tight Cocone's peeling is greedy: it starts at the first infinite cell it meets and walks in
// facet order, so where its choices are close, the cells it marks outside depend on that order.
// The legacy library's order is its cells' addresses (CGAL reports a facet from the cell at the
// lower address), which differ from run to run of one process and between platforms; the port's
// is tet_complex's, a spatial order. The surface-hugging pockets of a cell or two are where they
// differ. Given the same marking, the stages after it are compared bit for bit.

#ifndef CVC_GEOMETRY_POCKET_TUNNEL_MARKED_H
#define CVC_GEOMETRY_POCKET_TUNNEL_MARKED_H

#include <array>
#include <cvc/geometry/pocket_tunnel.h>
#include <set>
#include <vector>

namespace cvc::pocket_tunnel_detail {

// Cells and facets by their corners' input indices, ascending (a duplicate point by its first
// occurrence).
struct marking {
  std::set<std::array<int, 4>> outside; // the finite cells outside the surface
  std::set<std::array<int, 3>> cocone;  // the finite facets on it (both sides flagged)
};

// pocket_tunnel(points, params), with `given` in place of Tight Cocone's marking. A cell or facet
// of `given` that is not in the triangulation of `points` is an error (pocket_tunnel_error).
pocket_tunnels pocket_tunnel_marked(const std::vector<std::array<double, 3>> &points,
                                    const pocket_tunnel_params &params, marking given);

} // namespace cvc::pocket_tunnel_detail

#endif // CVC_GEOMETRY_POCKET_TUNNEL_MARKED_H
