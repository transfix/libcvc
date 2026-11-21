/*
  Copyright 2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu> (the legacy SecondaryStructures)
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

// SecondaryStructures' own stages on the cocone core (cvc/geometry/secondary_structures.h),
// internal to libcvc: its medial axis (medax.cpp), its skeleton graph (skel.cpp's add_u1_to_skel,
// add_u2_to_skel and do_star) and its selection of helices and sheets (helix.cpp, skel.cpp's
// filter and build*Geometry). The triangulation, Tight Cocone, U1 and U2 are the core's
// (../skeleton/cocone.h), which the legacy files duplicated.
//
// The legacy cell base's extra fields (e_tag, skel_id) live here, in arrays indexed by the cell's
// id, so the core's data structures stay as they are.

#ifndef CVC_GEOMETRY_SECONDARY_SECONDARY_H
#define CVC_GEOMETRY_SECONDARY_SECONDARY_H

#include "../skeleton/cocone.h"

#include <array>
#include <cstdint>
#include <cvc/geometry/secondary_structures.h>
#include <functional>
#include <vector>

namespace cvc::secondary_detail {

using cocone::step_task;

// medax.cpp: the inner medial axis. Every Voronoi facet inside the surface is on it at first; then
// the facets on its free boundary that neither the angle nor the ratio condition chose are peeled
// away, until none is left to peel. Marks VF_on_medax and VV_on_medax, which U1 and U2 read.
step_task compute_inner_medial_axis(cocone::Triangulation &triang, double theta, double ratio);

// The skeleton graph as it is built: the public secondary_skeleton and what building it needs.
struct skeleton_builder {
  secondary_skeleton s;
  std::vector<std::int64_t> cell_point;              // per cell id: its Voronoi vertex's point, -1
  std::vector<std::vector<std::uint32_t>> adjacency; // per point: its edges, in insertion order
  std::vector<std::vector<int>> edge_faces;          // per edge: its polygons
  std::vector<std::vector<int>> face_edges;          // per polygon: its edges
  std::vector<std::vector<int>> point_faces;         // per point: its polygons
  std::vector<std::vector<int>> components;          // per component: its polygons, as walked

  std::uint32_t point_of(cocone::Cell_handle c);
  std::uint32_t add_point(const cocone::Point &p);
  // The edge between a and b, or -1.
  std::int64_t find_edge(std::uint32_t a, std::uint32_t b) const;
  std::uint32_t add_edge(std::uint32_t a, std::uint32_t b);
  // Whether a and b are on polygons of one component (the legacy in_same_comp).
  bool in_same_component(std::uint32_t a, std::uint32_t b) const;
};

// add_u1_to_skel: a polygon per Voronoi facet inside the surface on U1, its edges, and the
// polygons' edge-connected components (through edges on at most two polygons).
step_task add_u1(const cocone::Triangulation &triang, skeleton_builder &b);
// add_u2_to_skel, from the core's U2 graph (its chains, in order): an edge from each chain's saddle
// point to its first cell (unless the saddle's two cells are in one component), then between
// consecutive cells, each with the radius of the tube around it.
step_task add_u2(const cocone::Graph &graph, skeleton_builder &b);
// do_star's areas: per component, the sum of its polygons fanned from their centroids.
step_task component_areas(skeleton_builder &b);

// Stages 5 and 6: the sheets, the helices and the output geometries, into r (which holds the
// skeleton). `ctx` makes the geometries. `stage`, when set, hears each phase begin ("sheets",
// "helices", "output") with an estimate of its units.
using stage_fn = std::function<void(const char *phase, std::size_t units)>;
step_task select(app &ctx, const secondary_params &p, secondary_structures &r,
                 stage_fn stage = {}); // by value: a coroutine keeps it

// Throws secondary_error when a skeleton's indices are out of range.
void check_skeleton(const secondary_skeleton &s);

// The eigenvector of a symmetric 3x3 matrix's smallest eigenvalue (Jacobi's method), its largest
// component positive.
std::array<double, 3> smallest_eigenvector(const double m[3][3]);

} // namespace cvc::secondary_detail

#endif // CVC_GEOMETRY_SECONDARY_SECONDARY_H
