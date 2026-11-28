/*
  Copyright 2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu> (the cocone and skeleton code)
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

// cvc::cocone: the vendored cocone core, internal to libcvc (VOLROVER-LIBRARY-PORTS-ROADMAP.md §2,
// "One cocone core, not four"). Ported from VolumeRover's src/Skeletonization (its init,
// robust_cc, util, intersect, tcocone, rcocone and hfn_util files), whose cocone files
// TightCocone, PocketTunnel and Curation each copied with 10-25 lines changed. The ports share
// this one copy: the skeleton's stages are in ../skeleton (skeleton_stages.h), PocketTunnel's in
// ../pocket_tunnel. PocketTunnel's copies of init.cpp and tcocone.cpp are this code; its util.cpp
// adds only cell_volume, which is here. Every port's stepper triangulates its points with
// triangulate.h's one step; tight_cocone_stages.h has TightCocone's front end (Robust Cocone, the
// triangulation, Tight Cocone's marking) and its surface, for cvc::tight_cocone and the ports that
// start from them (surface curation, super-secondary structures).
//
// What changed on the way in:
//   - CGAL's triangulation is cvc::tet_complex behind datastruct.h's adapter; CGAL:: calls are
//     cg:: ones with the same semantics;
//   - every long function is a coroutine (stepping.h), so a port's stepper runs it in slices;
//   - no process-wide state: the bounding box, the diagnostics and the parameters are the
//     caller's (each port's stepper context); the CGAL failure-handler statics are gone (EPICK's
//     constructions never reported failure anyway);
//   - debug files written to the current directory are counted in a diagnostics, which the ports
//     turn into warnings;
//   - five CGAL_assertion()s with side effects (has_vertex / find_acceptor filling the indices
//     they test) were compiled out of the legacy release builds, leaving the indices at -1; their
//     calls happen here unconditionally;
//   - functions nothing called (find_flow_direction, grow_maxima, identify_cospherical_neighbor,
//     the debug writers in op.cpp, PolyTess) stay behind.

#ifndef CVC_GEOMETRY_COCONE_COCONE_H
#define CVC_GEOMETRY_COCONE_COCONE_H

#include "datastruct.h"
#include "stepping.h"

#include <cstddef>
#include <vector>

namespace cvc::cocone {

// What the legacy code wrote to debug files or printed, counted instead.
struct diagnostics {
  std::size_t dirty_cells = 0;            // circumcentre or radius not finite (init)
  std::size_t degenerate_VF = 0;          // degen.cpp's *.degen_VF
  std::size_t no_target_VE = 0;           // u2's *.no_target_VE.degen
  std::size_t incorrect_intersection = 0; // u2's *.incorrect_intersection.degen
  std::size_t same_p = 0;                 // u2's *.same_p.degen
  std::size_t degenerate_polygon = 0;     // intersect.cpp's "+"
  std::size_t bad_chain = 0;              // u2's chains that stopped (COSPH, NOT_FOUND, ...)
};

// The work a step does, for progress: `done` units of about `total` in the current stage.
struct progress_counter {
  std::size_t done = 0, total = 1;
};

// ---- init.cpp (and robust_cc.cpp) ----
step_task initialize(Triangulation &triang);
step_task compute_voronoi_vertex_and_cell_radius(Triangulation &triang, diagnostics &diag);
Point nondg_voronoi_point(const Point &a, const Point &b, const Point &c, const Point &d,
                          bool &is_correct_computation);
Point dg_voronoi_point(const Point &a, const Point &b, const Point &c, const Point &d,
                       bool &is_correct_computation);
Point nondg_cc_tr_3(const Point &a, const Point &b, const Point &c, bool &is_correct_computation);
Point cc_tr_3(const Point &a, const Point &b, const Point &c);
double sq_cr_tr_3(const Point &a, const Point &b, const Point &c);
Point circumcenter(const Facet &f);
double circumradius(const Facet &f);

// ---- util.cpp (with intersect.cpp and hfn_util.cpp's predicates) ----
double cosine(const Vector &v, const Vector &w);
void normalize(Vector &v);
double length_of_seg(const Segment &s);
bool is_obtuse(const Point &p0, const Point &p1, const Point &p2);
double cell_volume(const Cell_handle &c); // |volume| of a finite cell (PocketTunnel's util.cpp)
int find_third_vertex_index(const Facet &f, Vertex_handle v, Vertex_handle w);
int edge_index(const int facet_index, const int first_vertex_index, const int second_vertex_index);
void vertex_indices(const int facet_index, const int edge_index, int &first_vertex,
                    int &second_vertex);
bool is_outside_bounding_box(const Point &p, const vector<double> &bounding_box);
bool is_cospherical_pair(const Triangulation &triang, const Facet &f);
void mark_VF_on_u1(Triangulation &triang, Cell_handle &c, int uid, int vid);
void mark_VF_visited(Triangulation &triang, Cell_handle &c, int uid, int vid);
void set_patch_id(Triangulation &triang, Cell_handle &c, int uid, int vid, const int &id);
bool is_surf_VF(const Triangulation &triang, const Cell_handle &c, const int uid, const int vid);
bool is_inf_VF(const Triangulation &triang, const Cell_handle &c, const int uid, const int vid);
bool is_there_any_common_element(const vector<int> &vec1, const vector<int> &vec2);
bool does_intersect_ray3_seg3_in_plane(const Ray &r, const Segment &s);
Point intersect_ray3_seg3(const Ray &r, const Segment &s, bool &is_correct_intersection);
bool does_intersect_convex_polygon_segment_3_in_3d(const vector<Point> &conv_poly, const Segment &s,
                                                   diagnostics &diag);
bool is_maxima(const Cell_handle &c);
bool is_outflow(const Facet &f);
bool is_transversal_flow(const Facet &f);
bool find_acceptor(const Cell_handle &c, const int &id, int &uid, int &vid, int &wid);
bool is_i2_saddle(const Facet &f);
bool is_i1_saddle(const Edge &e, const Triangulation &triang, diagnostics &diag);
bool is_degenerate_VF(const Triangulation &triang, const Cell_handle &c, const int &fid,
                      const int &uid, const int &vid, const Point &d, diagnostics &diag);

// ---- tcocone.cpp: Tight Cocone ----
step_task tcocone(const double DEFAULT_ANGLE, const double DEFAULT_SHARP, const double DEFAULT_FLAT,
                  const double DEFAULT_RATIO, Triangulation &triang);

// ---- rcocone.cpp: Robust Cocone, the optional pre-filter: the points it keeps ----
step_task robust_cocone(const double bb_ratio, const double theta_ff, const double theta_if,
                        Triangulation &triang, std::vector<Point> &result);

} // namespace cvc::cocone

#endif // CVC_GEOMETRY_COCONE_COCONE_H
