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

// cvc::cocone: the vendored cocone core and the skeleton stages built on it, internal to libcvc
// (VOLROVER-LIBRARY-PORTS-ROADMAP.md §2). Ported from VolumeRover's src/Skeletonization (its
// init, robust_cc, util, intersect, tcocone, rcocone, medax, hfn_util, u1, u2, graph, skel and
// degen files), whose cocone files TightCocone, PocketTunnel and Curation each copied; those ports
// reuse this core.
//
// What changed on the way in:
//   - CGAL's triangulation is cvc::tet_complex behind datastruct.h's adapter; CGAL:: calls are
//     cg:: ones with the same semantics;
//   - every long function is a coroutine (stepping.h), so the skeleton stepper runs it in slices;
//   - no process-wide state: the bounding box, the diagnostics and the parameters are the
//     caller's (skeleton_context); the CGAL failure-handler statics are gone (EPICK's
//     constructions never reported failure anyway);
//   - debug files written to the current directory are counted in Triangulation::diagnostics,
//     which the skeleton turns into warnings;
//   - five CGAL_assertion()s with side effects (has_vertex / find_acceptor filling the indices
//     they test) were compiled out of the legacy release builds, leaving the indices at -1; their
//     calls happen here unconditionally;
//   - functions nothing called (find_flow_direction, grow_maxima, identify_cospherical_neighbor,
//     the debug writers in op.cpp, PolyTess) stay behind.

#ifndef CVC_GEOMETRY_SKELETON_COCONE_H
#define CVC_GEOMETRY_SKELETON_COCONE_H

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

// ---- medax.cpp: the medial axis ----
step_task compute_medial_axis(Triangulation &triang, const double theta, const double ratio);

// ---- u1.cpp, u2.cpp, graph.cpp: the unstable manifolds of the index-1 and index-2 saddles ----
class GVertex {
public:
  GVertex() = default;
  explicit GVertex(Point p) : coord(p) {}
  void set_point(const Point &p) { coord = p; }
  Point point() const { return coord; }
  void set_on_um_i1(const bool &b) { v_on_um_i1 = b; }
  bool on_um_i1() const { return v_on_um_i1; }
  void add_inc_vert(const int i) {
    inc_vert_list.push_back(i);
    num_inc_vert++;
  }
  int inc_vert(int i) const { return inc_vert_list[std::size_t(i)]; }
  void add_inc_edge(const int i) {
    inc_edge_list.push_back(i);
    num_inc_edge++;
  }
  int inc_edge(int i) const { return inc_edge_list[std::size_t(i)]; }
  void set_out(const bool &b) { v_out = b; }
  bool out() const { return v_out; }
  int id = -1;
  bool visited = false;
  Cell_handle c = nullptr;
  int num_inc_vert = 0;
  int num_inc_edge = 0;
  vector<int> cluster_membership;

private:
  Point coord;
  bool v_on_um_i1 = false;
  bool v_out = false;
  vector<int> inc_vert_list;
  vector<int> inc_edge_list;
};

class GEdge {
public:
  GEdge() = default;
  GEdge(const int v1, const int v2) : endpoint{v1, v2} {}
  void set_endpoint(const int i, const int val) { endpoint[i] = val; }
  int get_endpoint(int i) const { return endpoint[i]; }
  void set_status(const COMPUTATION_STATUS &s) { status = s; }
  COMPUTATION_STATUS get_status() const { return status; }
  int id = -1;

private:
  int endpoint[2] = {-1, -1};
  COMPUTATION_STATUS status = SUCCESS;
};

class Graph {
public:
  void set_nv(const int n) { nv = n; }
  int get_nv() const { return nv; }
  void set_ne(const int n) { ne = n; }
  int get_ne() const { return ne; }
  vector<GVertex> vert_list;
  vector<GEdge> edge_list;

private:
  int nv = 0;
  int ne = 0;
};

step_task compute_u1(Triangulation &triang, diagnostics &diag);
step_task compute_u2(Triangulation &triang, Graph &graph, diagnostics &diag);
step_task create_graph(Graph &graph, const vector<vector<Cell_handle>> &chains,
                       const vector<COMPUTATION_STATUS> &chains_property,
                       const vector<Facet> &start_of_chains);

// ---- skel.cpp: the planar clusters (sheets) and polylines (strands) ----
class Polygon {
public:
  vector<Point> ordered_v_list;
  vector<Cell_handle> cell_list;
  double width = 0;
  Point centroid() const;
  double area() const;
};

class Polyline {
public:
  vector<Point> ordered_v_list;
  vector<vector<int>> patch_id_list; // the patches a vertex falls into (empty when free)
  vector<Cell_handle> cell_list;
};

class Skel {
public:
  double max_pgn_width = -DBL_MAX;
  vector<Polyline> L;
  vector<bool> L_invalid;
  vector<vector<Polygon>> pl;
  vector<Point> pl_C;
  vector<double> pl_A;
  vector<int> sorted_pl_id;
  vector<bool> is_big_pl;
  vector<vector<Point>> active_bdy;
  vector<int> pl_C_knotid;
};

step_task cluster_planar_patches(Triangulation &triang, Skel &skel);
step_task star(const Triangulation &triang, const Graph &graph, Skel &skel);
step_task sort_cluster_by_area(Skel &skel);
step_task filter_small_clusters(Skel &skel, int pl_cl_cnt, double threshold,
                                bool discard_by_threshold);
step_task port_graph_to_skel(Graph &graph, Skel &skel);

} // namespace cvc::cocone

#endif // CVC_GEOMETRY_SKELETON_COCONE_H
