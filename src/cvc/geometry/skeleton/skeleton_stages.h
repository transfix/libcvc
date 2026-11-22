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

// The skeleton's stages on the shared cocone core (../cocone/cocone.h), internal to libcvc
// (VOLROVER-LIBRARY-PORTS-ROADMAP.md §2). Ported from VolumeRover's src/Skeletonization (its medax,
// u1, u2, graph, skel and degen files); cocone.h's notes on what changed on the way in hold here
// too.

#ifndef CVC_GEOMETRY_SKELETON_SKELETON_STAGES_H
#define CVC_GEOMETRY_SKELETON_SKELETON_STAGES_H

#include "../cocone/cocone.h"

#include <cstddef>
#include <vector>

namespace cvc::cocone {

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

#endif // CVC_GEOMETRY_SKELETON_SKELETON_STAGES_H
