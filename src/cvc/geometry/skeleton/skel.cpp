/*
  Copyright 2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu>
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

// skel.cpp of VolumeRover's Skeletonization: the planar clusters of U1 (the sheets), their stars
// and areas, and the polylines of U2's graph (the strands).

#include "cocone.h"

namespace cvc::cocone {

Point Polygon::centroid() const {
  Vector c = cg::NULL_VECTOR;
  for (const Point &p : ordered_v_list)
    c = c + (p - cg::ORIGIN);
  return (cg::ORIGIN + (1. / (int)ordered_v_list.size()) * c);
}

double Polygon::area() const {
  Point c = centroid();
  double A = 0;
  const int n = (int)ordered_v_list.size();
  for (int i = 0; i < n; i++) {
    Triangle_3 t =
        Triangle_3(c, ordered_v_list[std::size_t(i)], ordered_v_list[std::size_t((i + 1) % n)]);
    A += std::sqrt(cg::to_double(t.squared_area()));
  }
  return A;
}

// Each connected set of U1's VFs (through VEs shared by exactly two of them) inside the surface
// is a cluster: its VFs as polygons, a weighted centroid and an area.
step_task cluster_planar_patches(Triangulation &triang, Skel &skel) {
  double max_pgn_width = -DBL_MAX;
  for (FEI eit = triang.finite_edges_begin(); eit != triang.finite_edges_end(); eit++) {
    SK_UNIT;
    Cell_handle c = (*eit).first;
    int uid = (*eit).second, vid = (*eit).third;
    // if already visited, continue.
    if (c->e_visited[uid][vid])
      continue;
    // start walk from a VF containing i1 saddle.
    if (!c->i1_saddle(uid, vid))
      continue;
    // for the purpose of protein structure, the outside patches are not in the skeleton.
    if (c->outside)
      continue;
    // mark the VF (dual to e) visited.
    mark_VF_visited(triang, c, uid, vid);
    // push this DE (dual VF) into a stack and start walk.
    vector<Edge> walk;
    walk.push_back((*eit));
    vector<Polygon> pgn_list;
    while (!walk.empty()) {
      SK_UNIT;
      Cell_handle cur_c = walk.back().first;
      int cur_uid = walk.back().second;
      int cur_vid = walk.back().third;
      walk.pop_back();
      // assign cluster id to this VF.
      set_patch_id(triang, cur_c, cur_uid, cur_vid, (int)skel.pl.size());
      // collect the VFs which are on u1 and are connected to the current VF (dual e) via a
      // common VE.
      Facet_circulator fcirc = triang.incident_facets(Edge(cur_c, cur_uid, cur_vid));
      Facet_circulator begin = fcirc;
      // create a polygon and store it in the cluster.
      Polygon pgn;
      do {
        Cell_handle new_c = (*fcirc).first;
        int new_fid = (*fcirc).second;
        pgn.ordered_v_list.push_back(new_c->voronoi());
        pgn.cell_list.push_back(new_c);
        // if this VF has all three incident VFs on um_i1 it is a nonmanifold edge .. do not go
        // through this edge.
        if (new_c->VF_on_um_i1((new_fid + 1) % 4, (new_fid + 2) % 4) &&
            new_c->VF_on_um_i1((new_fid + 2) % 4, (new_fid + 3) % 4) &&
            new_c->VF_on_um_i1((new_fid + 3) % 4, (new_fid + 1) % 4)) {
          fcirc++;
          continue;
        }
        int new_uid = -1, new_vid = -1;
        for (int k = 1; k < 4; k++) {
          if (new_c->vertex((new_fid + k) % 4)->id != cur_c->vertex(cur_uid)->id &&
              new_c->vertex((new_fid + k) % 4)->id != cur_c->vertex(cur_vid)->id) {
            vertex_indices(new_fid, (new_fid + k) % 4, new_uid, new_vid);
            break;
          }
        }
        int new_wid = 6 - new_fid - new_uid - new_vid;
        // look for new ones to add to it.
        if (new_c->VF_on_um_i1(new_uid, new_wid) && !new_c->e_visited[new_uid][new_wid]) {
          walk.push_back(Edge(new_c, new_uid, new_wid));
          mark_VF_visited(triang, new_c, new_uid, new_wid);
        }
        if (new_c->VF_on_um_i1(new_vid, new_wid) && !new_c->e_visited[new_vid][new_wid]) {
          walk.push_back(Edge(new_c, new_vid, new_wid));
          mark_VF_visited(triang, new_c, new_vid, new_wid);
        }
        fcirc++;
      } while (fcirc != begin);
      // the width of the polygon = the length of the dual Delaunay edge.
      double width = std::sqrt(
          cg::to_double((cur_c->vertex(cur_uid)->point() - cur_c->vertex(cur_vid)->point()) *
                        (cur_c->vertex(cur_uid)->point() - cur_c->vertex(cur_vid)->point())));
      pgn.width = width;
      pgn_list.push_back(std::move(pgn));
      if (width > max_pgn_width)
        max_pgn_width = width;
    }
    // a weighted centroid of the cluster.
    double total_area = 0;
    Vector centroid = cg::NULL_VECTOR;
    for (const Polygon &pg : pgn_list) {
      const double a = pg.area();
      total_area += a;
      centroid = centroid + a * (pg.centroid() - cg::ORIGIN);
    }
    skel.pl.push_back(std::move(pgn_list));
    skel.pl_C.push_back(cg::ORIGIN + (1. / total_area) * centroid);
    skel.pl_A.push_back(total_area);
    skel.pl_C_knotid.push_back(-1);
  }
  skel.max_pgn_width = max_pgn_width;
}

// The VVs on the boundary of planar clusters that link to something else: on two or more
// clusters, or on one cluster with a graph edge to a vertex of other clusters. A small cluster
// is drawn as a star from its centroid to these.
step_task star(const Triangulation &triang, const Graph &graph, Skel &skel) {
  skel.active_bdy.resize(skel.pl.size());
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); cit++) {
    SK_UNIT;
    if (!cit->VV_on_um_i1())
      continue;
    set<int> inc_clusters;
    for (int i = 0; i < 4; i++)
      for (int j = i + 1; j < 4; j++)
        if (cit->VF_on_um_i1(i, j)) {
          if (cit->patch_id[i][j] == -1)
            continue;
          inc_clusters.insert(cit->patch_id[i][j]);
        }
    if (inc_clusters.empty())
      continue;
    // the VV is on the boundary of two clusters: note it in all of them.
    if ((int)inc_clusters.size() > 1) {
      for (int k : inc_clusters)
        skel.active_bdy[std::size_t(k)].push_back(cit->voronoi());
      continue;
    }
    // on one cluster only: active if a graph edge from it reaches a vertex of other clusters.
    if (cit->g_vid == -1)
      continue;
    const GVertex &gv = graph.vert_list[std::size_t(cit->g_vid)];
    const vector<int> &C1 = gv.cluster_membership;
    bool is_active = false;
    for (int i = 0; i < gv.num_inc_vert; i++) {
      const vector<int> &C2 = graph.vert_list[std::size_t(gv.inc_vert(i))].cluster_membership;
      if (!is_there_any_common_element(C1, C2))
        is_active = true;
    }
    if (is_active)
      skel.active_bdy[std::size_t(*inc_clusters.begin())].push_back(cit->voronoi());
  }
}

// The clusters by area, largest first (ties in index order): the legacy selection sort, a pick
// per unit.
step_task sort_cluster_by_area(Skel &skel) {
  vector<bool> visited(skel.pl_A.size(), false);
  for (int i = 0; i < (int)skel.pl_A.size(); i++) {
    SK_UNIT;
    int ind = -1;
    double max = 0;
    for (int j = 0; j < (int)skel.pl_A.size(); j++) {
      if (visited[std::size_t(j)])
        continue;
      if (skel.pl_A[std::size_t(j)] > max) {
        max = skel.pl_A[std::size_t(j)];
        ind = j;
      }
    }
    if (ind == -1) {
      // every remaining area is 0 or NaN (the legacy code read past the array here): keep
      // them in index order.
      for (int j = 0; j < (int)skel.pl_A.size(); j++)
        if (!visited[std::size_t(j)]) {
          ind = j;
          break;
        }
    }
    visited[std::size_t(ind)] = true;
    skel.sorted_pl_id.push_back(ind);
  }
}

// Mark the big clusters: the first pl_cl_cnt by area, or (discard_by_threshold) those of at
// least threshold times the largest area.
step_task filter_small_clusters(Skel &skel, int pl_cl_cnt, double threshold,
                                bool discard_by_threshold) {
  skel.is_big_pl.assign(skel.sorted_pl_id.size(), true);
  if (skel.sorted_pl_id.empty())
    co_return;
  double max_area = skel.pl_A[std::size_t(skel.sorted_pl_id[0])];
  for (int i = 0; i < (int)skel.sorted_pl_id.size(); i++) {
    SK_UNIT;
    int cl_id = skel.sorted_pl_id[std::size_t(i)];
    if (discard_by_threshold) {
      if (skel.pl_A[std::size_t(cl_id)] < threshold * max_area)
        skel.is_big_pl[std::size_t(cl_id)] = false;
    } else {
      if (i >= pl_cl_cnt) {
        skel.is_big_pl[std::size_t(cl_id)] = false;
      } else {
        for (const Polygon &pg : skel.pl[std::size_t(cl_id)])
          for (Cell_handle c : pg.cell_list)
            c->set_big_pl(true);
      }
    }
  }
}

// The graph as a network of polylines: walks from each joint and leaf (degree other than 2)
// along vertices of degree 2, each ending at a joint or a leaf; stored in skel.L, shortest
// first.
step_task port_graph_to_skel(Graph &graph, Skel &skel) {
  vector<int> joints_and_leaves;
  for (int i = 0; i < graph.get_nv(); i++) {
    SK_UNIT;
    // the outside part of the graph is left out (the legacy __OUTSIDE__ was off).
    if (graph.vert_list[std::size_t(i)].out())
      continue;
    if (graph.vert_list[std::size_t(i)].num_inc_vert == 1 ||
        graph.vert_list[std::size_t(i)].num_inc_vert > 2)
      joints_and_leaves.push_back(i);
  }
  vector<vector<int>> polylines;
  while (!joints_and_leaves.empty()) {
    SK_UNIT;
    int g_vid = joints_and_leaves.back();
    graph.vert_list[std::size_t(g_vid)].visited = true;
    joints_and_leaves.pop_back();
    for (int i = 0; i < graph.vert_list[std::size_t(g_vid)].num_inc_vert; i++) {
      // polyline will store all the vertices visited in the current walk.
      vector<int> polyline;
      const GVertex &v =
          graph.vert_list[std::size_t(graph.vert_list[std::size_t(g_vid)].inc_vert(i))];
      // 0. v is already visited.
      if (v.visited)
        continue;
      // 1. v is another joint, or 2. a terminus.
      if (v.num_inc_vert > 2 || v.num_inc_vert == 1) {
        polyline.push_back(g_vid);
        polyline.push_back(v.id);
        polylines.push_back(polyline);
        continue;
      }
      // therefore v has two incident vertices.
      polyline.push_back(g_vid);
      vector<int> walk;
      walk.push_back(v.id);
      graph.vert_list[std::size_t(v.id)].visited = true;
      int p_vid = g_vid;
      while (!walk.empty()) {
        SK_UNIT;
        const GVertex &cur_v = graph.vert_list[std::size_t(walk.back())];
        walk.pop_back();
        polyline.push_back(cur_v.id);
        if (cur_v.num_inc_vert != 2)
          continue;
        int next_vid = cur_v.inc_vert(0) == p_vid ? cur_v.inc_vert(1) : cur_v.inc_vert(0);
        walk.push_back(next_vid);
        graph.vert_list[std::size_t(next_vid)].visited = true;
        p_vid = cur_v.id;
      }
      polylines.push_back(polyline);
    }
  }
  // A component that is a closed loop has no joint and no leaf, so the walks above never reach
  // it, and the legacy code dropped it (a torus's core circle gave no strand at all). Each such
  // loop is a closed polyline: from its lowest vertex, around, and back to it.
  for (int i = 0; i < graph.get_nv(); i++) {
    SK_UNIT;
    const GVertex &start = graph.vert_list[std::size_t(i)];
    if (start.visited || start.out() || start.num_inc_vert != 2)
      continue;
    vector<int> loop{i};
    graph.vert_list[std::size_t(i)].visited = true;
    int prev = i, cur = start.inc_vert(0);
    bool closed = false;
    while (true) {
      SK_UNIT;
      const GVertex &v = graph.vert_list[std::size_t(cur)];
      loop.push_back(cur);
      if (cur == i) {
        closed = true;
        break;
      }
      if (v.visited || v.num_inc_vert != 2)
        break; // not a pure loop after all (cannot happen for an unvisited degree-2 component)
      graph.vert_list[std::size_t(cur)].visited = true;
      const int next = v.inc_vert(0) == prev ? v.inc_vert(1) : v.inc_vert(0);
      prev = cur;
      cur = next;
    }
    if (closed && loop.size() > 3)
      polylines.push_back(loop);
  }
  // the polylines by length (the sum of squared segment lengths, as the legacy code measured).
  vector<double> lengths;
  for (const vector<int> &pl : polylines) {
    SK_UNIT;
    double length = 0;
    for (int j = 0; j < (int)pl.size() - 1; j++) {
      const GVertex &v1 = graph.vert_list[std::size_t(pl[std::size_t(j)])];
      const GVertex &v2 = graph.vert_list[std::size_t(pl[std::size_t(j + 1)])];
      length += cg::to_double((v1.point() - v2.point()) * (v1.point() - v2.point()));
    }
    lengths.push_back(length);
  }
  vector<int> sorted_length_chain_ids;
  vector<bool> visited(polylines.size(), false);
  for (int i = 0; i < (int)lengths.size(); i++) {
    SK_UNIT;
    int ind = -1;
    double min_length = DBL_MAX;
    for (int j = 0; j < (int)lengths.size(); j++) {
      if (visited[std::size_t(j)])
        continue;
      if (lengths[std::size_t(j)] < min_length) {
        ind = j;
        min_length = lengths[std::size_t(j)];
      }
    }
    if (ind == -1)
      continue; // "error in sorting lengths." (a NaN or infinite length)
    sorted_length_chain_ids.push_back(ind);
    visited[std::size_t(ind)] = true;
  }
  // store the sorted polylines in skel.
  skel.L.clear();
  for (int ch_id : sorted_length_chain_ids) {
    SK_UNIT;
    Polyline l;
    const vector<int> &pl = polylines[std::size_t(ch_id)];
    l.patch_id_list.resize(pl.size());
    for (int j = 0; j < (int)pl.size(); j++) {
      const GVertex &gv = graph.vert_list[std::size_t(pl[std::size_t(j)])];
      l.ordered_v_list.push_back(gv.point());
      l.cell_list.push_back(gv.c);
      for (int k : gv.cluster_membership)
        l.patch_id_list[std::size_t(j)].push_back(k);
    }
    skel.L.push_back(std::move(l));
    skel.L_invalid.push_back(false);
  }
}

} // namespace cvc::cocone
