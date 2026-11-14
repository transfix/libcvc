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

// u2.cpp and graph.cpp of VolumeRover's Skeletonization: U2, the unstable manifolds of the
// index-2 saddles of the distance function (circumcentres of Delaunay facets whose dual Voronoi
// edge they lie on), traced as chains of Voronoi vertices, and the graph made from those chains.
// The legacy debug files (<prefix>.no_target_VE.degen and friends, written to the current
// directory) are counted in the diagnostics instead.

#include "cocone.h"

namespace cvc::cocone {

namespace {

bool has_cospherical_pair_of_tetrahedra_dual_to_any_VE(const Triangulation &triang,
                                                       const vector<Cell_handle> &VF) {
  for (int i = 0; i < (int)VF.size(); i++) {
    const Cell_handle next = VF[std::size_t((i + 1) % ((int)VF.size()))];
    if (is_cospherical_pair(triang, Facet(VF[std::size_t(i)], VF[std::size_t(i)]->index(next))))
      return true;
  }
  return false;
}

// The VE of the VF (a ring of cells) that the ray from d through p crosses, other than the
// forbidden ones; the last one found, as in the legacy code.
bool find_target_VE(const vector<Cell_handle> &VF, const vector<Cell_handle> &forbidden_VE,
                    const Point &d, const Point &p, Cell_handle &c0, Cell_handle &c1) {
  bool found_target_VE = false;
  for (int i = 0; i < (int)VF.size(); i++) {
    const Cell_handle a = VF[std::size_t(i)], b = VF[std::size_t((i + 1) % ((int)VF.size()))];
    if ((int)forbidden_VE.size() == 2)
      if ((a->id == forbidden_VE[0]->id && b->id == forbidden_VE[1]->id) ||
          (a->id == forbidden_VE[1]->id && b->id == forbidden_VE[0]->id))
        continue;
    if ((int)forbidden_VE.size() == 1)
      if ((a->id == forbidden_VE[0]->id || b->id == forbidden_VE[0]->id))
        continue;
    if (does_intersect_ray3_seg3_in_plane(Ray(d, p), Segment(a->voronoi(), b->voronoi()))) {
      c0 = a;
      c1 = b;
      found_target_VE = true;
    }
  }
  return found_target_VE;
}

// ------------------------
// compute_nonVE_integral_curve
// ------------------------
// Follow the flow from the VV dual to e.first across the VF dual to e (driven from d), through
// the acceptor VFs of the transversal VEs it crosses, until it reaches a non-transversal VE: the
// VV it then flows to is dest. A walk with no bound on its length, so a coroutine: a unit per VF
// crossed.
step_task compute_nonVE_integral_curve(const Triangulation &triang, Point d, Edge e,
                                       Cell_handle &dest, COMPUTATION_STATUS &status,
                                       diagnostics &diag) {
  Cell_handle c = e.first;
  Point p = c->voronoi();
  // helps excluding the VE from intersection calculation on which p lies.
  vector<Cell_handle> forbidden_VE;
  forbidden_VE.push_back(c);
  status = ERROR_;
  for (std::size_t guard = 0;; ++guard) {
    SK_UNIT;
    if (guard > triang.num_cells()) {
      status = ERROR_; // a cycle: no flow crosses more VFs than there are cells
      co_return;
    }
    // create VF dual to e. store the cells.
    vector<Cell_handle> VF;
    Facet_circulator fcirc = triang.incident_facets(e);
    Facet_circulator begin = fcirc;
    bool infinite = false;
    do {
      if (triang.is_infinite((*fcirc).first)) {
        infinite = true;
        break;
      }
      VF.push_back((*fcirc).first);
      fcirc++;
    } while (fcirc != begin);
    if (infinite) {
      status = INFINITE_;
      co_return;
    }
    // 1. cospherical - not handled.
    if (has_cospherical_pair_of_tetrahedra_dual_to_any_VE(triang, VF)) {
      status = COSPH;
      co_return;
    }
    // 2. no target VE in the VF.
    Cell_handle c0, c1;
    if (!find_target_VE(VF, forbidden_VE, d, p, c0, c1)) {
      ++diag.no_target_VE;
      status = NOT_FOUND;
      co_return;
    }
    // 3. numerical instability: an incorrect intersection.
    bool is_correct_intersection = true;
    Point new_p = intersect_ray3_seg3(Ray(d, p), Segment(c0->voronoi(), c1->voronoi()),
                                      is_correct_intersection);
    if (!is_correct_intersection) {
      ++diag.incorrect_intersection;
      status = INTERSECT_PROBLEM;
      co_return;
    }
    // 4. the intersection is the starting point in the VF: a potential infinite loop.
    if (p == new_p) {
      ++diag.same_p;
      status = SAME_P;
      co_return;
    }
    p = new_p;
    // The VE on which p lies can be transversal or not. If not, the next VV the flow hits is
    // determined via the current VE: done.
    int id = c0->index(c1);
    if (!is_transversal_flow(Facet(c0, id))) {
      // p flows to c0 or c1 depending on the half-plane it is in.
      if (cg::orientation(c0->vertex((id + 1) % 4)->point(), c0->vertex((id + 2) % 4)->point(),
                          c0->vertex((id + 3) % 4)->point(), c0->vertex(id)->point()) ==
          cg::orientation(c0->vertex((id + 1) % 4)->point(), c0->vertex((id + 2) % 4)->point(),
                          c0->vertex((id + 3) % 4)->point(), p))
        dest = c0;
      else
        dest = c1;
      status = SUCCESS;
      co_return;
    }
    // If it is transversal, recur for the acceptor VF: the driver is the midpoint of the
    // acceptor DE. The legacy code called find_acceptor inside a CGAL_assertion(), which its
    // release builds compiled out, leaving the indices uninitialised.
    int uid = -1, vid = -1, wid = -1;
    if (!find_acceptor(c0, id, uid, vid, wid)) {
      status = ERROR_;
      co_return;
    }
    d = cg::midpoint(c0->vertex(uid)->point(), c0->vertex(vid)->point());
    e = Edge(c0, uid, vid);
    forbidden_VE.clear();
    forbidden_VE.push_back(c0);
    forbidden_VE.push_back(c1);
  }
}

} // namespace

// ------------------------
// compute_u2
// ------------------------
// Identify the index-2 saddles in the Voronoi diagram and compute their unstable manifolds.
// ------------------------
step_task compute_u2(Triangulation &triang, Graph &graph, diagnostics &diag) {
  vector<vector<Cell_handle>> chains;
  vector<COMPUTATION_STATUS> status_property;
  vector<Facet> start_of_chains;
  // set of index-2 saddle points.
  vector<Facet> i2set;
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); fit++) {
    SK_UNIT;
    Cell_handle c[2];
    int id[2];
    c[0] = (*fit).first;
    id[0] = (*fit).second;
    c[1] = c[0]->neighbor(id[0]);
    id[1] = c[1]->index(c[0]);
    c[0]->visited = false;
    c[1]->visited = false;
    if (triang.is_infinite(c[0]) || triang.is_infinite(c[1]))
      continue;
    if (!c[0]->VV_on_medax() || !c[1]->VV_on_medax())
      continue;
    if (!is_i2_saddle((*fit)))
      continue;
    i2set.push_back(Facet(c[0], id[0]));
  }
  // The order the saddles are traced in decides which pieces of chains that later cross the
  // surface are kept (a chain stops where it meets one traced before it). The legacy code took
  // them in its cells' memory order, so its graph depended on CGAL's allocation history; its
  // sort by circumradius (small to big, popped from the back) was switched off. Here they are
  // sorted that way, ties broken by the circumcentre and then the facet's points, all computed
  // from the facet's points in lexicographic order: a function of the point set, not of how its
  // cells are numbered.
  struct i2_key {
    double r2, cc[3];
    Point p[3];
    Facet f;
  };
  vector<i2_key> keyed;
  keyed.reserve(i2set.size());
  for (const Facet &f : i2set) {
    SK_UNIT;
    i2_key k;
    for (int i = 0; i < 3; ++i)
      k.p[i] = f.first->vertex((f.second + i + 1) % 4)->point();
    std::sort(k.p, k.p + 3);
    const Point c = cc_tr_3(k.p[0], k.p[1], k.p[2]);
    k.r2 = cg::to_double((c - k.p[0]) * (c - k.p[0]));
    k.cc[0] = c.x();
    k.cc[1] = c.y();
    k.cc[2] = c.z();
    k.f = f;
    keyed.push_back(k);
  }
  SK_RUN(sliced_sort(keyed, [](const i2_key &a, const i2_key &b) {
    if (a.r2 != b.r2)
      return a.r2 < b.r2;
    for (int i = 0; i < 3; ++i)
      if (a.cc[i] != b.cc[i])
        return a.cc[i] < b.cc[i];
    for (int i = 0; i < 3; ++i)
      if (a.p[i] != b.p[i])
        return a.p[i] < b.p[i];
    return false;
  }));
  vector<Facet> sorted_i2set;
  sorted_i2set.reserve(keyed.size());
  for (const i2_key &k : keyed)
    sorted_i2set.push_back(k.f);
  i2set = sorted_i2set;
  // number of chain components is twice the number of i2 saddle points.
  chains.resize(2 * i2set.size());
  status_property.resize(2 * i2set.size(), SUCCESS);
  start_of_chains.resize(2 * i2set.size());
  int i2cnt = -1;
  while (!sorted_i2set.empty()) {
    SK_UNIT;
    i2cnt++;
    Facet i2f = sorted_i2set.back();
    sorted_i2set.pop_back();
    vector<Cell_handle> subchains[2];
    subchains[0].push_back(i2f.first);
    subchains[1].push_back(i2f.first->neighbor(i2f.second));
    i2f.first->visited = true;
    i2f.first->neighbor(i2f.second)->visited = true;
    start_of_chains[std::size_t(2 * i2cnt)] = i2f;
    start_of_chains[std::size_t(2 * i2cnt + 1)] = i2f;
    // check if the VE holding the i2saddle point crosses the surface.
    if (i2f.first->outside != i2f.first->neighbor(i2f.second)->outside)
      status_property[std::size_t(2 * i2cnt)] = status_property[std::size_t(2 * i2cnt + 1)] = SURF;
    for (int i = 0; i < 2; i++) {
      const std::size_t ci = std::size_t(2 * i2cnt + i);
      bool current_subchain_is_already_traced = false;
      while (!subchains[i].empty()) {
        SK_UNIT;
        Cell_handle c = subchains[i].back();
        subchains[i].pop_back();
        // if the cell is at infinity continue.
        if (triang.is_infinite(c))
          continue;
        chains[ci].push_back(c);
        if (current_subchain_is_already_traced)
          continue;
        int out_cnt = 0;
        int ofid[2] = {-1, -1};
        // detect the outflows.
        for (int j = 0; j < 4; j++)
          if (is_outflow(Facet(c, j))) {
            if (out_cnt < 2)
              ofid[out_cnt] = j;
            ++out_cnt;
          }
        // if the flow is transversal: uid,vid correspond to the ACCEPTOR DE and uid,wid and
        // vid,wid to the DONOR DE, so angle(u,w,v) > 90 degrees.
        int uid = -1, vid = -1, wid = -1;
        bool flow_through_VF = false;
        // Follow the flow through the VE dual to (c, f) to the cell beyond it.
        auto follow_VE = [&](int f) {
          subchains[i].push_back(c->neighbor(f));
          if (c->neighbor(f)->visited)
            current_subchain_is_already_traced = true;
          c->neighbor(f)->visited = true;
          if (c->outside != c->neighbor(f)->outside)
            status_property[ci] = SURF;
        };
        if (out_cnt == 0) {
          // a maximum: the chain ends.
        } else if (out_cnt == 1) {
          // degenerate case: cospherical.
          if (is_cospherical_pair(triang, Facet(c, ofid[0])))
            continue;
          if (!is_transversal_flow(Facet(c, ofid[0])))
            follow_VE(ofid[0]);
          else {
            // The legacy code called find_acceptor inside a CGAL_assertion(), compiled out of
            // its release builds.
            find_acceptor(c, ofid[0], uid, vid, wid);
            flow_through_VF = true;
          }
        } else if (out_cnt == 2) {
          if (is_cospherical_pair(triang, Facet(c, ofid[0])))
            continue;
          if (is_cospherical_pair(triang, Facet(c, ofid[1])))
            continue;
          if (!is_transversal_flow(Facet(c, ofid[0])) & !is_transversal_flow(Facet(c, ofid[1]))) {
            // degenerate case: the legacy code only reported it.
          } else if (!is_transversal_flow(Facet(c, ofid[0])))
            follow_VE(ofid[0]);
          else if (!is_transversal_flow(Facet(c, ofid[1])))
            follow_VE(ofid[1]);
          else {
            // flow through the VF dual to the edge shared by both outflow facets.
            vertex_indices(ofid[0], ofid[1], uid, vid);
            wid = ofid[1]; // arbitrary assignment of wid.
            flow_through_VF = true;
          }
        } else {
          // more than two outflows: impossible for a non-degenerate cell; the chain stops.
        }
        if (flow_through_VF) {
          Point driver = cg::midpoint(c->vertex(uid)->point(), c->vertex(vid)->point());
          if (is_degenerate_VF(triang, c, ofid[0], uid, vid, driver, diag))
            continue;
          Cell_handle dest_c = nullptr;
          COMPUTATION_STATUS status = ERROR_;
          SK_RUN(compute_nonVE_integral_curve(triang, driver, Edge(c, uid, vid), dest_c, status,
                                              diag));
          if (status != SUCCESS) {
            status_property[ci] = status;
            ++diag.bad_chain;
            continue;
          }
          subchains[i].push_back(dest_c);
          if (dest_c->visited)
            current_subchain_is_already_traced = true;
          dest_c->visited = true;
          if (c->outside != dest_c->outside)
            status_property[ci] = SURF;
        }
      }
    }
  }
  // create the graph from the set of vertices and edges.
  SK_RUN(create_graph(graph, chains, status_property, start_of_chains));
}

// The graph of the chains: `chains' holds each chain's cells, in order; the edges join
// consecutive vertices. Only chains with status SUCCESS count, and duplicated vertices (at the
// start and end of chains) are merged.
step_task create_graph(Graph &graph, const vector<vector<Cell_handle>> &chains,
                       const vector<COMPUTATION_STATUS> &chains_property,
                       const vector<Facet> &start_of_chains) {
  vector<GVertex> &vert_list = graph.vert_list;
  vector<GEdge> &edge_list = graph.edge_list;
  vector<vector<int>> pure_chains;
  pure_chains.resize(chains.size());
  int current_pure_chain = -1;
  for (int i = 0; i < (int)chains.size(); i++) {
    SK_UNIT;
    if (chains_property[std::size_t(i)] != SUCCESS)
      continue;
    if ((int)chains[std::size_t(i)].size() == 0)
      continue;
    current_pure_chain++;
    vector<int> &pc = pure_chains[std::size_t(current_pure_chain)];
    Facet i2f = start_of_chains[std::size_t(i)];
    if (i2f.first->saddle_g_vid[i2f.second] == -1) {
      vert_list.push_back(GVertex(circumcenter(start_of_chains[std::size_t(i)])));
      GVertex &gv = vert_list.back();
      const int gid = (int)vert_list.size() - 1;
      gv.id = gid;
      gv.c = i2f.first;
      pc.push_back(gid);
      Cell_handle c[2];
      int id[2];
      c[0] = i2f.first;
      id[0] = i2f.second;
      c[1] = c[0]->neighbor(id[0]);
      id[1] = c[1]->index(c[0]);
      c[0]->saddle_g_vid[id[0]] = gid;
      c[1]->saddle_g_vid[id[1]] = gid;
      gv.set_out(c[0]->outside && c[1]->outside);
      // if either of the three VFs incident on the VE (dual to Facet(c[0], id[0])) is on um_i1,
      // this graph vertex is also on um_i1.
      int u = (id[0] + 1) % 4, v = (id[0] + 2) % 4, w = (id[0] + 3) % 4;
      if (c[0]->VF_on_um_i1(u, v) || c[0]->VF_on_um_i1(v, w) || c[0]->VF_on_um_i1(w, u))
        gv.set_on_um_i1(true);
      // collect the clusters the incident VFs fall into.
      if (c[0]->patch_id[u][v] != -1)
        gv.cluster_membership.push_back(c[0]->patch_id[u][v]);
      if (c[0]->patch_id[v][w] != -1 && c[0]->patch_id[v][w] != c[0]->patch_id[u][v])
        gv.cluster_membership.push_back(c[0]->patch_id[v][w]);
      if (c[0]->patch_id[w][u] != -1 && c[0]->patch_id[w][u] != c[0]->patch_id[u][v] &&
          c[0]->patch_id[w][u] != c[0]->patch_id[v][w])
        gv.cluster_membership.push_back(c[0]->patch_id[w][u]);
    } else {
      pc.push_back(i2f.first->saddle_g_vid[i2f.second]);
    }
    for (int j = 0; j < (int)chains[std::size_t(i)].size(); j++) {
      Cell_handle cj = chains[std::size_t(i)][std::size_t(j)];
      // if the cell is already included by another chain
      if (cj->g_vid != -1) {
        pc.push_back(cj->g_vid);
        continue;
      }
      // add its voronoi as a vertex in the graph
      vert_list.push_back(GVertex(cj->voronoi()));
      GVertex &gv = vert_list.back();
      const int gid = (int)vert_list.size() - 1;
      gv.id = gid;
      gv.c = cj;
      pc.push_back(gid);
      cj->g_vid = gid;
      gv.set_out(cj->outside);
      // keep the info if this cell also lies on um(i1).
      if (cj->VV_on_um_i1()) {
        gv.set_on_um_i1(true);
        for (int u = 0; u < 4; u++)
          for (int v = u + 1; v < 4; v++) {
            if (cj->patch_id[u][v] == -1)
              continue;
            bool found = false;
            for (int k : gv.cluster_membership)
              if (k == cj->patch_id[u][v])
                found = true;
            if (found)
              continue;
            gv.cluster_membership.push_back(cj->patch_id[u][v]);
          }
      }
    }
  }
  graph.set_nv((int)vert_list.size());
  for (int i = 0; i < (int)pure_chains.size(); i++) {
    SK_UNIT;
    const vector<int> &pc = pure_chains[std::size_t(i)];
    if (pc.empty())
      continue;
    for (int j = 0; j < (int)pc.size() - 1; j++) {
      const int a = pc[std::size_t(j)], b = pc[std::size_t(j + 1)];
      edge_list.push_back(GEdge(a, b));
      edge_list.back().id = (int)edge_list.size() - 1;
      edge_list.back().set_status(SUCCESS);
      // adjacency
      vert_list[std::size_t(a)].add_inc_vert(b);
      vert_list[std::size_t(b)].add_inc_vert(a);
      vert_list[std::size_t(a)].add_inc_edge((int)edge_list.size() - 1);
      vert_list[std::size_t(b)].add_inc_edge((int)edge_list.size() - 1);
    }
  }
  graph.set_ne((int)edge_list.size());
}

} // namespace cvc::cocone
