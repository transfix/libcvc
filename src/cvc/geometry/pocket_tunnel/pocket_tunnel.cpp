/*
  Copyright 2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu> (PocketTunnel)
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

// cvc::pocket_tunnel and pocket_tunnel_stepper (cvc/geometry/pocket_tunnel.h): PocketTunnel's
// pocket_tunnel_fromsurf() as one coroutine over the shared cocone core (../cocone/cocone.h), with
// its own stages ported from the legacy files:
//   - smax.cpp: compute_smax, grow_maximum, club_contiguous_segment, the cluster volumes and their
//     sort;
//   - handle.cpp: detect_handle, the mouths;
//   - op.cpp: convert_pocket_tunnel_to_rawc_geometry, the output.
// The legacy init.cpp, tcocone.cpp and the hfn_util.cpp predicates it uses (is_maxima,
// is_outflow) are the cocone core's; its robust_cc.cpp and rcocone.cpp were never called by
// pocket_tunnel_fromsurf (its Robust Cocone constants were unused), so they stay out.
//
// What changed on the way in:
//   - the clusters were a std::map<int, cell_cluster> of linked lists, the second list relabelled
//     on every merge; here a union-find over cell ids (by size, with path halving), so no merge
//     walks a whole cluster. A cluster's representative is never output, so the result is the
//     same;
//   - the volumes were summed by a linear search for each cell's cluster among those seen, and
//     sorted by repeatedly picking the largest left: both quadratic. Here a table and a stable
//     merge sort, which orders ties as the legacy did (by the first cell met);
//   - the output scanned every facet once for each feature shown; here once in all;
//   - a facet faces away from its cluster's cell by an exact orientation test, where the legacy
//     compared a determinant in doubles with 0;
//   - progress went to stderr and debug files to debug_output/; here a stepper's phase and
//     progress, and warnings;
//   - the legacy read the points as floats; here they stay doubles.

#include "../cocone/cocone.h"
#include "../cocone/triangulate.h"
#include "pocket_tunnel_marked.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cvc/geometry/pocket_tunnel.h>
#include <cvc/geometry/pocket_tunnel_params.h>
#include <optional>

namespace cvc {

const char *to_string(pocket_kind k) {
  switch (k) {
  case pocket_kind::void_:
    return "void";
  case pocket_kind::pocket:
    return "pocket";
  case pocket_kind::tunnel:
    return "tunnel";
  }
  return "?";
}

namespace {

std::string fmt_count(const char *what, std::size_t n) {
  char buf[256];
  std::snprintf(buf, sizeof buf, what, n);
  return buf;
}

// The legacy colours (op.cpp), as codes, and their RGB.
enum colour_code : std::uint8_t { COL_WHITE, COL_YELLOW, COL_RED, COL_PURPLE, COL_BLUE, COL_GREEN };
geometry::color_t rgb(std::uint8_t c) {
  switch (c) {
  case COL_YELLOW:
    return geometry::color_t{{1.0, 1.0, 0.0}};
  case COL_RED:
    return geometry::color_t{{1.0, 0.0, 0.0}};
  case COL_PURPLE:
    return geometry::color_t{{1.0, 0.0, 1.0}};
  case COL_BLUE:
    return geometry::color_t{{0.0, 0.0, 1.0}};
  case COL_GREEN:
    return geometry::color_t{{0.0, 1.0, 0.0}};
  default:
    return geometry::color_t{{1.0, 1.0, 1.0}};
  }
}

// The legacy cell_cluster map, by cell id: a union-find over the cells, and each cell's fields.
struct cluster_set {
  std::vector<int> parent, size;
  std::vector<char> in_cluster; // the cell flows into a maximum (it is in a grown cluster)
  // The cell's cluster's side of the surface: the side of the maximum it was grown from, and
  // true for a cell in no cluster (cell_cluster's default).
  std::vector<char> outside;
  std::vector<int> mouth_cnt, maxima; // by root

  void reset(std::size_t n) {
    parent.resize(n);
    for (std::size_t i = 0; i < n; ++i)
      parent[i] = int(i);
    size.assign(n, 1);
    in_cluster.assign(n, 0);
    outside.assign(n, 1);
    mouth_cnt.assign(n, 0);
    maxima.assign(n, 0);
  }
  int find(int i) {
    while (parent[std::size_t(i)] != i) {
      parent[std::size_t(i)] = parent[std::size_t(parent[std::size_t(i)])];
      i = parent[std::size_t(i)];
    }
    return i;
  }
  // Merges the clusters of a and b (merge_cluster, add_cell_to_cluster).
  void unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a == b)
      return;
    if (size[std::size_t(a)] < size[std::size_t(b)])
      std::swap(a, b);
    parent[std::size_t(b)] = a;
    size[std::size_t(a)] += size[std::size_t(b)];
    mouth_cnt[std::size_t(a)] += mouth_cnt[std::size_t(b)];
    maxima[std::size_t(a)] += maxima[std::size_t(b)];
  }
};

} // namespace

// The per-call context of §1.3: what the legacy pocket_tunnel_fromsurf() kept in its locals, and
// the coroutine that runs it.
struct pocket_tunnel_stepper::impl : cocone::stage_progress {
  std::vector<std::array<double, 3>> points;
  std::optional<geometry> source; // the geometry to read the points from, until they are read
  pocket_tunnel_params params;
  std::optional<pocket_tunnel_detail::marking> given; // Tight Cocone's marking, for the legacy A/B

  std::atomic<bool> cancel{false};
  bool finished = false, failed = false;
  std::exception_ptr error;

  // the context
  cocone::Triangulation triang;
  cocone::diagnostics diag;
  cluster_set cl;
  std::vector<int> cluster_roots;     // the outside clusters, in the order of their first cell
  std::vector<double> cluster_volume; // by position in cluster_roots
  std::vector<std::size_t> cluster_cells;
  std::vector<int> sorted; // positions in cluster_roots, the largest volume first
  std::size_t duplicates = 0;
  pocket_tunnels result;

  cocone::step_task task;

  cocone::step_task run();
  cocone::step_task grow_maximum(cocone::Cell_handle c_max);
  cocone::step_task club_contiguous_segment();
  cocone::step_task cluster_volumes();
  cocone::step_task detect_handle();
  cocone::step_task output();
  cocone::step_task apply_marking();
};

// ---------------------------------------------------------
// grow_maximum (smax.cpp)
// -------------------------
// The stable manifold of the maximum c_max: the cells whose flow, across the facets the
// circumcentres' flow crosses, ends in it. A unit is a facet taken from the boundary stack.
// ---------------------------------------------------------
cocone::step_task pocket_tunnel_stepper::impl::grow_maximum(cocone::Cell_handle c_max) {
  using namespace cvc::cocone;
  // mark it visited.
  c_max->visited = true;
  cl.in_cluster[std::size_t(c_max->id)] = 1;
  cl.outside[std::size_t(c_max->id)] = c_max->outside;
  cl.maxima[std::size_t(cl.find(c_max->id))] += 1;

  // Now grow the maximum through the other tetrahedra.
  vector<Facet> bdy_stack;
  for (int i = 0; i < 4; i++)
    bdy_stack.push_back(Facet(c_max, i));
  while (!bdy_stack.empty()) {
    SK_UNIT;
    Cell_handle old_c = bdy_stack.back().first;
    int old_id = bdy_stack.back().second;
    bdy_stack.pop_back();

    Cell_handle new_c = old_c->neighbor(old_id);
    int new_id = new_c->index(old_c);

    // If new_c is infinite there is no flow to check.
    if (triang.is_infinite(new_c))
      continue;
    // If new_c is already visited, continue.
    if (new_c->visited)
      continue;

    // Collect new_c only if new_c flows into old_c via the facet in between.
    if (is_outflow(Facet(new_c, new_id))) {
      if (is_outflow(Facet(old_c, old_id)))
        continue;
      new_c->visited = true;
      cl.in_cluster[std::size_t(new_c->id)] = 1;
      cl.outside[std::size_t(new_c->id)] = c_max->outside;

      // put the cells accessible via new_c into bdy_stack.
      for (int i = 1; i < 4; i++) {
        if (new_c->neighbor((new_id + i) % 4)->visited)
          continue;
        bdy_stack.push_back(Facet(new_c, (new_id + i) % 4));
      }
      // put new_c into the current cluster.
      cl.unite(c_max->id, new_c->id);
    }
  }
}

// ---------------------------------------------------------
// club_contiguous_segment (smax.cpp)
// ------------------------------------
// Merge the clusters outside the surface that share a facet. A unit is a finite facet.
// ---------------------------------------------------------
cocone::step_task pocket_tunnel_stepper::impl::club_contiguous_segment() {
  using namespace cvc::cocone;
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); fit++) {
    SK_UNIT;
    Cell_handle c[2] = {(*fit).first, (*fit).first->neighbor((*fit).second)};
    const std::size_t i0 = std::size_t(c[0]->id), i1 = std::size_t(c[1]->id);
    // if the two adjacent cells belong to the same cluster, continue.
    if (cl.find(c[0]->id) == cl.find(c[1]->id))
      continue;
    // if any one of them is not in any cluster, continue.
    if (!cl.in_cluster[i0] || !cl.in_cluster[i1])
      continue;
    // if any of the clusters is inside, continue.
    if (!cl.outside[i0] || !cl.outside[i1])
      continue;
    // merge the two clusters.
    cl.unite(c[0]->id, c[1]->id);
  }
}

// ---------------------------------------------------------
// calc_cluster_volume_and_store_with_cluster_rep (smax.cpp)
// -----------------------------------------------------------
// Each outside cluster's volume (and its cells), the clusters in the order of their first finite
// cell. A unit is a finite cell.
// ---------------------------------------------------------
cocone::step_task pocket_tunnel_stepper::impl::cluster_volumes() {
  using namespace cvc::cocone;
  std::vector<int> pos(triang.num_cells(), -1); // by root
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); cit++) {
    SK_UNIT;
    const std::size_t id = std::size_t(cit->id);
    if (!cl.in_cluster[id])
      continue;
    if (!cl.outside[id])
      continue;
    const double volume = cell_volume(cit);
    const int rep = cl.find(cit->id);
    int &p = pos[std::size_t(rep)];
    if (p < 0) {
      p = int(cluster_roots.size());
      cluster_roots.push_back(rep);
      cluster_volume.push_back(0.0);
      cluster_cells.push_back(0);
    }
    cluster_volume[std::size_t(p)] += volume;
    cluster_cells[std::size_t(p)] += 1;
  }
}

// ---------------------------------------------------------
// detect_handle (handle.cpp)
// ----------------------------
// A facet belongs to a mouth if both its cells are outside and one of them is in a cluster
// while the other is not (it flows away to infinity). A mouth is a patch of such facets of one
// cluster, connected across edges; each adds one to its cluster's mouth count. A unit is a
// finite facet, or a facet met around an edge while a mouth grows.
// ---------------------------------------------------------
cocone::step_task pocket_tunnel_stepper::impl::detect_handle() {
  using namespace cvc::cocone;
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); fit++) {
    SK_UNIT;
    Cell_handle c[2] = {(*fit).first, (*fit).first->neighbor((*fit).second)};
    int id[2] = {c[0]->index(c[1]), c[1]->index(c[0])};
    c[0]->f_visited[id[0]] = false;
    c[1]->f_visited[id[1]] = false;
  }

  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); fit++) {
    SK_UNIT;
    Cell_handle c[2] = {(*fit).first, (*fit).first->neighbor((*fit).second)};
    int id[2] = {c[0]->index(c[1]), c[1]->index(c[0])};

    if (c[0]->f_visited[id[0]])
      continue;
    // if any adjacent cell is inside, continue.
    if (!c[0]->outside || !c[1]->outside)
      continue;
    // if both the adjacent cells are in a cluster, or neither is, continue.
    if (cl.in_cluster[std::size_t(c[0]->id)] == cl.in_cluster[std::size_t(c[1]->id)])
      continue;

    // the cluster is the one of the cell which is in a cluster.
    const int cl_id = cl.in_cluster[std::size_t(c[0]->id)] ? cl.find(c[0]->id) : cl.find(c[1]->id);
    vector<Facet> mouth;
    mouth.push_back((*fit));
    c[0]->f_visited[id[0]] = true;
    c[1]->f_visited[id[1]] = true;

    while (!mouth.empty()) {
      Facet f = mouth.back();
      mouth.pop_back();

      // circulate around each edge to collect the facets of the same mouth.
      Cell_handle _c = f.first;
      int _id = f.second;
      for (int i = 1; i < 4; i++) {
        int u = (_id + i) % 4, v = (_id + i + 1) % 4;
        if (i == 3)
          v = (_id + 1) % 4;
        Facet_circulator fcirc = triang.incident_facets(Edge(_c, u, v));
        Facet_circulator begin = fcirc;
        do {
          SK_UNIT;
          Cell_handle cur_c[2] = {(*fcirc).first, (*fcirc).first->neighbor((*fcirc).second)};
          int cur_id[2] = {(*fcirc).second, cur_c[1]->index(cur_c[0])};
          fcirc++;
          // already in a mouth.
          if (cur_c[0]->f_visited[cur_id[0]])
            continue;
          // if any adjacent cell is inside, continue.
          if (!cur_c[0]->outside || !cur_c[1]->outside)
            continue;
          // if both the adjacent cells are in a cluster, or neither is, continue.
          if (cl.in_cluster[std::size_t(cur_c[0]->id)] == cl.in_cluster[std::size_t(cur_c[1]->id)])
            continue;
          const int cur_cl_id = cl.in_cluster[std::size_t(cur_c[0]->id)] ? cl.find(cur_c[0]->id)
                                                                         : cl.find(cur_c[1]->id);
          if (cur_cl_id != cl_id)
            continue;

          // push this facet to the mouth.
          mouth.push_back(Facet(cur_c[0], cur_id[0]));
          cur_c[0]->f_visited[cur_id[0]] = true;
          cur_c[1]->f_visited[cur_id[1]] = true;
        } while (fcirc != begin);
      }
    }
    cl.mouth_cnt[std::size_t(cl.find(cl_id))]++;
  }
}

cocone::step_task pocket_tunnel_stepper::impl::run() {
  using namespace cvc::cocone;
  // ---- the input and the parameters ----
  {
    const std::vector<std::string> problems = params_validate(params);
    if (!problems.empty()) {
      std::string m = "pocket_tunnel: invalid parameters: ";
      for (std::size_t i = 0; i < problems.size(); ++i)
        m += (i ? "; " : "") + problems[i];
      throw pocket_tunnel_error(m);
    }
  }
  if (!tet_complex_available())
    throw pocket_tunnel_error("pocket_tunnel: libcvc was built without Shewchuk's predicates "
                              "(CVC_WITH_SHEWCHUK), which the triangulation needs");
  if (source) {
    const geometry::points_t &gp = source->const_points();
    points.reserve(gp.size());
    for (std::size_t i = 0; i < gp.size(); ++i) {
      points.push_back({gp[i][0], gp[i][1], gp[i][2]});
      if (i % 1024 == 1023)
        SK_UNIT;
    }
    source.reset();
  }
  const pocket_tunnel_params &P = params;

  // ---- 1. the Delaunay triangulation ----
  SK_RUN(triangulate<pocket_tunnel_error>(std::exchange(points, {}), "pocket_tunnel", triang,
                                          *this, 0.0, 0.25, duplicates));
  double at = 0.27;
  const std::size_t C = triang.num_cells(), V = triang.number_of_vertices();

  // ---- the attributes, the Voronoi vertices and the cell radii ----
  stage("voronoi", at, at + 0.05, C + V);
  SK_RUN(initialize(triang));
  SK_RUN(compute_voronoi_vertex_and_cell_radius(triang, diag));
  at += 0.05;

  // ---- 2. Tight Cocone ----
  stage("tight cocone", at, at + 0.35, 16 * C);
  if (given)
    SK_RUN(apply_marking());
  else
    SK_RUN(tcocone(P.cocone_phi, P.sharp_phi, P.flat_phi, P.flatness_ratio, triang));
  at += 0.35;

  // ---- 3. S_max: the maxima's stable manifolds (compute_smax) ----
  stage("maxima", at, at + 0.10, 2 * C);
  cl.reset(C);
  for (ACI cit = triang.all_cells_begin(); cit != triang.all_cells_end(); cit++) {
    cit->visited = false;
    if (std::size_t(cit->id) % 64 == 63)
      SK_UNIT;
  }
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); cit++) {
    SK_UNIT;
    if (!is_maxima(cit))
      continue;
    SK_RUN(grow_maximum(cit));
  }
  at += 0.10;
  stage("merging", at, at + 0.05, 2 * C);
  SK_RUN(club_contiguous_segment());
  at += 0.05;
  stage("volumes", at, at + 0.03, C);
  SK_RUN(cluster_volumes());
  at += 0.03;
  stage("sorting", at, at + 0.02, cluster_roots.size() / 16 + 1);
  SK_UNIT; // a sort of fewer than 64 clusters yields nowhere else
  sorted.resize(cluster_roots.size());
  for (std::size_t i = 0; i < sorted.size(); ++i)
    sorted[i] = int(i);
  SK_RUN(sliced_sort(sorted, [this](int a, int b) {
    return cluster_volume[std::size_t(a)] > cluster_volume[std::size_t(b)];
  }));
  at += 0.02;

  // ---- 4. the mouths ----
  stage("mouths", at, at + 0.08, 3 * C);
  SK_RUN(detect_handle());
  at += 0.08;

  // ---- 5. the output ----
  stage("output", at, 1.0, 2 * C + triang.complex().num_points());
  SK_RUN(output());
}

// Tight Cocone's marking from `given` (pocket_tunnel_marked.h): a finite cell is outside when
// listed, an infinite one always (as walk_wt leaves them); a finite facet's cocone flag is set on
// both its sides when listed, every other flag cleared. A unit is a cell or a finite facet.
cocone::step_task pocket_tunnel_stepper::impl::apply_marking() {
  using namespace cvc::cocone;
  std::size_t cells = 0, facets = 0;
  for (ACI cit = triang.all_cells_begin(); cit != triang.all_cells_end(); cit++) {
    SK_UNIT;
    for (int i = 0; i < 4; ++i)
      cit->set_cocone_flag(i, false);
    cit->outside = triang.is_infinite(cit);
    if (cit->outside)
      continue;
    std::array<int, 4> k = {cit->vertex(0)->tc, cit->vertex(1)->tc, cit->vertex(2)->tc,
                            cit->vertex(3)->tc};
    std::sort(k.begin(), k.end());
    if (given->outside.count(k)) {
      cit->outside = true;
      ++cells;
    }
  }
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); fit++) {
    SK_UNIT;
    Cell_handle c = (*fit).first;
    const int i = (*fit).second;
    std::array<int, 3> k = {c->vertex((i + 1) % 4)->tc, c->vertex((i + 2) % 4)->tc,
                            c->vertex((i + 3) % 4)->tc};
    std::sort(k.begin(), k.end());
    if (!given->cocone.count(k))
      continue;
    c->set_cocone_flag(i, true);
    c->neighbor(i)->set_cocone_flag(c->neighbor(i)->index(c), true);
    ++facets;
  }
  if (cells != given->outside.size() || facets != given->cocone.size())
    throw pocket_tunnel_error(
        "pocket_tunnel: the given marking names " +
        std::to_string(given->outside.size() - cells + given->cocone.size() - facets) +
        " cells or facets that are not in the triangulation");
}

// convert_pocket_tunnel_to_rawc_geometry (op.cpp): the features, and the shown ones' facets.
cocone::step_task pocket_tunnel_stepper::impl::output() {
  using namespace cvc::cocone;
  pocket_tunnels &r = result;
  const tet_complex &tc = triang.complex();
  r.points = tc.num_vertices();

  // Every outside cluster, the largest first, with its kind.
  std::vector<int> rank_of_root(triang.num_cells(), -1); // shown rank, by cluster root
  r.features.reserve(sorted.size());
  for (std::size_t k = 0; k < sorted.size(); ++k) {
    const std::size_t p = std::size_t(sorted[k]);
    const int root = cluster_roots[p];
    pocket_feature f;
    f.mouths = cl.mouth_cnt[std::size_t(root)];
    f.kind = f.mouths < 1   ? pocket_kind::void_
             : f.mouths > 1 ? pocket_kind::tunnel
                            : pocket_kind::pocket;
    f.volume = cluster_volume[p];
    f.cells = cluster_cells[p];
    f.maxima = std::size_t(cl.maxima[std::size_t(root)]);
    (f.kind == pocket_kind::void_    ? r.voids
     : f.kind == pocket_kind::tunnel ? r.tunnels
                                     : r.pockets)++;
    r.features.push_back(f);
  }
  // Shown: the largest tunnels and voids, then the largest pockets.
  std::vector<int> shown; // feature indices, in rank order
  for (int pass = 0; pass < 2; ++pass) {
    const int want = pass == 0 ? params.tunnels : params.pockets;
    int got = 0;
    for (std::size_t k = 0; k < r.features.size() && got < want; ++k) {
      const bool is_pocket = r.features[k].kind == pocket_kind::pocket;
      if (is_pocket != (pass == 1))
        continue;
      r.features[k].shown = int(shown.size());
      shown.push_back(int(k));
      ++got;
    }
  }
  for (std::size_t k = 0; k < sorted.size(); ++k)
    if (r.features[k].shown >= 0)
      rank_of_root[std::size_t(cluster_roots[std::size_t(sorted[k])])] = r.features[k].shown;

  // The shown features' boundary facets, a list per feature in facet order, each facing away
  // from the feature's cell.
  struct facet_out {
    int v[3];
    bool on_surface;
  };
  std::vector<std::vector<facet_out>> facets(shown.size());
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); fit++) {
    SK_UNIT;
    Cell_handle c[2] = {(*fit).first, (*fit).first->neighbor((*fit).second)};
    int id[2] = {c[0]->index(c[1]), c[1]->index(c[0])};
    const int root[2] = {cl.find(c[0]->id), cl.find(c[1]->id)};
    if (root[0] == root[1])
      continue;
    for (int s = 0; s < 2; ++s) {
      const int rank = rank_of_root[std::size_t(root[s])];
      if (rank < 0)
        continue;
      Cell_handle temp_c = c[s];
      const int temp_id = id[s];
      Vertex_handle fv[3] = {temp_c->vertex((temp_id + 1) % 4), temp_c->vertex((temp_id + 2) % 4),
                             temp_c->vertex((temp_id + 3) % 4)};
      facet_out o;
      // point the facet away from the cell which is in this cluster.
      const bool positive = cg::orientation(fv[0]->point(), fv[1]->point(), fv[2]->point(),
                                            temp_c->vertex(temp_id)->point()) == cg::POSITIVE;
      o.v[0] = fv[0]->tc;
      o.v[1] = positive ? fv[2]->tc : fv[1]->tc;
      o.v[2] = positive ? fv[1]->tc : fv[2]->tc;
      const pocket_feature &f = r.features[std::size_t(shown[std::size_t(rank)])];
      if (f.kind == pocket_kind::void_)
        o.on_surface = c[0]->cocone_flag(id[0]);
      else
        o.on_surface = cl.outside[std::size_t(c[0]->id)] != cl.outside[std::size_t(c[1]->id)];
      facets[std::size_t(rank)].push_back(o);
    }
  }

  // The colours: per feature shown, in order, its colour at each of its points (red and purple
  // win within it), then applied as the legacy library applied them: red keeps among the tunnels
  // and voids, purple among the pockets.
  const std::size_t NP = std::size_t(tc.num_points());
  std::vector<std::uint8_t> colour(NP, COL_WHITE), mine(NP, COL_WHITE);
  std::vector<int> touched;
  geometry &g = r.surface;
  for (std::size_t rank = 0; rank < shown.size(); ++rank) {
    SK_UNIT;
    const pocket_feature &f = r.features[std::size_t(shown[rank])];
    for (const facet_out &o : facets[rank]) {
      std::uint8_t c;
      switch (f.kind) {
      case pocket_kind::tunnel:
        c = o.on_surface ? COL_YELLOW : COL_RED;
        break;
      case pocket_kind::void_:
        c = o.on_surface ? COL_PURPLE : COL_BLUE;
        break;
      default:
        c = o.on_surface ? COL_GREEN : COL_PURPLE;
        break;
      }
      for (int j = 0; j < 3; ++j) {
        std::uint8_t &m = mine[std::size_t(o.v[j])];
        if (m == COL_WHITE)
          touched.push_back(o.v[j]);
        if (m == COL_WHITE || c == COL_RED || c == COL_PURPLE)
          m = c;
      }
      g.tris().push_back(geometry::tri_t{
          {geometry::index_t(o.v[0]), geometry::index_t(o.v[1]), geometry::index_t(o.v[2])}});
      r.feature_of_tri.push_back(shown[rank]);
      r.tri_on_surface.push_back(o.on_surface);
    }
    const std::uint8_t sticky = f.kind == pocket_kind::pocket ? COL_PURPLE : COL_RED;
    for (int v : touched) {
      std::uint8_t &m = mine[std::size_t(v)];
      if (colour[std::size_t(v)] != sticky)
        colour[std::size_t(v)] = m;
      m = COL_WHITE;
    }
    touched.clear();
  }

  // Every input point, a duplicate coloured as the point it repeats.
  g.points().reserve(NP);
  g.colors().reserve(NP);
  for (std::size_t i = 0; i < NP; ++i) {
    const tet_complex::point &p = tc.points()[i];
    g.points().push_back(geometry::point_t{{p[0], p[1], p[2]}});
    g.colors().push_back(rgb(colour[std::size_t(tc.duplicate_of(int(i)))]));
    if (i % 64 == 63)
      SK_UNIT;
  }

  // Warnings: what the legacy library printed.
  auto warn = [&](std::size_t n, const char *what) {
    if (n)
      r.warnings.push_back(fmt_count(what, n));
  };
  warn(duplicates, "%zu duplicate points were merged");
  warn(diag.dirty_cells, "%zu cells' circumcentres or radii were not finite (approximated by a "
                         "facet's circumcentre)");
  const int want_t = params.tunnels, want_p = params.pockets;
  if (r.tunnels + r.voids < want_t)
    r.warnings.push_back("found " + std::to_string(r.tunnels + r.voids) +
                         " tunnels and voids, fewer than the " + std::to_string(want_t) +
                         " asked for");
  if (r.pockets < want_p)
    r.warnings.push_back("found " + std::to_string(r.pockets) + " pockets, fewer than the " +
                         std::to_string(want_p) + " asked for");
}

pocket_tunnel_stepper::pocket_tunnel_stepper(std::vector<std::array<double, 3>> points,
                                             pocket_tunnel_params params)
    : _impl(std::make_unique<impl>()) {
  _impl->points = std::move(points);
  _impl->params = params;
}

pocket_tunnel_stepper::pocket_tunnel_stepper(const geometry &surface, pocket_tunnel_params params)
    : _impl(std::make_unique<impl>()) {
  _impl->source.emplace(surface); // shares the arrays; the points are read in the first step
  _impl->params = params;
}

pocket_tunnel_stepper::~pocket_tunnel_stepper() = default;
pocket_tunnel_stepper::pocket_tunnel_stepper(pocket_tunnel_stepper &&) noexcept = default;
pocket_tunnel_stepper &
pocket_tunnel_stepper::operator=(pocket_tunnel_stepper &&) noexcept = default;

bool pocket_tunnel_stepper::step(std::size_t budget) {
  if (!_impl)
    return true;
  impl &m = *_impl;
  if (m.failed) {
    if (m.cancel.load(std::memory_order_acquire))
      return true;
    std::rethrow_exception(m.error);
  }
  if (m.finished)
    return true;
  try {
    if (!m.task)
      m.task = m.run();
    for (std::size_t n = 0; budget == 0 || n < budget; ++n) {
      if (m.cancel.load(std::memory_order_acquire))
        break;
      if (m.task.resume()) {
        m.finished = true;
        m.phase_name = "done";
        m.prog = 1.0;
        m.task = cocone::step_task();
        m.triang.reset(tet_complex{});
        return true;
      }
      if (++m.stage_units % 64 == 0)
        m.note_progress(double(m.stage_units) / double(m.stage_estimate));
    }
    if (m.cancel.load(std::memory_order_acquire)) {
      m.error = std::make_exception_ptr(pocket_tunnel_error("pocket_tunnel: cancelled"));
      m.failed = true;
      m.phase_name = "failed";
      m.task = cocone::step_task(); // frees every frame, the triangulation and the clusters
      m.triang.reset(tet_complex{});
      return true;
    }
    return false;
  } catch (...) {
    m.error = std::current_exception();
    m.failed = true;
    m.phase_name = "failed";
    m.task = cocone::step_task();
    throw;
  }
}

bool pocket_tunnel_stepper::done() const { return !_impl || _impl->finished || _impl->failed; }
const char *pocket_tunnel_stepper::phase() const { return _impl ? _impl->phase_name : "done"; }
double pocket_tunnel_stepper::progress() const { return _impl ? _impl->prog : 1.0; }

std::string pocket_tunnel_stepper::status() const {
  if (!_impl)
    return "done";
  if (_impl->failed) {
    try {
      std::rethrow_exception(_impl->error);
    } catch (const std::exception &e) {
      return std::string("failed: ") + e.what();
    } catch (...) {
      return "failed";
    }
  }
  if (_impl->finished) {
    char buf[160];
    std::snprintf(buf, sizeof buf, "done: %d pockets, %d tunnels, %d voids", _impl->result.pockets,
                  _impl->result.tunnels, _impl->result.voids);
    return buf;
  }
  char buf[96];
  std::snprintf(buf, sizeof buf, "%s %d%%", _impl->phase_name, int(_impl->prog * 100));
  return buf;
}

void pocket_tunnel_stepper::cancel() {
  if (_impl)
    _impl->cancel.store(true, std::memory_order_release);
}

pocket_tunnels pocket_tunnel_stepper::take() {
  if (_impl && _impl->failed)
    std::rethrow_exception(_impl->error);
  if (!_impl || !_impl->finished)
    throw pocket_tunnel_error("pocket_tunnel: take() before the stepper is done");
  pocket_tunnels r = std::move(_impl->result);
  _impl.reset();
  return r;
}

pocket_tunnels pocket_tunnel(const std::vector<std::array<double, 3>> &points,
                             const pocket_tunnel_params &params) {
  pocket_tunnel_stepper s(points, params);
  s.step(0);
  return s.take();
}

pocket_tunnels pocket_tunnel(const geometry &surface, const pocket_tunnel_params &params) {
  pocket_tunnel_stepper s(surface, params);
  s.step(0);
  return s.take();
}

struct pocket_tunnel_stepper_access {
  static void give(pocket_tunnel_stepper &s, pocket_tunnel_detail::marking m) {
    s._impl->given.emplace(std::move(m));
  }
};

pocket_tunnels
pocket_tunnel_detail::pocket_tunnel_marked(const std::vector<std::array<double, 3>> &points,
                                           const pocket_tunnel_params &params, marking given) {
  pocket_tunnel_stepper s(points, params);
  pocket_tunnel_stepper_access::give(s, std::move(given));
  s.step(0);
  return s.take();
}

std::unique_ptr<pocket_tunnel_stepper>
pocket_tunnel_stepper_for(std::vector<std::array<double, 3>> points, pocket_tunnel_params params) {
  return std::make_unique<pocket_tunnel_stepper>(std::move(points), params);
}

std::unique_ptr<result_job<pocket_tunnels>>
pocket_tunnel_job(std::vector<std::array<double, 3>> points, pocket_tunnel_params params) {
  return make_result_job(std::make_unique<pocket_tunnel_stepper>(std::move(points), params));
}

std::unique_ptr<result_job<pocket_tunnels>> pocket_tunnel_job(const geometry &surface,
                                                              pocket_tunnel_params params) {
  return make_result_job(std::make_unique<pocket_tunnel_stepper>(surface, params));
}

} // namespace cvc
