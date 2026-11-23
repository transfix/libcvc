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

// SecondaryStructures' medax.cpp (its medial axis) and skel.cpp's add_u1_to_skel, add_u2_to_skel
// and do_star (its skeleton graph), on the cocone core. The loops are the legacy ones, in its
// order; the legacy debug file ("test", the medial axis drawn as OFF) and the connected-component
// tags only that file used are left out.

#include "secondary.h"

#include <cmath>
#include <set>

namespace cvc::secondary_detail {

using namespace cvc::cocone;

namespace {

// The legacy cell base's e_tag[4][4] (a medial-axis candidate), a bit per vertex pair, by cell id.
struct tags {
  std::vector<std::uint16_t> bits;
  explicit tags(std::size_t cells) : bits(cells, 0) {}
  bool get(Cell_handle c, int i, int j) const {
    return (bits[std::size_t(c->id)] >> (4 * i + j)) & 1u;
  }
  void set(Cell_handle c, int i, int j) {
    bits[std::size_t(c->id)] |= std::uint16_t((1u << (4 * i + j)) | (1u << (4 * j + i)));
  }
};

// The Voronoi facet dual to e lies inside the surface: none of the cells around e is outside.
bool is_inside_VF(const Triangulation &triang, const Edge &e) {
  Facet_circulator fcirc = triang.incident_facets(e);
  Facet_circulator begin = fcirc;
  do {
    if ((*fcirc).first->outside)
      return false;
    fcirc++;
  } while (fcirc != begin);
  return true;
}

// The Voronoi facet dual to e has a free Voronoi edge: one whose other two facets are both off
// the medial axis.
bool is_VF_on_boundary_of_medax(const Triangulation &triang, const Edge &e) {
  Cell_handle c = e.first;
  int uid = e.second, vid = e.third;
  Facet_circulator fcirc = triang.incident_facets(e);
  Facet_circulator begin = fcirc;
  do {
    Cell_handle _c = (*fcirc).first;
    int _id = (*fcirc).second;
    int _uid = -1, _vid = -1;
    triang.has_vertex(_c, _id, c->vertex(uid), _uid);
    triang.has_vertex(_c, _id, c->vertex(vid), _vid);
    int _wid = 6 - _id - _uid - _vid;
    if (!_c->VF_on_medax(_uid, _wid) && !_c->VF_on_medax(_vid, _wid))
      return true;
    fcirc++;
  } while (fcirc != begin);
  return false;
}

// Calls f(cell, i, j) for the Voronoi facet dual to the edge (c, u, v), as it appears in every cell
// around that edge.
template <class F>
void for_each_around(const Triangulation &triang, Cell_handle c, int u, int v, F f) {
  Facet_circulator fcirc = triang.incident_facets(Edge(c, u, v));
  Facet_circulator begin = fcirc;
  do {
    Cell_handle _c = (*fcirc).first;
    int _id = (*fcirc).second;
    int _u = -1, _v = -1;
    triang.has_vertex(_c, _id, c->vertex(u), _u);
    triang.has_vertex(_c, _id, c->vertex(v), _v);
    f(_c, _u, _v);
    fcirc++;
  } while (fcirc != begin);
}

// compute_dia_of_tang_umbrella: the largest squared circumradius of the triangles in a flat
// vertex's tangent umbrella (those satisfying the cocone condition), and their normals.
step_task tangent_umbrella_diameters(const Triangulation &triang) {
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); ++fit) {
    SK_UNIT;
    Cell_handle ch = (*fit).first;
    int id = (*fit).second;
    // A finite facet must be incident to at least one finite cell.
    if (triang.is_infinite(ch)) {
      Cell_handle c = ch;
      ch = ch->neighbor(id);
      id = ch->index(c);
    }
    double cir_radius =
        sq_cr_tr_3(ch->vertex((id + 1) % 4)->point(), ch->vertex((id + 2) % 4)->point(),
                   ch->vertex((id + 3) % 4)->point());
    for (int i = 1; i < 4; ++i) {
      Vertex_handle vh = ch->vertex((id + i) % 4);
      double cos1 = cosine(ch->voronoi() - vh->point(), vh->normal());
      double cos2 = (triang.is_infinite(ch->neighbor(id)))
                        ? 1.0
                        : cosine(ch->neighbor(id)->voronoi() - vh->point(), vh->normal());
      if (cos1 > cos2)
        std::swap(cos1, cos2);
      // Test for disjoint intervals [cos1, cos2] and [min_cos, max_cos].
      if ((cos1 < 0.0) && (cos2 > 0.0)) {
        if (!vh->is_flat())
          continue;
        Vector facet_normal = cg::cross_product(
            ch->vertex((id + 2) % 4)->point() - ch->vertex((id + 1) % 4)->point(),
            ch->vertex((id + 3) % 4)->point() - ch->vertex((id + 1) % 4)->point());
        vh->normal_stack.push_back(facet_normal);
        if (vh->diameter() < cir_radius)
          vh->set_diameter(cir_radius);
      }
    }
  }
}

// detect_axis_candidates: tag the inside Voronoi facets the angle or the ratio condition chooses.
step_task detect_axis_candidates(const Triangulation &triang, const double theta,
                                 const double ratio, tags &tag) {
  double COS_MIN = cos(M_PI / 2.0 + theta);
  double COS_MAX = cos(M_PI / 2.0 - theta);
  for (FEI eit = triang.finite_edges_begin(); eit != triang.finite_edges_end(); ++eit) {
    SK_UNIT;
    Cell_handle ch = (*eit).first;
    Vertex_handle vh = ch->vertex((*eit).second);
    Vertex_handle wh = ch->vertex((*eit).third);
    if (!vh->is_flat() && !wh->is_flat())
      continue;
    // choose by the angle
    Vector ev = wh->point() - vh->point();
    Vector ew = vh->point() - wh->point();
    bool vchoose = true;
    for (const Vector &n : vh->normal_stack) {
      double vcos = cosine(n, ev);
      if ((vcos < COS_MAX) && (vcos > COS_MIN))
        vchoose = false;
    }
    bool wchoose = true;
    for (const Vector &n : wh->normal_stack) {
      double wcos = cosine(n, ew);
      if ((wcos < COS_MAX) && (wcos > COS_MIN))
        wchoose = false;
    }
    if (!vchoose || !wchoose) {
      // choose by the ratio
      double length = cg::to_double((vh->point() - wh->point()) * (vh->point() - wh->point()));
      if (!((length / vh->diameter() > ratio) && (length / wh->diameter() > ratio)))
        continue;
    }
    if (!is_inside_VF(triang, (*eit)))
      continue;
    for_each_around(triang, (*eit).first, (*eit).second, (*eit).third,
                    [&](Cell_handle c, int i, int j) { tag.set(c, i, j); });
  }
}

// Mark the facet dual to (c, u, v) visited and off the medial axis in every cell around it. (The
// legacy code marked e_visited[i][j] only, so a facet met as [j][i] could be peeled twice; the
// set peeled is the same, since peeling only ever frees more facets.)
void peel_VF(const Triangulation &triang, Cell_handle c, int u, int v) {
  for_each_around(triang, c, u, v, [](Cell_handle _c, int i, int j) {
    _c->set_VF_on_medax(i, j, false);
    _c->e_visited[i][j] = _c->e_visited[j][i] = true;
  });
}

// compute_axis: every inside facet on the medial axis, then peel the untagged ones from its free
// boundary (so what is left keeps the shape's topology), then mark the Voronoi vertices on it.
step_task compute_axis(Triangulation &triang, const tags &tag) {
  for (ACI cit = triang.all_cells_begin(); cit != triang.all_cells_end(); ++cit) {
    SK_UNIT;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        cit->e_visited[i][j] = false;
  }
  for (FEI eit = triang.finite_edges_begin(); eit != triang.finite_edges_end(); eit++) {
    SK_UNIT;
    if (!is_inside_VF(triang, (*eit)))
      continue;
    for_each_around(triang, (*eit).first, (*eit).second, (*eit).third,
                    [](Cell_handle c, int i, int j) { c->set_VF_on_medax(i, j, true); });
  }
  vector<Edge> peel;
  for (FEI eit = triang.finite_edges_begin(); eit != triang.finite_edges_end(); eit++) {
    SK_UNIT;
    if (!is_inside_VF(triang, (*eit)))
      continue;
    Cell_handle c = (*eit).first;
    int uid = (*eit).second, vid = (*eit).third;
    if (c->e_visited[uid][vid])
      continue;
    // A tagged facet is a true medial-axis candidate: it stays.
    if (tag.get(c, uid, vid))
      continue;
    // Otherwise it is peeled when it is on the boundary.
    if (!is_VF_on_boundary_of_medax(triang, (*eit)))
      continue;
    peel.push_back((*eit));
    peel_VF(triang, c, uid, vid);
    while (!peel.empty()) {
      SK_UNIT;
      Edge e = peel.back();
      peel.pop_back();
      Cell_handle ce = e.first;
      int ue = e.second, ve = e.third;
      Facet_circulator fcirc = triang.incident_facets(e);
      Facet_circulator begin = fcirc;
      do {
        Cell_handle _c = (*fcirc).first;
        int _id = (*fcirc).second;
        int _uid = -1, _vid = -1;
        triang.has_vertex(_c, _id, ce->vertex(ue), _uid);
        triang.has_vertex(_c, _id, ce->vertex(ve), _vid);
        // (_c, _uid, _vid) is this facet; the other two around the Voronoi edge dual to
        // (_c, _id) are (_c, _uid, _wid) and (_c, _vid, _wid).
        int _wid = 6 - _uid - _vid - _id;
        if (is_inside_VF(triang, Edge(_c, _uid, _wid)) && !_c->e_visited[_uid][_wid] &&
            !tag.get(_c, _uid, _wid) && is_VF_on_boundary_of_medax(triang, Edge(_c, _uid, _wid))) {
          peel.push_back(Edge(_c, _uid, _wid));
          peel_VF(triang, _c, _uid, _wid);
        }
        if (is_inside_VF(triang, Edge(_c, _vid, _wid)) && !_c->e_visited[_vid][_wid] &&
            !tag.get(_c, _vid, _wid) && is_VF_on_boundary_of_medax(triang, Edge(_c, _vid, _wid))) {
          peel.push_back(Edge(_c, _vid, _wid));
          peel_VF(triang, _c, _vid, _wid);
        }
        fcirc++;
      } while (fcirc != begin);
    }
  }
  // Mark the medial axis's Voronoi vertices.
  for (FEI eit = triang.finite_edges_begin(); eit != triang.finite_edges_end(); eit++) {
    SK_UNIT;
    if (!is_inside_VF(triang, (*eit)))
      continue;
    Cell_handle c = (*eit).first;
    if (!c->VF_on_medax((*eit).second, (*eit).third))
      continue;
    for_each_around(triang, c, (*eit).second, (*eit).third,
                    [](Cell_handle _c, int, int) { _c->set_VV_on_medax(true); });
  }
}

} // namespace

step_task compute_inner_medial_axis(Triangulation &triang, double theta, double ratio) {
  tags tag(triang.num_cells());
  SK_RUN(tangent_umbrella_diameters(triang));
  SK_RUN(detect_axis_candidates(triang, theta, ratio, tag));
  SK_RUN(compute_axis(triang, tag));
}

// ---- the skeleton graph --------------------------------------------------------------------

std::uint32_t skeleton_builder::add_point(const Point &p) {
  s.points.push_back({p.x(), p.y(), p.z()});
  adjacency.emplace_back();
  point_faces.emplace_back();
  return std::uint32_t(s.points.size() - 1);
}

std::uint32_t skeleton_builder::point_of(Cell_handle c) {
  std::int64_t &id = cell_point[std::size_t(c->id)];
  if (id < 0)
    id = add_point(c->voronoi());
  return std::uint32_t(id);
}

std::int64_t skeleton_builder::find_edge(std::uint32_t a, std::uint32_t b) const {
  for (std::uint32_t e : adjacency[a]) {
    const secondary_skeleton::edge &ed = s.edges[e];
    if ((ed.a == a && ed.b == b) || (ed.a == b && ed.b == a))
      return std::int64_t(e);
  }
  return -1;
}

std::uint32_t skeleton_builder::add_edge(std::uint32_t a, std::uint32_t b) {
  secondary_skeleton::edge e;
  e.a = a;
  e.b = b;
  s.edges.push_back(e);
  edge_faces.emplace_back();
  const auto id = std::uint32_t(s.edges.size() - 1);
  adjacency[a].push_back(id);
  if (b != a)
    adjacency[b].push_back(id);
  return id;
}

bool skeleton_builder::in_same_component(std::uint32_t a, std::uint32_t b) const {
  for (int fa : point_faces[a]) {
    const int ca = s.faces[std::size_t(fa)].component;
    if (ca < 0)
      continue;
    for (int fb : point_faces[b])
      if (s.faces[std::size_t(fb)].component == ca)
        return true;
  }
  return false;
}

step_task add_u1(const Triangulation &triang, skeleton_builder &b) {
  b.cell_point.assign(triang.num_cells(), -1);
  for (FEI eit = triang.finite_edges_begin(); eit != triang.finite_edges_end(); eit++) {
    SK_UNIT;
    Cell_handle c = (*eit).first;
    int uid = (*eit).second, vid = (*eit).third;
    if (!is_inside_VF(triang, (*eit)))
      continue;
    if (!c->VF_on_um_i1(uid, vid))
      continue;
    // A polygon of the Voronoi vertices around the edge.
    vector<std::uint32_t> vlist;
    Facet_circulator fcirc = triang.incident_facets((*eit));
    Facet_circulator begin = fcirc;
    do {
      vlist.push_back(b.point_of((*fcirc).first));
      fcirc++;
    } while (fcirc != begin);
    secondary_skeleton::face f;
    f.points = vlist;
    f.width = length_of_seg(Segment(c->vertex(uid)->point(), c->vertex(vid)->point()));
    b.s.faces.push_back(std::move(f));
    b.face_edges.emplace_back();
    const int fid = int(b.s.faces.size() - 1);
    for (std::size_t i = 0; i < vlist.size(); i++) {
      b.point_faces[vlist[i]].push_back(fid);
      const std::uint32_t p = vlist[i], q = vlist[(i + 1) % vlist.size()];
      std::int64_t e = b.find_edge(p, q);
      if (e < 0) {
        e = b.add_edge(p, q);
        b.s.edges[std::size_t(e)].on_u1 = true;
      }
      secondary_skeleton::edge &ed = b.s.edges[std::size_t(e)];
      ed.faces++;
      b.edge_faces[std::size_t(e)].push_back(fid);
      b.face_edges[std::size_t(fid)].push_back(int(e));
    }
  }
  // The polygons' components: walks across the edges on at most two polygons.
  vector<char> f_vis(b.s.faces.size(), 0);
  int comp_id = -1;
  for (std::size_t i = 0; i < b.s.faces.size(); i++) {
    SK_UNIT;
    if (f_vis[i])
      continue;
    vector<int> walk{int(i)};
    f_vis[i] = 1;
    comp_id++;
    b.s.faces[i].component = comp_id;
    vector<int> cur_comp{int(i)};
    while (!walk.empty()) {
      SK_UNIT;
      int fid = walk.back();
      walk.pop_back();
      for (int eid : b.face_edges[std::size_t(fid)]) {
        if (b.s.edges[std::size_t(eid)].faces > 2) // a non-manifold edge
          continue;
        for (int _fid : b.edge_faces[std::size_t(eid)]) {
          if (f_vis[std::size_t(_fid)])
            continue;
          walk.push_back(_fid);
          f_vis[std::size_t(_fid)] = 1;
          b.s.faces[std::size_t(_fid)].component = comp_id;
          cur_comp.push_back(_fid);
        }
      }
    }
    b.components.push_back(std::move(cur_comp));
  }
}

step_task add_u2(const Graph &graph, skeleton_builder &b) {
  const vector<GVertex> &gv = graph.vert_list;
  vector<std::int64_t> saddle_point(gv.size(), -1);
  // The graph's edges are its chains', in order: each chain is its saddle and then its cells. A
  // graph vertex is a cell's Voronoi vertex when that cell's g_vid is it, else a saddle (the
  // circumcentre of the facet whose saddle_g_vid is it).
  auto is_cell = [&](int g) { return gv[std::size_t(g)].c->g_vid == g; };
  for (const GEdge &ge : graph.edge_list) {
    SK_UNIT;
    const int ga = ge.get_endpoint(0), gb = ge.get_endpoint(1);
    Cell_handle cb = gv[std::size_t(gb)].c;
    if (!is_cell(gb))
      continue; // a chain starts at its saddle; nothing ends at one
    if (!is_cell(ga)) {
      // The saddle edge: from the circumcentre of the facet (c, k) to the chain's first cell.
      Cell_handle c = gv[std::size_t(ga)].c;
      int k = 0;
      while (k < 4 && c->saddle_g_vid[k] != ga)
        ++k;
      if (k == 4)
        continue;
      if (saddle_point[std::size_t(ga)] < 0)
        saddle_point[std::size_t(ga)] = b.add_point(gv[std::size_t(ga)].point());
      const auto vid = std::uint32_t(saddle_point[std::size_t(ga)]);
      const std::uint32_t uid = b.point_of(cb);
      Cell_handle c1 = c->neighbor(k);
      const std::int64_t p0 = b.cell_point[std::size_t(c->id)],
                         p1 = b.cell_point[std::size_t(c1->id)];
      if (p0 >= 0 && p1 >= 0 && b.in_same_component(std::uint32_t(p0), std::uint32_t(p1)))
        continue;
      std::int64_t e = b.find_edge(uid, vid);
      if (e < 0) {
        e = b.add_edge(uid, vid);
        b.s.edges[std::size_t(e)].width = circumradius(Facet(c, k));
      }
      b.s.edges[std::size_t(e)].on_u2 = true;
      continue;
    }
    // Two consecutive cells of a chain.
    Cell_handle ca = gv[std::size_t(ga)].c;
    const std::uint32_t uid = b.point_of(ca), vid = b.point_of(cb);
    std::int64_t e = b.find_edge(uid, vid);
    if (e < 0) {
      e = b.add_edge(uid, vid);
      // The tube's radius: the circumradius of the facet the cells share, else the smaller
      // cell's circumradius. (The legacy code took the smaller cell's squared circumradius.)
      double width = std::sqrt(std::min(ca->cell_radius(), cb->cell_radius()));
      const int k = ca->index(cb);
      if (ca != cb && k >= 0)
        width = circumradius(Facet(ca, k));
      b.s.edges[std::size_t(e)].width = width;
    }
    b.s.edges[std::size_t(e)].on_u2 = true;
  }
}

step_task component_areas(skeleton_builder &b) {
  b.s.component_area.assign(b.components.size(), 0.0);
  for (std::size_t i = 0; i < b.components.size(); i++) {
    double comp_area = 0;
    for (int fid : b.components[i]) {
      SK_UNIT;
      const secondary_skeleton::face &f = b.s.faces[std::size_t(fid)];
      const std::size_t n = f.points.size();
      Vector c = cg::NULL_VECTOR;
      auto pt = [&](std::size_t k) {
        const std::array<double, 3> &p = b.s.points[f.points[k]];
        return Point(p[0], p[1], p[2]);
      };
      for (std::size_t k = 0; k < n; k++)
        c = c + (pt(k) - cg::ORIGIN);
      c = (1. / double(n)) * c;
      for (std::size_t k = 0; k < n; k++) {
        Triangle_3 t(cg::ORIGIN + c, pt(k), pt((k + 1) % n));
        comp_area += std::sqrt(cg::to_double(t.squared_area()));
      }
    }
    b.s.component_area[i] = comp_area;
  }
}

} // namespace cvc::secondary_detail
