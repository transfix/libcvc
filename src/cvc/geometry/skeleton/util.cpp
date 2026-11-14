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

// util.cpp, intersect.cpp, hfn_util.cpp's predicates and degen.cpp of VolumeRover's
// Skeletonization: small helpers on vectors, facets and edges, the Voronoi facets (VF) dual to
// Delaunay edges, the flow on Voronoi edges (VE), and the degeneracy checks.

#include "cocone.h"

namespace cvc::cocone {

// The cosine of the angle between v and w.
double cosine(const Vector &v, const Vector &w) {
  return cg::to_double(v * w) / std::sqrt(cg::to_double(v * v) * cg::to_double(w * w));
}

void normalize(Vector &v) { v = (1. / (std::sqrt(cg::to_double(v * v)))) * v; }

// Whether the angle <p0, p2, p1> exceeds 90 degrees.
bool is_obtuse(const Point &p0, const Point &p1, const Point &p2) {
  Vector v0 = p0 - p2;
  Vector v1 = p1 - p2;
  return (cg::to_double(v0 * v1) < 0);
}

double length_of_seg(const Segment &s) {
  return std::sqrt(cg::to_double((s.point(0) - s.point(1)) * (s.point(0) - s.point(1))));
}

// The index of the facet's third vertex, neither v nor w.
int find_third_vertex_index(const Facet &f, Vertex_handle v, Vertex_handle w) {
  int id = f.second;
  for (int i = 1; i <= 3; ++i) {
    if (f.first->vertex((id + i) % 4) != v && f.first->vertex((id + i) % 4) != w)
      return (id + i) % 4;
  }
  return -1;
}

// The index of an edge in a facet: the index of the cell's vertex opposite the edge in that
// facet.
int edge_index(const int facet_index, const int first_vertex_index, const int second_vertex_index) {
  return 6 - facet_index - first_vertex_index - second_vertex_index;
}

// The indices of the two vertices of the edge `edge_index` of facet `facet_index`.
void vertex_indices(const int facet_index, const int edge_index, int &first_vertex,
                    int &second_vertex) {
  if ((facet_index == 0 && edge_index == 1) || (facet_index == 1 && edge_index == 0)) {
    first_vertex = 2;
    second_vertex = 3;
  } else if ((facet_index == 0 && edge_index == 2) || (facet_index == 2 && edge_index == 0)) {
    first_vertex = 1;
    second_vertex = 3;
  } else if ((facet_index == 0 && edge_index == 3) || (facet_index == 3 && edge_index == 0)) {
    first_vertex = 1;
    second_vertex = 2;
  } else if ((facet_index == 1 && edge_index == 2) || (facet_index == 2 && edge_index == 1)) {
    first_vertex = 0;
    second_vertex = 3;
  } else if ((facet_index == 1 && edge_index == 3) || (facet_index == 3 && edge_index == 1)) {
    first_vertex = 0;
    second_vertex = 2;
  } else if ((facet_index == 2 && edge_index == 3) || (facet_index == 3 && edge_index == 2)) {
    first_vertex = 0;
    second_vertex = 1;
  }
}

bool is_outside_bounding_box(const Point &p, const vector<double> &bounding_box) {
  return cg::to_double(p.x()) < bounding_box[0] || cg::to_double(p.x()) > bounding_box[1] ||
         cg::to_double(p.y()) < bounding_box[2] || cg::to_double(p.y()) > bounding_box[3] ||
         cg::to_double(p.z()) < bounding_box[4] || cg::to_double(p.z()) > bounding_box[5];
}

namespace {
// For each facet around the edge (c, uid, vid): that facet's cell and the indices of the edge's
// two vertices in it (the legacy mark_VF_* / set_patch_id loop).
template <class F>
void for_each_VF_cell(const Triangulation &triang, Cell_handle c, int uid, int vid, F f) {
  Facet_circulator fcirc = triang.incident_facets(Edge(c, uid, vid));
  Facet_circulator begin = fcirc;
  do {
    Cell_handle cur_c = (*fcirc).first;
    int cur_fid = (*fcirc).second;
    int cur_uid = -1, cur_vid = -1;
    for (int k = 1; k < 4; k++) {
      if (cur_c->vertex((cur_fid + k) % 4)->id != c->vertex(uid)->id &&
          cur_c->vertex((cur_fid + k) % 4)->id != c->vertex(vid)->id) {
        vertex_indices(cur_fid, (cur_fid + k) % 4, cur_uid, cur_vid);
        break;
      }
    }
    f(cur_c, cur_uid, cur_vid);
    fcirc++;
  } while (fcirc != begin);
}
} // namespace

// Mark the VF dual to (c, uid, vid) as on um(i1) for some i1.
void mark_VF_on_u1(Triangulation &triang, Cell_handle &c, int uid, int vid) {
  for_each_VF_cell(triang, c, uid, vid, [](Cell_handle cur_c, int cur_uid, int cur_vid) {
    cur_c->set_VF_on_um_i1(cur_uid, cur_vid, true);
  });
}

// Mark the VF dual to (c, uid, vid) visited.
void mark_VF_visited(Triangulation &triang, Cell_handle &c, int uid, int vid) {
  for_each_VF_cell(triang, c, uid, vid, [](Cell_handle cur_c, int cur_uid, int cur_vid) {
    cur_c->e_visited[cur_uid][cur_vid] = true;
    cur_c->e_visited[cur_vid][cur_uid] = true;
  });
}

// Set the patch id of the VF dual to (c, uid, vid).
void set_patch_id(Triangulation &triang, Cell_handle &c, int uid, int vid, const int &id) {
  for_each_VF_cell(triang, c, uid, vid, [id](Cell_handle cur_c, int cur_uid, int cur_vid) {
    cur_c->patch_id[cur_uid][cur_vid] = id;
    cur_c->patch_id[cur_vid][cur_uid] = id;
  });
}

bool is_surf_VF(const Triangulation &triang, const Cell_handle &c, const int uid, const int vid) {
  Facet_circulator fcirc = triang.incident_facets(Edge(c, uid, vid));
  Facet_circulator begin = fcirc;
  do {
    if ((*fcirc).first->cocone_flag((*fcirc).second))
      return true;
    fcirc++;
  } while (fcirc != begin);
  return false;
}

bool is_inf_VF(const Triangulation &triang, const Cell_handle &c, const int uid, const int vid) {
  Facet_circulator fcirc = triang.incident_facets(Edge(c, uid, vid));
  Facet_circulator begin = fcirc;
  do {
    if (triang.is_infinite((*fcirc).first))
      return true;
    fcirc++;
  } while (fcirc != begin);
  return false;
}

// Whether the two cells sharing facet f are cospherical (exactly, unperturbed).
bool is_cospherical_pair(const Triangulation &triang, const Facet &f) {
  Cell_handle c[2];
  int id[2];
  c[0] = f.first;
  id[0] = f.second;
  c[1] = c[0]->neighbor(id[0]);
  id[1] = c[1]->index(c[0]);
  // A cell beyond the hull is never cospherical with one inside (the legacy code tested the
  // infinite vertex's default-constructed point here).
  if (triang.is_infinite(c[0]) || triang.is_infinite(c[1]))
    return false;
  return triang.side_of_sphere(c[0], c[1]->vertex(id[1])->point()) == cg::ON_BOUNDARY;
}

bool is_there_any_common_element(const vector<int> &vec1, const vector<int> &vec2) {
  for (int a : vec1)
    for (int b : vec2)
      if (a == b)
        return true;
  return false;
}

// ---- intersect.cpp ----

namespace {
// The legacy frame: the segment along x from its first point, the ray's source in the xy plane.
struct seg_frame {
  Point org;
  Vector xaxis, yaxis;
  cg::Point_2<Rep> s0, s1, r0, r1;
};
seg_frame make_frame(const Ray &r, const Segment &s) {
  seg_frame f;
  f.org = s.point(0);
  Vector zaxis = cg::NULL_VECTOR;
  f.xaxis = s.point(1) - s.point(0);
  normalize(f.xaxis);
  zaxis = cg::cross_product(f.xaxis, r.source() - f.org);
  normalize(zaxis);
  f.yaxis = cg::cross_product(zaxis, f.xaxis);
  normalize(f.yaxis);
  f.s0 = cg::Point_2<Rep>(0, 0);
  f.s1 = cg::Point_2<Rep>(length_of_seg(s), 0);
  const Point p0 = r.source();
  f.r0 = cg::Point_2<Rep>(cg::to_double((p0 - f.org) * f.xaxis),
                          cg::to_double((p0 - f.org) * f.yaxis));
  const Point p1 = p0 + r.to_vector();
  f.r1 = cg::Point_2<Rep>(cg::to_double((p1 - f.org) * f.xaxis),
                          cg::to_double((p1 - f.org) * f.yaxis));
  return f;
}
} // namespace

// Whether a ray and a segment in one plane meet in a point.
bool does_intersect_ray3_seg3_in_plane(const Ray &r, const Segment &s) {
  const seg_frame f = make_frame(r, s);
  cg::Point_2<Rep> p;
  return cg::assign(p,
                    cg::intersection(cg::Ray_2<Rep>(f.r0, f.r1), cg::Segment_2<Rep>(f.s0, f.s1)));
}

// Where a ray and a segment in one plane meet.
Point intersect_ray3_seg3(const Ray &r, const Segment &s, bool &is_correct_intersection) {
  const seg_frame f = make_frame(r, s);
  cg::Point_2<Rep> p;
  is_correct_intersection =
      cg::assign(p, cg::intersection(cg::Ray_2<Rep>(f.r0, f.r1), cg::Segment_2<Rep>(f.s0, f.s1)));
  if (!is_correct_intersection)
    return cg::ORIGIN;
  double t = std::sqrt(cg::to_double((p - f.s0) * (p - f.s0))) / length_of_seg(s);
  return s.point(0) + t * (s.point(1) - s.point(0));
}

// Whether a segment meets a convex polygon in space: the polygon is fanned from its centroid and
// each triangle tested.
bool does_intersect_convex_polygon_segment_3_in_3d(const vector<Point> &conv_poly, const Segment &s,
                                                   diagnostics &diag) {
  Vector centroid = cg::NULL_VECTOR;
  vector<Triangle_3> t;
  for (const Point &p : conv_poly)
    centroid = centroid + (p - cg::ORIGIN);
  centroid = (1. / (int)conv_poly.size()) * centroid;
  for (int i = 0; i < (int)conv_poly.size(); i++)
    t.push_back(Triangle_3(cg::ORIGIN + centroid, conv_poly[std::size_t(i)],
                           conv_poly[std::size_t((i + 1) % ((int)conv_poly.size()))]));
  // Every triangle is tested, degenerate ones too (as the point or segment they span), as the
  // legacy code tested them; a polygon of coincident Voronoi vertices (cospherical cells) is
  // counted.
  bool is_degenerate_poly = true;
  for (const Triangle_3 &tr : t) {
    is_degenerate_poly &= tr.is_degenerate();
    if (cg::do_intersect(tr, s))
      return true;
  }
  if (is_degenerate_poly)
    ++diag.degenerate_polygon;
  return false;
}

// ---- hfn_util.cpp's predicates on the flow ----

bool is_maxima(const Cell_handle &c) {
  return (!is_outflow(Facet(c, 0)) && !is_outflow(Facet(c, 1)) && !is_outflow(Facet(c, 2)) &&
          !is_outflow(Facet(c, 3)));
}

bool is_outflow(const Facet &f) {
  Cell_handle c = f.first;
  int id = f.second;
  Point VV = c->voronoi();
  Point p[4] = {c->vertex((id + 1) % 4)->point(), c->vertex((id + 2) % 4)->point(),
                c->vertex((id + 3) % 4)->point(), c->vertex(id)->point()};
  return (cg::to_double((Tetrahedron(p[0], p[1], p[2], p[3]).volume()) *
                        (Tetrahedron(p[0], p[1], p[2], VV).volume())) < 0);
}

bool is_transversal_flow(const Facet &f) {
  Cell_handle c = f.first;
  int id = f.second;
  Point p[3];
  for (int i = 0; i < 3; i++)
    p[i] = c->vertex((id + i + 1) % 4)->point();
  for (int i = 0; i < 3; i++) {
    Vector v0 = p[(i + 1) % 3] - p[i];
    Vector v1 = p[(i + 2) % 3] - p[i];
    if (cg::to_double(v0 * v1) < 0)
      return true;
  }
  return false;
}

bool find_acceptor(const Cell_handle &c, const int &id, int &uid, int &vid, int &wid) {
  if (!is_transversal_flow(Facet(c, id)))
    return false;
  Point p[3] = {c->vertex((id + 1) % 4)->point(), c->vertex((id + 2) % 4)->point(),
                c->vertex((id + 3) % 4)->point()};
  for (int i = 0; i < 3; i++)
    if (is_obtuse(p[(i + 1) % 3], p[(i + 2) % 3], p[i]))
      wid = (id + i + 1) % 4;
  vertex_indices(id, wid, uid, vid);
  return true;
}

bool is_i2_saddle(const Facet &f) {
  Cell_handle c[2];
  int id[2];
  c[0] = f.first;
  id[0] = f.second;
  c[1] = c[0]->neighbor(id[0]);
  id[1] = c[1]->index(c[0]);
  Point p[3];
  p[0] = c[0]->vertex((id[0] + 1) % 4)->point();
  p[1] = c[0]->vertex((id[0] + 2) % 4)->point();
  p[2] = c[0]->vertex((id[0] + 3) % 4)->point();
  Tetrahedron t[2];
  t[0] = Tetrahedron(p[0], p[1], p[2], c[0]->voronoi());
  t[1] = Tetrahedron(p[0], p[1], p[2], c[1]->voronoi());
  if (cg::to_double(t[0].volume() * t[1].volume()) >= 0)
    return false;
  for (int i = 0; i < 3; i++) {
    Vector v1 = c[0]->vertex((id[0] + (i + 1) % 3 + 1) % 4)->point() -
                c[0]->vertex((id[0] + i + 1) % 4)->point();
    Vector v2 = c[0]->vertex((id[0] + (i + 2) % 3 + 1) % 4)->point() -
                c[0]->vertex((id[0] + i + 1) % 4)->point();
    if (cosine(v1, v2) < 0)
      return false;
  }
  return true;
}

bool is_i1_saddle(const Edge &e, const Triangulation &triang, diagnostics &diag) {
  vector<Point> VF;
  Facet_circulator fcirc = triang.incident_facets(e);
  Facet_circulator begin = fcirc;
  do {
    if (triang.is_infinite((*fcirc).first))
      return false;
    VF.push_back((*fcirc).first->voronoi());
    fcirc++;
  } while (fcirc != begin);
  return does_intersect_convex_polygon_segment_3_in_3d(
      VF, Segment(e.first->vertex(e.second)->point(), e.first->vertex(e.third)->point()), diag);
}

// ---- degen.cpp ----
// Takes a cell c, a facet (c, fid), an edge (c, uid, vid) and the driver d of the VF dual to the
// edge, and reports a degenerate VF: one of its VVs is infinite; the duals of two of its VVs are
// cospherical; or the vector from d to the VV dual(c) is not within the VF. The legacy code wrote
// the last case to "<prefix>.degen_VF" in the current directory; here it is counted.
bool is_degenerate_VF(const Triangulation &triang, const Cell_handle &c, const int &fid,
                      const int &uid, const int &vid, const Point &d, diagnostics &diag) {
  if (triang.is_infinite(c) || triang.is_infinite(c->neighbor(fid)) ||
      triang.is_infinite(c->neighbor(6 - fid - uid - vid)))
    return true;
  Facet_circulator fcirc = triang.incident_facets(Edge(c, uid, vid));
  Facet_circulator begin = fcirc;
  do {
    if (triang.is_infinite((*fcirc).first))
      return true; // check 1
    if (is_cospherical_pair(triang, Facet((*fcirc).first, (*fcirc).second)))
      return true; // check 2
    fcirc++;
  } while (fcirc != begin);
  // check 3
  Point vv[3];
  vv[0] = c->voronoi();
  vv[1] = c->neighbor(fid)->voronoi();
  vv[2] = c->neighbor(6 - fid - uid - vid)->voronoi();
  Vector v[3];
  v[0] = vv[0] - d;
  v[1] = vv[1] - d;
  v[2] = vv[2] - d;
  Vector v1xv0 = cg::cross_product(v[1], v[0]);
  Vector v0xv2 = cg::cross_product(v[0], v[2]);
  if (cg::to_double(v1xv0 * v0xv2) < 0) {
    ++diag.degenerate_VF;
    return true;
  }
  return false;
}

} // namespace cvc::cocone
