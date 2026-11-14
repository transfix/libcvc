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

// init.cpp and robust_cc.cpp of VolumeRover's Skeletonization: the triangulation's attributes,
// the Voronoi vertices and cell radii, and circumcentres that tolerate degenerate input.

#include "cocone.h"

namespace cvc::cocone {

// ---------------------------------------------------------
// initialize
// ----------
// Initialize some of the attributes of the triangulation.
// ---------------------------------------------------------
step_task initialize(Triangulation &triang) {
  // set vertex id.
  int id = 0;
  for (FVI vit = triang.finite_vertices_begin(); vit != triang.finite_vertices_end(); vit++) {
    SK_UNIT;
    vit->id = id++;
    vit->visited = false;
    vit->bad = false;
    vit->bad_neighbor = false;
  }
  // set cell id.
  id = 0;
  for (ACI cit = triang.all_cells_begin(); cit != triang.all_cells_end(); cit++) {
    SK_UNIT;
    cit->id = id++;
    cit->visited = false;
    cit->outside = false;
    cit->transp = false;
    for (int i = 0; i < 4; i++) {
      cit->set_cocone_flag(i, false);
      cit->neighbor(i)->set_cocone_flag(cit->neighbor(i)->index(cit), false);
      cit->bdy[i] = false;
      cit->opaque[i] = false;
      for (int k = 0; k < 4; k++)
        cit->umbrella_member[i][k] = -1;
    }
    // set the convex hull points.
    if (!triang.is_infinite(cit))
      continue;
    for (int i = 0; i < 4; i++) {
      if (!triang.is_infinite(cit->vertex(i)))
        continue;
      cit->vertex((i + 1) % 4)->set_convex_hull(true);
      cit->vertex((i + 2) % 4)->set_convex_hull(true);
      cit->vertex((i + 3) % 4)->set_convex_hull(true);
    }
  }
}

// ---------------------------------------------------------
// compute_voronoi_vertex_and_cell_radius
// --------------------------------------
// Compute all the finite voronoi vertices and the circumradius of all the finite cells. A cell
// whose circumcentre or radius does not fit in a double is "dirty": its circumcentre is
// approximated by one of its facets' (the legacy code also wrote such cells to "degen_tetra" in
// the current directory; here they are counted).
// ---------------------------------------------------------
step_task compute_voronoi_vertex_and_cell_radius(Triangulation &triang, diagnostics &diag) {
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); cit++) {
    SK_UNIT;
    bool is_correct_computation = true;
    cit->set_voronoi(nondg_voronoi_point(cit->vertex(0)->point(), cit->vertex(1)->point(),
                                         cit->vertex(2)->point(), cit->vertex(3)->point(),
                                         is_correct_computation));
    if (is_correct_computation) {
      bool is_correct_radius = true;
      // check if the radius fits good in double.
      double r = cg::to_double((cit->voronoi() - cit->vertex(0)->point()) *
                               (cit->voronoi() - cit->vertex(0)->point()));
      cit->set_cell_radius(r);
      if (std::isnan(r) || std::isinf(r))
        is_correct_radius = false;
      if (r <= 0)
        is_correct_radius = false;
      if (is_correct_radius)
        continue;
    }
    cit->set_dirty(true);
    ++diag.dirty_cells;
    // Approximate the circumcenter of the cell by the circumcenter of one of its facets.
    cit->set_voronoi(dg_voronoi_point(cit->vertex(0)->point(), cit->vertex(1)->point(),
                                      cit->vertex(2)->point(), cit->vertex(3)->point(),
                                      is_correct_computation));
    double r = cg::to_double((cit->voronoi() - cit->vertex(0)->point()) *
                             (cit->voronoi() - cit->vertex(0)->point()));
    cit->set_cell_radius(r);
  }
}

// -------------------------------------------------------------
// nondg_voronoi_point
// -------------------------------------------------------------
// The circumcentre, assuming no degeneracy. CGAL's EPICK computed it in doubles and never
// reported a failure (the legacy failure handler was commented out), so neither does this;
// a degenerate cell shows as a radius that is not finite.
// -------------------------------------------------------------
Point nondg_voronoi_point(const Point &a, const Point &b, const Point &c, const Point &d,
                          bool &is_correct_computation) {
  is_correct_computation = true;
  return cg::circumcenter(a, b, c, d);
}

// -------------------------------------------------------------
// dg_voronoi_point
// -------------------------------------------------------------
// A Voronoi point for a degenerate (coplanar) cell: the circumcentre of one of its facets that
// is not degenerate, or else the average of its four points.
// -------------------------------------------------------------
Point dg_voronoi_point(const Point &a, const Point &b, const Point &c, const Point &d,
                       bool &is_correct_computation) {
  Point cc = cg::ORIGIN;
  is_correct_computation = false;
  Point p[4] = {a, b, c, d};
  for (int i = 0; i < 4; i++) {
    // first check if the facet is degenerate or not.
    Triangle_3 t = Triangle_3(p[(i + 1) % 4], p[(i + 2) % 4], p[(i + 3) % 4]);
    if (t.is_degenerate())
      continue;
    cc = nondg_cc_tr_3(p[(i + 1) % 4], p[(i + 2) % 4], p[(i + 3) % 4], is_correct_computation);
    if (is_correct_computation)
      break;
  }
  if (is_correct_computation)
    return cc;
  // What should be the circumcenter of a tetrahedron whose four points are collinear? The
  // average.
  cc = cg::ORIGIN + (0.25 * Vector(a[0] + b[0] + c[0] + d[0], a[1] + b[1] + c[1] + d[1],
                                   a[2] + b[2] + c[2] + d[2]));
  return cc;
}

// The circumcentre of a triangle in space, assuming no degeneracy.
Point nondg_cc_tr_3(const Point &a, const Point &b, const Point &c, bool &is_correct_computation) {
  is_correct_computation = true;
  return cg::circumcenter(a, b, c);
}

// The circumcentre of a triangle in space; the average of the three points when they are
// collinear.
Point cc_tr_3(const Point &a, const Point &b, const Point &c) {
  if (!cg::collinear(a, b, c)) {
    bool ok = true;
    return nondg_cc_tr_3(a, b, c, ok);
  }
  return cg::ORIGIN +
         ((1. / 3.) * Vector(a[0] + b[0] + c[0], a[1] + b[1] + c[1], a[2] + b[2] + c[2]));
}

// The squared circumradius of a triangle in space.
double sq_cr_tr_3(const Point &a, const Point &b, const Point &c) {
  Point cc = cc_tr_3(a, b, c);
  return cg::to_double((cc - a) * (cc - a));
}

Point circumcenter(const Facet &f) {
  Cell_handle c = f.first;
  int id = f.second;
  Point p[3];
  for (int i = 0; i < 3; i++)
    p[i] = c->vertex((id + (i + 1)) % 4)->point();
  return cc_tr_3(p[0], p[1], p[2]);
}

double circumradius(const Facet &f) {
  Point c = circumcenter(f);
  Point p = f.first->vertex((f.second + 1) % 4)->point();
  return std::sqrt(cg::to_double((p - c) * (p - c)));
}

} // namespace cvc::cocone
