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

// rcocone.cpp of VolumeRover's Skeletonization: Robust Cocone, the optional pre-filter that keeps
// the points of a noisy sample which lie on a smooth surface (big Delaunay balls that intersect
// deeply are outside). The legacy version that wrote "<prefix>.tcip" stays behind; this is the one
// that returned the points.

#include "cocone.h"

namespace cvc::cocone {

namespace {

step_task compute_cell_radius(Triangulation &triang) {
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); cit++) {
    SK_UNIT;
    double r = cg::to_double((cit->voronoi() - cit->vertex(0)->point()) *
                             (cit->voronoi() - cit->vertex(0)->point()));
    if (std::isnan(r) || std::isinf(r))
      cit->set_cell_radius(DBL_MAX);
    else if (r <= 0)
      cit->set_cell_radius(0);
    else
      cit->set_cell_radius(r);
  }
}

// Insert le into a vertex's three nearest-neighbour distances (sorted, smallest first).
void push_nnd(Vertex_handle u, double le) {
  for (int i = 0; i < (int)u->nnd_vector.size(); i++) {
    if (u->nnd_vector[std::size_t(i)] < le)
      continue;
    double push_val = le;
    for (int j = i; j < (int)u->nnd_vector.size(); j++) {
      double temp = u->nnd_vector[std::size_t(j)];
      u->nnd_vector[std::size_t(j)] = push_val;
      push_val = temp;
    }
    break;
  }
}

// ------------------------------
// mark_big_tetrahedral_balls
// ------------------------------
// Mark the tetrahedra whose circumscribing balls are big compared to the distance to the nearest
// neighbours of one of their corners.
// ------------------------------
step_task mark_big_tetrahedral_balls(Triangulation &triang, const double &bb_ratio) {
  // nnd(v) = the average of the distances to the first three nearest neighbors.
  for (FEI eit = triang.finite_edges_begin(); eit != triang.finite_edges_end(); eit++) {
    SK_UNIT;
    Cell_handle ch = (*eit).first;
    Vertex_handle u = ch->vertex((*eit).second);
    Vertex_handle v = ch->vertex((*eit).third);
    double le = cg::to_double((u->point() - v->point()) * (u->point() - v->point()));
    push_nnd(u, le);
    push_nnd(v, le);
  }
  // for each cell check if the circumradius is big compared to the nnd of at least one of its
  // corners.
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); cit++) {
    SK_UNIT;
    double r = cit->cell_radius();
    for (int i = 0; i < 4; i++) {
      double nnd = (1. / 3.) * (cit->vertex(i)->nnd_vector[0] + cit->vertex(i)->nnd_vector[1] +
                                cit->vertex(i)->nnd_vector[2]);
      if (r > bb_ratio * nnd)
        cit->set_bb(true);
    }
  }
}

// -------------------------------
// identify_deep_intersection
// -------------------------------
// Identify which two big balls really intersect deeply.
// -------------------------------
step_task identify_deep_intersection(Triangulation &triang, const double theta_ff,
                                     const double theta_if) {
  for (AFI fit = triang.all_facets_begin(); fit != triang.all_facets_end(); fit++) {
    SK_UNIT;
    Cell_handle c[2];
    int id[2];
    c[0] = (*fit).first;
    id[0] = (*fit).second;
    c[1] = c[0]->neighbor(id[0]);
    id[1] = c[1]->index(c[0]);
    // case 1: both infinite: assumed outside, big and deeply intersecting.
    if (triang.is_infinite(c[0]) && triang.is_infinite(c[1]))
      continue;
    // case 2: one infinite and one finite.
    if (triang.is_infinite(c[0]) || triang.is_infinite(c[1])) {
      if (triang.is_infinite(c[0])) {
        Cell_handle temp = c[0];
        c[0] = c[1];
        c[1] = temp;
        id[0] = c[0]->index(c[1]);
        id[1] = c[1]->index(c[0]);
      }
      if (!c[0]->bb())
        continue;
      Point p[4];
      p[0] = c[0]->vertex(id[0])->point();
      p[1] = c[0]->vertex((id[0] + 1) % 4)->point();
      p[2] = c[0]->vertex((id[0] + 2) % 4)->point();
      p[3] = c[0]->vertex((id[0] + 3) % 4)->point();
      // if the cell is not flowing into the infinite cell then the intersection is not deep.
      if (cg::orientation(p[1], p[2], p[3], p[0]) ==
          cg::orientation(p[1], p[2], p[3], c[0]->voronoi()))
        continue;
      // deep intersection <=> r_f/r_t < sin(theta), r_f the facet's circumradius and r_t the
      // tetrahedron's.
      double r_f = sq_cr_tr_3(p[1], p[2], p[3]);
      double r_t = c[0]->cell_radius();
      if (r_f / r_t < sin(theta_if) * sin(theta_if)) {
        c[0]->set_deep_int(id[0], true);
        c[1]->set_deep_int(id[1], true);
      }
      continue;
    }
    // case 3: both finite.
    if (!c[0]->bb() || !c[1]->bb())
      continue;
    // Do the balls (p, r_p) and (q, r_q) intersect deeply: ||p-q||^2 against the radii.
    Point p = c[0]->voronoi();
    Point q = c[1]->voronoi();
    double r_p = c[0]->cell_radius();
    double r_q = c[1]->cell_radius();
    double d = cg::to_double((p - q) * (p - q));
    double r1 = (r_p > r_q) ? r_p : r_q;
    double r2 = r_p + r_q - r1;
    // d <= r1 + r2 - 2*sqrt(r1*r2)*cos(theta).
    if (d <= r1 + r2 - 2 * std::sqrt(r1 * r2) * cos(theta_ff)) {
      c[0]->set_deep_int(id[0], true);
      c[1]->set_deep_int(id[1], true);
    }
  }
}

// -------------------------------
// walk
// ------
// Walk among the cells to collect the connected components.
// -------------------------------
step_task rc_walk(Triangulation &triang) {
  for (ACI cit = triang.all_cells_begin(); cit != triang.all_cells_end(); cit++) {
    SK_UNIT;
    cit->visited = false;
    cit->outside = triang.is_infinite(cit);
  }
  // start from a cell which has a big circumscribing ball and which is flowing into infinity.
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); cit++) {
    SK_UNIT;
    if (cit->visited)
      continue;
    if (!cit->bb())
      continue;
    bool start_flag = false;
    Cell_handle start_cell = cit;
    for (int i = 0; i < 4; i++) {
      if (!triang.is_infinite(cit->neighbor(i)))
        continue;
      // so this cell has a face on the convex hull: does its ball intersect the infinite ball
      // deeply.
      if (cit->deep_int(i))
        start_flag = true;
    }
    if (!start_flag)
      continue;
    vector<Cell_handle> cell_stack;
    cell_stack.push_back(start_cell);
    start_cell->visited = true;
    while (!cell_stack.empty()) {
      SK_UNIT;
      Cell_handle c = cell_stack.back();
      cell_stack.pop_back();
      c->outside = true;
      // the neighbours with big balls, unvisited, finite and deeply intersecting.
      for (int i = 0; i < 4; i++) {
        if (c->neighbor(i)->visited)
          continue;
        if (triang.is_infinite(c->neighbor(i)))
          continue;
        if (!c->neighbor(i)->bb())
          continue;
        if (!c->deep_int(i))
          continue;
        cell_stack.push_back(c->neighbor(i));
        c->neighbor(i)->visited = true;
      }
    }
  }
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); cit++) {
    SK_UNIT;
    if (!cit->outside) {
      for (int i = 0; i < 4; i++) {
        if (triang.is_infinite(cit->neighbor(i))) {
          cit->vertex((i + 1) % 4)->set_on_smooth_surface(true);
          cit->vertex((i + 2) % 4)->set_on_smooth_surface(true);
          cit->vertex((i + 3) % 4)->set_on_smooth_surface(true);
        }
      }
      continue;
    }
    for (int i = 0; i < 4; i++)
      cit->vertex(i)->set_on_smooth_surface(true);
  }
}

} // namespace

step_task robust_cocone(const double bb_ratio, const double theta_ff, const double theta_if,
                        Triangulation &triang, std::vector<Point> &result) {
  SK_RUN(compute_cell_radius(triang));
  SK_RUN(mark_big_tetrahedral_balls(triang, bb_ratio));
  SK_RUN(identify_deep_intersection(triang, theta_ff, theta_if));
  SK_RUN(rc_walk(triang));
  result.clear();
  for (FVI vit = triang.finite_vertices_begin(); vit != triang.finite_vertices_end(); vit++) {
    SK_UNIT;
    if (vit->on_smooth_surface())
      result.push_back(vit->point());
  }
}

} // namespace cvc::cocone
