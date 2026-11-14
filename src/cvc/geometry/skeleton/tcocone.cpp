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

// tcocone.cpp of VolumeRover's Skeletonization: Cocone (poles, flat vertices, candidate facets,
// the umbrella safety check, pruning, the walk), then Tight Cocone's marking and peeling, which
// make the surface watertight. Each function is a coroutine that yields once per iteration of its
// loops and walks (stepping.h); otherwise the code is the legacy one.

#include "cocone.h"

namespace cvc::cocone {

namespace {

// Make (ch, id) name a finite cell: a finite facet always has one.
inline void finite_side(const Triangulation &triang, Cell_handle &ch, int &id) {
  if (triang.is_infinite(ch)) {
    Cell_handle c = ch;
    ch = ch->neighbor(id);
    id = ch->index(c);
  }
}

// ------------------------------------
// Surface Reconstruction with boundary
// ------------------------------------

// -----------------------------------------------------------------------
// compute_poles
// -------------
// Computes and stores normals and poles in the triangulation.
// -----------------------------------------------------------------------
step_task compute_poles(Triangulation &triang) {
  // Compute the center of mass of the sample points.
  Vector center_of_mass = cg::NULL_VECTOR;
  for (FVI vit = triang.finite_vertices_begin(); vit != triang.finite_vertices_end(); ++vit) {
    SK_UNIT;
    center_of_mass = center_of_mass + (vit->point() - cg::ORIGIN);
  }
  center_of_mass = (1. / triang.number_of_vertices()) * center_of_mass;
  // Compute the normals.
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); ++cit) {
    SK_UNIT;
    for (int id = 0; id < 4; ++id) {
      // If the neighboring cell of *cit opposite to the vertex with index id is infinite then
      // all vertices of *cit different from the vertex with index id are on the boundary of the
      // convex hull. For such vertices the normal is computed as the average of the normals of
      // the incident facets on the convex hull.
      if (triang.is_infinite(cit->neighbor(id))) {
        Vector facet_normal = cg::cross_product(
            cit->vertex((id + 2) % 4)->point() - cit->vertex((id + 1) % 4)->point(),
            cit->vertex((id + 3) % 4)->point() - cit->vertex((id + 1) % 4)->point());
        // Normalize and choose a consistent orientation.
        facet_normal = facet_normal / cg::sqrt(facet_normal.x() * facet_normal.x() +
                                               facet_normal.y() * facet_normal.y() +
                                               facet_normal.z() * facet_normal.z());
        if (facet_normal * (cit->vertex((id + 1) % 4)->point() - (cg::ORIGIN + center_of_mass)) < 0)
          facet_normal = -facet_normal;
        // Add facet normal to all vertices of the facet opposite to the vertex with index i.
        for (int i = 1; i <= 3; ++i) {
          Vertex_handle vh = cit->vertex((id + i) % 4);
          // Flatness marking shouldn't be done here!!
          if (cosine(vh->normal(), facet_normal) < 0) {
            vh->set_flat(false);
            continue;
          }
          Vector new_normal = vh->normal() + facet_normal;
          new_normal = new_normal /
                       cg::sqrt(new_normal.x() * new_normal.x() + new_normal.y() * new_normal.y() +
                                new_normal.z() * new_normal.z());
          vh->set_normal(new_normal);
        }
      }
      // Update the normal at the vertex with index id if necessary.
      Vertex_handle vh = cit->vertex(id);
      if ((!vh->on_convex_hull()) &&
          (vh->normal() * vh->normal()) <=
              (cit->voronoi() - vh->point()) * (cit->voronoi() - vh->point()))
        vh->set_normal(cit->voronoi() - vh->point());
    }
  }
  for (FVI vit = triang.finite_vertices_begin(); vit != triang.finite_vertices_end(); ++vit) {
    SK_UNIT;
    vit->set_pole(vit->point());
  }
  // Compute the poles.
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); ++cit) {
    SK_UNIT;
    for (int id = 0; id < 4; ++id) {
      Vertex_handle vh = cit->vertex(id);
      Vector old_pole = vh->pole() - vh->point();
      Vector new_pole = vh->point() - cit->voronoi();
      if ((cg::to_double(old_pole * old_pole) < cg::to_double(new_pole * new_pole)) &&
          ((cit->voronoi() - vh->point()) * vh->normal() < 0))
        vh->set_pole(cit->voronoi());
    }
  }
}

// Whether w's cocone meets the Voronoi edge dual to facet (ch, id): the legacy "cocone neighbor"
// test, cos1/cos2 against [min_cos, max_cos].
bool cocone_neighbor(const Triangulation &triang, Cell_handle ch, int id, Vertex_handle w,
                     double min_cos, double max_cos) {
  double cos1 = cosine(ch->voronoi() - w->point(), w->normal());
  double cos2 = (triang.is_infinite(ch->neighbor(id)))
                    ? 1.0
                    : cosine(ch->neighbor(id)->voronoi() - w->point(), w->normal());
  if (cos1 > cos2)
    swap(cos1, cos2);
  return !((cos2 < min_cos) || (cos1 > max_cos));
}

// -----------------------------------------------------------------------
// mark_flat_vertices
// ------------------
// Computes and marks flat vertices in the triangulation.
// -----------------------------------------------------------------------
step_task mark_flat_vertices(Triangulation &triang, double ratio, double cocone_phi,
                             double flat_phi) {
  double min_cos = cos(M_PI / 2.0 + cocone_phi); // Bounds for the co-cone.
  double max_cos = cos(M_PI / 2.0 - cocone_phi);
  // Compute the center of mass of the sample points.
  Vector center_of_mass = cg::NULL_VECTOR;
  for (FVI vit = triang.finite_vertices_begin(); vit != triang.finite_vertices_end(); ++vit) {
    SK_UNIT;
    center_of_mass = center_of_mass + (vit->point() - cg::ORIGIN);
  }
  center_of_mass = (1. / triang.number_of_vertices()) * center_of_mass;
  // Compute the coccone width, i.e. the diameter of the copolygon which is the largest distance
  // of the sample point to a vertex of the copolygon. To do so iterate over the Voronoi edges
  // which means iterating over their duals, i.e. the Delaunay facets.
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); ++fit) {
    SK_UNIT;
    Cell_handle ch = (*fit).first;
    int id = (*fit).second;
    finite_side(triang, ch, id);
    // Compute the facet normal or the Voronoi edge respectively.
    Vector facet_normal;
    if (triang.is_infinite(ch->neighbor(id))) {
      facet_normal =
          cg::cross_product(ch->vertex((id + 1) % 4)->point() - ch->vertex((id + 2) % 4)->point(),
                            ch->vertex((id + 1) % 4)->point() - ch->vertex((id + 3) % 4)->point());
      // Ensure that the orientation is correct.
      if (facet_normal * (ch->vertex((id + 1) % 4)->point() - (cg::ORIGIN + center_of_mass)) < 0)
        facet_normal = -facet_normal;
    } else // Facet normal connects to Voronoi vertices.
      facet_normal = ch->neighbor(id)->voronoi() - ch->voronoi();
    // Update the width of co-cones of the vertices incident to the facet if necessary, i.e. if
    // there is a co-polygon neighbor with larger distance to the vertex.
    for (int i = 1; i < 4; ++i) {
      Vertex_handle vh = ch->vertex((id + i) % 4);
      double cos1 = cosine(ch->voronoi() - vh->point(), vh->normal());
      double cos2 = (triang.is_infinite(ch->neighbor(id)))
                        ? 1.0
                        : cosine((ch->voronoi() + facet_normal) - vh->point(), vh->normal());
      if (cos1 > cos2)
        swap(cos1, cos2);
      if ((cos2 >= 0.0) && (cos1 <= 0.0)) {
        // Compute the intersection point of the Voronoi edge with the co-polygon, i.e. co-cone
        // with opening angle zero.
        Point p = ch->voronoi() + ((vh->point() - ch->voronoi()) * vh->normal()) /
                                      (facet_normal * vh->normal()) * facet_normal;
        double w = cg::to_double((p - vh->point()) * (p - vh->point()));
        if (vh->width() < w)
          vh->set_width(w);
      }
    }
  }
  // Now check the flatness property for every vertex.
  for (FVI vit = triang.finite_vertices_begin(); vit != triang.finite_vertices_end(); ++vit) {
    SK_UNIT;
    if (!vit->is_flat())
      continue;
    double scaled_width = ratio * vit->width();
    double second_height =
        cg::to_double((vit->pole() - vit->point()) * (vit->pole() - vit->point()));
    if (scaled_width > second_height) {
      vit->set_flat(false);
      continue;
    }
    // If the sample point lies on the boundary of the triangulation it obviously passes the
    // flatness test in the normal direction. Otherwise we have to do the test in the normal
    // direction.
    if (!vit->on_convex_hull()) {
      double first_height = cg::to_double((vit->normal() * vit->normal()));
      if (scaled_width > first_height)
        vit->set_flat(false);
    }
  }
  // Second stage of boundary detection.
  for (FVI vit = triang.finite_vertices_begin(); vit != triang.finite_vertices_end(); ++vit) {
    SK_UNIT;
    vit->tag = vit->is_flat();
  }
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); ++fit) {
    SK_UNIT;
    Cell_handle ch = (*fit).first;
    int id = (*fit).second;
    finite_side(triang, ch, id);
    for (int i = 1; i < 4; ++i) {
      Vertex_handle v = ch->vertex((id + i) % 4);
      // Process only flat vertices.
      if (!v->is_flat())
        continue;
      // Normal Check.
      for (int j = 1; j < 4; ++j) {
        if (j == i)
          continue;
        // Check if *v is cocone neighbor of *w.
        Vertex_handle w = ch->vertex((id + j) % 4);
        if (!cocone_neighbor(triang, ch, id, w, min_cos, max_cos))
          continue;
        double cos3 = cosine(v->normal(), w->normal());
        if ((cos3 < cos(flat_phi) && cos3 > cos(M_PI - flat_phi)))
          v->set_flat(false);
      }
    }
  }
  bool update = true;
  while (update) {
    update = false;
    for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); ++fit) {
      SK_UNIT;
      Cell_handle ch = (*fit).first;
      int id = (*fit).second;
      finite_side(triang, ch, id);
      for (int i = 1; i < 4; ++i) {
        Vertex_handle v = ch->vertex((id + i) % 4);
        // Process only non flat vertices which fulfill the ratio condition.
        if (v->is_flat() || !v->tag)
          continue;
        for (int j = 1; j < 4; ++j) {
          if (j == i)
            continue;
          Vertex_handle w = ch->vertex((id + j) % 4);
          if (!cocone_neighbor(triang, ch, id, w, min_cos, max_cos))
            continue;
          double cos3 = cosine(v->normal(), w->normal());
          if ((cos3 > cos(flat_phi) || cos3 < cos(M_PI - flat_phi))) {
            v->set_flat(true);
            update = true;
          }
        }
      }
    }
  }
}

// -----------------------------------------------------------------------
// filter_candidates
// -----------------
// First filter of candidate triangles.
// -----------------------------------------------------------------------
step_task filter_candidates(Triangulation &triang, double phi) {
  double min_cos = cos(M_PI / 2.0 + phi); // Bounds for the co-cone.
  double max_cos = cos(M_PI / 2.0 - phi);
  // Check for the co-cone condition for every finite facet.
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); ++fit) {
    SK_UNIT;
    Cell_handle ch = (*fit).first;
    int id = (*fit).second;
    finite_side(triang, ch, id);
    // Iterate over the vertices incident to the facet.
    bool choose_facet = true;
    bool tested = false;
    for (int i = 1; i < 4; ++i) {
      Vertex_handle vh = ch->vertex((id + i) % 4);
      // Only flat vertices are allowed to choose a facet.
      if (!vh->is_flat())
        continue;
      tested = true;
      if (!cocone_neighbor(triang, ch, id, vh, min_cos, max_cos))
        choose_facet = false;
    }
    // Set the cocone flag.
    ch->set_cocone_flag(id, choose_facet && tested);
    ch->neighbor(id)->set_cocone_flag(ch->neighbor(id)->index(ch), choose_facet && tested);
  }
}

// The cosine of the normals of facets (u, v, w) and (w, v, z) across their common edge vw.
inline double normals_cosine(Vertex_handle u, Vertex_handle v, Vertex_handle w, Vertex_handle z) {
  return cosine(cg::cross_product(u->point() - v->point(), w->point() - v->point()),
                cg::cross_product(w->point() - v->point(), z->point() - v->point()));
}

// The largest (max = true) or smallest cosine between the normals of two marked finite facets
// around the edge whose circulator starts at `begin`, and how many such facets there are.
double extreme_facet_cosine(const Triangulation &triang, Facet_circulator begin, Vertex_handle v,
                            Vertex_handle w, bool max, int &cnt) {
  Facet_circulator fcirc = begin;
  double best = max ? -2.0 : 2.0;
  cnt = 0;
  do {
    Cell_handle cell = (*fcirc).first;
    int fid = (*fcirc).second;
    // Consider only finite marked facets.
    if (!cell->cocone_flag(fid) || triang.is_infinite(*fcirc)) {
      ++fcirc;
      continue;
    }
    ++cnt;
    int iu = find_third_vertex_index(*fcirc, v, w);
    Vertex_handle u = cell->vertex(iu);
    Facet_circulator gcirc = fcirc;
    ++gcirc;
    for (; gcirc != begin; ++gcirc) {
      cell = (*gcirc).first;
      int gid = (*gcirc).second;
      if ((!cell->cocone_flag(gid)) || triang.is_infinite(*gcirc))
        continue;
      int iz = find_third_vertex_index(*gcirc, v, w);
      Vertex_handle z = cell->vertex(iz);
      double normals_cos = normals_cosine(u, v, w, z);
      if (max ? normals_cos > best : normals_cos < best)
        best = normals_cos;
    }
    ++fcirc;
  } while (fcirc != begin);
  return best;
}

// -----------------------------------------------------------------------
// safety_check
// --------------
// Mark vertices in the triangulation as boundary vertices if they do not have a non sharp
// umbrella in the complex. Mark candidate facets non removable if they are incident to an edge
// that has only two incident candidate triangles which make a large dihedral angle with each
// other.
// -----------------------------------------------------------------------
step_task safety_check(Triangulation &triang, double sharp_phi) {
  typedef cg::Triple<Cell_handle, int, int> Anchored_facet;
  double sharp_cos = cos(sharp_phi);
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); ++fit) {
    SK_UNIT;
    Cell_handle ch = (*fit).first;
    int id = (*fit).second;
    // Consider only marked facets.
    if (!ch->cocone_flag(id))
      continue;
    for (int i = 1; i < 4; ++i) {
      Vertex_handle v = ch->vertex((id + i) % 4);
      v->set_isolated(false);
      // If a non sharp umbrella for v has already been found then continue.
      if (v->has_umbrella())
        continue;
      vector<Anchored_facet> facet_stack;
      int iw = (i == 3) ? (id + 1) % 4 : (id + i + 1) % 4;
      facet_stack.push_back(Anchored_facet(ch, id, iw));
      list<Vertex_handle> umbrella_vertices;
      bool found = true;
      while (!facet_stack.empty()) {
        SK_UNIT;
        Cell_handle cell = facet_stack.back().first;
        int fid = facet_stack.back().second;
        iw = facet_stack.back().third;
        facet_stack.pop_back();
        Vertex_handle w = cell->vertex(iw);
        int iv = cell->index(v);
        int iu = edge_index(fid, iv, iw);
        Vertex_handle u = cell->vertex(iu);
        if (!found) {
          umbrella_vertices.erase(++find(umbrella_vertices.begin(), umbrella_vertices.end(), u),
                                  umbrella_vertices.end());
        }
        Facet_circulator begin = triang.incident_facets(Edge(cell, iv, iw));
        Facet_circulator fcirc = begin;
        found = false;
        do {
          cell = (*fcirc).first;
          fid = (*fcirc).second;
          // Consider only finite marked facets.
          if (!cell->cocone_flag(fid) || triang.is_infinite(*fcirc)) {
            ++fcirc;
            continue;
          }
          // Third vertex of *fcirc.
          int iz = find_third_vertex_index(*fcirc, v, w);
          Vertex_handle z = cell->vertex(iz);
          // Consider only facets different from the current facet.
          if (z->point() == u->point()) {
            ++fcirc;
            continue;
          }
          // If the cosine of the normals is larger than `sharp_cos' then check if a non sharp
          // umbrella was closed and otherwise push a new oriented facet on the stack and w in
          // the list umbrella_vertices.
          double current_cos = normals_cosine(u, v, w, z);
          if (current_cos >= sharp_cos) {
            found = true;
            if (find(umbrella_vertices.begin(), umbrella_vertices.end(), z) !=
                umbrella_vertices.end()) {
              v->set_umbrella(true);
              break;
            } else {
              umbrella_vertices.push_back(w);
              facet_stack.push_back(Anchored_facet(cell, fid, iz));
            }
          }
          ++fcirc;
        } while (fcirc != begin);
      }
    }
  }
  // Mark the facets which are not removable.
  for (FEI eit = triang.finite_edges_begin(); eit != triang.finite_edges_end(); ++eit) {
    SK_UNIT;
    Cell_handle ch = (*eit).first;
    Vertex_handle v = ch->vertex((*eit).second);
    Vertex_handle w = ch->vertex((*eit).third);
    Facet_circulator begin = triang.incident_facets(*eit);
    int cnt = 0;
    double max_cos = extreme_facet_cosine(triang, begin, v, w, true, cnt);
    if (cnt == 2 && max_cos >= 0.7) {
      Facet_circulator fcirc = begin;
      do {
        Cell_handle cell = (*fcirc).first;
        int fid = (*fcirc).second;
        // Consider only finite marked facets.
        if (!cell->cocone_flag(fid) || triang.is_infinite(*fcirc)) {
          ++fcirc;
          continue;
        }
        cell->set_removable(fid, false);
        cell->neighbor(fid)->set_removable(cell->neighbor(fid)->index(cell), false);
        ++fcirc;
      } while (fcirc != begin);
    }
  }
}

// -----------------------------------------------------------------------
// pruning
// -------
// Deselects all triangles incident to sharp edges.
// -----------------------------------------------------------------------
step_task pruning(Triangulation &triang, double sharp_phi) {
  double sharp_cos = cos(sharp_phi);
  vector<Edge> edge_stack;
  for (FEI eit = triang.finite_edges_begin(); eit != triang.finite_edges_end(); ++eit) {
    SK_UNIT;
    Cell_handle ch = (*eit).first;
    Vertex_handle v = ch->vertex((*eit).second);
    Vertex_handle w = ch->vertex((*eit).third);
    // Process only edges for which both endpoints have an incident umbrella.
    if (!(v->has_umbrella() && w->has_umbrella()))
      continue;
    edge_stack.push_back(*eit);
    while (!edge_stack.empty()) {
      SK_UNIT;
      // The edge we cycle around.
      Edge edge = edge_stack.back();
      edge_stack.pop_back();
      v = edge.first->vertex(edge.second);
      w = edge.first->vertex(edge.third);
      Facet_circulator begin = triang.incident_facets(edge);
      int cnt = 0;
      double max_cos = extreme_facet_cosine(triang, begin, v, w, true, cnt);
      // If the edge is sharp reset its incident facets.
      if (max_cos < sharp_cos) {
        Facet_circulator fcirc = begin;
        do {
          Cell_handle c = (*fcirc).first;
          int id = (*fcirc).second;
          // The legacy code found iv and iw inside CGAL_assertion()s, which its release builds
          // compiled out, leaving both -1.
          const int iv = c->index(v);
          const int iw = c->index(w);
          int iz = find_third_vertex_index(*fcirc, v, w);
          Vertex_handle z = c->vertex(iz);
          // Consider only removable, marked finite facets.
          if (!c->cocone_flag(id) || triang.is_infinite(*fcirc) || !c->removable(id)) {
            ++fcirc;
            continue;
          }
          // Reset cocone flags on both side of facet.
          c->set_cocone_flag(id, false);
          c->neighbor(id)->set_cocone_flag(c->neighbor(id)->index(c), false);
          // Push incident edges on stack for reconsideration.
          if (v->has_umbrella() || z->has_umbrella())
            edge_stack.push_back(Edge(c, iv, iz));
          if (w->has_umbrella() || z->has_umbrella())
            edge_stack.push_back(Edge(c, iw, iz));
          ++fcirc;
        } while (fcirc != begin);
      }
    }
  }
}

// -----------------------------------------------------------------------
// walk
// ----
// Extracts a surface.
// -----------------------------------------------------------------------
step_task walk(Triangulation &triang) {
  typedef cg::Triple<Cell_handle, int, int> Oriented_facet;
  // The tag is used to mark explored facets.
  for (ACI cit = triang.all_cells_begin(); cit != triang.all_cells_end(); ++cit) {
    SK_UNIT;
    cit->tag[0] = cit->tag[1] = cit->tag[2] = cit->tag[3] = false;
  }
  // The tag is used to mark explored vertices.
  for (FVI vit = triang.finite_vertices_begin(); vit != triang.finite_vertices_end(); ++vit) {
    SK_UNIT;
    vit->tag = false;
  }
  // Main loop of the walk covers different components of the surface.
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); ++fit) {
    SK_UNIT;
    Cell_handle ch = (*fit).first;
    int id = (*fit).second;
    // Consider only marked, untagged facets.
    if (!ch->cocone_flag(id) || ch->tag[id])
      continue;
    // If one of the vertices incident to *fit are already tagged then the *fit has to be
    // unmarked, because otherwise it would introduce a topological conflict.
    if (ch->vertex((id + 1) % 4)->tag || ch->vertex((id + 2) % 4)->tag ||
        ch->vertex((id + 3) % 4)->tag) {
      ch->set_cocone_flag(id, false);
      ch->neighbor(id)->set_cocone_flag(ch->neighbor(id)->index(ch), false);
      continue;
    }
    // Don't start from the inside of a completely marked tetrahedron or from the boundary.
    if ((ch->cocone_flag(0) && ch->cocone_flag(1) && ch->cocone_flag(2) && ch->cocone_flag(3)) ||
        !ch->vertex((id + 1) % 4)->has_umbrella() || !ch->vertex((id + 2) % 4)->has_umbrella() ||
        !ch->vertex((id + 3) % 4)->has_umbrella())
      continue;
    // Prepare the initial three oriented facets from which the walk starts.
    vector<Oriented_facet> facet_stack;
    for (int i = 1; i <= 3; ++i) {
      int j = (i == 3) ? (id + 1) % 4 : (id + i + 1) % 4;
      facet_stack.push_back(Oriented_facet(ch, id, edge_index(id, (id + i) % 4, j)));
    }
    ch->tag[id] = true;
    ch->neighbor(id)->tag[ch->neighbor(id)->index(ch)] = true;
    ch->vertex((id + 1) % 4)->tag = true;
    ch->vertex((id + 2) % 4)->tag = true;
    ch->vertex((id + 3) % 4)->tag = true;
    // Main loop of the walk on a single component.
    while (!facet_stack.empty()) {
      SK_UNIT;
      Cell_handle cell = facet_stack.back().first;
      int fid = facet_stack.back().second;
      int eid = facet_stack.back().third;
      facet_stack.pop_back();
      int iv, iw;
      vertex_indices(fid, eid, iv, iw);
      Vertex_handle v = cell->vertex(iv);
      Vertex_handle w = cell->vertex(iw);
      Cell_handle old_cell = cell;
      Cell_handle new_cell = old_cell->neighbor(eid);
      bool found = false;
      do {
        fid = old_cell->index(new_cell);
        Facet facet = Facet(old_cell, fid);
        iv = old_cell->index(v);
        iw = old_cell->index(w);
        Vertex_handle z = old_cell->vertex(edge_index(fid, iv, iw));
        if (old_cell->cocone_flag(fid)) {
          if (!(triang.is_infinite(facet) || found)) {
            found = true;
            if (!old_cell->tag[fid]) {
              z->tag = true;
              old_cell->tag[fid] = true;
              new_cell->tag[new_cell->index(old_cell)] = true;
              if (w->has_umbrella() || z->has_umbrella())
                facet_stack.push_back(Oriented_facet(old_cell, fid, iv));
              if (v->has_umbrella() || z->has_umbrella())
                facet_stack.push_back(Oriented_facet(old_cell, fid, iw));
            }
          } else {
            // Reset cocone flags of facet.
            old_cell->set_cocone_flag(fid, false);
            new_cell->set_cocone_flag(new_cell->index(old_cell), false);
          }
        }
        old_cell = new_cell;
        new_cell = new_cell->neighbor(new_cell->index(z));
      } while (new_cell != cell);
    }
  }
}

// -----------------------------------------------------------------------
// cocone
// ------
// reconstructs the surface of an unorganized set of points.
// -----------------------------------------------------------------------
step_task cocone(double cocone_angle, double sharp_angle, double flat_angle, double ratio,
                 Triangulation &triang) {
  SK_RUN(compute_poles(triang));
  SK_RUN(mark_flat_vertices(triang, ratio, cocone_angle, flat_angle));
  SK_RUN(filter_candidates(triang, cocone_angle));
  SK_RUN(safety_check(triang, sharp_angle));
  SK_RUN(pruning(triang, sharp_angle));
  SK_RUN(walk(triang));
}

// ----------------------------------------------------------------------------
// Watertight Surface Reconstruction
// ----------------------------------------------------------------------------
// 1. mark_bad_points(triang)
//      calls : i> identify_vertices_with_dirty_umbrella(triang)
//             ii> check_sharp_facets(triang)
// 2. mark_in_out(triang)
//      calls : propagate_in_out_marking(triang, stack)
// 3. create_surface(tring, bool b1, bool b2)
//      calls : grow_boundary(triang, stack, bool b1)
//      depending on b2 calls : create_surface(triang, false, false)
// 4. reset_cocone_flag(triang)
// 5. mark_isolated_points_and_reset_bad_points(triang, bool b)
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------
// identify_vertices_with_dirty_umbrella
// -------------------------------------
// Marking the points with no clean umbrella as bad points. Their neighbors on the surface are
// also marked as the bad points.
// -----------------------------------------------------------------------
step_task identify_vertices_with_dirty_umbrella(Triangulation &triang) {
  // Resets the flag needed for the subroutine.
  for (FVI vit = triang.finite_vertices_begin(); vit != triang.finite_vertices_end(); vit++) {
    SK_UNIT;
    vit->visited = false;
    vit->bad = false;
    vit->bad_neighbor = false;
    vit->bad_v.clear();
  }
  // Iterate over the facets. Take one facet on the surface; for each of its vertices v (if
  // not visited and not bad), walk around v's umbrella on the surface. If an edge has other
  // than one next surface facet, mark both its endpoints bad. After coming back to the
  // starting edge mark the vertex visited.
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); fit++) {
    SK_UNIT;
    Cell_handle ch = (*fit).first;
    int id = (*fit).second;
    finite_side(triang, ch, id);
    if (!ch->cocone_flag(id))
      continue;
    // A vertex already visited whose umbrella did not include this facet is bad.
    for (int k = 1; k < 4; k++) {
      if (!ch->vertex((id + k) % 4)->visited || ch->vertex((id + k) % 4)->bad)
        continue;
      int v_id = ch->vertex((id + k) % 4)->id;
      int v_index1 = -1;
      for (int j = 1; j < 4; j++)
        if (ch->vertex((id + j) % 4)->id == v_id)
          v_index1 = j;
      if (ch->umbrella_member[id][v_index1] != v_id)
        ch->vertex((id + k) % 4)->bad = true;
    }
    // Now search for the umbrella of each vertex (if it's not bad) taking the edge as the
    // starting point.
    for (int i = 1; i < 4; i++) {
      Vertex_handle v = ch->vertex((id + i) % 4);
      if (v->visited || v->bad)
        continue;
      v->visited = true;
      // i1 is the pivot around which I am searching for umbrella
      int i1 = (id + i) % 4;
      int i2 = (i == 3) ? (id + 1) % 4 : (id + i + 1) % 4;
      int start_id = ch->vertex(i2)->id;
      Cell_handle c = ch;
      int fid = id;
      bool found = true;
      while (found) {
        SK_UNIT;
        Cell_handle next_c = c;
        int next_id = fid;
        found = false;
        Facet_circulator begin = triang.incident_facets(Edge(c, i1, i2));
        Facet_circulator fcirc = begin;
        int k = 0; // k counts the facets on the surface around the edge
        do {
          Cell_handle cc = (*fcirc).first;
          int cid = (*fcirc).second;
          if (triang.is_infinite(cc->vertex((cid + 1) % 4)) ||
              triang.is_infinite(cc->vertex((cid + 2) % 4)) ||
              triang.is_infinite(cc->vertex((cid + 3) % 4))) {
            fcirc++;
            continue;
          }
          if (cc->cocone_flag(cid)) {
            finite_side(triang, cc, cid);
            // put the vertex id in the umbrella_member array of the cell cc and its neighbor
            int v_index1 = -1, v_index2 = -1;
            for (int j = 1; j < 4; j++) {
              if (cc->vertex((cid + j) % 4)->id == v->id)
                v_index1 = j;
              if (cc->neighbor(cid)->vertex((cc->neighbor(cid)->index(cc) + j) % 4)->id == v->id)
                v_index2 = j;
            }
            cc->umbrella_member[cid][v_index1] = v->id;
            cc->neighbor(cid)->umbrella_member[cc->neighbor(cid)->index(cc)][v_index2] = v->id;
            if ((cc->id == c->id && cid == fid) ||
                (cc->neighbor(cid)->id == c->id && cc->neighbor(cid)->index(cc) == fid)) {
              if ((cc->id != c->id || cid != fid) && (triang.is_infinite(cc->neighbor(cid))))
                ;
              else {
                fcirc++;
                continue;
              }
            }
            next_c = cc;
            next_id = cid;
            k++;
          }
          fcirc++;
        } while (fcirc != begin);
        if (k != 1) {
          c->vertex(i1)->bad = true;
          c->vertex(i2)->bad = true;
          break;
        }
        // get the next edge .. that is find the next cell and i1 and i2
        int ni1 = -1, ni2 = -1;
        for (int j = 1; j < 4; j++) {
          Vertex_handle nv = next_c->vertex((next_id + j) % 4);
          if (nv->id == c->vertex(i2)->id)
            continue;
          if (nv->id == v->id)
            ni1 = (next_id + j) % 4;
          else
            ni2 = (next_id + j) % 4;
        }
        c = next_c;
        fid = next_id;
        i1 = ni1;
        i2 = ni2;
        found = c->vertex(i2)->id != start_id;
      }
    }
  }
}

// ------------------------------------------------------------------------------
// check_sharp_facets
// --------------------
// Checks if some good points have edges with sharp cocone facets incident to them. These
// places create ambiguity in in-out marking, and mobius-strip-like behaviour is dangerous
// because a wrong in-out marking can destroy the whole surface.
// ------------------------------------------------------------------------------
step_task check_sharp_facets(Triangulation &triang) {
  for (FEI eit = triang.finite_edges_begin(); eit != triang.finite_edges_end(); eit++) {
    SK_UNIT;
    Cell_handle c = (*eit).first;
    Vertex_handle v = c->vertex((*eit).second);
    Vertex_handle w = c->vertex((*eit).third);
    if (v->bad && w->bad)
      continue;
    // this edge has a good point
    int cnt = 0;
    double min_cos = extreme_facet_cosine(triang, triang.incident_facets(*eit), v, w, false, cnt);
    if (cnt == 2) {
      if (min_cos <= -0.9) {
        v->bad = true;
        w->bad = true;
        continue;
      }
      if (min_cos < 0) {
        if (!v->is_flat())
          v->bad = true;
        if (!w->is_flat())
          w->bad = true;
      }
    }
  }
}

// -------------------------------------------------------------------------
// mark_bad_points
// ---------------
// The bad points: those without a clean umbrella (not homeomorphic to a disk: boundary,
// pinching, or an edge with more than two facets), and those on an edge with a sharp pair of
// facets.
// -------------------------------------------------------------------------
step_task mark_bad_points(Triangulation &triang) {
  SK_RUN(identify_vertices_with_dirty_umbrella(triang));
  SK_RUN(check_sharp_facets(triang));
}

typedef cg::Triple<Cell_handle, int, int> Pivot_vertex;

// -----------------------------------------------------------------------
// propagate_in_out_marking
// ------------------------
// Given a set of vertices and an initial marking of one of the cells the marking is propagated
// through whatever vertices can be reached from these initial vertices on the surface.
// -----------------------------------------------------------------------
step_task propagate_in_out_marking(Triangulation &triang, vector<Pivot_vertex> &v_stack) {
  while (!v_stack.empty()) {
    SK_UNIT;
    Cell_handle cell = v_stack.back().first;
    int fid = v_stack.back().second;
    int vid = v_stack.back().third;
    v_stack.pop_back();
    // Now move around the outside umbrella of vertex until it hits a cocone facet.
    vector<Pivot_vertex> f_stack;
    f_stack.push_back(Pivot_vertex(cell, fid, vid));
    vector<int> vis_id;
    vis_id.push_back(cell->id);
    while (!f_stack.empty()) {
      SK_UNIT;
      Cell_handle old_cell = f_stack.back().first;
      int old_fid = f_stack.back().second;
      int old_vid = f_stack.back().third;
      Vertex_handle v = old_cell->vertex(old_vid);
      f_stack.pop_back();
      Cell_handle new_cell = old_cell->neighbor(old_fid);
      int new_fid = new_cell->index(old_cell);
      int new_vid = -1;
      triang.has_vertex(new_cell, new_fid, v, new_vid);
      if (old_cell->cocone_flag(old_fid)) {
        // don't insert any more facet in f_stack, but try to get new vertices for v_stack: the
        // ones of the cocone facet.
        for (int i = 1; i < 4; i++) {
          int j = (i + old_fid) % 4;
          // The new vertex is different from the current vertex and it has to be good and it is
          // not visited yet.
          if (j == old_vid || old_cell->vertex(j)->bad || old_cell->vertex(j)->visited)
            continue;
          int iv = -1, iw = -1;
          vertex_indices(old_fid, j, iv, iw);
          v_stack.push_back(Pivot_vertex(old_cell, iv, j));
          v_stack.push_back(Pivot_vertex(old_cell, iw, j));
          old_cell->vertex(j)->visited = true;
        }
      } else {
        // Insert the new facets. These facets have to be different than the old one. And the
        // new cell captured has to be unvisited.
        int iv = -1, iw = -1;
        vertex_indices(new_fid, new_vid, iv, iw);
        bool f = false;
        for (int vi : vis_id)
          if (vi == new_cell->id)
            f = true;
        if (!f) {
          bool f1 = false, f2 = false;
          for (int vi : vis_id) {
            if (vi == new_cell->neighbor(iv)->id)
              f1 = true;
            if (vi == new_cell->neighbor(iw)->id)
              f2 = true;
          }
          if (!f1)
            f_stack.push_back(Pivot_vertex(new_cell, iv, new_vid));
          if (!f2)
            f_stack.push_back(Pivot_vertex(new_cell, iw, new_vid));
          vis_id.push_back(new_cell->id);
          new_cell->outside = true;
        }
      }
    }
  }
}

// The cell's facet of least circumradius, marked opaque on both sides (mark_in_out and
// create_surface's transparent cells).
void mark_opaque_facet(Cell_handle cit) {
  double min_r = DBL_MAX;
  int min = -1;
  for (int i = 0; i < 4; i++) {
    Point p = cit->vertex((i + 1) % 4)->point();
    Point q = cit->vertex((i + 2) % 4)->point();
    Point r = cit->vertex((i + 3) % 4)->point();
    double radius = sq_cr_tr_3(p, q, r);
    if (radius < min_r) {
      min = i;
      min_r = radius;
    }
  }
  if (min < 0)
    return; // every radius NaN: no opaque facet
  cit->opaque[min] = true;
  cit->neighbor(min)->opaque[cit->neighbor(min)->index(cit)] = true;
}

// -------------------------------------------------------------------------
// mark_in_out
// -----------
// The points having a clean (and preferably non-sharp) umbrella have their incident tetrahedra
// divided into two classes, inside and outside, marked consistently. Infinite tetrahedra, which
// are definitely outside, are the starting points.
// -------------------------------------------------------------------------
step_task mark_in_out(Triangulation &triang) {
  for (ACI cit = triang.all_cells_begin(); cit != triang.all_cells_end(); cit++) {
    SK_UNIT;
    // all the infinite cells are outside
    if (triang.is_infinite(cit)) {
      cit->outside = true;
      continue;
    }
    // A finite cell whose four points are all bad or isolated is transparent. All the vertices
    // are marked unvisited.
    bool mark_transp_flag = true;
    for (int i = 0; i < 4; i++) {
      cit->vertex(i)->visited = false;
      if (!cit->vertex(i)->bad && !cit->vertex(i)->is_isolated())
        mark_transp_flag = false;
    }
    cit->transp = mark_transp_flag;
    if (!mark_transp_flag)
      continue;
    mark_opaque_facet(cit);
  }
  // Start marking
  for (ACI cit = triang.all_cells_begin(); cit != triang.all_cells_end(); cit++) {
    SK_UNIT;
    bool start = false;
    vector<Pivot_vertex> v_stack;
    if (!triang.is_infinite(cit))
      continue;
    Cell_handle start_cell = cit;
    for (int i = 0; i < 4; i++) {
      // every marking has to start from an infinite cell
      if (!triang.is_infinite(start_cell->vertex(i)))
        continue;
      // push the good vertices (along with three facets incident) in a stack
      for (int j = 1; j < 4; j++) {
        int v_id = (i + j) % 4;
        if (start_cell->vertex(v_id)->bad || start_cell->vertex(v_id)->is_isolated() ||
            start_cell->vertex(v_id)->visited)
          continue;
        v_stack.push_back(Pivot_vertex(start_cell, (v_id + 1) % 4, v_id));
        v_stack.push_back(Pivot_vertex(start_cell, (v_id + 2) % 4, v_id));
        v_stack.push_back(Pivot_vertex(start_cell, (v_id + 3) % 4, v_id));
        start_cell->vertex(v_id)->visited = true;
        start = true;
      }
      break;
    }
    if (!start)
      continue;
    SK_RUN(propagate_in_out_marking(triang, v_stack));
  }
}

// -----------------------------------------------------------------------
// grow_boundary
// -------------
// Given a starting boundary, it grows the boundary within the outside tetrahedra.
// -----------------------------------------------------------------------
step_task grow_boundary(Triangulation &triang, vector<Facet> &bdy_stack,
                        bool postponne_walk_to_do_io_marking) {
  while (!bdy_stack.empty()) {
    SK_UNIT;
    Cell_handle c = bdy_stack.back().first;
    int id = bdy_stack.back().second;
    bdy_stack.pop_back();
    if (!c->bdy[id])
      continue;
    // If the neighboring cell is not outside then if the facet is not opaque cross the facet
    // otherwise don't cross.
    if (!c->neighbor(id)->outside) {
      if (!c->neighbor(id)->transp)
        continue;
      if (c->opaque[id])
        continue;
    }
    Cell_handle ch_n = c->neighbor(id);
    int id_n = ch_n->index(c);
    c->bdy[id] = false;
    ch_n->bdy[id_n] = false;
    for (int i = 1; i < 4; i++) {
      int j = (i + id_n) % 4;
      if (ch_n->bdy[j]) {
        ch_n->bdy[j] = false;
        ch_n->neighbor(j)->bdy[ch_n->neighbor(j)->index(ch_n)] = false;
      } else {
        ch_n->bdy[j] = true;
        ch_n->neighbor(j)->bdy[ch_n->neighbor(j)->index(ch_n)] = true;
        if (!ch_n->neighbor(j)->visited)
          bdy_stack.push_back(Facet(ch_n, j));
      }
    }
    ch_n->visited = true;
    ch_n->outside = true; // needed for the transparent tetrahedra, not outside at the beginning
    if (!postponne_walk_to_do_io_marking)
      continue;
    // Postpone the walk to see if an unvisited good (not isolated) point has been reached, so
    // that its in-out marking can be done now.
    if (!ch_n->vertex(id_n)->is_isolated() && !ch_n->vertex(id_n)->bad &&
        !ch_n->vertex(id_n)->visited) {
      vector<Pivot_vertex> v_stack;
      v_stack.push_back(Pivot_vertex(ch_n, (id_n + 1) % 4, id_n));
      v_stack.push_back(Pivot_vertex(ch_n, (id_n + 2) % 4, id_n));
      v_stack.push_back(Pivot_vertex(ch_n, (id_n + 3) % 4, id_n));
      ch_n->vertex(id_n)->visited = true;
      SK_RUN(propagate_in_out_marking(triang, v_stack));
    }
  }
}

// -----------------------------------------------------------------------
// create_surface
// --------------
// Given a set of inside and outside tetrahedra this procedure grows a boundary within the
// outside tetrahedra and separates the outside from inside. This boundary is the surface.
// -----------------------------------------------------------------------
step_task create_surface(Triangulation &triang, bool postponne_walk_to_do_io_marking,
                         bool do_second_phase_of_walk) {
  // before starting the walk among the outside tetrahedra set/reset the required flags.
  for (ACI cit = triang.all_cells_begin(); cit != triang.all_cells_end(); cit++) {
    SK_UNIT;
    cit->visited = false;
    for (int i = 0; i < 4; i++) {
      cit->vertex(i)->tag = false;
      cit->bdy[i] = false;
    }
    // the infinite tetrahedra are always outside.
    if (triang.is_infinite(cit) && !cit->outside)
      cit->outside = true;
  }
  // Start growing the boundary. The stack holds the facets to be processed to grow the
  // boundary.
  vector<Facet> bdy_stack;
  for (ACI cit = triang.all_cells_begin(); cit != triang.all_cells_end(); cit++) {
    SK_UNIT;
    bool start = false;
    // Walk has to start from an unvisited infinite cell, which is guaranteed to be outside.
    if (!triang.is_infinite(cit) || cit->visited)
      continue;
    for (int i = 0; i < 4; i++) {
      if (!triang.is_infinite(cit->vertex(i)))
        continue;
      for (int j = 1; j < 4; j++) {
        // If any vertex is bad or isolated take a new cell .. a little conservative.
        if (cit->vertex((i + j) % 4)->bad || cit->vertex((i + j) % 4)->is_isolated()) {
          start = false;
          break;
        }
        start = true;
      }
      break;
    }
    if (!start)
      continue;
    // Got a fresh boundary to grow: put the facets in the bdy_stack.
    Cell_handle start_cell = cit;
    start_cell->visited = true;
    for (int i = 0; i < 4; i++) {
      start_cell->bdy[i] = true;
      start_cell->neighbor(i)->bdy[start_cell->neighbor(i)->index(start_cell)] = true;
      bdy_stack.push_back(Facet(start_cell, i));
    }
    SK_RUN(grow_boundary(triang, bdy_stack, postponne_walk_to_do_io_marking));
  }
  if (!do_second_phase_of_walk)
    co_return;
  // Some vertices (good, not isolated) are still unvisited. For them some blobs show up outside
  // the surface. To get rid of them we need to make another walk with the cells with one or
  // more good, unvisited, non-isolated points and rest bad/isolated points transparent.
  bool walk_flag = false;
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); cit++) {
    SK_UNIT;
    for (int i = 0; i < 4; i++) {
      cit->opaque[i] = false;
      if (!cit->vertex(i)->bad && !cit->vertex(i)->is_isolated() && !cit->vertex(i)->visited) {
        cit->vertex(i)->bad = true;
        walk_flag = true;
      }
    }
  }
  if (!walk_flag)
    co_return;
  // phase two of walk: any cell having all four bad/isolated is made transparent, and its
  // opaque facet chosen.
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); cit++) {
    SK_UNIT;
    bool mark_transp_flag = true;
    for (int i = 0; i < 4; i++)
      if (!cit->vertex(i)->bad && !cit->vertex(i)->is_isolated())
        mark_transp_flag = false;
    cit->transp = mark_transp_flag;
    if (!mark_transp_flag)
      continue;
    mark_opaque_facet(cit);
  }
  // again, without postponing the walk and without a second phase.
  SK_RUN(create_surface(triang, false, false));
}

// ------------------------------------------------------------------------
// reset_cocone_flag
// ------------------
// Resets the cocone flag after doing the inside-outside marking
// ------------------------------------------------------------------------
step_task reset_cocone_flag(Triangulation &triang) {
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); ++fit) {
    SK_UNIT;
    Cell_handle ch = (*fit).first;
    int id = (*fit).second;
    ch->set_cocone_flag(id, false);
    ch->neighbor(id)->set_cocone_flag(ch->neighbor(id)->index(ch), false);
  }
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); ++fit) {
    SK_UNIT;
    Cell_handle ch = (*fit).first;
    int id = (*fit).second;
    if (ch->bdy[id]) {
      ch->set_cocone_flag(id, true);
      ch->neighbor(id)->set_cocone_flag(ch->neighbor(id)->index(ch), true);
    }
  }
}

// -----------------------------------------------------------------------
// mark_isolated_points_and_reset_bad_points
// -----------------------------------------
// Marking the isolated points. Resets the bad-point flag if the boolean parameter is true.
// -----------------------------------------------------------------------
step_task mark_isolated_points_and_reset_bad_points(Triangulation &triang,
                                                    bool reset_bad_point_flag) {
  for (FVI vit = triang.finite_vertices_begin(); vit != triang.finite_vertices_end(); vit++) {
    SK_UNIT;
    vit->set_isolated(true);
    if (reset_bad_point_flag)
      vit->bad = false;
  }
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); fit++) {
    SK_UNIT;
    Cell_handle ch = (*fit).first;
    int id = (*fit).second;
    if (!ch->cocone_flag(id))
      continue;
    for (int i = 1; i < 4; i++)
      ch->vertex((id + i) % 4)->set_isolated(false);
  }
}

// -----------------------------------------------------------------------
// walk_wt
// --------
// Walking on watertight surface to get rid of the outside tetrahedron inside the surface: a
// confirmatory walk that ensures the consistency of the outside marking.
// -----------------------------------------------------------------------
step_task walk_wt(Triangulation &triang) {
  for (ACI cit = triang.all_cells_begin(); cit != triang.all_cells_end(); cit++) {
    SK_UNIT;
    cit->visited = false;
    cit->outside = false;
    if (triang.is_infinite(cit))
      cit->outside = true;
  }
  vector<Cell_handle> c_stack;
  for (ACI cit = triang.all_cells_begin(); cit != triang.all_cells_end(); cit++) {
    SK_UNIT;
    if (!triang.is_infinite(cit) || cit->visited)
      continue;
    cit->visited = true;
    for (int i = 0; i < 4; i++) {
      if (cit->cocone_flag(i) || cit->neighbor(i)->visited)
        continue;
      c_stack.push_back(cit->neighbor(i));
      cit->neighbor(i)->outside = true;
      cit->neighbor(i)->visited = true;
    }
    while (!c_stack.empty()) {
      SK_UNIT;
      Cell_handle ch = c_stack.back();
      c_stack.pop_back();
      for (int i = 0; i < 4; i++) {
        Cell_handle c = ch->neighbor(i);
        if (ch->cocone_flag(i) || c->visited)
          continue;
        c_stack.push_back(c);
        c->outside = true;
        c->visited = true;
      }
    }
  }
}

} // namespace

// ------------------------------------------------------------------
// tcocone
// ------------
// Takes a triangulation of a point set sampled from a surface and reconstructs a watertight
// surface out of it. As a first step it uses cocone to get a surface with its boundary
// identified. Then it performs two operations called marking and peeling.
// ------------------------------------------------------------------
step_task tcocone(const double DEFAULT_ANGLE, const double DEFAULT_SHARP, const double DEFAULT_FLAT,
                  const double DEFAULT_RATIO, Triangulation &triang) {
  // Initial mesh generation (with boundary) using cocone.
  SK_RUN(cocone(DEFAULT_ANGLE, DEFAULT_SHARP, DEFAULT_FLAT, DEFAULT_RATIO, triang));
  // Mark the isolated points in the initial mesh; the points without a proper umbrella or on
  // edges with sharp facet pairs are bad.
  SK_RUN(mark_isolated_points_and_reset_bad_points(triang, true));
  SK_RUN(mark_bad_points(triang));
  // Marking (step 1 of Tight Cocone): the inside and outside of every good point, consistently.
  SK_RUN(mark_in_out(triang));
  // Peeling (step 2 of Tight Cocone): the inside and outside of the surface.
  SK_RUN(create_surface(triang, true, true));
  // Conform the surface to the inside/outside marking.
  SK_RUN(reset_cocone_flag(triang));
  SK_RUN(mark_isolated_points_and_reset_bad_points(triang, true));
  // Walk on the watertight surface to confirm the marking.
  SK_RUN(walk_wt(triang));
}

} // namespace cvc::cocone
