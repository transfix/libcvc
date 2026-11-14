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

// medax.cpp and u1.cpp of VolumeRover's Skeletonization: the medial axis (the Voronoi facets
// chosen by the angle and ratio conditions), and U1, the unstable manifolds of the index-1
// saddles grown from the medial-axis facets that contain one.

#include "cocone.h"

namespace cvc::cocone {

namespace {

//-------------------------------------------------------------------
// tang_umbrella
//-------------
// Choose the dual triangles of Voronoi edges which are intersected by the tangent polygon
// (refer to the medial paper).
//------------------------------------------------------------------
step_task tang_umbrella(const Triangulation &triang) {
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
    // Compute the radius of the circumcircle of the triangle.
    double t = cg::to_double((ch->vertex((id + 1) % 4)->point() - ch->voronoi()) *
                             (ch->neighbor(id)->voronoi() - ch->voronoi())) /
               cg::to_double((ch->voronoi() - ch->neighbor(id)->voronoi()) *
                             (ch->voronoi() - ch->neighbor(id)->voronoi()));
    double cir_radius =
        cg::to_double((ch->voronoi() + t * (ch->neighbor(id)->voronoi() - ch->voronoi()) -
                       ch->vertex((id + 1) % 4)->point()) *
                      (ch->voronoi() + t * (ch->neighbor(id)->voronoi() - ch->voronoi()) -
                       ch->vertex((id + 1) % 4)->point()));
    // Iterate over the vertices incident to the facet.
    for (int i = 1; i < 4; ++i) {
      Vertex_handle vh = ch->vertex((id + i) % 4);
      double cos1 = cosine(ch->voronoi() - vh->point(), vh->normal());
      double cos2 = (triang.is_infinite(ch->neighbor(id)))
                        ? 1.0
                        : cosine(ch->neighbor(id)->voronoi() - vh->point(), vh->normal());
      if (cos1 > cos2)
        swap(cos1, cos2);
      if ((cos1 < 0.0) && (cos2 > 0.0)) {
        if (!vh->is_flat())
          continue;
        // Add the normals to the normal_stack
        Vector facet_normal = cg::cross_product(
            ch->vertex((id + 2) % 4)->point() - ch->vertex((id + 1) % 4)->point(),
            ch->vertex((id + 3) % 4)->point() - ch->vertex((id + 1) % 4)->point());
        vh->normal_stack.push_back(facet_normal);
        // Update diameter
        if (vh->diameter() < cir_radius)
          vh->set_diameter(cir_radius);
      }
    }
  }
}

//--------------------------------------------------------------------
// compute_axis
//------------------
// Mark the medial axis's Voronoi facets (inside and outside).
//---------------------------------------------------------------------
step_task compute_axis(Triangulation &triang, const double theta, const double ratio) {
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
    Vertex_handle u = (*eit).first->vertex((*eit).second);
    Vertex_handle v = (*eit).first->vertex((*eit).third);
    Facet_circulator fcirc = triang.incident_facets((*eit));
    Facet_circulator begin_fcirc = fcirc;
    do {
      Cell_handle c = (*fcirc).first;
      int fid = (*fcirc).second;
      // The legacy code found u_id and v_id inside CGAL_assertion()s, which its release builds
      // compiled out, leaving both -1 (and set_VF_on_medax writing out of bounds).
      int u_id = -1, v_id = -1;
      triang.has_vertex(c, fid, u, u_id);
      triang.has_vertex(c, fid, v, v_id);
      c->set_VF_on_medax(u_id, v_id, true);
      c->set_VV_on_medax(true);
      fcirc++;
    } while (fcirc != begin_fcirc);
  }
}

} // namespace

step_task compute_medial_axis(Triangulation &triang, const double theta,
                              const double medial_ratio) {
  SK_RUN(tang_umbrella(triang));
  SK_RUN(compute_axis(triang, theta, medial_ratio));
}

// ------------------------
// compute_u1
// ------------------------
// Identify the index-1 saddles: the points where a VF meets its dual DE. Compute their unstable
// manifolds.
// ------------------------
step_task compute_u1(Triangulation &triang, diagnostics &diag) {
  // reset the f_visited field to false in every cell.
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); fit++) {
    SK_UNIT;
    (*fit).first->f_visited[(*fit).second] = false;
    (*fit)
        .first->neighbor((*fit).second)
        ->f_visited[(*fit).first->neighbor((*fit).second)->index((*fit).first)] = false;
  }
  vector<Facet> dual_VE_list;
  for (FEI eit = triang.finite_edges_begin(); eit != triang.finite_edges_end(); eit++) {
    SK_UNIT;
    Cell_handle c = (*eit).first;
    int uid = (*eit).second, vid = (*eit).third;
    // if VF is crossing the surface, continue.
    if (is_surf_VF(triang, c, uid, vid))
      continue;
    // if infinite VF continue.
    if (is_inf_VF(triang, c, uid, vid))
      continue;
    // if not on medial axis, continue.
    if (!c->VF_on_medax(uid, vid))
      continue;
    // if it's not i1saddle continue.
    if (!is_i1_saddle((*eit), triang, diag))
      continue;
    // collect its VEs into dual_VE_list. markings: 1. VF contains an i1 saddle. 2. VF is on
    // um(i1).
    Facet_circulator fcirc = triang.incident_facets((*eit));
    Facet_circulator begin = fcirc;
    do {
      Cell_handle cur_c = (*fcirc).first;
      int cur_fid = (*fcirc).second;
      int cur_uid = -1, cur_vid = -1, cur_wid = -1;
      // the edge (c, uid, vid) as (cur_c, cur_uid, cur_vid).
      for (int i = 1; i < 4; i++)
        if (cur_c->vertex((cur_fid + i) % 4)->id != c->vertex(uid)->id &&
            cur_c->vertex((cur_fid + i) % 4)->id != c->vertex(vid)->id) {
          cur_wid = (cur_fid + i) % 4;
          vertex_indices(cur_fid, cur_wid, cur_uid, cur_vid);
          break;
        }
      // markings are done first.
      cur_c->set_i1_saddle(cur_uid, cur_vid, true);
      cur_c->set_VF_on_um_i1(cur_uid, cur_vid, true);
      // if the current VE (= dual(fcirc)) is already visited continue.
      if (cur_c->f_visited[cur_fid]) {
        fcirc++;
        continue;
      }
      // if it's not transversal, continue.
      if (!is_transversal_flow((*fcirc))) {
        fcirc++;
        continue;
      }
      dual_VE_list.push_back(*fcirc);
      cur_c->f_visited[cur_fid] = true;
      cur_c->neighbor(cur_fid)->f_visited[cur_c->neighbor(cur_fid)->index(cur_c)] = true;
      fcirc++;
    } while (fcirc != begin);
  }
  // at this stage we have a list of VFs (which contain i1 saddles), which is what um(i1)
  // currently is. Through its transversal edges um(i1) grows.
  while (!dual_VE_list.empty()) {
    SK_UNIT;
    Facet f = dual_VE_list.back();
    dual_VE_list.pop_back();
    Cell_handle c = f.first;
    int fid = f.second;
    // convention: uid,vid correspond to the ACCEPTOR DE; uid,wid and vid,wid to the DONOR DE,
    // so <u,w,v > 90 degrees. The legacy code called find_acceptor inside a CGAL_assertion(),
    // which its release builds compiled out, leaving the indices -1.
    int uid = -1, vid = -1, wid = -1;
    if (!find_acceptor(c, fid, uid, vid, wid))
      continue;
    // include this acceptor VF into um(i1), unless it is infinite or crosses the surface.
    if (is_surf_VF(triang, c, uid, vid) || is_inf_VF(triang, c, uid, vid))
      continue;
    mark_VF_on_u1(triang, c, uid, vid);
    // push its unvisited transversal VEs into dual_VE_list.
    Facet_circulator fcirc = triang.incident_facets(Edge(c, uid, vid));
    Facet_circulator begin = fcirc;
    do {
      Cell_handle c0 = (*fcirc).first;
      int fid0 = (*fcirc).second;
      if (c0->f_visited[fid0]) {
        fcirc++;
        continue;
      }
      Cell_handle c1 = c0->neighbor(fid0);
      int fid1 = c1->index(c0);
      if (!is_transversal_flow(Facet(c0, fid0))) {
        fcirc++;
        continue;
      }
      dual_VE_list.push_back((*fcirc));
      c0->f_visited[fid0] = true;
      c1->f_visited[fid1] = true;
      fcirc++;
    } while (fcirc != begin);
  }
}

} // namespace cvc::cocone
