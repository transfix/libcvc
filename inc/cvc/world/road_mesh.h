/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
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

// road_mesh.h — turn the road GRAPH into a drawable 3D road SURFACE.
//
// cvc::world::roads gives a graph (width-tagged segments + junction nodes); this extrudes it into a
// real surface mesh the renderers can draw instead of (or alongside) a flat texture bake. Each run
// of segments between junctions (and dead ends, and changes of width) is one width-wide STRIP,
// its bends rounded by corner cutting and mitred, so a winding road is one smooth band with no gap
// or overlap at a bend, laid on the terrain by sampling the height field at each vertex. A round
// PAD sits at every junction and dead end, a hair above the strips it covers, so roads meeting at
// an angle join cleanly. A single dark yellow CENTRE STRIPE runs down each road wide enough for
// one, stopping short of the junctions. Arterials are drawn a shade darker than local streets
// (per-vertex colour). Output is plain cvc::geometry (triangles + up normals + colour) — GL-free
// and reusable by the native lab and the wasm demos alike. Deterministic: a pure function of the
// network + params.

#ifndef CVC_WORLD_ROAD_MESH_H
#define CVC_WORLD_ROAD_MESH_H

#include <cvc/geometry/geometry.h>
#include <cvc/world/roads.h> // road_network, road_height_fn

namespace cvc {
namespace world {

struct road_mesh_params {
  double lift = 0.08;        // raise the surface this far above the sampled ground (metres), so it
                             // sits just over the terrain instead of z-fighting it
  bool intersections = true; // a round pad at every junction and dead end
  int pad_segments = 12;     // rim vertices of a junction pad (higher = rounder)
  double pad_lift = 0.02;    // a pad sits this far above the strips it covers
  int smooth_passes = 2;     // corner-cutting passes rounding each road's bends (0: the segments)
  double sample_step = 4.0;  // the ground is sampled at least this often along a road (0: only at
                             // its vertices), so a long straight follows the ground under it
  bool centerline = true;    // a centre stripe on each road at least this wide:
  double centerline_min_width = 4.5;             // (metres)
  double centerline_width = 0.15;                // the stripe's width
  double centerline_lift = 0.03;                 // above the road surface
  double centerline_rgb[3] = {0.62, 0.47, 0.10}; // dark yellow
  double arterial_rgb[3] = {0.24, 0.24, 0.26};   // asphalt colour for major roads
  double local_rgb[3] = {0.32, 0.32, 0.34};      // ... and for local streets (a shade lighter)
};

// Extrude the network into a road-surface mesh (see the file header). `height` places the surface
// on the terrain (empty => flat z = 0). Empty network => empty mesh.
cvc::geometry extrude_roads(const road_network &net, const road_mesh_params &p = {},
                            const road_height_fn &height = {});

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_ROAD_MESH_H
