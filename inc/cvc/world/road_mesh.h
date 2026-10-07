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
// real surface mesh the renderers can draw instead of (or alongside) a flat texture bake: a
// width-wide quad RIBBON per segment, laid on the terrain by sampling the height field at each
// corner, plus a rounded PAD polygon at every junction so roads meeting at an angle join cleanly
// with no gap or overlap seam. Arterials are drawn a shade darker than local streets (per-vertex
// colour). Output is plain cvc::geometry (triangles + up normals + colour) — GL-free and reusable
// by the native lab and the wasm demos alike. Deterministic: a pure function of the network +
// params.

#ifndef CVC_WORLD_ROAD_MESH_H
#define CVC_WORLD_ROAD_MESH_H

#include <cvc/geometry/geometry.h>
#include <cvc/world/roads.h> // road_network, road_height_fn

namespace cvc {
namespace world {

struct road_mesh_params {
  double lift = 0.08;        // raise the surface this far above the sampled ground (metres), so it
                             // sits just over the terrain instead of z-fighting it
  bool intersections = true; // emit a rounded pad at every junction (degree >= 2) node
  int pad_segments = 12;     // rim vertices of a junction pad (higher = rounder)
  double arterial_rgb[3] = {0.24, 0.24, 0.26}; // asphalt colour for major roads
  double local_rgb[3] = {0.32, 0.32, 0.34};    // ... and for local streets (a shade lighter)
};

// Extrude the network into a road-surface mesh (see the file header). `height` places the surface
// on the terrain (empty => flat z = 0). Empty network => empty mesh.
cvc::geometry extrude_roads(const road_network &net, const road_mesh_params &p = {},
                            const road_height_fn &height = {});

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_ROAD_MESH_H
