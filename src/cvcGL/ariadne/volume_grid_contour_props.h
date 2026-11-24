/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The `volume_grid` node type's contour props (volume_grid_nodes.cpp's realize calls this; the keys
// are VolumeGridContours'):
//
//   - node: grid
//     type: volume_grid
//     tool: contours
//     objects:                       # the object rows, while there are none
//       - { name: axon, color: [0.2, 0.8, 0.3], interpolation: spline, samples: 6 }
//     contours: { data: traced }     # contours the app loads (a data: entry; item: contours by
//                                    #   default) or { state: <key> }: grid.contour.source
//     contour: { point_size: 0.8, auto_tile: true }   # grid.contour.* (its table's keys)
//     tile: { curation_delta: 0.01 } # grid.tile.* (tile_params' keys)

#ifndef CVC_GL_ARIADNE_VOLUME_GRID_CONTOUR_PROPS_H
#define CVC_GL_ARIADNE_VOLUME_GRID_CONTOUR_PROPS_H

#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {
struct SceneNode;
}
namespace gl {
class VolumeGridNode;
namespace ariadne {

// Seed the grid's contour keys from the node's props (a key that has a value keeps it: an init:
// program or a restored session wins). Returns the store path of the dataset `contours:` names
// ("" for none), whose loading the scene waits for.
std::string seed_volume_grid_contour_props(cvc::app &app, const std::string &prefix,
                                           VolumeGridNode &g, const cvc::ariadne::SceneNode &n,
                                           std::vector<std::string> *warnings);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_VOLUME_GRID_CONTOUR_PROPS_H
