/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// volume_grid_nodes.h -- the `volume_grid` scene node type: VolumeGridRover rebuilt as an Ariadne
// node (cvc::gl::VolumeGridNode; docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, P8,
// docs/ARIADNE.md §5.4). The volume_grid.ari component lays out its views and panel.
//
//   - node: grid
//     type: volume_grid
//     source: { file: head.rawiv }       # or { sdf: { mesh: stanford.bunny, dim: 64 } }
//     fit: { height: 100, up: y }        # as a volren node's (an SDF source)
//     var: 0                             # the file's variable and timestep
//     time: 0
//     region_reads: auto                 # auto | true | false: read a file plane by plane
//     view: xy                           # the switch: xy | xz | zy
//     tool: points                       # navigate | points | probe
//     slice: { x: -1, y: -1, z: 40 }     # the planes (-1 the middle)
//     display: { point_scale: 1, show_points: true, crosshairs: true, grey: false }
//     classes:                           # the point classes it starts with (no points)
//       - { name: membrane, color: [1, 0.2, 0.2], radius: 2 }
//       - { name: background, color: [0.2, 0.4, 1] }
//     points: seeds.vgr                  # classes and points read at the first tick (.vgr, .pts)
//     transfer_function: { preset: grayscale }   # as a volume_slice node's
//
// The props seed the node's keys (<node>.grid.*, VolumeGridNode.h); the keys are live after that.
// A prop seeds its key only while the key is empty, so a document reloaded over a live tree, or a
// restored session, keeps what the keys hold. The node's three planes are its children
// <id>.slice_xy, .slice_xz and .slice_zy (realized nodes "<id>_slice_xy" ... tagged with the
// node's id and their own name), so a viewport with `show: [<id>]` shows this grid alone and a
// slice view (mode: slice) the plane on its axis; "<id>_surfaces" (tags <id>, surfaces) is the
// group later phases draw surfaces into, which slice views hide. <node>.transfer_function is
// bound (tf_binding.h, target volume_slice), so a tf_editor colours all three planes and shows the
// volume's histogram (for a file read plane by plane, the middle z plane's, over the file's
// range); it follows a `load`.

#ifndef CVC_GL_ARIADNE_VOLUME_GRID_NODES_H
#define CVC_GL_ARIADNE_VOLUME_GRID_NODES_H

namespace cvc {
class app;
namespace gl {
namespace ariadne {

// Register the "volume_grid" scene node type on `app`. Idempotent; call before realize_scene
// (register_cvcgl_extensions does).
void register_volume_grid_node_type(cvc::app &app);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_VOLUME_GRID_NODES_H
