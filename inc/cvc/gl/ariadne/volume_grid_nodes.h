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
//   data:                                # the app loads the data: the node reads no file
//     head: head.rawiv                   # a volume (as: volume, by its extension)
//     seeds: seeds.vgr                   # point classes (as: point_classes)
//     bunny: { generate: sdf, params: { mesh: stanford.bunny, dim: 64 } }   # cvcGL's generator
//
//   - node: grid                         # a scene node
//     type: volume_grid
//     source: { data: head }             # the volume: a dataset's (item: volume, the default),
//                                        # { state: <key> } (a program's, a filter's) or
//                                        # { node: <id>, key: <key> } (another node's grid.volume)
//     var: 0                             # the variable and timestep the volume holds
//     time: 0
//     view: xy                           # the switch: xy | xz | zy
//     tool: points                       # navigate | points | probe
//     slice: { x: -1, y: -1, z: 40 }     # the planes (-1 the middle)
//     display: { point_scale: 1, show_points: true, crosshairs: true, grey: false }
//     classes:                           # the point classes it starts with (no points)
//       - { name: membrane, color: [1, 0.2, 0.2], radius: 2 }
//       - { name: background, color: [0.2, 0.4, 1] }
//     points: { data: seeds }            # point classes: a dataset's (item: classes) or
//                                        # { state: <key> }; they replace the classes each time
//                                        # they change
//     session: { data: last }            # a session (item: session) or { state: <key> }, applied
//                                        # each time it changes
//     transfer_function: { preset: grayscale }   # as a volume_slice node's
//
// The node views data the app holds on the state tree and follows them: the volume it cuts is
// the dataset's own (its voxels shared, not copied), so another node showing that dataset and
// this grid show one volume, and when it changes (the dataset loaded again, a program writing a
// new volume to the key) both redraw. The sources resolve to grid.source.volume, .points and
// .session (paths in the document's scope); editing one follows the new path. The app saves what
// the node publishes (grid.points, grid.session) with (data-save PATH FILE); the component's
// buttons load with (data-load ...) and point the source at what they loaded. A `source: { file
// }` or `{ sdf }` is not read: it warns, naming the data: entry to write instead.
//
// The props seed the node's keys (<node>.grid.*, VolumeGridNode.h); the keys are live after that.
// A prop seeds its key only while the key is empty, so a document reloaded over a live tree, or a
// restored session, keeps what the keys hold. The node's three planes are its children
// <id>.slice_xy, .slice_xz and .slice_zy (realized nodes "<id>_slice_xy" ... tagged with the
// node's id and their own name), so a viewport with `show: [<id>]` shows this grid alone and a
// slice view (mode: slice) the plane on its axis; "<id>_surfaces" (tags <id>, surfaces) is the
// group later phases draw surfaces into, which slice views hide. <node>.transfer_function is
// bound (tf_binding.h, target volume_slice), so a tf_editor colours all three planes and shows the
// volume's histogram (for a slice_source read plane by plane, the middle z plane's, over the
// file's range); it follows the volume as it changes. While a dataset the node follows is loading
// the scene has work pending (RealizedScene::pending_work), so a host that waits for a settled
// frame waits for the data.

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
