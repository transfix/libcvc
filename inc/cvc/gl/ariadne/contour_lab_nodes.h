/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// contour_lab_nodes.h -- the `contour_lab` scene node type: planar contours on a stack of slices,
// drawn as polylines, and the surfaces the contour tiler (cvc::tile_contours, cvc/geometry/tiler.h)
// builds through them (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §3 and §7, P4).
//
//   - node: dendrites
//     type: contour_lab
//     source: { file: series.ser }       # any contour format (.ser, .contour, .config, .cvcc)
//     # source: { node: grid }           # or another node's contour_stack (its tiler.contours
//     #                                  #   or grid.contours data key; `key:` names another): a
//     #                                  #   top-level node realized before this one, or a state
//     #                                  #   path (with a '.')
//     # source: { state: demo.stack }    # or the contour_stack on a state key's data channel
//     read: { first: 1, last: 40 }       # how a file is read (contour_read_params' fields)
//     tiler: { curation_delta: 0.01 }    # the tiler's parameters (tile_params' fields)
//     auto_tile: true                    # tile once the contours load (and after each load)
//     components:                        # per component: its colour, shown, tiled
//       ax010: { color: [0.2, 0.8, 0.3], visible: true, tile: true }
//     line_width: 2                      # the contours' polylines
//     opacity: 1                         # the surfaces'
//
// Keys, under <node>.tiler (the node's state, graphics.root.children.<id>.tiler):
//   request              "load", "tile" or "stop" (several separated by spaces run in turn): load
//                        reads the source.* keys again, tile starts the tiler as a sliced task on
//                        the app's scheduler (cvc/core/sliced_task.h), stop ends it at its next
//                        step, keeping the surfaces the node had
//   auto                 tile after every load (auto_tile:)
//   source.file, source.node, source.key, source.state
//                        the source, as the source: block gave it; a change applies at the next
//                        load. A node or state source is followed: new contours there load again
//   read.*               contour_read_params, for a file (param_table: unit / min / max / doc)
//   <tile_params field>  collinear_epsilon, curation_delta, remove_intersections,
//                        correspondence_overlap, include, exclude, z_first, z_last, cap_ends, color
//   components           how many there are; row i at components.<i>: name, color ("r,g,b" in
//                        [0, 1]), visible, tile (unticked: left out of the next tile), triangles
//   view.contours, view.tiles, view.opacity, view.line_width
//   info.*               slices, first, last, contours, vertices, components, z_spacing,
//                        surfaces, triangles
//   warnings             how many the last tile gave; warnings.<i> each one
//   label                the slice pair being tiled ("slices 3 - 4"; "" otherwise)
//   busy, stoppable, status, progress (0-100), phase, stats.work_ms, pid
//                        the job's (cvc/core/job_keys.h); phase names the slice pair while it
//                        tiles ("tiling slices 3 - 4 42%")
// Data channel: <node>.tiler.contours holds the stack (a std::shared_ptr<const contour_stack>, so
// another contour_lab, tiler.tile or a program reads it), and <node>.tiler.surfaces.<component>
// each surface (a cvc::geometry; a component's '.' and white space are '_' in the key).
//
// Children: <id>_contours (tag `contours`) holds a polyline node per component, and <id>_tiles
// (tag `tiles`) a surface node per component tiled, so a viewport's show: / hide: picks either.
//
// A file is read whole, in one step, when the scene is realized and at each load: a Reconstruct
// series of a few hundred sections reads in well under a second. Tiling is the long part, and it
// is resumable (§1.9).

#ifndef CVC_GL_ARIADNE_CONTOUR_LAB_NODES_H
#define CVC_GL_ARIADNE_CONTOUR_LAB_NODES_H

namespace cvc {
class app;
namespace gl {
namespace ariadne {

// Register the "contour_lab" scene node type on `app`. Idempotent; call before realize_scene
// (register_cvcgl_extensions does).
void register_contour_lab_node_type(cvc::app &app);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_CONTOUR_LAB_NODES_H
