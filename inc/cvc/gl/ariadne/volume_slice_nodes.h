/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// volume_slice_nodes.h -- the `volume_slice` scene node type: one axis-aligned plane of a volume,
// a textured quad coloured through the node's transfer function (cvc::gl::VolumeSliceNode;
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §6 item 1, docs/ARIADNE.md §5.4).
//
//   - node: ct_z
//     type: volume_slice
//     source: { file: head.rawiv }       # or { sdf: { mesh: stanford.bunny, dim: 64 } }
//     fit: { height: 100, up: y }        # as a volren node's (an SDF source)
//     axis: z                            # x | y | z (or a view's name: xy, xz, zy)
//     index: 40                          # the plane along the axis; -1 (the default) the middle
//     filter: nearest                    # nearest (the default) | linear
//     opaque: true                       # false takes the opacity from the transfer function
//     overlay_offset: 2                  # how far back the plane draws, under its overlays
//     region_reads: auto                 # auto | true | false: read a file plane by plane
//     transfer_function: { preset: grayscale }   # the §10.1 model form; grayscale by default
//     children: [ ... ]                  # overlays: drawn on top of the plane
//   - node: ct_x
//     type: volume_slice
//     volume_of: ct_z                    # another plane of ct_z's volume, through its
//     axis: x                            #   transfer function (no source: of its own)
//
// The props seed the node's keys (<node>.slice.*, VolumeSliceNode.h); the keys are live after
// that, so a slider bound to <node>.slice.index steps through the volume. `region_reads: auto`
// reads a file of more than 64 Mi voxels one plane at a time (cvc::slice_source), keeping the
// last eight; smaller files and SDFs are read whole. <node>.transfer_function is bound
// (tf_binding.h, target volume_slice), so a tf_editor colours the plane and shows its histogram
// (for a file read plane by plane, the histogram of the first plane shown, over the file's range).
// `transfer_function:` takes `preset:` and the model's own keys (color, alpha, isocontours,
// domain, opacity_cubed, resolution), each replacing the preset's. A node with `volume_of: <id>`
// shares the volume of the volume_slice node <id> (realized before it) and colours through that
// node's <id>.transfer_function, so one editor drives every plane: the classic orthoslicer is
// three such nodes over one volume, read or computed once.

#ifndef CVC_GL_ARIADNE_VOLUME_SLICE_NODES_H
#define CVC_GL_ARIADNE_VOLUME_SLICE_NODES_H

#include <cvc/tf/model.h>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {
struct SceneNode;
}
namespace gl {
namespace ariadne {

// Register the "volume_slice" scene node type on `app`. Idempotent; call before realize_scene
// (register_cvcgl_extensions does).
void register_volume_slice_node_type(cvc::app &app);

// A node's `transfer_function:` prop as a model: `preset:` (grayscale when absent), then each of
// the model's own keys it gives (color, alpha, isocontours, domain, opacity_cubed, resolution)
// replacing the preset's. Problems are appended to `warnings`, naming the node. (volume_slice's and
// volume_grid's.)
cvc::tf::model transfer_function_prop(const cvc::ariadne::SceneNode &n,
                                      std::vector<std::string> *warnings);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_VOLUME_SLICE_NODES_H
