/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Ariadne's `type: volume_render` (cvc::gl::VolumeRenderNode, cvc/gl/VolumeRenderNode.h): one
// volume the app holds on the state tree, drawn by the renderer <node>.renderer picks -- VTK's GPU
// mapper (vtk), libcvc's raycaster (volren) or view-aligned texture slices (volslice) -- and
// switchable while the document runs, through one transfer function.
//
//   data:
//     head: { generate: sdf, params: { mesh: stanford.bunny, dim: 64 } }
//   scene:
//     nodes:
//       - node: head
//         type: volume_render
//         source: { data: head }        # or { state: <key> }, or { node: <id>, key: <key> }
//         renderer: volren              # vtk (the default), volren, volslice or none
//         transfer_function: { preset: grayscale }
//
// The node reads no file. source: names the state key whose data channel holds the volume: a
// data: entry's output (item: volume by default), a key a program or a filter writes, or another
// node's published volume ({ node: <id> } follows its `volume`, or its `grid.volume` while that
// is the one holding data; key: names another). It is resolved into <node>.source.volume
// (relative to the document's scope; a leading '/' is the root's), and editing that key follows
// the new one. The volume is followed as it changes and drawn as it is held: the renderers share
// its voxels.
//
// Keys under the node: renderer (seeded from renderer:), status (what draws, or what the node
// waits for), source.volume, source.status (a key that holds no volume); on the data channel,
// volume. <node>.transfer_function is the model a tf_editor binds: each renderer the node makes is
// bound to it, so an edit carries over a switch. Each renderer's own settings are under
// <node>.children.<renderer> (vtk, volren, volslice), where volren_controls and
// volslice_controls bind.
#ifndef CVC_GL_ARIADNE_VOLUME_RENDER_NODES_H
#define CVC_GL_ARIADNE_VOLUME_RENDER_NODES_H

namespace cvc {
class app;
namespace gl {
namespace ariadne {

// Register `volume_render` on `app` (register_cvcgl_extensions does).
void register_volume_render_node_type(cvc::app &app);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_VOLUME_RENDER_NODES_H
