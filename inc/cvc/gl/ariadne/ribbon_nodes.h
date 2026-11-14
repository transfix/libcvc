/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_ARIADNE_RIBBON_NODES_H__
#define __CVC_GL_ARIADNE_RIBBON_NODES_H__

// Scene node types for lines drawn from data (docs/ARIADNE.md §5.4): `type: ribbon` over
// cvc::gl::RibbonNode and `type: draped_link` over cvc::gl::DrapedLinkNode, both streaming nodes,
// so data that grows or moves uploads only what changed.
//
//   - node: track
//     type: ribbon
//     source: { state: tracks.a }   # or { stream: <token> }, or { data: <name>, item: <output> }
//     fields: xyz                   # per point: x, y, z, and optionally w (width) and v (value)
//     width: 2                      # metres, the whole ribbon (a w field: each point's own)
//     color: c1                     # [r, g, b], "#rrggbb" or a style slot c0..c7
//     ramp: quality                 # with a v field: colour by value (quality, heat, or stops)
//     lift: 0.3                     # above the points, or above the ground
//     ground: terrain               # drape: each point's z from this heightfield node
//     window: { tail: 200, units: m }   # or { from, to }: what is drawn, moved with no upload
//
//   - node: link
//     type: draped_link
//     ground: terrain               # the heightfield it hugs
//     between: [car_a, car_b]       # two nodes, followed each frame (or from:/to:, or source:)
//     value: links.ab.quality       # a number or a state key, coloured through ramp:
//     ramp: quality
//
// A ribbon reads its source each frame it changed. Points it has already drawn and that did not
// change are not written again: a list that grew appends, one whose tail changed rewrites from the
// first point that differs, and only those centres are uploaded (RibbonNode::replaceFrom). A
// link's endpoints, width, colour and opacity are uniforms, so moving or restyling it uploads
// nothing.
//
// Both are custom node types: register_ribbon_node_types() is part of the cvcGL extension bundle
// (register_cvcgl_extensions), and `visible:`, `tags:`, `transform:` and views apply to them as to
// any node.

namespace cvc {
class app;
namespace gl {
namespace ariadne {

// Register `ribbon` and `draped_link` on `app`. Idempotent; call it before realize_scene().
void register_ribbon_node_types(cvc::app &app);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_ARIADNE_RIBBON_NODES_H__
