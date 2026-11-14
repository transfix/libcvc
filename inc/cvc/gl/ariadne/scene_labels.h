/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_ARIADNE_SCENE_LABELS_H__
#define __CVC_GL_ARIADNE_SCENE_LABELS_H__

// A scene node's `label:` (docs/ARIADNE.md §5.21), drawn by the scene's
// cvc::gl::LabelLayerNode: text over the node, offset above it, faded and culled by the camera's
// distance, on a backdrop with an edge, at a per-label scale, nearer labels over farther ones.
//
//   - node: car
//     source: { box: [4, 2, 1.5] }
//     label:
//       text: Alpha             # or bind: <state key> (read each frame)
//       offset: 2               # metres above the top of the node (or [x, y, z])
//       color: [1, 1, 1]        # the text; [r, g, b], "#rrggbb" or c0..c7
//       backdrop: [0, 0, 0]     # the box behind it (none: no box); backdrop_opacity: 0.6
//       edge: c0                # a frame round the box
//       size: 14                # pixels at scale 1
//       scale: 1                # a number or a state key
//       focus: { key: cam.follow, value: car, scale: 1.4 }   # drawn larger while key == value
//       fade: [300, 600]        # whole nearer than 300 m, gone past 600 m
//
// The label reads the node's transform where it is drawn; nothing is written to state per frame.
// The scene's `entity_labels:` gives every world entity (WorldVehicles) a label in the same keys,
// its text "{name}" by default ({name}, {id}, {archetype} and {team} are filled in).

#include <cvc/ariadne/value.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {
struct SceneNode;
}
namespace gl {
class GraphicsNode;
class SceneGraph;
struct LabelStyle;
namespace ariadne {
struct RealizedScene;

// `label:` (a map, or a string: its text) as a style, and its text; warnings name `who`.
LabelStyle parse_label_style(const cvc::ariadne::Value &label, const std::string &who,
                             std::vector<std::string> *warnings);

// Give `node` the label its spec's `label:` declares, if any: a label on the scene's layer (its
// bound keys resolved against out.bind_prefix, and read by the layer as it draws).
void apply_scene_label(SceneGraph &sg, const std::shared_ptr<GraphicsNode> &node,
                       const cvc::ariadne::SceneNode &n, RealizedScene &out,
                       std::vector<std::string> *warnings);

// A world entity's label, from the scene's `entity_labels:` (`style`: a map, or true for the
// default look) and the entity's fields (`fields`: name, id, archetype, team), for its node.
void apply_entity_label(SceneGraph &sg, const std::shared_ptr<GraphicsNode> &node,
                        const cvc::ariadne::Value &style,
                        const std::vector<std::pair<std::string, std::string>> &fields,
                        const std::string &bind_prefix, std::vector<std::string> *warnings);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_ARIADNE_SCENE_LABELS_H__
