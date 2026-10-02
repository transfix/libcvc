/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/ariadne/scene.h>                  // SceneNode (props bag)
#include <cvc/ariadne/stream/stream_channel.h>  // stream_channel, subscribe, deliver_mode
#include <cvc/ariadne/stream/stream_registry.h> // stream_registry::for_app / lookup
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h> // register_scene_node_type, RealizedScene
#include <cvc/gl/ariadne/stream_node.h>
#include <cvc/gl/ariadne/stream_texture_binding.h> // StreamTextureBinding
#include <memory>
#include <string>
#include <vector>

namespace sx = cvc::ariadne::stream;

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

// A unit quad in the XY plane with full-frame UVs. make_plane() (scene_realize.cpp) emits
// NO uvs and GeometryNode::setTexture only samples through uvs, so the stream quad must
// carry its own. UVs are authored plain [0..1]; setTexture resolves the top-left-image vs
// VTK-bottom-left-origin mismatch by flipping the TCoords' V (no pixel copy).
cvc::geometry stream_quad(double w, double h) {
  const double hw = 0.5 * w, hh = 0.5 * h;
  cvc::geometry g;
  g.points() = {{{-hw, -hh, 0}}, {{hw, -hh, 0}}, {{hw, hh, 0}}, {{-hw, hh, 0}}};
  g.uvs() = {{{0.0, 0.0}}, {{1.0, 0.0}}, {{1.0, 1.0}}, {{0.0, 1.0}}};
  g.tris() = {{{0, 1, 2}}, {{0, 2, 3}}};
  return g;
}

std::shared_ptr<GraphicsNode> realize_stream_node(SceneGraph &sg, const cvc::ariadne::SceneNode &n,
                                                  GraphicsNode *parent, RealizedScene &out,
                                                  std::vector<std::string> *warnings) {
  const auto warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: stream node '" + n.id + "': " + m);
  };

  // The stream is named by its canonical token ("stream" or, as an alias, "token") — the
  // value (stream-open) returned in its handle dict. A missing token is an authoring error:
  // reject the node rather than build a quad that can never show anything.
  std::string token = n.props.str("stream");
  if (token.empty())
    token = n.props.str("token");
  if (token.empty()) {
    warn("missing required 'stream' token attribute");
    return nullptr;
  }

  // Quad dimensions default to a 1x1 square; the author sizes/aspects it via width/height
  // (the frame is mapped across the whole quad regardless, so set these to the video aspect).
  const double w = n.props.num("width", 1.0);
  const double h = n.props.num("height", 1.0);

  const cvc::geometry quad = stream_quad(w > 0.0 ? w : 1.0, h > 0.0 ? h : 1.0);
  std::shared_ptr<GraphicsNode> gn =
      parent ? parent->createChild<GeometryNode>(n.id, quad) : sg.addGraphics(n.id, quad);
  std::shared_ptr<GeometryNode> node = std::dynamic_pointer_cast<GeometryNode>(gn);
  if (!node) {
    warn("could not create a geometry node for the stream quad");
    return gn;
  }
  node->setUseSingleColor(false); // show the streamed texture, not a flat material colour

  // Resolve the ALREADY-OPEN stream and subscribe latest-wins. The realizer never opens a
  // stream; if none is live yet the quad is still returned (texture-less) so the scene is
  // structurally intact — open the stream before the scene realizes.
  cvc::app &app = sg.appContext();
  sx::stream_channel *ch = sx::stream_registry::for_app(app).lookup(token);
  if (!ch) {
    warn("no live stream for token '" + token + "' (open it before the scene realizes)");
    return node;
  }
  std::shared_ptr<sx::subscription> sub = ch->subscribe(sx::deliver_mode::latest);
  if (!sub) {
    warn("stream '" + token + "' cannot admit another subscriber (frame pool full)");
    return node;
  }

  // The binding holds the node by weak_ptr (the SceneGraph owns the node); the RealizedScene
  // owns the binding via this per-frame tick closure, so it — and its subscription — are torn
  // down with the scene, on the render thread tick_scene() runs on (the binding's contract).
  auto binding =
      std::make_shared<StreamTextureBinding>(app, token, std::weak_ptr<GeometryNode>(node), sub);
  out.custom_ticks.push_back([binding](vtkRenderer *) { binding->tick(); });
  return node;
}

} // namespace

void register_stream_node_type() {
  if (has_scene_node_type("stream"))
    return; // idempotent — registering the same type twice would just overwrite it
  register_scene_node_type("stream", realize_stream_node);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
