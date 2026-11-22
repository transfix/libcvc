/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// `type: volume_render`. See inc/cvc/gl/ariadne/volume_render_nodes.h.

#include <cstdint>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/scene.h>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeRenderNode.h>
#include <cvc/gl/ariadne/node_data.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/tf_binding.h>
#include <cvc/gl/ariadne/volume_render_nodes.h>
#include <cvc/gl/ariadne/volume_slice_nodes.h> // transfer_function_prop
#include <cvc/state/state.h>
#include <cvc/volume/volume.h>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

namespace {

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

void put(cvc::state &s, const std::string &v) {
  if (s.value() != v)
    s.value(v);
}

// What the node follows and binds, ticked on the frame thread.
struct live {
  cvc::app *app = nullptr;
  std::weak_ptr<VolumeRenderNode> node;
  std::shared_ptr<VolumeFollower> follower; // <node>.source.volume, followed (node_data.h)
  tf_binding_options opt;                   // the seed and the model key
  std::shared_ptr<tf_binding> binding;
  std::uint64_t bound = std::numeric_limits<std::uint64_t>::max();

  void tick() {
    const std::shared_ptr<VolumeRenderNode> n = node.lock();
    if (!n) {
      binding.reset();
      return;
    }
    if (follower->tick()) {
      n->setVolume(follower->volume());
      if (follower->source() && !follower->volume())
        put(n->getState("source.status"),
            "error: a volume_render draws a volume in memory, not a file read plane by plane");
    }
    n->tick();
    if (n->rendererVersion() != bound) {
      bound = n->rendererVersion();
      bind(n);
    }
    if (binding)
      binding->tick();
  }

  // One binding for the model at a time: the renderer drawing now, or (none drawing) the node
  // itself, which lowers nothing but keeps <tf>.range and the overlays on the volume.
  void bind(const std::shared_ptr<VolumeRenderNode> &n) {
    binding.reset(); // the old binding stops before the new one owns the key
    tf_binding_options o = opt;
    std::weak_ptr<VolumeRenderNode> weak = n;
    o.volume_source = [weak]() -> std::shared_ptr<const cvc::volume> {
      const std::shared_ptr<VolumeRenderNode> v = weak.lock();
      return v ? v->volume() : nullptr;
    };
    std::shared_ptr<GraphicsNode> target_node = n->renderer();
    tf_target target = tf_target::volume_slice;
    switch (target_node ? n->rendererKind() : VolumeRenderNode::kind::none) {
    case VolumeRenderNode::kind::vtk:
      target = tf_target::volume;
      break;
    case VolumeRenderNode::kind::volren:
      target = tf_target::volren;
      break;
    case VolumeRenderNode::kind::volslice:
      target = tf_target::volslice;
      break;
    default:
      target_node = n;
      break;
    }
    const std::shared_ptr<const cvc::volume> vol = n->volume();
    binding =
        make_transfer_function(target_node, target, vol ? *vol : cvc::volume(*app), std::move(o));
  }
};

std::shared_ptr<GraphicsNode> realize_volume_render(SceneGraph &sg, const ari::SceneNode &n,
                                                    GraphicsNode *parent, RealizedScene &out,
                                                    std::vector<std::string> *warnings) {
  cvc::app &app = sg.appContext();
  const std::string prefix = out.bind_prefix;
  // source: { data | state | node } (node_data.h); { node: <id> } is a volume_grid's grid.volume
  // when that grid was realized before this node, else that node's volume.
  const NodeDataSource source = volume_data_source(sg, out, n);
  if (!source) {
    if (!n.source_file.empty() || !n.source_sdf_mesh.empty()) {
      const std::string entry =
          !n.source_file.empty() ? n.source_file
                                 : "{ generate: sdf, params: { mesh: " + n.source_sdf_mesh + " } }";
      warn(warnings, "ari: scene node '" + n.id +
                         "' (volume_render) reads no file: load it as app data (data: { " + n.id +
                         ": " + entry + " }) and give source: { data: " + n.id + " }");
    } else {
      warn(warnings, "ari: scene node '" + n.id +
                         "' (volume_render) has no source: give source: { data: <name> }, "
                         "{ state: <key> } or { node: <id> }");
    }
  }

  GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
  if (!parent && sg.hasGraphics(n.id)) // last-wins parity with sg.addGraphics (see volren)
    sg.removeGraphics(n.id);
  std::shared_ptr<VolumeRenderNode> v = pr->addGraphicsChild<VolumeRenderNode>(n.id);
  if (!v)
    return nullptr;
  if (!parent)
    sg.registerGraphics(n.id, v);

  // A prop seeds its key only while the key is empty (a restored session's value stays).
  const auto seed = [](cvc::state &key, const std::string &text) {
    if (key.value().empty() && !text.empty())
      key.value(text);
  };
  seed(v->getState("source.volume"), source.path); // a program may point it elsewhere
  const std::string renderer = n.props.str("renderer", "vtk");
  VolumeRenderNode::kind k;
  if (!VolumeRenderNode::kindOf(renderer, k))
    warn(warnings, "ari: scene node '" + n.id + "': renderer '" + renderer +
                       "' is not vtk, volren, volslice or none (nothing is drawn until it is)");
  seed(v->getState("renderer"), renderer);

  auto l = std::make_shared<live>();
  l->app = &app;
  l->node = v;
  l->follower = std::make_shared<VolumeFollower>(v, prefix);
  l->opt.seed = transfer_function_prop(n, warnings);
  l->opt.model_key = v->stateName("transfer_function");
  l->tick(); // a volume already on the tree is drawn now, so the scene frames on it
  out.custom_ticks.push_back([l](vtkRenderer *) { l->tick(); });

  // While the dataset it draws is loading, the scene has work in flight.
  wait_for_datasets(out, app, v, {source});
  return v;
}

} // namespace

void register_volume_render_node_type(cvc::app &app) {
  NodeTypeOptions options;
  options.reads_data = true; // source: { data } is the volume the node follows
  register_scene_node_type(app, "volume_render", realize_volume_render, options);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
