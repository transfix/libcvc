/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// `type: volume_render`. See inc/cvc/gl/ariadne/volume_render_nodes.h.

#include <boost/any.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <cstdint>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/scene.h>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeRenderNode.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/tf_binding.h>
#include <cvc/gl/ariadne/volume_render_nodes.h>
#include <cvc/gl/ariadne/volume_slice_nodes.h> // transfer_function_prop
#include <cvc/state/state.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_slice.h>
#include <limits>
#include <memory>
#include <string>
#include <typeindex>
#include <typeinfo>
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

// A key's data as a volume to draw: a std::shared_ptr<const cvc::volume> (a data: entry's), a
// std::shared_ptr<cvc::volume>, a cvc::volume (a handle on its voxels) or a slice_source holding
// its volume in memory. Null for anything else (a file read plane by plane has no whole volume).
std::shared_ptr<const cvc::volume> volume_payload(const boost::any &d) {
  if (const auto *c = boost::any_cast<std::shared_ptr<const cvc::volume>>(&d))
    return *c;
  if (const auto *m = boost::any_cast<std::shared_ptr<cvc::volume>>(&d))
    return *m;
  if (const auto *v = boost::any_cast<cvc::volume>(&d))
    return std::make_shared<const cvc::volume>(*v);
  if (const auto *s = boost::any_cast<std::shared_ptr<cvc::slice_source>>(&d))
    if (*s)
      if (const cvc::volume *iv = (*s)->in_memory_volume())
        return std::shared_ptr<const cvc::volume>(*s, iv); // kept by the source
  return nullptr;
}

template <class T> bool pointee(const boost::any &d, const void *&id) {
  if (const auto *p = boost::any_cast<std::shared_ptr<T>>(&d)) {
    id = p->get();
    return true;
  }
  return false;
}

// One followed key, as VolumeGridNode follows its sources: a change is another node (the key
// edited, a link moved), a new lastMod, or a new payload type or pointee (a write within the
// clock's resolution leaves lastMod as it was).
struct follower {
  std::string path, node;
  boost::posix_time::ptime mod;
  std::type_index type = typeid(void);
  const void *id = nullptr;

  bool changed(cvc::app &app, const std::string &at, boost::any &payload) {
    cvc::state *s = at.empty() ? nullptr : app.root().findThroughLinks(at);
    if (!s) {
      const bool moved = !node.empty() || at != path;
      *this = follower{};
      path = at;
      payload = boost::any();
      return moved; // the key names nothing (now): nothing to draw
    }
    boost::any d = s->data();
    const void *p = nullptr;
    (void)(pointee<const cvc::volume>(d, p) || pointee<cvc::volume>(d, p) ||
           pointee<cvc::slice_source>(d, p));
    const std::string n = s->fullName();
    const boost::posix_time::ptime m = s->lastMod();
    const std::type_index t(d.type());
    if (at == path && n == node && m == mod && t == type && p == id)
      return false;
    path = at;
    node = n;
    mod = m;
    type = t;
    id = p;
    payload = std::move(d);
    return true;
  }
};

// What the node follows and binds, ticked on the frame thread.
struct live {
  cvc::app *app = nullptr;
  std::weak_ptr<VolumeRenderNode> node;
  std::string scope;
  bool node_auto = false; // source: { node } without key: its volume, else its grid.volume
  follower volume;
  tf_binding_options opt; // the seed and the model key
  std::shared_ptr<tf_binding> binding;
  std::uint64_t bound = std::numeric_limits<std::uint64_t>::max();

  std::string resolve(const std::string &key) const {
    if (key.empty())
      return std::string();
    if (key[0] == '/')
      return key.substr(1);
    return scope.empty() ? key : scope + cvc::state::SEPARATOR + key;
  }

  bool holds_data(const std::string &at) const {
    cvc::state *s = at.empty() ? nullptr : app->root().findThroughLinks(at);
    return s && !s->data().empty();
  }

  void tick() {
    const std::shared_ptr<VolumeRenderNode> n = node.lock();
    if (!n) {
      binding.reset();
      return;
    }
    std::string at = resolve(n->getState("source.volume").value());
    static const std::string tail = ".volume";
    if (node_auto && at.size() > tail.size() &&
        at.compare(at.size() - tail.size(), tail.size(), tail) == 0 && !holds_data(at)) {
      const std::string grid = at.substr(0, at.size() - tail.size()) + ".grid.volume";
      if (holds_data(grid))
        at = grid;
    }
    boost::any d;
    if (volume.changed(*app, at, d)) {
      const std::shared_ptr<const cvc::volume> v = volume_payload(d);
      if (v || d.empty())
        n->setVolume(v);
      put(n->getState("source.status"),
          v || d.empty() ? std::string() : "error: " + at + " holds no volume");
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

// source: { data: <name>, item }, { state: <key> } or { node: <id>, key: <key> } as the key
// <node>.source.volume holds (relative to the document's scope), and the dataset to wait for.
std::string volume_source(cvc::app &app, const std::string &prefix, const ari::SceneNode &n,
                          bool &node_auto, std::string &dataset,
                          std::vector<std::string> *warnings) {
  node_auto = false;
  if (!n.source_data.empty()) {
    dataset = n.source_data[0] == '/'
                  ? n.source_data
                  : std::string(ari::kDataRoot) + cvc::state::SEPARATOR + n.source_data;
    ari::DataStore::of(app)->touch(ari::resolve_bind(prefix, dataset)); // a lazy one loads now
    return dataset + cvc::state::SEPARATOR +
           (n.source_item.empty() ? std::string("volume") : n.source_item);
  }
  if (const ari::Value *src = n.props.find("source"); src && src->is_map()) {
    if (src->has("state"))
      return src->str("state");
    if (src->has("node")) {
      const std::string id = src->str("node");
      const std::string at =
          id.find('.') != std::string::npos
              ? id
              : std::string("graphics.root.children") + cvc::state::SEPARATOR + id;
      node_auto = !src->has("key");
      return at + cvc::state::SEPARATOR + src->str("key", "volume");
    }
  }
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
  return std::string();
}

std::shared_ptr<GraphicsNode> realize_volume_render(SceneGraph &sg, const ari::SceneNode &n,
                                                    GraphicsNode *parent, RealizedScene &out,
                                                    std::vector<std::string> *warnings) {
  cvc::app &app = sg.appContext();
  const std::string prefix = out.bind_prefix;
  bool node_auto = false;
  std::string dataset;
  const std::string source = volume_source(app, prefix, n, node_auto, dataset, warnings);

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
  seed(v->getState("source.volume"), source);
  const std::string renderer = n.props.str("renderer", "vtk");
  VolumeRenderNode::kind k;
  if (!VolumeRenderNode::kindOf(renderer, k))
    warn(warnings, "ari: scene node '" + n.id + "': renderer '" + renderer +
                       "' is not vtk, volren, volslice or none (nothing is drawn until it is)");
  seed(v->getState("renderer"), renderer);

  auto l = std::make_shared<live>();
  l->app = &app;
  l->node = v;
  l->scope = prefix;
  l->node_auto = node_auto;
  l->opt.seed = transfer_function_prop(n, warnings);
  l->opt.model_key = v->stateName("transfer_function");
  l->tick(); // a volume already on the tree is drawn now, so the scene frames on it
  out.custom_ticks.push_back([l](vtkRenderer *) { l->tick(); });

  // While the dataset it draws is loading, the scene has work in flight.
  if (!dataset.empty()) {
    const std::string loading = ari::resolve_bind(prefix, dataset);
    std::weak_ptr<VolumeRenderNode> weak = v;
    cvc::app *ap = &app;
    out.pending_work.push_back([weak, ap, loading]() {
      if (!weak.lock())
        return false;
      const std::string st = ari::data_status(*ap, loading);
      return st == "fetching" || st == "unpacking" || st == "generating" || st == "decoding";
    });
  }
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
