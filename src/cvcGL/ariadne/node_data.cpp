/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A scene node's data on the state tree. See inc/cvc/gl/ariadne/node_data.h.

#include <boost/any.hpp>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/scene.h>
#include <cvc/core/app.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/gl/ariadne/node_data.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_slice.h>
#include <utility>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

namespace {

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

// A dataset's output: data.<name>.<item> (a name with a leading '/' is the root's). A lazy dataset
// loads now.
NodeDataSource dataset_source(cvc::app &app, const std::string &prefix, const std::string &name,
                              const std::string &item) {
  NodeDataSource s;
  s.dataset = name[0] == '/' ? name : std::string(ari::kDataRoot) + cvc::state::SEPARATOR + name;
  s.path = s.dataset + cvc::state::SEPARATOR + item;
  ari::DataStore::of(app)->touch(ari::resolve_bind(prefix, s.dataset));
  return s;
}

// The node realized by `id` before now: a top-level one, or one at any depth by its id.
std::shared_ptr<GraphicsNode> realized_node(SceneGraph &sg, const RealizedScene &out,
                                            const std::string &id) {
  if (std::shared_ptr<GraphicsNode> g = sg.getGraphics(id))
    return g;
  for (const RealizedScene::TaggedNode &t : out.nodes)
    if (t.id == id)
      if (std::shared_ptr<GraphicsNode> g = t.node.lock())
        return g;
  return nullptr;
}

// The state path a { node: <id> } names: a path when it has a '.', else the realized node's own
// ('/'-rooted: absolute), else where a top-level node's keys are.
std::string node_path(SceneGraph &sg, const RealizedScene &out, const std::string &id) {
  if (id.find('.') != std::string::npos)
    return id;
  if (std::shared_ptr<GraphicsNode> g = realized_node(sg, out, id))
    return "/" + g->stateName();
  return std::string("graphics.root.children") + cvc::state::SEPARATOR + id;
}

// For a file read plane by plane: its middle plane down z as a one-voxel-thick volume, with the
// file's range, for a transfer function's range and histogram.
std::shared_ptr<const cvc::volume> stand_in(cvc::app &app, cvc::slice_source &src) {
  auto v = std::make_shared<cvc::volume>(app);
  try {
    const cvc::slice_grid &g = src.grid();
    const std::shared_ptr<const cvc::volume_slice_data> s =
        src.slice(cvc::slice_axis::z, g.count(cvc::slice_axis::z) / 2);
    *v = cvc::volume(app, cvc::dimension(s->width, s->height, 1), cvc::Double);
    for (std::uint64_t r = 0; r < s->height; ++r)
      for (std::uint64_t c = 0; c < s->width; ++c)
        (*v)(c, r, 0, s->at(c, r));
  } catch (const std::exception &) {
    // the plane could not be read: the range alone
  }
  v->min(src.min());
  v->max(src.max());
  return v;
}

} // namespace

NodeDataSource node_data_source(SceneGraph &sg, const RealizedScene &out,
                                const cvc::ariadne::SceneNode &n, const std::string &item,
                                const std::string &node_key) {
  cvc::app &app = sg.appContext();
  if (!n.source_data.empty())
    return dataset_source(app, out.bind_prefix, n.source_data,
                          n.source_item.empty() ? item : n.source_item);
  const ari::Value *src = n.props.find("source");
  if (!src || !src->is_map())
    return {};
  if (src->has("state"))
    return {src->str("state"), std::string()};
  if (src->has("node"))
    return {node_path(sg, out, src->str("node")) + cvc::state::SEPARATOR +
                src->str("key", node_key),
            std::string()};
  return {};
}

NodeDataSource volume_data_source(SceneGraph &sg, const RealizedScene &out,
                                  const cvc::ariadne::SceneNode &n) {
  std::string key = "volume";
  if (const ari::Value *src = n.props.find("source"); src && src->is_map() && src->has("node"))
    if (std::dynamic_pointer_cast<VolumeGridNode>(realized_node(sg, out, src->str("node"))))
      key = "grid.volume";
  return node_data_source(sg, out, n, "volume", key);
}

NodeDataSource prop_data_source(cvc::app &app, const std::string &prefix,
                                const cvc::ariadne::SceneNode &n, const char *prop,
                                const char *item, std::vector<std::string> *warnings) {
  const ari::Value *v = n.props.find(prop);
  if (!v)
    return {};
  if (v->is_map() && v->has("data"))
    return dataset_source(app, prefix, v->str("data"), v->str("item", item));
  if (v->is_map() && v->has("state"))
    return {v->str("state"), std::string()};
  warn(warnings, "ari: scene node '" + n.id + "': " + prop +
                     ": is { data: <name> } (a data: entry the app loads) or { state: <key> }; "
                     "the node reads no file -- ignored");
  return {};
}

void wait_for_datasets(RealizedScene &out, cvc::app &app, std::weak_ptr<GraphicsNode> node,
                       const std::vector<NodeDataSource> &sources) {
  std::vector<std::string> loading;
  for (const NodeDataSource &s : sources)
    if (!s.dataset.empty())
      loading.push_back(ari::resolve_bind(out.bind_prefix, s.dataset));
  if (loading.empty())
    return;
  cvc::app *ap = &app;
  out.pending_work.push_back([node = std::move(node), ap, loading]() {
    if (!node.lock())
      return false;
    for (const std::string &d : loading) {
      const std::string st = ari::data_status(*ap, d);
      if (st == "fetching" || st == "unpacking" || st == "generating" || st == "decoding")
        return true;
    }
    return false;
  });
}

// --- VolumeFollower --------------------------------------------------------------------------

VolumeFollower::VolumeFollower(const std::shared_ptr<GraphicsNode> &node, std::string scope)
    : m_node(node), m_scope(std::move(scope)) {}

std::string VolumeFollower::sourcePath() const {
  const std::shared_ptr<GraphicsNode> node = m_node.lock();
  if (!node)
    return std::string();
  return ari::resolve_bind(m_scope, node->getState("source.volume").value());
}

void VolumeFollower::status(const std::string &s) {
  if (m_said && s == m_status)
    return;
  m_status = s;
  m_said = true;
  if (const std::shared_ptr<GraphicsNode> node = m_node.lock())
    node->getState("source.status").value(s);
}

bool VolumeFollower::tick() {
  const std::shared_ptr<GraphicsNode> node = m_node.lock();
  if (!node)
    return false;
  const std::string key = node->getState("source.volume").value();
  const std::string path = sourcePath();
  bool adopted = false;
  boost::any d;
  if (path == node->stateName("volume")) {
    status("error: source.volume names this node's own volume");
  } else if (m_followed.changed(node->app().root(), path, d, cvc::volume_payload_pointee)) {
    std::shared_ptr<cvc::slice_source> src;
    std::shared_ptr<const cvc::volume> vol;
    if (cvc::volume_payload(d, src, vol)) {
      // The same volume written again (or a link moved to it) changes nothing.
      if (!(vol && vol == m_volume) && !(!vol && src == m_source)) {
        m_source = std::move(src);
        m_volume = std::move(vol);
        m_histogram = m_volume ? m_volume : stand_in(node->app(), *m_source);
        node->getState("volume").data(m_volume ? boost::any(m_volume) : boost::any());
        adopted = true;
      }
      status(std::string());
    } else if (!d.empty()) {
      status("error: " + key + " holds no volume");
    }
  }
  // Not there yet (a dataset still loading, a program that has not written it): said once.
  if (!m_source && path.empty())
    status("no volume: source.volume names none");
  else if (!m_source && m_status.empty())
    status("waiting for a volume at " + key);
  return adopted;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
