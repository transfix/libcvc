/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The `volume_grid` scene node type. See inc/cvc/gl/ariadne/volume_grid_nodes.h.

#include <algorithm>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/scene.h>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/tf_binding.h>
#include <cvc/gl/ariadne/volume_grid_nodes.h>
#include <cvc/gl/ariadne/volume_slice_nodes.h> // transfer_function_prop
#include <cvc/state/state.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/volume/volume_slice.h>
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

// Seed `key` from `text` while it is empty (a reload over a live tree keeps the live value).
void seed(cvc::state &key, const std::string &text) {
  if (key.value().empty() && !text.empty())
    key.value(text);
}

// A colour prop ([r, g, b] or "r,g,b") as the "r,g,b" a row and a color: widget hold.
std::string color_text(const ari::Value &v) {
  if (!v.is_seq())
    return v.scalar;
  std::string out;
  for (std::size_t i = 0; i < v.items.size(); ++i)
    out += (i ? "," : "") + v.items[i].scalar;
  return out;
}

// One of the node's data on the state tree, as a document names it: the key grid.source.* holds
// (relative to the document's scope, or '/'-rooted) and, for a dataset, its store entry (the
// document's data.<name>), whose loading the scene waits for.
struct data_source {
  std::string path;
  std::string dataset;
};

// A dataset's output: data.<name>.<item> (a name with a leading '/' is the root's).
data_source dataset_source(cvc::app &app, const std::string &prefix, const std::string &name,
                           const std::string &item) {
  data_source s;
  s.dataset = name[0] == '/' ? name : std::string(ari::kDataRoot) + cvc::state::SEPARATOR + name;
  s.path = s.dataset + cvc::state::SEPARATOR + item;
  ari::DataStore::of(app)->touch(ari::resolve_bind(prefix, s.dataset)); // a lazy one loads now
  return s;
}

// points: and session: -- { data: <name>, item: <output> } or { state: <key> }.
data_source prop_source(cvc::app &app, const std::string &prefix, const ari::SceneNode &n,
                        const char *prop, const char *item, std::vector<std::string> *warnings) {
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

// The volume: source: { data: <name>, item }, { state: <key> } or { node: <id>, key: <key> }.
data_source volume_source(cvc::app &app, const std::string &prefix, const ari::SceneNode &n,
                          std::vector<std::string> *warnings) {
  if (!n.source_data.empty())
    return dataset_source(app, prefix, n.source_data,
                          n.source_item.empty() ? std::string("volume") : n.source_item);
  if (const ari::Value *src = n.props.find("source"); src && src->is_map()) {
    if (src->has("state"))
      return {src->str("state"), std::string()};
    if (src->has("node")) {
      // A top-level node's keys are graphics.root.children.<id>; a dotted one is a path.
      const std::string id = src->str("node");
      const std::string at =
          id.find('.') != std::string::npos
              ? id
              : std::string("graphics.root.children") + cvc::state::SEPARATOR + id;
      return {at + cvc::state::SEPARATOR + src->str("key", "grid.volume"), std::string()};
    }
  }
  if (!n.source_file.empty() || !n.source_sdf_mesh.empty()) {
    const std::string entry =
        !n.source_file.empty() ? n.source_file
                               : "{ generate: sdf, params: { mesh: " + n.source_sdf_mesh + " } }";
    warn(warnings, "ari: scene node '" + n.id +
                       "' (volume_grid) reads no file: load it as app data (data: { " + n.id +
                       ": " + entry + " }) and give source: { data: " + n.id + " }");
  } else {
    warn(warnings, "ari: scene node '" + n.id +
                       "' (volume_grid) has no source: give source: { data: <name> }, "
                       "{ state: <key> } or { node: <id> }");
  }
  return {};
}

// The props that seed <node>.grid.*.
void seed_keys(VolumeGridNode &g, const ari::SceneNode &n, const data_source &volume,
               const data_source &points, const data_source &session,
               std::vector<std::string> *warnings) {
  cvc::state &grid = g.getState("grid");
  for (const char *k : {"view", "tool"})
    if (n.props.has(k))
      seed(grid(k), n.props.str(k));
  if (const ari::Value *s = n.props.find("slice")) {
    if (s->is_map()) {
      for (const char *a : {"x", "y", "z"})
        if (s->has(a))
          seed(grid(std::string("slice.") + a), s->str(a));
    } else {
      warn(warnings, "ari: scene node '" + n.id + "': slice: must be a map of x, y, z");
    }
  }
  if (const ari::Value *d = n.props.find("display")) {
    if (d->is_map()) {
      const std::vector<std::string> names = cvc::param_names<cvc::volume_grid_display_params>();
      for (const auto &e : d->entries) {
        if (std::find(names.begin(), names.end(), e.first) == names.end())
          warn(warnings, "ari: scene node '" + n.id + "': display: '" + e.first +
                             "' is not one of its keys -- ignored");
        else
          seed(grid("display." + e.first), e.second.scalar);
      }
    } else {
      warn(warnings, "ari: scene node '" + n.id + "': display: must be a map");
    }
  }
  seed(grid("source.volume"), volume.path);
  seed(grid("source.points"), points.path);
  seed(grid("source.session"), session.path);
  for (const char *k : {"var", "time"})
    if (n.props.has(k))
      seed(grid(std::string("source.") + k), n.props.str(k));
  // classes: the rows a list: binds, while there are none.
  if (const ari::Value *c = n.props.find("classes")) {
    cvc::state &rows = grid("classes");
    if (!c->is_seq()) {
      warn(warnings, "ari: scene node '" + n.id + "': classes: must be a list");
    } else if (rows.value().empty()) {
      for (std::size_t i = 0; i < c->items.size(); ++i) {
        const ari::Value &row = c->items[i];
        cvc::state &r = rows(std::to_string(i));
        if (!row.is_map() || !row.has("name")) {
          warn(warnings,
               "ari: scene node '" + n.id + "': classes[" + std::to_string(i) + "] needs a name");
          continue;
        }
        r("name").value(row.str("name"));
        if (const ari::Value *col = row.find("color"))
          r("color").value(color_text(*col));
        if (row.has("radius"))
          r("radius").value(row.str("radius"));
      }
      rows.value(std::to_string(c->items.size()));
    }
  }
}

std::shared_ptr<GraphicsNode> realize_volume_grid(SceneGraph &sg, const ari::SceneNode &n,
                                                  GraphicsNode *parent, RealizedScene &out,
                                                  std::vector<std::string> *warnings) {
  cvc::app &app = sg.appContext();
  const std::string prefix = out.bind_prefix;
  const data_source volume = volume_source(app, prefix, n, warnings);
  const data_source points = prop_source(app, prefix, n, "points", "classes", warnings);
  const data_source session = prop_source(app, prefix, n, "session", "session", warnings);
  if (n.props.has("region_reads"))
    warn(warnings, "ari: scene node '" + n.id +
                       "': region_reads: the node reads no file (a host gives a large one read "
                       "plane by plane as a slice_source on the volume's key) -- ignored");

  GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
  if (!parent && sg.hasGraphics(n.id)) // last-wins parity with sg.addGraphics (see volren)
    sg.removeGraphics(n.id);
  std::shared_ptr<VolumeGridNode> g = pr->addGraphicsChild<VolumeGridNode>(n.id);
  if (!g)
    return nullptr;
  if (!parent)
    sg.registerGraphics(n.id, g);
  g->setScope(prefix);
  g->ensureChildren();
  seed_keys(*g, n, volume, points, session, warnings);

  // The views find the planes by their tags: the grid's id (a view showing this grid alone) and
  // their own name (slice_xy ...).
  for (cvc::slice_axis a : {cvc::slice_axis::z, cvc::slice_axis::y, cvc::slice_axis::x}) {
    std::vector<std::string> tags = n.tags;
    tags.push_back(n.id);
    tags.push_back(VolumeGridNode::sliceName(a));
    out.nodes.push_back({n.id + "_" + VolumeGridNode::sliceName(a), tags, g->slice(a)});
  }
  std::vector<std::string> tags = n.tags;
  tags.push_back(n.id);
  tags.push_back("surfaces");
  out.nodes.push_back({n.id + "_surfaces", tags, g->surfaces()});

  g->tick(); // a volume already on the tree is taken now, so the scene frames on it

  // The transfer function, over the volume (or the stand-in plane of a slice_source read plane by
  // plane); it follows the volume as it changes, and has none until the first arrives.
  tf_binding_options opt;
  opt.seed = transfer_function_prop(n, warnings);
  std::weak_ptr<VolumeGridNode> weak = g;
  opt.volume_source = [weak]() -> std::shared_ptr<const cvc::volume> {
    const std::shared_ptr<VolumeGridNode> node = weak.lock();
    return node ? node->histogramVolume() : nullptr;
  };
  const std::shared_ptr<const cvc::volume> now = g->histogramVolume();
  attach_transfer_function(out, g, tf_target::volume_slice, now ? *now : cvc::volume(app), opt,
                           warnings);

  out.custom_ticks.push_back([weak](vtkRenderer *) {
    if (std::shared_ptr<VolumeGridNode> node = weak.lock())
      node->tick();
  });
  // While a dataset the node follows is loading, the scene has work in flight.
  std::vector<std::string> loading;
  for (const data_source *s : {&volume, &points, &session})
    if (!s->dataset.empty())
      loading.push_back(ari::resolve_bind(prefix, s->dataset));
  if (!loading.empty()) {
    cvc::app *ap = &app;
    out.pending_work.push_back([weak, ap, loading]() {
      if (!weak.lock())
        return false;
      for (const std::string &d : loading) {
        const std::string st = ari::data_status(*ap, d);
        if (st == "fetching" || st == "unpacking" || st == "generating" || st == "decoding")
          return true;
      }
      return false;
    });
  }
  return g;
}

} // namespace

void register_volume_grid_node_type(cvc::app &app) {
  NodeTypeOptions options;
  options.reads_data = true; // source: { data } is the volume the node follows
  register_scene_node_type(app, "volume_grid", realize_volume_grid, options);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
