/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The `volume_slice` scene node type. See inc/cvc/gl/ariadne/volume_slice_nodes.h.

#include <algorithm>
#include <cstdlib>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/gl/ariadne/node_data.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/tf_binding.h>
#include <cvc/gl/ariadne/volume_slice_nodes.h>
#include <cvc/tf/model.h>
#include <cvc/volume/volume_file_info.h>
#include <cvc/volume/volume_slice.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

namespace {

// A file larger than this many voxels is read one plane at a time under region_reads: auto.
constexpr double kRegionVoxels = 67108864.0; // 64 Mi

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

void json_string(const std::string &s, std::string &out) {
  out += '"';
  for (char c : s) {
    if (c == '"' || c == '\\')
      out += '\\';
    out += c;
  }
  out += '"';
}

// A neutral Value as JSON: numbers and booleans as they are, other scalars as strings.
void to_json(const ari::Value &v, std::string &out) {
  if (v.is_map()) {
    out += '{';
    for (std::size_t i = 0; i < v.entries.size(); ++i) {
      out += i ? "," : "";
      json_string(v.entries[i].first, out);
      out += ':';
      to_json(v.entries[i].second, out);
    }
    out += '}';
  } else if (v.is_seq()) {
    out += '[';
    for (std::size_t i = 0; i < v.items.size(); ++i) {
      out += i ? "," : "";
      to_json(v.items[i], out);
    }
    out += ']';
  } else {
    const std::string &s = v.scalar;
    char *end = nullptr;
    std::strtod(s.c_str(), &end);
    const bool number = !s.empty() && end && *end == '\0';
    if (number || s == "true" || s == "false")
      out += s;
    else
      json_string(s, out);
  }
}

// The props that seed <node>.slice.*: axis (by any name parse_slice_axis takes), index, filter,
// opaque, overlay_offset.
void seed_keys(VolumeSliceNode &vn, const ari::SceneNode &n, std::vector<std::string> *warnings) {
  if (n.props.has("axis")) {
    cvc::slice_axis a = cvc::slice_axis::z;
    if (cvc::parse_slice_axis(n.props.str("axis"), a))
      vn.getState("slice.axis").value(std::string(cvc::slice_axis_name(a)));
    else
      warn(warnings, "ari: scene node '" + n.id + "': axis '" + n.props.str("axis") +
                         "' is not x, y or z (or xy, xz, zy) -- using z");
  }
  for (const char *k : {"index", "filter", "opaque", "overlay_offset"})
    if (n.props.has(k))
      vn.getState(std::string("slice.") + k).value(n.props.str(k));
}

// The volume_slice node a `volume_of: <id>` prop names, realized before this one (top level, or at
// any depth by id), or null.
std::shared_ptr<VolumeSliceNode> shared_slice(SceneGraph &sg, const RealizedScene &out,
                                              const std::string &id) {
  if (auto s = std::dynamic_pointer_cast<VolumeSliceNode>(sg.getGraphics(id)))
    return s;
  for (const RealizedScene::TaggedNode &t : out.nodes)
    if (t.id == id)
      if (auto s = std::dynamic_pointer_cast<VolumeSliceNode>(t.node.lock()))
        return s;
  return nullptr;
}

// A new VolumeSliceNode for `n` under `parent` (or the root, last wins), its slice.* keys seeded.
std::shared_ptr<VolumeSliceNode> make_slice(SceneGraph &sg, const ari::SceneNode &n,
                                            GraphicsNode *parent,
                                            std::vector<std::string> *warnings) {
  GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
  if (!parent && sg.hasGraphics(n.id)) // last-wins parity with sg.addGraphics (see volren)
    sg.removeGraphics(n.id);
  std::shared_ptr<VolumeSliceNode> vn = pr->addGraphicsChild<VolumeSliceNode>(n.id);
  if (!vn)
    return nullptr;
  if (!parent)
    sg.registerGraphics(n.id, vn);
  seed_keys(*vn, n, warnings);
  return vn;
}

// source: { data | state | node }: the volume the app holds, followed (node_data.h). The plane
// cuts that volume, its voxels shared, and cuts each new one the key holds.
std::shared_ptr<GraphicsNode> realize_followed_slice(SceneGraph &sg, const ari::SceneNode &n,
                                                     GraphicsNode *parent, RealizedScene &out,
                                                     const NodeDataSource &src,
                                                     std::vector<std::string> *warnings) {
  cvc::app &app = sg.appContext();
  if (n.props.has("region_reads"))
    warn(warnings, "ari: scene node '" + n.id +
                       "': region_reads: the node reads no file (a host gives a large one read "
                       "plane by plane as a slice_source on the volume's key) -- ignored");
  if (n.has_fit)
    warn(warnings, "ari: scene node '" + n.id +
                       "' (volume_slice): fit: places a volume the node reads itself; a data: "
                       "entry's volume is placed where it is made (an sdf generator's fit:) -- "
                       "ignored");
  std::shared_ptr<VolumeSliceNode> vn = make_slice(sg, n, parent, warnings);
  if (!vn)
    return nullptr;
  if (cvc::state &key = vn->getState("source.volume"); key.value().empty())
    key.value(src.path); // a reload over a live tree keeps where a program pointed it

  auto follower = std::make_shared<VolumeFollower>(vn, out.bind_prefix);
  std::weak_ptr<VolumeSliceNode> weak = vn;
  const auto take = [weak, follower]() {
    if (!follower->tick())
      return;
    if (const std::shared_ptr<VolumeSliceNode> node = weak.lock())
      node->setSource(follower->source());
  };
  take(); // a volume already on the tree is cut now, so the scene frames on it

  tf_binding_options opt;
  opt.seed = transfer_function_prop(n, warnings);
  opt.volume_source = [follower]() { return follower->histogramVolume(); };
  const std::shared_ptr<const cvc::volume> now = follower->histogramVolume();
  attach_transfer_function(out, vn, tf_target::volume_slice, now ? *now : cvc::volume(app), opt,
                           warnings);
  out.custom_ticks.push_back([weak, take](vtkRenderer *) {
    take();
    if (std::shared_ptr<VolumeSliceNode> node = weak.lock())
      node->tick();
  });
  wait_for_datasets(out, app, vn, {src});
  return vn;
}

std::shared_ptr<GraphicsNode> realize_volume_slice(SceneGraph &sg, const ari::SceneNode &n,
                                                   GraphicsNode *parent, RealizedScene &out,
                                                   std::vector<std::string> *warnings) {
  // volume_of: <id>: another plane of the volume that node draws, through its transfer function.
  // It follows that node's volume: a plane whose source is still loading, or changes, is followed.
  if (n.props.has("volume_of")) {
    const std::string other = n.props.str("volume_of");
    const std::shared_ptr<VolumeSliceNode> from = shared_slice(sg, out, other);
    if (!from) {
      warn(warnings, "ari: scene node '" + n.id + "': volume_of: '" + other +
                         "' names no volume_slice node realized before it");
      return nullptr;
    }
    std::shared_ptr<VolumeSliceNode> vn = make_slice(sg, n, parent, warnings);
    if (!vn)
      return nullptr;
    if (n.props.has("transfer_function"))
      warn(warnings, "ari: scene node '" + n.id + "': transfer_function: is " + other +
                         "'s (volume_of: " + other + ") -- ignored");
    vn->setModelKey(from->modelKey().empty() ? from->stateName("transfer_function")
                                             : from->modelKey());
    if (from->source())
      vn->setSource(from->source());
    std::weak_ptr<VolumeSliceNode> weak = vn;
    std::weak_ptr<VolumeSliceNode> shown = from;
    out.custom_ticks.push_back([weak, shown](vtkRenderer *) {
      const std::shared_ptr<VolumeSliceNode> node = weak.lock();
      if (!node)
        return;
      if (const std::shared_ptr<VolumeSliceNode> f = shown.lock())
        if (f->source() && f->source() != node->source())
          node->setSource(f->source());
      node->tick();
    });
    return vn;
  }
  if (const NodeDataSource followed = volume_data_source(sg, out, n))
    return realize_followed_slice(sg, n, parent, out, followed, warnings);
  if (n.source_file.empty() && n.source_sdf_mesh.empty()) {
    warn(warnings, "ari: scene node '" + n.id +
                       "' (volume_slice) has no source (data, state, node, file or sdf)");
    return nullptr;
  }
  cvc::app &app = sg.appContext();
  const std::string region = n.props.str("region_reads", "auto");
  if (region != "auto" && region != "true" && region != "false")
    warn(warnings, "ari: scene node '" + n.id + "': region_reads '" + region +
                       "' is not auto, true or false -- using auto");
  std::shared_ptr<cvc::slice_source> src;
  std::shared_ptr<cvc::volume> whole;
  try {
    if (!n.source_file.empty() && region != "false") {
      const ari::ResolvedFile f = ari::resolve_to_file(app, n.source_file);
      if (!f.ok) {
        warn(warnings, "ari: scene node '" + n.id + "': " + f.error);
        return nullptr;
      }
      bool by_plane = region == "true";
      if (!by_plane) {
        const cvc::volume_file_info info(app, f.path);
        by_plane = static_cast<double>(info.XDim()) * static_cast<double>(info.YDim()) *
                       static_cast<double>(info.ZDim()) >
                   kRegionVoxels;
      }
      if (by_plane)
        src = std::make_shared<cvc::slice_source>(app, f.path);
    }
    if (!src) {
      whole = std::make_shared<cvc::volume>(load_node_volume(sg, n, warnings));
      src = std::make_shared<cvc::slice_source>(*whole);
    }
  } catch (const std::exception &e) {
    warn(warnings, "ari: scene node '" + n.id + "': " + e.what());
    return nullptr;
  }

  GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
  if (!parent && sg.hasGraphics(n.id)) // last-wins parity with sg.addGraphics (see volren)
    sg.removeGraphics(n.id);
  std::shared_ptr<VolumeSliceNode> vn = pr->addGraphicsChild<VolumeSliceNode>(n.id);
  if (!vn)
    return nullptr;
  if (!parent)
    sg.registerGraphics(n.id, vn);
  seed_keys(*vn, n, warnings);

  // The transfer function's binding: the volume (or, read plane by plane, a stand-in holding the
  // first plane shown and the file's range) gives the range and the histogram.
  cvc::volume bound(app);
  if (whole) {
    bound = *whole;
  } else {
    cvc::slice_axis a = cvc::slice_axis::z;
    cvc::parse_slice_axis(vn->getState("slice.axis").value(), a);
    const std::uint64_t count = src->grid().count(a);
    std::uint64_t idx = count / 2;
    try {
      const long long want = std::stoll(vn->getState("slice.index").value());
      if (want >= 0)
        idx = std::min<std::uint64_t>(static_cast<std::uint64_t>(want), count - 1);
    } catch (...) {
    }
    try {
      const std::shared_ptr<const cvc::volume_slice_data> s = src->slice(a, idx);
      bound = cvc::volume(app, cvc::dimension(s->width, s->height, 1), cvc::Double);
      for (std::uint64_t r = 0; r < s->height; ++r)
        for (std::uint64_t c = 0; c < s->width; ++c)
          bound(c, r, 0, s->at(c, r));
    } catch (const std::exception &e) {
      warn(warnings, "ari: scene node '" + n.id + "': " + e.what());
    }
    bound.min(src->min());
    bound.max(src->max());
  }
  tf_binding_options opt;
  opt.seed = transfer_function_prop(n, warnings);
  attach_transfer_function(out, vn, tf_target::volume_slice, bound, opt, warnings);

  vn->setSource(src); // cuts and colours the first plane now
  std::weak_ptr<VolumeSliceNode> weak = vn;
  out.custom_ticks.push_back([weak](vtkRenderer *) {
    if (std::shared_ptr<VolumeSliceNode> node = weak.lock())
      node->tick();
  });
  return vn;
}

} // namespace

cvc::tf::model transfer_function_prop(const ari::SceneNode &n, std::vector<std::string> *warnings) {
  const ari::Value *t = n.props.find("transfer_function");
  cvc::tf::model m = cvc::tf::model::preset("grayscale");
  if (!t)
    return m;
  if (!t->is_map()) {
    warn(warnings, "ari: scene node '" + n.id + "': transfer_function: must be a map -- ignored");
    return m;
  }
  if (t->has("preset"))
    m = cvc::tf::model::preset(t->str("preset"));
  ari::Value rest;
  rest.kind = ari::Value::Kind::Map;
  for (const auto &e : t->entries)
    if (e.first != "preset")
      rest.entries.push_back(e);
  if (rest.entries.empty())
    return m;
  std::string json;
  to_json(rest, json);
  cvc::tf::model given;
  std::string err;
  if (!cvc::tf::from_string(json, given, &err)) {
    warn(warnings, "ari: scene node '" + n.id + "': transfer_function: " + err);
    return m;
  }
  if (rest.has("color"))
    m.color = given.color;
  if (rest.has("alpha"))
    m.alpha = given.alpha;
  if (rest.has("isocontours"))
    m.isocontours = given.isocontours;
  if (rest.has("domain"))
    m.domain = given.domain;
  if (rest.has("opacity_cubed"))
    m.opacity_cubed = given.opacity_cubed;
  if (rest.has("resolution"))
    m.resolution = given.resolution;
  m.normalize();
  return m;
}

void register_volume_slice_node_type(cvc::app &app) {
  NodeTypeOptions options;
  options.reads_data = true; // source: { data } is the volume the plane cuts, followed
  register_scene_node_type(app, "volume_slice", realize_volume_slice, options);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
