/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The `volume_grid` scene node type. See inc/cvc/gl/ariadne/volume_grid_nodes.h.

#include <algorithm>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/tf_binding.h>
#include <cvc/gl/ariadne/volume_grid_nodes.h>
#include <cvc/gl/ariadne/volume_slice_nodes.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume_file_info.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/volume/volume_slice.h>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

namespace {

// A file larger than this many voxels is read one plane at a time under region_reads: auto, as a
// volume_slice node's is.
constexpr double kRegionVoxels = 67108864.0; // 64 Mi

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

// A file read for the grid: whole, or (region_reads: true, or auto past kRegionVoxels) plane by
// plane. A relative path resolves against `base_dir`, then the working directory.
ari::ResolvedFile resolve(cvc::app &app, const std::string &file, const std::string &base_dir) {
  ari::ResolvedFile f = ari::resolve_to_file(app, file, base_dir);
  if (f.ok && !base_dir.empty()) {
    std::error_code ec;
    if (!std::filesystem::exists(f.path, ec)) {
      ari::ResolvedFile cwd = ari::resolve_to_file(app, file, std::string());
      if (cwd.ok && std::filesystem::exists(cwd.path, ec))
        return cwd;
    }
  }
  return f;
}

std::shared_ptr<cvc::slice_source> read_source(cvc::app &app, const std::string &file,
                                               const std::string &base_dir, int var, int time,
                                               const std::string &region) {
  const ari::ResolvedFile f = resolve(app, file, base_dir);
  if (!f.ok)
    throw std::runtime_error(f.error);
  bool by_plane = region == "true";
  if (region == "auto") {
    const cvc::volume_file_info info(app, f.path);
    by_plane = static_cast<double>(info.XDim()) * static_cast<double>(info.YDim()) *
                   static_cast<double>(info.ZDim()) >
               kRegionVoxels;
  }
  if (by_plane)
    return std::make_shared<cvc::slice_source>(app, f.path, static_cast<unsigned>(var),
                                               static_cast<unsigned>(time));
  cvc::volume vol(app);
  cvc::readVolumeFile(app, vol, f.path, static_cast<unsigned>(var), static_cast<unsigned>(time));
  return std::make_shared<cvc::slice_source>(vol);
}

// The props that seed <node>.grid.*.
void seed_keys(VolumeGridNode &g, const ari::SceneNode &n, std::vector<std::string> *warnings) {
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
  seed(grid("source.file"), n.source_file);
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
  if (n.props.has("points"))
    g.getState("grid.request").value("load_classes " + n.props.str("points"));
}

std::shared_ptr<GraphicsNode> realize_volume_grid(SceneGraph &sg, const ari::SceneNode &n,
                                                  GraphicsNode *parent, RealizedScene &out,
                                                  std::vector<std::string> *warnings) {
  if (n.source_file.empty() && n.source_sdf_mesh.empty()) {
    warn(warnings, "ari: scene node '" + n.id + "' (volume_grid) has no source (file or sdf)");
    return nullptr;
  }
  cvc::app &app = sg.appContext();
  const std::string region = n.props.str("region_reads", "auto");
  if (region != "auto" && region != "true" && region != "false")
    warn(warnings, "ari: scene node '" + n.id + "': region_reads '" + region +
                       "' is not auto, true or false -- using auto");
  const std::string reads = region == "true" || region == "false" ? region : "auto";
  const int var = std::max(0, static_cast<int>(n.props.num("var", 0)));
  const int time = std::max(0, static_cast<int>(n.props.num("time", 0)));
  std::shared_ptr<cvc::slice_source> src;
  try {
    if (!n.source_file.empty())
      src = read_source(app, n.source_file, n.base_dir, var, time, reads);
    else
      src = std::make_shared<cvc::slice_source>(load_node_volume(sg, n, warnings));
  } catch (const std::exception &e) {
    warn(warnings, "ari: scene node '" + n.id + "': " + e.what());
    return nullptr;
  }

  GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
  if (!parent && sg.hasGraphics(n.id)) // last-wins parity with sg.addGraphics (see volren)
    sg.removeGraphics(n.id);
  std::shared_ptr<VolumeGridNode> g = pr->addGraphicsChild<VolumeGridNode>(n.id);
  if (!g)
    return nullptr;
  if (!parent)
    sg.registerGraphics(n.id, g);
  g->setBaseDir(n.base_dir);
  seed_keys(*g, n, warnings);
  // `request: load` reads grid.source.* the way the document's source was read.
  cvc::app *ap = &app;
  const std::string base_dir = n.base_dir;
  g->setLoader([ap, base_dir, reads](const cvc::volume_grid_source_params &p, std::string &error) {
    try {
      return read_source(*ap, p.file, base_dir, p.var, p.time, reads);
    } catch (const std::exception &e) {
      error = e.what();
      return std::shared_ptr<cvc::slice_source>();
    }
  });
  g->setSource(src, n.source_file, var, time);

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

  // The transfer function, over the volume (or the stand-in plane of a file read plane by plane);
  // it follows a load.
  tf_binding_options opt;
  opt.seed = transfer_function_prop(n, warnings);
  std::weak_ptr<VolumeGridNode> weak = g;
  opt.volume_source = [weak]() -> std::shared_ptr<const cvc::volume> {
    const std::shared_ptr<VolumeGridNode> node = weak.lock();
    return node ? node->histogramVolume() : nullptr;
  };
  attach_transfer_function(out, g, tf_target::volume_slice, *g->histogramVolume(), opt, warnings);

  out.custom_ticks.push_back([weak](vtkRenderer *) {
    if (std::shared_ptr<VolumeGridNode> node = weak.lock())
      node->tick();
  });
  g->tick(); // the planes and the keys exist at once, so the scene frames on them
  return g;
}

} // namespace

void register_volume_grid_node_type(cvc::app &app) {
  register_scene_node_type(app, "volume_grid", realize_volume_grid);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
