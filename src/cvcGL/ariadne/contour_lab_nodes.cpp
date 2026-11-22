/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// contour_lab_nodes.cpp -- the contour_lab scene node type (see contour_lab_nodes.h): a contour
// stack read from a file or another node, drawn as polylines, and tiled into surfaces by
// cvc::tile_stepper as a sliced task on the app's scheduler.
//
// Threading, as fe_lab's (mesh_nodes.cpp): the tick runs on the thread that pumps the scheduler
// and does every state, scene and data-channel write. The job owns its stepper and a copy of the
// stack and touches nothing else; the tick polls it (progress, phase, the slice pair being tiled)
// and applies its result once it is done. Teardown cancels the task.

#include <algorithm>
#include <atomic>
#include <boost/any.hpp>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/job_keys.h>
#include <cvc/core/param_table.h>
#include <cvc/core/result_job.h>
#include <cvc/core/sliced_task.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/tiler.h>
#include <cvc/geometry/tiler_params.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/contour_lab_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/types.h>
#include <exception>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;
namespace se = cvc::state_exec;

namespace {

// --- small helpers -------------------------------------------------------------------------------

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

std::string num(double v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.6g", v);
  return buf;
}

// A component's name as one state key segment: '.' and white space become '_'.
std::string key_of(const std::string &name) {
  std::string k = name.empty() ? std::string("_") : name;
  for (char &c : k)
    if (c == '.' || std::isspace(static_cast<unsigned char>(c)))
      c = '_';
  return k;
}

// A name as a regular expression matching exactly it (Perl syntax, as tile_params' patterns).
std::string exact_pattern(const std::string &name) {
  std::string p;
  for (char c : name) {
    if (std::string("\\^$.|?*+()[]{}").find(c) != std::string::npos)
      p += '\\';
    p += c;
  }
  return p;
}

// "r,g,b", "r g b" or "[r, g, b]" (the colour widget's form, and a JSON array), each in [0, 1].
bool parse_rgb(const std::string &s, double out[3]) {
  std::string t = s;
  for (char &c : t)
    if (c == ',' || c == '[' || c == ']')
      c = ' ';
  std::istringstream is(t);
  double r = 0, g = 0, b = 0;
  if (!(is >> r >> g >> b) || !std::isfinite(r) || !std::isfinite(g) || !std::isfinite(b))
    return false;
  out[0] = std::clamp(r, 0.0, 1.0);
  out[1] = std::clamp(g, 0.0, 1.0);
  out[2] = std::clamp(b, 0.0, 1.0);
  return true;
}

std::string rgb_text(const double c[3]) { return num(c[0]) + "," + num(c[1]) + "," + num(c[2]); }

void json_string(const std::string &s, std::string &out) {
  out += '"';
  for (char c : s) {
    if (c == '"' || c == '\\')
      out += '\\';
    out += c;
  }
  out += '"';
}

// A prop's value as a parameter key's text: a scalar as it is, a sequence as a JSON array (numbers
// bare, the rest quoted), which param_table reads for a list field.
std::string prop_text(const ari::Value &v) {
  if (!v.is_seq())
    return v.as_string();
  std::string out = "[";
  for (std::size_t i = 0; i < v.items.size(); ++i) {
    out += i ? ", " : "";
    const std::string &s = v.items[i].scalar;
    char *end = nullptr;
    std::strtod(s.c_str(), &end);
    if (!s.empty() && end && *end == '\0')
      out += s;
    else
      json_string(s, out);
  }
  return out + "]";
}

// Seed each key a prop map gives (when the key has no value yet: an init: program or a restored
// session wins) under `base`, warning of a key the table has no field for.
template <class P>
void seed_param_props(cvc::app &a, const std::string &base, const ari::Value *props,
                      const std::string &where, std::vector<std::string> *warnings) {
  if (!props)
    return;
  if (!props->is_map()) {
    warn(warnings, where + ": must be a map -- ignored");
    return;
  }
  std::set<std::string> names;
  for (const cvc::param_field_info &f : cvc::param_field_infos<P>())
    names.insert(f.name);
  for (const auto &e : props->entries) {
    if (!names.count(e.first)) {
      warn(warnings, where + ": no parameter '" + e.first + "' -- ignored");
      continue;
    }
    const std::string path = ari::resolve_bind(base, e.first);
    if (ari::read_or<std::string>(a, path, "").empty())
      ari::write<std::string>(a, path, prop_text(e.second));
  }
}

std::string seed_str(cvc::app &a, const std::string &base, const char *key,
                     const std::string &def) {
  const std::string p = ari::resolve_bind(base, key);
  ari::read_or_seed<std::string>(a, p, def);
  return p;
}
std::string seed_num(cvc::app &a, const std::string &base, const char *key, double def) {
  const std::string p = ari::resolve_bind(base, key);
  ari::read_or_seed<double>(a, p, def);
  return p;
}
std::string seed_bool(cvc::app &a, const std::string &base, const char *key, bool def) {
  const std::string p = ari::resolve_bind(base, key);
  ari::read_bool_or_seed(a, p, def ? 1 : 0);
  return p;
}
bool read_bool(cvc::app &a, const std::string &path, bool def) {
  return ari::to_bool_int(ari::read_or<std::string>(a, path, def ? "1" : "0"), def ? 1 : 0) != 0;
}

// A request key holds one command or several separated by spaces ("load tile"); take the first.
std::string first_word(const std::string &s, std::string *rest) {
  const std::size_t b = s.find_first_not_of(" \t");
  if (b == std::string::npos) {
    rest->clear();
    return std::string();
  }
  const std::size_t e = s.find_first_of(" \t", b);
  const std::size_t r = e == std::string::npos ? e : s.find_first_not_of(" \t", e);
  *rest = r == std::string::npos ? std::string() : s.substr(r);
  return s.substr(b, e == std::string::npos ? e : e - b);
}

// The contour stack a data channel holds: the stack itself, a shared pointer to one, or a data
// object a program stored there (state-data-set) holding either. Null for anything else.
std::shared_ptr<const cvc::contour_stack> stack_of(const boost::any &a) {
  if (const auto *p = boost::any_cast<std::shared_ptr<const cvc::contour_stack>>(&a))
    return *p;
  if (const auto *p = boost::any_cast<std::shared_ptr<cvc::contour_stack>>(&a))
    return *p;
  if (const auto *p = boost::any_cast<cvc::contour_stack>(&a))
    return std::make_shared<const cvc::contour_stack>(*p);
  if (const auto *v = boost::any_cast<se::value_t>(&a))
    if (const auto *d = std::get_if<se::data_object_ptr>(&v->v))
      if (*d)
        return stack_of((*d)->payload);
  if (const auto *d = boost::any_cast<se::data_object_ptr>(&a))
    if (*d)
      return stack_of((*d)->payload);
  return nullptr;
}

// --- the job -------------------------------------------------------------------------------------

// The slice pair being tiled, which tile_stepper::label() names on the stepping thread: copied out
// after each step, so the tick reads it while a pool chunk runs.
struct TileLabel {
  void set(std::string s) {
    std::lock_guard<std::mutex> lock(m);
    text = std::move(s);
  }
  std::string get() const {
    std::lock_guard<std::mutex> lock(m);
    return text;
  }

private:
  mutable std::mutex m;
  std::string text;
};

// tile_stepper with its label published after each step (what result_job adapts).
struct LabelledTileStepper {
  cvc::tile_stepper s;
  std::shared_ptr<TileLabel> label;

  bool step(std::size_t budget) {
    const bool done = s.step(budget);
    label->set(done ? std::string() : s.label());
    return done;
  }
  double progress() const { return s.progress(); }
  const char *phase() const { return s.phase(); }
  std::string status() const { return s.status(); }
  bool done() const { return s.done(); }
  void cancel() { s.cancel(); }
  cvc::tile_result take() { return s.take(); }
};

// --- the node ------------------------------------------------------------------------------------

struct Component {
  std::string name, key;
  double color[3] = {1.0, 1.0, 1.0};
  std::string color_seen; // the row's colour text last applied
  int visible_seen = -1;
  std::weak_ptr<GeometryNode> contour, tile;
};

struct ContourLabRT {
  cvc::app *app = nullptr;
  SceneGraph *sg = nullptr;
  std::string id, base, base_dir, bind_prefix;
  std::weak_ptr<GraphicsNode> node, contours_group, tiles_group;
  std::map<std::string, ari::Value> component_props; // components: from the document, by name

  std::shared_ptr<const cvc::contour_stack> stack;
  std::vector<Component> comps;
  std::set<std::string> surface_keys; // surfaces.<key> written by the last tile

  // A node or state source: the data key followed, and when it last changed.
  std::string follow_path;
  boost::posix_time::ptime follow_seen;

  std::unique_ptr<cvc::job_keys> keys;
  std::shared_ptr<cvc::result_job<cvc::tile_result>> job;
  std::shared_ptr<TileLabel> label;
  std::vector<double> job_color; // tile_params::color of the running tile (empty: each row's)
  int task = -1;

  double line_width = 2.0;
  int view_contours_seen = -1, view_tiles_seen = -1;
  double opacity_seen = -1.0;

  ~ContourLabRT() {
    if (job)
      job->cancel();
    if (task >= 0 && app)
      cvc::cancel_sliced_job(*app, task);
  }

  std::string k(const char *key) const { return ari::resolve_bind(base, key); }
  std::string k(const std::string &key) const { return ari::resolve_bind(base, key); }
  void status(const std::string &s) { keys->write("status", s); }
};

// The data key a node or state source names (empty when it names none): `node` a scene node's id
// (or a state path, with a '.') and `key` the data key under it, by default the first of
// grid.contours and tiler.contours that holds data; `state` a bind path.
std::string source_data_path(ContourLabRT &rt, std::string *error) {
  cvc::app &a = *rt.app;
  const std::string node = ari::read_or<std::string>(a, rt.k("source.node"), "");
  const std::string key = ari::read_or<std::string>(a, rt.k("source.key"), "");
  const std::string state = ari::read_or<std::string>(a, rt.k("source.state"), "");
  if (!node.empty()) {
    if (node == rt.id) {
      *error = "source: { node: " + node + " } is this node";
      return std::string();
    }
    std::string at;
    if (node.find('.') != std::string::npos) {
      at = node;
    } else if (rt.sg) {
      if (std::shared_ptr<GraphicsNode> g = rt.sg->getGraphics(node))
        at = g->stateName();
    }
    if (at.empty()) {
      *error = "source: { node: " + node + " } names no scene node";
      return std::string();
    }
    if (!key.empty())
      return at + cvc::state::SEPARATOR + key;
    for (const char *k : {"grid.contours", "tiler.contours"}) {
      const std::string p = at + cvc::state::SEPARATOR + k;
      if (cvc::state *s = ari::read_node(a, p))
        if (!s->data().empty())
          return p;
    }
    return at + cvc::state::SEPARATOR + "tiler.contours"; // followed until data lands there
  }
  if (!state.empty())
    return ari::resolve_bind(rt.bind_prefix, state);
  return std::string();
}

void clear_surfaces(ContourLabRT &rt) {
  for (const std::string &key : rt.surface_keys)
    try {
      ari::write_node(*rt.app, rt.k("surfaces." + key)).data(boost::any());
    } catch (const std::exception &) {
    }
  rt.surface_keys.clear();
  if (std::shared_ptr<GraphicsNode> tg = rt.tiles_group.lock()) {
    const std::vector<std::shared_ptr<GraphicsNode>> kids = tg->getGraphicsChildren();
    for (const auto &c : kids)
      tg->removeGraphicsChild(c);
  }
  for (Component &c : rt.comps)
    c.tile.reset();
  ari::write<double>(*rt.app, rt.k("info.surfaces"), 0.0);
  ari::write<double>(*rt.app, rt.k("info.triangles"), 0.0);
}

// The component's polylines: every polygon of every slice, closed, at its slice's plane.
cvc::geometry contour_lines(const cvc::contour_stack &s, const std::string &name) {
  cvc::geometry g;
  cvc::geometry::points_t &pts = g.points();
  cvc::geometry::lines_t &lines = g.lines();
  for (const int idx : s.indices()) {
    const double z = s.slice_z(idx);
    for (const cvc::contour_polygon &p : s.polygons(idx, name)) {
      const std::size_t n = p.size();
      if (n < 2)
        continue;
      const std::size_t first = pts.size();
      for (std::size_t i = 0; i < n; ++i)
        pts.push_back({p.xy[2 * i], p.xy[2 * i + 1], z});
      for (std::size_t i = 0; i < n; ++i)
        lines.push_back({static_cast<cvc::geometry::index_t>(first + i),
                         static_cast<cvc::geometry::index_t>(first + (i + 1) % n)});
    }
  }
  return g;
}

// Take a new stack: its components' rows (a name already listed keeps its colour, visibility and
// tick; a new one takes the document's components: entry, else the stack's colour), its
// polylines, its info, and the data channel. The old surfaces go.
void take_stack(ContourLabRT &rt, std::shared_ptr<const cvc::contour_stack> stack) {
  cvc::app &a = *rt.app;
  struct Row {
    std::string color;
    std::string visible, tile;
  };
  std::map<std::string, Row> old;
  const int old_count = static_cast<int>(ari::read_or<double>(a, rt.k("components"), 0.0));
  for (int i = 0; i < old_count; ++i) {
    const std::string r = "components." + std::to_string(i) + ".";
    const std::string name = ari::read_or<std::string>(a, rt.k(r + "name"), "");
    if (!name.empty())
      old[name] = {ari::read_or<std::string>(a, rt.k(r + "color"), ""),
                   ari::read_or<std::string>(a, rt.k(r + "visible"), ""),
                   ari::read_or<std::string>(a, rt.k(r + "tile"), "")};
  }

  clear_surfaces(rt);
  rt.stack = std::move(stack);
  rt.comps.clear();
  std::shared_ptr<GraphicsNode> cg = rt.contours_group.lock();
  if (cg) {
    const std::vector<std::shared_ptr<GraphicsNode>> kids = cg->getGraphicsChildren();
    for (const auto &c : kids)
      cg->removeGraphicsChild(c);
  }

  const cvc::contour_stack &s = *rt.stack;
  for (std::size_t i = 0; i < s.components.size(); ++i) {
    const cvc::contour_component &cc = s.components[i];
    Component c;
    c.name = cc.name;
    c.key = key_of(cc.name);
    for (int j = 0; j < 3; ++j)
      c.color[j] = cc.color.size() == 3 ? std::clamp(cc.color[j], 0.0, 1.0) : 1.0;
    std::string visible = cc.visible ? "1" : "0", tile = "1";
    const auto prop = rt.component_props.find(cc.name);
    if (prop != rt.component_props.end()) {
      const ari::Value &p = prop->second;
      if (const ari::Value *col = p.find("color"))
        parse_rgb(prop_text(*col), c.color);
      if (p.has("visible"))
        visible = p.flag("visible") ? "1" : "0";
      if (p.has("tile"))
        tile = p.flag("tile") ? "1" : "0";
    }
    const auto was = old.find(cc.name);
    if (was != old.end()) {
      parse_rgb(was->second.color, c.color);
      if (!was->second.visible.empty())
        visible = was->second.visible;
      if (!was->second.tile.empty())
        tile = was->second.tile;
    }
    const std::string r = "components." + std::to_string(i) + ".";
    ari::write<std::string>(a, rt.k(r + "name"), c.name);
    ari::write<std::string>(a, rt.k(r + "color"), rgb_text(c.color));
    ari::write<std::string>(a, rt.k(r + "visible"), visible);
    ari::write<std::string>(a, rt.k(r + "tile"), tile);
    ari::write<std::string>(a, rt.k(r + "triangles"), "-");
    if (cg) {
      const cvc::geometry lines = contour_lines(s, cc.name);
      if (lines.num_points() > 0) {
        std::shared_ptr<GeometryNode> g = cg->createChild<GeometryNode>(c.key, lines);
        if (g) {
          g->setRenderMode(GeometryRenderMode::LINES);
          g->setUseSingleColor(true);
          g->setColor(c.color[0], c.color[1], c.color[2]);
          g->setLineWidth(rt.line_width);
          g->setRenderLinesAsTubes(rt.line_width > 1.0);
          g->setDepthOffset(4.0); // over the surfaces they bound
          c.contour = g;
        }
      }
    }
    c.color_seen = rgb_text(c.color);
    rt.comps.push_back(std::move(c));
  }
  ari::write<double>(a, rt.k("components"), static_cast<double>(s.components.size()));
  rt.view_contours_seen = rt.view_tiles_seen = -1; // apply visibility to the new nodes

  const std::vector<int> idx = s.indices();
  ari::write<double>(a, rt.k("info.slices"), static_cast<double>(idx.size()));
  ari::write<std::string>(a, rt.k("info.first"), idx.empty() ? "-" : std::to_string(idx.front()));
  ari::write<std::string>(a, rt.k("info.last"), idx.empty() ? "-" : std::to_string(idx.back()));
  ari::write<double>(a, rt.k("info.contours"), static_cast<double>(s.polygon_count()));
  ari::write<double>(a, rt.k("info.vertices"), static_cast<double>(s.vertex_count()));
  ari::write<double>(a, rt.k("info.components"), static_cast<double>(s.components.size()));
  ari::write<std::string>(a, rt.k("info.z_spacing"), num(s.z_spacing));
  try {
    ari::write_node(a, rt.k("contours")).data(boost::any(rt.stack));
  } catch (const std::exception &) {
  }
}

// Read the source the source.* keys name. True with the stack taken; false with the status saying
// why (the node keeps what it had).
bool load_source(ContourLabRT &rt) {
  cvc::app &a = *rt.app;
  const std::string file = ari::read_or<std::string>(a, rt.k("source.file"), "");
  std::string error;
  const std::string data_path = file.empty() ? source_data_path(rt, &error) : std::string();
  rt.follow_path = data_path;
  if (!error.empty()) {
    rt.status("error: " + error);
    return false;
  }
  std::shared_ptr<const cvc::contour_stack> stack;
  std::string from;
  if (!file.empty()) {
    const cvc::param_result<cvc::contour_read_params> rp =
        cvc::params_from_state<cvc::contour_read_params>(ari::write_node(a, rt.k("read")));
    if (!rp.ok()) {
      rt.status("error: read." + rp.message());
      return false;
    }
    // A relative path resolves against the document's directory, else the working directory.
    ari::ResolvedFile f = ari::resolve_to_file(a, file, rt.base_dir);
    std::error_code ec;
    if (f.ok && !rt.base_dir.empty() && !std::filesystem::exists(f.path, ec)) {
      ari::ResolvedFile cwd = ari::resolve_to_file(a, file, std::string());
      if (cwd.ok && std::filesystem::exists(cwd.path, ec))
        f = std::move(cwd);
    }
    if (!f.ok) {
      rt.status("error: " + f.error);
      return false;
    }
    try {
      stack = std::make_shared<const cvc::contour_stack>(cvc::read_contours(a, f.path, rp.value));
    } catch (const std::exception &e) {
      rt.status(std::string("error: ") + e.what());
      return false;
    }
    from = file;
  } else if (!data_path.empty()) {
    cvc::state *s = ari::read_node(a, data_path);
    if (s)
      rt.follow_seen = s->lastMod();
    const boost::any d = s ? s->data() : boost::any();
    if (d.empty()) {
      rt.status("waiting for contours at " + data_path);
      return false;
    }
    stack = stack_of(d);
    if (!stack) {
      rt.status("error: " + data_path + " holds no contour stack");
      return false;
    }
    from = data_path;
  } else {
    rt.status("no source: give source: { file }, { node } or { state }");
    return false;
  }
  const std::vector<std::string> problems = cvc::contour_problems(*stack);
  if (!problems.empty()) {
    rt.status("error: " + from + ": " + problems.front());
    return false;
  }
  take_stack(rt, std::move(stack));
  rt.status(num(static_cast<double>(rt.stack->polygon_count())) + " contours on " +
            num(static_cast<double>(rt.stack->indices().size())) + " slices, " +
            num(static_cast<double>(rt.stack->components.size())) + " components");
  return true;
}

void launch_tile(ContourLabRT &rt) {
  cvc::app &a = *rt.app;
  if (!rt.stack || rt.stack->empty()) {
    rt.status("no contours to tile: load a source first");
    return;
  }
  const cvc::param_result<cvc::tile_params> rp =
      cvc::params_from_state<cvc::tile_params>(ari::write_node(a, rt.base));
  if (!rp.ok()) {
    rt.status("error: " + rp.message());
    return;
  }
  cvc::tile_params p = rp.value;
  std::size_t tiled = 0;
  for (std::size_t i = 0; i < rt.comps.size(); ++i) {
    if (read_bool(a, rt.k("components." + std::to_string(i) + ".tile"), true))
      ++tiled;
    else
      p.exclude.push_back(exact_pattern(rt.comps[i].name));
  }
  if (tiled == 0) {
    rt.status("no component ticked to tile");
    return;
  }
  rt.label = std::make_shared<TileLabel>();
  std::unique_ptr<cvc::result_job<cvc::tile_result>> job =
      cvc::make_result_job(std::make_unique<LabelledTileStepper>(
          LabelledTileStepper{cvc::tile_stepper(*rt.stack, p), rt.label}));
  rt.job = std::move(job);
  rt.job_color = p.color;
  rt.keys->started("tiling...");
  ari::write<std::string>(a, rt.k("label"), "");
  cvc::sliced_task_options o;
  o.kind = "contour_lab";
  o.subject = "tile " + rt.id;
  o.owner = "contour_lab:" + rt.base;
  std::string why;
  rt.task = cvc::schedule_sliced_job(a, rt.job, o, &why);
  if (rt.task < 0) {
    rt.job.reset();
    rt.keys->finished("error: the job could not start: " + why);
  } else {
    rt.keys->pid(rt.task);
  }
}

// The tile's surfaces: a node per component under <id>_tiles in its row's colour (or the tile's
// one colour), on the data channel at surfaces.<key>, and the counts.
void apply_tile(ContourLabRT &rt, cvc::tile_result r) {
  cvc::app &a = *rt.app;
  clear_surfaces(rt);
  std::shared_ptr<GraphicsNode> tg = rt.tiles_group.lock();
  const double opacity = std::clamp(ari::read_or<double>(a, rt.k("view.opacity"), 1.0), 0.0, 1.0);
  std::size_t triangles = 0;
  for (std::size_t i = 0; i < rt.comps.size(); ++i) {
    Component &c = rt.comps[i];
    const std::string row = "components." + std::to_string(i) + ".triangles";
    const auto it = r.components.find(c.name);
    if (it == r.components.end()) {
      ari::write<std::string>(a, rt.k(row), "-");
      continue;
    }
    const cvc::geometry &g = it->second;
    const std::size_t n = g.num_tris();
    triangles += n;
    ari::write<double>(a, rt.k(row), static_cast<double>(n));
    try {
      ari::write_node(a, rt.k("surfaces." + c.key)).data(boost::any(g));
      rt.surface_keys.insert(c.key);
    } catch (const std::exception &) {
    }
    if (!tg || n == 0)
      continue;
    std::shared_ptr<GeometryNode> node = tg->createChild<GeometryNode>(c.key, g);
    if (!node)
      continue;
    node->setUseSingleColor(true);
    if (rt.job_color.size() == 3)
      node->setColor(rt.job_color[0], rt.job_color[1], rt.job_color[2]);
    else
      node->setColor(c.color[0], c.color[1], c.color[2]);
    node->setOpacity(opacity);
    c.tile = node;
  }
  rt.view_tiles_seen = -1;
  rt.opacity_seen = opacity;
  ari::write<double>(a, rt.k("info.surfaces"), static_cast<double>(rt.surface_keys.size()));
  ari::write<double>(a, rt.k("info.triangles"), static_cast<double>(triangles));
  const int old = static_cast<int>(ari::read_or<double>(a, rt.k("warnings"), 0.0));
  for (std::size_t i = 0; i < r.warnings.size(); ++i)
    ari::write<std::string>(a, rt.k("warnings." + std::to_string(i)), r.warnings[i]);
  for (int i = static_cast<int>(r.warnings.size()); i < old; ++i)
    ari::write<std::string>(a, rt.k("warnings." + std::to_string(i)), "");
  ari::write<double>(a, rt.k("warnings"), static_cast<double>(r.warnings.size()));
  std::string summary = "tiled " + std::to_string(rt.surface_keys.size()) +
                        " components: " + std::to_string(triangles) + " triangles";
  if (!r.warnings.empty())
    summary += " (" + std::to_string(r.warnings.size()) + " warnings)";
  rt.keys->finished(summary);
}

// Show or hide a child, through its own visible key (as mesh_nodes' show_child).
void show(cvc::app &a, GeometryNode *g, bool want) {
  if (!g)
    return;
  ari::write<int>(a, g->stateName("visible"), want ? 1 : 0);
  if (g->isVisible() != want)
    g->setVisible(want);
}

// Each row's colour and visibility, and the view keys, onto the nodes.
void sync_components(ContourLabRT &rt) {
  cvc::app &a = *rt.app;
  const int vc = read_bool(a, rt.k("view.contours"), true) ? 1 : 0;
  const int vt = read_bool(a, rt.k("view.tiles"), true) ? 1 : 0;
  const bool view_changed = vc != rt.view_contours_seen || vt != rt.view_tiles_seen;
  rt.view_contours_seen = vc;
  rt.view_tiles_seen = vt;
  const double opacity = std::clamp(ari::read_or<double>(a, rt.k("view.opacity"), 1.0), 0.0, 1.0);
  const bool opacity_changed = opacity != rt.opacity_seen;
  rt.opacity_seen = opacity;
  const double lw = std::clamp(ari::read_or<double>(a, rt.k("view.line_width"), 2.0), 0.5, 20.0);
  const bool lw_changed = lw != rt.line_width;
  rt.line_width = lw;
  for (std::size_t i = 0; i < rt.comps.size(); ++i) {
    Component &c = rt.comps[i];
    const std::string r = "components." + std::to_string(i) + ".";
    std::shared_ptr<GeometryNode> cn = c.contour.lock(), tn = c.tile.lock();
    const std::string col = ari::read_or<std::string>(a, rt.k(r + "color"), c.color_seen);
    if (col != c.color_seen) {
      c.color_seen = col;
      if (parse_rgb(col, c.color)) {
        if (cn)
          cn->setColor(c.color[0], c.color[1], c.color[2]);
        if (tn && rt.job_color.size() != 3)
          tn->setColor(c.color[0], c.color[1], c.color[2]);
      }
    }
    const int vis = read_bool(a, rt.k(r + "visible"), true) ? 1 : 0;
    if (vis != c.visible_seen || view_changed) {
      c.visible_seen = vis;
      show(a, cn.get(), vis && vc);
      show(a, tn.get(), vis && vt);
    }
    if (tn && opacity_changed)
      tn->setOpacity(opacity);
    if (cn && lw_changed) {
      cn->setLineWidth(lw);
      cn->setRenderLinesAsTubes(lw > 1.0);
    }
  }
}

void tick_contour_lab(ContourLabRT &rt) {
  if (!rt.node.lock())
    return;
  cvc::app &a = *rt.app;
  const std::string request_key = rt.k("request");

  if (rt.job) {
    std::string rest;
    if (first_word(ari::read_or<std::string>(a, request_key, ""), &rest) == "stop") {
      ari::write<std::string>(a, request_key, rest); // what follows waits for the job
      rt.job->cancel();
      rt.keys->stopping();
    }
    const std::string label = rt.label ? rt.label->get() : std::string();
    const char *phase = rt.job->phase();
    const std::string text = label.empty() || !phase ? std::string(phase ? phase : "")
                                                     : std::string(phase) + " " + label;
    rt.keys->progress(static_cast<int>(std::lround(100.0 * rt.job->progress())), text.c_str());
    ari::write<std::string>(a, rt.k("label"), label);
    if (rt.job->done()) {
      std::shared_ptr<cvc::result_job<cvc::tile_result>> job = std::move(rt.job);
      rt.task = -1;
      ari::write<std::string>(a, rt.k("label"), "");
      // A failure or a stop keeps what the node had. done() is true at once after a cancel, while
      // a pool chunk may still be inside a step, so a stop is reported without asking the job.
      if (job->failed())
        rt.keys->finished(*job);
      else if (job->has_result())
        apply_tile(rt, job->take_result());
      else
        rt.keys->finished(std::string("cancelled"));
    }
  } else {
    std::string rest;
    const std::string req = first_word(ari::read_or<std::string>(a, request_key, ""), &rest);
    if (!req.empty()) {
      ari::write<std::string>(a, request_key, rest);
      if (req == "load") {
        if (load_source(rt) && read_bool(a, rt.k("auto"), false))
          launch_tile(rt);
      } else if (req == "tile") {
        launch_tile(rt);
      } else if (req == "stop") {
        rt.status("nothing to stop");
      } else {
        rt.status("unknown request '" + req + "' (load, tile or stop)");
      }
    } else if (!rt.follow_path.empty()) {
      // A node or state source: new contours there load (and tile, with auto) again.
      cvc::state *s = ari::read_node(a, rt.follow_path);
      if (s && s->lastMod() != rt.follow_seen && !s->data().empty()) {
        if (load_source(rt) && read_bool(a, rt.k("auto"), false))
          launch_tile(rt);
      }
    }
  }
  sync_components(rt);
}

std::shared_ptr<GraphicsNode> realize_contour_lab(SceneGraph &sg, const ari::SceneNode &n,
                                                  GraphicsNode *parent, RealizedScene &out,
                                                  std::vector<std::string> *warnings) {
  cvc::app &app = sg.appContext();
  const std::string where = "ari: scene node '" + n.id + "' (contour_lab)";
  GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
  if (!parent && sg.hasGraphics(n.id)) // last wins, as the other node types
    sg.removeGraphics(n.id);
  std::shared_ptr<GeometryNode> g = pr->addGraphicsChild<GeometryNode>(n.id);
  if (!g)
    return nullptr;
  if (!parent)
    sg.registerGraphics(n.id, g);
  std::shared_ptr<GraphicsNode> cg = g->createChild(n.id + "_contours");
  std::shared_ptr<GraphicsNode> tg = g->createChild(n.id + "_tiles");
  if (cg)
    out.nodes.push_back({n.id + "_contours", {"contours"}, cg});
  if (tg)
    out.nodes.push_back({n.id + "_tiles", {"tiles"}, tg});

  auto rt = std::make_shared<ContourLabRT>();
  rt->app = &app;
  rt->sg = &sg;
  rt->id = n.id;
  rt->base = g->stateName("tiler");
  rt->base_dir = n.base_dir;
  rt->bind_prefix = out.bind_prefix;
  rt->node = g;
  rt->contours_group = cg;
  rt->tiles_group = tg;
  rt->keys = std::make_unique<cvc::job_keys>(cvc::state::instance(app), rt->base);

  const ari::Value &pr_ = n.props;
  std::string src_node, src_key, src_state;
  if (const ari::Value *src = pr_.find("source"); src && src->is_map()) {
    src_node = src->str("node");
    src_key = src->str("key");
    src_state = src->str("state");
  }
  seed_str(app, rt->base, "source.file", n.source_file);
  seed_str(app, rt->base, "source.node", src_node);
  seed_str(app, rt->base, "source.key", src_key);
  seed_str(app, rt->base, "source.state", src_state);
  seed_bool(app, rt->base, "auto", pr_.flag("auto_tile"));
  seed_str(app, rt->base, "request", "");
  seed_bool(app, rt->base, "view.contours", pr_.flag("show_contours", true));
  seed_bool(app, rt->base, "view.tiles", pr_.flag("show_tiles", true));
  seed_num(app, rt->base, "view.opacity", pr_.num("opacity", 1.0));
  seed_num(app, rt->base, "view.line_width", pr_.num("line_width", 2.0));
  rt->line_width = std::clamp(ari::read_or<double>(app, rt->k("view.line_width"), 2.0), 0.5, 20.0);
  seed_param_props<cvc::contour_read_params>(app, rt->k("read"), pr_.find("read"), where + ": read",
                                             warnings);
  seed_param_props<cvc::tile_params>(app, rt->base, pr_.find("tiler"), where + ": tiler", warnings);
  // Every parameter key seeded (its default where nothing set it), and each described: its unit,
  // range, doc and default.
  (void)cvc::params_from_state<cvc::contour_read_params>(ari::write_node(app, rt->k("read")));
  (void)cvc::params_from_state<cvc::tile_params>(ari::write_node(app, rt->base));
  cvc::params_describe_state<cvc::contour_read_params>(ari::write_node(app, rt->k("read")));
  cvc::params_describe_state<cvc::tile_params>(ari::write_node(app, rt->base));
  if (const ari::Value *cp = pr_.find("components")) {
    if (cp->is_map())
      for (const auto &e : cp->entries)
        rt->component_props[e.first] = e.second;
    else
      warn(warnings, where + ": components: must be a map of component names -- ignored");
  }
  rt->keys->idle();
  ari::write<double>(app, rt->k("warnings"), 0.0);
  ari::write<std::string>(app, rt->k("label"), "");
  ari::write<double>(app, rt->k("info.surfaces"), 0.0);
  ari::write<double>(app, rt->k("info.triangles"), 0.0);

  // The contours load now, so the scene has them when the host frames it; a node or state source
  // whose data is not there yet is followed until it is. Tiling waits for the first tick. Rows a
  // realize before this one left (a reloaded document) keep their edits for the names that are
  // still there; with no contours there are no rows.
  if (load_source(*rt)) {
    if (read_bool(app, rt->k("auto"), false))
      ari::write<std::string>(app, rt->k("request"), "tile");
  } else {
    ari::write<double>(app, rt->k("components"), 0.0);
    if (rt->follow_path.empty())
      warn(warnings, where + ": " + ari::read_or<std::string>(app, rt->k("status"), ""));
  }

  out.custom_ticks.push_back([rt](vtkRenderer *) { tick_contour_lab(*rt); });
  std::weak_ptr<ContourLabRT> weak = rt;
  out.pending_work.push_back([weak]() {
    const std::shared_ptr<ContourLabRT> r = weak.lock();
    return r && r->node.lock() &&
           (r->job || !ari::read_or<std::string>(*r->app, r->k("request"), "").empty());
  });
  return g;
}

} // namespace

void register_contour_lab_node_type(cvc::app &app) {
  register_scene_node_type(app, "contour_lab", realize_contour_lab);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
