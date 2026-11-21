/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeGridNode. See inc/cvc/gl/VolumeGridNode.h.

#include <algorithm>
#include <boost/any.hpp>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/job_keys.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/state/state.h>
#include <cvc/tf/model.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <filesystem>
#include <sstream>

namespace cvc {
namespace gl {

namespace {

std::string num(double d) { return cvc::param_detail::format_double(d); }

void put(cvc::state &s, const std::string &v) {
  if (s.value() != v)
    s.value(v);
}

bool starts_with(const std::string &s, const char *p) { return s.rfind(p, 0) == 0; }

std::string trim(const std::string &s) {
  const std::size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
    return std::string();
  return s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
}

// A crosshair's colour: the axis it runs along, x red, y green, z blue (as the axis glyph). Where
// two planes cross, each draws the line, and in the 3D view the two are one line in one colour.
cvc::geometry::color_t axis_color(int a) {
  switch (a) {
  case 0:
    return {{1.0, 0.35, 0.35}};
  case 1:
    return {{0.35, 1.0, 0.35}};
  default:
    return {{0.45, 0.65, 1.0}};
  }
}

constexpr int kDiskSides = 16;
constexpr double kPi = 3.14159265358979323846;

// The point in object space at plane coordinates (r, u) on the plane `plane` along axis `a`.
cvc::geometry::point_t at(cvc::slice_axis a, double plane, double r, double u) {
  const std::pair<int, int> ru = cvc::grid_plane_axes(a);
  cvc::geometry::point_t p{{0.0, 0.0, 0.0}};
  p[static_cast<int>(a)] = plane;
  p[ru.first] = r;
  p[ru.second] = u;
  return p;
}

} // namespace

VolumeGridNode::VolumeGridNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
    : NullGraphicNode(ctx, statePath, name) {
  cvc::params_describe_state<cvc::volume_grid_params>(getState("grid"));
  cvc::params_describe_state<cvc::volume_grid_display_params>(getState("grid.display"));
  cvc::params_describe_state<cvc::volume_grid_source_params>(getState("grid.source"));
}

VolumeGridNode::~VolumeGridNode() = default;

const char *VolumeGridNode::sliceName(cvc::slice_axis a) {
  switch (a) {
  case cvc::slice_axis::x:
    return "slice_zy";
  case cvc::slice_axis::y:
    return "slice_xz";
  case cvc::slice_axis::z:
    return "slice_xy";
  }
  return "slice_xy";
}

std::shared_ptr<VolumeSliceNode> VolumeGridNode::slice(cvc::slice_axis a) const {
  return m_slices[static_cast<int>(a)];
}

std::shared_ptr<GeometryNode> VolumeGridNode::pointsOverlay(cvc::slice_axis a) const {
  return m_points[static_cast<int>(a)];
}

std::shared_ptr<GeometryNode> VolumeGridNode::linesOverlay(cvc::slice_axis a) const {
  return m_lines[static_cast<int>(a)];
}

void VolumeGridNode::ensureChildren() {
  if (m_slices[0])
    return;
  const std::string tf = stateName("transfer_function");
  for (cvc::slice_axis a : {cvc::slice_axis::z, cvc::slice_axis::y, cvc::slice_axis::x}) {
    const int ax = static_cast<int>(a);
    std::shared_ptr<VolumeSliceNode> s = addGraphicsChild<VolumeSliceNode>(sliceName(a));
    s->getState("slice.axis").value(std::string(cvc::slice_axis_name(a)));
    s->setModelKey(tf);
    m_slices[ax] = s;
    for (int k = 0; k < 2; ++k) {
      std::shared_ptr<GeometryNode> g = s->addGraphicsChild<GeometryNode>(k ? "lines" : "points");
      g->setUseSingleColor(false);
      g->setAmbient(1.0); // unlit: the colours are the colours
      g->setDiffuse(0.0);
      g->setSpecular(0.0);
      if (k) {
        g->setRenderMode(GeometryRenderMode::LINES); // and again after each setGeometry
        g->setLineWidth(2.0);
      }
      g->setVisible(false); // until it has something to draw
      (k ? m_lines : m_points)[ax] = g;
    }
  }
  m_surfaces = addGraphicsChild<NullGraphicNode>("surfaces");
  // The grey display's model: the planes colour through it instead of the transfer function.
  getState("grey_transfer_function").value(cvc::tf::to_string(cvc::tf::model::preset("grayscale")));
  cvc::job_keys(app().root(), getState("grid").fullName()).idle();
  // The probe's keys exist from the start (empty: no voxel yet), so what binds them reads text.
  for (const char *k : {"i", "j", "k", "x", "y", "z", "value"})
    put(getState(std::string("grid.cursor.") + k), std::string());
  put(getState("grid.cursor.inside"), "0");
  put(getState("grid.selection.count"), "0");
}

void VolumeGridNode::setSource(std::shared_ptr<cvc::slice_source> src, const std::string &file,
                               int var, int time) {
  if (!src)
    return;
  ensureChildren();
  m_source = std::move(src);
  m_file = file;
  m_var = var;
  m_time = time;
  const cvc::slice_grid &g = m_source->grid();
  m_editor.set_dims(
      {static_cast<int>(g.dim.xdim), static_cast<int>(g.dim.ydim), static_cast<int>(g.dim.zdim)});
  m_editor.set_shown(var, time);
  m_editor.clear_selection();
  m_gestures.reset();
  m_band = cvc::grid_pick_outcome{};

  // The volume the transfer function's histogram is taken over.
  if (const cvc::volume *v = m_source->in_memory_volume()) {
    m_histogramVolume = std::shared_ptr<const cvc::volume>(m_source, v); // kept by the source
    getState("grid.volume").data(boost::any(m_histogramVolume));
  } else {
    auto stand_in = std::make_shared<cvc::volume>(app());
    try {
      const std::shared_ptr<const cvc::volume_slice_data> s =
          m_source->slice(cvc::slice_axis::z, g.count(cvc::slice_axis::z) / 2);
      *stand_in = cvc::volume(app(), cvc::dimension(s->width, s->height, 1), cvc::Double);
      for (std::uint64_t r = 0; r < s->height; ++r)
        for (std::uint64_t c = 0; c < s->width; ++c)
          (*stand_in)(c, r, 0, s->at(c, r));
    } catch (const std::exception &e) {
      status(std::string("error: ") + e.what());
    }
    stand_in->min(m_source->min());
    stand_in->max(m_source->max());
    m_histogramVolume = stand_in;
    getState("grid.volume").data(boost::any());
  }
  for (const auto &s : m_slices)
    s->setSource(m_source);
  publishInfo();
  // Points the new volume doesn't hold are kept, and said.
  const std::vector<int> dims{static_cast<int>(g.dim.xdim), static_cast<int>(g.dim.ydim),
                              static_cast<int>(g.dim.zdim)};
  const std::vector<std::string> problems = cvc::point_class_problems(m_editor.classes(), dims);
  if (!problems.empty())
    status("note: " + problems.front());
  ++m_keysVersion; // the planes are placed again on the next tick
  m_overlaySignature.clear();
}

void VolumeGridNode::publishInfo() {
  const cvc::slice_grid &g = m_source->grid();
  put(getState("grid.info.dims"), std::to_string(g.dim.xdim) + " " + std::to_string(g.dim.ydim) +
                                      " " + std::to_string(g.dim.zdim));
  put(getState("grid.info.min"), num(m_source->min()));
  put(getState("grid.info.max"), num(m_source->max()));
  put(getState("grid.info.span"),
      num(g.spacing(0)) + " " + num(g.spacing(1)) + " " + num(g.spacing(2)));
  put(getState("grid.info.file"), m_file);
}

void VolumeGridNode::status(const std::string &s) { put(getState("grid.status"), s); }

void VolumeGridNode::handleStateChanged(const std::string &child) {
  if (starts_with(child, "grid.")) {
    const std::string k = child.substr(5);
    bool read = true; // false: a key the node publishes, or one it doesn't read
    if (k == "pick.seq")
      ++m_pickVersion;
    else if (k == "pointer.seq")
      ++m_pointerVersion;
    else if (k == "request")
      ++m_requestVersion;
    else if (k == "classes" || (starts_with(k, "classes.") && k != "classes.current" &&
                                !(k.size() > 7 && k.compare(k.size() - 7, 7, ".points") == 0)))
      ++m_rowsVersion;
    else if (k == "view" || k == "tool" || k == "classes.current" || starts_with(k, "slice.") ||
             starts_with(k, "display.") || starts_with(k, "source."))
      ++m_keysVersion;
    else
      read = false;
    if (read)
      if (SceneGraph *sg = getSceneGraph())
        sg->requestRender();
  }
  NullGraphicNode::handleStateChanged(child);
}

bool VolumeGridNode::voxelOf(const double world[3], std::array<std::int64_t, 3> &ijk) const {
  double local[3];
  worldToLocal(world, local);
  std::int64_t v[3];
  const bool inside = m_source->grid().voxel_of(local, v);
  for (int c = 0; c < 3; ++c)
    ijk[c] = v[c];
  return inside;
}

std::string VolumeGridNode::resolvePath(const std::string &file) const {
  namespace fs = std::filesystem;
  fs::path p(file);
  if (p.is_relative() && !m_baseDir.empty())
    p = fs::path(m_baseDir) / p;
  return p.string();
}

void VolumeGridNode::readKeys() {
  cvc::state &grid = getState("grid");
  const cvc::param_result<cvc::volume_grid_params> p =
      cvc::params_from_state<cvc::volume_grid_params>(grid, m_params);
  const cvc::param_result<cvc::volume_grid_display_params> d =
      cvc::params_from_state<cvc::volume_grid_display_params>(grid("display"), m_display);
  std::string problems;
  if (!p.ok())
    problems = p.message();
  else
    m_params = p.value;
  if (!d.ok())
    problems += (problems.empty() ? "" : "; ") + d.message();
  else
    m_display = d.value;
  cvc::params_from_state<cvc::volume_grid_source_params>(grid("source")); // seeded, read by load
  if (!problems.empty())
    status("error: " + problems);
  // The grey display: every plane colours through the grey model, else the transfer function.
  if (m_display.grey != m_grey) {
    m_grey = m_display.grey;
    const std::string key = stateName(m_grey ? "grey_transfer_function" : "transfer_function");
    for (const auto &s : m_slices)
      s->setModelKey(key);
  }
  syncSlices();
  fixCurrent();
}

void VolumeGridNode::syncSlices() {
  const cvc::slice_grid &g = m_source->grid();
  cvc::state &grid = getState("grid");
  const std::int64_t want[3] = {m_params.slice_x, m_params.slice_y, m_params.slice_z};
  for (int a = 0; a < 3; ++a) {
    const std::int64_t last = static_cast<std::int64_t>(g.count(a)) - 1;
    const std::int64_t idx = want[a] < 0 ? last / 2 : std::min(want[a], last);
    const std::string key = std::string("slice.") + "xyz"[a];
    put(grid(key), std::to_string(idx)); // -1 resolved, or clamped to the last plane
    // A slider with range: key spans the planes (the table's min, -1, is the middle's alias).
    if (grid(key).property("min") != "0")
      grid(key).property("min", "0");
    if (grid(key).property("max") != std::to_string(last))
      grid(key).property("max", std::to_string(last));
    put(m_slices[a]->getState("slice.index"), std::to_string(idx));
    m_index[a] = idx;
  }
  m_params.slice_x = m_index[0];
  m_params.slice_y = m_index[1];
  m_params.slice_z = m_index[2];
}

void VolumeGridNode::fixCurrent() {
  // The class a click adds to: kept while it names a class, else the first (or none).
  cvc::state &cur = getState("grid.classes.current");
  const std::size_t n = m_editor.classes().classes.size();
  long long c = -2;
  const std::string text = cur.value();
  if (!text.empty() && !cvc::param_detail::parse_integer(text, c))
    c = -2;
  if (text == m_autoCurrent && c == -1 && n > 0) {
    c = -2; // the -1 this node wrote while there were no classes, not someone's choice
  } else if (c == -1) {
    return; // none, on purpose
  }
  if (c < 0 || c >= static_cast<long long>(n)) {
    m_autoCurrent = n ? "0" : "-1";
    put(cur, m_autoCurrent);
  }
}

void VolumeGridNode::publishPoints(bool rows) {
  m_published = std::make_shared<const cvc::point_classes>(m_editor.classes());
  getState("grid.points").data(boost::any(m_published));
  if (rows) {
    cvc::write_point_class_rows(getState("grid.classes"), m_editor.classes());
    m_rowsSeen = m_rowsVersion; // ... and these
  }
  m_pointsVersionShown = m_editor.version();
  fixCurrent();
}

void VolumeGridNode::publishSelection() {
  put(getState("grid.selection.count"), std::to_string(m_editor.selection().size()));
  getState("grid.selection")
      .data(boost::any(std::make_shared<const cvc::point_classes>(m_editor.selection_classes())));
  m_selectionVersionShown = m_editor.selection_version();
}

void VolumeGridNode::readRows() {
  std::vector<std::string> problems;
  const std::vector<cvc::point_class> rows =
      cvc::read_point_class_rows(getState("grid.classes"), m_var, m_time, &problems);
  const cvc::point_classes next = cvc::reconcile_point_class_rows(m_editor.classes(), rows);
  if (next != m_editor.classes()) {
    m_editor.set_classes(next); // a row renamed, restyled, added or removed: an undo step
    publishPoints(true);
  }
  if (!problems.empty())
    status("error: " + problems.front());
}

void VolumeGridNode::adoptData() {
  const boost::any d = getState("grid.points").data();
  std::shared_ptr<const cvc::point_classes> p;
  if (const auto *c = boost::any_cast<std::shared_ptr<const cvc::point_classes>>(&d))
    p = *c;
  else if (const auto *m = boost::any_cast<std::shared_ptr<cvc::point_classes>>(&d))
    p = *m;
  else if (const auto *v = boost::any_cast<cvc::point_classes>(&d))
    p = std::make_shared<const cvc::point_classes>(*v);
  if (!p || p == m_published)
    return;
  m_editor.set_classes(*p);
  publishPoints(true);
  const cvc::slice_grid &g = m_source->grid();
  const std::vector<std::string> problems = cvc::point_class_problems(
      m_editor.classes(),
      {static_cast<int>(g.dim.xdim), static_cast<int>(g.dim.ydim), static_cast<int>(g.dim.zdim)});
  status(problems.empty()
             ? "classes set (" + std::to_string(m_editor.classes().point_count()) + " points)"
             : "note: " + problems.front());
}

void VolumeGridNode::cursorFrom(const std::string &base) {
  cvc::state &from = getState(base);
  cvc::state &to = getState("grid.cursor");
  for (const char *k : {"i", "j", "k", "x", "y", "z", "value"}) {
    cvc::state *s = from.findDescendant(k);
    put(to(k), s ? s->value() : std::string());
  }
  cvc::state *i = from.findDescendant("i");
  put(to("inside"), i && !i->value().empty() ? "1" : "0");
}

void VolumeGridNode::consumePointer() {
  cvc::state *seq = getState("grid.pointer").findDescendant("seq");
  if (!seq || seq->value() == m_lastPointerSeq)
    return;
  m_lastPointerSeq = seq->value();
  cursorFrom("grid.pointer");
}

void VolumeGridNode::consumePick() {
  cvc::state &pk = getState("grid.pick");
  cvc::state *seq = pk.findDescendant("seq");
  if (!seq || seq->value() == m_lastPickSeq)
    return;
  m_lastPickSeq = seq->value();
  const auto field = [&pk](const char *k) {
    cvc::state *s = pk.findDescendant(k);
    return s ? s->value() : std::string();
  };
  cvc::grid_pick p;
  if (!cvc::parse_grid_pick_phase(field("phase"), p.phase))
    return;
  // The plane: the slice the pick met (its axis), else the view the switch shows.
  if (!cvc::parse_slice_axis(field("axis"), p.axis))
    p.axis = cvc::grid_view_axis(m_params.view);
  // The voxel: the one the pick names inside the volume, else (past its edge) the nearest grid
  // point to where it met the plane, unclamped, so a rubber band can start outside.
  long long ijk[3];
  const char *names[3] = {"i", "j", "k"};
  bool named = true;
  for (int c = 0; c < 3; ++c)
    named = named && cvc::param_detail::parse_integer(field(names[c]), ijk[c]);
  if (named) {
    for (int c = 0; c < 3; ++c)
      p.ijk[c] = ijk[c];
    p.inside = true;
  } else {
    double w[3];
    const char *coords[3] = {"x", "y", "z"};
    for (int c = 0; c < 3; ++c)
      if (!cvc::param_detail::parse_double(field(coords[c]), w[c]))
        return; // a miss: nothing to go on
    p.inside = voxelOf(w, p.ijk);
  }
  p.ijk[static_cast<int>(p.axis)] = m_index[static_cast<int>(p.axis)]; // on the plane shown
  cvc::parse_grid_pick_mods(field("mods"), p);
  cursorFrom("grid.pick");

  long long current = -1;
  cvc::param_detail::parse_integer(getState("grid.classes.current").value(), current);
  cvc::grid_pick_outcome o;
  try {
    o = m_gestures.apply(m_editor, m_params.tool, static_cast<long>(current), m_display.point_scale,
                         p);
  } catch (const std::exception &e) {
    status(std::string("error: ") + e.what());
    return;
  }
  cvc::state &grid = getState("grid");
  for (int a = 0; a < 3; ++a)
    if (o.slice[a] >= 0)
      put(grid(std::string("slice.") + "xyz"[a]), std::to_string(o.slice[a]));
  m_band = o;
  if (!o.note.empty())
    status(o.note);
}

void VolumeGridNode::runRequest(const std::string &request) {
  const std::string line = trim(request);
  const std::size_t sp = line.find_first_of(" \t");
  const std::string cmd = line.substr(0, sp);
  const std::string arg = sp == std::string::npos ? std::string() : trim(line.substr(sp));
  const auto need_file = [&]() {
    if (arg.empty())
      throw std::invalid_argument(cmd + " needs a file");
    return resolvePath(arg);
  };
  const cvc::slice_grid &g = m_source->grid();
  const std::vector<int> dims{static_cast<int>(g.dim.xdim), static_cast<int>(g.dim.ydim),
                              static_cast<int>(g.dim.zdim)};
  const auto read_params = [&]() {
    cvc::point_class_read_params rp;
    rp.dims = dims;
    rp.bbox = {g.box.minx, g.box.miny, g.box.minz, g.box.maxx, g.box.maxy, g.box.maxz};
    rp.variable = m_var;
    rp.timestep = m_time;
    return rp;
  };
  try {
    if (cmd == "load") {
      const cvc::param_result<cvc::volume_grid_source_params> sp =
          cvc::params_from_state<cvc::volume_grid_source_params>(getState("grid.source"));
      if (!sp.ok())
        throw std::invalid_argument(sp.message());
      if (sp.value.file.empty())
        throw std::invalid_argument("source.file is empty: nothing to load");
      std::string error;
      std::shared_ptr<cvc::slice_source> src;
      if (m_loader) {
        src = m_loader(sp.value, error);
      } else {
        cvc::volume vol(app());
        cvc::readVolumeFile(app(), vol, resolvePath(sp.value.file),
                            static_cast<unsigned>(sp.value.var),
                            static_cast<unsigned>(sp.value.time));
        src = std::make_shared<cvc::slice_source>(vol);
      }
      if (!src)
        throw std::runtime_error(error.empty() ? "could not read " + sp.value.file : error);
      setSource(src, sp.value.file, sp.value.var, sp.value.time);
      status("loaded " + sp.value.file);
    } else if (cmd == "save_classes") {
      const std::string f = need_file();
      cvc::write_point_classes(app(), m_editor.classes(), f);
      status("saved " + std::to_string(m_editor.classes().classes.size()) + " classes to " + arg);
    } else if (cmd == "load_classes" || cmd == "load_points") {
      const std::string f = need_file();
      cvc::point_classes read = cvc::read_point_classes(app(), f, read_params());
      if (cmd == "load_points") { // add the file's to the classes of those names
        cvc::point_classes merged = m_editor.classes();
        for (const cvc::point_class &c : read.classes) {
          if (merged.class_index(c.name, c.variable, c.timestep) < 0) {
            merged.set_class(c); // a new class, with its colour and radius
            continue;
          }
          for (std::size_t p = 0; p < c.size(); ++p)
            merged.add(c.name, c.points[3 * p], c.points[3 * p + 1], c.points[3 * p + 2],
                       c.variable, c.timestep);
        }
        read = std::move(merged);
      }
      m_editor.set_classes(std::move(read));
      publishPoints(true);
      status("read " + std::to_string(m_editor.classes().point_count()) + " points in " +
             std::to_string(m_editor.classes().classes.size()) + " classes from " + arg);
    } else if (cmd == "delete_selected") {
      const std::size_t n = m_editor.delete_selected();
      status("deleted " + std::to_string(n) + " point" + (n == 1 ? "" : "s"));
    } else if (cmd == "select_all") {
      m_editor.select_all();
      status(std::to_string(m_editor.selection().size()) + " points selected");
    } else if (cmd == "select_none") {
      m_editor.clear_selection();
      status("");
    } else if (cmd == "undo") {
      status(m_editor.undo() ? "undone" : "nothing to undo");
    } else if (cmd == "step") {
      long long n = 1;
      if (!arg.empty() && !cvc::param_detail::parse_integer(arg, n))
        throw std::invalid_argument("step '" + arg + "': not a whole number");
      const int a = static_cast<int>(cvc::grid_view_axis(m_params.view));
      const std::int64_t last = static_cast<std::int64_t>(g.count(a)) - 1;
      const std::int64_t to = std::clamp<std::int64_t>(m_index[a] + n, 0, last);
      put(getState("grid")(std::string("slice.") + "xyz"[a]), std::to_string(to));
    } else if (cmd == "save_session") {
      const std::string f = need_file();
      cvc::volume_grid_session s;
      for (const std::string &full : getState("grid").children()) {
        const std::string rel = full.substr(getState().fullName().size() + 1);
        if (!cvc::volume_grid_session_key(rel))
          continue;
        const std::string v = getState(rel).value();
        if (!v.empty())
          s.keys[rel] = v;
      }
      s.transfer_function = getState("transfer_function").value();
      s.classes = m_editor.classes();
      cvc::write_volume_grid_session(s, f);
      status("saved the session to " + arg);
    } else if (cmd == "load_session") {
      const std::string f = need_file();
      const cvc::volume_grid_session s = cvc::read_volume_grid_session(f);
      const cvc::volume_grid_source_params before =
          cvc::params_from_state<cvc::volume_grid_source_params>(getState("grid.source")).value;
      for (const auto &kv : s.keys)
        getState(kv.first).value(kv.second);
      if (!s.transfer_function.empty())
        getState("transfer_function").value(s.transfer_function);
      const cvc::volume_grid_source_params after =
          cvc::params_from_state<cvc::volume_grid_source_params>(getState("grid.source")).value;
      if (!after.file.empty() && (after.file != before.file || after.var != m_var ||
                                  after.time != m_time || after.file != m_file))
        runRequest("load");
      m_editor.set_classes(s.classes);
      publishPoints(true);
      ++m_keysVersion;
      status("read the session from " + arg);
    } else if (!cmd.empty()) {
      throw std::invalid_argument("'" + cmd + "' is not a request");
    }
  } catch (const std::exception &e) {
    status(std::string("error: ") + cmd + ": " + e.what());
  }
}

bool VolumeGridNode::tick() {
  if (!m_source)
    return false;
  if (m_keysVersion != m_keysSeen) {
    m_keysSeen = m_keysVersion;
    readKeys();
  }
  adoptData(); // a data write need not signal as a value does: compared every frame
  if (m_rowsVersion != m_rowsSeen) {
    m_rowsSeen = m_rowsVersion;
    readRows();
  }
  if (m_pointerVersion != m_pointerSeen) {
    m_pointerSeen = m_pointerVersion;
    consumePointer();
  }
  if (m_pickVersion != m_pickSeen) {
    m_pickSeen = m_pickVersion;
    consumePick();
  }
  if (m_requestVersion != m_requestSeen) {
    m_requestSeen = m_requestVersion;
    cvc::state &r = getState("grid.request");
    const std::string request = r.value();
    if (!request.empty()) {
      r.value(std::string()); // consumed
      runRequest(request);
    }
  }
  // A pick or request may have moved a plane: place them now, not a frame late.
  if (m_keysVersion != m_keysSeen) {
    m_keysSeen = m_keysVersion;
    readKeys();
  }
  if (m_editor.version() != m_pointsVersionShown)
    publishPoints(true);
  if (m_editor.selection_version() != m_selectionVersionShown)
    publishSelection();

  bool changed = false;
  std::ostringstream sig;
  sig << m_editor.version() << '|' << m_editor.selection_version() << '|' << m_index[0] << ','
      << m_index[1] << ',' << m_index[2] << '|' << m_display.point_scale << ','
      << m_display.show_points << ',' << m_display.crosshairs << '|' << (m_band.band ? 1 : 0) << ','
      << static_cast<int>(m_band.band_axis) << ',' << m_band.band_index << ','
      << m_band.band_rect[0] << ',' << m_band.band_rect[1] << ',' << m_band.band_rect[2] << ','
      << m_band.band_rect[3] << '|' << m_source.get();
  if (sig.str() != m_overlaySignature) {
    m_overlaySignature = sig.str();
    rebuildOverlays();
    changed = true;
  }
  for (const auto &s : m_slices)
    changed = s->tick() || changed;
  return changed;
}

void VolumeGridNode::rebuildOverlays() {
  ++m_overlayBuilds;
  const cvc::slice_grid &g = m_source->grid();
  for (cvc::slice_axis a : {cvc::slice_axis::x, cvc::slice_axis::y, cvc::slice_axis::z}) {
    const int ax = static_cast<int>(a);
    const std::pair<int, int> ru = cvc::grid_plane_axes(a);
    const double plane = g.plane(a, static_cast<std::uint64_t>(m_index[ax]));
    const double sr = g.spacing(ru.first), su = g.spacing(ru.second);
    const auto coord = [&g](int axis, double index) {
      return g.lo(axis) + index * g.spacing(axis);
    };

    // Points: a disk per point of a shown class on this plane, its radius in voxels.
    cvc::geometry pts;
    std::vector<cvc::point_ref> on;
    if (m_display.show_points)
      on = m_editor.points_on(a, m_index[ax]);
    for (const cvc::point_ref &r : on) {
      const cvc::point_class &c = m_editor.classes().classes[r.cls];
      const cvc::geometry::color_t col{{c.color.size() > 0 ? c.color[0] : 1.0,
                                        c.color.size() > 1 ? c.color[1] : 0.0,
                                        c.color.size() > 2 ? c.color[2] : 0.0}};
      const double rad = m_editor.pick_radius(r.cls, m_display.point_scale);
      const double cr = coord(ru.first, r.ijk[ru.first]), cu = coord(ru.second, r.ijk[ru.second]);
      const auto base = static_cast<cvc::geometry::index_t>(pts.points().size());
      pts.points().push_back(at(a, plane, cr, cu));
      pts.colors().push_back(col);
      for (int k = 0; k < kDiskSides; ++k) {
        const double t = 2.0 * kPi * k / kDiskSides;
        pts.points().push_back(
            at(a, plane, cr + rad * sr * std::cos(t), cu + rad * su * std::sin(t)));
        pts.colors().push_back(col);
      }
      for (int k = 0; k < kDiskSides; ++k)
        pts.tris().push_back(
            {{base, static_cast<cvc::geometry::index_t>(base + 1 + k),
              static_cast<cvc::geometry::index_t>(base + 1 + (k + 1) % kDiskSides)}});
    }

    // Lines: the selection's rings, the crosshairs, a rubber band.
    cvc::geometry lines;
    const auto segment = [&lines](const cvc::geometry::point_t &p, const cvc::geometry::point_t &q,
                                  const cvc::geometry::color_t &col) {
      const auto i = static_cast<cvc::geometry::index_t>(lines.points().size());
      lines.points().push_back(p);
      lines.points().push_back(q);
      lines.colors().push_back(col);
      lines.colors().push_back(col);
      lines.lines().push_back({{i, i + 1}});
    };
    const cvc::geometry::color_t white{{1.0, 1.0, 1.0}};
    for (const cvc::point_ref &r : on) {
      if (!m_editor.selected(r))
        continue;
      const double rad = 1.35 * m_editor.pick_radius(r.cls, m_display.point_scale);
      const double cr = coord(ru.first, r.ijk[ru.first]), cu = coord(ru.second, r.ijk[ru.second]);
      for (int k = 0; k < kDiskSides; ++k) {
        const double t0 = 2.0 * kPi * k / kDiskSides, t1 = 2.0 * kPi * (k + 1) / kDiskSides;
        segment(at(a, plane, cr + rad * sr * std::cos(t0), cu + rad * su * std::sin(t0)),
                at(a, plane, cr + rad * sr * std::cos(t1), cu + rad * su * std::sin(t1)), white);
      }
    }
    if (m_display.crosshairs) {
      // Where the other two planes cross this one, across the whole plane (half a voxel past its
      // edge, as the quad is drawn).
      const double r0 = g.lo(ru.first) - 0.5 * sr, r1 = g.hi(ru.first) + 0.5 * sr;
      const double u0 = g.lo(ru.second) - 0.5 * su, u1 = g.hi(ru.second) + 0.5 * su;
      const double rp = g.plane(static_cast<cvc::slice_axis>(ru.first),
                                static_cast<std::uint64_t>(m_index[ru.first]));
      const double up = g.plane(static_cast<cvc::slice_axis>(ru.second),
                                static_cast<std::uint64_t>(m_index[ru.second]));
      segment(at(a, plane, rp, u0), at(a, plane, rp, u1), axis_color(ru.second));
      segment(at(a, plane, r0, up), at(a, plane, r1, up), axis_color(ru.first));
    }
    if (m_band.band && m_band.band_axis == a) {
      const double b0r =
          coord(ru.first, static_cast<double>(std::min(m_band.band_rect[0], m_band.band_rect[2]))) -
          0.5 * sr;
      const double b1r =
          coord(ru.first, static_cast<double>(std::max(m_band.band_rect[0], m_band.band_rect[2]))) +
          0.5 * sr;
      const double b0u =
          coord(ru.second,
                static_cast<double>(std::min(m_band.band_rect[1], m_band.band_rect[3]))) -
          0.5 * su;
      const double b1u =
          coord(ru.second,
                static_cast<double>(std::max(m_band.band_rect[1], m_band.band_rect[3]))) +
          0.5 * su;
      const cvc::geometry::color_t band{{1.0, 0.9, 0.3}};
      segment(at(a, plane, b0r, b0u), at(a, plane, b1r, b0u), band);
      segment(at(a, plane, b1r, b0u), at(a, plane, b1r, b1u), band);
      segment(at(a, plane, b1r, b1u), at(a, plane, b0r, b1u), band);
      segment(at(a, plane, b0r, b1u), at(a, plane, b0r, b0u), band);
    }

    for (auto [node, geom] : {std::pair<GeometryNode *, cvc::geometry *>{m_points[ax].get(), &pts},
                              {m_lines[ax].get(), &lines}}) {
      const bool any = geom->num_points() > 0;
      if (any) {
        node->setGeometry(*geom);
        // setGeometry picks a mode from what the geometry holds, a surface for a lines-only one,
        // which draws no lines: the lines overlay asks for its own every time.
        if (geom == &lines)
          node->setRenderMode(GeometryRenderMode::LINES);
      }
      node->setVisible(any);
    }
  }
}

} // namespace gl
} // namespace cvc
