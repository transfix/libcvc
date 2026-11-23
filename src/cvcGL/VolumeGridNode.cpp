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
#include <cvc/gl/VolumeGridContours.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/state/followed_key.h>
#include <cvc/state/state.h>
#include <cvc/tf/model.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume.h>
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

// The object a followed key's payload points at, for the payloads the grid reads as pointers (a
// volume's, the points' and the session's): a new one written within the clock's resolution is
// still a change (cvc::followed_key).
const void *payload_pointee(const boost::any &d) {
  if (const void *v = cvc::volume_payload_pointee(d)) // a program's data object's too
    return v;
  return cvc::any_pointee<const cvc::point_classes, cvc::point_classes,
                          const cvc::volume_grid_session, cvc::volume_grid_session>(d);
}

// Whether a key's data is a volume the planes can cut, without making one: a pointer to one (a
// data: entry's, a program's data object, a slice_source) or a cvc::volume held by value.
bool holds_volume(const boost::any &d) {
  return cvc::volume_payload_pointee(d) != nullptr || boost::any_cast<cvc::volume>(&d) != nullptr;
}

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
  m_contours = std::make_unique<VolumeGridContours>(*this);
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
    s->setVisible(false); // until there is a volume to cut: no view frames on an empty plane
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
      g->setVisible(false); // until the first build (rebuildOverlays), with its plane's volume
      (k ? m_lines : m_points)[ax] = g;
    }
  }
  m_surfaces = addGraphicsChild<NullGraphicNode>("surfaces");
  m_surfaces->setVisible(false); // with the planes: an empty group's box frames no view
  m_contours->ensureChildren();  // after the planes' own overlays, so it draws over them
  // The grey display's model: the planes colour through it instead of the transfer function.
  getState("grey_transfer_function").value(cvc::tf::to_string(cvc::tf::model::preset("grayscale")));
  cvc::job_keys(app().root(), getState("grid").fullName()).idle();
  // The probe's keys exist from the start (empty: no voxel yet), so what binds them reads text.
  for (const char *k : {"i", "j", "k", "x", "y", "z", "value"})
    put(getState(std::string("grid.cursor.") + k), std::string());
  put(getState("grid.cursor.inside"), "0");
  put(getState("grid.selection.count"), "0");
}

void VolumeGridNode::setSource(std::shared_ptr<cvc::slice_source> src, const std::string &label,
                               int var, int time) {
  if (!src)
    return;
  std::shared_ptr<const cvc::volume> vol;
  if (const cvc::volume *v = src->in_memory_volume())
    vol = std::shared_ptr<const cvc::volume>(src, v); // kept by the source
  adopt(std::move(src), std::move(vol), label, var, time);
}

void VolumeGridNode::adopt(std::shared_ptr<cvc::slice_source> src,
                           std::shared_ptr<const cvc::volume> vol, const std::string &label,
                           int var, int time) {
  ensureChildren();
  m_source = std::move(src);
  m_label = label;
  m_waitingFor.clear();
  m_var = var;
  m_time = time;
  put(getState("grid.source.var"), std::to_string(var)); // what readKeys reads back
  put(getState("grid.source.time"), std::to_string(time));
  const cvc::slice_grid &g = m_source->grid();
  m_editor.set_dims(
      {static_cast<int>(g.dim.xdim), static_cast<int>(g.dim.ydim), static_cast<int>(g.dim.zdim)});
  m_editor.set_shown(var, time);
  m_editor.clear_selection();
  m_gestures.reset();
  m_band = cvc::grid_pick_outcome{};
  m_contours->setFrame(cvc::grid_frame_of(g));

  // The volume the transfer function's histogram is taken over: the app's own, published as is.
  if (vol) {
    m_histogramVolume = std::move(vol);
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
  for (const auto &s : m_slices) {
    s->setSource(m_source);
    s->setVisible(true);
  }
  m_surfaces->setVisible(true);
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
  put(getState("grid.info.source"), m_label);
}

std::string VolumeGridNode::sourcePath(const std::string &which) {
  const std::string key = getState("grid.source")(which).value();
  if (key.empty())
    return std::string();
  if (key[0] == '/')
    return key.substr(1);
  return m_scope.empty() ? key : m_scope + cvc::state::SEPARATOR + key;
}

// Whether the key grid.source.<which> names data that changed since `f` last saw them
// (cvc::followed_key): another node than before (the key edited, or a link moved: a dataset
// loaded again under its name), or the same node's data written since. `payload` is then what
// it holds (empty when nothing).
bool VolumeGridNode::follow(const char *which, cvc::followed_key &f, boost::any &payload) {
  return f.changed(app().root(), sourcePath(which), payload, payload_pointee);
}

void VolumeGridNode::followSources() {
  boost::any d;
  if (follow("volume", m_followVolume, d)) {
    std::shared_ptr<cvc::slice_source> src;
    std::shared_ptr<const cvc::volume> vol;
    if (m_followVolume.node() == getState("grid.volume").fullName()) {
      status("error: grid.source.volume names this grid's own grid.volume");
    } else if (cvc::volume_payload(d, src, vol)) {
      // The same volume written again (or a link moved to it) changes nothing.
      if (!(vol && vol == m_histogramVolume) && !(!vol && src == m_source)) {
        const cvc::volume_grid_source_params sp =
            cvc::params_from_state<cvc::volume_grid_source_params>(getState("grid.source")).value;
        adopt(src, vol, getState("grid.source.volume").value(), sp.var, sp.time);
      }
    } else if (!d.empty()) {
      status("error: " + m_followVolume.path() + " holds no volume");
    }
  }
  // Not there yet (a dataset still loading, a program that has not written it): said once.
  if (!m_source) {
    const std::string key = getState("grid.source.volume").value();
    const std::string say =
        key.empty() ? "no volume: grid.source.volume names none" : "waiting for a volume at " + key;
    if (say != m_waitingFor) {
      m_waitingFor = say;
      status(say);
    }
  }
  if (follow("points", m_followPoints, d))
    adoptPoints(d, getState("grid.source.points").value());
  if (follow("session", m_followSession, d)) {
    std::shared_ptr<const cvc::volume_grid_session> s;
    if (const auto *c = boost::any_cast<std::shared_ptr<const cvc::volume_grid_session>>(&d))
      s = *c;
    else if (const auto *m = boost::any_cast<std::shared_ptr<cvc::volume_grid_session>>(&d))
      s = *m;
    else if (const auto *v = boost::any_cast<cvc::volume_grid_session>(&d))
      s = std::make_shared<const cvc::volume_grid_session>(*v);
    if (s)
      applySession(*s, getState("grid.source.session").value());
    else if (!d.empty())
      status("error: " + m_followSession.path() + " holds no volume grid session");
  }
}

// grid.volumes, what the panel's Source combo offers. The document's data: entries are the links
// under <scope>.data (with no scope that is the store's own node, whose other children are the
// requests the links point at, so only the links count), and an entry's outputs are the children
// of the node it points at. Made again only when grid.source.volume, an entry, where it points or
// its rev changes.
void VolumeGridNode::publishVolumes() {
  const std::string current = getState("grid.source.volume").value();
  cvc::state &root = app().root();
  cvc::state *data =
      current.empty()
          ? nullptr
          : root.findThroughLinks(m_scope.empty() ? std::string("data")
                                                  : m_scope + cvc::state::SEPARATOR + "data");
  std::vector<std::pair<std::string, cvc::state *>> entries;
  std::string signature = current;
  if (data)
    for (const std::string &name : data->childNames()) {
      cvc::state &link = (*data)(name);
      if (!link.isLink())
        continue;
      const std::string target = link.linkTarget();
      cvc::state *node = root.findThroughLinks(target);
      if (!node)
        continue;
      cvc::state *rev = root.findThroughLinks(target + cvc::state::SEPARATOR + "rev");
      signature += '\n' + name + '=' + target + '@' + (rev ? rev->value() : std::string());
      entries.emplace_back(name, node);
    }
  if (signature == m_volumesSignature)
    return;
  m_volumesSignature = signature;
  std::string list;
  bool listed = current.empty();
  for (const auto &[name, node] : entries)
    for (const std::string &out : node->childNames())
      if (holds_volume((*node)(out).data())) {
        const std::string path =
            std::string("data") + cvc::state::SEPARATOR + name + cvc::state::SEPARATOR + out;
        listed = listed || path == current;
        list += (list.empty() ? "" : "\n") + path;
      }
  if (!listed)
    list += (list.empty() ? "" : "\n") + current;
  put(getState("grid.volumes"), list);
}

void VolumeGridNode::adoptPoints(const boost::any &d, const std::string &from) {
  std::shared_ptr<const cvc::point_classes> p;
  if (const auto *c = boost::any_cast<std::shared_ptr<const cvc::point_classes>>(&d))
    p = *c;
  else if (const auto *m = boost::any_cast<std::shared_ptr<cvc::point_classes>>(&d))
    p = *m;
  else if (const auto *v = boost::any_cast<cvc::point_classes>(&d))
    p = std::make_shared<const cvc::point_classes>(*v);
  if (!p) {
    if (!d.empty())
      status("error: " + from + " holds no point classes");
    return;
  }
  m_editor.set_classes(*p); // an undo step
  publishPoints(true);
  std::vector<std::string> problems;
  if (m_source) {
    const cvc::slice_grid &g = m_source->grid();
    problems = cvc::point_class_problems(
        m_editor.classes(),
        {static_cast<int>(g.dim.xdim), static_cast<int>(g.dim.ydim), static_cast<int>(g.dim.zdim)});
  }
  status(problems.empty()
             ? "read " + std::to_string(m_editor.classes().point_count()) + " points in " +
                   std::to_string(m_editor.classes().classes.size()) + " classes from " + from
             : "note: " + problems.front());
}

cvc::volume_grid_session VolumeGridNode::session() {
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
  s.contours = m_contours->contours();
  return s;
}

void VolumeGridNode::applySession(const cvc::volume_grid_session &s, const std::string &from) {
  for (const auto &kv : s.keys)
    getState(kv.first).value(kv.second);
  if (!s.transfer_function.empty())
    getState("transfer_function").value(s.transfer_function);
  m_editor.set_classes(s.classes);
  publishPoints(true);
  m_contours->setContours(s.contours, from);
  ++m_keysVersion;
  status("applied the session from " + from);
}

// grid.session, rebuilt when what it holds changed: the keys, the classes, the transfer function.
void VolumeGridNode::publishSession() {
  const std::string sig =
      std::to_string(m_editor.version()) + '|' + std::to_string(m_contours->editor().version()) +
      '|' + std::to_string(m_keysVersion.load()) + '|' + getState("transfer_function").value();
  if (sig == m_sessionSignature)
    return;
  m_sessionSignature = sig;
  getState("grid.session")
      .data(boost::any(std::make_shared<const cvc::volume_grid_session>(session())));
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
    else if (!m_contours || !m_contours->stateChanged(k)) // null while constructing
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
  // The variable and timestep the volume holds: the editor shows those classes.
  const cvc::param_result<cvc::volume_grid_source_params> sp =
      cvc::params_from_state<cvc::volume_grid_source_params>(grid("source"));
  if (!sp.ok()) {
    problems += (problems.empty() ? "" : "; ") + sp.message();
  } else if (sp.value.var != m_var || sp.value.time != m_time) {
    m_var = sp.value.var;
    m_time = sp.value.time;
    m_editor.set_shown(m_var, m_time);
    m_overlaySignature.clear();
  }
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
  if (m_params.tool == cvc::grid_tool::contours) {
    // Contours take where the pick met the plane, not a voxel: the plane coordinates of the point
    // (the voxel's centre for a pick that named only its voxel).
    double w[3], local[3];
    const char *coords[3] = {"x", "y", "z"};
    bool at = true;
    for (int c = 0; c < 3; ++c)
      at = at && cvc::param_detail::parse_double(field(coords[c]), w[c]);
    if (at) {
      worldToLocal(w, local);
    } else {
      const std::int64_t v[3] = {p.ijk[0], p.ijk[1], p.ijk[2]};
      m_source->grid().voxel_center(v, local);
    }
    const std::pair<int, int> ru = cvc::grid_plane_axes(p.axis);
    cvc::contour_pick cp;
    cp.phase = static_cast<cvc::contour_pick::phase_t>(p.phase);
    cp.axis = p.axis;
    cp.index = m_index[static_cast<int>(p.axis)];
    cp.r = local[ru.first];
    cp.u = local[ru.second];
    cp.inside = p.inside;
    cp.shift = p.shift;
    cp.ctrl = p.ctrl;
    cp.alt = p.alt;
    m_contours->pick(cp);
    return;
  }

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
  const cvc::slice_grid &g = m_source->grid();
  try {
    if (m_contours->request(cmd, arg, m_params.tool == cvc::grid_tool::contours))
      return;
    if (cmd == "delete_selected") {
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
    } else if (!cmd.empty()) {
      throw std::invalid_argument("'" + cmd + "' is not a request");
    }
  } catch (const std::exception &e) {
    status(std::string("error: ") + cmd + ": " + e.what());
  }
}

bool VolumeGridNode::tick() {
  if (!m_seeded) {
    // Every parameter key has its default from the first tick (a document's props seeded theirs
    // before it), so what binds them reads a value while the volume is still on its way.
    m_seeded = true;
    cvc::state &grid = getState("grid");
    cvc::params_from_state<cvc::volume_grid_params>(grid);
    cvc::params_from_state<cvc::volume_grid_display_params>(grid("display"));
    cvc::params_from_state<cvc::volume_grid_source_params>(grid("source"));
    if (grid("classes").value().empty())
      put(grid("classes"), "0");
  }
  followSources(); // the app's volume, points and session, as they change
  publishVolumes(); // what the Source combo offers, while waiting for a volume too
  if (!m_source) {
    m_contours->tick(m_index); // its keys and its source: the curves wait for the planes
    publishSession();
    return false;
  }
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
  publishSession();

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
  changed = m_contours->tick(m_index) || changed;
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

    // Set even when empty, and shown from the first build on: an overlay shown later joins the
    // renderer after the other and draws over it, so whether a plane's disks or its lines win
    // where they cross would follow whether the points came before the volume or after.
    for (auto [node, geom] : {std::pair<GeometryNode *, cvc::geometry *>{m_points[ax].get(), &pts},
                              {m_lines[ax].get(), &lines}}) {
      node->setGeometry(*geom);
      // setGeometry picks a mode from what the geometry holds, a surface for a lines-only one,
      // which draws no lines: the lines overlay asks for its own every time.
      if (geom == &lines)
        node->setRenderMode(GeometryRenderMode::LINES);
      node->setVisible(true);
    }
  }
}

} // namespace gl
} // namespace cvc
