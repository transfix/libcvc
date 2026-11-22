/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeGridContours. See inc/cvc/gl/VolumeGridContours.h.
//
// Threading, as contour_lab's: everything here runs on the thread that ticks the scene, which
// pumps the app's scheduler and does every state, scene and data-channel write. The tile job owns
// its stepper and a copy of the curves and touches nothing else; the tick polls it and applies its
// result once it is done. Teardown cancels the task.

#include <algorithm>
#include <cctype>
#include <cmath>
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
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/VolumeGridContours.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/state/followed_key.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/volume/volume_slice.h>
#include <sstream>

namespace cvc {
namespace gl {

namespace {

namespace se = cvc::state_exec;

std::string num(double d) { return cvc::param_detail::format_double(d); }

void put(cvc::state &s, const std::string &v) {
  if (s.value() != v)
    s.value(v);
}

bool starts_with(const std::string &s, const char *p) { return s.rfind(p, 0) == 0; }
bool ends_with(const std::string &s, const char *p) {
  const std::size_t n = std::char_traits<char>::length(p);
  return s.size() >= n && s.compare(s.size() - n, n, p) == 0;
}

// An object's name as one state key segment: '.' and white space become '_'.
std::string key_of(const std::string &name) {
  std::string k = name.empty() ? std::string("_") : name;
  for (char &c : k)
    if (c == '.' || std::isspace(static_cast<unsigned char>(c)))
      c = '_';
  return k;
}

cvc::geometry::color_t color_of(const std::vector<double> &c) {
  return {{c.size() > 0 ? c[0] : 1.0, c.size() > 1 ? c[1] : 1.0, c.size() > 2 ? c[2] : 1.0}};
}

constexpr int kDiskSides = 12;
constexpr double kPi = 3.14159265358979323846;

// The point in object space at plane coordinates (r, u) on the plane at `plane` along axis `a`.
cvc::geometry::point_t at(cvc::slice_axis a, double plane, double r, double u) {
  const std::pair<int, int> ru = cvc::grid_plane_axes(a);
  cvc::geometry::point_t p{{0.0, 0.0, 0.0}};
  p[static_cast<int>(a)] = plane;
  p[ru.first] = r;
  p[ru.second] = u;
  return p;
}

// What a data channel holds, unwrapped from the data object a program stores there.
boost::any unwrap(const boost::any &a) {
  if (const auto *v = boost::any_cast<se::value_t>(&a))
    if (const auto *d = std::get_if<se::data_object_ptr>(&v->v))
      if (*d)
        return (*d)->payload;
  if (const auto *d = boost::any_cast<se::data_object_ptr>(&a))
    if (*d)
      return (*d)->payload;
  return a;
}

template <class T> std::shared_ptr<const T> held(const boost::any &a) {
  if (const auto *c = boost::any_cast<std::shared_ptr<const T>>(&a))
    return *c;
  if (const auto *m = boost::any_cast<std::shared_ptr<T>>(&a))
    return *m;
  if (const auto *v = boost::any_cast<T>(&a))
    return std::make_shared<const T>(*v);
  return nullptr;
}

// The object a followed payload points at (a pointer the data channel holds, or a data object),
// so a write within the clock's resolution still tells (cvc::followed_key).
const void *payload_id(const boost::any &a) {
  if (const auto *v = boost::any_cast<se::value_t>(&a))
    if (const auto *d = std::get_if<se::data_object_ptr>(&v->v))
      return d->get();
  if (const auto *d = boost::any_cast<se::data_object_ptr>(&a))
    return d->get();
  return cvc::any_pointee<const cvc::grid_contours, cvc::grid_contours, const cvc::contour_stack,
                          cvc::contour_stack>(a);
}

// grid_tile_stepper with its label copied out after each step, so the tick reads it while a pool
// chunk steps.
struct LabelledGridTileStepper {
  cvc::grid_tile_stepper s;
  std::shared_ptr<VolumeGridContours::Label> label;

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
  cvc::grid_tile_result take() { return s.take(); }
};

} // namespace

void VolumeGridContours::Label::set(std::string s) {
  std::lock_guard<std::mutex> lock(m);
  text = std::move(s);
}

std::string VolumeGridContours::Label::get() const {
  std::lock_guard<std::mutex> lock(m);
  return text;
}

VolumeGridContours::VolumeGridContours(VolumeGridNode &grid) : m_grid(grid) {
  cvc::params_describe_state<cvc::volume_grid_contour_params>(m_grid.getState("grid.contour"));
  cvc::params_describe_state<cvc::tile_params>(m_grid.getState("grid.tile"));
  m_keys =
      std::make_unique<cvc::job_keys>(m_grid.app().root(), m_grid.getState("grid.tile").fullName());
}

VolumeGridContours::~VolumeGridContours() {
  if (m_job)
    m_job->cancel();
  if (m_task >= 0)
    cvc::cancel_sliced_job(m_grid.app(), m_task);
}

std::shared_ptr<GeometryNode> VolumeGridContours::curvesOverlay(cvc::slice_axis a) const {
  return m_curves[static_cast<int>(a)];
}

std::shared_ptr<GeometryNode> VolumeGridContours::pointsOverlay(cvc::slice_axis a) const {
  return m_points[static_cast<int>(a)];
}

void VolumeGridContours::ensureChildren() {
  if (m_curves[0] || !m_grid.slice(cvc::slice_axis::z))
    return;
  for (cvc::slice_axis a : {cvc::slice_axis::z, cvc::slice_axis::y, cvc::slice_axis::x}) {
    const int ax = static_cast<int>(a);
    std::shared_ptr<VolumeSliceNode> s = m_grid.slice(a);
    for (int k = 0; k < 2; ++k) {
      std::shared_ptr<GeometryNode> g =
          s->addGraphicsChild<GeometryNode>(k ? "contour_points" : "contours");
      g->setUseSingleColor(false);
      g->setAmbient(1.0); // unlit: the colours are the colours
      g->setDiffuse(0.0);
      g->setSpecular(0.0);
      if (!k) {
        g->setRenderMode(GeometryRenderMode::LINES); // and again after each setGeometry
        g->setLineWidth(2.0);
      }
      g->setVisible(false); // until the first build, with its plane's volume
      (k ? m_points : m_curves)[ax] = g;
    }
  }
  std::shared_ptr<NullGraphicNode> surfaces = m_grid.surfaces();
  m_curves3d = surfaces->addGraphicsChild<GeometryNode>("contours");
  m_curves3d->setUseSingleColor(false);
  m_curves3d->setAmbient(1.0);
  m_curves3d->setDiffuse(0.0);
  m_curves3d->setSpecular(0.0);
  m_curves3d->setRenderMode(GeometryRenderMode::LINES);
  m_curves3d->setLineWidth(2.0);
  m_curves3d->setDepthOffset(4.0); // over the surfaces they bound
  m_curves3d->setVisible(false);   // until there is a curve: an empty node frames no view
  m_tiles = surfaces->addGraphicsChild<NullGraphicNode>("tiles");
  m_tiles->setVisible(false); // until there is a surface
}

// On the first tick, not with the children: a document's props seed their keys between the two.
void VolumeGridContours::seed() {
  if (m_seeded)
    return;
  m_seeded = true;
  // Every parameter key has its default (a document's props seeded theirs before), so what binds
  // them reads a value while the volume is on its way.
  cvc::params_from_state<cvc::volume_grid_contour_params>(m_grid.getState("grid.contour"));
  cvc::params_from_state<cvc::tile_params>(m_grid.getState("grid.tile"));
  if (m_grid.getState("grid.objects").value().empty())
    put(m_grid.getState("grid.objects"), "0");
  m_keys->idle();
  put(m_grid.getState("grid.tile.label"), "");
  put(m_grid.getState("grid.tile.warnings"), "0");
  put(m_grid.getState("grid.contour.selected"), "0");
  put(m_grid.getState("grid.contour.active"), "");
  put(m_grid.getState("grid.contour.info"), "no objects");
}

void VolumeGridContours::setFrame(const cvc::grid_frame &f) {
  m_editor.set_frame(f);
  m_haveFrame = true;
  m_gestures.reset();
  m_band = cvc::contour_pick_outcome{};
  if (!m_pending.empty()) {
    const boost::any d = std::move(m_pending);
    m_pending = boost::any();
    adoptPayload(d, m_pendingFrom);
  }
  m_overlaySignature.clear();
}

void VolumeGridContours::status(const std::string &s) { put(m_grid.getState("grid.status"), s); }

bool VolumeGridContours::stateChanged(const std::string &k) {
  if (k == "objects" ||
      (starts_with(k, "objects.") && k != "objects.current" && !ends_with(k, ".curves") &&
       !ends_with(k, ".points") && !ends_with(k, ".triangles"))) {
    ++m_rowsVersion;
    return true;
  }
  const auto tile_key = [&k]() {
    if (!starts_with(k, "tile."))
      return false;
    const std::vector<std::string> names = cvc::param_names<cvc::tile_params>();
    return std::find(names.begin(), names.end(), k.substr(5)) != names.end();
  };
  if (k == "objects.current" ||
      (starts_with(k, "contour.") && k != "contour.selected" && k != "contour.info" &&
       k != "contour.active") ||
      tile_key()) {
    ++m_keysVersion;
    return true;
  }
  return false;
}

std::string VolumeGridContours::resolve(const std::string &key) const {
  if (key.empty())
    return std::string();
  if (key[0] == '/')
    return key.substr(1);
  const std::string &scope = m_grid.scope();
  return scope.empty() ? key : scope + cvc::state::SEPARATOR + key;
}

void VolumeGridContours::followSource() {
  boost::any d;
  if (!m_follow.changed(m_grid.app().root(), resolve(m_params.source), d, payload_id))
    return;
  const std::string &node = m_follow.node();
  if (node == m_grid.getState("grid.curves").fullName() ||
      node == m_grid.getState("grid.contours").fullName()) {
    status("error: grid.contour.source names this grid's own contours");
    return;
  }
  if (!d.empty())
    adoptPayload(d, m_params.source);
}

void VolumeGridContours::adoptPayload(const boost::any &raw, const std::string &from) {
  const boost::any d = unwrap(raw);
  try {
    if (const std::shared_ptr<const cvc::grid_contours> c = held<cvc::grid_contours>(d)) {
      cvc::grid_contours next = *c;
      if (m_haveFrame)
        next.frame = m_editor.frame(); // drawn on this grid: its planes
      m_editor.set_contours(std::move(next));
      status("read " + std::to_string(c->curve_count()) + " curves of " +
             std::to_string(c->objects.size()) + " objects from " + from);
    } else if (const std::shared_ptr<const cvc::contour_stack> s = held<cvc::contour_stack>(d)) {
      if (!m_haveFrame) { // the planes come with the volume
        m_pending = raw;
        m_pendingFrom = from;
        status("contours at " + from + " wait for the volume");
        return;
      }
      std::vector<std::string> warnings;
      m_editor.set_contours(
          cvc::grid_contours_from_stack(*s, m_editor.frame(), cvc::slice_axis::z, &warnings));
      status(warnings.empty() ? "read " + std::to_string(s->polygon_count()) + " contours on " +
                                    std::to_string(s->indices().size()) + " slices from " + from
                              : "note: " + from + ": " + warnings.front());
    } else {
      status("error: " + from + " holds no contours (a contour_stack or volume grid contours)");
      return;
    }
  } catch (const std::exception &e) {
    status("error: " + from + ": " + e.what());
    return;
  }
  if (m_params.auto_tile)
    m_tileDue = true;
}

void VolumeGridContours::adoptData() {
  const boost::any d = unwrap(m_grid.getState("grid.curves").data());
  const std::shared_ptr<const cvc::grid_contours> c = held<cvc::grid_contours>(d);
  if (!c || c == m_published)
    return;
  cvc::grid_contours next = *c;
  if (m_haveFrame)
    next.frame = m_editor.frame();
  try {
    m_editor.set_contours(std::move(next));
  } catch (const std::exception &e) {
    status(std::string("error: grid.curves: ") + e.what());
    m_published = c; // said once
    return;
  }
  status("contours set (" + std::to_string(c->curve_count()) + " curves)");
  if (m_params.auto_tile)
    m_tileDue = true;
}

void VolumeGridContours::readRows() {
  std::vector<std::string> problems;
  const std::vector<cvc::grid_contour_object> rows =
      cvc::read_contour_object_rows(m_grid.getState("grid.objects"), &problems);
  try {
    m_editor.set_objects(cvc::reconcile_contour_object_rows(m_editor.contours(), rows));
  } catch (const std::exception &e) {
    status(std::string("error: ") + e.what());
  }
  if (!problems.empty())
    status("error: " + problems.front());
}

void VolumeGridContours::readKeys() {
  const cvc::param_result<cvc::volume_grid_contour_params> p =
      cvc::params_from_state<cvc::volume_grid_contour_params>(m_grid.getState("grid.contour"),
                                                              m_params);
  if (!p.ok())
    status("error: contour." + p.message());
  else
    m_params = p.value;
  fixCurrent();
}

long VolumeGridContours::current() const {
  long long c = -1;
  if (!cvc::param_detail::parse_integer(m_grid.getState("grid.objects.current").value(), c))
    return -1;
  return static_cast<long>(c);
}

void VolumeGridContours::fixCurrent() {
  // The object a click draws: kept while it names one, else the first (or none).
  cvc::state &cur = m_grid.getState("grid.objects.current");
  const std::size_t n = m_editor.contours().objects.size();
  long long c = -2;
  const std::string text = cur.value();
  if (!text.empty() && !cvc::param_detail::parse_integer(text, c))
    c = -2;
  if (text == m_autoCurrent && c == -1 && n > 0)
    c = -2; // the -1 written while there were no objects, not someone's choice
  else if (c == -1)
    return; // none, on purpose
  if (c < 0 || c >= static_cast<long long>(n)) {
    m_autoCurrent = n ? "0" : "-1";
    put(cur, m_autoCurrent);
  }
}

void VolumeGridContours::publish() {
  const cvc::grid_contours &c = m_editor.contours();
  m_published = std::make_shared<const cvc::grid_contours>(c);
  m_grid.getState("grid.curves").data(boost::any(m_published));
  cvc::write_contour_object_rows(m_grid.getState("grid.objects"), c);
  writeTriangleRows();
  m_rowsSeen = m_rowsVersion; // the rows just written are these
  // The stacks: what contour_lab, tiler.tile and the contour formats take.
  std::size_t open = 0, open_axis = 0;
  try {
    m_grid.getState("grid.contours")
        .data(boost::any(std::make_shared<const cvc::contour_stack>(
            cvc::grid_contours_stack(c, cvc::slice_axis::z, &open))));
    m_grid.getState("grid.contours.xz")
        .data(boost::any(std::make_shared<const cvc::contour_stack>(
            cvc::grid_contours_stack(c, cvc::slice_axis::y, &open_axis))));
    open += open_axis;
    m_grid.getState("grid.contours.zy")
        .data(boost::any(std::make_shared<const cvc::contour_stack>(
            cvc::grid_contours_stack(c, cvc::slice_axis::x, &open_axis))));
    open += open_axis;
  } catch (const std::exception &e) {
    status(std::string("error: ") + e.what());
  }
  const std::size_t curves = c.curve_count();
  put(m_grid.getState("grid.contour.info"),
      c.objects.empty()
          ? std::string("no objects")
          : std::to_string(c.objects.size()) + " object" + (c.objects.size() == 1 ? "" : "s") +
                ", " + std::to_string(curves) + " curve" + (curves == 1 ? "" : "s") +
                (open ? " (" + std::to_string(open) + " open)" : std::string()) + ", " +
                std::to_string(c.point_count()) + " points");
  m_versionShown = m_editor.version();
  fixCurrent();
}

// Each row's surface triangles ("-" for an object with none).
void VolumeGridContours::writeTriangleRows() {
  const cvc::grid_contours &c = m_editor.contours();
  for (std::size_t i = 0; i < c.objects.size(); ++i) {
    const auto it = m_surfaceTris.find(c.objects[i].name);
    put(m_grid.getState("grid.objects")(std::to_string(i))("triangles"),
        it == m_surfaceTris.end() ? std::string("-") : std::to_string(it->second));
  }
}

void VolumeGridContours::publishSelection() {
  put(m_grid.getState("grid.contour.selected"), std::to_string(m_editor.selection().size()));
  std::string active;
  if (const auto &a = m_editor.active()) {
    const cvc::grid_curve &cv = m_editor.curve(a->curve);
    active = m_editor.contours().objects[a->curve.object].name + " on " +
             cvc::slice_axis_name(cv.axis) + " = " + std::to_string(cv.index) + ", " +
             std::to_string(cv.size()) + " point" + (cv.size() == 1 ? "" : "s");
  }
  put(m_grid.getState("grid.contour.active"), active);
  m_selectionShown = m_editor.selection_version();
}

double VolumeGridContours::pickRadius(cvc::slice_axis a) const {
  const std::pair<int, int> ru = cvc::grid_plane_axes(a);
  const cvc::grid_frame &f = m_editor.frame();
  return 1.5 * m_params.point_size * std::min(f.spacing[ru.first], f.spacing[ru.second]);
}

void VolumeGridContours::pick(const cvc::contour_pick &p) {
  if (!m_haveFrame)
    return;
  cvc::contour_pick q = p;
  if (m_newCurve && !q.shift && !q.ctrl && !q.alt)
    q.shift = true; // new_curve: the click starts one
  cvc::contour_pick_outcome o;
  try {
    o = m_gestures.apply(m_editor, current(), pickRadius(q.axis), m_params.snap, q);
  } catch (const std::exception &e) {
    status(std::string("error: ") + e.what());
    return;
  }
  if (q.phase == cvc::contour_pick::phase_t::up && o.changed)
    m_newCurve = false;
  if (o.object >= 0 && o.object != current())
    put(m_grid.getState("grid.objects.current"), std::to_string(o.object));
  m_band = o;
  if (!o.note.empty())
    status(o.note);
}

void VolumeGridContours::resetGesture() {
  m_gestures.reset();
  m_band = cvc::contour_pick_outcome{};
}

// trace <iso> [first last [step]]: the curves where the shown view's planes cross iso, added to
// the current object (VolumeGridRover's isocontouring); the shown plane alone by default.
void VolumeGridContours::trace(const std::string &arg) {
  std::istringstream in(arg);
  std::vector<std::string> words;
  for (std::string w; in >> w;)
    words.push_back(w);
  double iso = 0;
  long long first = 0, last = 0, step = 1;
  cvc::grid_view view = cvc::grid_view::xy;
  cvc::param_detail::enum_from_name(m_grid.getState("grid.view").value(), view);
  const cvc::slice_axis a = cvc::grid_view_axis(view);
  const std::shared_ptr<cvc::slice_source> src = m_grid.source();
  if (!src) {
    status("error: trace: no volume");
    return;
  }
  const long long planes = static_cast<long long>(src->grid().count(a));
  cvc::param_detail::parse_integer(
      m_grid.getState("grid")(std::string("slice.") + "xyz"[static_cast<int>(a)]).value(), first);
  last = first;
  if (words.empty() || !cvc::param_detail::parse_double(words[0], iso) ||
      (words.size() >= 3 && (!cvc::param_detail::parse_integer(words[1], first) ||
                             !cvc::param_detail::parse_integer(words[2], last))) ||
      (words.size() >= 4 && (!cvc::param_detail::parse_integer(words[3], step) || step < 1)) ||
      words.size() == 2 || words.size() > 4) {
    status("error: trace takes an isovalue, and optionally the first and last planes and a step");
    return;
  }
  const long c = current();
  if (c < 0 || static_cast<std::size_t>(c) >= m_editor.contours().objects.size()) {
    status("error: trace: no object to trace into: add one");
    return;
  }
  first = std::clamp<long long>(first, 0, planes - 1);
  last = std::clamp<long long>(last, 0, planes - 1);
  std::vector<cvc::grid_curve> curves;
  long long traced = 0;
  try {
    for (long long k = first; k <= last; k += step, ++traced)
      for (std::vector<double> &ru :
           cvc::trace_slice(*src->slice(a, static_cast<std::uint64_t>(k)), m_editor.frame(), iso)) {
        cvc::grid_curve cv;
        cv.axis = a;
        cv.index = k;
        cv.ru = std::move(ru);
        cv.closed = true;
        curves.push_back(std::move(cv));
      }
    const std::size_t n = curves.size();
    m_editor.add_curves(static_cast<std::size_t>(c), std::move(curves));
    status("traced " + std::to_string(n) + " curve" + (n == 1 ? "" : "s") + " of " +
           m_editor.contours().objects[static_cast<std::size_t>(c)].name + " at " + num(iso) +
           " on " + std::to_string(traced) + " plane" + (traced == 1 ? "" : "s"));
  } catch (const std::exception &e) {
    status(std::string("error: trace: ") + e.what());
    return;
  }
  if (m_params.auto_tile)
    m_tileDue = true;
}

bool VolumeGridContours::request(const std::string &cmd, const std::string &arg, bool tool) {
  if (cmd == "trace") {
    trace(arg);
    return true;
  }
  if (cmd == "tile") {
    launchTile();
    return true;
  }
  if (cmd == "stop_tile" || (cmd == "stop" && m_job)) {
    stopTile();
    return true;
  }
  if (cmd == "close_curve") {
    status(m_editor.close_active() ? "closed the curve"
                                   : "no curve to close: draw three points or more first");
    return true;
  }
  if (cmd == "new_curve") {
    m_newCurve = true;
    m_editor.set_active(std::nullopt);
    status("the next click starts a new curve");
    return true;
  }
  if (cmd == "delete_curve") {
    std::set<cvc::grid_curve_ref> curves;
    if (const auto &a = m_editor.active())
      curves.insert(a->curve);
    else
      for (const cvc::grid_point_ref &p : m_editor.selection())
        curves.insert(p.curve_ref());
    if (curves.empty()) {
      status("no curve to delete: select one of its points, or draw one");
      return true;
    }
    for (auto it = curves.rbegin(); it != curves.rend(); ++it) // later curves first
      m_editor.delete_curve(*it);
    status("deleted " + std::to_string(curves.size()) + " curve" + (curves.size() == 1 ? "" : "s"));
    return true;
  }
  if (cmd == "clear_object") {
    const long c = current();
    if (c < 0 || static_cast<std::size_t>(c) >= m_editor.contours().objects.size()) {
      status("no object to clear");
      return true;
    }
    const std::size_t n = m_editor.clear_object(static_cast<std::size_t>(c));
    status("cleared " + std::to_string(n) + " curve" + (n == 1 ? "" : "s") + " of " +
           m_editor.contours().objects[static_cast<std::size_t>(c)].name);
    return true;
  }
  if (!tool)
    return false;
  if (cmd == "delete_selected") {
    const std::size_t n = m_editor.delete_selected();
    status("deleted " + std::to_string(n) + " control point" + (n == 1 ? "" : "s"));
  } else if (cmd == "select_all") {
    m_editor.select_all();
    status(std::to_string(m_editor.selection().size()) + " control points selected");
  } else if (cmd == "select_none") {
    m_editor.clear_selection();
    status("");
  } else if (cmd == "undo") {
    status(m_editor.undo() ? "undone" : "nothing to undo");
  } else {
    return false;
  }
  return true;
}

const cvc::grid_contours &VolumeGridContours::contours() const { return m_editor.contours(); }

void VolumeGridContours::setContours(const cvc::grid_contours &c, const std::string &from) {
  cvc::grid_contours next = c;
  if (m_haveFrame)
    next.frame = m_editor.frame();
  try {
    m_editor.set_contours(std::move(next));
  } catch (const std::exception &e) {
    status("error: " + from + ": " + e.what());
    return;
  }
  if (m_params.auto_tile && !c.empty())
    m_tileDue = true;
}

bool VolumeGridContours::busy() const {
  return m_job != nullptr || m_tileDue ||
         (!m_pending.empty() && m_params.auto_tile); // a stack waiting for the volume
}

void VolumeGridContours::stopTile() {
  if (!m_job) {
    status("no tile to stop");
    return;
  }
  m_job->cancel();
  m_keys->stopping();
}

void VolumeGridContours::launchTile() {
  m_tileDue = false;
  if (m_job) {
    status("a tile is running: stop it first");
    return;
  }
  const cvc::param_result<cvc::tile_params> rp =
      cvc::params_from_state<cvc::tile_params>(m_grid.getState("grid.tile"));
  if (!rp.ok()) {
    m_keys->finished("error: tile." + rp.message());
    status("error: tile." + rp.message());
    return;
  }
  m_label = std::make_shared<Label>();
  std::unique_ptr<cvc::result_job<cvc::grid_tile_result>> job;
  try {
    job = cvc::make_result_job(std::make_unique<LabelledGridTileStepper>(
        LabelledGridTileStepper{cvc::grid_tile_stepper(m_editor.contours(), rp.value), m_label}));
  } catch (const std::exception &e) {
    m_keys->finished(std::string("error: ") + e.what());
    status(std::string("error: ") + e.what());
    return;
  }
  m_job = std::move(job);
  m_jobColor = rp.value.color;
  m_keys->started("tiling...");
  put(m_grid.getState("grid.tile.label"), "");
  cvc::sliced_task_options o;
  o.kind = "volume_grid";
  o.subject = "tile " + m_grid.getState().fullName();
  o.owner = "volume_grid:" + m_grid.getState("grid.tile").fullName();
  std::string why;
  m_task = cvc::schedule_sliced_job(m_grid.app(), m_job, o, &why);
  if (m_task < 0) {
    m_job.reset();
    m_keys->finished("error: the job could not start: " + why);
  } else {
    m_keys->pid(m_task);
  }
}

void VolumeGridContours::pollTile() {
  if (!m_job)
    return;
  const std::string label = m_label ? m_label->get() : std::string();
  const char *phase = m_job->phase();
  const std::string text =
      label.empty() || !phase ? std::string(phase ? phase : "") : std::string(phase) + " " + label;
  m_keys->progress(static_cast<int>(std::lround(100.0 * m_job->progress())), text.c_str());
  put(m_grid.getState("grid.tile.label"), label);
  if (!m_job->done())
    return;
  std::shared_ptr<cvc::result_job<cvc::grid_tile_result>> job = std::move(m_job);
  m_task = -1;
  put(m_grid.getState("grid.tile.label"), "");
  // A failure or a stop keeps what the grid had. done() is true at once after a cancel, while a
  // pool chunk may still be inside a step, so a stop is reported without asking the job.
  if (job->failed()) {
    m_keys->finished(*job);
    status("tile " + job->status());
  } else if (job->has_result()) {
    applyTile(job->take_result());
  } else {
    m_keys->finished(std::string("cancelled"));
    status("tile cancelled");
  }
}

void VolumeGridContours::clearSurfaces() {
  for (const auto &kv : m_surfaceKeys)
    m_grid.getState("grid.surfaces")(kv.second).data(boost::any());
  m_surfaceKeys.clear();
  m_surfaceNodes.clear();
  m_surfaceTris.clear();
  writeTriangleRows();
  if (m_tiles) {
    const std::vector<std::shared_ptr<GraphicsNode>> kids = m_tiles->getGraphicsChildren();
    for (const auto &k : kids)
      m_tiles->removeGraphicsChild(k);
    m_tiles->setVisible(false);
  }
  m_surfaceSignature.clear();
}

void VolumeGridContours::applyTile(cvc::grid_tile_result r) {
  clearSurfaces();
  std::size_t triangles = 0;
  for (auto &[name, g] : r.surfaces) {
    const std::string key = key_of(name);
    triangles += g.num_tris();
    m_surfaceKeys[name] = key;
    m_surfaceTris[name] = g.num_tris();
    m_grid.getState("grid.surfaces")(key).data(boost::any(g));
    if (!m_tiles || g.num_tris() == 0)
      continue;
    std::shared_ptr<GeometryNode> node = m_tiles->createChild<GeometryNode>(key, g);
    if (!node)
      continue;
    node->setUseSingleColor(true);
    node->setAmbient(0.3); // readable in a view whose lights are behind it (a mirror viewport)
    m_surfaceNodes[name] = node;
  }
  m_surfaceSignature.clear(); // colours, opacity and visibility applied on this tick
  cvc::state &w = m_grid.getState("grid.tile.warnings");
  long long old = 0;
  cvc::param_detail::parse_integer(w.value(), old);
  for (std::size_t i = 0; i < r.warnings.size(); ++i)
    put(w(std::to_string(i)), r.warnings[i]);
  for (long long i = static_cast<long long>(r.warnings.size()); i < old; ++i)
    w.removeChild(std::to_string(i));
  put(w, std::to_string(r.warnings.size()));
  std::string summary = "tiled " + std::to_string(m_surfaceKeys.size()) + " object" +
                        (m_surfaceKeys.size() == 1 ? "" : "s") + ": " + std::to_string(triangles) +
                        " triangles";
  if (!r.warnings.empty())
    summary += " (" + std::to_string(r.warnings.size()) + " warning" +
               (r.warnings.size() == 1 ? "" : "s") + ")";
  m_keys->finished(summary);
  status(summary);
  writeTriangleRows();
}

// Each surface in its object's colour (or the tile's one colour), shown with its object and the
// show_surfaces key, at surface_opacity.
void VolumeGridContours::syncSurfaces() {
  if (!m_tiles)
    return;
  const cvc::grid_contours &c = m_editor.contours();
  std::ostringstream sig;
  sig << m_params.show_surfaces << '|' << m_params.surface_opacity << '|' << m_surfaceNodes.size();
  for (const cvc::grid_contour_object &o : c.objects) {
    sig << '|' << o.name << ':' << o.visible;
    for (double v : o.color)
      sig << ',' << v;
  }
  if (sig.str() == m_surfaceSignature)
    return;
  m_surfaceSignature = sig.str();
  bool any = false;
  for (const auto &[name, weak] : m_surfaceNodes) {
    std::shared_ptr<GeometryNode> g = weak.lock();
    if (!g)
      continue;
    const long i = c.object_index(name);
    const cvc::grid_contour_object *o = i >= 0 ? &c.objects[static_cast<std::size_t>(i)] : nullptr;
    const std::vector<double> &col =
        m_jobColor.size() == 3 ? m_jobColor : (o ? o->color : std::vector<double>{1.0, 1.0, 1.0});
    g->setColor(col[0], col[1], col[2]);
    g->setOpacity(std::clamp(m_params.surface_opacity, 0.0, 1.0));
    const bool show = m_params.show_surfaces && (!o || o->visible);
    if (g->isVisible() != show)
      g->setVisible(show);
    any = any || show;
  }
  if (m_tiles->isVisible() != any)
    m_tiles->setVisible(any);
}

bool VolumeGridContours::tick(const std::array<std::int64_t, 3> &planes) {
  seed();
  if (m_keysVersion != m_keysSeen) {
    m_keysSeen = m_keysVersion;
    readKeys();
  }
  followSource();
  adoptData();
  if (m_rowsVersion != m_rowsSeen) {
    m_rowsSeen = m_rowsVersion;
    readRows();
  }
  if (m_editor.version() != m_versionShown)
    publish();
  if (m_editor.selection_version() != m_selectionShown || m_versionShown == m_editor.version())
    publishSelection();
  if (m_tileDue && !m_job && m_haveFrame)
    launchTile();
  pollTile();
  syncSurfaces();
  if (!m_haveFrame || !m_curves[0])
    return false;
  std::ostringstream sig;
  sig << m_editor.version() << '|' << m_editor.selection_version() << '|' << planes[0] << ','
      << planes[1] << ',' << planes[2] << '|' << m_params.show << m_params.control_points
      << m_params.show_3d << ',' << m_params.point_size << ',' << m_params.line_width << '|'
      << (m_band.band ? 1 : 0) << ',' << static_cast<int>(m_band.band_axis) << ','
      << m_band.band_index << ',' << m_band.band_rect[0] << ',' << m_band.band_rect[1] << ','
      << m_band.band_rect[2] << ',' << m_band.band_rect[3];
  if (sig.str() == m_overlaySignature)
    return false;
  m_overlaySignature = sig.str();
  rebuildOverlays(planes);
  return true;
}

void VolumeGridContours::rebuildOverlays(const std::array<std::int64_t, 3> &planes) {
  ++m_overlayBuilds;
  const cvc::grid_contours &c = m_editor.contours();
  const cvc::grid_frame &f = m_editor.frame();
  const cvc::geometry::color_t white{{1.0, 1.0, 1.0}}, amber{{1.0, 0.85, 0.2}};
  const std::optional<cvc::grid_contour_editor::active_curve> &act = m_editor.active();

  // A polyline of plane coordinates, at `plane` along `a`, into `g` (closed: back to its first).
  const auto polyline = [](cvc::geometry &g, cvc::slice_axis a, double plane,
                           const std::vector<double> &ru, bool closed,
                           const cvc::geometry::color_t &col) {
    const std::size_t n = ru.size() / 2;
    if (n < 2)
      return;
    const auto base = static_cast<cvc::geometry::index_t>(g.points().size());
    for (std::size_t i = 0; i < n; ++i) {
      g.points().push_back(at(a, plane, ru[2 * i], ru[2 * i + 1]));
      g.colors().push_back(col);
    }
    for (std::size_t i = 0; i + 1 < n; ++i)
      g.lines().push_back({{base + i, base + i + 1}});
    if (closed && n > 2)
      g.lines().push_back({{base + n - 1, base}});
  };

  for (cvc::slice_axis a : {cvc::slice_axis::x, cvc::slice_axis::y, cvc::slice_axis::z}) {
    const int ax = static_cast<int>(a);
    const std::pair<int, int> ru = cvc::grid_plane_axes(a);
    const double plane = f.plane(a, planes[ax]);
    const double sr = f.spacing[ru.first], su = f.spacing[ru.second];
    cvc::geometry lines, disks;
    if (m_params.show) {
      for (const cvc::grid_curve_ref &cr : m_editor.curves_on(a, planes[ax])) {
        const cvc::grid_contour_object &o = c.objects[cr.object];
        const cvc::grid_curve &cv = o.curves[cr.curve];
        const cvc::geometry::color_t col = color_of(o.color);
        polyline(lines, a, plane,
                 cvc::sample_grid_curve(cv.ru, cv.closed, o.interpolation, o.samples), cv.closed,
                 col);
        if (!m_params.control_points)
          continue;
        const double rad = m_params.point_size;
        for (std::size_t i = 0; i < cv.size(); ++i) {
          const double cr_ = cv.ru[2 * i], cu = cv.ru[2 * i + 1];
          const auto base = static_cast<cvc::geometry::index_t>(disks.points().size());
          disks.points().push_back(at(a, plane, cr_, cu));
          disks.colors().push_back(col);
          for (int k = 0; k < kDiskSides; ++k) {
            const double t = 2.0 * kPi * k / kDiskSides;
            disks.points().push_back(
                at(a, plane, cr_ + rad * sr * std::cos(t), cu + rad * su * std::sin(t)));
            disks.colors().push_back(col);
          }
          for (int k = 0; k < kDiskSides; ++k)
            disks.tris().push_back(
                {{base, static_cast<cvc::geometry::index_t>(base + 1 + k),
                  static_cast<cvc::geometry::index_t>(base + 1 + (k + 1) % kDiskSides)}});
          // The selection's rings, and the active end's.
          const cvc::grid_point_ref here{cr.object, cr.curve, i};
          const bool end = act && act->curve == cr && i == (act->back ? cv.size() - 1 : 0);
          if (m_editor.selected(here) || end) {
            std::vector<double> ring;
            const double rr = (end ? 2.0 : 1.5) * rad;
            for (int k = 0; k < kDiskSides; ++k) {
              const double t = 2.0 * kPi * k / kDiskSides;
              ring.insert(ring.end(), {cr_ + rr * sr * std::cos(t), cu + rr * su * std::sin(t)});
            }
            polyline(lines, a, plane, ring, true, end ? amber : white);
          }
        }
      }
    }
    if (m_band.band && m_band.band_axis == a && m_band.band_index == planes[ax]) {
      const std::array<double, 4> &b = m_band.band_rect;
      polyline(lines, a, plane, {b[0], b[1], b[2], b[1], b[2], b[3], b[0], b[3]}, true,
               {{1.0, 0.9, 0.3}});
    }
    // Set even when empty and shown from the first build on, as the grid's own overlays: one shown
    // later joins the renderer after the others and draws over them.
    m_curves[ax]->setGeometry(lines);
    m_curves[ax]->setRenderMode(GeometryRenderMode::LINES);
    m_curves[ax]->setLineWidth(m_params.line_width);
    m_curves[ax]->setVisible(true);
    m_points[ax]->setGeometry(disks);
    m_points[ax]->setVisible(true);
  }

  // Every plane's curves, for the 3D view.
  cvc::geometry all;
  if (m_params.show_3d)
    for (const cvc::grid_contour_object &o : c.objects) {
      if (!o.visible)
        continue;
      for (const cvc::grid_curve &cv : o.curves)
        polyline(all, cv.axis, f.plane(cv.axis, cv.index),
                 cvc::sample_grid_curve(cv.ru, cv.closed, o.interpolation, o.samples), cv.closed,
                 color_of(o.color));
    }
  m_curves3d->setGeometry(all);
  m_curves3d->setRenderMode(GeometryRenderMode::LINES);
  m_curves3d->setLineWidth(m_params.line_width);
  const bool show3d = all.num_points() > 0;
  if (m_curves3d->isVisible() != show3d)
    m_curves3d->setVisible(show3d);
}

} // namespace gl
} // namespace cvc
