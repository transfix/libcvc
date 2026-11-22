/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The data side of VolumeGridRover. See inc/cvc/volume/volume_grid.h.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cvc/geometry/tiler_params.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/volume/volume_grid_segment.h>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>

namespace cvc {

namespace {

namespace pt = boost::property_tree;

std::string num(double d) { return param_detail::format_double(d); }

// The plane coordinates of a voxel looking down `a`.
template <class V> std::pair<long long, long long> plane_of(slice_axis a, const V &ijk) {
  const std::pair<int, int> ru = grid_plane_axes(a);
  return {static_cast<long long>(ijk[ru.first]), static_cast<long long>(ijk[ru.second])};
}

std::array<int, 3> voxel_of(const std::vector<int> &pts, std::size_t p) {
  return {pts[3 * p], pts[3 * p + 1], pts[3 * p + 2]};
}

std::string voxel_text(const std::array<int, 3> &v) {
  return "(" + std::to_string(v[0]) + ", " + std::to_string(v[1]) + ", " + std::to_string(v[2]) +
         ")";
}

// Drop repeated voxels from a point list, keeping each one's first place.
void dedupe(std::vector<int> &pts) {
  std::set<std::array<int, 3>> seen;
  std::vector<int> out;
  out.reserve(pts.size());
  for (std::size_t p = 0; p < pts.size() / 3; ++p)
    if (seen.insert(voxel_of(pts, p)).second)
      out.insert(out.end(), {pts[3 * p], pts[3 * p + 1], pts[3 * p + 2]});
  pts.swap(out);
}

} // namespace

// ---- parameters ---------------------------------------------------------------------------------

slice_axis grid_view_axis(grid_view v) {
  switch (v) {
  case grid_view::xy:
    return slice_axis::z;
  case grid_view::xz:
    return slice_axis::y;
  case grid_view::zy:
    return slice_axis::x;
  }
  return slice_axis::z;
}

grid_view grid_view_of(slice_axis a) {
  switch (a) {
  case slice_axis::x:
    return grid_view::zy;
  case slice_axis::y:
    return grid_view::xz;
  case slice_axis::z:
    return grid_view::xy;
  }
  return grid_view::xy;
}

std::pair<int, int> grid_plane_axes(slice_axis a) {
  int r = 0, u = 1;
  slice_plane_axes(a, r, u);
  return {r, u};
}

// ---- the editor ---------------------------------------------------------------------------------

void point_class_editor::set_dims(const std::array<int, 3> &dims) {
  for (int d : dims)
    if (d < 1)
      throw std::invalid_argument("point_class_editor: dims " + std::to_string(dims[0]) + " " +
                                  std::to_string(dims[1]) + " " + std::to_string(dims[2]) +
                                  ": every dimension needs at least one voxel");
  _dims = dims;
}

bool point_class_editor::inside(const std::array<int, 3> &ijk) const {
  for (int c = 0; c < 3; ++c)
    if (ijk[c] < 0 || ijk[c] >= _dims[c])
      return false;
  return true;
}

void point_class_editor::set_shown(int variable, int timestep) {
  if (variable == _variable && timestep == _timestep)
    return;
  _variable = variable;
  _timestep = timestep;
  drop_unselectable();
}

bool point_class_editor::shown(std::size_t cls) const {
  return cls < _pc.classes.size() && _pc.classes[cls].variable == _variable &&
         _pc.classes[cls].timestep == _timestep;
}

void point_class_editor::set_classes(point_classes pc, bool undoable) {
  if (undoable)
    push_undo();
  _pc = std::move(pc);
  if (!_selection.empty()) {
    _selection.clear();
    ++_selection_version;
  }
  changed();
}

std::vector<point_ref> point_class_editor::points_on(slice_axis a, std::int64_t index) const {
  const int ax = static_cast<int>(a);
  std::vector<point_ref> out;
  for (std::size_t c = 0; c < _pc.classes.size(); ++c) {
    if (!shown(c))
      continue;
    const std::vector<int> &pts = _pc.classes[c].points;
    for (std::size_t p = 0; p < pts.size() / 3; ++p)
      if (pts[3 * p + ax] == index)
        out.push_back({c, voxel_of(pts, p)});
  }
  return out;
}

double point_class_editor::pick_radius(std::size_t cls, double scale) const {
  const double r = cls < _pc.classes.size() ? _pc.classes[cls].radius : 1.0;
  const double s = std::isfinite(scale) && scale > 0 ? scale : 1.0;
  // At least half a voxel, so the voxel a point sits on always finds it.
  return std::max(0.5, (std::isfinite(r) && r > 0 ? r : 1.0) * s);
}

std::optional<point_ref> point_class_editor::nearest(slice_axis a, const std::array<int, 3> &v,
                                                     double scale, long cls) const {
  const int ax = static_cast<int>(a);
  const std::pair<int, int> ru = grid_plane_axes(a);
  std::optional<point_ref> best;
  double best_d = 0.0;
  for (std::size_t c = 0; c < _pc.classes.size(); ++c) {
    if (!shown(c) || (cls >= 0 && static_cast<std::size_t>(cls) != c))
      continue;
    const double radius = pick_radius(c, scale);
    const std::vector<int> &pts = _pc.classes[c].points;
    for (std::size_t p = 0; p < pts.size() / 3; ++p) {
      if (pts[3 * p + ax] != v[ax])
        continue; // only the points on this plane: the ones the view shows
      const double dr = pts[3 * p + ru.first] - v[ru.first];
      const double du = pts[3 * p + ru.second] - v[ru.second];
      const double d = std::sqrt(dr * dr + du * du);
      if (d < radius && (!best || d < best_d)) {
        best = point_ref{c, voxel_of(pts, p)};
        best_d = d;
      }
    }
  }
  return best;
}

bool point_class_editor::add(std::size_t cls, const std::array<int, 3> &v) {
  if (cls >= _pc.classes.size())
    throw std::out_of_range("point_class_editor: no class " + std::to_string(cls) + " (there are " +
                            std::to_string(_pc.classes.size()) + ")");
  if (!inside(v))
    throw std::invalid_argument("point_class_editor: voxel " + voxel_text(v) +
                                " lies outside the volume (" + std::to_string(_dims[0]) + " x " +
                                std::to_string(_dims[1]) + " x " + std::to_string(_dims[2]) + ")");
  point_class &c = _pc.classes[cls];
  if (c.contains(v[0], v[1], v[2]))
    return false;
  push_undo();
  c.points.insert(c.points.end(), {v[0], v[1], v[2]});
  changed();
  return true;
}

bool point_class_editor::remove(const point_ref &r) {
  if (r.cls >= _pc.classes.size())
    return false;
  std::vector<int> &pts = _pc.classes[r.cls].points;
  for (std::size_t p = 0; p < pts.size() / 3; ++p)
    if (voxel_of(pts, p) == r.ijk) {
      push_undo();
      pts.erase(pts.begin() + 3 * static_cast<std::ptrdiff_t>(p),
                pts.begin() + 3 * static_cast<std::ptrdiff_t>(p) + 3);
      const auto it = std::lower_bound(_selection.begin(), _selection.end(), r);
      if (it != _selection.end() && *it == r) {
        _selection.erase(it);
        ++_selection_version;
      }
      changed();
      return true;
    }
  return false;
}

point_class_editor::toggled point_class_editor::toggle(std::size_t cls, slice_axis a,
                                                       const std::array<int, 3> &v, double scale,
                                                       point_ref *which) {
  if (cls >= _pc.classes.size())
    throw std::out_of_range("point_class_editor: no class " + std::to_string(cls) + " (there are " +
                            std::to_string(_pc.classes.size()) + ")");
  if (!shown(cls))
    throw std::invalid_argument("point_class_editor: class '" + _pc.classes[cls].name +
                                "' marks variable " + std::to_string(_pc.classes[cls].variable) +
                                ", timestep " + std::to_string(_pc.classes[cls].timestep) +
                                ", not the one shown");
  if (const std::optional<point_ref> r = nearest(a, v, scale, static_cast<long>(cls))) {
    remove(*r);
    if (which)
      *which = *r;
    return toggled::removed;
  }
  add(cls, v);
  if (which)
    *which = point_ref{cls, v};
  return toggled::added;
}

bool point_class_editor::selected(const point_ref &r) const {
  return std::binary_search(_selection.begin(), _selection.end(), r);
}

void point_class_editor::select(const point_ref &r, selection_mode m) {
  if (!shown(r.cls) || !_pc.classes[r.cls].contains(r.ijk[0], r.ijk[1], r.ijk[2]))
    return;
  std::vector<point_ref> next;
  switch (m) {
  case selection_mode::replace:
    next = {r};
    break;
  case selection_mode::add:
    next = _selection;
    if (!selected(r))
      next.insert(std::lower_bound(next.begin(), next.end(), r), r);
    break;
  case selection_mode::remove:
    next = _selection;
    next.erase(std::remove(next.begin(), next.end(), r), next.end());
    break;
  }
  if (next != _selection) {
    _selection.swap(next);
    ++_selection_version;
  }
}

std::size_t point_class_editor::select_rect(slice_axis a, std::int64_t index, int r0, int u0,
                                            int r1, int u1, selection_mode m) {
  const int rlo = std::min(r0, r1), rhi = std::max(r0, r1);
  const int ulo = std::min(u0, u1), uhi = std::max(u0, u1);
  const std::pair<int, int> ru = grid_plane_axes(a);
  std::vector<point_ref> in;
  for (const point_ref &r : points_on(a, index)) {
    const int pr = r.ijk[ru.first], pu = r.ijk[ru.second];
    if (pr >= rlo && pr <= rhi && pu >= ulo && pu <= uhi)
      in.push_back(r);
  }
  std::sort(in.begin(), in.end());
  std::vector<point_ref> next;
  switch (m) {
  case selection_mode::replace:
    next = std::move(in);
    break;
  case selection_mode::add:
    std::set_union(_selection.begin(), _selection.end(), in.begin(), in.end(),
                   std::back_inserter(next));
    break;
  case selection_mode::remove:
    std::set_difference(_selection.begin(), _selection.end(), in.begin(), in.end(),
                        std::back_inserter(next));
    break;
  }
  if (next != _selection) {
    _selection.swap(next);
    ++_selection_version;
  }
  return _selection.size();
}

void point_class_editor::select_all() {
  std::vector<point_ref> next;
  for (std::size_t c = 0; c < _pc.classes.size(); ++c)
    if (shown(c))
      for (std::size_t p = 0; p < _pc.classes[c].size(); ++p)
        next.push_back({c, voxel_of(_pc.classes[c].points, p)});
  std::sort(next.begin(), next.end());
  if (next != _selection) {
    _selection.swap(next);
    ++_selection_version;
  }
}

void point_class_editor::clear_selection() {
  if (_selection.empty())
    return;
  _selection.clear();
  ++_selection_version;
}

point_classes point_class_editor::selection_classes() const {
  point_classes out = _pc;
  for (point_class &c : out.classes)
    c.points.clear();
  for (const point_ref &r : _selection)
    if (r.cls < out.classes.size())
      out.classes[r.cls].points.insert(out.classes[r.cls].points.end(),
                                       {r.ijk[0], r.ijk[1], r.ijk[2]});
  return out;
}

std::pair<int, int> point_class_editor::move_selection(slice_axis a, int dr, int du,
                                                       bool undoable) {
  if (_selection.empty() || (dr == 0 && du == 0))
    return {0, 0};
  const std::pair<int, int> ru = grid_plane_axes(a);
  // Cut the move short so every selected point stays inside.
  for (const point_ref &r : _selection) {
    const int pr = r.ijk[ru.first], pu = r.ijk[ru.second];
    dr = std::clamp(dr, -pr, _dims[ru.first] - 1 - pr);
    du = std::clamp(du, -pu, _dims[ru.second] - 1 - pu);
  }
  if (dr == 0 && du == 0)
    return {0, 0};
  if (undoable)
    push_undo();
  std::vector<point_ref> moved;
  moved.reserve(_selection.size());
  for (std::size_t c = 0; c < _pc.classes.size(); ++c) {
    std::vector<int> &pts = _pc.classes[c].points;
    bool any = false;
    for (std::size_t p = 0; p < pts.size() / 3; ++p) {
      const point_ref r{c, voxel_of(pts, p)};
      if (!selected(r))
        continue;
      pts[3 * p + ru.first] += dr;
      pts[3 * p + ru.second] += du;
      moved.push_back({c, voxel_of(pts, p)});
      any = true;
    }
    if (any)
      dedupe(pts); // a point moved onto another of its class merges with it
  }
  std::sort(moved.begin(), moved.end());
  moved.erase(std::unique(moved.begin(), moved.end()), moved.end());
  _selection.swap(moved);
  ++_selection_version;
  changed();
  return {dr, du};
}

std::size_t point_class_editor::delete_selected() {
  if (_selection.empty())
    return 0;
  push_undo();
  std::size_t gone = 0;
  for (std::size_t c = 0; c < _pc.classes.size(); ++c) {
    std::vector<int> &pts = _pc.classes[c].points;
    std::vector<int> keep;
    keep.reserve(pts.size());
    for (std::size_t p = 0; p < pts.size() / 3; ++p) {
      if (selected({c, voxel_of(pts, p)}))
        ++gone;
      else
        keep.insert(keep.end(), {pts[3 * p], pts[3 * p + 1], pts[3 * p + 2]});
    }
    pts.swap(keep);
  }
  _selection.clear();
  ++_selection_version;
  changed();
  return gone;
}

void point_class_editor::checkpoint() { push_undo(); }

bool point_class_editor::undo() {
  if (_undo.empty())
    return false;
  _pc = std::move(_undo.back());
  _undo.pop_back();
  if (!_selection.empty()) {
    _selection.clear();
    ++_selection_version;
  }
  changed();
  return true;
}

void point_class_editor::push_undo() {
  _undo.push_back(_pc);
  while (_undo.size() > undo_limit)
    _undo.pop_front();
}

void point_class_editor::changed() { ++_version; }

void point_class_editor::drop_unselectable() {
  const std::size_t before = _selection.size();
  _selection.erase(std::remove_if(_selection.begin(), _selection.end(),
                                  [this](const point_ref &r) { return !shown(r.cls); }),
                   _selection.end());
  if (_selection.size() != before)
    ++_selection_version;
}

// ---- gestures -----------------------------------------------------------------------------------

bool parse_grid_pick_phase(const std::string &text, grid_pick::phase_t &out) {
  if (text == "down")
    out = grid_pick::phase_t::down;
  else if (text == "drag")
    out = grid_pick::phase_t::drag;
  else if (text == "up")
    out = grid_pick::phase_t::up;
  else
    return false;
  return true;
}

void parse_grid_pick_mods(const std::string &text, grid_pick &out) {
  out.shift = out.ctrl = out.alt = false;
  std::size_t at = 0;
  while (at <= text.size()) {
    std::size_t end = text.find(',', at);
    if (end == std::string::npos)
      end = text.size();
    const std::string m = text.substr(at, end - at);
    if (m == "shift")
      out.shift = true;
    else if (m == "ctrl")
      out.ctrl = true;
    else if (m == "alt")
      out.alt = true;
    at = end + 1;
  }
}

grid_pick_outcome point_class_gestures::apply(point_class_editor &ed, grid_tool tool, long current,
                                              double scale, const grid_pick &p) {
  grid_pick_outcome out;
  const std::array<int, 3> &dims = ed.dims();
  const auto clamped = [&dims](const std::array<std::int64_t, 3> &v) {
    std::array<int, 3> c{};
    for (int k = 0; k < 3; ++k)
      c[k] = static_cast<int>(std::clamp<std::int64_t>(v[k], 0, dims[k] - 1));
    return c;
  };
  const auto navigate = [&](const grid_pick &q) {
    const std::array<int, 3> v = clamped(q.ijk);
    for (int k = 0; k < 3; ++k)
      if (k != static_cast<int>(q.axis))
        out.slice[k] = v[k];
  };
  const auto band = [&](const grid_pick &q) {
    out.band = true;
    out.band_axis = _axis;
    out.band_index = _start[static_cast<int>(_axis)];
    const std::pair<long long, long long> a = plane_of(_axis, _start), b = plane_of(_axis, q.ijk);
    out.band_rect = {a.first, a.second, b.first, b.second};
  };

  switch (p.phase) {
  case grid_pick::phase_t::down:
    _active = true;
    _axis = p.axis;
    _start = _last = p.ijk;
    _moved = false;
    if (tool == grid_tool::navigate) {
      _kind = kind::navigate;
      navigate(p);
    } else if (tool == grid_tool::points && p.shift && (p.ctrl || p.alt)) {
      _kind = kind::move;
    } else {
      _kind = kind::click;
      _mode = p.shift             ? selection_mode::add
              : (p.ctrl || p.alt) ? selection_mode::remove
                                  : selection_mode::replace;
    }
    return out;

  case grid_pick::phase_t::drag:
    if (!_active || p.axis != _axis)
      return out;
    if (_kind == kind::navigate) {
      navigate(p);
    } else if (tool == grid_tool::points && _kind == kind::move) {
      const std::pair<long long, long long> from = plane_of(_axis, _last),
                                            to = plane_of(_axis, p.ijk);
      const int dr = static_cast<int>(to.first - from.first),
                du = static_cast<int>(to.second - from.second);
      if (dr != 0 || du != 0) {
        if (!_moved) { // one undo step for the whole drag
          ed.checkpoint();
          _moved = true;
        }
        const std::pair<int, int> m = ed.move_selection(_axis, dr, du, false);
        const std::pair<int, int> ru = grid_plane_axes(_axis);
        _last[ru.first] += m.first;
        _last[ru.second] += m.second;
        if (m.first != 0 || m.second != 0)
          out.points_changed = out.selection_changed = true;
      }
    } else if (tool == grid_tool::points) {
      if (_kind == kind::click && p.ijk != _start)
        _kind = kind::band;
      if (_kind == kind::band)
        band(p);
      _last = p.ijk;
    }
    return out;

  case grid_pick::phase_t::up: {
    if (!_active)
      return out;
    _active = false;
    if (_kind == kind::navigate) {
      navigate(p);
      return out;
    }
    if (tool != grid_tool::points || p.axis != _axis)
      return out;
    if (_kind == kind::move) {
      if (_moved)
        out.note = "moved " + std::to_string(ed.selection().size()) + " point" +
                   (ed.selection().size() == 1 ? "" : "s");
      return out;
    }
    if (_kind == kind::click && p.ijk != _start)
      _kind = kind::band; // pressed and released on different voxels: a band, however quick
    const std::uint64_t sel = ed.selection_version();
    if (_kind == kind::band) {
      const std::pair<long long, long long> a = plane_of(_axis, _start), b = plane_of(_axis, p.ijk);
      const auto cut = [](long long v) {
        return static_cast<int>(std::clamp<long long>(v, -(1LL << 30), 1LL << 30));
      };
      const std::size_t n = ed.select_rect(_axis, _start[static_cast<int>(_axis)], cut(a.first),
                                           cut(a.second), cut(b.first), cut(b.second), _mode);
      out.note = std::to_string(n) + " point" + (n == 1 ? "" : "s") + " selected";
    } else {
      bool in = true;
      for (int k = 0; k < 3; ++k)
        in = in && _start[k] >= 0 && _start[k] < dims[k];
      if (!in)
        return out; // a click past the volume's edge
      const std::array<int, 3> v = clamped(_start);
      if (_mode != selection_mode::replace) { // Shift / Ctrl / Alt click: (de)select one point
        if (const std::optional<point_ref> r = ed.nearest(_axis, v, scale))
          ed.select(*r, _mode);
      } else if (current < 0 || !ed.shown(static_cast<std::size_t>(current))) {
        out.note = "no class to mark: add one, or pick one of the variable and timestep shown";
      } else {
        const std::size_t cls = static_cast<std::size_t>(current);
        point_ref which;
        const point_class_editor::toggled t = ed.toggle(cls, _axis, v, scale, &which);
        out.points_changed = true;
        out.note = (t == point_class_editor::toggled::added ? "added " : "removed ") +
                   voxel_text(which.ijk) +
                   (t == point_class_editor::toggled::added ? " to " : " from ") +
                   ed.classes().classes[cls].name;
      }
    }
    out.selection_changed = out.selection_changed || ed.selection_version() != sel;
    return out;
  }
  }
  return out;
}

// ---- class rows ---------------------------------------------------------------------------------

std::vector<point_class> read_point_class_rows(state &rows, int variable, int timestep,
                                               std::vector<std::string> *problems) {
  const auto problem = [problems](const std::string &m) {
    if (problems)
      problems->push_back(m);
  };
  std::vector<point_class> out;
  long long count = 0;
  const std::string n = rows.value();
  if (!n.empty() && (!param_detail::parse_integer(n, count) || count < 0)) {
    problem("classes: '" + n + "' is not a row count");
    count = 0;
  }
  count = std::min<long long>(count, 4096);
  const auto field = [&rows](long long i, const char *f) {
    state *s = rows.findDescendant(std::to_string(i) + "." + f);
    return s ? s->value() : std::string();
  };
  for (long long i = 0; i < count; ++i) {
    point_class c;
    c.name = field(i, "name");
    c.variable = variable;
    c.timestep = timestep;
    c.color = point_class_palette(static_cast<std::size_t>(i));
    const std::string row = "classes." + std::to_string(i);
    std::string why;
    if (const std::string t = field(i, "color"); !t.empty()) {
      std::vector<double> col;
      if (param_detail::value_from_text(t, col, why) && col.size() == 3)
        c.color = col;
      else
        problem(row + ".color: '" + t + "' is not r,g,b");
    }
    if (const std::string t = field(i, "radius"); !t.empty()) {
      double r = 0;
      if (param_detail::parse_double(t, r) && std::isfinite(r) && r > 0)
        c.radius = r;
      else
        problem(row + ".radius: '" + t + "' is not a positive number");
    }
    for (auto [f, dest] :
         {std::pair<const char *, int *>{"variable", &c.variable}, {"timestep", &c.timestep}}) {
      const std::string t = field(i, f);
      if (t.empty())
        continue;
      long long v = 0;
      if (param_detail::parse_integer(t, v) && v >= 0 && v <= 1 << 20)
        *dest = static_cast<int>(v);
      else
        problem(row + "." + f + ": '" + t + "' is not a whole number >= 0");
    }
    out.push_back(std::move(c));
  }
  return out;
}

void write_point_class_rows(state &rows, const point_classes &pc) {
  const auto put = [](state &s, const std::string &v) {
    if (s.value() != v)
      s.value(v);
  };
  const std::size_t n = pc.classes.size();
  for (std::size_t i = 0; i < n; ++i) {
    const point_class &c = pc.classes[i];
    state &row = rows(std::to_string(i));
    put(row("name"), c.name);
    std::string col;
    for (std::size_t k = 0; k < c.color.size(); ++k)
      col += (k ? "," : "") + num(c.color[k]);
    put(row("color"), col);
    put(row("radius"), num(c.radius));
    put(row("variable"), std::to_string(c.variable));
    put(row("timestep"), std::to_string(c.timestep));
    put(row("points"), std::to_string(c.size()));
  }
  for (const std::string &child : rows.childNames()) {
    long long i = 0;
    if (param_detail::parse_integer(child, i) && i >= static_cast<long long>(n))
      rows.removeChild(child);
  }
  put(rows, std::to_string(n)); // last: a watcher on the count sees the rows written
}

point_classes reconcile_point_class_rows(const point_classes &old,
                                         const std::vector<point_class> &rows) {
  point_classes out;
  out.classes.reserve(rows.size());
  if (rows.size() == old.classes.size()) {
    for (std::size_t i = 0; i < rows.size(); ++i) {
      point_class c = rows[i];
      c.points = old.classes[i].points;
      out.classes.push_back(std::move(c));
    }
    return out;
  }
  std::vector<bool> taken(old.classes.size(), false);
  for (const point_class &r : rows) {
    point_class c = r;
    c.points.clear();
    for (std::size_t k = 0; k < old.classes.size(); ++k)
      if (!taken[k] && old.classes[k].name == r.name && old.classes[k].variable == r.variable &&
          old.classes[k].timestep == r.timestep) {
        taken[k] = true;
        c.points = old.classes[k].points;
        break;
      }
    out.classes.push_back(std::move(c));
  }
  return out;
}

// ---- sessions -----------------------------------------------------------------------------------

bool volume_grid_session_key(const std::string &path) {
  if (path == "grid.classes.current" || path == "grid.objects.current")
    return true;
  if (path == "grid.contour.source")
    return false; // where the contours come from: the document's
  const auto in = [&path](const std::string &base, const std::vector<std::string> &names) {
    for (const std::string &n : names)
      if (path == base + n)
        return true;
    return false;
  };
  return in("grid.", param_names<volume_grid_params>()) ||
         in("grid.display.", param_names<volume_grid_display_params>()) ||
         in("grid.contour.", param_names<volume_grid_contour_params>()) ||
         in("grid.tile.", param_names<tile_params>()) ||
         in("grid.source.", {"var", "time"}) || // not where the data come from: the document's
         volume_grid_segment_session_key(path); // the Segmentation tab's parameters
}

namespace {

void json_string(const std::string &s, std::string &out) {
  out += '"';
  for (unsigned char ch : s) {
    switch (ch) {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\r':
      out += "\\r";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      if (ch < 0x20) {
        char b[8];
        std::snprintf(b, sizeof b, "\\u%04x", ch);
        out += b;
      } else {
        out += static_cast<char>(ch);
      }
    }
  }
  out += '"';
}

std::string text_of(const pt::ptree &t, const std::string &key, const std::string &what) {
  const auto it = t.find(key);
  if (it == t.not_found())
    throw std::runtime_error(what + ": no \"" + key + "\"");
  return it->second.data();
}

double number_of(const std::string &text, const std::string &what) {
  double d = 0;
  if (!param_detail::parse_double(text, d) || !std::isfinite(d))
    throw std::runtime_error(what + ": '" + text + "' is not a number");
  return d;
}

int int_of(const std::string &text, const std::string &what) {
  long long v = 0;
  if (!param_detail::parse_integer(text, v) || v < std::numeric_limits<int>::min() ||
      v > std::numeric_limits<int>::max())
    throw std::runtime_error(what + ": '" + text + "' is not a whole number");
  return static_cast<int>(v);
}

} // namespace

std::string volume_grid_session_json(const volume_grid_session &s) {
  std::string out = "{\n  \"format\": \"cvc.volume_grid.session\",\n  \"version\": 1,\n";
  out += "  \"keys\": {";
  bool first = true;
  for (const auto &[k, v] : s.keys) {
    out += first ? "\n    " : ",\n    ";
    first = false;
    json_string(k, out);
    out += ": ";
    json_string(v, out);
  }
  out += first ? "},\n" : "\n  },\n";
  out += "  \"transfer_function\": ";
  json_string(s.transfer_function, out);
  out += ",\n  \"classes\": [";
  for (std::size_t i = 0; i < s.classes.classes.size(); ++i) {
    const point_class &c = s.classes.classes[i];
    out += i ? ",\n    {" : "\n    {";
    out += "\"name\": ";
    json_string(c.name, out);
    out += ", \"color\": [";
    for (std::size_t k = 0; k < c.color.size(); ++k)
      out += (k ? ", " : "") + num(c.color[k]);
    out += "], \"radius\": " + num(c.radius) + ", \"variable\": " + std::to_string(c.variable) +
           ", \"timestep\": " + std::to_string(c.timestep) + ", \"points\": [";
    for (std::size_t k = 0; k < c.points.size(); ++k)
      out += (k ? ", " : "") + std::to_string(c.points[k]);
    out += "]}";
  }
  out += s.classes.classes.empty() ? "]" : "\n  ]";
  if (!s.contours.empty()) {
    std::string c = grid_contours_json(s.contours);
    while (!c.empty() && (c.back() == '\n' || c.back() == ' '))
      c.pop_back();
    out += ",\n  \"contours\": " + c;
  }
  out += "\n}\n";
  return out;
}

void write_volume_grid_session(const volume_grid_session &s, const std::string &filename) {
  std::vector<std::string> problems = point_class_problems(s.classes);
  if (problems.empty())
    problems = grid_contours_problems(s.contours);
  if (!problems.empty())
    throw std::invalid_argument("volume_grid session " + filename + ": " + problems.front());
  for (const auto &kv : s.keys)
    if (!volume_grid_session_key(kv.first))
      throw std::invalid_argument("volume_grid session " + filename + ": '" + kv.first +
                                  "' is not a key a session keeps");
  const std::string text = volume_grid_session_json(s);
  std::ofstream f(filename, std::ios::binary);
  if (!f || !(f << text) || !f.flush())
    throw std::runtime_error("volume_grid session " + filename + ": can't write it");
}

volume_grid_session volume_grid_session_from_json(const std::string &json,
                                                  const std::string &what) {
  pt::ptree root;
  try {
    std::istringstream in(json);
    pt::read_json(in, root);
  } catch (const std::exception &e) {
    throw std::runtime_error(what + ": not JSON (" + std::string(e.what()) + ")");
  }
  if (root.get<std::string>("format", "") != "cvc.volume_grid.session")
    throw std::runtime_error(what + ": not a volume_grid session (no \"format\": "
                                    "\"cvc.volume_grid.session\")");
  const int version = int_of(root.get<std::string>("version", "1"), what + ": version");
  if (version < 1 || version > 1)
    throw std::runtime_error(what + ": version " + std::to_string(version) +
                             " (this libcvc reads version 1)");
  volume_grid_session s;
  if (const auto keys = root.get_child_optional("keys")) {
    for (const auto &kv : *keys) {
      if (!kv.second.empty())
        throw std::runtime_error(what + ": keys." + kv.first + " is not a value");
      if (!volume_grid_session_key(kv.first))
        throw std::runtime_error(what + ": '" + kv.first + "' is not a key a session keeps");
      s.keys[kv.first] = kv.second.data();
    }
  }
  s.transfer_function = root.get<std::string>("transfer_function", "");
  if (const auto classes = root.get_child_optional("classes")) {
    std::size_t i = 0;
    for (const auto &item : *classes) {
      const std::string w = what + ": classes[" + std::to_string(i++) + "]";
      const pt::ptree &c = item.second;
      point_class pc;
      pc.name = text_of(c, "name", w);
      if (const auto col = c.get_child_optional("color")) {
        pc.color.clear();
        for (const auto &v : *col)
          pc.color.push_back(number_of(v.second.data(), w + ".color"));
      }
      if (const auto r = c.get_optional<std::string>("radius"))
        pc.radius = number_of(*r, w + ".radius");
      if (const auto v = c.get_optional<std::string>("variable"))
        pc.variable = int_of(*v, w + ".variable");
      if (const auto t = c.get_optional<std::string>("timestep"))
        pc.timestep = int_of(*t, w + ".timestep");
      if (const auto pts = c.get_child_optional("points"))
        for (const auto &v : *pts)
          pc.points.push_back(int_of(v.second.data(), w + ".points"));
      s.classes.classes.push_back(std::move(pc));
    }
  }
  if (const auto contours = root.get_child_optional("contours")) {
    // Back to text and through grid_contours_from_json: the tree keeps every value's text, so the
    // contours read back bit for bit.
    std::ostringstream text;
    pt::write_json(text, *contours, false);
    s.contours = grid_contours_from_json(text.str(), what + ": contours");
  }
  const std::vector<std::string> problems = point_class_problems(s.classes);
  if (!problems.empty())
    throw std::runtime_error(what + ": " + problems.front());
  return s;
}

volume_grid_session read_volume_grid_session(const std::string &filename) {
  std::ifstream f(filename, std::ios::binary);
  if (!f)
    throw std::runtime_error("volume_grid session " + filename + ": can't read it");
  std::ostringstream text;
  text << f.rdbuf();
  return volume_grid_session_from_json(text.str(), "volume_grid session " + filename);
}

} // namespace cvc
