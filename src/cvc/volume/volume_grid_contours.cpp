/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeGridRover's contours. See inc/cvc/volume/volume_grid_contours.h.

#include <algorithm>
#include <atomic>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cvc/state/state.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume_grid.h> // grid_plane_axes
#include <cvc/volume/volume_grid_contours.h>
#include <exception>
#include <iterator>
#include <set>
#include <sstream>
#include <type_traits>

namespace cvc {

namespace {

namespace pt = boost::property_tree;

std::string num(double d) { return param_detail::format_double(d); }

int ax(slice_axis a) { return static_cast<int>(a); }

const char *plane_name(slice_axis a) {
  switch (a) {
  case slice_axis::x:
    return "ZY";
  case slice_axis::y:
    return "XZ";
  case slice_axis::z:
    return "XY";
  }
  return "XY";
}

std::string where_curve(const grid_contours &c, std::size_t o, std::size_t k) {
  const grid_curve &cv = c.objects[o].curves[k];
  return "object '" + c.objects[o].name + "' curve " + std::to_string(k) + " (" +
         slice_axis_name(cv.axis) + " = " + std::to_string(cv.index) + ")";
}

bool finite3(const std::array<double, 3> &v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

} // namespace

// ---- the model ----------------------------------------------------------------------------------

double grid_frame::plane(slice_axis a, std::int64_t index) const {
  return origin[ax(a)] + static_cast<double>(index) * spacing[ax(a)];
}

std::int64_t grid_frame::nearest_index(slice_axis a, double coord) const {
  const double s = spacing[ax(a)];
  if (!(s > 0) || !std::isfinite(coord))
    return 0;
  return static_cast<std::int64_t>(std::llround((coord - origin[ax(a)]) / s));
}

grid_frame grid_frame_of(const slice_grid &g) {
  grid_frame f;
  for (int a = 0; a < 3; ++a) {
    f.dims[a] = static_cast<int>(std::max<std::uint64_t>(1, g.count(a)));
    f.origin[a] = g.lo(a);
    f.spacing[a] = g.spacing(a);
  }
  return f;
}

long grid_contours::object_index(const std::string &name) const {
  for (std::size_t i = 0; i < objects.size(); ++i)
    if (objects[i].name == name)
      return static_cast<long>(i);
  return -1;
}

std::size_t grid_contours::curve_count() const {
  std::size_t n = 0;
  for (const grid_contour_object &o : objects)
    n += o.curves.size();
  return n;
}

std::size_t grid_contours::point_count() const {
  std::size_t n = 0;
  for (const grid_contour_object &o : objects)
    for (const grid_curve &c : o.curves)
      n += c.size();
  return n;
}

std::vector<std::string> grid_contours_problems(const grid_contours &c) {
  std::vector<std::string> out;
  const grid_frame &f = c.frame;
  for (int a = 0; a < 3; ++a)
    if (f.dims[a] < 1)
      out.push_back("frame: dims[" + std::to_string(a) + "] is " + std::to_string(f.dims[a]) +
                    ", not 1 or more");
  if (!finite3(f.origin))
    out.push_back("frame: the origin isn't finite");
  for (int a = 0; a < 3; ++a)
    if (!(f.spacing[a] > 0) || !std::isfinite(f.spacing[a]))
      out.push_back("frame: spacing[" + std::to_string(a) + "] is " + num(f.spacing[a]) +
                    ", not positive and finite");
  std::set<std::string> names;
  for (std::size_t o = 0; o < c.objects.size(); ++o) {
    const grid_contour_object &obj = c.objects[o];
    const std::string w = "object " + std::to_string(o) + " ('" + obj.name + "')";
    if (obj.name.empty())
      out.push_back("object " + std::to_string(o) + ": no name");
    else if (!names.insert(obj.name).second)
      out.push_back(w + ": another object has that name");
    if (obj.color.size() != 3)
      out.push_back(w + ": colour of " + std::to_string(obj.color.size()) +
                    " values, not three (r, g, b)");
    else
      for (double v : obj.color)
        if (!(v >= 0.0 && v <= 1.0)) {
          out.push_back(w + ": colour value " + num(v) + " outside [0, 1]");
          break;
        }
    if (obj.samples < 1 || obj.samples > grid_contour_max_samples)
      out.push_back(w + ": samples " + std::to_string(obj.samples) + " outside [1, " +
                    std::to_string(grid_contour_max_samples) + "]");
    for (std::size_t k = 0; k < obj.curves.size(); ++k) {
      const grid_curve &cv = obj.curves[k];
      const std::string cw = where_curve(c, o, k);
      if (cv.ru.size() % 2)
        out.push_back(cw + ": an odd number of coordinates (" + std::to_string(cv.ru.size()) + ")");
      if (cv.ru.empty())
        out.push_back(cw + ": no points");
      for (double v : cv.ru)
        if (!std::isfinite(v)) {
          out.push_back(cw + ": a coordinate isn't finite");
          break;
        }
      if (cv.closed && cv.size() < 3)
        out.push_back(cw + ": closed with " + std::to_string(cv.size()) +
                      " points, fewer than three");
    }
  }
  return out;
}

std::vector<double> sample_grid_curve(const std::vector<double> &ru, bool closed,
                                      contour_interpolation interpolation, int samples) {
  if (ru.size() % 2)
    throw std::invalid_argument("sample_grid_curve: an odd number of coordinates (" +
                                std::to_string(ru.size()) + ")");
  if (samples < 1 || samples > grid_contour_max_samples)
    throw std::invalid_argument("sample_grid_curve: samples " + std::to_string(samples) +
                                " outside [1, " + std::to_string(grid_contour_max_samples) + "]");
  // The control points, with a point repeated in a row (and a closed curve's last at its first)
  // dropped: a span of no length has no direction for the spline.
  std::vector<std::array<double, 2>> p;
  for (std::size_t i = 0; i + 1 < ru.size(); i += 2) {
    const std::array<double, 2> q{ru[i], ru[i + 1]};
    if (p.empty() || p.back() != q)
      p.push_back(q);
  }
  if (closed && p.size() > 1 && p.back() == p.front())
    p.pop_back();
  const std::size_t n = p.size();
  if (interpolation == contour_interpolation::linear || samples == 1 || n < 3) {
    std::vector<double> out;
    out.reserve(2 * n);
    for (const auto &q : p)
      out.insert(out.end(), {q[0], q[1]});
    return out;
  }

  // Centripetal Catmull-Rom (Barry and Goldman's pyramid, alpha 1/2): through every control point,
  // with no cusp or self-intersection within a span, periodic on a closed curve. An open curve's
  // ends get a reflected neighbour.
  const auto at = [&](long i) -> std::array<double, 2> {
    const long m = static_cast<long>(n);
    if (closed)
      return p[static_cast<std::size_t>(((i % m) + m) % m)];
    if (i < 0)
      return {2 * p[0][0] - p[1][0], 2 * p[0][1] - p[1][1]};
    if (i >= m)
      return {2 * p[n - 1][0] - p[n - 2][0], 2 * p[n - 1][1] - p[n - 2][1]};
    return p[static_cast<std::size_t>(i)];
  };
  const auto knot = [](const std::array<double, 2> &a, const std::array<double, 2> &b) {
    const double d = std::sqrt(std::hypot(b[0] - a[0], b[1] - a[1]));
    return d > 0 ? d : 1e-12;
  };
  const auto lerp = [](const std::array<double, 2> &a, const std::array<double, 2> &b, double ta,
                       double tb, double t) -> std::array<double, 2> {
    const double w = (t - ta) / (tb - ta);
    return {a[0] + w * (b[0] - a[0]), a[1] + w * (b[1] - a[1])};
  };
  std::vector<double> out;
  const std::size_t spans = closed ? n : n - 1;
  out.reserve(2 * (spans * static_cast<std::size_t>(samples) + 1));
  for (std::size_t s = 0; s < spans; ++s) {
    const long i = static_cast<long>(s);
    const std::array<double, 2> p0 = at(i - 1), p1 = at(i), p2 = at(i + 1), p3 = at(i + 2);
    const double t0 = 0, t1 = t0 + knot(p0, p1), t2 = t1 + knot(p1, p2), t3 = t2 + knot(p2, p3);
    out.insert(out.end(), {p1[0], p1[1]});
    for (int j = 1; j < samples; ++j) {
      const double t = t1 + (t2 - t1) * j / samples;
      const std::array<double, 2> a1 = lerp(p0, p1, t0, t1, t), a2 = lerp(p1, p2, t1, t2, t),
                                  a3 = lerp(p2, p3, t2, t3, t);
      const std::array<double, 2> b1 = lerp(a1, a2, t0, t2, t), b2 = lerp(a2, a3, t1, t3, t);
      const std::array<double, 2> c = lerp(b1, b2, t1, t2, t);
      out.insert(out.end(), {c[0], c[1]});
    }
  }
  if (!closed)
    out.insert(out.end(), {p[n - 1][0], p[n - 1][1]});
  return out;
}

// ---- stacks -------------------------------------------------------------------------------------

contour_stack grid_contours_stack(const grid_contours &c, slice_axis axis, std::size_t *open) {
  const std::vector<std::string> problems = grid_contours_problems(c);
  if (!problems.empty())
    throw std::invalid_argument("grid_contours_stack: " + problems.front());
  contour_stack s;
  s.z_origin = c.frame.origin[ax(axis)];
  s.z_spacing = c.frame.spacing[ax(axis)];
  std::size_t skipped = 0;
  for (const grid_contour_object &o : c.objects) {
    contour_component cc;
    cc.name = o.name;
    cc.color = o.color;
    cc.visible = o.visible;
    s.components.push_back(cc);
    for (const grid_curve &cv : o.curves) {
      if (cv.axis != axis)
        continue;
      if (!cv.closed) {
        ++skipped;
        continue;
      }
      contour_polygon poly;
      poly.xy = sample_grid_curve(cv.ru, true, o.interpolation, o.samples);
      if (poly.size() < 3) // its points coincide: nothing to enclose
        continue;
      const int idx = static_cast<int>(std::clamp<std::int64_t>(
          cv.index, std::numeric_limits<int>::min() / 2, std::numeric_limits<int>::max() / 2));
      s.slices[idx].components[o.name].push_back(std::move(poly));
    }
  }
  if (open)
    *open = skipped;
  return s;
}

grid_contours grid_contours_from_stack(const contour_stack &s, const grid_frame &frame,
                                       slice_axis axis, std::vector<std::string> *warnings) {
  const std::vector<std::string> problems = contour_problems(s);
  if (!problems.empty())
    throw std::invalid_argument("grid_contours_from_stack: " + problems.front());
  grid_contours c;
  c.frame = frame;
  for (std::size_t i = 0; i < s.components.size(); ++i) {
    grid_contour_object o;
    o.name = s.components[i].name;
    o.color = s.components[i].color;
    o.visible = s.components[i].visible;
    c.objects.push_back(std::move(o));
  }
  const double sp = frame.spacing[ax(axis)];
  for (const auto &[index, slice] : s.slices) {
    const double z = s.slice_z(index);
    const std::int64_t k = frame.nearest_index(axis, z);
    if (warnings) {
      const double off = std::abs(z - frame.plane(axis, k));
      if (off > 0.01 * sp || k < 0 || k >= frame.dims[ax(axis)])
        warnings->push_back("slice " + std::to_string(index) + " (z = " + num(z) + ") is put on " +
                            slice_axis_name(axis) + " = " + std::to_string(k) +
                            (k < 0 || k >= frame.dims[ax(axis)]
                                 ? ", past the volume"
                                 : ", off its plane by " + num(off)));
    }
    for (const auto &[name, polys] : slice.components) {
      const long o = c.object_index(name);
      if (o < 0)
        continue; // contour_problems reports a name with no component
      for (const contour_polygon &poly : polys) {
        grid_curve cv;
        cv.axis = axis;
        cv.index = k;
        cv.ru = poly.xy;
        cv.closed = true;
        c.objects[static_cast<std::size_t>(o)].curves.push_back(std::move(cv));
      }
    }
  }
  return c;
}

// ---- tracing ------------------------------------------------------------------------------------

namespace {

using vec2 = std::array<double, 2>;

double point_line_distance(const vec2 &p, const vec2 &a, const vec2 &b) {
  const double dx = b[0] - a[0], dy = b[1] - a[1];
  const double len = std::hypot(dx, dy);
  if (len == 0)
    return std::hypot(p[0] - a[0], p[1] - a[1]);
  return std::abs(dx * (a[1] - p[1]) - dy * (a[0] - p[0])) / len;
}

// Douglas-Peucker on pts[i..j] (both kept), marking what stays.
void simplify(const std::vector<vec2> &pts, std::size_t i, std::size_t j, double tol,
              std::vector<bool> &keep) {
  if (j <= i + 1)
    return;
  double worst = -1;
  std::size_t at = i;
  for (std::size_t k = i + 1; k < j; ++k) {
    const double d = point_line_distance(pts[k], pts[i], pts[j]);
    if (d > worst) {
      worst = d;
      at = k;
    }
  }
  if (worst > tol) {
    keep[at] = true;
    simplify(pts, i, at, tol, keep);
    simplify(pts, at, j, tol, keep);
  }
}

double loop_area(const std::vector<vec2> &p) {
  double a = 0;
  for (std::size_t i = 0, n = p.size(); i < n; ++i) {
    const vec2 &q = p[i], &r = p[(i + 1) % n];
    a += q[0] * r[1] - r[0] * q[1];
  }
  return 0.5 * a;
}

} // namespace

std::vector<std::vector<double>> trace_slice(const volume_slice_data &s, const grid_frame &f,
                                             double iso, double tolerance, double min_area) {
  if (!std::isfinite(iso))
    throw std::invalid_argument("trace_slice: the isovalue " + num(iso) + " isn't finite");
  if (!(tolerance >= 0) || !(min_area >= 0))
    throw std::invalid_argument("trace_slice: a negative tolerance or area");
  // The samples with a border of one past the edge, counted as above iso, so every loop closes.
  const long W = static_cast<long>(s.width) + 2, H = static_cast<long>(s.height) + 2;
  if (s.width == 0 || s.height == 0)
    return {};
  const auto value = [&](long c, long r) {
    if (c < 1 || r < 1 || c > static_cast<long>(s.width) || r > static_cast<long>(s.height))
      return iso + 1.0;
    const double v = s.at(static_cast<std::uint64_t>(c - 1), static_cast<std::uint64_t>(r - 1));
    return std::isnan(v) ? iso + 1.0 : v;
  };
  const auto in = [&](long c, long r) { return value(c, r) < iso; };
  // Edges: horizontal (c, r)-(c+1, r) and vertical (c, r)-(c, r+1), each its id.
  const auto h_edge = [W](long c, long r) { return 2 * (r * W + c); };
  const auto v_edge = [W](long c, long r) { return 2 * (r * W + c) + 1; };
  std::map<long, vec2> at;
  const auto cross = [&](long id) {
    const auto it = at.find(id);
    if (it != at.end())
      return;
    const long cell = id / 2, c = cell % W, r = cell / W;
    const bool vertical = id % 2;
    const double v0 = value(c, r), v1 = vertical ? value(c, r + 1) : value(c + 1, r);
    const double t = v1 != v0 ? std::clamp((iso - v0) / (v1 - v0), 0.0, 1.0) : 0.5;
    at[id] = vertical ? vec2{double(c), r + t} : vec2{c + t, double(r)};
  };
  std::vector<std::pair<long, long>> segs;
  for (long r = 0; r + 1 < H; ++r)
    for (long c = 0; c + 1 < W; ++c) {
      const int k = (in(c, r) ? 1 : 0) | (in(c + 1, r) ? 2 : 0) | (in(c + 1, r + 1) ? 4 : 0) |
                    (in(c, r + 1) ? 8 : 0);
      if (k == 0 || k == 15)
        continue;
      const long B = h_edge(c, r), R = v_edge(c + 1, r), T = h_edge(c, r + 1), L = v_edge(c, r);
      const auto seg = [&](long a, long b) {
        cross(a);
        cross(b);
        segs.emplace_back(a, b);
      };
      const bool centre =
          0.25 * (value(c, r) + value(c + 1, r) + value(c + 1, r + 1) + value(c, r + 1)) < iso;
      switch (k) {
      case 1:
      case 14:
        seg(L, B);
        break;
      case 2:
      case 13:
        seg(B, R);
        break;
      case 3:
      case 12:
        seg(L, R);
        break;
      case 4:
      case 11:
        seg(R, T);
        break;
      case 6:
      case 9:
        seg(B, T);
        break;
      case 7:
      case 8:
        seg(L, T);
        break;
      case 5: // bl and tr below
        if (centre) {
          seg(B, R);
          seg(L, T);
        } else {
          seg(L, B);
          seg(R, T);
        }
        break;
      case 10: // br and tl below
        if (centre) {
          seg(L, B);
          seg(R, T);
        } else {
          seg(B, R);
          seg(L, T);
        }
        break;
      }
    }
  // Chain the segments into loops: every edge crossed is in two of them.
  std::map<long, std::vector<std::size_t>> by_edge;
  for (std::size_t i = 0; i < segs.size(); ++i) {
    by_edge[segs[i].first].push_back(i);
    by_edge[segs[i].second].push_back(i);
  }
  std::vector<bool> used(segs.size(), false);
  struct loop {
    std::vector<double> ru;
    double area;
  };
  std::vector<loop> loops;
  const std::pair<int, int> axes = grid_plane_axes(s.axis);
  const double sr = f.spacing[axes.first], su = f.spacing[axes.second];
  for (std::size_t start = 0; start < segs.size(); ++start) {
    if (used[start])
      continue;
    std::vector<vec2> pts;
    used[start] = true;
    const long first = segs[start].first;
    long edge = segs[start].second;
    pts.push_back(at[first]);
    while (edge != first) {
      pts.push_back(at[edge]);
      std::size_t next = segs.size();
      for (std::size_t si : by_edge[edge])
        if (!used[si]) {
          next = si;
          break;
        }
      if (next == segs.size())
        break; // cannot happen with the border; a loop left open is dropped below
      used[next] = true;
      edge = segs[next].first == edge ? segs[next].second : segs[next].first;
    }
    if (edge != first || pts.size() < 3)
      continue;
    // Simplified between the first point and the one farthest from it.
    std::size_t opp = 0;
    double opp_d = -1;
    for (std::size_t i = 1; i < pts.size(); ++i) {
      const double d = std::hypot(pts[i][0] - pts[0][0], pts[i][1] - pts[0][1]);
      if (d > opp_d) {
        opp_d = d;
        opp = i;
      }
    }
    std::vector<vec2> closed = pts;
    closed.push_back(pts[0]);
    std::vector<bool> keep(closed.size(), false);
    keep[0] = keep[opp] = keep[closed.size() - 1] = true;
    simplify(closed, 0, opp, tolerance, keep);
    simplify(closed, opp, closed.size() - 1, tolerance, keep);
    std::vector<vec2> kept;
    for (std::size_t i = 0; i + 1 < closed.size(); ++i)
      if (keep[i])
        kept.push_back(closed[i]);
    double area = loop_area(kept);
    if (kept.size() < 3 || std::abs(area) < std::max(min_area, 1e-12))
      continue;
    if (area < 0) {
      std::reverse(kept.begin(), kept.end());
      area = -area;
    }
    loop l;
    l.area = area;
    for (const vec2 &p : kept)
      l.ru.insert(l.ru.end(), {f.origin[axes.first] + (p[0] - 1) * sr,
                               f.origin[axes.second] + (p[1] - 1) * su});
    loops.push_back(std::move(l));
  }
  std::stable_sort(loops.begin(), loops.end(),
                   [](const loop &a, const loop &b) { return a.area > b.area; });
  std::vector<std::vector<double>> out;
  for (loop &l : loops)
    out.push_back(std::move(l.ru));
  return out;
}

// ---- the editor ---------------------------------------------------------------------------------

void grid_contour_editor::add_curves(std::size_t object, std::vector<grid_curve> curves) {
  if (object >= _c.objects.size())
    throw std::out_of_range("add_curves: no object " + std::to_string(object) + " (there are " +
                            std::to_string(_c.objects.size()) + ")");
  if (curves.empty())
    return;
  grid_contours next = _c;
  for (grid_curve &cv : curves)
    next.objects[object].curves.push_back(std::move(cv));
  const std::vector<std::string> problems = grid_contours_problems(next);
  if (!problems.empty())
    throw std::invalid_argument("add_curves: " + problems.front());
  push_undo();
  _c = std::move(next);
  changed();
}

void grid_contour_editor::set_frame(const grid_frame &f) {
  if (f == _c.frame)
    return;
  _c.frame = f;
  changed();
}

void grid_contour_editor::set_contours(grid_contours c, bool undoable) {
  const std::vector<std::string> problems = grid_contours_problems(c);
  if (!problems.empty())
    throw std::invalid_argument("contours: " + problems.front());
  if (undoable)
    push_undo();
  _c = std::move(c);
  _active.reset();
  if (!_selection.empty()) {
    _selection.clear();
    ++_selection_version;
  }
  changed();
}

std::size_t grid_contour_editor::add_object(grid_contour_object o) {
  if (o.name.empty())
    throw std::invalid_argument("add_object: an object needs a name");
  if (_c.object_index(o.name) >= 0)
    throw std::invalid_argument("add_object: there is already an object '" + o.name + "'");
  grid_contours next = _c;
  next.objects.push_back(std::move(o));
  const std::vector<std::string> problems = grid_contours_problems(next);
  if (!problems.empty())
    throw std::invalid_argument("add_object: " + problems.front());
  push_undo();
  _c = std::move(next);
  changed();
  return _c.objects.size() - 1;
}

void grid_contour_editor::set_objects(std::vector<grid_contour_object> objects, bool undoable) {
  grid_contours next = _c;
  next.objects = std::move(objects);
  const std::vector<std::string> problems = grid_contours_problems(next);
  if (!problems.empty())
    throw std::invalid_argument("objects: " + problems.front());
  if (next == _c)
    return;
  if (undoable)
    push_undo();
  const bool same_shape = next.objects.size() == _c.objects.size();
  _c = std::move(next);
  if (!same_shape) {
    _active.reset();
    if (!_selection.empty()) {
      _selection.clear();
      ++_selection_version;
    }
  }
  drop_unselectable();
  changed();
}

std::vector<grid_curve_ref> grid_contour_editor::curves_on(slice_axis a, std::int64_t index,
                                                           bool visible) const {
  std::vector<grid_curve_ref> out;
  for (std::size_t o = 0; o < _c.objects.size(); ++o) {
    if (visible && !_c.objects[o].visible)
      continue;
    for (std::size_t k = 0; k < _c.objects[o].curves.size(); ++k) {
      const grid_curve &cv = _c.objects[o].curves[k];
      if (cv.axis == a && cv.index == index)
        out.push_back({o, k});
    }
  }
  return out;
}

const grid_curve &grid_contour_editor::curve(const grid_curve_ref &r) const {
  return _c.objects.at(r.object).curves.at(r.curve);
}

std::array<double, 2> grid_contour_editor::point(const grid_point_ref &r) const {
  const grid_curve &cv = curve(r.curve_ref());
  if (r.point >= cv.size())
    throw std::out_of_range("point: curve has " + std::to_string(cv.size()) + " points");
  return {cv.ru[2 * r.point], cv.ru[2 * r.point + 1]};
}

std::optional<grid_point_ref> grid_contour_editor::nearest_point(slice_axis a, std::int64_t index,
                                                                 double r, double u, double radius,
                                                                 long object) const {
  std::optional<grid_point_ref> best;
  double best_d = radius * radius;
  for (const grid_curve_ref &cr : curves_on(a, index)) {
    if (object >= 0 && cr.object != static_cast<std::size_t>(object))
      continue;
    const grid_curve &cv = curve(cr);
    for (std::size_t i = 0; i < cv.size(); ++i) {
      const double dr = cv.ru[2 * i] - r, du = cv.ru[2 * i + 1] - u;
      const double d = dr * dr + du * du;
      if (d <= best_d && (!best || d < best_d)) {
        best_d = d;
        best = grid_point_ref{cr.object, cr.curve, i};
      }
    }
  }
  return best;
}

void grid_contour_editor::set_active(std::optional<active_curve> a) {
  if (a) {
    if (a->curve.object >= _c.objects.size() ||
        a->curve.curve >= _c.objects[a->curve.object].curves.size() ||
        _c.objects[a->curve.object].curves[a->curve.curve].closed)
      a.reset();
  }
  _active = a;
}

grid_point_ref grid_contour_editor::add_point(std::size_t object, slice_axis a, std::int64_t index,
                                              double r, double u, bool new_curve) {
  if (object >= _c.objects.size())
    throw std::out_of_range("add_point: no object " + std::to_string(object) + " (there are " +
                            std::to_string(_c.objects.size()) + ")");
  if (!std::isfinite(r) || !std::isfinite(u))
    throw std::invalid_argument("add_point: (" + num(r) + ", " + num(u) + ") isn't finite");
  std::vector<grid_curve> &curves = _c.objects[object].curves;
  const auto open_here = [&](std::size_t k) {
    return k < curves.size() && !curves[k].closed && curves[k].axis == a &&
           curves[k].index == index && curves[k].size() > 0;
  };
  long target = -1;
  bool back = true;
  if (!new_curve) {
    if (_active && _active->curve.object == object && open_here(_active->curve.curve)) {
      target = static_cast<long>(_active->curve.curve);
      back = _active->back;
    } else {
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t k = 0; k < curves.size(); ++k) {
        if (!open_here(k))
          continue;
        const grid_curve &cv = curves[k];
        const std::size_t last = cv.size() - 1;
        const double df = std::hypot(cv.ru[0] - r, cv.ru[1] - u);
        const double db = std::hypot(cv.ru[2 * last] - r, cv.ru[2 * last + 1] - u);
        if (db < best) { // the back first: a one-point curve grows after its point
          best = db;
          target = static_cast<long>(k);
          back = true;
        }
        if (df < best) {
          best = df;
          target = static_cast<long>(k);
          back = false;
        }
      }
    }
  }
  push_undo();
  grid_point_ref ref;
  ref.object = object;
  if (target < 0) {
    grid_curve cv;
    cv.axis = a;
    cv.index = index;
    cv.ru = {r, u};
    curves.push_back(std::move(cv));
    ref.curve = curves.size() - 1;
    ref.point = 0;
    back = true;
  } else {
    grid_curve &cv = curves[static_cast<std::size_t>(target)];
    ref.curve = static_cast<std::size_t>(target);
    if (back) {
      cv.ru.insert(cv.ru.end(), {r, u});
      ref.point = cv.size() - 1;
    } else {
      cv.ru.insert(cv.ru.begin(), {r, u});
      ref.point = 0;
    }
  }
  _active = active_curve{ref.curve_ref(), back};
  _selection.assign(1, ref);
  ++_selection_version;
  changed();
  return ref;
}

bool grid_contour_editor::close_curve(const grid_curve_ref &c) {
  if (c.object >= _c.objects.size() || c.curve >= _c.objects[c.object].curves.size())
    return false;
  grid_curve &cv = _c.objects[c.object].curves[c.curve];
  if (cv.closed || cv.size() < 3)
    return false;
  push_undo();
  _c.objects[c.object].curves[c.curve].closed = true;
  if (_active && _active->curve == c)
    _active.reset();
  changed();
  return true;
}

bool grid_contour_editor::close_active() { return _active && close_curve(_active->curve); }

bool grid_contour_editor::open_curve(const grid_curve_ref &c) {
  if (c.object >= _c.objects.size() || c.curve >= _c.objects[c.object].curves.size() ||
      !_c.objects[c.object].curves[c.curve].closed)
    return false;
  push_undo();
  _c.objects[c.object].curves[c.curve].closed = false;
  _active = active_curve{c, true};
  changed();
  return true;
}

bool grid_contour_editor::delete_curve(const grid_curve_ref &c) {
  if (c.object >= _c.objects.size() || c.curve >= _c.objects[c.object].curves.size())
    return false;
  push_undo();
  std::vector<grid_curve> &curves = _c.objects[c.object].curves;
  curves.erase(curves.begin() + static_cast<long>(c.curve));
  // The selection and the active curve follow the curves after it down one.
  std::vector<grid_point_ref> sel;
  for (grid_point_ref p : _selection) {
    if (p.object == c.object && p.curve == c.curve)
      continue;
    if (p.object == c.object && p.curve > c.curve)
      --p.curve;
    sel.push_back(p);
  }
  if (sel != _selection) {
    _selection.swap(sel);
    ++_selection_version;
  }
  if (_active && _active->curve.object == c.object) {
    if (_active->curve.curve == c.curve)
      _active.reset();
    else if (_active->curve.curve > c.curve)
      --_active->curve.curve;
  }
  changed();
  return true;
}

std::size_t grid_contour_editor::clear_object(std::size_t object) {
  if (object >= _c.objects.size() || _c.objects[object].curves.empty())
    return 0;
  push_undo();
  const std::size_t n = _c.objects[object].curves.size();
  _c.objects[object].curves.clear();
  const std::size_t before = _selection.size();
  _selection.erase(std::remove_if(_selection.begin(), _selection.end(),
                                  [object](const grid_point_ref &p) { return p.object == object; }),
                   _selection.end());
  if (_selection.size() != before)
    ++_selection_version;
  if (_active && _active->curve.object == object)
    _active.reset();
  changed();
  return n;
}

bool grid_contour_editor::remove_object(std::size_t object) {
  if (object >= _c.objects.size())
    return false;
  push_undo();
  _c.objects.erase(_c.objects.begin() + static_cast<long>(object));
  std::vector<grid_point_ref> sel;
  for (grid_point_ref p : _selection) {
    if (p.object == object)
      continue;
    if (p.object > object)
      --p.object;
    sel.push_back(p);
  }
  if (sel != _selection) {
    _selection.swap(sel);
    ++_selection_version;
  }
  if (_active) {
    if (_active->curve.object == object)
      _active.reset();
    else if (_active->curve.object > object)
      --_active->curve.object;
  }
  changed();
  return true;
}

void grid_contour_editor::move_point(const grid_point_ref &p, double r, double u, bool undoable) {
  const grid_curve &cv = curve(p.curve_ref());
  if (p.point >= cv.size())
    throw std::out_of_range("move_point: curve has " + std::to_string(cv.size()) + " points");
  if (!std::isfinite(r) || !std::isfinite(u))
    throw std::invalid_argument("move_point: (" + num(r) + ", " + num(u) + ") isn't finite");
  if (cv.ru[2 * p.point] == r && cv.ru[2 * p.point + 1] == u)
    return;
  if (undoable)
    push_undo();
  grid_curve &m = _c.objects[p.object].curves[p.curve];
  m.ru[2 * p.point] = r;
  m.ru[2 * p.point + 1] = u;
  changed();
}

void grid_contour_editor::move_selection(double dr, double du, bool undoable) {
  if (_selection.empty() || (dr == 0 && du == 0) || !std::isfinite(dr) || !std::isfinite(du))
    return;
  if (undoable)
    push_undo();
  for (const grid_point_ref &p : _selection) {
    grid_curve &m = _c.objects[p.object].curves[p.curve];
    m.ru[2 * p.point] += dr;
    m.ru[2 * p.point + 1] += du;
  }
  changed();
}

bool grid_contour_editor::selected(const grid_point_ref &p) const {
  return std::binary_search(_selection.begin(), _selection.end(), p);
}

void grid_contour_editor::select(const grid_point_ref &p, contour_selection_mode m) {
  if (p.object >= _c.objects.size() || !_c.objects[p.object].visible ||
      p.curve >= _c.objects[p.object].curves.size() ||
      p.point >= _c.objects[p.object].curves[p.curve].size())
    return;
  const auto it = std::lower_bound(_selection.begin(), _selection.end(), p);
  const bool in = it != _selection.end() && *it == p;
  switch (m) {
  case contour_selection_mode::replace:
    if (_selection.size() == 1 && in)
      return;
    _selection.assign(1, p);
    break;
  case contour_selection_mode::add:
    if (in)
      return;
    _selection.insert(it, p);
    break;
  case contour_selection_mode::remove:
    if (!in)
      return;
    _selection.erase(it);
    break;
  }
  ++_selection_version;
}

std::size_t grid_contour_editor::select_rect(slice_axis a, std::int64_t index, double r0, double u0,
                                             double r1, double u1, contour_selection_mode m) {
  const double rl = std::min(r0, r1), rh = std::max(r0, r1);
  const double ul = std::min(u0, u1), uh = std::max(u0, u1);
  std::vector<grid_point_ref> in;
  for (const grid_curve_ref &cr : curves_on(a, index)) {
    const grid_curve &cv = curve(cr);
    for (std::size_t i = 0; i < cv.size(); ++i) {
      const double r = cv.ru[2 * i], u = cv.ru[2 * i + 1];
      if (r >= rl && r <= rh && u >= ul && u <= uh)
        in.push_back({cr.object, cr.curve, i});
    }
  }
  std::vector<grid_point_ref> next;
  switch (m) {
  case contour_selection_mode::replace:
    next = in;
    break;
  case contour_selection_mode::add:
    std::set_union(_selection.begin(), _selection.end(), in.begin(), in.end(),
                   std::back_inserter(next));
    break;
  case contour_selection_mode::remove:
    std::set_difference(_selection.begin(), _selection.end(), in.begin(), in.end(),
                        std::back_inserter(next));
    break;
  }
  if (next != _selection) {
    _selection.swap(next);
    ++_selection_version;
  }
  return m == contour_selection_mode::remove ? _selection.size() : in.size();
}

void grid_contour_editor::select_all() {
  std::vector<grid_point_ref> all;
  for (std::size_t o = 0; o < _c.objects.size(); ++o) {
    if (!_c.objects[o].visible)
      continue;
    for (std::size_t k = 0; k < _c.objects[o].curves.size(); ++k)
      for (std::size_t i = 0; i < _c.objects[o].curves[k].size(); ++i)
        all.push_back({o, k, i});
  }
  if (all != _selection) {
    _selection.swap(all);
    ++_selection_version;
  }
}

void grid_contour_editor::clear_selection() {
  if (_selection.empty())
    return;
  _selection.clear();
  ++_selection_version;
}

std::size_t grid_contour_editor::delete_selected() {
  if (_selection.empty())
    return 0;
  push_undo();
  const std::size_t n = _selection.size();
  // Points go from the back of each curve, so the indices before them hold.
  for (auto it = _selection.rbegin(); it != _selection.rend(); ++it) {
    std::vector<double> &ru = _c.objects[it->object].curves[it->curve].ru;
    ru.erase(ru.begin() + static_cast<long>(2 * it->point),
             ru.begin() + static_cast<long>(2 * it->point + 2));
  }
  // Empty curves go; a closed curve left too small opens. The active curve follows its curve.
  std::optional<active_curve> act = _active;
  for (std::size_t o = 0; o < _c.objects.size(); ++o) {
    std::vector<grid_curve> &curves = _c.objects[o].curves;
    std::vector<grid_curve> kept;
    for (std::size_t k = 0; k < curves.size(); ++k) {
      grid_curve &cv = curves[k];
      const bool here = act && act->curve.object == o && act->curve.curve == k;
      if (cv.ru.empty()) {
        if (here)
          act.reset();
        continue;
      }
      if (cv.closed && cv.size() < 3)
        cv.closed = false;
      if (here)
        act->curve.curve = kept.size();
      kept.push_back(std::move(cv));
    }
    curves.swap(kept);
  }
  _active = act;
  if (_active && _c.objects[_active->curve.object].curves[_active->curve.curve].closed)
    _active.reset();
  _selection.clear();
  ++_selection_version;
  changed();
  return n;
}

void grid_contour_editor::checkpoint() { push_undo(); }

bool grid_contour_editor::undo() {
  if (_undo.empty())
    return false;
  _c = std::move(_undo.back());
  _undo.pop_back();
  _active.reset();
  if (!_selection.empty()) {
    _selection.clear();
    ++_selection_version;
  }
  changed();
  return true;
}

void grid_contour_editor::push_undo() {
  if (_undo.size() >= undo_limit)
    _undo.pop_front();
  _undo.push_back(_c);
}

void grid_contour_editor::changed() { ++_version; }

void grid_contour_editor::drop_unselectable() {
  const std::size_t before = _selection.size();
  _selection.erase(std::remove_if(_selection.begin(), _selection.end(),
                                  [this](const grid_point_ref &p) {
                                    return p.object >= _c.objects.size() ||
                                           !_c.objects[p.object].visible ||
                                           p.curve >= _c.objects[p.object].curves.size() ||
                                           p.point >= _c.objects[p.object].curves[p.curve].size();
                                  }),
                   _selection.end());
  if (_selection.size() != before)
    ++_selection_version;
  if (_active && (_active->curve.object >= _c.objects.size() ||
                  _active->curve.curve >= _c.objects[_active->curve.object].curves.size()))
    _active.reset();
}

// ---- gestures -----------------------------------------------------------------------------------

contour_pick_outcome grid_contour_gestures::apply(grid_contour_editor &ed, long current,
                                                  double radius, bool snap, const contour_pick &p) {
  contour_pick_outcome out;
  const grid_frame &f = ed.frame();
  const std::pair<int, int> ru = grid_plane_axes(p.axis);
  const auto snapped = [&](double r, double u) -> std::array<double, 2> {
    if (!snap)
      return {r, u};
    const double sr = f.spacing[ru.first], su = f.spacing[ru.second];
    return {f.origin[ru.first] + std::round((r - f.origin[ru.first]) / sr) * sr,
            f.origin[ru.second] + std::round((u - f.origin[ru.second]) / su) * su};
  };
  const auto plane_text = [&](const contour_pick &q) {
    return std::string(slice_axis_name(q.axis)) + " = " + std::to_string(q.index);
  };
  // A click on a control point: the active curve's other end closes it; any other point is
  // selected, and an open curve's end becomes the active end.
  const auto click_point = [&](const grid_point_ref &on) {
    const grid_curve &cv = ed.curve(on.curve_ref());
    const std::optional<grid_contour_editor::active_curve> &act = ed.active();
    const std::size_t last = cv.size() - 1;
    if (act && act->curve == on.curve_ref() && !cv.closed && cv.size() >= 3 &&
        on.point == (act->back ? 0 : last)) {
      ed.close_curve(on.curve_ref());
      out.changed = true;
      out.note = "closed a curve of " + ed.contours().objects[on.object].name + " on " +
                 plane_text(p) + " (" + std::to_string(cv.size()) + " points)";
      return;
    }
    ed.select(on, contour_selection_mode::replace);
    if (!cv.closed && (on.point == 0 || on.point == last))
      ed.set_active(grid_contour_editor::active_curve{on.curve_ref(), on.point == last});
    else
      ed.set_active(std::nullopt);
    out.object = static_cast<long>(on.object);
  };

  switch (p.phase) {
  case contour_pick::phase_t::down:
    _active = true;
    _axis = p.axis;
    _index = p.index;
    _r0 = _lr = p.r;
    _u0 = _lu = p.u;
    _moved = false;
    _mode = p.shift             ? contour_selection_mode::add
            : (p.ctrl || p.alt) ? contour_selection_mode::remove
                                : contour_selection_mode::replace;
    _on = ed.nearest_point(p.axis, p.index, p.r, p.u, radius);
    _kind = _on && _mode == contour_selection_mode::replace ? kind::move : kind::click;
    return out;

  case contour_pick::phase_t::drag: {
    if (!_active || p.axis != _axis || p.index != _index)
      return out;
    const double travel = std::hypot(p.r - _r0, p.u - _u0);
    if (_kind == kind::move) {
      if (!_moved && travel <= 0.25 * radius)
        return out; // a hand's tremor, not a drag
      if (!_moved) {
        ed.checkpoint(); // one undo step for the whole drag
        _moved = true;
      }
      if (ed.selected(*_on) && ed.selection().size() > 1) {
        double dr = p.r - _lr, du = p.u - _lu;
        if (snap) {
          const double sr = f.spacing[ru.first], su = f.spacing[ru.second];
          dr = std::round(dr / sr) * sr;
          du = std::round(du / su) * su;
        }
        if (dr != 0 || du != 0) {
          ed.move_selection(dr, du, false);
          _lr += dr;
          _lu += du;
          out.changed = true;
        }
      } else {
        const std::array<double, 2> to = snapped(p.r, p.u);
        ed.move_point(*_on, to[0], to[1], false);
        _lr = p.r;
        _lu = p.u;
        out.changed = true;
      }
      return out;
    }
    if (_kind == kind::click && travel > 0.5 * radius)
      _kind = kind::band;
    if (_kind == kind::band) {
      out.band = true;
      out.band_axis = _axis;
      out.band_index = _index;
      out.band_rect = {_r0, _u0, p.r, p.u};
    }
    _lr = p.r;
    _lu = p.u;
    return out;
  }

  case contour_pick::phase_t::up: {
    if (!_active)
      return out;
    _active = false;
    if (p.axis != _axis || p.index != _index)
      return out;
    const std::uint64_t sel = ed.selection_version();
    const double travel = std::hypot(p.r - _r0, p.u - _u0);
    if (_kind == kind::move) {
      if (_moved) {
        out.changed = true;
        out.note = ed.selected(*_on) && ed.selection().size() > 1
                       ? "moved " + std::to_string(ed.selection().size()) + " points"
                       : "moved a point of " + ed.contours().objects[_on->object].name;
      } else {
        click_point(*_on);
      }
    } else if (_kind == kind::band || travel > 0.5 * radius) {
      const std::size_t n = ed.select_rect(_axis, _index, _r0, _u0, p.r, p.u, _mode);
      out.note = std::to_string(n) + " point" + (n == 1 ? "" : "s") + " selected";
    } else if (_on && _mode != contour_selection_mode::replace) {
      ed.select(*_on, _mode); // Shift / Ctrl / Alt click on a point: (de)select it
    } else if (_mode == contour_selection_mode::remove) {
      // Ctrl / Alt click on nothing: nothing
    } else if (!p.inside) {
      out.note = "a click past the volume's edge adds no point";
    } else if (current < 0 || static_cast<std::size_t>(current) >= ed.contours().objects.size()) {
      out.note = "no object to draw: add one";
    } else if (!ed.contours().objects[static_cast<std::size_t>(current)].visible) {
      out.note = "the object '" + ed.contours().objects[static_cast<std::size_t>(current)].name +
                 "' is hidden: show it to draw";
    } else {
      const std::size_t o = static_cast<std::size_t>(current);
      const std::array<double, 2> at = snapped(_r0, _u0);
      const bool fresh = _mode == contour_selection_mode::add; // Shift+click: a new curve
      const grid_point_ref ref = ed.add_point(o, _axis, _index, at[0], at[1], fresh);
      out.changed = true;
      const std::size_t n = ed.curve(ref.curve_ref()).size();
      out.note = (n == 1 ? "started a curve of " : "added point " + std::to_string(n) + " to ") +
                 ed.contours().objects[o].name + " on " + plane_text(p);
    }
    out.selection_changed = ed.selection_version() != sel;
    return out;
  }
  }
  return out;
}

// ---- object rows --------------------------------------------------------------------------------

std::vector<grid_contour_object> read_contour_object_rows(state &rows,
                                                          std::vector<std::string> *problems) {
  const auto problem = [problems](const std::string &m) {
    if (problems)
      problems->push_back(m);
  };
  std::vector<grid_contour_object> out;
  long long count = 0;
  const std::string n = rows.value();
  if (!n.empty() && (!param_detail::parse_integer(n, count) || count < 0)) {
    problem("objects: '" + n + "' is not a row count");
    count = 0;
  }
  count = std::min<long long>(count, 4096);
  const auto field = [&rows](long long i, const char *f) {
    state *s = rows.findDescendant(std::to_string(i) + "." + f);
    return s ? s->value() : std::string();
  };
  for (long long i = 0; i < count; ++i) {
    grid_contour_object o;
    o.name = field(i, "name");
    o.color = point_class_palette(static_cast<std::size_t>(i));
    const std::string row = "objects." + std::to_string(i);
    std::string why;
    if (const std::string t = field(i, "color"); !t.empty()) {
      std::vector<double> col;
      if (param_detail::value_from_text(t, col, why) && col.size() == 3 &&
          std::all_of(col.begin(), col.end(), [](double v) { return v >= 0 && v <= 1; }))
        o.color = col;
      else
        problem(row + ".color: '" + t + "' is not r,g,b in [0, 1]");
    }
    if (const std::string t = field(i, "interpolation"); !t.empty()) {
      if (!param_detail::enum_from_name(t, o.interpolation))
        problem(row + ".interpolation: '" + t + "' is not linear or spline");
    }
    if (const std::string t = field(i, "samples"); !t.empty()) {
      long long v = 0;
      if (param_detail::parse_integer(t, v) && v >= 1 && v <= grid_contour_max_samples)
        o.samples = static_cast<int>(v);
      else
        problem(row + ".samples: '" + t + "' is not a whole number in [1, " +
                std::to_string(grid_contour_max_samples) + "]");
    }
    if (const std::string t = field(i, "visible"); !t.empty()) {
      bool v = true;
      if (param_detail::parse_bool(t, v))
        o.visible = v;
      else
        problem(row + ".visible: '" + t + "' is not 1 or 0");
    }
    out.push_back(std::move(o));
  }
  return out;
}

void write_contour_object_rows(state &rows, const grid_contours &c) {
  const auto put = [](state &s, const std::string &v) {
    if (s.value() != v)
      s.value(v);
  };
  const std::size_t n = c.objects.size();
  for (std::size_t i = 0; i < n; ++i) {
    const grid_contour_object &o = c.objects[i];
    state &row = rows(std::to_string(i));
    put(row("name"), o.name);
    std::string col;
    for (std::size_t k = 0; k < o.color.size(); ++k)
      col += (k ? "," : "") + num(o.color[k]);
    put(row("color"), col);
    std::string interp;
    param_detail::enum_name(o.interpolation, interp);
    put(row("interpolation"), interp);
    put(row("samples"), std::to_string(o.samples));
    put(row("visible"), o.visible ? "1" : "0");
    std::size_t points = 0;
    for (const grid_curve &cv : o.curves)
      points += cv.size();
    put(row("curves"), std::to_string(o.curves.size()));
    put(row("points"), std::to_string(points));
  }
  for (const std::string &child : rows.childNames()) {
    long long i = 0;
    if (param_detail::parse_integer(child, i) && i >= static_cast<long long>(n))
      rows.removeChild(child);
  }
  put(rows, std::to_string(n)); // last: a watcher on the count sees the rows written
}

std::vector<grid_contour_object>
reconcile_contour_object_rows(const grid_contours &old,
                              const std::vector<grid_contour_object> &rows) {
  std::vector<grid_contour_object> out;
  out.reserve(rows.size());
  if (rows.size() == old.objects.size()) {
    for (std::size_t i = 0; i < rows.size(); ++i) {
      grid_contour_object o = rows[i];
      o.curves = old.objects[i].curves;
      out.push_back(std::move(o));
    }
  } else {
    std::vector<bool> taken(old.objects.size(), false);
    for (const grid_contour_object &r : rows) {
      grid_contour_object o = r;
      o.curves.clear();
      for (std::size_t k = 0; k < old.objects.size(); ++k)
        if (!taken[k] && !r.name.empty() && old.objects[k].name == r.name) {
          taken[k] = true;
          o.curves = old.objects[k].curves;
          break;
        }
      out.push_back(std::move(o));
    }
  }
  // Every object named, and no name twice.
  std::set<std::string> used;
  for (std::size_t i = 0; i < out.size(); ++i) {
    if (!out[i].name.empty() && used.insert(out[i].name).second)
      continue;
    for (std::size_t k = i + 1;; ++k) {
      const std::string name = "object " + std::to_string(k);
      if (std::none_of(out.begin(), out.end(),
                       [&name](const grid_contour_object &o) { return o.name == name; }) &&
          used.insert(name).second) {
        out[i].name = name;
        break;
      }
    }
  }
  return out;
}

// ---- tiling -------------------------------------------------------------------------------------

namespace {

// A geometry from the tiler's stack frame back to object space: x and y the plane's (r, u) and z
// along the axis, each scaled by `scale` from the frame's origin. The axes trade places, which for
// the XZ and ZY planes is a reflection, so their triangles' winding is turned back.
void to_object_space(geometry &g, slice_axis a, double scale, const grid_frame &f) {
  const std::pair<int, int> ru = grid_plane_axes(a);
  const int w = ax(a);
  const auto place = [&](double x, double y, double z, bool point) {
    geometry::point_t p{{0.0, 0.0, 0.0}};
    p[ru.first] = point ? f.origin[ru.first] + x * scale : x;
    p[ru.second] = point ? f.origin[ru.second] + y * scale : y;
    p[w] = point ? f.origin[w] + z * scale : z;
    return p;
  };
  for (geometry::point_t &p : g.points())
    p = place(p[0], p[1], p[2], true);
  for (geometry::normal_t &n : g.normals())
    n = place(n[0], n[1], n[2], false);
  if (a != slice_axis::z)
    for (geometry::tri_t &t : g.tris())
      std::swap(t[1], t[2]);
}

void append(geometry &into, const geometry &g) {
  const auto base = static_cast<geometry::index_t>(into.num_points());
  const bool normals = into.num_points() == 0 || into.normals().size() == into.points().size();
  const bool colors = into.num_points() == 0 || into.colors().size() == into.points().size();
  into.points().insert(into.points().end(), g.points().begin(), g.points().end());
  if (normals && g.normals().size() == g.points().size())
    into.normals().insert(into.normals().end(), g.normals().begin(), g.normals().end());
  else
    into.normals().clear();
  if (colors && g.colors().size() == g.points().size())
    into.colors().insert(into.colors().end(), g.colors().begin(), g.colors().end());
  else
    into.colors().clear();
  for (geometry::tri_t t : g.tris()) {
    for (auto &i : t)
      i += base;
    into.tris().push_back(t);
  }
}

} // namespace

struct grid_tile_stepper::impl {
  struct part {
    slice_axis axis = slice_axis::z;
    std::unique_ptr<tile_stepper> s;
    double weight = 0; // its share of the progress
  };
  grid_frame frame;
  double scale = 1;
  std::vector<part> parts;
  std::size_t cur = 0;
  double done_weight = 0; // the finished parts' shares
  grid_tile_result result;
  std::atomic<bool> cancelled{false};
  bool finished = false;
  std::exception_ptr error;
};

grid_tile_stepper::grid_tile_stepper(const grid_contours &c, tile_params params)
    : _impl(std::make_unique<impl>()) {
  const std::vector<std::string> problems = grid_contours_problems(c);
  if (!problems.empty())
    throw tiler_error("tile_grid_contours: " + problems.front());
  const std::vector<std::string> pp = tile_params_problems(params);
  if (!pp.empty())
    throw tiler_error("tile_grid_contours: " + pp.front());
  impl &m = *_impl;
  m.frame = c.frame;
  m.scale = std::min({c.frame.spacing[0], c.frame.spacing[1], c.frame.spacing[2]});
  const double s = m.scale;
  // The tiler's lengths and areas, in its units.
  tile_params tp = params;
  tp.curation_delta /= s;
  tp.collinear_epsilon /= s * s;
  if (tp.correspondence_overlap >= 0)
    tp.correspondence_overlap /= s * s;

  for (const grid_contour_object &o : c.objects) {
    std::size_t open = 0;
    for (const grid_curve &cv : o.curves)
      open += cv.closed ? 0 : 1;
    if (open)
      m.result.warnings.push_back("tile_grid_contours: " + std::to_string(open) + " open curve" +
                                  (open == 1 ? "" : "s") + " of '" + o.name +
                                  "' left out (close a curve to tile it)");
  }
  double total = 0;
  for (slice_axis a : {slice_axis::z, slice_axis::y, slice_axis::x}) {
    contour_stack st = grid_contours_stack(c, a);
    if (st.polygon_count() == 0)
      continue;
    const std::pair<int, int> ru = grid_plane_axes(a);
    for (auto &[index, slice] : st.slices)
      for (auto &[name, polys] : slice.components)
        for (contour_polygon &poly : polys)
          for (std::size_t i = 0; i + 1 < poly.xy.size(); i += 2) {
            poly.xy[i] = (poly.xy[i] - c.frame.origin[ru.first]) / s;
            poly.xy[i + 1] = (poly.xy[i + 1] - c.frame.origin[ru.second]) / s;
          }
    st.z_origin = 0;
    st.z_spacing = c.frame.spacing[ax(a)] / s;
    impl::part p;
    p.axis = a;
    p.weight = static_cast<double>(std::max<std::size_t>(1, st.vertex_count()));
    total += p.weight;
    p.s = std::make_unique<tile_stepper>(std::move(st), tp);
    m.parts.push_back(std::move(p));
  }
  if (m.parts.empty())
    throw tiler_error("tile_grid_contours: no closed curve to tile");
  for (impl::part &p : m.parts)
    p.weight /= total;
}

grid_tile_stepper::~grid_tile_stepper() = default;
grid_tile_stepper::grid_tile_stepper(grid_tile_stepper &&) noexcept = default;
grid_tile_stepper &grid_tile_stepper::operator=(grid_tile_stepper &&) noexcept = default;

bool grid_tile_stepper::step(std::size_t budget) {
  impl &m = *_impl;
  if (m.error)
    std::rethrow_exception(m.error);
  if (m.finished)
    return true;
  try {
    while (m.cur < m.parts.size()) {
      if (m.cancelled.load()) {
        m.parts.clear();
        m.result = grid_tile_result{}; // no result
        m.finished = true;
        return true;
      }
      impl::part &p = m.parts[m.cur];
      if (!p.s->step(budget) && budget > 0)
        return false;
      if (!p.s->done())
        continue; // budget 0 ran it to the end, or it was cancelled inside the step
      tile_result r = p.s->take();
      const std::string prefix = std::string(plane_name(p.axis)) + ": ";
      for (std::string &w : r.warnings)
        m.result.warnings.push_back(prefix + w);
      for (auto &[name, g] : r.components) {
        to_object_space(g, p.axis, m.scale, m.frame);
        auto it = m.result.surfaces.find(name);
        if (it == m.result.surfaces.end())
          m.result.surfaces.emplace(name, std::move(g));
        else
          append(it->second, g);
      }
      m.done_weight += p.weight;
      p.s.reset();
      ++m.cur;
      if (budget > 0 && m.cur < m.parts.size())
        return false; // a step ends where the orientation changes, as where a phase does
    }
  } catch (...) {
    m.error = std::current_exception();
    throw;
  }
  m.finished = true;
  return true;
}

bool grid_tile_stepper::done() const { return _impl->finished; }

const char *grid_tile_stepper::phase() const {
  const impl &m = *_impl;
  if (m.error)
    return "failed";
  if (m.finished)
    return "done";
  if (m.cur < m.parts.size() && m.parts[m.cur].s)
    return m.parts[m.cur].s->phase();
  return "tiling";
}

std::string grid_tile_stepper::label() const {
  const impl &m = *_impl;
  if (m.finished || m.error || m.cur >= m.parts.size() || !m.parts[m.cur].s)
    return std::string();
  const std::string inner = m.parts[m.cur].s->label();
  return std::string(plane_name(m.parts[m.cur].axis)) + (inner.empty() ? "" : " " + inner);
}

double grid_tile_stepper::progress() const {
  const impl &m = *_impl;
  if (m.finished)
    return 1.0;
  double p = m.done_weight;
  if (m.cur < m.parts.size() && m.parts[m.cur].s)
    p += m.parts[m.cur].weight * m.parts[m.cur].s->progress();
  return std::clamp(p, 0.0, 1.0);
}

std::string grid_tile_stepper::status() const {
  const impl &m = *_impl;
  if (m.error)
    return "failed";
  if (m.finished) {
    std::size_t tris = 0;
    for (const auto &kv : m.result.surfaces)
      tris += kv.second.num_tris();
    return "tiled " + std::to_string(m.result.surfaces.size()) + " object" +
           (m.result.surfaces.size() == 1 ? "" : "s") + ": " + std::to_string(tris) + " triangles";
  }
  const std::string l = label();
  return std::string(phase()) + (l.empty() ? "" : " " + l);
}

// The flag alone: the parts belong to the stepping thread, which resets each as it finishes. A
// step with a budget runs one part's step, so the cancel is seen within one budget.
void grid_tile_stepper::cancel() { _impl->cancelled.store(true); }

grid_tile_result grid_tile_stepper::take() {
  impl &m = *_impl;
  if (m.error)
    std::rethrow_exception(m.error);
  if (!m.finished)
    throw tiler_error("tile_grid_contours: take() before the stepper is done");
  return std::move(m.result);
}

grid_tile_result tile_grid_contours(const grid_contours &c, const tile_params &params) {
  grid_tile_stepper s(c, params);
  s.step(0);
  return s.take();
}

std::unique_ptr<result_job<grid_tile_result>> tile_grid_contours_job(const grid_contours &c,
                                                                     tile_params params) {
  return make_result_job(std::make_unique<grid_tile_stepper>(c, std::move(params)));
}

// ---- files --------------------------------------------------------------------------------------

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

template <class V> std::string json_numbers(const V &v) {
  std::string out = "[";
  for (std::size_t i = 0; i < v.size(); ++i)
    out += (i ? ", " : "") + num(static_cast<double>(v[i]));
  return out + "]";
}

double number_of(const std::string &text, const std::string &what) {
  double d = 0;
  if (!param_detail::parse_double(text, d) || !std::isfinite(d))
    throw std::runtime_error(what + ": '" + text + "' is not a number");
  return d;
}

long long integer_of(const std::string &text, const std::string &what) {
  long long v = 0;
  if (!param_detail::parse_integer(text, v))
    throw std::runtime_error(what + ": '" + text + "' is not a whole number");
  return v;
}

std::vector<double> numbers_of(const pt::ptree &t, const std::string &what) {
  std::vector<double> out;
  for (const auto &v : t) {
    if (!v.second.empty())
      throw std::runtime_error(what + ": not a list of numbers");
    out.push_back(number_of(v.second.data(), what));
  }
  return out;
}

bool bool_of(const std::string &text, const std::string &what) {
  bool b = false;
  if (!param_detail::parse_bool(text, b))
    throw std::runtime_error(what + ": '" + text + "' is not true or false");
  return b;
}

} // namespace

std::string grid_contours_json(const grid_contours &c) {
  const std::vector<std::string> problems = grid_contours_problems(c);
  if (!problems.empty())
    throw std::invalid_argument("grid_contours_json: " + problems.front());
  std::string out = "{\n  \"format\": \"cvc.volume_grid.contours\",\n  \"version\": 1,\n";
  out += "  \"frame\": {\"dims\": " + json_numbers(c.frame.dims) +
         ", \"origin\": " + json_numbers(c.frame.origin) +
         ", \"spacing\": " + json_numbers(c.frame.spacing) + "},\n";
  out += "  \"objects\": [";
  for (std::size_t o = 0; o < c.objects.size(); ++o) {
    const grid_contour_object &obj = c.objects[o];
    out += o ? ",\n    {" : "\n    {";
    out += "\"name\": ";
    json_string(obj.name, out);
    std::string interp;
    param_detail::enum_name(obj.interpolation, interp);
    out += ", \"color\": " + json_numbers(obj.color) + ", \"interpolation\": \"" + interp +
           "\", \"samples\": " + std::to_string(obj.samples) +
           ", \"visible\": " + (obj.visible ? "true" : "false") + ", \"curves\": [";
    for (std::size_t k = 0; k < obj.curves.size(); ++k) {
      const grid_curve &cv = obj.curves[k];
      out += k ? ",\n      {" : "\n      {";
      out += std::string("\"axis\": \"") + slice_axis_name(cv.axis) +
             "\", \"index\": " + std::to_string(cv.index) +
             ", \"closed\": " + (cv.closed ? "true" : "false") +
             ", \"points\": " + json_numbers(cv.ru) + "}";
    }
    out += obj.curves.empty() ? "]}" : "\n    ]}";
  }
  out += c.objects.empty() ? "]\n}\n" : "\n  ]\n}\n";
  return out;
}

grid_contours grid_contours_from_json(const std::string &json, const std::string &what) {
  pt::ptree root;
  try {
    std::istringstream in(json);
    pt::read_json(in, root);
  } catch (const std::exception &e) {
    throw std::runtime_error(what + ": not JSON (" + std::string(e.what()) + ")");
  }
  if (root.get<std::string>("format", "") != "cvc.volume_grid.contours")
    throw std::runtime_error(what + ": not volume grid contours (no \"format\": "
                                    "\"cvc.volume_grid.contours\")");
  const long long version = integer_of(root.get<std::string>("version", "1"), what + ": version");
  if (version != 1)
    throw std::runtime_error(what + ": version " + std::to_string(version) +
                             " (this libcvc reads version 1)");
  grid_contours c;
  if (const auto frame = root.get_child_optional("frame")) {
    const auto three = [&](const char *key, auto &dest) {
      const auto t = frame->get_child_optional(key);
      if (!t)
        return;
      const std::vector<double> v = numbers_of(*t, what + ": frame." + key);
      if (v.size() != 3)
        throw std::runtime_error(what + ": frame." + key + " has " + std::to_string(v.size()) +
                                 " values, not three");
      for (int a = 0; a < 3; ++a)
        dest[a] = static_cast<std::remove_reference_t<decltype(dest[0])>>(v[a]);
    };
    three("dims", c.frame.dims);
    three("origin", c.frame.origin);
    three("spacing", c.frame.spacing);
  }
  if (const auto objects = root.get_child_optional("objects")) {
    std::size_t o = 0;
    for (const auto &item : *objects) {
      const std::string w = what + ": objects[" + std::to_string(o++) + "]";
      const pt::ptree &t = item.second;
      grid_contour_object obj;
      obj.name = t.get<std::string>("name", "");
      if (const auto col = t.get_child_optional("color"))
        obj.color = numbers_of(*col, w + ".color");
      if (const auto s = t.get_optional<std::string>("interpolation"))
        if (!param_detail::enum_from_name(*s, obj.interpolation))
          throw std::runtime_error(w + ".interpolation: '" + *s + "' is not linear or spline");
      if (const auto s = t.get_optional<std::string>("samples")) {
        const long long v = integer_of(*s, w + ".samples");
        obj.samples = static_cast<int>(std::clamp<long long>(v, -1, 1 << 20));
      }
      if (const auto s = t.get_optional<std::string>("visible"))
        obj.visible = bool_of(*s, w + ".visible");
      if (const auto curves = t.get_child_optional("curves")) {
        std::size_t k = 0;
        for (const auto &ci : *curves) {
          const std::string cw = w + ".curves[" + std::to_string(k++) + "]";
          const pt::ptree &ct = ci.second;
          grid_curve cv;
          const std::string axis = ct.get<std::string>("axis", "z");
          if (!parse_slice_axis(axis, cv.axis))
            throw std::runtime_error(cw + ".axis: '" + axis + "' is not x, y or z");
          cv.index = integer_of(ct.get<std::string>("index", "0"), cw + ".index");
          if (const auto s = ct.get_optional<std::string>("closed"))
            cv.closed = bool_of(*s, cw + ".closed");
          if (const auto pts = ct.get_child_optional("points"))
            cv.ru = numbers_of(*pts, cw + ".points");
          obj.curves.push_back(std::move(cv));
        }
      }
      c.objects.push_back(std::move(obj));
    }
  }
  const std::vector<std::string> problems = grid_contours_problems(c);
  if (!problems.empty())
    throw std::runtime_error(what + ": " + problems.front());
  return c;
}

} // namespace cvc
