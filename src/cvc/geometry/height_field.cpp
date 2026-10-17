/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cvc/core/exception.h>
#include <cvc/geometry/height_field.h>
#include <limits>
#include <string>

namespace cvc {

namespace {

constexpr std::uint32_t kNone = 0xffffffffu;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

template <class T> void detach(boost::shared_ptr<T> &p) {
  if (!p)
    p.reset(new T);
  else if (!p.unique())
    p.reset(new T(*p));
}

} // namespace

height_field::height_field()
    : _heights(new std::vector<float>), _cell_materials(new std::vector<face_material_t>),
      _vertex(new std::vector<std::uint32_t>), _first_face(new std::vector<std::uint32_t>) {}

height_field::height_field(int rows, int cols, double x0, double y0, double dx, double dy,
                           std::vector<float> heights, std::vector<face_material_t> cell_materials)
    : height_field() {
  if (rows < 2 || cols < 2)
    throw cvc::geometry_error("height_field: a grid needs at least 2 x 2 samples");
  if (!(dx > 0.0) || !(dy > 0.0) || !std::isfinite(dx) || !std::isfinite(dy) ||
      !std::isfinite(x0) || !std::isfinite(y0))
    throw cvc::geometry_error(
        "height_field: the origin must be finite and the spacing finite and > 0");
  const std::size_t n = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
  if (n > 0xfffffffeu)
    throw cvc::geometry_error("height_field: more samples than a mesh can index");
  if (!heights.empty() && heights.size() != n)
    throw cvc::geometry_error("height_field: " + std::to_string(heights.size()) + " heights for " +
                              std::to_string(rows) + " x " + std::to_string(cols) + " samples");
  const std::size_t cells = static_cast<std::size_t>(rows - 1) * static_cast<std::size_t>(cols - 1);
  if (!cell_materials.empty() && cell_materials.size() != cells)
    throw cvc::geometry_error("height_field: " + std::to_string(cell_materials.size()) +
                              " cell materials for " + std::to_string(cells) + " cells");
  for (const float h : heights)
    if (std::isinf(h))
      throw cvc::geometry_error("height_field: an infinite height (NaN marks a hole)");
  _rows = rows;
  _cols = cols;
  _x0 = x0;
  _y0 = y0;
  _dx = dx;
  _dy = dy;
  if (heights.empty())
    heights.assign(n, 0.0f);
  _heights.reset(new std::vector<float>(std::move(heights)));
  _cell_materials.reset(new std::vector<face_material_t>(std::move(cell_materials)));
  rebuild();
}

height_field height_field::from_bounds(int rows, int cols, double min_x, double min_y, double max_x,
                                       double max_y, std::vector<float> heights) {
  if (rows < 2 || cols < 2)
    throw cvc::geometry_error("height_field: a grid needs at least 2 x 2 samples");
  return height_field(rows, cols, min_x, min_y, (max_x - min_x) / (cols - 1),
                      (max_y - min_y) / (rows - 1), std::move(heights));
}

height_field::height_field(const height_field &other)
    : geometry(other), _rows(other._rows), _cols(other._cols), _x0(other._x0), _y0(other._y0),
      _dx(other._dx), _dy(other._dy), _heights(other._heights),
      _cell_materials(other._cell_materials), _vertex(other._vertex),
      _first_face(other._first_face) {}

height_field &height_field::operator=(const height_field &other) {
  if (this == &other)
    return *this;
  geometry::copy(other);
  _rows = other._rows;
  _cols = other._cols;
  _x0 = other._x0;
  _y0 = other._y0;
  _dx = other._dx;
  _dy = other._dy;
  _heights = other._heights;
  _cell_materials = other._cell_materials;
  _vertex = other._vertex;
  _first_face = other._first_face;
  return *this;
}

height_field::~height_field() = default;

void height_field::pre_write(ARRAY_TYPE at) {
  if (!_writing) {
    switch (at) {
    case POINTS:
    case BOUNDARY:
    case LINES:
    case TRIS:
    case QUADS:
    case TETS:
    case HEXS:
    case FACE_MATERIALS:
      throw cvc::read_only_error(
          "height_field: the mesh follows the grid; edit it with set_height/set_heights/"
          "set_cell_material, or copy it into a plain cvc::geometry");
    default:
      break; // per-vertex attributes (colours, functions, ...) stay writable
    }
  }
  geometry::pre_write(at);
}

void height_field::check(int r, int c, const char *what) const {
  if (r < 0 || c < 0 || r >= _rows || c >= _cols)
    throw cvc::index_out_of_bounds(std::string("height_field::") + what + ": (" +
                                   std::to_string(r) + ", " + std::to_string(c) +
                                   ") is outside the grid");
}

float height_field::height_at(int r, int c) const {
  check(r, c, "height_at");
  return (*_heights)[static_cast<std::size_t>(r) * _cols + c];
}

face_material_t height_field::cell_material(int r, int c) const {
  if (r < 0 || c < 0 || r >= cell_rows() || c >= cell_cols())
    throw cvc::index_out_of_bounds("height_field::cell_material: the cell is outside the grid");
  return _cell_materials->empty()
             ? 0
             : (*_cell_materials)[static_cast<std::size_t>(r) * cell_cols() + c];
}

std::int64_t height_field::vertex_of(int r, int c) const {
  check(r, c, "vertex_of");
  const std::uint32_t v = (*_vertex)[static_cast<std::size_t>(r) * _cols + c];
  return v == kNone ? -1 : static_cast<std::int64_t>(v);
}

std::int64_t height_field::first_face_of(int r, int c) const {
  if (r < 0 || c < 0 || r >= cell_rows() || c >= cell_cols())
    throw cvc::index_out_of_bounds("height_field::first_face_of: the cell is outside the grid");
  const std::uint32_t f = (*_first_face)[static_cast<std::size_t>(r) * cell_cols() + c];
  return f == kNone ? -1 : static_cast<std::int64_t>(f);
}

// The upward normal at a sample from the grid's slope: central differences,
// one-sided at an edge or next to a hole.
geometry::vector_t height_field::grid_normal(int r, int c) const {
  const std::vector<float> &h = *_heights;
  auto at = [&](int rr, int cc) { return h[static_cast<std::size_t>(rr) * _cols + cc]; };
  const double h0 = at(r, c);
  auto slope = [&](int r1, int c1, int r2, int c2, double step) {
    const bool lo = r1 >= 0 && c1 >= 0 && r1 < _rows && c1 < _cols && !std::isnan(at(r1, c1));
    const bool hi = r2 >= 0 && c2 >= 0 && r2 < _rows && c2 < _cols && !std::isnan(at(r2, c2));
    if (lo && hi)
      return (at(r2, c2) - at(r1, c1)) / (2.0 * step);
    if (hi)
      return (at(r2, c2) - h0) / step;
    if (lo)
      return (h0 - at(r1, c1)) / step;
    return 0.0;
  };
  const double gx = slope(r, c - 1, r, c + 1, _dx);
  const double gy = slope(r - 1, c, r + 1, c, _dy);
  const double len = std::sqrt(gx * gx + gy * gy + 1.0);
  vector_t n;
  n[0] = -gx / len;
  n[1] = -gy / len;
  n[2] = 1.0 / len;
  return n;
}

void height_field::rebuild() {
  _writing = true;
  try {
    const std::size_t n = static_cast<std::size_t>(_rows) * static_cast<std::size_t>(_cols);
    const std::vector<float> &h = *_heights;
    const std::size_t old_vertices = num_points();

    std::vector<std::uint32_t> vertex(n, kNone);
    points_t pts;
    normals_t nrm;
    uvs_t uv;
    pts.reserve(n);
    nrm.reserve(n);
    uv.reserve(n);
    for (int r = 0; r < _rows; ++r)
      for (int c = 0; c < _cols; ++c) {
        const float z = h[static_cast<std::size_t>(r) * _cols + c];
        if (std::isnan(z))
          continue;
        vertex[static_cast<std::size_t>(r) * _cols + c] = static_cast<std::uint32_t>(pts.size());
        point_t p;
        p[0] = _x0 + c * _dx;
        p[1] = _y0 + r * _dy;
        p[2] = z;
        pts.push_back(p);
        nrm.push_back(grid_normal(r, c));
        uv_t t;
        t[0] = _cols > 1 ? static_cast<double>(c) / (_cols - 1) : 0.0;
        t[1] = _rows > 1 ? static_cast<double>(r) / (_rows - 1) : 0.0;
        uv.push_back(t);
      }

    const int cr = cell_rows(), cc = cell_cols();
    std::vector<std::uint32_t> first(static_cast<std::size_t>(cr) * cc, kNone);
    tris_t tri;
    face_materials_t fm;
    tri.reserve(static_cast<std::size_t>(cr) * cc * 2);
    const bool any_material = !_cell_materials->empty();
    if (any_material)
      fm.reserve(tri.capacity());
    for (int r = 0; r < cr; ++r)
      for (int c = 0; c < cc; ++c) {
        const std::uint32_t a = vertex[static_cast<std::size_t>(r) * _cols + c];         // SW
        const std::uint32_t b = vertex[static_cast<std::size_t>(r) * _cols + c + 1];     // SE
        const std::uint32_t d = vertex[static_cast<std::size_t>(r + 1) * _cols + c + 1]; // NE
        const std::uint32_t e = vertex[static_cast<std::size_t>(r + 1) * _cols + c];     // NW
        if (a == kNone || b == kNone || d == kNone || e == kNone)
          continue;
        first[static_cast<std::size_t>(r) * cc + c] = static_cast<std::uint32_t>(tri.size());
        tri_t t1, t2;
        t1[0] = a;
        t1[1] = b;
        t1[2] = d;
        t2[0] = a;
        t2[1] = d;
        t2[2] = e;
        tri.push_back(t1);
        tri.push_back(t2);
        if (any_material) {
          const face_material_t m = (*_cell_materials)[static_cast<std::size_t>(r) * cc + c];
          fm.push_back(m);
          fm.push_back(m);
        }
      }

    // Per-vertex attributes the caller set (colours, functions, ...) survive a
    // rebuild that keeps the vertex count; otherwise they no longer line up.
    if (pts.size() != old_vertices) {
      if (!const_colors().empty())
        colors().clear();
      if (!const_functions().empty())
        functions().clear();
      if (!const_curvatures().empty())
        curvatures().clear();
      if (!const_tangents().empty())
        tangents().clear();
      if (!const_boundary().empty())
        boundary().clear();
    }
    points().swap(pts);
    normals().swap(nrm);
    uvs().swap(uv);
    tris().swap(tri);
    if (!const_quads().empty())
      quads().clear();
    face_materials().swap(fm);
    set_geometry_type(SURFACE_TRI);
    _vertex.reset(new std::vector<std::uint32_t>(std::move(vertex)));
    _first_face.reset(new std::vector<std::uint32_t>(std::move(first)));
  } catch (...) {
    _writing = false;
    throw;
  }
  _writing = false;
}

void height_field::set_heights(std::vector<float> heights) {
  const std::size_t n = static_cast<std::size_t>(_rows) * static_cast<std::size_t>(_cols);
  if (heights.size() != n)
    throw cvc::geometry_error("height_field::set_heights: " + std::to_string(heights.size()) +
                              " heights for " + std::to_string(n) + " samples");
  for (const float h : heights)
    if (std::isinf(h))
      throw cvc::geometry_error("height_field: an infinite height (NaN marks a hole)");
  _heights.reset(new std::vector<float>(std::move(heights)));
  rebuild();
}

void height_field::set_cell_materials(std::vector<face_material_t> ids) {
  const std::size_t cells = static_cast<std::size_t>(cell_rows()) * cell_cols();
  if (!ids.empty() && ids.size() != cells)
    throw cvc::geometry_error("height_field::set_cell_materials: " + std::to_string(ids.size()) +
                              " ids for " + std::to_string(cells) + " cells");
  _cell_materials.reset(new std::vector<face_material_t>(std::move(ids)));
  rebuild();
}

void height_field::rebuild_vertex_normal(int r, int c) {
  if (r < 0 || c < 0 || r >= _rows || c >= _cols)
    return;
  const std::uint32_t v = (*_vertex)[static_cast<std::size_t>(r) * _cols + c];
  if (v != kNone)
    normals()[v] = grid_normal(r, c);
}

void height_field::set_height(int r, int c, float h) {
  check(r, c, "set_height");
  if (std::isinf(h))
    throw cvc::geometry_error("height_field: an infinite height (NaN marks a hole)");
  const std::size_t i = static_cast<std::size_t>(r) * _cols + c;
  const bool was_hole = std::isnan((*_heights)[i]);
  detach(_heights);
  (*_heights)[i] = h;
  if (was_hole != std::isnan(h)) {
    rebuild(); // the topology changed
    return;
  }
  if (std::isnan(h))
    return; // a hole stays a hole
  _writing = true;
  try {
    points()[(*_vertex)[i]][2] = h;
    // Central differences: this sample's slope and its four neighbours' change.
    rebuild_vertex_normal(r, c);
    rebuild_vertex_normal(r, c - 1);
    rebuild_vertex_normal(r, c + 1);
    rebuild_vertex_normal(r - 1, c);
    rebuild_vertex_normal(r + 1, c);
  } catch (...) {
    _writing = false;
    throw;
  }
  _writing = false;
}

void height_field::set_cell_material(int r, int c, face_material_t id) {
  if (r < 0 || c < 0 || r >= cell_rows() || c >= cell_cols())
    throw cvc::index_out_of_bounds("height_field::set_cell_material: the cell is outside the grid");
  const std::size_t cells = static_cast<std::size_t>(cell_rows()) * cell_cols();
  const std::size_t k = static_cast<std::size_t>(r) * cell_cols() + c;
  if (_cell_materials->empty()) {
    if (id == 0)
      return; // already material 0
    _cell_materials.reset(new std::vector<face_material_t>(cells, 0));
    (*_cell_materials)[k] = id;
    rebuild(); // the mesh had no face materials yet
    return;
  }
  detach(_cell_materials);
  (*_cell_materials)[k] = id;
  const std::uint32_t f = (*_first_face)[k];
  if (f == kNone)
    return; // a cell with a hole has no faces
  _writing = true;
  try {
    face_materials_t &fm = face_materials();
    fm[f] = id;
    fm[f + 1] = id;
  } catch (...) {
    _writing = false;
    throw;
  }
  _writing = false;
}

height_field &height_field::clear() {
  *this = height_field();
  return *this;
}

bool height_field::cell_of(double x, double y, int &r, int &c) const {
  if (_rows < 2 || _cols < 2)
    return false;
  const double u = (x - _x0) / _dx, v = (y - _y0) / _dy;
  if (!(u >= 0.0) || !(v >= 0.0) || u > cell_cols() || v > cell_rows())
    return false; // also NaN
  c = std::min(static_cast<int>(u), cell_cols() - 1);
  r = std::min(static_cast<int>(v), cell_rows() - 1);
  return true;
}

double height_field::height(double x, double y) const {
  int r, c;
  if (!cell_of(x, y, r, c))
    return kNaN;
  const std::vector<float> &h = *_heights;
  const double ha = h[static_cast<std::size_t>(r) * _cols + c];
  const double hb = h[static_cast<std::size_t>(r) * _cols + c + 1];
  const double hd = h[static_cast<std::size_t>(r + 1) * _cols + c + 1];
  const double he = h[static_cast<std::size_t>(r + 1) * _cols + c];
  if (std::isnan(ha) || std::isnan(hb) || std::isnan(hd) || std::isnan(he))
    return kNaN;
  const double u = (x - _x0) / _dx - c, v = (y - _y0) / _dy - r;
  // The diagonal runs from (r, c) to (r+1, c+1): below it (u >= v) the triangle
  // is SW-SE-NE, above it SW-NE-NW.
  if (u >= v)
    return ha + u * (hb - ha) + v * (hd - hb);
  return ha + v * (he - ha) + u * (hd - he);
}

geometry::vector_t height_field::normal(double x, double y) const {
  vector_t n;
  n[0] = n[1] = n[2] = kNaN;
  int r, c;
  if (!cell_of(x, y, r, c))
    return n;
  const std::vector<float> &h = *_heights;
  const double ha = h[static_cast<std::size_t>(r) * _cols + c];
  const double hb = h[static_cast<std::size_t>(r) * _cols + c + 1];
  const double hd = h[static_cast<std::size_t>(r + 1) * _cols + c + 1];
  const double he = h[static_cast<std::size_t>(r + 1) * _cols + c];
  if (std::isnan(ha) || std::isnan(hb) || std::isnan(hd) || std::isnan(he))
    return n;
  const double u = (x - _x0) / _dx - c, v = (y - _y0) / _dy - r;
  // The plane's slopes in metres per metre.
  double gx, gy;
  if (u >= v) {
    gx = (hb - ha) / _dx;
    gy = (hd - hb) / _dy;
  } else {
    gx = (hd - he) / _dx;
    gy = (he - ha) / _dy;
  }
  const double len = std::sqrt(gx * gx + gy * gy + 1.0);
  n[0] = -gx / len;
  n[1] = -gy / len;
  n[2] = 1.0 / len;
  return n;
}

face_material_t height_field::material(double x, double y) const {
  int r, c;
  if (!cell_of(x, y, r, c) || _cell_materials->empty())
    return 0;
  return (*_cell_materials)[static_cast<std::size_t>(r) * cell_cols() + c];
}

} // namespace cvc
