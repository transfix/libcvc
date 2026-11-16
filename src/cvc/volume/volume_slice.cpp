/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Axis-aligned slices of a volume. See inc/cvc/volume/volume_slice.h.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cvc/tf/model.h>
#include <cvc/volume/volume_file_info.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/volume/volume_slice.h>
#include <stdexcept>
#include <string>

namespace cvc {

namespace {

std::string dims_text(const dimension &d) {
  return std::to_string(d.xdim) + "x" + std::to_string(d.ydim) + "x" + std::to_string(d.zdim);
}

double axis_lo(const bounding_box &b, int axis) {
  return axis == 0 ? b.minx : axis == 1 ? b.miny : b.minz;
}
double axis_hi(const bounding_box &b, int axis) {
  return axis == 0 ? b.maxx : axis == 1 ? b.maxy : b.maxz;
}

} // namespace

bool parse_slice_axis(std::string_view text, slice_axis &out) {
  std::string s;
  for (char c : text)
    if (!std::isspace(static_cast<unsigned char>(c)))
      s += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (s == "x" || s == "0" || s == "zy" || s == "yz") {
    out = slice_axis::x;
    return true;
  }
  if (s == "y" || s == "1" || s == "xz" || s == "zx") {
    out = slice_axis::y;
    return true;
  }
  if (s == "z" || s == "2" || s == "xy" || s == "yx") {
    out = slice_axis::z;
    return true;
  }
  return false;
}

const char *slice_axis_name(slice_axis a) {
  return a == slice_axis::x ? "x" : a == slice_axis::y ? "y" : "z";
}

const char *slice_view_name(slice_axis a) {
  return a == slice_axis::x ? "zy" : a == slice_axis::y ? "xz" : "xy";
}

void slice_plane_axes(slice_axis a, int &right, int &up) {
  switch (a) {
  case slice_axis::x:
    right = 2;
    up = 1;
    break;
  case slice_axis::y:
    right = 0;
    up = 2;
    break;
  default:
    right = 0;
    up = 1;
  }
}

std::uint64_t slice_grid::count(int axis) const {
  return axis == 0 ? dim.xdim : axis == 1 ? dim.ydim : dim.zdim;
}

double slice_grid::lo(int axis) const { return axis_lo(box, axis); }
double slice_grid::hi(int axis) const { return axis_hi(box, axis); }

double slice_grid::spacing(int axis) const {
  const std::uint64_t n = count(axis);
  const double extent = hi(axis) - lo(axis);
  if (n > 1)
    return extent / static_cast<double>(n - 1);
  return extent > 0.0 ? extent : 1.0;
}

double slice_grid::plane(slice_axis a, std::uint64_t index) const {
  const int ax = static_cast<int>(a);
  const std::uint64_t n = count(ax);
  if (n <= 1)
    return lo(ax);
  return lo(ax) + static_cast<double>(std::min(index, n - 1)) * spacing(ax);
}

std::uint64_t slice_grid::nearest_index(slice_axis a, double coord) const {
  const int ax = static_cast<int>(a);
  const std::uint64_t n = count(ax);
  if (n <= 1 || !std::isfinite(coord))
    return 0;
  const double f = std::round((coord - lo(ax)) / spacing(ax));
  if (!(f > 0.0))
    return 0;
  return std::min(static_cast<std::uint64_t>(f), n - 1);
}

bool slice_grid::voxel_of(const double p[3], std::int64_t ijk[3]) const {
  bool inside = true;
  for (int a = 0; a < 3; ++a) {
    const double f = std::round((p[a] - lo(a)) / spacing(a));
    if (!std::isfinite(f)) {
      ijk[a] = -1;
      inside = false;
      continue;
    }
    ijk[a] = static_cast<std::int64_t>(std::clamp(f, -9.0e15, 9.0e15));
    inside = inside && ijk[a] >= 0 && static_cast<std::uint64_t>(ijk[a]) < count(a);
  }
  return inside;
}

void slice_grid::voxel_center(const std::int64_t ijk[3], double p[3]) const {
  for (int a = 0; a < 3; ++a)
    p[a] = lo(a) + static_cast<double>(ijk[a]) * spacing(a);
}

slice_grid grid_of(const volume &v) { return slice_grid{v.voxel_dimensions(), v.boundingBox()}; }

void volume_slice_data::voxel(std::uint64_t column, std::uint64_t row, std::uint64_t ijk[3]) const {
  int right = 0, up = 1;
  slice_plane_axes(axis, right, up);
  ijk[static_cast<int>(axis)] = index;
  ijk[right] = column;
  ijk[up] = row;
}

volume_slice_data extract_slice(const volume &vol, slice_axis axis, std::uint64_t index) {
  const dimension &d = vol.voxel_dimensions();
  if (d.xdim == 0 || d.ydim == 0 || d.zdim == 0)
    throw std::invalid_argument("extract_slice: the volume is empty (" + dims_text(d) + ")");
  const slice_grid g{d, vol.boundingBox()};
  const int ax = static_cast<int>(axis);
  if (index >= g.count(ax))
    throw std::out_of_range("extract_slice: slice " + std::to_string(index) + " down " +
                            slice_axis_name(axis) + " is outside the " + dims_text(d) +
                            " volume (0 to " + std::to_string(g.count(ax) - 1) + ")");
  int right = 0, up = 1;
  slice_plane_axes(axis, right, up);
  volume_slice_data s;
  s.axis = axis;
  s.index = index;
  s.width = g.count(right);
  s.height = g.count(up);
  s.values.resize(static_cast<std::size_t>(s.width * s.height));
  std::uint64_t ijk[3] = {0, 0, 0};
  ijk[ax] = index;
  std::size_t at = 0;
  for (std::uint64_t r = 0; r < s.height; ++r) {
    ijk[up] = r;
    for (std::uint64_t c = 0; c < s.width; ++c) {
      ijk[right] = c;
      s.values[at++] = vol(ijk[0], ijk[1], ijk[2]);
    }
  }
  return s;
}

std::vector<std::uint8_t> color_slice(const volume_slice_data &s, const tf::model &m, double lo,
                                      double hi, bool opaque) {
  double elo = 0.0, ehi = 1.0;
  tf::effective_domain(m, true, lo, hi, elo, ehi);
  const int n = std::clamp(m.resolution, 2, 65536);
  const std::vector<std::uint8_t> table = m.bake_rgba8(n);
  std::vector<std::uint8_t> px(static_cast<std::size_t>(s.width * s.height) * 4, 0);
  if (table.size() < static_cast<std::size_t>(n) * 4 || s.values.size() < s.width * s.height)
    return px;
  const double scale = static_cast<double>(n - 1);
  std::size_t out = 0;
  for (std::uint64_t y = 0; y < s.height; ++y) {
    const std::uint64_t r = s.height - 1 - y; // image order: the top row first
    const double *row = s.values.data() + r * s.width;
    for (std::uint64_t c = 0; c < s.width; ++c, out += 4) {
      const double v = row[c];
      if (std::isnan(v))
        continue; // transparent black
      const double t = std::clamp(tf::to_normalized(v, elo, ehi), 0.0, 1.0);
      const std::size_t e = static_cast<std::size_t>(std::lround(t * scale)) * 4;
      px[out + 0] = table[e + 0];
      px[out + 1] = table[e + 1];
      px[out + 2] = table[e + 2];
      px[out + 3] = opaque ? 255 : table[e + 3];
    }
  }
  return px;
}

slice_source::slice_source(const volume &vol, std::size_t cache)
    : _volume(vol), _grid(grid_of(vol)), _min(vol.min()), _max(vol.max()),
      _capacity(std::max<std::size_t>(cache, 1)) {}

slice_source::slice_source(app &ctx, const std::string &file, unsigned int var, unsigned int time,
                           std::size_t cache)
    : _ctx(&ctx), _file(file), _var(var), _time(time), _capacity(std::max<std::size_t>(cache, 1)) {
  try {
    volume_file_info info(ctx, file);
    if (var >= info.numVariables() || time >= info.numTimesteps())
      throw std::runtime_error("it has " + std::to_string(info.numVariables()) +
                               " variable(s) and " + std::to_string(info.numTimesteps()) +
                               " timestep(s), not variable " + std::to_string(var) +
                               " at timestep " + std::to_string(time));
    _grid = slice_grid{info.voxel_dimensions(), info.boundingBox()};
    if (_grid.dim.xdim == 0 || _grid.dim.ydim == 0 || _grid.dim.zdim == 0)
      throw std::runtime_error("its volume is empty (" + dims_text(_grid.dim) + ")");
    _min = info.min(var, time);
    _max = info.max(var, time);
  } catch (const std::exception &e) {
    throw std::runtime_error("slice_source: cannot read '" + file + "': " + e.what());
  }
}

std::shared_ptr<const volume_slice_data> slice_source::slice(slice_axis a, std::uint64_t index) {
  std::lock_guard<std::mutex> lock(_m);
  for (auto it = _cache.begin(); it != _cache.end(); ++it)
    if (it->axis == a && it->index == index) {
      cached hit = *it;
      _cache.erase(it);
      _cache.push_front(hit);
      return hit.data;
    }
  std::shared_ptr<volume_slice_data> s;
  if (_volume) {
    s = std::make_shared<volume_slice_data>(extract_slice(*_volume, a, index));
  } else {
    const int ax = static_cast<int>(a);
    if (index >= _grid.count(ax))
      throw std::out_of_range("slice_source: slice " + std::to_string(index) + " down " +
                              slice_axis_name(a) + " is outside the " + dims_text(_grid.dim) +
                              " volume in '" + _file + "' (0 to " +
                              std::to_string(_grid.count(ax) - 1) + ")");
    std::uint64_t off[3] = {0, 0, 0};
    std::uint64_t n[3] = {_grid.dim.xdim, _grid.dim.ydim, _grid.dim.zdim};
    off[ax] = index;
    n[ax] = 1;
    volume plane(*_ctx);
    try {
      readVolumeFile(*_ctx, plane, _file, _var, _time, off[0], off[1], off[2],
                     dimension(n[0], n[1], n[2]));
    } catch (const std::exception &e) {
      throw std::runtime_error("slice_source: cannot read slice " + std::to_string(index) +
                               " down " + slice_axis_name(a) + " of '" + _file + "': " + e.what());
    }
    s = std::make_shared<volume_slice_data>(extract_slice(plane, a, 0));
    s->index = index;
    ++_reads;
  }
  _cache.push_front(cached{a, index, s});
  while (_cache.size() > _capacity)
    _cache.pop_back();
  return s;
}

double slice_source::value(const std::uint64_t ijk[3], slice_axis a) {
  for (int k = 0; k < 3; ++k)
    if (ijk[k] >= _grid.count(k))
      throw std::out_of_range("slice_source: voxel (" + std::to_string(ijk[0]) + ", " +
                              std::to_string(ijk[1]) + ", " + std::to_string(ijk[2]) +
                              ") is outside the " + dims_text(_grid.dim) + " volume");
  if (_volume)
    return (*_volume)(ijk[0], ijk[1], ijk[2]);
  int right = 0, up = 1;
  slice_plane_axes(a, right, up);
  const std::shared_ptr<const volume_slice_data> s = slice(a, ijk[static_cast<int>(a)]);
  return s->at(ijk[right], ijk[up]);
}

std::size_t slice_source::reads() const {
  std::lock_guard<std::mutex> lock(_m);
  return _reads;
}

} // namespace cvc
