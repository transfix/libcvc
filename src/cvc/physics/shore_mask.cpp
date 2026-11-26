/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// shore_mask.cpp — a wake's coast from a shore grid (see shore_mask.h).

#include <algorithm>
#include <cmath>
#include <cvc/physics/shore_mask.h>
#include <stdexcept>

namespace cvc {
namespace physics {

namespace {

// FNV-1a over the bytes of every value the mask reads.
struct digester {
  std::uint64_t h = 0xcbf29ce484222325ull;
  void bytes(const void *p, std::size_t n) {
    const auto *b = static_cast<const unsigned char *>(p);
    for (std::size_t i = 0; i < n; ++i) {
      h ^= b[i];
      h *= 0x100000001b3ull;
    }
  }
  void add(double v) { bytes(&v, sizeof v); }
  void add(std::int64_t v) { bytes(&v, sizeof v); }
  void add(const std::vector<float> &v) {
    add(std::int64_t(v.size()));
    if (!v.empty())
      bytes(v.data(), v.size() * sizeof(float));
  }
};

double smooth(double r) { return r * r * (3.0 - 2.0 * r); }

} // namespace

shore_grid_mask::shore_grid_mask(std::shared_ptr<const cvc::ocean::shore_grid> grid,
                                 std::vector<float> slope,
                                 std::shared_ptr<const cvc::ocean::shore_layer> layer)
    : _grid(std::move(grid)), _slope(std::move(slope)), _layer(std::move(layer)) {
  if (!_grid || _grid->rows < 2 || _grid->cols < 2 || !(_grid->cell > 0.0) ||
      !std::isfinite(_grid->cell) || !std::isfinite(_grid->min_x) || !std::isfinite(_grid->min_y))
    throw std::invalid_argument("shore_grid_mask: the grid must have 2 x 2 samples or more, a "
                                "cell above 0 and a finite corner");
  if (_grid->shore_m.size() != _grid->count())
    throw std::invalid_argument("shore_grid_mask: the grid needs a shore distance on every "
                                "sample");
  if (!_slope.empty() && _slope.size() != _grid->count())
    throw std::invalid_argument("shore_grid_mask: the slope must be one value a sample, or none");
  if (_layer && _layer->empty())
    _layer.reset();
  digester d;
  d.bytes("shore_grid_mask v1", 18);
  const cvc::ocean::shore_grid &g = *_grid;
  d.add(std::int64_t(g.rows));
  d.add(std::int64_t(g.cols));
  d.add(g.min_x);
  d.add(g.min_y);
  d.add(g.cell);
  d.add(g.shore_m);
  d.add(_slope);
  if (_layer) {
    const cvc::ocean::shore_layer &L = *_layer;
    d.add(std::int64_t(L.rows));
    d.add(std::int64_t(L.cols));
    d.add(L.min_x);
    d.add(L.min_y);
    d.add(L.cell);
    d.add(L.reflection);
  } else {
    d.add(std::int64_t(-1));
  }
  _digest = d.h;
}

double shore_grid_mask::sample(const std::vector<float> &f, double x, double y) const {
  const cvc::ocean::shore_grid &g = *_grid;
  const double fc = std::clamp((x - g.min_x) / g.cell, 0.0, double(g.cols - 1));
  const double fr = std::clamp((y - g.min_y) / g.cell, 0.0, double(g.rows - 1));
  const int c0 = std::min(g.cols - 2, int(std::floor(fc))),
            r0 = std::min(g.rows - 2, int(std::floor(fr)));
  const double u = fc - c0, v = fr - r0;
  const std::size_t i = std::size_t(r0) * std::size_t(g.cols) + std::size_t(c0);
  const std::size_t up = std::size_t(g.cols);
  return (f[i] * (1 - u) + f[i + 1] * u) * (1 - v) + (f[i + up] * (1 - u) + f[i + up + 1] * u) * v;
}

double shore_grid_mask::mask(double x, double y) const {
  if (!std::isfinite(x) || !std::isfinite(y))
    return 0.0;
  const double s = sample(_grid->shore_m, x, y);
  if (!(s > 0.0))
    return 0.0; // land
  const double kr = _layer ? _layer->sample_field(_layer->reflection, x, y) : 0.0;
  const double slope = _slope.empty() ? 0.0 : sample(_slope, x, y);
  const double ss = std::clamp((slope - kSteepFrom) / (kSteepTo - kSteepFrom), 0.0, 1.0);
  const double steep = std::min(1.0, std::max(std::isfinite(kr) ? kr : 0.0, smooth(ss)));
  const double r = std::min(1.0, s / kBeachWidth);
  const double beach = kBeachMask + (1.0 - kBeachMask) * smooth(r);
  const double m = beach + (1.0 - beach) * steep;
  return std::isfinite(m) ? std::clamp(m, 0.0, 1.0) : 0.0;
}

} // namespace physics
} // namespace cvc
