/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Filters on a volume the app holds. See inc/cvc/volume/volume_filter.h.

#include <cmath>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_filter.h>
#include <stdexcept>
#include <utility>

namespace cvc {

struct volume_filter_stepper::impl {
  std::shared_ptr<const volume> in;
  std::shared_ptr<volume> out;
  volume_filter_params p;
  std::int64_t nx = 0, ny = 0, nz = 0;
  std::int64_t row = 0, rows = 0; // the next row (j + k * ny) and how many
  bool finished = false;
  // bilateral: the weights, as voxels::bilateralFilter makes them
  int r = 2, diam = 5;
  double range = 0.0;
  double radiometric[256] = {};
  std::vector<double> spatial;

  impl(std::shared_ptr<const volume> v, const volume_filter_params &params)
      : in(std::move(v)), p(params) {
    if (!in)
      throw std::invalid_argument("volume_filter: no volume");
    nx = static_cast<std::int64_t>(in->XDim());
    ny = static_cast<std::int64_t>(in->YDim());
    nz = static_cast<std::int64_t>(in->ZDim());
    if (nx == 0 || ny == 0 || nz == 0)
      throw std::invalid_argument("volume_filter: the volume is empty");
    if (!(p.radius >= 0 && p.radius <= 16))
      throw std::invalid_argument("volume_filter: radius " + std::to_string(p.radius) +
                                  " is not in 0..16");
    if (!(p.spatial_sigma > 0.0) || !(p.radiometric_sigma > 0.0))
      throw std::invalid_argument("volume_filter: spatial_sigma and radiometric_sigma must be > 0");
    rows = ny * nz;
    // A volume of its own: the input's grid and description, its voxels deep-copied (so writing
    // them never touches the input's, which other nodes are showing).
    out = std::make_shared<volume>(*in);
    out->voxels::copy();
    r = static_cast<int>(p.radius);
    diam = 2 * r + 1;
    range = in->max() - in->min();
    if (!(range > 0.0)) { // constant: nothing to blend, and no scale to blend on
      finished = true;
      return;
    }
    for (int c = 0; c < 256; ++c) {
      const double f = c * (range / 255.0);
      radiometric[c] = std::exp(f * f / (-p.radiometric_sigma * p.radiometric_sigma * 2.0));
    }
    spatial.reserve(static_cast<std::size_t>(diam) * diam * diam);
    for (int k = -r; k <= r; ++k)
      for (int j = -r; j <= r; ++j)
        for (int i = -r; i <= r; ++i)
          spatial.push_back(
              std::exp(static_cast<double>(k * k + j * j + i * i) /
                       (-p.spatial_sigma * p.spatial_sigma * 2.0)));
  }

  // One row of the bilateral filter: voxels::bilateralFilter's loop for (j, k).
  void bilateral_row(std::int64_t j, std::int64_t k) {
    const volume &src = *in;
    for (std::int64_t i = 0; i < nx; ++i) {
      const double fsample = src(i, j, k);
      double sum = 0.0, denom = 0.0;
      for (int z = 0; z < diam; ++z) {
        const std::int64_t kk = k + z - r;
        if (kk < 0 || kk >= nz)
          continue;
        for (int y = 0; y < diam; ++y) {
          const std::int64_t jj = j + y - r;
          if (jj < 0 || jj >= ny)
            continue;
          for (int x = 0; x < diam; ++x) {
            const std::int64_t ii = i + x - r;
            if (ii < 0 || ii >= nx)
              continue;
            const double v = src(ii, jj, kk);
            const double nd = std::fabs(fsample - v) / range * 255.0;
            const int c = nd < 255.0 ? static_cast<int>(nd) : 255; // NaN too
            const double w = radiometric[c] * spatial[(z * diam + y) * diam + x];
            denom += w;
            sum += w * v;
          }
        }
      }
      (*out)(i, j, k, sum / denom);
    }
  }
};

volume_filter_stepper::volume_filter_stepper(std::shared_ptr<const volume> in,
                                             const volume_filter_params &p)
    : _impl(std::make_unique<impl>(std::move(in), p)) {}
volume_filter_stepper::~volume_filter_stepper() = default;
volume_filter_stepper::volume_filter_stepper(volume_filter_stepper &&) noexcept = default;
volume_filter_stepper &volume_filter_stepper::operator=(volume_filter_stepper &&) noexcept = default;

bool volume_filter_stepper::step(std::size_t budget) {
  impl &d = *_impl;
  std::size_t voxels = 0;
  while (!d.finished && d.row < d.rows) {
    d.bilateral_row(d.row % d.ny, d.row / d.ny);
    ++d.row;
    voxels += static_cast<std::size_t>(d.nx);
    if (budget && voxels >= budget)
      break;
  }
  if (!d.finished && d.row >= d.rows) {
    d.out->unsetMinMax(); // its range is the filtered voxels', taken when asked
    d.finished = true;
  }
  return d.finished;
}

bool volume_filter_stepper::done() const { return _impl->finished; }

double volume_filter_stepper::progress() const {
  const impl &d = *_impl;
  return d.finished ? 1.0 : static_cast<double>(d.row) / static_cast<double>(d.rows);
}

const char *volume_filter_stepper::phase() const { return "bilateral"; }

std::string volume_filter_stepper::status() const {
  const impl &d = *_impl;
  if (d.finished)
    return "bilateral: " + std::to_string(d.nx) + " x " + std::to_string(d.ny) + " x " +
           std::to_string(d.nz) + " voxels, radius " + std::to_string(d.r);
  return "bilateral " + std::to_string(static_cast<int>(100.0 * progress())) + "%";
}

std::shared_ptr<const volume> volume_filter_stepper::take() {
  if (!_impl->finished)
    throw std::logic_error("volume_filter: take() before the filter is done");
  return std::shared_ptr<const volume>(std::move(_impl->out));
}

std::unique_ptr<result_job<std::shared_ptr<const volume>>>
volume_filter_job(std::shared_ptr<const volume> in, const volume_filter_params &p) {
  return make_result_job(volume_filter_stepper(std::move(in), p));
}

volume filter_volume(const volume &in, const volume_filter_params &p) {
  volume_filter_stepper s(std::make_shared<const volume>(in), p);
  s.step(0);
  return *s.take();
}

} // namespace cvc
