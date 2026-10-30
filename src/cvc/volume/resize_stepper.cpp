/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "resample_detail.h"

#include <algorithm>
#include <cvc/volume/resize_stepper.h>
#include <optional>
#include <stdexcept>

namespace cvc {

namespace {
resample_detail::resampler resampler_for(data_type t) {
  switch (t) {
  case UChar:
    return &resample_detail::resample_voxels<unsigned char>;
  case UShort:
    return &resample_detail::resample_voxels<unsigned short>;
  case UInt:
    return &resample_detail::resample_voxels<unsigned int>;
  case Float:
    return &resample_detail::resample_voxels<float>;
  case Double:
    return &resample_detail::resample_voxels<double>;
  case UInt64:
    return &resample_detail::resample_voxels<uint64>;
  case Char:
    return &resample_detail::resample_voxels<signed char>;
  case Int:
    return &resample_detail::resample_voxels<int>;
  case Int64:
    return &resample_detail::resample_voxels<int64>;
  case Undefined:
    break;
  }
  return nullptr;
}
} // namespace

struct resize_stepper::impl {
  std::optional<volume> src, out;
  resample_detail::resample_grid grid;
  resample_detail::resampler fn = nullptr;
  uint64 pos = 0, n = 0;
  bool ready = false;
};

resize_stepper::resize_stepper(const volume &src, const dimension &target)
    : _impl(std::make_unique<impl>()) {
  if (target.isNull())
    throw null_dimension("Null voxels dimension.");
  impl &m = *_impl;
  if (src.voxel_dimensions() == target || target < dimension(2, 2, 2)) {
    // Nothing to do, or resize()'s own way for a grid within 2^3 (it resizes to 2^3 first).
    m.out.emplace(src);
    if (!(src.voxel_dimensions() == target))
      m.out->resize(target);
    m.ready = true;
    return;
  }
  m.fn = resampler_for(src.voxelType());
  if (!m.fn) { // what resize() makes of it
    m.out.emplace(src);
    m.out->resize(target);
    m.ready = true;
    return;
  }
  m.src.emplace(src);
  m.out.emplace(src.ctx(), target, src.voxelType(), src.boundingBox());
  m.out->desc(src.desc());
  m.n = target[0] * target[1] * target[2];
  resample_detail::resample_grid &g = m.grid;
  g.src = m.src->data_ptr();
  g.dst = **m.out; // a fresh volume: its own buffer, which nothing else shares
  g.sx = src.XDim();
  g.sy = src.YDim();
  g.sz = src.ZDim();
  g.nx = target[0];
  g.ny = target[1];
  g.nz = target[2];
  g.scale_x = (double)(src.XDim() - 1) / (target[0] - 1);
  g.scale_y = (double)(src.YDim() - 1) / (target[1] - 1);
  g.scale_z = (double)(src.ZDim() - 1) / (target[2] - 1);
}

resize_stepper::resize_stepper(resize_stepper &&) noexcept = default;
resize_stepper &resize_stepper::operator=(resize_stepper &&) noexcept = default;
resize_stepper::~resize_stepper() = default;

bool resize_stepper::step(std::size_t budget) {
  impl &m = *_impl;
  if (m.ready)
    return true;
  const uint64 count = budget == 0 ? m.n - m.pos : std::min<uint64>(budget, m.n - m.pos);
  m.fn(m.grid, m.pos, count);
  m.pos += count;
  if (m.pos == m.n) {
    m.ready = true;
    m.src.reset();
  }
  return m.ready;
}

bool resize_stepper::done() const { return _impl->ready; }

double resize_stepper::progress() const {
  return _impl->ready ? 1.0 : double(_impl->pos) / double(std::max<uint64>(1, _impl->n));
}

volume resize_stepper::take() {
  if (!_impl->ready || !_impl->out)
    throw std::logic_error("resize_stepper::take: not ready");
  volume v = std::move(*_impl->out);
  _impl->out.reset();
  return v;
}

} // namespace cvc
