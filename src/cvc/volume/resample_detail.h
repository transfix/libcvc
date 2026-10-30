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

#ifndef CVC_VOLUME_RESAMPLE_DETAIL_H
#define CVC_VOLUME_RESAMPLE_DETAIL_H

#include <cstddef>
#include <cvc/core/types.h>
#include <limits>
#include <type_traits>

namespace cvc {
namespace resample_detail {

// voxels::resize(dimension) (resizeTrilinearCPU in dimension mode), re-expressed so it can stop
// and resume at any output voxel, with no OpenMP and no cvc::app: contour_spectrum's max_dim
// resample and cvc::resize_stepper (the mesher's resample to (2^n+1)^3).
// Output voxel (i, j, k) samples the source at (i * scale_x, j * scale_y, k * scale_z), scale =
// (source - 1) / (target - 1); the same eight taps, the same edge rule (a tap past the last voxel
// reads the last one) and the same getTriVal arithmetic, then the same conversion to the voxel
// type, so the grid is bit-identical to resize()'s (its CPU path; a CUDA build may differ).
struct resample_grid {
  const unsigned char *src = nullptr;
  unsigned char *dst = nullptr;
  uint64 sx = 0, sy = 0, sz = 0; // source dimensions
  uint64 nx = 0, ny = 0, nz = 0; // target dimensions
  double scale_x = 0.0, scale_y = 0.0, scale_z = 0.0;
};

using resampler = void (*)(const resample_grid &g, uint64 first, uint64 count);

// voxels::setValueRaw's static_cast, made total: an integer type is clamped first, which only
// matters where the cast itself would be undefined (a uint64 or int64 extreme rounds to 2^63 or
// 2^64 as a double); interpolated values otherwise stay within their corners' range.
template <class T> T to_voxel(double v) {
  if constexpr (std::is_integral_v<T>) {
    if (v <= static_cast<double>(std::numeric_limits<T>::lowest()))
      return std::numeric_limits<T>::lowest();
    if (v >= static_cast<double>(std::numeric_limits<T>::max()))
      return std::numeric_limits<T>::max();
  }
  return static_cast<T>(v);
}

// getTriVal (cvc/utility/utility.h) with resX = resY = resZ = 1, as resize() calls it.
inline double trilinear(const double *val, double x_ratio, double y_ratio, double z_ratio) {
  if (x_ratio == 1)
    x_ratio = 0;
  if (y_ratio == 1)
    y_ratio = 0;
  if (z_ratio == 1)
    z_ratio = 0;
  const double temp1 = val[0] + (val[1] - val[0]) * x_ratio;
  const double temp2 = val[4] + (val[5] - val[4]) * x_ratio;
  const double temp3 = val[2] + (val[3] - val[2]) * x_ratio;
  const double temp4 = val[6] + (val[7] - val[6]) * x_ratio;
  const double temp5 = temp1 + (temp3 - temp1) * y_ratio;
  const double temp6 = temp2 + (temp4 - temp2) * y_ratio;
  return temp5 + (temp6 - temp5) * z_ratio;
}

template <class T> void resample_voxels(const resample_grid &g, uint64 first, uint64 count) {
  const T *src = reinterpret_cast<const T *>(g.src);
  T *dst = reinterpret_cast<T *>(g.dst);
  const uint64 sx = g.sx, sxy = g.sx * g.sy;
  uint64 i = first % g.nx;
  const uint64 row = first / g.nx;
  uint64 j = row % g.ny, k = row / g.ny;
  uint64 o = first;
  const uint64 end = first + count;
  while (o < end) {
    const double z = double(k) * g.scale_z;
    const uint64 zi = uint64(z);
    const double zp = z - double(zi);
    const double y = double(j) * g.scale_y;
    const uint64 yi = uint64(y);
    const double yp = y - double(yi);
    for (; i < g.nx && o < end; ++i, ++o) {
      const double x = double(i) * g.scale_x;
      const uint64 xi = uint64(x);
      const double xp = x - double(xi);
      uint64 idx[8];
      idx[0] = zi * sxy + yi * sx + xi;
      idx[1] = idx[0] + 1;
      idx[2] = zi * sxy + (yi + 1) * sx + xi;
      idx[3] = idx[2] + 1;
      idx[4] = (zi + 1) * sxy + yi * sx + xi;
      idx[5] = idx[4] + 1;
      idx[6] = (zi + 1) * sxy + (yi + 1) * sx + xi;
      idx[7] = idx[6] + 1;
      if (xi >= g.sx - 1) {
        idx[1] = idx[0];
        idx[3] = idx[2];
        idx[5] = idx[4];
        idx[7] = idx[6];
      }
      if (yi >= g.sy - 1) {
        idx[2] = idx[0];
        idx[3] = idx[1];
        idx[6] = idx[4];
        idx[7] = idx[5];
      }
      if (zi >= g.sz - 1) {
        idx[4] = idx[0];
        idx[5] = idx[1];
        idx[6] = idx[2];
        idx[7] = idx[3];
      }
      double val[8];
      for (int c = 0; c < 8; ++c)
        val[c] = static_cast<double>(src[static_cast<std::size_t>(idx[c])]);
      dst[static_cast<std::size_t>(o)] = to_voxel<T>(trilinear(val, xp, yp, zp));
    }
    if (i == g.nx) {
      i = 0;
      if (++j == g.ny) {
        j = 0;
        ++k;
      }
    }
  }
}

} // namespace resample_detail
} // namespace cvc

#endif
