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

// cvc::compute_contour_spectrum and cvc::contour_spectrum_builder (see
// inc/cvc/volume/contour_spectrum.h).
//
// Third-party credit: the signature functions and their closed-form per-tetrahedron integrals are
// those of C. L. Bajaj, V. Pascucci and D. R. Schikore, "The Contour Spectrum", IEEE Visualization
// 1997, as implemented in libContour (Dan Schikore 1997, updated by Emilio Camahort 1999), which
// libcvc vendors under src/cvc/geometry/cvc-mesher/contour (THIRD_PARTY_NOTICES.md, the contour
// library). This file re-implements its regular-grid path: Datareg3::compLength / compArea /
// compMaxArea / compGradient (datareg3.cpp) over sortVerts / tetSurfIntegral / tetVolIntegral
// (compute.h). Where the port keeps one of the original's quirks on purpose, or departs from it
// (the surface area's middle term, the start bucket, max_volume), the comment says so.
//
// The computation is a resumable state machine (contour_spectrum_builder::impl): every loop that
// grows with the volume -- the default-domain scan, the max_dim resample, the conversion of each
// plane onto the integration scale and the cell loop -- keeps its position between steps and stops
// when the step's budget is spent, so no step does more than one cell (or one O(samples) setup)
// past its budget. compute_contour_spectrum drives the same machine, so stepping never changes a
// result: every value is computed by the same expressions in the same order.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cvc/geometry/geometry.h>
#include <cvc/volume/contour_mesh.h>
#include <cvc/volume/contour_spectrum.h>
#include <cvc/volume/volume.h>
#include <exception>
#include <limits>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace cvc {
namespace {

// Values are integrated with the domain mapped affinely onto [0, kScale]. sortVerts separates
// equal vertex values by an ABSOLUTE floor of 1e-5, tuned on uchar data: on this scale the floor
// keeps that meaning whatever the data range (on raw values a volume spanning 1e-3 would smear
// every flat tetrahedron over 7% of the domain, and one spanning 1e12 would lose the separation to
// rounding). Area and volume are invariant under the map, since every formula uses value ratios;
// the gradient weight scales by the slope squared, which the final pass undoes.
constexpr double kScale = 255.0;

// The budget's units, weighted so one unit is about the same time whatever the phase: the time to
// scan one voxel for the domain (~5 ns on a desktop core), measured against the other kinds of
// work with this file's own code (MSVC /O2 /fp:strict; a cell's fixed part, five tetrahedra with
// their sorts, cross sections, square roots and divisions, measured 150 to 600 voxel scans
// depending on the machine's load, a sample update 2.3).
constexpr uint64 kScanUnits = 1;        // one voxel scanned for the default domain
constexpr uint64 kResampleUnits = 8;    // one voxel of the max_dim grid (eight reads, seven lerps)
constexpr uint64 kConvertUnits = 1;     // one voxel mapped onto the integration scale
constexpr uint64 kCellUnits = 200;      // one cell's fixed part
constexpr uint64 kUpdateUnits = 2;      // one (tetrahedron, sample) update of a cell
constexpr uint64 kSkippedCellUnits = 2; // one cell with a non-finite corner
constexpr uint64 kCheckedCellUnits = 2; // one cell only checked for one (a degenerate domain)
// once: the accumulators (kSampleUnits per sample), and the final pass, kSampleUnits per sample
constexpr uint64 kSampleUnits = 1;

// compute_contour_spectrum's step size when it has a progress callback to call (once per step):
// ~20 ms of work.
constexpr std::size_t kOneShotBudget = std::size_t(1) << 22;

constexpr uint64 kUnlimited = std::numeric_limits<uint64>::max();

struct vec3 {
  double x, y, z;
};

vec3 operator-(const vec3 &a, const vec3 &b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
vec3 operator*(double s, const vec3 &a) { return {s * a.x, s * a.y, s * a.z}; }
vec3 cross(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double dot(const vec3 &a, const vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double length(const vec3 &a) { return std::sqrt(dot(a, a)); }
// (1 - t) * a + t * b, the edge point the original forms.
vec3 mix(const vec3 &a, const vec3 &b, double t) {
  return {(1.0 - t) * a.x + t * b.x, (1.0 - t) * a.y + t * b.y, (1.0 - t) * a.z + t * b.z};
}

// Cell corners in Datareg3::getCellValues order, as offsets from voxel (i, j, k).
constexpr int kCorner[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1},
                               {0, 1, 0}, {1, 1, 0}, {1, 1, 1}, {0, 1, 1}};
// The five tetrahedra compLength / compArea / compGradient integrate per cell, in their argument
// order (it decides which vertex sortVerts keeps first on a tie). Every cell is split the same
// way, so neighbouring cells cut a shared face along different diagonals; kept for parity.
constexpr int kTet[5][4] = {{0, 1, 3, 4}, {3, 1, 2, 6}, {6, 1, 5, 4}, {7, 3, 6, 4}, {6, 3, 1, 4}};

struct accumulator {
  std::size_t n = 0;
  // The weight of the middle term of the cross-section area on [v2, v3] (Bernstein form
  // (1-s)^2 area1 + 2 s(1-s) mid + s^2 area2, where mid makes the area C1 at v2 and v3): 2 is
  // exact, libContour's tetSurfIntegral uses 1.
  double mid_weight = 2.0;
  std::vector<double> fx; // sample isovalues on the integration scale
  std::vector<double> area, grad, vol;
  // Difference array of the volume of tetrahedra lying wholly below a sample: the original added
  // the full volume to every such bucket, one at a time, for every tetrahedron.
  std::vector<double> vol_tail;

  explicit accumulator(std::size_t samples)
      : n(samples), fx(samples), area(samples, 0.0), grad(samples, 0.0), vol(samples, 0.0),
        vol_tail(samples + 1, 0.0) {
    for (std::size_t b = 0; b < n; ++b)
      fx[b] = kScale * static_cast<double>(b) / static_cast<double>(n - 1);
  }

  // The first sample at or above v (n if none). The original's ceil((n-1)(v1-min)/(max-min)),
  // but clamped (below the domain it wrapped to a huge unsigned index and dropped the
  // tetrahedron, so min_volume lost it) and nudged so rounding never puts a sample on the wrong
  // side of v, which keeps every interpolation parameter below in [0, 1).
  std::size_t first_at_or_above(double v) const {
    const double t = std::ceil(v * static_cast<double>(n - 1) / kScale);
    std::size_t b =
        !(t > 0.0) ? 0 : (t >= static_cast<double>(n) ? n : static_cast<std::size_t>(t));
    while (b > 0 && fx[b - 1] >= v)
      --b;
    while (b < n && fx[b] < v)
      ++b;
    return b;
  }
};

// sortVerts (compute.h): the same compare-exchange sequence, so equal values keep the original's
// vertex order, then the same separation of (nearly) equal values.
void sort_verts(int o[4], double f[4]) {
  const auto exchange = [&](int a, int b) {
    if (f[b] < f[a]) {
      std::swap(f[a], f[b]);
      std::swap(o[a], o[b]);
    }
  };
  exchange(2, 3);
  exchange(1, 2);
  exchange(0, 1);
  exchange(2, 3);
  exchange(1, 2);
  exchange(2, 3);
  const double eps = std::max(1e-5, (f[3] - f[1]) / 4000.0);
  if (f[1] <= f[0] + eps)
    f[1] += eps;
  if (f[2] <= f[1] + eps)
    f[2] += 2 * eps;
  if (f[3] <= f[2] + eps)
    f[3] += 4 * eps;
}

// One tetrahedron: tetSurfIntegral twice (scaling 1 for the area, the cell's squared gradient
// `weight` for the gradient) and tetVolIntegral, which share sortVerts and the two cross sections.
// `x` and `f` are the cell's corner positions and values (integration scale). Returns the number
// of samples it updated (the work it did beyond its fixed cost).
std::size_t integrate_tet(accumulator &acc, const vec3 *x, const double *f, const int *tet,
                          double weight, double tet_volume) {
  int o[4] = {tet[0], tet[1], tet[2], tet[3]};
  double v[4] = {f[o[0]], f[o[1]], f[o[2]], f[o[3]]};
  sort_verts(o, v);
  const vec3 &x1 = x[o[0]], &x2 = x[o[1]], &x3 = x[o[2]], &x4 = x[o[3]];
  const double v1 = v[0], v2 = v[1], v3 = v[2], v4 = v[3];

  // The cross sections at v2 (through x2, cutting edges 1-3 and 1-4) and at v3 (through x3,
  // cutting edges 2-4 and 1-4), with the original's guards.
  const vec3 a13 = mix(x3, x1, v1 != v3 ? (v3 - v2) / (v3 - v1) : 0.0);
  const vec3 a14 = mix(x4, x1, v1 != v4 ? (v4 - v2) / (v4 - v1) : 0.0);
  const double area1 = 0.5 * length(cross(a13 - x2, a14 - x2));
  const vec3 b24 = mix(x4, x2, v2 != v4 ? (v4 - v3) / (v4 - v2) : 0.0);
  const vec3 b14 = mix(x4, x1, v4 != v1 ? (v4 - v3) / (v4 - v1) : 0.0);
  const double area2 = 0.5 * length(cross(b24 - x3, b14 - x3));

  // Both kernels fall back to this middle coefficient when neither end interval has a length
  // (unreachable once sort_verts has separated the values, short of magnitudes that swallow its
  // epsilon): twice the area of the cross section halfway between v1 = v2 and v3 = v4.
  const double cross_mid = 2.0 * length(cross(0.5 * (x2 - x1), 0.5 * (x4 - x3)));

  // tetSurfIntegral's middle coefficient, the one that keeps the area C1 at v2 (or at v3).
  double mid_area;
  if (v2 - v1 != 0.0)
    mid_area = area1 * (1.0 + (v3 - v2) / (v2 - v1));
  else if (v4 - v3 != 0.0)
    mid_area = area2 * (1.0 + (v3 - v2) / (v4 - v3));
  else
    mid_area = cross_mid - 0.5 * (area1 + area2);

  // tetVolIntegral picks it by the longer end interval instead.
  const double d21 = v2 - v1, d43 = v4 - v3;
  double mid_vol;
  if (d21 >= d43 && d21 > 0.0)
    mid_vol = area1 * (1.0 + (v3 - v2) / d21);
  else if (d43 > d21)
    mid_vol = area2 * (1.0 + (v3 - v2) / d43);
  else
    mid_vol = cross_mid - 0.5 * (area1 + area2); // the original divided by zero here
  // The piecewise cubic is normalized to reach the tetrahedron's volume at v4.
  const double vol2 = ((v3 - v1) * area1 + (v3 - v2) * mid_vol + (v4 - v2) * area2) / 3.0;
  double factor = vol2 > 0.0 ? tet_volume / vol2 : 0.0;
  if (!std::isfinite(factor))
    factor = 0.0;
  const double cum1 = area1 * (v2 - v1) / 3.0;
  const double cum2 = cum1 + (area1 / 3.0 + mid_vol / 3.0 + area2 / 3.0) * (v3 - v2);

  // The sample ranges [v1, v2), [v2, v3), [v3, v4) are each empty when their ends coincide, so the
  // divisions below are safe and the original's "v1 == v3" / "v4 == v2" branches never fire.
  const double *fx = acc.fx.data();
  const std::size_t n = acc.n;
  const std::size_t first = acc.first_at_or_above(v1);
  std::size_t b = first;
  for (; b < n && fx[b] < v2; ++b) {
    const double s = (fx[b] - v1) / (v2 - v1);
    const double a = s * s * area1;
    acc.area[b] += a;
    acc.grad[b] += weight * a;
    acc.vol[b] += factor * s * s * s * area1 * (v2 - v1) / 3.0;
  }
  for (; b < n && fx[b] < v3; ++b) {
    const double s = (fx[b] - v2) / (v3 - v2);
    const double t = 1.0 - s, s2 = s * s, s3 = s2 * s;
    const double a = t * t * area1 + acc.mid_weight * s * t * mid_area + s2 * area2;
    acc.area[b] += a;
    acc.grad[b] += weight * a;
    acc.vol[b] += factor * (cum1 + (area1 * (s - s2 + s3 / 3.0) +
                                    2.0 * mid_vol * (s2 / 2.0 - s3 / 3.0) + area2 * (s3 / 3.0)) *
                                       (v3 - v2));
  }
  for (; b < n && fx[b] < v4; ++b) {
    const double s = (fx[b] - v3) / (v4 - v3);
    const double t = 1.0 - s;
    const double a = t * t * area2;
    acc.area[b] += a;
    acc.grad[b] += weight * a;
    acc.vol[b] += factor * (cum2 + area2 * (s - s * s + s * s * s / 3.0) * (v4 - v3));
  }
  acc.vol_tail[b] += tet_volume;
  return b - first;
}

// One triangle of a surface, where the isovalues cut segments rather than surfaces: the segment's
// length (into area), the area below each sample (into vol) and |grad f|^2 times the length (into
// grad), all exact, since f is linear on the triangle. `x` and `f` are its corners and values;
// `weight` is |grad f|^2 on the scale `f` is on. Returns the number of samples it updated.
std::size_t integrate_tri(accumulator &acc, const vec3 *x, const double *f, double weight,
                          double tri_area) {
  int o[3] = {0, 1, 2};
  double v[3] = {f[0], f[1], f[2]};
  const auto exchange = [&](int a, int b) {
    if (v[b] < v[a]) {
      std::swap(v[a], v[b]);
      std::swap(o[a], o[b]);
    }
  };
  exchange(0, 1);
  exchange(1, 2);
  exchange(0, 1);
  const vec3 &x1 = x[o[0]], &x2 = x[o[1]], &x3 = x[o[2]];
  const double v1 = v[0], v2 = v[1], v3 = v[2];
  // The longest cut: through x2 at v2, ending on edge 1-3. Every other cut is a scaled copy of it.
  const double mid = v3 > v1 ? length(x2 - mix(x1, x3, (v2 - v1) / (v3 - v1))) : 0.0;
  // A sample range below is empty when its ends coincide, so no division is by zero.
  const double *fx = acc.fx.data();
  const std::size_t n = acc.n;
  const std::size_t first = acc.first_at_or_above(v1);
  std::size_t b = first;
  for (; b < n && fx[b] < v2; ++b) {
    const double d = fx[b] - v1;
    const double len = mid * d / (v2 - v1);
    acc.area[b] += len;
    acc.grad[b] += weight * len;
    acc.vol[b] += tri_area * d * d / ((v2 - v1) * (v3 - v1));
  }
  for (; b < n && fx[b] < v3; ++b) {
    const double d = v3 - fx[b];
    const double len = mid * d / (v3 - v2);
    acc.area[b] += len;
    acc.grad[b] += weight * len;
    acc.vol[b] += tri_area * (1.0 - d * d / ((v3 - v1) * (v3 - v2)));
  }
  acc.vol_tail[b] += tri_area;
  return b - first;
}

// ---- typed kernels (one set per voxel type, picked once per builder) ----------------------------

// Voxels [first, first + count) mapped onto the integration scale.
using plane_reader = void (*)(const unsigned char *raw, std::size_t first, std::size_t count,
                              double lo, double inv, double *out);

template <class T>
void read_plane(const unsigned char *raw, std::size_t first, std::size_t count, double lo,
                double inv, double *out) {
  const T *p = reinterpret_cast<const T *>(raw) + first;
  for (std::size_t i = 0; i < count; ++i)
    out[i] = (static_cast<double>(p[i]) - lo) * inv;
}

// The running finite extrema of the voxels scanned so far.
struct extrema_state {
  bool any = false;
  double lo = 0.0, hi = 0.0;
};

using extrema_scanner = void (*)(const unsigned char *raw, std::size_t first, std::size_t count,
                                 extrema_state &s);

template <class T>
void scan_extrema(const unsigned char *raw, std::size_t first, std::size_t count,
                  extrema_state &s) {
  const T *p = reinterpret_cast<const T *>(raw) + first;
  bool any = s.any;
  double lo = s.lo, hi = s.hi;
  for (std::size_t i = 0; i < count; ++i) {
    const double v = static_cast<double>(p[i]);
    if constexpr (std::is_floating_point_v<T>) {
      if (!std::isfinite(v))
        continue;
    }
    if (!any) {
      lo = hi = v;
      any = true;
    } else if (v < lo) {
      lo = v;
    } else if (v > hi) {
      hi = v;
    }
  }
  s.any = any;
  s.lo = lo;
  s.hi = hi;
}

// The max_dim resample: voxels::resize(dimension) (resizeTrilinearCPU in dimension mode),
// re-expressed so it can stop and resume at any output voxel, with no OpenMP and no cvc::app.
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
double trilinear(const double *val, double x_ratio, double y_ratio, double z_ratio) {
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

struct kernels {
  extrema_scanner scan = nullptr;
  resampler resample = nullptr;
  plane_reader read = nullptr;
};

// Calls fn with a null pointer of the voxel type's C++ type.
template <class F> auto visit_voxel_type(data_type t, F &&fn) {
  switch (t) {
  case UChar:
    return fn(static_cast<const unsigned char *>(nullptr));
  case UShort:
    return fn(static_cast<const unsigned short *>(nullptr));
  case UInt:
    return fn(static_cast<const unsigned int *>(nullptr));
  case Float:
    return fn(static_cast<const float *>(nullptr));
  case Double:
    return fn(static_cast<const double *>(nullptr));
  case UInt64:
    return fn(static_cast<const uint64 *>(nullptr));
  case Char:
    return fn(static_cast<const signed char *>(nullptr));
  case Int:
    return fn(static_cast<const int *>(nullptr));
  case Int64:
    return fn(static_cast<const int64 *>(nullptr));
  case Undefined:
    break;
  }
  throw contour_spectrum_error("compute_contour_spectrum: undefined voxel type");
}

kernels kernels_for(data_type t) {
  return visit_voxel_type(t, [](auto tag) {
    using T = std::remove_const_t<std::remove_pointer_t<decltype(tag)>>;
    return kernels{&scan_extrema<T>, &resample_voxels<T>, &read_plane<T>};
  });
}

// A signature function (or total_volume) past the double range: refused, so a returned spectrum
// is always finite.
contour_spectrum_error overflow_error() {
  return contour_spectrum_error(
      "compute_contour_spectrum: a signature function overflows a double (data too steep for the "
      "gradient, or a bounding box too large for the area or volume)");
}

// a * b, or false when it does not fit in a size_t (wasm32 has a 32-bit one).
bool fits_size(uint64 a, uint64 b, uint64 &out) {
  const uint64 limit = std::numeric_limits<std::size_t>::max();
  if (a != 0 && b > limit / a)
    return false;
  out = a * b;
  return true;
}

} // namespace

double contour_spectrum::at(const std::vector<double> contour_spectrum::*fn, double value) const {
  if (!fn)
    return 0.0;
  const std::vector<double> &f = this->*fn;
  if (f.empty())
    return 0.0;
  if (f.size() == 1 || !(domain_max != domain_min))
    return f.front();
  const double t = (value - domain_min) / (domain_max - domain_min);
  if (!(t > 0.0)) // also a NaN isovalue
    return f.front();
  if (!(t < 1.0))
    return f.back();
  const double pos = t * static_cast<double>(f.size() - 1);
  const std::size_t i = static_cast<std::size_t>(pos);
  if (i + 1 >= f.size())
    return f.back();
  const double frac = pos - static_cast<double>(i);
  return f[i] * (1.0 - frac) + f[i + 1] * frac;
}

// ---- the builder --------------------------------------------------------------------------------

struct contour_spectrum_builder::impl {
  enum class stage { start, domain, resample, integrate, finish, done, failed };

  contour_spectrum_options opt;
  // The shallow copy the computation reads (shares the voxels); released once it is not needed.
  std::optional<volume> src;
  // vol's min/max cache as it was when the builder was made: the default domain uses the set
  // values, never vol.min()/max() on an unset cache (which would write it).
  bool min_set = false, max_set = false;
  double min_value = 0.0, max_value = 0.0;

  stage st = stage::start;
  std::exception_ptr error;
  double failed_progress = 0.0;
  uint64 work = 0; // units done
  uint64 left = 0; // units left in the current step
  kernels fn;

  // "domain"
  extrema_state ext;
  uint64 scan_pos = 0, scan_count = 0;
  double dmin = 0.0, dmax = 0.0, range = 0.0, inv = 0.0;

  // The grid integrated (max_dim caps every axis; resampling keeps the bounding box, so its
  // spacing is volume::XSpan() etc. of the new dimensions).
  uint64 nx = 0, ny = 0, nz = 0, nvox = 0, ncells = 0;
  std::size_t plane = 0;
  double sx = 0.0, sy = 0.0, sz = 0.0, cell_volume = 0.0;
  bool resampling = false, degenerate = false;
  bool real_type = false; // Float or Double: a voxel can be NaN or infinite

  // "resampling": the grid, in the volume's own voxel type (double storage only for alignment)
  std::unique_ptr<double[]> resampled;
  resample_grid rg;
  uint64 res_pos = 0;

  // "integrating"
  const unsigned char *raw = nullptr; // the voxels integrated: src's, or the resampled grid
  std::optional<accumulator> acc;
  vec3 corner[8] = {};
  double tet_volume[5] = {};
  std::unique_ptr<double[]> below, above;  // planes ck and ck + 1, mapped onto the scale
  uint64 below_ready = 0, above_ready = 0; // voxels of each converted so far
  uint64 ck = 0, cj = 0, ci = 0;           // the next cell
  uint64 skipped = 0;                      // cells with a non-finite corner

  // the final pass ("integrating" too): one sample per unit into the result
  contour_spectrum cs;
  std::size_t fin_pos = 0;
  bool finish_started = false, taken = false;
  double tail = 0.0, grad_scale = 0.0;

  // progress() weights: each phase's units, the integration's without its sample updates (which
  // are not known in advance; progress there is the fraction of cells done)
  double w_domain = 0.0, w_resample = 0.0, w_integrate = 0.0, w_finish = 0.0;

  impl(const volume &vol, const contour_spectrum_options &o)
      : opt(o), min_set(vol.minIsSet()), max_set(vol.maxIsSet()) {
    if (min_set)
      min_value = vol.min();
    if (max_set)
      max_value = vol.max();
    src.emplace(vol);
  }

  void spend(uint64 units) {
    work += units;
    left = units >= left ? 0 : left - units;
  }

  void run(uint64 budget) {
    left = budget;
    while (left > 0) {
      switch (st) {
      case stage::start:
        begin();
        break;
      case stage::domain:
        scan();
        break;
      case stage::resample:
        resample();
        break;
      case stage::integrate:
        integrate();
        break;
      case stage::finish:
        finish();
        break;
      case stage::done:
      case stage::failed:
        return;
      }
    }
  }

  // The input checks, the grid and the plan; no units.
  void begin() {
    const volume &v = *src;
    if (v.XDim() < 2 || v.YDim() < 2 || v.ZDim() < 2)
      throw contour_spectrum_error("compute_contour_spectrum: every axis needs at least 2 voxels");
    if (v.voxelType() == Undefined)
      throw contour_spectrum_error("compute_contour_spectrum: undefined voxel type");
    if (opt.samples < 2)
      throw contour_spectrum_error("compute_contour_spectrum: at least 2 samples are needed");
    fn = kernels_for(v.voxelType());
    real_type = v.voxelType() == Float || v.voxelType() == Double;

    dimension grid = v.voxel_dimensions();
    if (opt.max_dim > 0) {
      const uint64 cap = std::max<uint64>(opt.max_dim, 2);
      grid = dimension(std::min(v.XDim(), cap), std::min(v.YDim(), cap), std::min(v.ZDim(), cap));
    }
    nx = grid.xdim;
    ny = grid.ydim;
    nz = grid.zdim;
    uint64 plane64 = 0, bytes = 0;
    if (!fits_size(nx, ny, plane64) || !fits_size(plane64, nz, nvox) ||
        !fits_size(plane64, sizeof(double), bytes) || !fits_size(nvox, v.voxelSize(), bytes))
      throw contour_spectrum_error("compute_contour_spectrum: the volume is too large to address");
    plane = static_cast<std::size_t>(plane64);
    const auto spacing = [&](double lo, double hi, uint64 len) {
      return opt.world_units ? (hi - lo) / static_cast<double>(len - 1) : 1.0;
    };
    sx = spacing(v.XMin(), v.XMax(), nx);
    sy = spacing(v.YMin(), v.YMax(), ny);
    sz = spacing(v.ZMin(), v.ZMax(), nz);
    cell_volume = sx * sy * sz;
    ncells = (nx - 1) * (ny - 1) * (nz - 1);
    resampling = grid != v.voxel_dimensions();

    const bool scanning = !opt.has_domain && !(min_set && max_set);
    const uint64 source_voxels = v.XDim() * v.YDim() * v.ZDim();
    const double samples = static_cast<double>(opt.samples);
    w_domain = scanning ? double(kScanUnits) * static_cast<double>(source_voxels) : 0.0;
    w_resample = resampling ? double(kResampleUnits) * static_cast<double>(nvox) : 0.0;
    w_integrate = double(kConvertUnits) * static_cast<double>(nvox) +
                  double(kCellUnits) * static_cast<double>(ncells) + double(kSampleUnits) * samples;
    w_finish = double(kSampleUnits) * samples;

    if (opt.has_domain) {
      dmin = opt.domain_min;
      dmax = opt.domain_max;
      if (dmax < dmin)
        std::swap(dmin, dmax);
      plan();
    } else if (!scanning) {
      dmin = min_value;
      dmax = max_value;
      plan();
    } else {
      scan_count = source_voxels;
      st = stage::domain;
    }
  }

  // The number of items of `cost` units each that the rest of this step's budget pays for (at
  // least 1: the last one may overrun it).
  uint64 affordable(uint64 cost, uint64 remaining) const {
    return std::min(remaining, left / cost + (left % cost != 0));
  }

  // "domain": the finite extrema of the source voxels, `left` at a time.
  void scan() {
    const uint64 chunk = affordable(kScanUnits, scan_count - scan_pos);
    fn.scan(src->data_ptr(), static_cast<std::size_t>(scan_pos), static_cast<std::size_t>(chunk),
            ext);
    scan_pos += chunk;
    spend(chunk * kScanUnits);
    if (scan_pos == scan_count) {
      // [vol.min(), vol.max()]: the set value, or the finite data extremum (0 when none).
      dmin = min_set ? min_value : ext.lo;
      dmax = max_set ? max_value : ext.hi;
      plan();
    }
  }

  // After the domain: the degenerate answer, or the resample, or straight to the integration.
  void plan() {
    if (!std::isfinite(dmin) || !std::isfinite(dmax))
      throw contour_spectrum_error("compute_contour_spectrum: the isovalue domain is not finite");
    range = dmax - dmin;
    // The gradient goes back to data units by (range / kScale)^2, which overflows past kScale *
    // sqrt(DBL_MAX) (~3.4e156), as the width itself does past DBL_MAX (a finite domain such as
    // [-DBL_MAX, DBL_MAX], or a -DBL_MAX nodata value next to ordinary data under the default
    // domain): the isovalues (when the width overflows) and the gradient would come back NaN or
    // infinite, so such a domain is refused.
    grad_scale = (range / kScale) * (range / kScale);
    if (!std::isfinite(range) || !std::isfinite(grad_scale))
      throw contour_spectrum_error("compute_contour_spectrum: the isovalue domain is too wide to "
                                   "scale (wider than 255 * sqrt(DBL_MAX), ~3.4e156)");
    inv = kScale / range;
    degenerate = !(range > 0.0) || !std::isfinite(inv);
    if (degenerate) {
      // All-zero functions, but total_volume leaves out the cells with a non-finite corner as on
      // any other domain, so a voxel type that can hold one has the cells of the grid integrated
      // (after the max_dim resample) checked, and nothing else: mapped with lo = 0 and inv = 0, a
      // finite voxel reads 0 and a NaN or infinite one NaN. An integer type has nothing to find.
      inv = 0.0;
      if (!real_type) {
        st = stage::finish; // no resample for nothing
        return;
      }
      // Never more than begin()'s estimate, so progress() stays monotone.
      w_integrate = double(kConvertUnits) * static_cast<double>(nvox) +
                    double(kCheckedCellUnits) * static_cast<double>(ncells);
    }
    st = resampling ? stage::resample : stage::integrate;
  }

  // "resampling": `left` voxels of the grid at a time into our own buffer (vol is untouched).
  void resample() {
    if (!resampled) {
      const volume &v = *src;
      const std::size_t bytes = static_cast<std::size_t>(nvox * v.voxelSize());
      resampled.reset(new double[(bytes + sizeof(double) - 1) / sizeof(double)]);
      rg.src = v.data_ptr();
      rg.dst = reinterpret_cast<unsigned char *>(resampled.get());
      rg.sx = v.XDim();
      rg.sy = v.YDim();
      rg.sz = v.ZDim();
      rg.nx = nx;
      rg.ny = ny;
      rg.nz = nz;
      // resize()'s inSpace
      rg.scale_x = (double)(v.XDim() - 1) / (nx - 1);
      rg.scale_y = (double)(v.YDim() - 1) / (ny - 1);
      rg.scale_z = (double)(v.ZDim() - 1) / (nz - 1);
    }
    const uint64 chunk = affordable(kResampleUnits, nvox - res_pos);
    fn.resample(rg, res_pos, chunk);
    res_pos += chunk;
    spend(chunk * kResampleUnits);
    if (res_pos == nvox) {
      raw = rg.dst;
      rg.src = nullptr;
      src.reset(); // the source is no longer read
      st = stage::integrate;
    }
  }

  // Converts voxels [ready, need) of plane `index` into `out`, at most `left` of them.
  void convert(double *out, uint64 index, uint64 &ready, uint64 need) {
    const uint64 chunk = affordable(kConvertUnits, need - ready);
    fn.read(raw, static_cast<std::size_t>(index * plane + ready), static_cast<std::size_t>(chunk),
            degenerate ? 0.0 : dmin, inv, out + ready);
    ready += chunk;
    spend(chunk * kConvertUnits);
  }

  // "integrating": the cells in (k, j, i) order, converting the rows they need just in time.
  // Under a degenerate domain the cells are only checked for a non-finite corner.
  void integrate() {
    if (!below) {
      if (!raw)
        raw = src->data_ptr();
      below.reset(new double[plane]); // uninitialized: every voxel is converted before use
      above.reset(new double[plane]);
      if (!degenerate) {
        acc.emplace(opt.samples);
        acc->mid_weight = opt.libcontour_area ? 1.0 : 2.0;
        // Every cell has the same corner positions relative to its first voxel (the integrals
        // are translation invariant), so the geometry and the five tetrahedron volumes are
        // computed once.
        for (int c = 0; c < 8; ++c)
          corner[c] = {kCorner[c][0] * sx, kCorner[c][1] * sy, kCorner[c][2] * sz};
        for (int t = 0; t < 5; ++t) {
          const vec3 &p0 = corner[kTet[t][0]];
          tet_volume[t] = std::fabs(dot(corner[kTet[t][3]] - p0,
                                        cross(corner[kTet[t][1]] - p0, corner[kTet[t][2]] - p0))) /
                          6.0;
        }
        spend(kSampleUnits * opt.samples);
      }
    }
    const std::size_t row = static_cast<std::size_t>(nx);
    while (left > 0) {
      // Cell row cj of slab ck reads rows cj and cj + 1 of planes ck and ck + 1.
      const uint64 need = (cj + 2) * nx;
      if (below_ready < need) {
        convert(below.get(), ck, below_ready, need);
        continue;
      }
      if (above_ready < need) {
        convert(above.get(), ck + 1, above_ready, need);
        continue;
      }
      const double *lower = below.get(), *upper = above.get();
      const std::size_t base = static_cast<std::size_t>(cj) * row;
      while (ci + 1 < nx) {
        const std::size_t p = base + static_cast<std::size_t>(ci);
        const double c[8] = {lower[p],           lower[p + 1],   upper[p + 1],
                             upper[p],           lower[p + row], lower[p + row + 1],
                             upper[p + row + 1], upper[p + row]};
        ++ci;
        bool finite = true;
        for (double value : c)
          finite = finite && std::isfinite(value);
        if (!finite) { // a NaN or infinite voxel: the cell has no defined isosurface
          ++skipped;
          spend(kSkippedCellUnits);
        } else if (degenerate) {
          spend(kCheckedCellUnits); // nothing to integrate
        } else {
          accumulator &a = *acc;
          // compGradient's weight: the squared cell-average forward difference, in index units.
          const double gx = (-c[0] + c[1] - c[4] + c[5] - c[3] + c[2] - c[7] + c[6]) / 4.0;
          const double gy = (-c[0] - c[1] + c[4] + c[5] - c[3] - c[2] + c[7] + c[6]) / 4.0;
          const double gz = (-c[0] - c[1] - c[4] - c[5] + c[3] + c[2] + c[7] + c[6]) / 4.0;
          const double weight = gx * gx + gy * gy + gz * gz;
          std::size_t updates = 0;
          for (int t = 0; t < 5; ++t)
            updates += integrate_tet(a, corner, c, kTet[t], weight, tet_volume[t]);
          spend(kCellUnits + kUpdateUnits * static_cast<uint64>(updates));
        }
        if (left == 0)
          break;
      }
      if (ci + 1 < nx)
        return; // the budget ran out mid-row
      ci = 0;
      if (++cj + 1 < ny)
        continue;
      cj = 0; // the slab is done: plane ck + 1 (all converted) becomes the lower one
      std::swap(below, above);
      below_ready = plane;
      above_ready = 0;
      if (++ck + 1 >= nz) {
        st = stage::finish;
        return;
      }
    }
  }

  // The final pass, one sample at a time.
  void finish() {
    const std::size_t n = opt.samples;
    if (!finish_started) {
      finish_started = true;
      below.reset();
      above.reset();
      resampled.reset();
      raw = nullptr;
      src.reset();
      cs.domain_min = dmin;
      cs.domain_max = dmax;
      // Without the cells with a non-finite corner whatever the domain (a degenerate one checked
      // them too, unless the voxel type cannot hold such a value).
      cs.total_volume = static_cast<double>(ncells - skipped) * cell_volume;
      if (!std::isfinite(cs.total_volume))
        throw overflow_error();
      for (std::vector<double> *f :
           {&cs.isovalue, &cs.surface_area, &cs.min_volume, &cs.max_volume, &cs.gradient})
        f->reserve(n);
    }
    while (left > 0 && fin_pos < n) {
      const std::size_t b = fin_pos++;
      cs.isovalue.push_back(b + 1 == n
                                ? dmax
                                : dmin + (static_cast<double>(b) / static_cast<double>(n - 1)) *
                                             (dmax - dmin));
      if (acc) {
        tail += acc->vol_tail[b];
        const double area = acc->area[b], min_volume = acc->vol[b] + tail;
        // grad_scale is finite (plan() refuses a domain where it is not), but the product can
        // still pass the double range: data rising faster than ~1e154 per voxel.
        const double gradient = acc->grad[b] * grad_scale;
        if (!std::isfinite(area) || !std::isfinite(min_volume) || !std::isfinite(gradient))
          throw overflow_error();
        cs.surface_area.push_back(area);
        cs.min_volume.push_back(min_volume);
        // Datareg3::compMaxArea reports min_volume.back() - min_volume, which drops a plateau at
        // the maximum (min_volume excludes it at every sample); total - min_volume is {f >= w}
        // itself.
        cs.max_volume.push_back(std::max(0.0, cs.total_volume - min_volume));
        cs.gradient.push_back(gradient);
      } else { // a degenerate domain
        cs.surface_area.push_back(0.0);
        cs.min_volume.push_back(0.0);
        cs.max_volume.push_back(0.0);
        cs.gradient.push_back(0.0);
      }
      spend(kSampleUnits);
    }
    if (fin_pos == n) {
      acc.reset();
      st = stage::done;
    }
  }

  double progress() const {
    switch (st) {
    case stage::start:
      return 0.0;
    case stage::done:
      return 1.0;
    case stage::failed:
      return failed_progress;
    default:
      break;
    }
    const auto part = [](uint64 done, uint64 total) {
      return total > 0 ? static_cast<double>(done) / static_cast<double>(total) : 0.0;
    };
    const double fd = st > stage::domain ? 1.0 : part(scan_pos, scan_count);
    const double fr = st > stage::resample    ? 1.0
                      : st == stage::resample ? part(res_pos, nvox)
                                              : 0.0;
    const double fi = st > stage::integrate    ? 1.0
                      : st == stage::integrate ? part((ck * (ny - 1) + cj) * (nx - 1) + ci, ncells)
                                               : 0.0;
    const double ff = st == stage::finish ? part(fin_pos, opt.samples) : 0.0;
    const double total = w_domain + w_resample + w_integrate + w_finish;
    const double p = (w_domain * fd + w_resample * fr + w_integrate * fi + w_finish * ff) / total;
    return std::min(1.0, std::max(0.0, p));
  }

  void fail(std::exception_ptr e) {
    failed_progress = progress();
    error = std::move(e);
    st = stage::failed;
    src.reset();
    resampled.reset();
    raw = nullptr;
    below.reset();
    above.reset();
    acc.reset();
    cs = contour_spectrum();
  }
};

contour_spectrum_builder::contour_spectrum_builder(const volume &vol,
                                                   const contour_spectrum_options &opt)
    : _impl(std::make_unique<impl>(vol, opt)) {}

contour_spectrum_builder::~contour_spectrum_builder() = default;
contour_spectrum_builder::contour_spectrum_builder(contour_spectrum_builder &&) noexcept = default;
contour_spectrum_builder &
contour_spectrum_builder::operator=(contour_spectrum_builder &&) noexcept = default;

bool contour_spectrum_builder::step(std::size_t budget) {
  if (!_impl)
    throw contour_spectrum_error("contour_spectrum_builder: a moved-from builder has no work");
  impl &s = *_impl;
  if (s.st == impl::stage::failed)
    std::rethrow_exception(s.error);
  if (s.st == impl::stage::done)
    return true;
  try {
    s.run(budget == 0 ? kUnlimited : static_cast<uint64>(budget));
    if (s.opt.progress && !s.opt.progress(s.progress()))
      throw contour_spectrum_error("compute_contour_spectrum: cancelled");
  } catch (...) {
    s.fail(std::current_exception());
    throw;
  }
  return s.st == impl::stage::done;
}

bool contour_spectrum_builder::done() const {
  return !_impl || _impl->st == impl::stage::done || _impl->st == impl::stage::failed;
}

double contour_spectrum_builder::progress() const { return _impl ? _impl->progress() : 1.0; }

const char *contour_spectrum_builder::phase() const {
  if (!_impl)
    return "done";
  switch (_impl->st) {
  case impl::stage::start:
  case impl::stage::domain:
    return "domain";
  case impl::stage::resample:
    return "resampling";
  case impl::stage::integrate:
  case impl::stage::finish:
    return "integrating";
  case impl::stage::done:
  case impl::stage::failed:
    break;
  }
  return "done";
}

uint64 contour_spectrum_builder::work_done() const { return _impl ? _impl->work : 0; }

uint64 contour_spectrum_builder::max_overrun(std::size_t samples) {
  // The largest indivisible piece of work, less the one unit of budget that must be left for a
  // step to start it: a cell updating every sample in all five tetrahedra (the accumulator
  // setup, kSampleUnits * samples, and a resampled voxel are smaller).
  const uint64 n = std::max<uint64>(samples, 2);
  return kCellUnits + 5 * kUpdateUnits * n - 1;
}

contour_spectrum contour_spectrum_builder::take() {
  if (!_impl)
    throw contour_spectrum_error("contour_spectrum_builder: a moved-from builder has no spectrum");
  impl &s = *_impl;
  if (s.st == impl::stage::failed)
    std::rethrow_exception(s.error);
  if (s.st != impl::stage::done || s.taken)
    throw contour_spectrum_error(s.taken ? "contour_spectrum_builder: the spectrum was taken"
                                         : "contour_spectrum_builder: the spectrum is not ready");
  s.taken = true;
  return std::move(s.cs);
}

contour_spectrum compute_contour_spectrum(const volume &vol, const contour_spectrum_options &opt) {
  contour_spectrum_builder builder(vol, opt);
  const std::size_t budget = opt.progress ? kOneShotBudget : 0;
  while (!builder.step(budget)) {
  }
  return builder.take();
}

// --- on a simplicial mesh ------------------------------------------------------------------------

namespace {

constexpr uint64 kTetUnits = 40; // one tetrahedron's fixed part (a grid cell is five of them)
constexpr uint64 kTriUnits = 20; // one triangle's

contour_spectrum_error mesh_error(const std::string &what) {
  return contour_spectrum_error("compute_contour_spectrum (mesh): " + what);
}

} // namespace

struct mesh_contour_spectrum_builder::impl {
  enum class stage { start, domain, integrate, finish, done, failed };

  std::shared_ptr<const contour_mesh> mesh;
  contour_spectrum_options opt;
  stage st = stage::start;
  std::exception_ptr error;
  double failed_progress = 0.0;
  uint64 work = 0, left = 0;

  bool solid = false; // tetrahedra (else triangles)
  std::size_t npts = 0, nelem = 0;
  std::size_t pos = 0; // the domain scan's value, the integration's element
  extrema_state ext;
  double dmin = 0.0, dmax = 0.0, inv = 0.0, grad_scale = 0.0;
  bool degenerate = false;
  std::optional<accumulator> acc;
  double total = 0.0; // the finite elements' volume (or area)
  std::size_t fin_pos = 0;
  double tail = 0.0;
  contour_spectrum cs;
  bool taken = false;

  impl(std::shared_ptr<const contour_mesh> m, const contour_spectrum_options &o)
      : mesh(std::move(m)), opt(o) {}

  void spend(uint64 units) {
    work += units;
    left = units >= left ? 0 : left - units;
  }

  const geometry &geom() const { return *mesh->mesh; }

  void begin() {
    if (!mesh || !mesh->mesh)
      throw mesh_error("no mesh");
    if (opt.samples < 2)
      throw mesh_error("at least 2 samples are needed");
    npts = geom().const_points().size();
    if (mesh->values.size() != npts)
      throw mesh_error("the mesh has " + std::to_string(npts) + " points but " +
                       std::to_string(mesh->values.size()) + " values");
    solid = !geom().const_tets().empty();
    nelem = solid ? geom().const_tets().size() : geom().const_tris().size();
    if (nelem == 0)
      throw mesh_error("the mesh has neither tetrahedra nor triangles");
    if (opt.has_domain) {
      dmin = std::min(opt.domain_min, opt.domain_max);
      dmax = std::max(opt.domain_min, opt.domain_max);
      plan();
    } else {
      st = stage::domain;
    }
  }

  void scan() {
    const std::size_t chunk = static_cast<std::size_t>(std::min<uint64>(left, npts - pos));
    const double *v = mesh->values.data();
    for (std::size_t i = pos; i < pos + chunk; ++i) {
      if (!std::isfinite(v[i]))
        continue;
      if (!ext.any) {
        ext.lo = ext.hi = v[i];
        ext.any = true;
      } else if (v[i] < ext.lo) {
        ext.lo = v[i];
      } else if (v[i] > ext.hi) {
        ext.hi = v[i];
      }
    }
    pos += chunk;
    spend(chunk * kScanUnits);
    if (pos == npts) {
      if (!ext.any)
        throw mesh_error("no value is finite");
      dmin = ext.lo;
      dmax = ext.hi;
      plan();
    }
  }

  // As the grid's: the values go onto [0, kScale] over the domain, which sort_verts' separation
  // assumes; the gradient comes back to data units by grad_scale.
  void plan() {
    if (!std::isfinite(dmin) || !std::isfinite(dmax))
      throw mesh_error("the isovalue domain is not finite");
    const double range = dmax - dmin;
    grad_scale = (range / kScale) * (range / kScale);
    if (!std::isfinite(range) || !std::isfinite(grad_scale))
      throw mesh_error("the isovalue domain is too wide to scale (wider than 255 * "
                       "sqrt(DBL_MAX), ~3.4e156)");
    inv = kScale / range;
    degenerate = !(range > 0.0) || !std::isfinite(inv);
    if (degenerate)
      inv = 0.0;
    acc.emplace(opt.samples);
    acc->mid_weight = opt.libcontour_area ? 1.0 : 2.0;
    spend(kSampleUnits * opt.samples);
    pos = 0;
    st = stage::integrate;
  }

  // One element's corners and values (mapped onto the integration scale), or false when one of
  // its values is not finite.
  template <std::size_t N, class Element> bool corners(const Element &e, vec3 *x, double *f) const {
    const geometry::points_t &P = geom().const_points();
    for (std::size_t c = 0; c < N; ++c) {
      const auto i = e[c];
      if (i >= npts)
        throw mesh_error("an element names point " + std::to_string(i) + " of " +
                         std::to_string(npts));
      const double v = mesh->values[static_cast<std::size_t>(i)];
      if (!std::isfinite(v))
        return false;
      x[c] = {P[i][0], P[i][1], P[i][2]};
      f[c] = (v - dmin) * inv;
    }
    return true;
  }

  void integrate() {
    accumulator &a = *acc;
    while (left > 0 && pos < nelem) {
      const std::size_t e = pos++;
      if (solid) {
        vec3 x[4];
        double f[4];
        if (!corners<4>(geom().const_tets()[e], x, f)) {
          spend(kSkippedCellUnits);
          continue;
        }
        const vec3 e1 = x[1] - x[0], e2 = x[2] - x[0], e3 = x[3] - x[0];
        const vec3 c23 = cross(e2, e3), c31 = cross(e3, e1), c12 = cross(e1, e2);
        const double det = dot(e1, c23);
        const double volume = std::fabs(det) / 6.0;
        total += volume;
        if (degenerate || !(volume > 0.0)) {
          spend(kTetUnits);
          continue;
        }
        // f's exact gradient: the one vector g with g . e_i = f_i - f_0.
        const double d1 = f[1] - f[0], d2 = f[2] - f[0], d3 = f[3] - f[0];
        const vec3 g = (1.0 / det) * vec3{d1 * c23.x + d2 * c31.x + d3 * c12.x,
                                          d1 * c23.y + d2 * c31.y + d3 * c12.y,
                                          d1 * c23.z + d2 * c31.z + d3 * c12.z};
        static constexpr int kWhole[4] = {0, 1, 2, 3};
        const std::size_t updates = integrate_tet(a, x, f, kWhole, dot(g, g), volume);
        spend(kTetUnits + kUpdateUnits * static_cast<uint64>(updates));
      } else {
        vec3 x[3];
        double f[3];
        if (!corners<3>(geom().const_tris()[e], x, f)) {
          spend(kSkippedCellUnits);
          continue;
        }
        const vec3 e1 = x[1] - x[0], e2 = x[2] - x[0];
        const vec3 nrm = cross(e1, e2);
        const double nn = dot(nrm, nrm);
        const double area = 0.5 * std::sqrt(nn);
        total += area;
        if (degenerate || !(nn > 0.0)) {
          spend(kTriUnits);
          continue;
        }
        // The in-plane gradient: g . e1 = f1 - f0, g . e2 = f2 - f0, g . n = 0.
        const vec3 u = cross(e2, nrm), w = cross(nrm, e1);
        const double d1 = f[1] - f[0], d2 = f[2] - f[0];
        const vec3 g =
            (1.0 / nn) * vec3{d1 * u.x + d2 * w.x, d1 * u.y + d2 * w.y, d1 * u.z + d2 * w.z};
        const std::size_t updates = integrate_tri(a, x, f, dot(g, g), area);
        spend(kTriUnits + kUpdateUnits * static_cast<uint64>(updates));
      }
    }
    if (pos == nelem)
      st = stage::finish;
  }

  void finish() {
    const std::size_t n = opt.samples;
    if (fin_pos == 0 && cs.isovalue.empty()) {
      if (!std::isfinite(total))
        throw overflow_error();
      cs.domain_min = dmin;
      cs.domain_max = dmax;
      cs.total_volume = total;
      for (std::vector<double> *f :
           {&cs.isovalue, &cs.surface_area, &cs.min_volume, &cs.max_volume, &cs.gradient})
        f->reserve(n);
    }
    while (left > 0 && fin_pos < n) {
      const std::size_t b = fin_pos++;
      cs.isovalue.push_back(b + 1 == n
                                ? dmax
                                : dmin + (static_cast<double>(b) / static_cast<double>(n - 1)) *
                                             (dmax - dmin));
      if (degenerate) {
        cs.surface_area.push_back(0.0);
        cs.min_volume.push_back(0.0);
        cs.max_volume.push_back(0.0);
        cs.gradient.push_back(0.0);
      } else {
        tail += acc->vol_tail[b];
        const double area = acc->area[b], min_volume = acc->vol[b] + tail;
        const double gradient = acc->grad[b] * grad_scale;
        if (!std::isfinite(area) || !std::isfinite(min_volume) || !std::isfinite(gradient))
          throw overflow_error();
        cs.surface_area.push_back(area);
        cs.min_volume.push_back(min_volume);
        cs.max_volume.push_back(std::max(0.0, total - min_volume));
        cs.gradient.push_back(gradient);
      }
      spend(kSampleUnits);
    }
    if (fin_pos == n) {
      acc.reset();
      mesh.reset();
      st = stage::done;
    }
  }

  void run(uint64 budget) {
    left = budget;
    if (st == stage::start)
      begin();
    while (left > 0 && st != stage::done) {
      switch (st) {
      case stage::domain:
        scan();
        break;
      case stage::integrate:
        integrate();
        break;
      case stage::finish:
        finish();
        break;
      default:
        return;
      }
    }
  }

  double progress() const {
    switch (st) {
    case stage::start:
      return 0.0;
    case stage::done:
      return 1.0;
    case stage::failed:
      return failed_progress;
    default:
      break;
    }
    // Weighted by each phase's fixed units (the sample updates are not known in advance).
    const double wd = opt.has_domain ? 0.0 : static_cast<double>(npts);
    const double wi =
        static_cast<double>(nelem) * static_cast<double>(solid ? kTetUnits : kTriUnits);
    const double wf = static_cast<double>(opt.samples);
    const double fd = st == stage::domain && npts > 0 ? double(pos) / double(npts) : 1.0;
    const double fi = st == stage::domain      ? 0.0
                      : st == stage::integrate ? (nelem > 0 ? double(pos) / double(nelem) : 1.0)
                                               : 1.0;
    const double ff = st == stage::finish ? double(fin_pos) / wf : 0.0;
    const double p = (wd * fd + wi * fi + wf * ff) / (wd + wi + wf);
    return std::min(1.0, std::max(0.0, p));
  }

  void fail(std::exception_ptr e) {
    failed_progress = progress();
    error = std::move(e);
    st = stage::failed;
    mesh.reset();
    acc.reset();
    cs = contour_spectrum();
  }
};

mesh_contour_spectrum_builder::mesh_contour_spectrum_builder(
    std::shared_ptr<const contour_mesh> mesh, const contour_spectrum_options &opt)
    : _impl(std::make_unique<impl>(std::move(mesh), opt)) {}

mesh_contour_spectrum_builder::~mesh_contour_spectrum_builder() = default;
mesh_contour_spectrum_builder::mesh_contour_spectrum_builder(
    mesh_contour_spectrum_builder &&) noexcept = default;
mesh_contour_spectrum_builder &
mesh_contour_spectrum_builder::operator=(mesh_contour_spectrum_builder &&) noexcept = default;

bool mesh_contour_spectrum_builder::step(std::size_t budget) {
  if (!_impl)
    throw contour_spectrum_error("mesh_contour_spectrum_builder: a moved-from builder has no work");
  impl &s = *_impl;
  if (s.st == impl::stage::failed)
    std::rethrow_exception(s.error);
  if (s.st == impl::stage::done)
    return true;
  try {
    s.run(budget == 0 ? kUnlimited : static_cast<uint64>(budget));
    if (s.opt.progress && !s.opt.progress(s.progress()))
      throw mesh_error("cancelled");
  } catch (...) {
    s.fail(std::current_exception());
    throw;
  }
  return s.st == impl::stage::done;
}

bool mesh_contour_spectrum_builder::done() const {
  return !_impl || _impl->st == impl::stage::done || _impl->st == impl::stage::failed;
}

double mesh_contour_spectrum_builder::progress() const { return _impl ? _impl->progress() : 1.0; }

const char *mesh_contour_spectrum_builder::phase() const {
  if (!_impl)
    return "done";
  switch (_impl->st) {
  case impl::stage::start:
  case impl::stage::domain:
    return _impl->opt.has_domain ? "integrating" : "domain";
  case impl::stage::integrate:
  case impl::stage::finish:
    return "integrating";
  case impl::stage::done:
  case impl::stage::failed:
    break;
  }
  return "done";
}

uint64 mesh_contour_spectrum_builder::work_done() const { return _impl ? _impl->work : 0; }

contour_spectrum mesh_contour_spectrum_builder::take() {
  if (!_impl)
    throw contour_spectrum_error(
        "mesh_contour_spectrum_builder: a moved-from builder has no spectrum");
  impl &s = *_impl;
  if (s.st == impl::stage::failed)
    std::rethrow_exception(s.error);
  if (s.st != impl::stage::done || s.taken)
    throw contour_spectrum_error(s.taken
                                     ? "mesh_contour_spectrum_builder: the spectrum was taken"
                                     : "mesh_contour_spectrum_builder: the spectrum is not ready");
  s.taken = true;
  return std::move(s.cs);
}

contour_spectrum compute_contour_spectrum(const contour_mesh &mesh,
                                          const contour_spectrum_options &opt) {
  // Borrowed for the call: an aliasing pointer that owns nothing.
  mesh_contour_spectrum_builder builder(
      std::shared_ptr<const contour_mesh>(std::shared_ptr<const contour_mesh>(), &mesh), opt);
  const std::size_t budget = opt.progress ? kOneShotBudget : 0;
  while (!builder.step(budget)) {
  }
  return builder.take();
}

#ifndef CVC_ENABLE_MESHER
// The libContour oracle lives in src/cvc/geometry/cvc-mesher/contour_spectrum_libcontour.cpp,
// which only a CVC_ENABLE_MESHER build compiles.
bool contour_spectrum_libcontour_available() { return false; }

contour_spectrum compute_contour_spectrum_libcontour(const volume &, bool) {
  throw contour_spectrum_error(
      "compute_contour_spectrum_libcontour: libcvc was built without CVC_ENABLE_MESHER");
}
#endif

} // namespace cvc
