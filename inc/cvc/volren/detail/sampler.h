// Raw-buffer volume sampler for the ray-march inner loop.
//
// cvc::voxels::operator()(i,j,k) is bounds-checked and switch-dispatched per
// call; the marcher instead takes the buffer pointer once (with the volume's
// storage pinned via voxels::active_storage() for the render's duration --
// see raycaster.cpp) and samples through this lightweight view.  This is the
// modern replacement for the legacy Volume/DataPtr union + xinc/yinc tables,
// and it handles every cvc::data_type (the legacy FLOAT branch was
// assert(0); RawV multi-variable volumes are out of scope for the port).
// Derived from volrover's volren/libiso (C) 2000-2005 University of Texas at
// Austin (Park/Zhang/Rivera, advisor Bajaj), LGPL 2.1 -- the same license as
// libcvc.
#ifndef CVC_VOLREN_DETAIL_SAMPLER_H
#define CVC_VOLREN_DETAIL_SAMPLER_H

#include <cstddef>
#include <cstdint>
#include <cvc/core/types.h>
#include <cvc/volren/types.h>

namespace cvc {
namespace volren {
namespace detail {

// Non-owning typed view over a node-centered voxel grid.
// Voxel (i,j,k) sits at world position min + (i,j,k) * span; there are
// (dim-1) cells per axis and cell indices are valid in [0, dim-2].
struct grid_sampler {
  const unsigned char *data = nullptr;
  cvc::data_type type = cvc::UChar;
  std::int64_t dimx = 0, dimy = 0, dimz = 0;
  vec3d minb; // world position of voxel (0,0,0)
  vec3d span; // voxel spacing per axis (all > 0)

  float at(std::int64_t i, std::int64_t j, std::int64_t k) const {
    const std::size_t n = std::size_t(i) + std::size_t(j) * std::size_t(dimx) +
                          std::size_t(k) * std::size_t(dimx) * std::size_t(dimy);
    switch (type) {
    case cvc::UChar:
      return float(reinterpret_cast<const unsigned char *>(data)[n]);
    case cvc::UShort:
      return float(reinterpret_cast<const std::uint16_t *>(data)[n]);
    case cvc::UInt:
      return float(reinterpret_cast<const std::uint32_t *>(data)[n]);
    case cvc::Float:
      return reinterpret_cast<const float *>(data)[n];
    case cvc::Double:
      return float(reinterpret_cast<const double *>(data)[n]);
    case cvc::UInt64:
      return float(reinterpret_cast<const std::uint64_t *>(data)[n]);
    case cvc::Char:
      return float(reinterpret_cast<const signed char *>(data)[n]);
    case cvc::Int:
      return float(reinterpret_cast<const std::int32_t *>(data)[n]);
    case cvc::Int64:
      return float(reinterpret_cast<const std::int64_t *>(data)[n]);
    default:
      return 0.f;
    }
  }

  // Edge-clamped fetch (the legacy spline-neighborhood clamp).
  float at_clamped(std::int64_t i, std::int64_t j, std::int64_t k) const {
    i = i < 0 ? 0 : (i > dimx - 1 ? dimx - 1 : i);
    j = j < 0 ? 0 : (j > dimy - 1 ? dimy - 1 : j);
    k = k < 0 ? 0 : (k > dimz - 1 ? dimz - 1 : k);
    return at(i, j, k);
  }

  // The 8 corner values of cell (ci,cj,ck) in BINARY order
  // (bit0 = x, bit1 = y, bit2 = z), matching vrGetVertDensities.
  void corners(std::int64_t ci, std::int64_t cj, std::int64_t ck, float out[8]) const {
    out[0] = at(ci, cj, ck);
    out[1] = at(ci + 1, cj, ck);
    out[2] = at(ci, cj + 1, ck);
    out[3] = at(ci + 1, cj + 1, ck);
    out[4] = at(ci, cj, ck + 1);
    out[5] = at(ci + 1, cj, ck + 1);
    out[6] = at(ci, cj + 1, ck + 1);
    out[7] = at(ci + 1, cj + 1, ck + 1);
  }

  // Local [0,1]^3 coordinates of world point p inside cell (ci,cj,ck).
  void local_weights(const vec3d &p, std::int64_t ci, std::int64_t cj, std::int64_t ck,
                     float w[3]) const {
    w[0] = float((p.x - (minb.x + double(ci) * span.x)) / span.x);
    w[1] = float((p.y - (minb.y + double(cj) * span.y)) / span.y);
    w[2] = float((p.z - (minb.z + double(ck) * span.z)) / span.z);
  }

  // Cell index containing world point p; returns false when p falls outside
  // the valid cell range (the legacy per-env clip that lets several volumes
  // coexist along one ray).
  bool cell_index(const vec3d &p, std::int64_t idx[3]) const {
    const double qx = (p.x - minb.x) / span.x;
    const double qy = (p.y - minb.y) / span.y;
    const double qz = (p.z - minb.z) / span.z;
    // Range-check in the double domain BEFORE casting: a NaN or huge
    // quotient (NaN voxel coordinates, wild model transforms) fails these
    // comparisons instead of hitting an undefined float->int conversion.
    if (!(qx > -1.0 && qx < double(dimx)) || !(qy > -1.0 && qy < double(dimy)) ||
        !(qz > -1.0 && qz < double(dimz)))
      return false;
    // Truncation toward zero matches the legacy (int) cast: a point within
    // one span left of minb truncates into cell 0, which keeps the entry
    // sample robust against FP rounding on the box face; anything further
    // out was rejected above.
    idx[0] = std::int64_t(qx);
    idx[1] = std::int64_t(qy);
    idx[2] = std::int64_t(qz);
    return idx[0] <= dimx - 2 && idx[1] <= dimy - 2 && idx[2] <= dimz - 2;
  }
};

// Trilinear interpolation over corner values in BINARY order with local
// weights w[3] -- arithmetic-identical to the legacy vrTriInterp (which lerps
// z, then y, then x).
inline float trilinear(const float w[3], const float v[8]) {
  const float c00 = (1.f - w[2]) * v[0] + w[2] * v[4];
  const float c10 = (1.f - w[2]) * v[2] + w[2] * v[6];
  const float c0 = (1.f - w[1]) * c00 + w[1] * c10;

  const float c01 = (1.f - w[2]) * v[1] + w[2] * v[5];
  const float c11 = (1.f - w[2]) * v[3] + w[2] * v[7];
  const float c1 = (1.f - w[1]) * c01 + w[1] * c11;

  return (1.f - w[0]) * c0 + w[0] * c1;
}

} // namespace detail
} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_DETAIL_SAMPLER_H
