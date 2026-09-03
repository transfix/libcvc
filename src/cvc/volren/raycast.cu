// raycast.cu -- the CUDA (GPU) volume raycaster, a device transcription of
// raycaster.cpp's render_ray.  Compiled only when CVC_ENABLE_CUDA and (per
// src/cvc/CMakeLists.txt) WITH --use_fast_math in Release: volume rendering
// has no bit/float-equivalence contract, so this is a SEMANTIC mirror of the
// CPU march, not a float-equal one.  Parity is an image-level property.
//
// One thread per pixel, dim3(16,16) blocks.  Everything the ray needs lives in
// registers/local memory: the spline-gradient neighborhood cache (the state
// that used to force vrCopyEnv), the per-ray last-cell tracker, and a bounded
// isosurface hit buffer.  No shared memory, no textures (manual trilinear like
// every other kernel in this tree), default stream only.
//
// Derived from volrover's volren/libiso (C) 2000-2005 University of Texas at
// Austin (Park/Zhang/Rivera, advisor Bajaj), LGPL 2.1 -- the same license as
// libcvc.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cuda_runtime.h>
#include <cvc/core/types.h>
#include <cvc/utility/cuda_utils.h>
#include <cvc/volren/detail/mc_tables.h>
#include <cvc/volren/raycaster_cuda.h>
#include <mutex>
#include <set>

namespace cvc {
namespace volren {

namespace {

// ---------------------------------------------------------------------------
// Marching-cubes tables in constant memory
// ---------------------------------------------------------------------------
// detail/mc_tables.h compiles cleanly under nvcc (it is header-only constexpr
// data with no includes), but `inline constexpr` arrays live in HOST storage:
// device code cannot take their address.  So the values are copied verbatim
// into __constant__ mirrors once per process -- no duplicated table literals,
// and the header stays untouched.

// Layout-compatible mirror of detail::mc_edge_info (6 ints, same order).
struct dev_mc_edge {
  int dir;
  int di, dj, dk;
  int v1, v2;
};
static_assert(sizeof(dev_mc_edge) == sizeof(detail::mc_edge_info),
              "dev_mc_edge must mirror detail::mc_edge_info for the bulk copy");

__constant__ unsigned char c_cube_edges[256][13];
__constant__ signed char c_tri_cases[256][16];
__constant__ dev_mc_edge c_mc_edges[12];
__constant__ int c_vertex_from_binary[8];

void ensure_mc_tables() {
  // __constant__ storage is PER DEVICE: each device loads its own module copy,
  // zero-filled until something writes it, and cudaMemcpyToSymbol only writes
  // the calling thread's CURRENT device.  Guarding this with a process-wide
  // once_flag would upload to whichever device happened to be current first
  // and leave every other device's tables zeroed -- cube_edges[code][0] == 0
  // reads as "no isosurface in this cell" for all 256 codes, so isosurfaces
  // would silently vanish on the second GPU instead of failing loudly.
  // Track it per device instead.
  int device = 0;
  CUDA_CHECK(cudaGetDevice(&device));

  static std::mutex mutex;
  static std::set<int> uploaded;
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (uploaded.count(device))
      return;
  }

  CUDA_CHECK(cudaMemcpyToSymbol(c_cube_edges, detail::cube_edges, sizeof(detail::cube_edges)));
  CUDA_CHECK(cudaMemcpyToSymbol(c_tri_cases, detail::tri_cases, sizeof(detail::tri_cases)));
  CUDA_CHECK(cudaMemcpyToSymbol(c_mc_edges, detail::mc_edges, sizeof(detail::mc_edges)));
  CUDA_CHECK(cudaMemcpyToSymbol(c_vertex_from_binary, detail::mc_vertex_from_binary,
                                sizeof(detail::mc_vertex_from_binary)));

  // Recorded only after every upload succeeded, so a transient failure is
  // retried on the next render rather than leaving the tables half-written.
  std::lock_guard<std::mutex> lock(mutex);
  uploaded.insert(device);
}

// ---------------------------------------------------------------------------
// Device vector math (the volren::vec3d subset the march uses)
// ---------------------------------------------------------------------------
// World traversal stays in double like the CPU path.  --use_fast_math only
// relaxes SINGLE-precision div/sqrt, so the only double-precision divergence
// is FMA contraction.

struct dvec {
  double x, y, z;
};

__device__ inline dvec dv(double x, double y, double z) {
  dvec r;
  r.x = x;
  r.y = y;
  r.z = z;
  return r;
}
__device__ inline dvec operator+(const dvec &a, const dvec &b) {
  return dv(a.x + b.x, a.y + b.y, a.z + b.z);
}
__device__ inline dvec operator-(const dvec &a, const dvec &b) {
  return dv(a.x - b.x, a.y - b.y, a.z - b.z);
}
__device__ inline dvec operator*(const dvec &a, double s) { return dv(a.x * s, a.y * s, a.z * s); }
__device__ inline dvec operator-(const dvec &a) { return dv(-a.x, -a.y, -a.z); }
__device__ inline double dot(const dvec &a, const dvec &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
__device__ inline dvec cross(const dvec &a, const dvec &b) {
  return dv(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
// volren::normalized: shorter than epsilon normalizes to ZERO (the legacy
// vrNormalize contract shading relies on for flat regions).
__device__ inline dvec dnormalized(const dvec &v) {
  const double len = sqrt(dot(v, v));
  return len <= 1e-12 ? dv(0.0, 0.0, 0.0) : dv(v.x / len, v.y / len, v.z / len);
}
__device__ inline double component(const dvec &v, int i) {
  return i == 0 ? v.x : (i == 1 ? v.y : v.z);
}

// ---------------------------------------------------------------------------
// Device-side request
// ---------------------------------------------------------------------------
// The camera is pre-expanded exactly like raycaster.cpp's ray_generator, and
// the settings vectors are flat fixed arrays, so the whole thing rides in the
// kernel parameter block (~1.5 KB, well inside the 4 KB limit) -- the
// dev_field/dev_veh convention from nav/drive.cu, no __constant__ scene state.
struct dev_request {
  // Camera (ray_generator).
  dvec eye, right, true_up, forward;
  double tan_half, parallel_scale, aspect;
  int perspective;
  int width, height;

  // Volume (detail::grid_sampler), device pointer.
  const unsigned char *data;
  int type; // cvc::data_type
  long long dimx, dimy, dimz;
  dvec minb, span;

  // Scene-graph placement: world -> local affine inverse, row-major.
  int transformed;
  double w2l[16];

  // Baked transfer function (device pointer to lut_size RGBA quads).
  int tf_active;
  const float *lut;
  int lut_size;
  double tf_lo, tf_inv_width;

  // Gradient-magnitude opacity ramp.
  int ramp_enabled;
  double ramp0, ramp1, ramp2, ramp_plateau;

  int iso_count;
  cuda_isosurface iso[cuda_limits::max_isosurfaces];
  int light_count;
  cuda_light lights[cuda_limits::max_lights];
  int plane_count;
  cuda_cut_plane planes[cuda_limits::max_cut_planes];

  dvec scene_min, scene_max;
  int steps;
  double unit_step;
  float opacity_cutoff, depth_alpha_threshold, ambient;
  int two_sided;
  float background[3];
  float tf_shininess;

  int shaded, unshaded, window_enabled;
  double window_min, window_max;
};

// ---------------------------------------------------------------------------
// Sampling (detail::grid_sampler)
// ---------------------------------------------------------------------------

__device__ inline float grid_at(const dev_request &q, long long i, long long j, long long k) {
  const std::size_t n = std::size_t(i) + std::size_t(j) * std::size_t(q.dimx) +
                        std::size_t(k) * std::size_t(q.dimx) * std::size_t(q.dimy);
  switch (q.type) {
  case cvc::UChar:
    return float(reinterpret_cast<const unsigned char *>(q.data)[n]);
  case cvc::UShort:
    return float(reinterpret_cast<const std::uint16_t *>(q.data)[n]);
  case cvc::UInt:
    return float(reinterpret_cast<const std::uint32_t *>(q.data)[n]);
  case cvc::Float:
    return reinterpret_cast<const float *>(q.data)[n];
  case cvc::Double:
    return float(reinterpret_cast<const double *>(q.data)[n]);
  case cvc::UInt64:
    return float(reinterpret_cast<const unsigned long long *>(q.data)[n]);
  case cvc::Char:
    return float(reinterpret_cast<const signed char *>(q.data)[n]);
  case cvc::Int:
    return float(reinterpret_cast<const std::int32_t *>(q.data)[n]);
  case cvc::Int64:
    return float(reinterpret_cast<const long long *>(q.data)[n]);
  default:
    return 0.f;
  }
}

__device__ inline float grid_at_clamped(const dev_request &q, long long i, long long j,
                                        long long k) {
  i = i < 0 ? 0 : (i > q.dimx - 1 ? q.dimx - 1 : i);
  j = j < 0 ? 0 : (j > q.dimy - 1 ? q.dimy - 1 : j);
  k = k < 0 ? 0 : (k > q.dimz - 1 ? q.dimz - 1 : k);
  return grid_at(q, i, j, k);
}

// The 8 corner values of a cell in BINARY order (bit0 = x, bit1 = y, bit2 = z).
__device__ inline void grid_corners(const dev_request &q, long long ci, long long cj, long long ck,
                                    float out[8]) {
  out[0] = grid_at(q, ci, cj, ck);
  out[1] = grid_at(q, ci + 1, cj, ck);
  out[2] = grid_at(q, ci, cj + 1, ck);
  out[3] = grid_at(q, ci + 1, cj + 1, ck);
  out[4] = grid_at(q, ci, cj, ck + 1);
  out[5] = grid_at(q, ci + 1, cj, ck + 1);
  out[6] = grid_at(q, ci, cj + 1, ck + 1);
  out[7] = grid_at(q, ci + 1, cj + 1, ck + 1);
}

__device__ inline void grid_local_weights(const dev_request &q, const dvec &p, long long ci,
                                          long long cj, long long ck, float w[3]) {
  w[0] = float((p.x - (q.minb.x + double(ci) * q.span.x)) / q.span.x);
  w[1] = float((p.y - (q.minb.y + double(cj) * q.span.y)) / q.span.y);
  w[2] = float((p.z - (q.minb.z + double(ck) * q.span.z)) / q.span.z);
}

// Range-checked in the double domain BEFORE the cast, so NaN / huge quotients
// fail the comparisons instead of hitting an undefined float->int conversion.
__device__ inline bool grid_cell_index(const dev_request &q, const dvec &p, long long idx[3]) {
  const double qx = (p.x - q.minb.x) / q.span.x;
  const double qy = (p.y - q.minb.y) / q.span.y;
  const double qz = (p.z - q.minb.z) / q.span.z;
  if (!(qx > -1.0 && qx < double(q.dimx)) || !(qy > -1.0 && qy < double(q.dimy)) ||
      !(qz > -1.0 && qz < double(q.dimz)))
    return false;
  idx[0] = (long long)qx; // truncation toward zero, matching the legacy (int) cast
  idx[1] = (long long)qy;
  idx[2] = (long long)qz;
  return idx[0] <= q.dimx - 2 && idx[1] <= q.dimy - 2 && idx[2] <= q.dimz - 2;
}

// detail::trilinear -- lerps z, then y, then x (arithmetic order preserved).
__device__ inline float trilinear(const float w[3], const float v[8]) {
  const float c00 = (1.f - w[2]) * v[0] + w[2] * v[4];
  const float c10 = (1.f - w[2]) * v[2] + w[2] * v[6];
  const float c0 = (1.f - w[1]) * c00 + w[1] * c10;

  const float c01 = (1.f - w[2]) * v[1] + w[2] * v[5];
  const float c11 = (1.f - w[2]) * v[3] + w[2] * v[7];
  const float c1 = (1.f - w[1]) * c01 + w[1] * c11;

  return (1.f - w[0]) * c0 + w[0] * c1;
}

// ---------------------------------------------------------------------------
// Quadratic-B-spline (de Boor) gradient -- detail::spline_gradient_cache
// ---------------------------------------------------------------------------
// Kept in per-thread local memory with the CPU cache's exact array SHAPES
// (including the padding the index dance never touches) so the two can be
// diffed line for line.  The 4^3 neighborhood and difference tensors refresh
// only when the cell index changes; the previous cell stays in registers.
struct spline_cache {
  long long cell[3];
  float val[4][4][4];
  float deriv[4][4][3][3];

  __device__ void reset() {
    cell[0] = -1;
    cell[1] = -1;
    cell[2] = -1;
  }

  __device__ void refresh(const dev_request &q) {
    for (int k = -1; k <= 2; ++k)
      for (int j = -1; j <= 2; ++j)
        for (int i = -1; i <= 2; ++i)
          val[k + 1][j + 1][i + 1] = grid_at_clamped(q, cell[0] + i, cell[1] + j, cell[2] + k);

    for (int g = 0; g < 3; ++g)
      for (int a = 1; a < 3; ++a)
        for (int b = 1; b < 3; ++b)
          deriv[a][b][g][0] = val[a][b][g + 1] - val[a][b][g];

    for (int g = 1; g < 3; ++g)
      for (int j = 0; j < 3; ++j)
        for (int a = 1; a < 3; ++a)
          deriv[g][a][j][1] = val[a][j + 1][g] - val[a][j][g];

    for (int g = 1; g < 3; ++g)
      for (int b = 1; b < 3; ++b)
        for (int i = 0; i < 3; ++i)
          deriv[b][g][i][2] = val[i + 1][b][g] - val[i][b][g];
  }

  __device__ dvec evaluate(const dev_request &q, const long long idx[3], const float w[3]) {
    if (idx[0] != cell[0] || idx[1] != cell[1] || idx[2] != cell[2]) {
      cell[0] = idx[0];
      cell[1] = idx[1];
      cell[2] = idx[2];
      refresh(q);
    }

    float d[4][4][2];
    float normal[3];
    for (int l = 0; l < 3; ++l) {
      for (int a = 1; a < 3; ++a) {
        for (int b = 1; b < 3; ++b) {
          float delta = 0.5f * (1.0f - w[l]);
          d[a][b][0] = delta * deriv[a][b][0][l] + (1.0f - delta) * deriv[a][b][1][l];
          delta = 0.5f * (2.0f - w[l]);
          d[a][b][1] = delta * deriv[a][b][1][l] + (1.0f - delta) * deriv[a][b][2][l];

          delta = 1.0f - w[l];
          d[a][b][0] = delta * d[a][b][0] + (1.0f - delta) * d[a][b][1];
        }
      }
      int m = (l + 1) % 3;
      for (int a = 1; a < 3; ++a) {
        const float delta = 1.0f - w[m];
        d[a][0][0] = delta * d[a][1][0] + (1.0f - delta) * d[a][2][0];
      }
      m = (l + 2) % 3;
      const float delta = 1.0f - w[m];
      normal[l] = delta * d[1][0][0] + (1.0f - delta) * d[2][0][0];
    }
    return dv(normal[0], normal[1], normal[2]);
  }
};

// ---------------------------------------------------------------------------
// Per-cell ray/isosurface intersection -- detail::cell_intersect
// ---------------------------------------------------------------------------

struct mc_tri {
  dvec vert[3];
};

__device__ inline int extract_contour(float isovalue, const long long id[3], const dvec &orig,
                                      const dvec &span, const float func[8], mc_tri tris[5]) {
  int code = 0;
  for (int v = 0; v < 8; ++v)
    if (func[v] < isovalue)
      code |= 1 << v;

  const int nedges = c_cube_edges[code][0];
  if (nedges == 0)
    return 0;

  dvec edge_v[12];
  for (int e = 0; e < nedges; ++e) {
    const int edge = c_cube_edges[code][1 + e];
    const dev_mc_edge ei = c_mc_edges[edge];
    const double i = double(id[0] + ei.di);
    const double j = double(id[1] + ei.dj);
    const double k = double(id[2] + ei.dk);
    const double x =
        (double(isovalue) - double(func[ei.v1])) / (double(func[ei.v2]) - double(func[ei.v1]));
    dvec p = dv(orig.x + span.x * i, orig.y + span.y * j, orig.z + span.z * k);
    switch (ei.dir) {
    case 0:
      p.x = orig.x + span.x * (i + x);
      break;
    case 1:
      p.y = orig.y + span.y * (j + x);
      break;
    default:
      p.z = orig.z + span.z * (k + x);
      break;
    }
    edge_v[edge] = p;
  }

  int ntris = 0;
  for (int t = 0; c_tri_cases[code][t] != -1; t += 3, ++ntris)
    for (int v = 0; v < 3; ++v)
      tris[ntris].vert[v] = edge_v[c_tri_cases[code][t + v]];
  return ntris;
}

// 2D dominant-axis projection point-in-triangle test (libiso's in_triangle).
__device__ inline bool in_triangle(const dvec &point, const mc_tri &tri, const dvec &normal) {
  int i1, i2;
  const double ax = fabs(normal.x), ay = fabs(normal.y), az = fabs(normal.z);
  if (ax >= ay && ax >= az) {
    i1 = 1;
    i2 = 2;
  } else if (ay >= ax && ay >= az) {
    i1 = 0;
    i2 = 2;
  } else {
    i1 = 0;
    i2 = 1;
  }
  const double u0 = component(point, i1) - component(tri.vert[0], i1);
  const double v0 = component(point, i2) - component(tri.vert[0], i2);
  const double u1 = component(tri.vert[1], i1) - component(tri.vert[0], i1);
  const double v1 = component(tri.vert[1], i2) - component(tri.vert[0], i2);
  const double u2 = component(tri.vert[2], i1) - component(tri.vert[0], i1);
  const double v2 = component(tri.vert[2], i2) - component(tri.vert[0], i2);

  if (u1 == 0.0) {
    const double denom = u2;
    if (denom == 0.0 || v1 == 0.0)
      return false; // degenerate triangle
    const double beta = u0 / denom;
    if (beta < 0.0 || beta > 1.0)
      return false;
    const double alpha = (v0 - beta * v2) / v1;
    return alpha >= 0.0 && alpha + beta <= 1.0;
  }
  const double denom = v2 * u1 - u2 * v1;
  if (denom == 0.0)
    return false;
  const double beta = (v0 * u1 - u0 * v1) / denom;
  if (beta < 0.0 || beta > 1.0)
    return false;
  const double alpha = (u0 - beta * u2) / u1;
  return alpha >= 0.0 && alpha + beta <= 1.0;
}

// Returns t >= 0 on a hit, negative on a miss (libiso's contract).
__device__ inline double intersect_triangle(const dvec &org, const dvec &dir, const mc_tri &tri,
                                            dvec &point) {
  const dvec e1 = tri.vert[1] - tri.vert[0];
  const dvec e2 = tri.vert[2] - tri.vert[0];
  const dvec n = dnormalized(cross(e1, e2));
  const double fz = dot(n, tri.vert[0] - org);
  const double fm = dot(n, dir);
  if (fm == 0.0)
    return -1.0;
  const double t = fz / fm;
  if (t < 0.0)
    return t;
  point = org + dir * t;
  return in_triangle(point, tri, n) ? t : -1.0;
}

// iso_intersectW: nearest MC-triangle hit in one cell.  Writes the hit's
// (unclamped) local cell weights and its ray parameter.
__device__ inline bool intersect_isosurface_in_cell(const dvec &org, const dvec &dir,
                                                    float isovalue, const long long id[3],
                                                    const dvec &orig, const dvec &span,
                                                    const float func[8], float w[3],
                                                    double &t_hit) {
  mc_tri tris[5];
  const int nt = extract_contour(isovalue, id, orig, span, func, tris);
  if (nt == 0)
    return false;

  double t0 = -1.0;
  dvec hit = dv(0.0, 0.0, 0.0);
  for (int i = 0; i < nt; ++i) {
    dvec p;
    const double t = intersect_triangle(org, dir, tris[i], p);
    if (t < 0.0)
      continue;
    if (t0 < 0.0 || t < t0) {
      t0 = t;
      hit = p;
    }
  }
  if (t0 < 0.0)
    return false;

  w[0] = float((hit.x - (orig.x + double(id[0]) * span.x)) / span.x);
  w[1] = float((hit.y - (orig.y + double(id[1]) * span.y)) / span.y);
  w[2] = float((hit.z - (orig.z + double(id[2]) * span.z)) / span.z);
  t_hit = t0;
  return true;
}

// ---------------------------------------------------------------------------
// Shading, transfer function, clipping -- detail::shading + transfer_function.h
// ---------------------------------------------------------------------------

// detail::blinn_phong, including the ported fixes: lights ACCUMULATE, each
// channel uses its own light channel, the specular exponent is real, ambient
// is a real term, and the 0.9 output gain is kept.
__device__ inline void blinn_phong(const dev_request &q, const float base[3], const dvec &normal,
                                   const dvec &view, float shininess, float out[3]) {
  float r = q.ambient * base[0];
  float g = q.ambient * base[1];
  float b = q.ambient * base[2];

  for (int i = 0; i < q.light_count; ++i) {
    const cuda_light &l = q.lights[i];
    const dvec ldir = dnormalized(dv(l.direction[0], l.direction[1], l.direction[2]));
    const dvec half = dnormalized(ldir + view);
    float ndotl = float(dot(normal, ldir));
    float ndoth = float(dot(normal, half));

    if (q.two_sided) {
      ndotl = fabsf(ndotl);
      ndoth = fabsf(ndoth);
    } else if (ndotl >= 0.f) {
      ndoth = ndoth > 0.f ? ndoth : 0.f;
    } else {
      ndotl = 0.f;
      ndoth = 0.f;
    }

    const float spec = ndoth > 0.f ? powf(ndoth, shininess) : 0.f;
    r += base[0] * ndotl * l.color[0] + l.color[0] * spec;
    g += base[1] * ndotl * l.color[1] + l.color[1] * spec;
    b += base[2] * ndotl * l.color[2] + l.color[2] * spec;
  }

  const float gain = defaults::shading_gain;
  out[0] = fminf(gain * r, 1.f);
  out[1] = fminf(gain * g, 1.f);
  out[2] = fminf(gain * b, 1.f);
}

// baked_transfer_function::sample -- nearest entry, clamped, NaN -> entry 0.
__device__ inline void lut_sample(const dev_request &q, float value, float out[4]) {
  if (!q.tf_active) {
    out[0] = out[1] = out[2] = out[3] = 0.f;
    return;
  }
  double t = (double(value) - q.tf_lo) * q.tf_inv_width;
  // The inverted test routes NaN (a NaN voxel in a Float volume) to entry 0
  // instead of computing an undefined index.
  if (!(t > 0.0))
    t = 0.0;
  else if (t > 1.0)
    t = 1.0;
  const int i = int(t * double(q.lut_size - 1) + 0.5);
  const float *e = q.lut + std::size_t(i) * 4;
  out[0] = e[0];
  out[1] = e[1];
  out[2] = e[2];
  out[3] = e[3];
}

// gradient_opacity_ramp::factor -- NaN magnitude maps to 0; magnitudes above
// ramp2 are cut off (the documented deviation from the legacy gradtbl).
__device__ inline float gradient_factor(const dev_request &q, double magnitude) {
  if (!q.ramp_enabled)
    return 1.0f;
  if (!(magnitude >= q.ramp0) || magnitude > q.ramp2)
    return 0.0f;
  if (magnitude >= q.ramp1)
    return float(q.ramp_plateau);
  const double span = q.ramp1 - q.ramp0;
  if (span <= 0.0)
    return float(q.ramp_plateau);
  return float(q.ramp_plateau * (magnitude - q.ramp0) / span);
}

__device__ inline bool culled_by_planes(const dev_request &q, const dvec &p) {
  for (int i = 0; i < q.plane_count; ++i) {
    const cuda_cut_plane &c = q.planes[i];
    const dvec n = dv(c.normal[0], c.normal[1], c.normal[2]);
    if (dot(p - dv(c.point[0], c.point[1], c.point[2]), n) < 0.0)
      return true;
  }
  return false;
}

// The inverted test maps NaN to 0 instead of an undefined float->uchar cast.
__device__ inline unsigned char to_byte(float c) {
  const float v = !(c > 0.f) ? 0.f : (c < 1.f ? c : 1.f);
  return (unsigned char)(v * 255.0f + 0.5f);
}

// Slab-method ray/AABB intersection; entry clamps to t = 0 so a camera INSIDE
// the scene renders from its position.  The finiteness guard turns a
// NaN/degenerate box into a miss rather than an unbounded march.
__device__ inline bool intersect_box(const dvec &org, const dvec &dir, const double omin[3],
                                     const double omax[3], double &t0, double &t1) {
  t0 = 0.0;
  t1 = INFINITY;
  const double o[3] = {org.x, org.y, org.z};
  const double d[3] = {dir.x, dir.y, dir.z};
  for (int a = 0; a < 3; ++a) {
    if (d[a] == 0.0) {
      if (o[a] < omin[a] || o[a] > omax[a])
        return false;
      continue;
    }
    double ta = (omin[a] - o[a]) / d[a];
    double tb = (omax[a] - o[a]) / d[a];
    if (ta > tb) {
      const double tmp = ta;
      ta = tb;
      tb = tmp;
    }
    t0 = t0 > ta ? t0 : ta;
    t1 = t1 < tb ? t1 : tb;
  }
  return t0 <= t1 && isfinite(t0) && isfinite(t1);
}

// ---------------------------------------------------------------------------
// Scene-graph transform helpers (prepared_volume's to_local_*/normal_to_world)
// ---------------------------------------------------------------------------

__device__ inline dvec to_local_point(const dev_request &q, const dvec &p) {
  if (!q.transformed)
    return p;
  const double *m = q.w2l;
  return dv(m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3],
            m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
            m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]);
}

__device__ inline dvec to_local_vector(const dev_request &q, const dvec &v) {
  if (!q.transformed)
    return v;
  const double *m = q.w2l;
  return dv(m[0] * v.x + m[1] * v.y + m[2] * v.z, m[4] * v.x + m[5] * v.y + m[6] * v.z,
            m[8] * v.x + m[9] * v.y + m[10] * v.z);
}

// transpose(inverse(A)) * n, using the already-inverted linear part.
__device__ inline dvec normal_to_world(const dev_request &q, const dvec &n) {
  if (!q.transformed)
    return n;
  const double *i = q.w2l;
  return dv(i[0] * n.x + i[4] * n.y + i[8] * n.z, i[1] * n.x + i[5] * n.y + i[9] * n.z,
            i[2] * n.x + i[6] * n.y + i[10] * n.z);
}

// ---------------------------------------------------------------------------
// Compositing state
// ---------------------------------------------------------------------------

struct iso_hit {
  double t;
  float color[3];
  float opacity;
};

struct ray_accum {
  float r, g, b, a;
  bool depth_set;
  float depth;
};

// Front-to-back associated-color compositing; also latches the depth map the
// first time accumulated alpha crosses the threshold.
__device__ inline void composite(ray_accum &acc, const dev_request &q, const float c[3], float a,
                                 double t, double z_scale) {
  const float ratio = a * (1.f - acc.a);
  acc.r += c[0] * ratio;
  acc.g += c[1] * ratio;
  acc.b += c[2] * ratio;
  acc.a += ratio;
  if (!acc.depth_set && acc.a >= q.depth_alpha_threshold) {
    acc.depth = float(t * z_scale);
    acc.depth_set = true;
  }
}

// Insert one hit keeping the buffer sorted by t.  DEVIATION from the CPU path,
// which collects an unbounded std::vector and stable_sorts it: the per-thread
// buffer is capped at cuda_limits::max_iso_hits_per_ray and on overflow the
// FARTHEST hit is dropped, keeping the NEAREST ones.
//
// The cap IS reachable, and not only by pathological input: collection and
// compositing are separate phases here (the DDA runs to completion before the
// first sample composites), so the opacity cutoff cannot bound the hit count
// the way it bounds visible work.  8 isosurfaces at low opacity through a
// convoluted surface exceed 32 crossings on some rays.  Dropping the farthest
// is the right policy -- those are the hits the front-to-back accumulation
// would have contributed least, and usually nothing -- but it IS a silent
// divergence from the CPU image for such a scene, not an unreachable branch.
// Raising the cap costs per-thread local memory on every ray.
//
// Insertion stops at equal t, so the order of equal-t hits matches the CPU
// stable_sort.
__device__ inline void push_hit(iso_hit *hits, int &n, const iso_hit &h) {
  constexpr int cap = cuda_limits::max_iso_hits_per_ray;
  int pos = n;
  while (pos > 0 && hits[pos - 1].t > h.t)
    --pos;
  if (pos >= cap)
    return; // buffer full and this hit is farther than everything kept
  const int last = n < cap ? n : cap - 1;
  for (int i = last; i > pos; --i)
    hits[i] = hits[i - 1];
  hits[pos] = h;
  if (n < cap)
    ++n;
}

__device__ inline void composite_hits_up_to(ray_accum &acc, const dev_request &q,
                                            const iso_hit *hits, int nhits, int &cursor,
                                            double t_limit, double z_scale) {
  while (cursor < nhits && hits[cursor].t <= t_limit && acc.a < q.opacity_cutoff) {
    const iso_hit &h = hits[cursor++];
    // An isosurface hit latches the depth map on its own -- the frame contract
    // is "first iso hit or first threshold-crossing sample, whichever first".
    if (!acc.depth_set) {
      acc.depth = float(h.t * z_scale);
      acc.depth_set = true;
    }
    composite(acc, q, h.color, h.opacity, h.t, z_scale);
  }
}

// ---------------------------------------------------------------------------
// The kernel
// ---------------------------------------------------------------------------

__global__ void volren_raycast_kernel(const dev_request q, unsigned char *color, float *depth) {
  const int px = int(blockIdx.x * blockDim.x + threadIdx.x);
  const int py = int(blockIdx.y * blockDim.y + threadIdx.y);
  if (px >= q.width || py >= q.height)
    return;

  const std::size_t pixel = std::size_t(py) * std::size_t(q.width) + std::size_t(px);
  unsigned char *cpx = color + pixel * 4;

  // ray_generator::at -- NDC through the pixel CENTER, v = +1 at the TOP row.
  const double u = (double(px) + 0.5) / double(q.width) * 2.0 - 1.0;
  const double v = 1.0 - (double(py) + 0.5) / double(q.height) * 2.0;
  dvec org, dir;
  if (q.perspective) {
    org = q.eye;
    dir = dnormalized(q.forward + q.right * (u * q.tan_half * q.aspect) +
                      q.true_up * (v * q.tan_half));
  } else {
    org = q.eye + q.right * (u * q.parallel_scale * q.aspect) + q.true_up * (v * q.parallel_scale);
    dir = q.forward;
  }

  const double smin[3] = {q.scene_min.x, q.scene_min.y, q.scene_min.z};
  const double smax[3] = {q.scene_max.x, q.scene_max.y, q.scene_max.z};
  double t0 = 0.0, t1 = 0.0;
  if (!intersect_box(org, dir, smin, smax, t0, t1)) {
    depth[pixel] = INFINITY;
    cpx[0] = to_byte(q.background[0]);
    cpx[1] = to_byte(q.background[1]);
    cpx[2] = to_byte(q.background[2]);
    cpx[3] = 0;
    return;
  }

  const dvec view_vec = -dir; // toward the viewer, unit
  const double z_scale = dot(dir, q.forward);

  ray_accum acc;
  acc.r = acc.g = acc.b = acc.a = 0.f;
  acc.depth_set = false;
  acc.depth = INFINITY;

  spline_cache spline;
  spline.reset();

  // ---- Isosurface hits: exact per-cell ray traversal ----------------------
  // Every cell the ray actually crosses is enumerated with an Amanatides-Woo
  // DDA and MC-intersected exactly (the legacy tracer only tested cells a
  // march SAMPLE landed in -- the black-speckle artifact); the hits are then
  // merged into the compositing stream at their ray parameter.
  iso_hit hits[cuda_limits::max_iso_hits_per_ray];
  int nhits = 0;

  if (q.iso_count > 0) {
    const dvec lorg = to_local_point(q, org);
    const dvec ldir = to_local_vector(q, dir); // unnormalized: t preserved

    const double vmin[3] = {q.minb.x, q.minb.y, q.minb.z};
    const double vmax[3] = {q.minb.x + q.span.x * double(q.dimx - 1),
                            q.minb.y + q.span.y * double(q.dimy - 1),
                            q.minb.z + q.span.z * double(q.dimz - 1)};
    double tv0 = 0.0, tv1 = 0.0;
    if (intersect_box(lorg, ldir, vmin, vmax, tv0, tv1)) {
      tv0 = tv0 > t0 ? tv0 : t0;
      tv1 = tv1 < t1 ? tv1 : t1;
      if (tv0 <= tv1) {
        // Start half a hair inside so the entry cell resolves.
        const double t_start = tv0 + (tv1 - tv0) * 1e-9;
        long long idx[3];
        if (grid_cell_index(q, lorg + ldir * t_start, idx)) {
          const double ld[3] = {ldir.x, ldir.y, ldir.z};
          const double lmin[3] = {q.minb.x, q.minb.y, q.minb.z};
          const double lspan[3] = {q.span.x, q.span.y, q.span.z};
          const long long dims[3] = {q.dimx, q.dimy, q.dimz};
          const double lorg_a[3] = {lorg.x, lorg.y, lorg.z};
          long long stepc[3];
          double t_max[3], t_delta[3];
          for (int a = 0; a < 3; ++a) {
            if (ld[a] > 0.0) {
              stepc[a] = 1;
              t_delta[a] = lspan[a] / ld[a];
              t_max[a] = ((lmin[a] + double(idx[a] + 1) * lspan[a]) - lorg_a[a]) / ld[a];
            } else if (ld[a] < 0.0) {
              stepc[a] = -1;
              t_delta[a] = -lspan[a] / ld[a];
              t_max[a] = ((lmin[a] + double(idx[a]) * lspan[a]) - lorg_a[a]) / ld[a];
            } else {
              stepc[a] = 0;
              t_delta[a] = INFINITY;
              t_max[a] = INFINITY;
            }
          }

          const long long max_cells = dims[0] + dims[1] + dims[2] + 3;
          double t_cell = tv0;
          for (long long n = 0; n < max_cells && t_cell <= tv1; ++n) {
            float vals[8];
            grid_corners(q, idx[0], idx[1], idx[2], vals);
            float min_val = vals[0], max_val = vals[0];
            for (int j = 1; j < 8; ++j) {
              // NOT fminf/fmaxf: those are IEEE fmin/fmax and return the
              // non-NaN operand, whereas the CPU's std::min/std::max keep a
              // NaN accumulator.  Since the fold seeds from vals[0], a NaN
              // first corner must poison the range on both backends or the
              // isovalue-bracket test disagrees.
              min_val = vals[j] < min_val ? vals[j] : min_val;
              max_val = max_val < vals[j] ? vals[j] : max_val;
            }

            float func[8];
            bool cell_filled = false;
            for (int s = 0; s < q.iso_count; ++s) {
              const cuda_isosurface &surf = q.iso[s];
              if (surf.value < min_val || surf.value > max_val)
                continue;
              if (!cell_filled) {
                for (int vtx = 0; vtx < 8; ++vtx)
                  func[vtx] = vals[c_vertex_from_binary[vtx]];
                cell_filled = true;
              }
              float w[3];
              double t_hit = 0.0;
              if (!intersect_isosurface_in_cell(lorg, ldir, float(surf.value), idx, q.minb, q.span,
                                                func, w, t_hit))
                continue;
              if (t_hit < t0 || t_hit > tv1 + q.unit_step)
                continue;
              if (q.plane_count > 0 && culled_by_planes(q, org + dir * t_hit))
                continue;
              const dvec grad = spline.evaluate(q, idx, w);
              const dvec normal = dnormalized(normal_to_world(q, grad));
              iso_hit h;
              h.t = t_hit;
              blinn_phong(q, surf.color, normal, view_vec, surf.shininess, h.color);
              h.opacity = surf.opacity;
              push_hit(hits, nhits, h);
            }

            // Advance to the neighboring cell across the nearest boundary.
            int axis = 0;
            if (t_max[1] < t_max[axis])
              axis = 1;
            if (t_max[2] < t_max[axis])
              axis = 2;
            t_cell = t_max[axis];
            idx[axis] += stepc[axis];
            if (idx[axis] < 0 || idx[axis] > dims[axis] - 2)
              break;
            t_max[axis] += t_delta[axis];
          }
        }
      }
    }
  }

  int hit_cursor = 0;
  long long last_cell[3] = {-1, -1, -1};

  // The step count is bounded by construction (unit_step = scene diagonal /
  // steps, and a ray's span inside the box never exceeds that diagonal); the
  // guard is a watchdog so a pathological request cannot hang the device.
  const int max_iterations = q.steps + 16;
  int iteration = 0;
  for (double t = t0; t <= t1 && acc.a < q.opacity_cutoff && iteration < max_iterations;
       t += q.unit_step, ++iteration) {
    composite_hits_up_to(acc, q, hits, nhits, hit_cursor, t, z_scale);
    const dvec pnt = org + dir * t;
    if (q.plane_count > 0 && culled_by_planes(q, pnt))
      continue;

    // Sample in volume-local space (the scene-graph model transform).
    const dvec lpnt = to_local_point(q, pnt);
    long long idx[3];
    if (!grid_cell_index(q, lpnt, idx))
      continue;
    if (idx[0] == last_cell[0] && idx[1] == last_cell[1] && idx[2] == last_cell[2])
      continue; // one contribution per cell (the volren sampling model)
    last_cell[0] = idx[0];
    last_cell[1] = idx[1];
    last_cell[2] = idx[2];

    float vals[8];
    float min_val = 0.f, max_val = 0.f;
    bool have_corners = false;

    // 2. Unshaded transfer function (legacy COL_DENSITY).
    if (q.unshaded) {
      grid_corners(q, idx[0], idx[1], idx[2], vals);
      min_val = max_val = vals[0];
      for (int j = 1; j < 8; ++j) {
        // std::min/std::max semantics, not IEEE fmin/fmax -- see the DDA fold.
        min_val = vals[j] < min_val ? vals[j] : min_val;
        max_val = max_val < vals[j] ? vals[j] : max_val;
      }
      have_corners = true;
      const bool in_window =
          !q.window_enabled || (double(min_val) <= q.window_max && double(max_val) >= q.window_min);
      if (in_window) {
        float w[3];
        grid_local_weights(q, lpnt, idx[0], idx[1], idx[2], w);
        const float den = trilinear(w, vals);
        float s[4];
        lut_sample(q, den, s);
        if (s[3] > 0.f)
          composite(acc, q, s, s[3], t, z_scale);
      }
    }

    // 3. Shaded transfer function (legacy RAY_CASTING).
    if (q.shaded) {
      if (!have_corners) {
        grid_corners(q, idx[0], idx[1], idx[2], vals);
        have_corners = true;
      }
      float w[3];
      grid_local_weights(q, lpnt, idx[0], idx[1], idx[2], w);
      const float den = trilinear(w, vals);
      if (q.window_enabled && (double(den) < q.window_min || double(den) > q.window_max))
        continue;
      const dvec grad = spline.evaluate(q, idx, w);
      float s[4];
      lut_sample(q, den, s);
      const float a = s[3] * gradient_factor(q, sqrt(dot(grad, grad)));
      if (a > 0.f) {
        const dvec normal = dnormalized(normal_to_world(q, grad));
        float shaded[3];
        blinn_phong(q, s, normal, view_vec, q.tf_shininess, shaded);
        composite(acc, q, shaded, a, t, z_scale);
      }
    }
  }
  // Hits between the last sample and the exit point.
  composite_hits_up_to(acc, q, hits, nhits, hit_cursor, t1, z_scale);

  depth[pixel] = acc.depth;

  // Over-blend the remaining transparency with the background (replaces the
  // legacy divide-by-alpha normalization of saturated rays).
  const float rest = 1.f - acc.a;
  cpx[0] = to_byte(acc.r + q.background[0] * rest);
  cpx[1] = to_byte(acc.g + q.background[1] * rest);
  cpx[2] = to_byte(acc.b + q.background[2] * rest);
  cpx[3] = to_byte(acc.a);
}

// ---------------------------------------------------------------------------
// Host side
// ---------------------------------------------------------------------------

// cudaFree(nullptr) is a documented no-op, so the destructor needs no guard.
// RAII (rather than drive.cu's explicit free list) because CUDA_CHECK throws.
struct device_buffer {
  void *p = nullptr;
  device_buffer() = default;
  device_buffer(const device_buffer &) = delete;
  device_buffer &operator=(const device_buffer &) = delete;
  ~device_buffer() { cudaFree(p); }
  void alloc(std::size_t bytes) { CUDA_CHECK(cudaMalloc(&p, bytes)); }
};

std::size_t voxel_size(cvc::data_type t) {
  switch (t) {
  case cvc::UChar:
  case cvc::Char:
    return 1;
  case cvc::UShort:
    return 2;
  case cvc::UInt:
  case cvc::Float:
  case cvc::Int:
    return 4;
  case cvc::Double:
  case cvc::UInt64:
  case cvc::Int64:
    return 8;
  default:
    return 0;
  }
}

} // namespace

bool raycast_cuda_available() {
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

frame raycast_cuda(const raycast_cuda_request &req) {
  // Validate everything BEFORE the first allocation (the drive.cu convention),
  // so an invalid request never leaks device memory.
  if (!req.data)
    throw volren_error("raycast_cuda: null volume buffer");
  if (req.dimx < 2 || req.dimy < 2 || req.dimz < 2)
    throw volren_error("raycast_cuda: volumes need at least 2 voxels per axis");
  if (!(req.span[0] > 0.0) || !(req.span[1] > 0.0) || !(req.span[2] > 0.0))
    throw volren_error("raycast_cuda: volume span must be positive on every axis");
  const std::size_t vsize = voxel_size(req.type);
  if (vsize == 0)
    throw volren_error("raycast_cuda: unsupported cvc::data_type");
  if (req.steps < 1)
    throw volren_error("raycast_cuda: steps must be >= 1");
  if (!(req.unit_step > 0.0) || !std::isfinite(req.unit_step))
    throw volren_error("raycast_cuda: unit_step must be positive and finite");
  if (!(req.opacity_cutoff > 0.f) || req.opacity_cutoff > 1.f)
    throw volren_error("raycast_cuda: opacity_cutoff must be in (0, 1]");
  if (req.isosurface_count < 0 || req.isosurface_count > cuda_limits::max_isosurfaces)
    throw volren_error("raycast_cuda: too many isosurfaces for the CUDA backend");
  if (req.light_count < 0 || req.light_count > cuda_limits::max_lights)
    throw volren_error("raycast_cuda: too many lights for the CUDA backend");
  if (req.cut_plane_count < 0 || req.cut_plane_count > cuda_limits::max_cut_planes)
    throw volren_error("raycast_cuda: too many cut planes for the CUDA backend");
  if (req.tf_active && (!req.lut || req.lut_size < 2))
    throw volren_error("raycast_cuda: an active transfer function needs a LUT of >= 2 entries");

  // basis() carries all the camera validation (raster, fov, degenerate pose)
  // and throws cvc::volren_error, matching the CPU entry point.
  const view_basis basis = req.cam.basis();
  const int width = req.cam.width;
  const int height = req.cam.height;

  ensure_mc_tables();

  dev_request q = {};
  q.eye = {req.cam.eye[0], req.cam.eye[1], req.cam.eye[2]};
  q.right = {basis.right.x, basis.right.y, basis.right.z};
  q.true_up = {basis.true_up.x, basis.true_up.y, basis.true_up.z};
  q.forward = {-basis.back.x, -basis.back.y, -basis.back.z};
  q.tan_half = std::tan(0.5 * req.cam.vfov_degrees * 3.14159265358979323846 / 180.0);
  q.parallel_scale = req.cam.parallel_scale;
  q.aspect = req.cam.aspect();
  q.perspective = req.cam.projection == camera::projection_type::perspective;
  q.width = width;
  q.height = height;

  q.type = int(req.type);
  q.dimx = req.dimx;
  q.dimy = req.dimy;
  q.dimz = req.dimz;
  q.minb = {req.minb[0], req.minb[1], req.minb[2]};
  q.span = {req.span[0], req.span[1], req.span[2]};

  q.transformed = req.transformed ? 1 : 0;
  for (int i = 0; i < 16; ++i)
    q.w2l[i] = req.world_to_local[i];

  q.tf_active = req.tf_active ? 1 : 0;
  q.lut_size = req.lut_size;
  q.tf_lo = req.tf_lo;
  q.tf_inv_width = req.tf_hi > req.tf_lo ? 1.0 / (req.tf_hi - req.tf_lo) : 1.0;

  q.ramp_enabled = req.gradient_ramp_enabled ? 1 : 0;
  q.ramp0 = req.ramp0;
  q.ramp1 = req.ramp1;
  q.ramp2 = req.ramp2;
  q.ramp_plateau = req.gradient_plateau;

  q.iso_count = req.isosurface_count;
  for (int i = 0; i < req.isosurface_count; ++i)
    q.iso[i] = req.isosurfaces[i];
  q.light_count = req.light_count;
  for (int i = 0; i < req.light_count; ++i)
    q.lights[i] = req.lights[i];
  q.plane_count = req.cut_plane_count;
  for (int i = 0; i < req.cut_plane_count; ++i)
    q.planes[i] = req.cut_planes[i];

  q.scene_min = {req.scene_min[0], req.scene_min[1], req.scene_min[2]};
  q.scene_max = {req.scene_max[0], req.scene_max[1], req.scene_max[2]};
  q.steps = req.steps;
  q.unit_step = req.unit_step;
  q.opacity_cutoff = req.opacity_cutoff;
  q.depth_alpha_threshold = req.depth_alpha_threshold;
  q.ambient = req.ambient;
  q.two_sided = req.two_sided ? 1 : 0;
  q.background[0] = req.background[0];
  q.background[1] = req.background[1];
  q.background[2] = req.background[2];
  q.tf_shininess = req.tf_shininess;
  q.shaded = req.shaded ? 1 : 0;
  q.unshaded = req.unshaded ? 1 : 0;
  q.window_enabled = req.window_enabled ? 1 : 0;
  q.window_min = req.window_min;
  q.window_max = req.window_max;

  // Per-render upload of the raw volume buffer and the LUT.  FUTURE WORK: a
  // resident device-side volume cache (the sim_world_cuda model) keyed on the
  // pinned host buffer, so a camera-only change re-launches without the H2D.
  const std::size_t voxels = std::size_t(req.dimx) * std::size_t(req.dimy) * std::size_t(req.dimz);
  const std::size_t volume_bytes = voxels * vsize;
  const std::size_t color_bytes = std::size_t(width) * std::size_t(height) * 4;
  const std::size_t depth_bytes = std::size_t(width) * std::size_t(height) * sizeof(float);

  device_buffer d_volume, d_lut, d_color, d_depth;
  d_volume.alloc(volume_bytes);
  CUDA_CHECK(cudaMemcpy(d_volume.p, req.data, volume_bytes, cudaMemcpyHostToDevice));
  if (q.tf_active) {
    const std::size_t lut_bytes = std::size_t(req.lut_size) * 4 * sizeof(float);
    d_lut.alloc(lut_bytes);
    CUDA_CHECK(cudaMemcpy(d_lut.p, req.lut, lut_bytes, cudaMemcpyHostToDevice));
  }
  d_color.alloc(color_bytes);
  d_depth.alloc(depth_bytes);

  q.data = static_cast<const unsigned char *>(d_volume.p);
  q.lut = static_cast<const float *>(d_lut.p);

  // One thread per pixel.  Default stream only: nothing else is in flight.
  const dim3 threads(16, 16);
  const dim3 blocks((unsigned(width) + threads.x - 1) / threads.x,
                    (unsigned(height) + threads.y - 1) / threads.y);
  // Clear any error left on this thread by an unrelated earlier CUDA call:
  // cudaGetLastError() reports the last error since it was last READ, so
  // without this a failure some other libcvc code already handled would be
  // attributed to our launch and force a spurious CPU fallback.
  (void)cudaGetLastError();
  volren_raycast_kernel<<<blocks, threads>>>(q, static_cast<unsigned char *>(d_color.p),
                                             static_cast<float *>(d_depth.p));
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  frame out;
  out.color = cvc::image(width, height, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  out.depth = cvc::image(width, height, cvc::image::pixel_format::GRAY, cvc::image::data_type::f32);
  CUDA_CHECK(cudaMemcpy(out.color.data(), d_color.p, color_bytes, cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(out.depth.data(), d_depth.p, depth_bytes, cudaMemcpyDeviceToHost));
  return out;
}

} // namespace volren
} // namespace cvc
