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
#include <cstring>
#include <cuda_runtime.h>
#include <cvc/core/types.h>
#include <cvc/utility/cuda_utils.h>
#include <cvc/volren/detail/mc_tables.h>
#include <cvc/volren/raycaster_cuda.h>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

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
// One volume's marching state (detail::grid_sampler + its slice of
// volume_settings), device pointers throughout.  These live in DEVICE MEMORY,
// not in the kernel parameter block: at ~600 bytes each,
// cuda_limits::max_volumes of them is ~10 KB and the parameter block caps at
// 4 KB on every architecture this builds for.  dev_request therefore carries a
// pointer plus a count, and the array is staged with one cudaMemcpy per render
// (it is small and changes with the settings, unlike the voxels, which stay
// resident -- see volume_cache below).
struct dev_volume {
  const unsigned char *data;
  int type; // cvc::data_type
  long long dimx, dimy, dimz;
  dvec minb, span;
  // Local-space cull box for the per-ray active test: grid_cell_index() can
  // only succeed strictly inside it, plus a one-voxel margin per face so the
  // rejection is immune to the rounding gap between solving for t on a face
  // and dividing a marched point by span.
  dvec cull_lo, cull_hi;

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

  int shaded, unshaded, window_enabled;
  double window_min, window_max;

  // cuda_volume::ao_ok -- this volume is a signed distance field AND the scene
  // asked for ambient occlusion.  0 keeps its isosurface hits on the pre-AO
  // shading path exactly.
  int ao_ok;
};

// One built light-view shadow map, device pointer for its depth raster.  The
// FRAME rides in the parameter block (~128 bytes) because every sample's
// lookup reads all of it; only the raster lives in device memory.
struct dev_shadow_map {
  dvec eye, right, up, forward;
  double parallel_scale, texel_world;
  int width, height;
  const float *depth;
  // shadow_mode::deep: `slices` 0 keeps the hard lookup, which is what makes a
  // hard-mode frame bit-identical to the kernel before deep maps existed.
  int slices;
  double depth_min, slice_dz;
  const float *profile;
};

// The camera is pre-expanded exactly like raycaster.cpp's ray_generator, and
// the scene-level settings vectors are flat fixed arrays, so the request
// itself still rides in the kernel parameter block (~1 KB) -- the
// dev_field/dev_veh convention from nav/drive.cu, no __constant__ scene state.
struct dev_request {
  // Camera (ray_generator).
  dvec eye, right, true_up, forward;
  double tan_half, parallel_scale, aspect;
  int perspective;
  int width, height;

  // The scene's volumes, in device memory, in the CPU march's fixed order.
  const dev_volume *vols;
  int nvol;

  int light_count;
  cuda_light lights[cuda_limits::max_lights];
  int plane_count;
  cuda_cut_plane planes[cuda_limits::max_cut_planes];

  // Volumetric shadows.  shadow_count == 0 is the pre-shadow path: no lookup,
  // and blinn_phong gets a null visibility array.
  int shadow_count;
  dev_shadow_map shadows[cuda_limits::max_shadow_maps];
  // Light index -> index into `shadows`, or -1.  Resolved on the host so the
  // kernel never scans the maps looking for a light.
  int light_map[cuda_limits::max_lights];
  float shadow_strength;
  double shadow_bias_constant, shadow_slope_scale;
  // Percentage-closer filtering, resolved on the host: `shadow_pcf_half` 0 is
  // the single-tap lookup and therefore the pre-PCF instruction stream.
  int shadow_pcf_half;
  double shadow_pcf_radius, shadow_pcf_widen;

  dvec scene_min, scene_max;
  int steps;
  double unit_step;
  float opacity_cutoff, depth_alpha_threshold, ambient;
  int two_sided;
  // Ambient shaping (detail::ambient_scale + detail::sdf_occlusion).
  // `ambient_shaped` 0 makes every shading site pass the flat
  // {ambient, ambient, ambient} triple, which is the pre-shaping expression bit
  // for bit -- and with it the whole hemisphere/AO tail is dead code the
  // scheduler drops.
  int ambient_shaped, ambient_hemisphere;
  float ambient_sky[3], ambient_ground[3];
  dvec ambient_up;
  float ao_strength;
  double ao_radius;
  int ao_samples;
  float shading_gain;
  float background[3];
  float tf_shininess;
  float specular;
  int supersample; // sub-samples per pixel EDGE; the pixel marches its square
  // Does ANY volume in the scene ask for a transfer-function sample?  Resolved
  // on the host so the kernel can drop the whole `steps`-long march loop for an
  // isosurface-only scene, where every iteration of it does the window test,
  // to_local_point and grid_cell_index and then composites nothing.  The
  // isosurface hits still composite -- see march_ray's tail.
  int any_tf;

  // ---- deep-shadow profile capture (raycast_cuda_request) ----------------
  // Read only by the CAPTURE == 1 kernel instantiation; the ordinary render
  // never touches these.  `prof_out` is DEVICE memory, (prof_slices + 1) floats
  // per pixel, pre-filled by the kernel itself before the march.
  float *prof_out;
  int prof_slices;
  double prof_z0, prof_dz;
};

// Per-ray capture state.  It exists ONLY inside the CAPTURE == 1
// instantiation: composite() takes it by pointer and the compile-time flag
// discards every reference to it in the ordinary kernel, so there is no per-ray
// slice array, no extra predicate in the compositing recurrence, and no change
// to the ordinary path's per-thread frame.  Contributions arrive in
// non-decreasing t, so one forward cursor streams the knots straight to global
// memory, each written exactly once.
struct prof_state {
  float *out;        // this ray's terminal slot; knot k+1 is out[(1+k) * plane]
  std::size_t plane; // width * height (the payload is PLANE-major)
  int slices;
  double z0, dz;
  int cursor;
  bool done;
};

// ---------------------------------------------------------------------------
// Sampling (detail::grid_sampler)
// ---------------------------------------------------------------------------

__device__ inline float grid_at(const dev_volume &V, long long i, long long j, long long k) {
  const std::size_t n = std::size_t(i) + std::size_t(j) * std::size_t(V.dimx) +
                        std::size_t(k) * std::size_t(V.dimx) * std::size_t(V.dimy);
  switch (V.type) {
  case cvc::UChar:
    return float(reinterpret_cast<const unsigned char *>(V.data)[n]);
  case cvc::UShort:
    return float(reinterpret_cast<const std::uint16_t *>(V.data)[n]);
  case cvc::UInt:
    return float(reinterpret_cast<const std::uint32_t *>(V.data)[n]);
  case cvc::Float:
    return reinterpret_cast<const float *>(V.data)[n];
  case cvc::Double:
    return float(reinterpret_cast<const double *>(V.data)[n]);
  case cvc::UInt64:
    return float(reinterpret_cast<const unsigned long long *>(V.data)[n]);
  case cvc::Char:
    return float(reinterpret_cast<const signed char *>(V.data)[n]);
  case cvc::Int:
    return float(reinterpret_cast<const std::int32_t *>(V.data)[n]);
  case cvc::Int64:
    return float(reinterpret_cast<const long long *>(V.data)[n]);
  default:
    return 0.f;
  }
}

__device__ inline float grid_at_clamped(const dev_volume &V, long long i, long long j,
                                        long long k) {
  i = i < 0 ? 0 : (i > V.dimx - 1 ? V.dimx - 1 : i);
  j = j < 0 ? 0 : (j > V.dimy - 1 ? V.dimy - 1 : j);
  k = k < 0 ? 0 : (k > V.dimz - 1 ? V.dimz - 1 : k);
  return grid_at(V, i, j, k);
}

// The 8 corner values of a cell in BINARY order (bit0 = x, bit1 = y, bit2 = z).
__device__ inline void grid_corners(const dev_volume &V, long long ci, long long cj, long long ck,
                                    float out[8]) {
  out[0] = grid_at(V, ci, cj, ck);
  out[1] = grid_at(V, ci + 1, cj, ck);
  out[2] = grid_at(V, ci, cj + 1, ck);
  out[3] = grid_at(V, ci + 1, cj + 1, ck);
  out[4] = grid_at(V, ci, cj, ck + 1);
  out[5] = grid_at(V, ci + 1, cj, ck + 1);
  out[6] = grid_at(V, ci, cj + 1, ck + 1);
  out[7] = grid_at(V, ci + 1, cj + 1, ck + 1);
}

__device__ inline void grid_local_weights(const dev_volume &V, const dvec &p, long long ci,
                                          long long cj, long long ck, float w[3]) {
  w[0] = float((p.x - (V.minb.x + double(ci) * V.span.x)) / V.span.x);
  w[1] = float((p.y - (V.minb.y + double(cj) * V.span.y)) / V.span.y);
  w[2] = float((p.z - (V.minb.z + double(ck) * V.span.z)) / V.span.z);
}

// Range-checked in the double domain BEFORE the cast, so NaN / huge quotients
// fail the comparisons instead of hitting an undefined float->int conversion.
__device__ inline bool grid_cell_index(const dev_volume &V, const dvec &p, long long idx[3]) {
  const double qx = (p.x - V.minb.x) / V.span.x;
  const double qy = (p.y - V.minb.y) / V.span.y;
  const double qz = (p.z - V.minb.z) / V.span.z;
  if (!(qx > -1.0 && qx < double(V.dimx)) || !(qy > -1.0 && qy < double(V.dimy)) ||
      !(qz > -1.0 && qz < double(V.dimz)))
    return false;
  idx[0] = (long long)qx; // truncation toward zero, matching the legacy (int) cast
  idx[1] = (long long)qy;
  idx[2] = (long long)qz;
  return idx[0] <= V.dimx - 2 && idx[1] <= V.dimy - 2 && idx[2] <= V.dimz - 2;
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
//
// The CPU march carries ONE cache PER VOLUME (scratch.spline[v]); a thread here
// carries a single cache whose key also names the volume.  The cache is pure
// memoization of grid_at_clamped over a 4^3 neighborhood, so keying it on
// (volume, cell) returns exactly what a per-volume cache would -- it only
// re-reads more often when a ray alternates between volumes.  That costs
// nothing in practice (the march re-evaluates only on entering a NEW cell, so
// the cache misses either way) and it keeps per-thread local memory constant
// instead of scaling with cuda_limits::max_volumes: 16 caches would be 13 KB
// per thread.
struct spline_cache {
  int vol;
  long long cell[3];
  float val[4][4][4];
  float deriv[4][4][3][3];

  __device__ void reset() {
    vol = -1;
    cell[0] = -1;
    cell[1] = -1;
    cell[2] = -1;
  }

  __device__ void refresh(const dev_volume &V) {
    for (int k = -1; k <= 2; ++k)
      for (int j = -1; j <= 2; ++j)
        for (int i = -1; i <= 2; ++i)
          val[k + 1][j + 1][i + 1] = grid_at_clamped(V, cell[0] + i, cell[1] + j, cell[2] + k);

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

  __device__ dvec evaluate(const dev_volume &V, int v, const long long idx[3], const float w[3]) {
    if (v != vol || idx[0] != cell[0] || idx[1] != cell[1] || idx[2] != cell[2]) {
      vol = v;
      cell[0] = idx[0];
      cell[1] = idx[1];
      cell[2] = idx[2];
      refresh(V);
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

// detail::ambient_scale -- the per-channel ambient scale for one sample: the
// flat constant, optionally tinted by the sky/ground hemisphere and attenuated
// by `occlusion` (1 == fully open).  Callers take the `ambient_shaped == 0`
// branch for every scene that asked for neither, and that branch writes the
// same triple the kernel used to inline.
__device__ inline void ambient_scale(const dev_request &q, const dvec &normal, float occlusion,
                                     float out[3]) {
  float r = q.ambient, g = q.ambient, b = q.ambient;
  if (q.ambient_hemisphere) {
    // A degenerate `up` normalizes to {0,0,0}, putting every normal at the
    // equator: a flat 50/50 blend rather than a special case.
    const dvec up = dnormalized(q.ambient_up);
    const float f = float(0.5 + 0.5 * dot(normal, up));
    r *= q.ambient_ground[0] + (q.ambient_sky[0] - q.ambient_ground[0]) * f;
    g *= q.ambient_ground[1] + (q.ambient_sky[1] - q.ambient_ground[1]) * f;
    b *= q.ambient_ground[2] + (q.ambient_sky[2] - q.ambient_ground[2]) * f;
  }
  out[0] = r * occlusion;
  out[1] = g * occlusion;
  out[2] = b * occlusion;
}

// detail::sdf_occlusion -- the ambient-occlusion cone over a signed distance
// field, transcribed term for term (the host header carries the reasoning: why
// one fetch measures a whole sphere, why the falloff is 1/i and not 1/2^i, and
// why a tap that leaves the grid counts as unoccluded at full weight).
__device__ inline float sdf_occlusion(const dev_volume &V, const dvec &p, const dvec &n, double iso,
                                      double radius, int samples) {
  // No direction, no cone -- the host header carries why a degenerate normal
  // must read as UNOCCLUDED rather than as sealed.
  if (!(dot(n, n) > 0.0))
    return 0.f;
  double occ = 0.0, wsum = 0.0;
  for (int i = 1; i <= samples; ++i) {
    const double h = radius * double(i) / double(samples);
    const double w = 1.0 / double(i);
    wsum += w;
    const dvec qp = p + n * h;
    long long idx[3];
    if (!grid_cell_index(V, qp, idx))
      continue; // outside the volume: nothing there to occlude with
    float vals[8], w3[3];
    grid_corners(V, idx[0], idx[1], idx[2], vals);
    grid_local_weights(V, qp, idx[0], idx[1], idx[2], w3);
    const double d = double(trilinear(w3, vals)) - iso;
    double frac = (h - d) / h;
    if (!(frac > 0.0)) // inverted: a NaN voxel reads as UNOCCLUDED, not as dark
      frac = 0.0;
    else if (frac > 1.0) // d < 0: the tap is inside geometry
      frac = 1.0;
    occ += w * frac;
  }
  return wsum > 0.0 ? float(occ / wsum) : 0.f;
}

// detail::local_outward -- the outward unit direction in the volume's LOCAL
// frame.  The span divide is what the shading normal deliberately skips; the AO
// cone cannot skip it because it marches along this vector.
__device__ inline dvec local_outward(const dev_volume &V, const dvec &grad) {
  return dnormalized(dv(grad.x / V.span.x, grad.y / V.span.y, grad.z / V.span.z));
}

// detail::blinn_phong, including the ported fixes: lights ACCUMULATE, each
// channel uses its own light channel, the specular exponent is real, ambient is
// a real per-channel term, and the legacy 0.9 output gain survives as
// q.shading_gain's default.  `vis`, when non-null, is one shadow factor in
// [0,1] per light; it scales DIFFUSE and SPECULAR only, so ambient survives and
// a shadowed region falls into shade rather than crushing to black.
__device__ inline void blinn_phong(const dev_request &q, const float base[3], const dvec &normal,
                                   const dvec &view, const float ambient[3], float shininess,
                                   float out[3], const float *vis = nullptr) {
  float r = ambient[0] * base[0];
  float g = ambient[1] * base[1];
  float b = ambient[2] * base[2];

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

    // q.specular folds into the lobe, so at its 1.0 default the multiply is
    // exact and the expression is unchanged.
    const float spec = ndoth > 0.f ? powf(ndoth, shininess) * q.specular : 0.f;
    const float v = vis ? vis[i] : 1.f;
    r += (base[0] * ndotl * l.color[0] + l.color[0] * spec) * v;
    g += (base[1] * ndotl * l.color[1] + l.color[1] * spec) * v;
    b += (base[2] * ndotl * l.color[2] + l.color[2] * spec) * v;
  }

  const float gain = q.shading_gain;
  out[0] = fminf(gain * r, 1.f);
  out[1] = fminf(gain * g, 1.f);
  out[2] = fminf(gain * b, 1.f);
}

// ---------------------------------------------------------------------------
// Volumetric shadows -- the device transcription of detail/shadow_map.h
// ---------------------------------------------------------------------------
// shadow_view::project: three dot products against the stored orthonormal
// light-view frame.  No matrix and no near/far, because the light camera is
// ORTHOGRAPHIC: its frame::depth stores exactly dot(p - eye, forward), the
// same quantity computed here.  Range-checked in the double domain before the
// int cast so NaN fails the test instead of converting undefined.
__device__ inline bool shadow_project(const dev_shadow_map &m, const dvec &p, int &ix, int &iy,
                                      double &depth) {
  const dvec rel = p - m.eye;
  const double s = dot(rel, m.right);
  const double t = dot(rel, m.up);
  depth = dot(rel, m.forward);
  const double u = s / m.parallel_scale;
  const double v = t / m.parallel_scale;
  const double fx = (u + 1.0) * 0.5 * double(m.width);
  const double fy = (1.0 - v) * 0.5 * double(m.height);
  if (!(fx >= 0.0 && fx < double(m.width)) || !(fy >= 0.0 && fy < double(m.height)))
    return false;
  ix = int(fx); // fx >= 0, so truncation == floor
  iy = int(fy);
  return true;
}

// detail::shadow_bias + detail::shadow_visibility, fused: the constant term is
// already folded host-side into q.shadow_bias_constant.  The cos floor of 0.1
// caps tan at ~9.95 so a flat-gradient sample gets a conservative (LIT) bias
// rather than a division by zero.
// One texel's answer, hard or deep -- detail::shadow_tap / shadow_tap_deep.
__device__ inline float shadow_tap(const dev_request &q, const dev_shadow_map &m, int tx, int ty,
                                   double depth, double bias) {
  if (tx < 0 || tx >= m.width || ty < 0 || ty >= m.height)
    return 1.f; // the filter reached off the map: that tap is LIT

  // ---- shadow_mode::deep: detail::shadow_visibility_deep ------------------
  // Two channels: the exact terminal depth in slot 0 (tested with the SAME
  // expression the hard path uses below, so an opaque occluder agrees with it
  // exactly), then a piecewise-linear reconstruction of accumulated alpha over
  // the knot grid.  Interpolated linearly in ALPHA against light-space DEPTH --
  // the host header carries the reasoning.
  if (m.slices > 0) {
    // PLANE-major (detail::shadow_visibility_deep carries the layout and why):
    // neighbouring threads hold neighbouring texels, so each load below is one
    // coalesced transaction per warp.
    const std::size_t plane = std::size_t(m.width) * std::size_t(m.height);
    const float *cell = m.profile + std::size_t(ty) * std::size_t(m.width) + std::size_t(tx);
    if (depth > double(cell[0]) + bias)
      return 1.f - q.shadow_strength;
    const double u = (depth - bias - m.depth_min) / m.slice_dz;
    if (!(u > 0.0)) // in front of the first knot; NaN lands here too
      return 1.f;
    float alpha;
    if (u >= double(m.slices)) {
      alpha = cell[std::size_t(m.slices) * plane];
    } else {
      const int i = int(u); // u > 0 and u < slices, so i is in [0, slices)
      const double f = u - double(i);
      // Knot 0 is implicit: nothing accumulates before the scene box.
      const double a0 = i == 0 ? 0.0 : double(cell[std::size_t(i) * plane]);
      const double a1 = double(cell[std::size_t(i + 1) * plane]);
      alpha = float(a0 + (a1 - a0) * f);
    }
    return 1.f - q.shadow_strength * alpha;
  }

  const double map_depth = double(m.depth[std::size_t(ty) * std::size_t(m.width) + tx]);
  // Inverted-NaN safe: +inf (the light ray hit nothing) never shadows.
  return depth > map_depth + bias ? (1.f - q.shadow_strength) : 1.f;
}

// detail::pcf_offset -- tap i of a k-half-width grid, as an integer texel
// offset, rounded to nearest and AWAY FROM ZERO at a half so the footprint stays
// exactly symmetric about the receiver (the host header carries why that
// matters).
__device__ inline int pcf_offset(int i, int k, double radius) {
  const double x = double(i) * radius / double(k);
  const double m = floor(fabs(x) + 0.5);
  return x < 0.0 ? -int(m) : int(m);
}

__device__ inline float shadow_visibility(const dev_request &q, const dev_shadow_map &m,
                                          const dvec &p, double n_dot_l) {
  double cos_t = fabs(n_dot_l);
  if (!(cos_t > 0.1)) // inverted: NaN floors too
    cos_t = 0.1;
  else if (cos_t > 1.0)
    cos_t = 1.0;
  const double tan_t = sqrt(1.0 - cos_t * cos_t) / cos_t;
  // The PCF widening factor is 1.0 exactly for an unfiltered render, so this is
  // the pre-PCF expression there.
  const double bias =
      q.shadow_bias_constant + q.shadow_slope_scale * m.texel_world * tan_t * q.shadow_pcf_widen;

  int ix = 0, iy = 0;
  double depth = 0.0;
  if (!shadow_project(m, p, ix, iy, depth))
    return 1.f; // outside the map: fail LIT, the non-destructive direction

  // ONE projection, many texels: the light camera is orthographic, so the
  // receiver's light-space depth is the same whichever texel it is compared
  // against.  The taps offset the MAP index, never the query point.
  const int k = q.shadow_pcf_half;
  if (k == 0)
    return shadow_tap(q, m, ix, iy, depth, bias);
  float sum = 0.f;
  for (int j = -k; j <= k; ++j) {
    const int ty = iy + pcf_offset(j, k, q.shadow_pcf_radius);
    for (int i = -k; i <= k; ++i)
      sum += shadow_tap(q, m, ix + pcf_offset(i, k, q.shadow_pcf_radius), ty, depth, bias);
  }
  return sum / float((2 * k + 1) * (2 * k + 1));
}

// Fills `vis` with one factor per light and returns it, or returns null when
// nothing casts -- which is what makes the shadows-off path bit-identical to
// the kernel before shadows existed.
__device__ inline const float *shadow_factors(const dev_request &q, const dvec &p,
                                              const dvec &normal, float *vis) {
  if (q.shadow_count == 0)
    return nullptr;
  for (int i = 0; i < q.light_count; ++i) {
    vis[i] = 1.f;
    const int m = q.light_map[i];
    if (m < 0)
      continue;
    const dev_shadow_map &sm = q.shadows[m];
    const dvec ldir = -sm.forward; // forward == -light direction
    const double n_dot_l = dot(normal, ldir);
    // A one-sided surface facing away from the light already gets neither
    // diffuse nor specular from it, so there is nothing to attenuate -- and
    // skipping the lookup deletes the whole grazing/back-facing acne class.
    if (!q.two_sided && !(n_dot_l > 0.0))
      continue;
    vis[i] = shadow_visibility(q, sm, p, n_dot_l);
  }
  return vis;
}

// baked_transfer_function::sample -- nearest entry, clamped, NaN -> entry 0.
__device__ inline void lut_sample(const dev_volume &V, float value, float out[4]) {
  if (!V.tf_active) {
    out[0] = out[1] = out[2] = out[3] = 0.f;
    return;
  }
  double t = (double(value) - V.tf_lo) * V.tf_inv_width;
  // The inverted test routes NaN (a NaN voxel in a Float volume) to entry 0
  // instead of computing an undefined index.
  if (!(t > 0.0))
    t = 0.0;
  else if (t > 1.0)
    t = 1.0;
  const int i = int(t * double(V.lut_size - 1) + 0.5);
  const float *e = V.lut + std::size_t(i) * 4;
  out[0] = e[0];
  out[1] = e[1];
  out[2] = e[2];
  out[3] = e[3];
}

// gradient_opacity_ramp::factor -- NaN magnitude maps to 0; magnitudes above
// ramp2 are cut off (the documented deviation from the legacy gradtbl).
__device__ inline float gradient_factor(const dev_volume &V, double magnitude) {
  if (!V.ramp_enabled)
    return 1.0f;
  if (!(magnitude >= V.ramp0) || magnitude > V.ramp2)
    return 0.0f;
  if (magnitude >= V.ramp1)
    return float(V.ramp_plateau);
  const double span = V.ramp1 - V.ramp0;
  if (span <= 0.0)
    return float(V.ramp_plateau);
  return float(V.ramp_plateau * (magnitude - V.ramp0) / span);
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

__device__ inline dvec to_local_point(const dev_volume &V, const dvec &p) {
  if (!V.transformed)
    return p;
  const double *m = V.w2l;
  return dv(m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3],
            m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
            m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]);
}

__device__ inline dvec to_local_vector(const dev_volume &V, const dvec &v) {
  if (!V.transformed)
    return v;
  const double *m = V.w2l;
  return dv(m[0] * v.x + m[1] * v.y + m[2] * v.z, m[4] * v.x + m[5] * v.y + m[6] * v.z,
            m[8] * v.x + m[9] * v.y + m[10] * v.z);
}

// transpose(inverse(A)) * n, using the already-inverted linear part.
__device__ inline dvec normal_to_world(const dev_volume &V, const dvec &n) {
  if (!V.transformed)
    return n;
  const double *i = V.w2l;
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
// first time accumulated alpha crosses the threshold.  CAPTURE == 1 adds the
// deep-shadow profile emission, and CAPTURE == 0 compiles to exactly the
// function that existed before deep maps -- `pc` is a null pointer no
// instruction ever reads.
template <int CAPTURE>
__device__ inline void composite(ray_accum &acc, const dev_request &q, const float c[3], float a,
                                 double t, double z_scale, prof_state *pc) {
  // The light-space depth is computed inside the capture branches, mirroring
  // raycaster.cpp: with CAPTURE == 0 nothing here is even written down.
  // t * z_scale rather than plain t so the terminal depth is the SAME
  // expression frame::depth latches -- which is what lets an opaque occluder's
  // deep lookup agree with the hard map exactly.
  if (CAPTURE && !pc->done) {
    // Every knot strictly in front of this contribution closes at the alpha
    // accumulated so far.
    const double zd = t * z_scale;
    while (pc->cursor < pc->slices && pc->z0 + double(pc->cursor + 1) * pc->dz < zd) {
      pc->out[std::size_t(1 + pc->cursor) * pc->plane] = acc.a;
      ++pc->cursor;
    }
  }
  const float pre = acc.a; // read before the update; used only when capturing
  const float ratio = a * (1.f - acc.a);
  acc.r += c[0] * ratio;
  acc.g += c[1] * ratio;
  acc.b += c[2] * ratio;
  acc.a += ratio;
  if (!acc.depth_set && acc.a >= q.depth_alpha_threshold) {
    acc.depth = float(t * z_scale);
    acc.depth_set = true;
  }
  if (CAPTURE && !pc->done && acc.a >= q.opacity_cutoff) {
    // The light ray is done here, at an EXACT depth.  Freeze the remaining
    // knots at the alpha from BEFORE this contribution: the slices describe
    // only what lies strictly in front of the terminal depth, so a receiver
    // ahead of an opaque occluder is never dimmed by the interpolation ramping
    // into the step.
    pc->out[0] = float(t * z_scale);
    while (pc->cursor < pc->slices) {
      pc->out[std::size_t(1 + pc->cursor) * pc->plane] = pre;
      ++pc->cursor;
    }
    pc->done = true;
  }
}

// Insert one hit keeping the buffer sorted by t.  DEVIATION from the CPU path,
// which keeps an unbounded per-ray hit list: the per-thread
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
//
// Returns the index the hit landed at, or `cap` when it was dropped.  The DDA
// needs that to know whether its occlusion scan cursor is still valid.
__device__ inline int push_hit(iso_hit *hits, int &n, const iso_hit &h) {
  constexpr int cap = cuda_limits::max_iso_hits_per_ray;
  int pos = n;
  while (pos > 0 && hits[pos - 1].t > h.t)
    --pos;
  if (pos >= cap)
    return cap; // buffer full and this hit is farther than everything kept
  const int last = n < cap ? n : cap - 1;
  for (int i = last; i > pos; --i)
    hits[i] = hits[i - 1];
  hits[pos] = h;
  if (n < cap)
    ++n;
  return pos;
}

template <int CAPTURE>
__device__ inline void composite_hits_up_to(ray_accum &acc, const dev_request &q,
                                            const iso_hit *hits, int nhits, int &cursor,
                                            double t_limit, double z_scale, prof_state *pc) {
  while (cursor < nhits && hits[cursor].t <= t_limit && acc.a < q.opacity_cutoff) {
    const iso_hit &h = hits[cursor++];
    // An isosurface hit latches the depth map on its own -- the frame contract
    // is "first iso hit or first threshold-crossing sample, whichever first".
    if (!acc.depth_set) {
      acc.depth = float(h.t * z_scale);
      acc.depth_set = true;
    }
    composite<CAPTURE>(acc, q, h.color, h.opacity, h.t, z_scale, pc);
  }
}

// ---------------------------------------------------------------------------
// One ray -- the device transcription of raycaster.cpp's march_ray
// ---------------------------------------------------------------------------
// Leaves the ASSOCIATED (premultiplied) accumulation and the latched depth in
// `acc`; the background over-blend belongs to the resolve, so that a pixel's
// sub-samples can be averaged after it (see the kernel).
template <int CAPTURE>
__device__ inline void march_ray(const dev_request &q, const dvec &org, const dvec &dir,
                                 ray_accum &acc, prof_state *pc) {
  acc.r = acc.g = acc.b = acc.a = 0.f;
  acc.depth_set = false;
  acc.depth = INFINITY;

  const double smin[3] = {q.scene_min.x, q.scene_min.y, q.scene_min.z};
  const double smax[3] = {q.scene_max.x, q.scene_max.y, q.scene_max.z};
  double t0 = 0.0, t1 = 0.0;
  if (!intersect_box(org, dir, smin, smax, t0, t1))
    return;

  // ---- Per-ray volume culling --------------------------------------------
  // The mirror of raycaster.cpp's active list: each volume's [t_enter, t_exit]
  // against ITS box (through the model transform) is solved once, so a march
  // step visits only the volumes whose window contains t instead of running
  // to_local_point + grid_cell_index on every volume at every step.  Pure work
  // elimination -- the cull box strictly contains grid_cell_index()'s
  // acceptance region and the window is padded by one step.
  int act[cuda_limits::max_volumes];
  double act_lo[cuda_limits::max_volumes], act_hi[cuda_limits::max_volumes];
  int nact = 0;
  for (int v = 0; v < q.nvol; ++v) {
    const dev_volume &V = q.vols[v];
    const dvec lorg = to_local_point(V, org);
    const dvec ldir = to_local_vector(V, dir); // unnormalized: t preserved
    const double clo[3] = {V.cull_lo.x, V.cull_lo.y, V.cull_lo.z};
    const double chi[3] = {V.cull_hi.x, V.cull_hi.y, V.cull_hi.z};
    double tv0 = 0.0, tv1 = 0.0;
    if (!intersect_box(lorg, ldir, clo, chi, tv0, tv1))
      continue;
    tv0 = tv0 > t0 ? tv0 : t0;
    tv1 = tv1 < t1 ? tv1 : t1;
    if (!(tv0 <= tv1))
      continue;
    act[nact] = v;
    act_lo[nact] = tv0 - q.unit_step;
    act_hi[nact] = tv1 + q.unit_step;
    ++nact;
  }

  // A ray that reaches no volume composites nothing, so the march below would
  // walk every step to produce exactly this.
  if (nact == 0)
    return;

  const dvec view_vec = -dir; // toward the viewer, unit
  const double z_scale = dot(dir, q.forward);

  // Per-light shadow visibility for whichever sample is being shaded.  32
  // bytes of per-thread local memory, against the 3232-byte frame the volume
  // trackers, the spline cache and the hit buffer already carry, and untouched
  // (shadow_factors returns null immediately) when nothing casts.
  float vis[cuda_limits::max_lights];

  spline_cache spline;
  spline.reset();

  // ---- Isosurface hits: exact per-cell ray traversal ----------------------
  // Every cell the ray actually crosses is enumerated with an Amanatides-Woo
  // DDA and MC-intersected exactly (the legacy tracer only tested cells a
  // march SAMPLE landed in -- the black-speckle artifact); the hits are then
  // merged into the compositing stream at their ray parameter.  Every volume
  // feeds the SAME t-ordered buffer, exactly as the CPU path merges all
  // volumes' hits into one stable-sorted vector.
  iso_hit hits[cuda_limits::max_iso_hits_per_ray];
  int nhits = 0;

  for (int a = 0; a < nact; ++a) {
    const int v = act[a];
    const dev_volume &V = q.vols[v];
    if (V.iso_count == 0)
      continue;
    // Recomputed rather than carried per active volume: 24 flops beats the
    // 768 bytes of extra per-thread local memory an lorg/ldir array costs.
    const dvec lorg = to_local_point(V, org);
    const dvec ldir = to_local_vector(V, dir); // unnormalized: t preserved

    const double vmin[3] = {V.minb.x, V.minb.y, V.minb.z};
    const double vmax[3] = {V.minb.x + V.span.x * double(V.dimx - 1),
                            V.minb.y + V.span.y * double(V.dimy - 1),
                            V.minb.z + V.span.z * double(V.dimz - 1)};
    double tv0 = 0.0, tv1 = 0.0;
    if (intersect_box(lorg, ldir, vmin, vmax, tv0, tv1)) {
      tv0 = tv0 > t0 ? tv0 : t0;
      tv1 = tv1 < t1 ? tv1 : t1;
      if (tv0 <= tv1) {
        // Start half a hair inside so the entry cell resolves.
        const double t_start = tv0 + (tv1 - tv0) * 1e-9;
        long long idx[3];
        if (grid_cell_index(V, lorg + ldir * t_start, idx)) {
          const double ld[3] = {ldir.x, ldir.y, ldir.z};
          const double lmin[3] = {V.minb.x, V.minb.y, V.minb.z};
          const double lspan[3] = {V.span.x, V.span.y, V.span.z};
          const long long dims[3] = {V.dimx, V.dimy, V.dimz};
          const double lorg_a[3] = {lorg.x, lorg.y, lorg.z};
          long long stepc[3];
          double t_max[3], t_delta[3];
          for (int ax = 0; ax < 3; ++ax) {
            if (ld[ax] > 0.0) {
              stepc[ax] = 1;
              t_delta[ax] = lspan[ax] / ld[ax];
              t_max[ax] = ((lmin[ax] + double(idx[ax] + 1) * lspan[ax]) - lorg_a[ax]) / ld[ax];
            } else if (ld[ax] < 0.0) {
              stepc[ax] = -1;
              t_delta[ax] = -lspan[ax] / ld[ax];
              t_max[ax] = ((lmin[ax] + double(idx[ax]) * lspan[ax]) - lorg_a[ax]) / ld[ax];
            } else {
              stepc[ax] = 0;
              t_delta[ax] = INFINITY;
              t_max[ax] = INFINITY;
            }
          }

          // ---- Occlusion cutoff for this walk ---------------------------
          // Collection and compositing are separate phases, so the march
          // loop's opacity cutoff -- which bounds VISIBLE work -- cannot bound
          // the DDA: without this the ray walks every cell of every volume
          // even when the very first surface it crossed is opaque.  `hits` is
          // kept sorted by t, so the alpha that the strictly-nearer hits will
          // already have accumulated by the time anything found from here on
          // composites is just a forward scan over that prefix, run with the
          // exact recurrence composite() uses.  Once it reaches the cutoff,
          // composite_hits_up_to would refuse every remaining hit of this
          // walk, so the rest of the walk is provably invisible.  Alpha only
          // ever grows (later volumes and transfer-function samples add to
          // it), so a prefix that saturates now still saturates then.
          int sat_cursor = 0;
          float sat_alpha = 0.f;

          const long long max_cells = dims[0] + dims[1] + dims[2] + 3;
          double t_cell = tv0;
          for (long long n = 0; n < max_cells && t_cell <= tv1; ++n) {
            // Hits found from here on have t >= t_cell, so everything already
            // recorded strictly nearer than t_cell composites ahead of them.
            while (sat_cursor < nhits && hits[sat_cursor].t < t_cell) {
              const float a = hits[sat_cursor++].opacity;
              sat_alpha += a * (1.f - sat_alpha);
            }
            if (sat_alpha >= q.opacity_cutoff)
              break;
            // Same argument for the CAP: the buffer keeps the nearest hits and
            // hits[cap-1].t only falls as this walk inserts, so once the cell
            // starts at or past it, push_hit would discard everything left.
            if (nhits == cuda_limits::max_iso_hits_per_ray &&
                t_cell >= hits[cuda_limits::max_iso_hits_per_ray - 1].t)
              break;

            float vals[8];
            grid_corners(V, idx[0], idx[1], idx[2], vals);
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
            for (int s = 0; s < V.iso_count; ++s) {
              const cuda_isosurface &surf = V.iso[s];
              if (surf.value < min_val || surf.value > max_val)
                continue;
              if (!cell_filled) {
                for (int vtx = 0; vtx < 8; ++vtx)
                  func[vtx] = vals[c_vertex_from_binary[vtx]];
                cell_filled = true;
              }
              float w[3];
              double t_hit = 0.0;
              if (!intersect_isosurface_in_cell(lorg, ldir, float(surf.value), idx, V.minb, V.span,
                                                func, w, t_hit))
                continue;
              if (t_hit < t0 || t_hit > tv1 + q.unit_step)
                continue;
              // One world hit point for the cut-plane test and the shadow
              // lookup alike.
              const dvec hit_p = org + dir * t_hit;
              if (q.plane_count > 0 && culled_by_planes(q, hit_p))
                continue;
              const dvec grad = spline.evaluate(V, v, idx, w);
              const dvec normal = dnormalized(normal_to_world(V, grad));
              iso_hit h;
              h.t = t_hit;
              // Ambient occlusion, in the volume's LOCAL frame: the cone marches
              // a signed distance field, and both the hit point and the outward
              // direction are already local here.
              float ao_vis = 1.f;
              if (V.ao_ok)
                ao_vis = 1.f - q.ao_strength * sdf_occlusion(V, lorg + ldir * t_hit,
                                                             local_outward(V, grad), surf.value,
                                                             q.ao_radius, q.ao_samples);
              float amb[3] = {q.ambient, q.ambient, q.ambient};
              if (q.ambient_shaped)
                ambient_scale(q, normal, ao_vis, amb);
              // Shaded at COLLECTION time, and visibility depends only on
              // (p, N) -- never on accumulated alpha -- so the lookup belongs
              // here, exactly as on the CPU path.
              blinn_phong(q, surf.color, normal, view_vec, amb, surf.shininess, h.color,
                          shadow_factors(q, hit_p, normal, vis));
              h.opacity = surf.opacity;
              if (push_hit(hits, nhits, h) < sat_cursor) {
                // The hit landed BEHIND the occlusion scan cursor, which the
                // walk's monotone t only allows if the cell solve returned a t
                // a hair below the cell's own entry parameter.  sat_alpha no
                // longer describes hits[0, sat_cursor), so rebuild it.
                sat_cursor = 0;
                sat_alpha = 0.f;
              }
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

  // An isosurface-only scene has nothing for the march loop to composite: its
  // body would run the window test, to_local_point and grid_cell_index at
  // every one of `steps` samples and take neither transfer-function branch.
  // Skipping straight to the tail composites the same hits in the same order
  // against the same t limit, so the frame is byte-identical.
  if (q.any_tf) {
    // One tracker per ACTIVE slot (not per scene volume): the march contributes
    // at most once per cell entered, per volume.
    long long last_cell[cuda_limits::max_volumes][3];
    for (int a = 0; a < nact; ++a) {
      last_cell[a][0] = -1;
      last_cell[a][1] = -1;
      last_cell[a][2] = -1;
    }

    // The step count is bounded by construction (unit_step = scene diagonal /
    // steps, and a ray's span inside the box never exceeds that diagonal); the
    // guard is a watchdog so a pathological request cannot hang the device.
    const int max_iterations = q.steps + 16;
    int iteration = 0;
    for (double t = t0; t <= t1 && acc.a < q.opacity_cutoff && iteration < max_iterations;
         t += q.unit_step, ++iteration) {
      composite_hits_up_to<CAPTURE>(acc, q, hits, nhits, hit_cursor, t, z_scale, pc);
      const dvec pnt = org + dir * t;
      if (q.plane_count > 0 && culled_by_planes(q, pnt))
        continue;

      for (int a = 0; a < nact; ++a) {
        // Skip the volumes whose slab window this step is outside of: the
        // to_local_point + grid_cell_index below could only miss there.
        if (t < act_lo[a] || t > act_hi[a])
          continue;
        const int v = act[a];
        const dev_volume &V = q.vols[v];

        // Sample in volume-local space (the scene-graph model transform).
        const dvec lpnt = to_local_point(V, pnt);
        long long idx[3];
        if (!grid_cell_index(V, lpnt, idx))
          continue;
        if (idx[0] == last_cell[a][0] && idx[1] == last_cell[a][1] && idx[2] == last_cell[a][2])
          continue; // one contribution per cell (the volren sampling model)
        last_cell[a][0] = idx[0];
        last_cell[a][1] = idx[1];
        last_cell[a][2] = idx[2];

        float vals[8];
        float min_val = 0.f, max_val = 0.f;
        bool have_corners = false;

        // 2. Unshaded transfer function (legacy COL_DENSITY).
        if (V.unshaded) {
          grid_corners(V, idx[0], idx[1], idx[2], vals);
          min_val = max_val = vals[0];
          for (int j = 1; j < 8; ++j) {
            // std::min/std::max semantics, not IEEE fmin/fmax -- see the DDA fold.
            min_val = vals[j] < min_val ? vals[j] : min_val;
            max_val = max_val < vals[j] ? vals[j] : max_val;
          }
          have_corners = true;
          const bool in_window = !V.window_enabled || (double(min_val) <= V.window_max &&
                                                       double(max_val) >= V.window_min);
          if (in_window) {
            float w[3];
            grid_local_weights(V, lpnt, idx[0], idx[1], idx[2], w);
            const float den = trilinear(w, vals);
            float s[4];
            lut_sample(V, den, s);
            if (s[3] > 0.f)
              composite<CAPTURE>(acc, q, s, s[3], t, z_scale, pc);
          }
        }

        // 3. Shaded transfer function (legacy RAY_CASTING).
        if (V.shaded) {
          if (!have_corners) {
            grid_corners(V, idx[0], idx[1], idx[2], vals);
            have_corners = true;
          }
          float w[3];
          grid_local_weights(V, lpnt, idx[0], idx[1], idx[2], w);
          const float den = trilinear(w, vals);
          if (V.window_enabled && (double(den) < V.window_min || double(den) > V.window_max))
            continue;
          const dvec grad = spline.evaluate(V, v, idx, w);
          float s[4];
          lut_sample(V, den, s);
          const float aa = s[3] * gradient_factor(V, sqrt(dot(grad, grad)));
          if (aa > 0.f) {
            const dvec normal = dnormalized(normal_to_world(V, grad));
            // NO ambient-occlusion cone here -- a shaded transfer-function
            // sample fires once per CELL ENTERED, and a medium with no surface
            // has no "distance to the surface".  The hemisphere still applies.
            float amb[3] = {q.ambient, q.ambient, q.ambient};
            if (q.ambient_shaped)
              ambient_scale(q, normal, 1.f, amb);
            float shaded[3];
            blinn_phong(q, s, normal, view_vec, amb, q.tf_shininess, shaded,
                        shadow_factors(q, pnt, normal, vis));
            composite<CAPTURE>(acc, q, shaded, aa, t, z_scale, pc);
          }
        }
      }
    }
  }
  // Hits between the last sample and the exit point.
  composite_hits_up_to<CAPTURE>(acc, q, hits, nhits, hit_cursor, t1, z_scale, pc);

  // The ray ended without saturating: every remaining knot sees the whole
  // accumulation.  (A ray that returned early above never got here and keeps
  // the kernel's pre-filled zeros, which is the same statement.)
  if (CAPTURE && !pc->done) {
    while (pc->cursor < pc->slices) {
      pc->out[std::size_t(1 + pc->cursor) * pc->plane] = acc.a;
      ++pc->cursor;
    }
  }
}

// ---------------------------------------------------------------------------
// The kernel
// ---------------------------------------------------------------------------
// One thread per PIXEL (not per sub-sample): a supersampled pixel marches its
// q.supersample^2 rays serially and resolves them in registers, so the frame
// buffers, the launch geometry and the per-thread footprint are all unchanged
// by supersampling -- only the loop trip count moves.  Splitting sub-samples
// across threads would need either an n^2-larger raster plus a reduction pass
// or atomics, and would put the resolve's ordering at the mercy of the
// scheduler; the renderer promises determinism.

//
// CAPTURE == 1 is the light-view instantiation that also emits the deep-shadow
// transmittance profile.  A separate instantiation rather than a runtime `if
// (q.prof_out)`: the ordinary render is the hot one, it is issue-bound, and a
// predicate inside the compositing recurrence plus live capture state would be
// paid by every scene to serve a pass that runs once per shadow rebuild.
template <int CAPTURE>
__global__ void volren_raycast_kernel(const dev_request q, unsigned char *color, float *depth) {
  const int px = int(blockIdx.x * blockDim.x + threadIdx.x);
  const int py = int(blockIdx.y * blockDim.y + threadIdx.y);
  if (px >= q.width || py >= q.height)
    return;

  const std::size_t pixel = std::size_t(py) * std::size_t(q.width) + std::size_t(px);
  unsigned char *cpx = color + pixel * 4;

  // Pre-fill this ray's payload so a ray that returns early out of march_ray --
  // missed the scene box, reached no volume -- is already correct: nothing
  // occludes it, which is zero alpha everywhere and a +inf terminal.  Mirrors
  // raycaster.cpp, which pre-fills the whole image on the host.
  prof_state pc;
  pc.out = nullptr;
  pc.plane = 0;
  pc.slices = 0;
  pc.z0 = pc.dz = 0.0;
  pc.cursor = 0;
  pc.done = false;
  if (CAPTURE) {
    pc.out = q.prof_out + pixel;
    pc.plane = std::size_t(q.width) * std::size_t(q.height);
    pc.slices = q.prof_slices;
    pc.z0 = q.prof_z0;
    pc.dz = q.prof_dz;
    pc.out[0] = INFINITY;
    for (int k = 0; k < pc.slices; ++k)
      pc.out[std::size_t(1 + k) * pc.plane] = 0.f;
  }

  // The mirror of raycaster.cpp's supersampled resolve: a REGULAR n x n grid of
  // sub-pixel offsets ((i+0.5)/n, (j+0.5)/n), an unweighted mean of the
  // sub-samples' STRAIGHT (background-over-blended, not premultiplied) RGBA,
  // and the NEAREST finite depth rather than an averaged one.  raycaster.cpp
  // carries the reasoning for all three; the arithmetic here is written to
  // match it expression for expression.
  const int ss = q.supersample;
  // In DOUBLE, matching raycaster.cpp: --use_fast_math would turn a float
  // 1.f/9.f into an approximate reciprocal and hand the resolve a divisor the
  // CPU never used, for no speed anywhere that matters (once per pixel).
  const float inv_samples = float(1.0 / double(ss * ss));
  float sum_r = 0.f, sum_g = 0.f, sum_b = 0.f, sum_a = 0.f;
  float nearest = INFINITY;

  for (int sj = 0; sj < ss; ++sj)
    for (int si = 0; si < ss; ++si) {
      // ray_generator::at -- NDC through the sub-sample, v = +1 at the TOP row.
      // With ss == 1 the offset is 0.5 exactly, so this is the pixel center.
      const double u = (double(px) + (double(si) + 0.5) / double(ss)) / double(q.width) * 2.0 - 1.0;
      const double v =
          1.0 - (double(py) + (double(sj) + 0.5) / double(ss)) / double(q.height) * 2.0;
      dvec org, dir;
      if (q.perspective) {
        org = q.eye;
        dir = dnormalized(q.forward + q.right * (u * q.tan_half * q.aspect) +
                          q.true_up * (v * q.tan_half));
      } else {
        org = q.eye + q.right * (u * q.parallel_scale * q.aspect) +
              q.true_up * (v * q.parallel_scale);
        dir = q.forward;
      }

      ray_accum acc;
      march_ray<CAPTURE>(q, org, dir, acc, &pc);

      // Over-blend the remaining transparency with the background (replaces the
      // legacy divide-by-alpha normalization of saturated rays), then
      // accumulate the resolved straight RGBA.
      const float rest = 1.f - acc.a;
      sum_r += acc.r + q.background[0] * rest;
      sum_g += acc.g + q.background[1] * rest;
      sum_b += acc.b + q.background[2] * rest;
      sum_a += acc.a;
      if (acc.depth < nearest)
        nearest = acc.depth;
    }

  depth[pixel] = nearest;
  cpx[0] = to_byte(sum_r * inv_samples);
  cpx[1] = to_byte(sum_g * inv_samples);
  cpx[2] = to_byte(sum_b * inv_samples);
  cpx[3] = to_byte(sum_a * inv_samples);
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

// ---------------------------------------------------------------------------
// Resident device volume cache
// ---------------------------------------------------------------------------
// Voxels are the only thing a render uploads that is both large and usually
// unchanged, so they stay on the device between renders: a camera-only change
// re-launches with zero H2D traffic.
//
// INVALIDATION RULE (also stated in raycaster_cuda.h):
//   1. An entry is keyed on (device, host base pointer, byte length) and holds
//      the caller's voxels::active_storage() handle, so the host block cannot
//      be freed while cached and its address cannot be recycled by a different
//      volume -- the classic free/re-malloc-at-the-same-address staleness is
//      structurally impossible rather than merely unlikely.
//   2. Because the cache CO-OWNS the block, every write through the supported
//      cvc::volume API copy-on-writes to a different block (voxels::preWrite
//      sees a non-unique buffer), which lands on a different key: a mutated
//      volume is a cache miss by construction.
//   3. The remaining hole is an in-place write through the unchecked legacy
//      escape hatch voxels::data_ptr(), which no copy-on-write can see.  The
//      owner announces those with raycaster::invalidate_device_volume(), which
//      bumps cuda_volume::generation; a generation mismatch RETIRES the block
//      and uploads a fresh one.  (Retire rather than rewrite in place: another
//      render may be mid-kernel on that block.)  Two raycasters that disagree
//      about a shared buffer's generation simply re-upload on alternate
//      renders -- always current, never stale.
// Eviction is least-recently-used against a byte budget; a block an in-flight
// render is using is never freed, and the lease that marks it in-use is
// released even when the render throws.
struct cache_key {
  int device = 0;
  const void *host = nullptr;
  std::size_t bytes = 0;

  bool operator<(const cache_key &o) const {
    if (device != o.device)
      return device < o.device;
    if (host != o.host)
      return std::less<const void *>()(host, o.host);
    return bytes < o.bytes;
  }
};

// One device allocation.  Identified by a serial number rather than by its key
// so a superseded block can outlive the key that used to name it: a render
// already reading it keeps it alive until its lease drops.
struct cache_block {
  cache_key key;
  void *dptr = nullptr;
  std::size_t bytes = 0;
  std::shared_ptr<void> pin; // co-owner of the host block -- see rule 1/2
  std::uint64_t generation = 0;
  std::uint64_t used_at = 0;
  int in_flight = 0;
  bool current = false; // still the block `key` resolves to
};

class volume_cache {
public:
  static volume_cache &instance() {
    static volume_cache c;
    return c;
  }

  // Device pointer holding the bytes of `key.host`, uploading only when the
  // block is not resident or its generation moved.  Returns the block id; the
  // caller must release() it.
  std::uint64_t acquire(const cache_key &key, const std::shared_ptr<void> &pin,
                        std::uint64_t generation, const unsigned char *&dev) {
    std::lock_guard<std::mutex> lock(_mutex);

    auto indexed = _index.find(key);
    if (indexed != _index.end()) {
      cache_block &b = _blocks.at(indexed->second);
      if (b.generation == generation) {
        b.in_flight++;
        b.used_at = ++_clock;
        dev = static_cast<const unsigned char *>(b.dptr);
        return indexed->second;
      }
      // Stale.  Retire rather than overwrite in place: another render may be
      // mid-kernel on this very block, and rewriting its bytes underneath a
      // live launch would be a data race for the sake of saving one
      // allocation on a path that only runs when a volume actually changed.
      retire_locked(indexed);
    }

    evict_locked(key.bytes);
    void *dptr = nullptr;
    cudaError_t err = cudaMalloc(&dptr, key.bytes);
    if (err != cudaSuccess) {
      // Out of device memory: drop everything not in flight and try once more
      // before giving up (the caller degrades to the CPU march).
      (void)cudaGetLastError();
      evict_locked(std::size_t(-1));
      CUDA_CHECK(cudaMalloc(&dptr, key.bytes));
    }
    try {
      CUDA_CHECK(cudaMemcpy(dptr, key.host, key.bytes, cudaMemcpyHostToDevice));
    } catch (...) {
      // Nothing is recorded, so a failed upload leaves no half-written block
      // behind for the next render to trust.
      cudaFree(dptr);
      throw;
    }
    _uploaded += key.bytes;

    const std::uint64_t id = ++_next_id;
    cache_block b;
    b.key = key;
    b.dptr = dptr;
    b.bytes = key.bytes;
    b.pin = pin;
    b.generation = generation;
    b.used_at = ++_clock;
    b.in_flight = 1;
    b.current = true;
    _resident += key.bytes;
    _blocks.emplace(id, std::move(b));
    _index[key] = id;
    dev = static_cast<const unsigned char *>(dptr);
    return id;
  }

  void release(std::uint64_t id) {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _blocks.find(id);
    if (it == _blocks.end())
      return;
    if (it->second.in_flight > 0)
      it->second.in_flight--;
    if (it->second.in_flight == 0 && !it->second.current)
      free_locked(it);
  }

  void clear() {
    std::lock_guard<std::mutex> lock(_mutex);
    evict_locked(std::size_t(-1));
  }

  void set_budget(std::size_t bytes) {
    std::lock_guard<std::mutex> lock(_mutex);
    _budget = bytes;
    evict_locked(0);
  }
  std::size_t budget() {
    std::lock_guard<std::mutex> lock(_mutex);
    return _budget;
  }
  std::size_t resident() {
    std::lock_guard<std::mutex> lock(_mutex);
    return _resident;
  }
  std::uint64_t uploaded() {
    std::lock_guard<std::mutex> lock(_mutex);
    return _uploaded;
  }

private:
  volume_cache() = default;
  volume_cache(const volume_cache &) = delete;
  volume_cache &operator=(const volume_cache &) = delete;

  // Frees every device block at process teardown.  Errors are swallowed on
  // purpose: by the time a function-local static is destroyed the primary
  // context may already be gone, and cudaFree then reports a failure that
  // nothing can act on.
  ~volume_cache() {
    for (auto &kv : _blocks)
      cudaFree(kv.second.dptr);
    (void)cudaGetLastError();
  }

  void free_locked(std::map<std::uint64_t, cache_block>::iterator it) {
    cudaFree(it->second.dptr);
    _resident -= it->second.bytes;
    _blocks.erase(it);
  }

  // Unname a block: it stops answering lookups immediately, and its memory
  // goes back as soon as the last render using it releases.
public:
  // Retire every entry staged from `host`, whatever its length or device.
  // Used when the owner is about to drop the host buffer itself (a rebuilt
  // shadow raster), where the usual same-key/new-generation retire cannot
  // fire because the replacement lands at a different address.
  void forget_host(const void *host) {
    if (host == nullptr)
      return;
    std::lock_guard<std::mutex> lock(_mutex);
    for (auto it = _index.begin(); it != _index.end();) {
      if (it->first.host == host) {
        auto victim = it++;
        retire_locked(victim);
      } else {
        ++it;
      }
    }
  }

private:
  void retire_locked(std::map<cache_key, std::uint64_t>::iterator indexed) {
    auto it = _blocks.find(indexed->second);
    _index.erase(indexed);
    if (it == _blocks.end())
      return;
    it->second.current = false;
    if (it->second.in_flight == 0)
      free_locked(it);
  }

  // Free least-recently-used blocks until `need` more bytes fit under the
  // budget.  `need == size_t(-1)` means "free everything free-able".  Blocks
  // an in-flight render is using are never freed -- exceeding the budget is
  // strictly better than pulling a buffer out from under a live kernel.
  void evict_locked(std::size_t need) {
    const bool drop_all = need == std::size_t(-1);
    for (;;) {
      if (!drop_all && _resident + need <= _budget)
        return;
      auto victim = _blocks.end();
      for (auto it = _blocks.begin(); it != _blocks.end(); ++it) {
        if (it->second.in_flight > 0)
          continue;
        if (victim == _blocks.end() || it->second.used_at < victim->second.used_at)
          victim = it;
      }
      if (victim == _blocks.end())
        return; // nothing evictable
      if (victim->second.current)
        _index.erase(victim->second.key);
      free_locked(victim);
    }
  }

  std::mutex _mutex;
  std::map<std::uint64_t, cache_block> _blocks; // every live allocation, by id
  std::map<cache_key, std::uint64_t> _index;    // the CURRENT block per key
  std::size_t _resident = 0;
  std::size_t _budget = cuda_limits::default_cache_bytes;
  std::uint64_t _clock = 0;
  std::uint64_t _next_id = 0;
  std::uint64_t _uploaded = 0; // voxel bytes ever pushed H2D
};

// Marks the blocks one render depends on as in-flight and releases them on the
// way out -- including when the render throws partway through staging.
class cache_lease {
public:
  cache_lease() = default;
  cache_lease(const cache_lease &) = delete;
  cache_lease &operator=(const cache_lease &) = delete;
  ~cache_lease() {
    for (std::uint64_t id : _held)
      volume_cache::instance().release(id);
  }

  const unsigned char *acquire(const cache_key &key, const std::shared_ptr<void> &pin,
                               std::uint64_t generation) {
    const unsigned char *dev = nullptr;
    const std::uint64_t id = volume_cache::instance().acquire(key, pin, generation, dev);
    _held.push_back(id);
    return dev;
  }

private:
  std::vector<std::uint64_t> _held;
};

} // namespace

bool raycast_cuda_available() {
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

frame raycast_cuda(const raycast_cuda_request &req) {
  // Validate everything BEFORE the first allocation (the drive.cu convention),
  // so an invalid request never leaks device memory.
  if (req.volume_count < 1)
    throw volren_error("raycast_cuda: no volumes in the request");
  if (req.volume_count > cuda_limits::max_volumes)
    throw volren_error("raycast_cuda: more volumes than cuda_limits::max_volumes");
  if (req.steps < 1)
    throw volren_error("raycast_cuda: steps must be >= 1");
  if (!(req.unit_step > 0.0) || !std::isfinite(req.unit_step))
    throw volren_error("raycast_cuda: unit_step must be positive and finite");
  if (!(req.opacity_cutoff > 0.f) || req.opacity_cutoff > 1.f)
    throw volren_error("raycast_cuda: opacity_cutoff must be in (0, 1]");
  if (req.supersample < 1 || req.supersample > limits::max_supersample)
    throw volren_error("raycast_cuda: supersample must be in [1, limits::max_supersample]");
  if (req.light_count < 0 || req.light_count > cuda_limits::max_lights)
    throw volren_error("raycast_cuda: too many lights for the CUDA backend");
  if (req.cut_plane_count < 0 || req.cut_plane_count > cuda_limits::max_cut_planes)
    throw volren_error("raycast_cuda: too many cut planes for the CUDA backend");
  if (req.shadow_map_count < 0 || req.shadow_map_count > cuda_limits::max_shadow_maps)
    throw volren_error("raycast_cuda: more shadow maps than cuda_limits::max_shadow_maps");
  for (int i = 0; i < req.shadow_map_count; ++i) {
    const cuda_shadow_map &m = req.shadow_maps[i];
    if (!m.depth || m.width < 1 || m.height < 1)
      throw volren_error("raycast_cuda: a shadow map needs a depth raster and positive dimensions");
    if (!(m.parallel_scale > 0.0) || !std::isfinite(m.parallel_scale))
      throw volren_error("raycast_cuda: shadow map parallel_scale must be positive and finite");
    if (m.light_index < 0 || m.light_index >= req.light_count)
      throw volren_error("raycast_cuda: a shadow map names a light that is not in the request");
    if (m.slices > 0) {
      if (m.slices > limits::max_shadow_depth_slices)
        throw volren_error("raycast_cuda: more depth slices than limits::max_shadow_depth_slices");
      if (!m.profile)
        throw volren_error("raycast_cuda: a deep shadow map needs a transmittance profile");
      if (!(m.slice_dz > 0.0) || !std::isfinite(m.slice_dz) || !std::isfinite(m.depth_min))
        throw volren_error("raycast_cuda: deep shadow map knot grid must be positive and finite");
    }
  }

  // Deep-shadow profile CAPTURE (the light-view pass).  A ray parameter is a
  // light-space depth only under an orthographic camera with one ray per
  // texel, so anything else is refused rather than silently mis-binned -- the
  // caller falls back to the CPU marcher, which produces the same data.
  if (req.profile_slices < 0 || req.profile_slices > limits::max_shadow_depth_slices)
    throw volren_error("raycast_cuda: profile_slices out of range");
  if (req.profile_slices > 0) {
    if (!req.profile_out)
      throw volren_error("raycast_cuda: profile capture needs an output buffer");
    if (req.supersample != 1)
      throw volren_error("raycast_cuda: profile capture requires supersample == 1");
    if (req.cam.projection != camera::projection_type::orthographic)
      throw volren_error("raycast_cuda: profile capture requires an orthographic camera");
    if (!(req.profile_dz > 0.0) || !std::isfinite(req.profile_dz) ||
        !std::isfinite(req.profile_z_near))
      throw volren_error("raycast_cuda: profile knot grid must be positive and finite");
  }

  std::size_t vsize[cuda_limits::max_volumes];
  std::size_t volume_bytes[cuda_limits::max_volumes];
  std::size_t lut_offset[cuda_limits::max_volumes];
  std::size_t lut_floats = 0;
  for (int v = 0; v < req.volume_count; ++v) {
    const cuda_volume &s = req.volumes[v];
    if (!s.data)
      throw volren_error("raycast_cuda: null volume buffer");
    if (s.dimx < 2 || s.dimy < 2 || s.dimz < 2)
      throw volren_error("raycast_cuda: volumes need at least 2 voxels per axis");
    if (!(s.span[0] > 0.0) || !(s.span[1] > 0.0) || !(s.span[2] > 0.0))
      throw volren_error("raycast_cuda: volume span must be positive on every axis");
    vsize[v] = voxel_size(s.type);
    if (vsize[v] == 0)
      throw volren_error("raycast_cuda: unsupported cvc::data_type");
    if (s.isosurface_count < 0 || s.isosurface_count > cuda_limits::max_isosurfaces)
      throw volren_error("raycast_cuda: too many isosurfaces for the CUDA backend");
    if (s.tf_active && (!s.lut || s.lut_size < 2))
      throw volren_error("raycast_cuda: an active transfer function needs a LUT of >= 2 entries");
    volume_bytes[v] = std::size_t(s.dimx) * std::size_t(s.dimy) * std::size_t(s.dimz) * vsize[v];
    lut_offset[v] = lut_floats;
    if (s.tf_active)
      lut_floats += std::size_t(s.lut_size) * 4;
  }

  // basis() carries all the camera validation (raster, fov, degenerate pose)
  // and throws cvc::volren_error, matching the CPU entry point.
  const view_basis basis = req.cam.basis();
  const int width = req.cam.width;
  const int height = req.cam.height;

  ensure_mc_tables();

  int device = 0;
  CUDA_CHECK(cudaGetDevice(&device));

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
  q.nvol = req.volume_count;

  q.light_count = req.light_count;
  for (int i = 0; i < req.light_count; ++i)
    q.lights[i] = req.lights[i];
  q.plane_count = req.cut_plane_count;
  for (int i = 0; i < req.cut_plane_count; ++i)
    q.planes[i] = req.cut_planes[i];

  q.shadow_count = req.shadow_map_count;
  q.shadow_strength = req.shadow_strength;
  q.shadow_bias_constant = req.shadow_bias_constant;
  q.shadow_slope_scale = req.shadow_slope_scale;
  q.shadow_pcf_half = req.shadow_pcf_half;
  q.shadow_pcf_radius = req.shadow_pcf_radius;
  q.shadow_pcf_widen = req.shadow_pcf_widen;
  for (int i = 0; i < cuda_limits::max_lights; ++i)
    q.light_map[i] = -1;
  for (int i = 0; i < req.shadow_map_count; ++i) {
    const cuda_shadow_map &m = req.shadow_maps[i];
    dev_shadow_map &d = q.shadows[i];
    d.eye = {m.eye[0], m.eye[1], m.eye[2]};
    d.right = {m.right[0], m.right[1], m.right[2]};
    d.up = {m.up[0], m.up[1], m.up[2]};
    d.forward = {m.forward[0], m.forward[1], m.forward[2]};
    d.parallel_scale = m.parallel_scale;
    d.texel_world = m.texel_world;
    d.width = m.width;
    d.height = m.height;
    d.depth = nullptr; // patched once the rasters are staged
    d.slices = m.slices;
    d.depth_min = m.depth_min;
    d.slice_dz = m.slice_dz;
    d.profile = nullptr; // ditto
    // A repeated light index would be a caller bug; last one wins, matching
    // the CPU path's map_of_light assignment.
    q.light_map[m.light_index] = i;
  }

  q.scene_min = {req.scene_min[0], req.scene_min[1], req.scene_min[2]};
  q.scene_max = {req.scene_max[0], req.scene_max[1], req.scene_max[2]};
  q.steps = req.steps;
  q.unit_step = req.unit_step;
  q.opacity_cutoff = req.opacity_cutoff;
  q.depth_alpha_threshold = req.depth_alpha_threshold;
  q.ambient = req.ambient;
  q.two_sided = req.two_sided ? 1 : 0;
  q.ambient_shaped = req.ambient_shaped ? 1 : 0;
  q.ambient_hemisphere = req.ambient_hemisphere ? 1 : 0;
  for (int i = 0; i < 3; ++i) {
    q.ambient_sky[i] = req.ambient_sky[i];
    q.ambient_ground[i] = req.ambient_ground[i];
  }
  q.ambient_up = {req.ambient_up[0], req.ambient_up[1], req.ambient_up[2]};
  q.ao_strength = req.ao_strength;
  q.ao_radius = req.ao_radius;
  q.ao_samples = req.ao_samples;
  q.shading_gain = req.shading_gain;
  q.specular = req.specular;
  q.background[0] = req.background[0];
  q.background[1] = req.background[1];
  q.background[2] = req.background[2];
  q.tf_shininess = req.tf_shininess;
  q.supersample = req.supersample;
  // Resolved once here rather than per ray: an isosurface-only scene skips the
  // whole march loop on the device (see march_ray).
  q.any_tf = 0;
  for (int v = 0; v < req.volume_count; ++v)
    if (req.volumes[v].shaded || req.volumes[v].unshaded)
      q.any_tf = 1;
  q.prof_out = nullptr; // patched once the output buffer is allocated
  q.prof_slices = req.profile_slices;
  q.prof_z0 = req.profile_z_near;
  q.prof_dz = req.profile_dz;

  // ---- staging -----------------------------------------------------------
  // Voxels come from the resident cache (usually no H2D at all); the LUTs are
  // small, change with the settings, and are staged contiguously so the whole
  // scene's tables cost ONE copy.  The lease releases the cache entries even
  // if anything below throws, and the transient buffers are RAII.
  cache_lease lease;
  std::vector<dev_volume> dv(req.volume_count);
  std::vector<float> lut_host(lut_floats);

  for (int v = 0; v < req.volume_count; ++v) {
    const cuda_volume &s = req.volumes[v];
    dev_volume &d = dv[v];
    std::memset(&d, 0, sizeof(d));

    cache_key key;
    key.device = device;
    key.host = s.data;
    key.bytes = volume_bytes[v];
    d.data = lease.acquire(key, s.pin, s.generation);

    d.type = int(s.type);
    d.dimx = s.dimx;
    d.dimy = s.dimy;
    d.dimz = s.dimz;
    d.minb = {s.minb[0], s.minb[1], s.minb[2]};
    d.span = {s.span[0], s.span[1], s.span[2]};
    // The cull box: grid_cell_index() accepts local x in
    // (minb - span, minb + span * (dim-1)); one extra voxel per face makes the
    // per-ray rejection immune to face-vs-point rounding.
    d.cull_lo = {s.minb[0] - 2.0 * s.span[0], s.minb[1] - 2.0 * s.span[1],
                 s.minb[2] - 2.0 * s.span[2]};
    d.cull_hi = {s.minb[0] + s.span[0] * double(s.dimx), s.minb[1] + s.span[1] * double(s.dimy),
                 s.minb[2] + s.span[2] * double(s.dimz)};

    d.transformed = s.transformed ? 1 : 0;
    for (int i = 0; i < 16; ++i)
      d.w2l[i] = s.world_to_local[i];

    d.tf_active = s.tf_active ? 1 : 0;
    d.lut_size = s.lut_size;
    d.tf_lo = s.tf_lo;
    d.tf_inv_width = s.tf_hi > s.tf_lo ? 1.0 / (s.tf_hi - s.tf_lo) : 1.0;
    if (s.tf_active)
      std::memcpy(lut_host.data() + lut_offset[v], s.lut,
                  std::size_t(s.lut_size) * 4 * sizeof(float));

    d.ramp_enabled = s.gradient_ramp_enabled ? 1 : 0;
    d.ramp0 = s.ramp0;
    d.ramp1 = s.ramp1;
    d.ramp2 = s.ramp2;
    d.ramp_plateau = s.gradient_plateau;

    d.iso_count = s.isosurface_count;
    for (int i = 0; i < s.isosurface_count; ++i)
      d.iso[i] = s.isosurfaces[i];

    d.shaded = s.shaded ? 1 : 0;
    d.unshaded = s.unshaded ? 1 : 0;
    d.window_enabled = s.window_enabled ? 1 : 0;
    d.window_min = s.window_min;
    d.window_max = s.window_max;
    d.ao_ok = s.ao_ok ? 1 : 0;
  }

  const std::size_t color_bytes = std::size_t(width) * std::size_t(height) * 4;
  const std::size_t depth_bytes = std::size_t(width) * std::size_t(height) * sizeof(float);

  device_buffer d_lut, d_vols, d_color, d_depth, d_prof_out;
  // Shadow rasters ride the RESIDENT block cache, exactly like voxels: the maps
  // are rebuilt only when the light pass's own inputs change, so a steady-state
  // frame (all camera motion) must not re-stage them.  That was a defensible
  // 1 MB per map while a map was one depth raster; a deep map's profile is
  // 17.8 MB at the defaults and re-staging it per frame measured at +6.7 ms
  // against a 6 ms pass.  The lease releases the entries even if anything below
  // throws.
  for (int i = 0; i < req.shadow_map_count; ++i) {
    const cuda_shadow_map &m = req.shadow_maps[i];
    const std::size_t texels = std::size_t(m.width) * std::size_t(m.height);
    cache_key dk;
    dk.device = device;
    dk.host = m.depth;
    dk.bytes = texels * sizeof(float);
    q.shadows[i].depth = reinterpret_cast<const float *>(lease.acquire(dk, m.pin, m.generation));
    if (m.slices > 0) {
      cache_key pk;
      pk.device = device;
      pk.host = m.profile;
      pk.bytes = texels * std::size_t(m.slices + 1) * sizeof(float);
      q.shadows[i].profile =
          reinterpret_cast<const float *>(lease.acquire(pk, m.pin, m.generation));
    }
  }

  if (lut_floats > 0) {
    d_lut.alloc(lut_floats * sizeof(float));
    CUDA_CHECK(
        cudaMemcpy(d_lut.p, lut_host.data(), lut_floats * sizeof(float), cudaMemcpyHostToDevice));
    for (int v = 0; v < req.volume_count; ++v)
      if (dv[v].tf_active)
        dv[v].lut = static_cast<const float *>(d_lut.p) + lut_offset[v];
  }

  d_vols.alloc(std::size_t(req.volume_count) * sizeof(dev_volume));
  CUDA_CHECK(cudaMemcpy(d_vols.p, dv.data(), std::size_t(req.volume_count) * sizeof(dev_volume),
                        cudaMemcpyHostToDevice));
  q.vols = static_cast<const dev_volume *>(d_vols.p);

  d_color.alloc(color_bytes);
  d_depth.alloc(depth_bytes);

  const std::size_t prof_out_floats =
      req.profile_slices > 0
          ? std::size_t(width) * std::size_t(height) * std::size_t(req.profile_slices + 1)
          : 0;
  if (prof_out_floats > 0) {
    d_prof_out.alloc(prof_out_floats * sizeof(float));
    q.prof_out = static_cast<float *>(d_prof_out.p);
  }

  // One thread per pixel.  Default stream only: nothing else is in flight.
  const dim3 threads(16, 16);
  const dim3 blocks((unsigned(width) + threads.x - 1) / threads.x,
                    (unsigned(height) + threads.y - 1) / threads.y);
  // Clear any error left on this thread by an unrelated earlier CUDA call:
  // cudaGetLastError() reports the last error since it was last READ, so
  // without this a failure some other libcvc code already handled would be
  // attributed to our launch and force a spurious CPU fallback.
  (void)cudaGetLastError();
  // The capture instantiation is a DIFFERENT kernel, so the ordinary render
  // launches exactly the code it launched before deep shadows existed.
  if (req.profile_slices > 0)
    volren_raycast_kernel<1><<<blocks, threads>>>(q, static_cast<unsigned char *>(d_color.p),
                                                  static_cast<float *>(d_depth.p));
  else
    volren_raycast_kernel<0><<<blocks, threads>>>(q, static_cast<unsigned char *>(d_color.p),
                                                  static_cast<float *>(d_depth.p));
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  frame out;
  out.color = cvc::image(width, height, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  out.depth = cvc::image(width, height, cvc::image::pixel_format::GRAY, cvc::image::data_type::f32);
  CUDA_CHECK(cudaMemcpy(out.color.data(), d_color.p, color_bytes, cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(out.depth.data(), d_depth.p, depth_bytes, cudaMemcpyDeviceToHost));
  if (prof_out_floats > 0)
    CUDA_CHECK(cudaMemcpy(req.profile_out, d_prof_out.p, prof_out_floats * sizeof(float),
                          cudaMemcpyDeviceToHost));
  return out;
}

void raycast_cuda_set_cache_budget(std::size_t bytes) {
  volume_cache::instance().set_budget(bytes);
}

std::size_t raycast_cuda_cache_budget() { return volume_cache::instance().budget(); }

std::size_t raycast_cuda_cache_bytes() { return volume_cache::instance().resident(); }

std::uint64_t raycast_cuda_cache_upload_bytes() { return volume_cache::instance().uploaded(); }

void raycast_cuda_forget_host_buffer(const void *host) {
  volume_cache::instance().forget_host(host);
}

void raycast_cuda_clear_cache() { volume_cache::instance().clear(); }

} // namespace volren
} // namespace cvc
