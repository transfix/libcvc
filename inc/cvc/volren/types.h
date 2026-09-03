// cvc::volren -- software raycast volume renderer.
//
// Modern C++ port of volrover's `volren` library (with the ray/isosurface
// intersection subset of `libiso` absorbed as an internal detail).  See
// docs/VOLREN_API.md for the full design and the fidelity notes versus the
// legacy C implementation.
#ifndef CVC_VOLREN_TYPES_H
#define CVC_VOLREN_TYPES_H

#include <array>
#include <cmath>
#include <cstddef>
#include <cvc/core/exception.h>

namespace cvc {

CVC_DEF_EXCEPTION(volren_error);

namespace volren {

// Small double-precision vector used throughout the ray-march.  World
// coordinates in libcvc are doubles (cvc::bounding_box), so the traversal is
// double; sample values stay float like the legacy renderer.
struct vec3d {
  double x = 0.0, y = 0.0, z = 0.0;

  constexpr vec3d() = default;
  constexpr vec3d(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  explicit constexpr vec3d(const std::array<double, 3> &a) : x(a[0]), y(a[1]), z(a[2]) {}

  constexpr std::array<double, 3> to_array() const { return {x, y, z}; }

  constexpr vec3d operator+(const vec3d &o) const { return {x + o.x, y + o.y, z + o.z}; }
  constexpr vec3d operator-(const vec3d &o) const { return {x - o.x, y - o.y, z - o.z}; }
  constexpr vec3d operator*(double s) const { return {x * s, y * s, z * s}; }
  constexpr vec3d operator-() const { return {-x, -y, -z}; }
  vec3d &operator+=(const vec3d &o) {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }
};

constexpr double dot(const vec3d &a, const vec3d &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr vec3d cross(const vec3d &a, const vec3d &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double length(const vec3d &v) { return std::sqrt(dot(v, v)); }

// Normalized copy; a vector shorter than `epsilon` normalizes to zero (the
// legacy vrNormalize contract, which shading relies on for flat regions).
inline vec3d normalized(const vec3d &v, double epsilon = 1e-12) {
  const double len = length(v);
  return len <= epsilon ? vec3d{} : vec3d{v.x / len, v.y / len, v.z / len};
}

// Linear RGBA color, channels in [0,1].
struct rgba_f {
  float r = 0.f, g = 0.f, b = 0.f, a = 0.f;
};

// Affine 4x4 transform, ROW-MAJOR storage, points as column vectors
// (p' = M * p) -- exactly the cvcGL convention (GraphicsNode::setTransform's
// row-major double[16], translation in column 3), so a scene-graph node's
// composed world matrix feeds a volren volume verbatim.
struct mat4 {
  // Row-major: m[row * 4 + col].  Defaults to identity.
  std::array<double, 16> m{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  static mat4 identity() { return mat4(); }
  static mat4 from_row_major(const double values[16]) {
    mat4 out;
    for (int i = 0; i < 16; ++i)
      out.m[i] = values[i];
    return out;
  }

  bool is_identity() const {
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        if (m[r * 4 + c] != (r == c ? 1.0 : 0.0))
          return false;
    return true;
  }

  bool is_affine(double epsilon = 1e-12) const {
    return std::fabs(m[12]) <= epsilon && std::fabs(m[13]) <= epsilon &&
           std::fabs(m[14]) <= epsilon && std::fabs(m[15] - 1.0) <= epsilon;
  }

  vec3d transform_point(const vec3d &p) const {
    return {m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3],
            m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
            m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]};
  }

  // Linear part only (directions ignore translation).
  vec3d transform_vector(const vec3d &v) const {
    return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[4] * v.x + m[5] * v.y + m[6] * v.z,
            m[8] * v.x + m[9] * v.y + m[10] * v.z};
  }

  // Composition: (a * b) applies b first, then a (column-vector convention).
  mat4 operator*(const mat4 &b) const {
    mat4 out;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c) {
        double sum = 0.0;
        for (int k = 0; k < 4; ++k)
          sum += m[r * 4 + k] * b.m[k * 4 + c];
        out.m[r * 4 + c] = sum;
      }
    return out;
  }

  // Inverse of an affine transform.  Throws cvc::volren_error when the matrix
  // is not affine or its linear part is singular.
  mat4 affine_inverse() const;

  // transpose(inverse(linear part)) applied to v -- the normal transform.
  vec3d transform_normal(const vec3d &v) const;
};

// Every tunable the legacy renderer hid in a #define, as a typed constant.
namespace defaults {
// Early-ray-termination threshold on accumulated opacity (legacy THRESHOLD_OPC).
inline constexpr float opacity_cutoff = 0.95f;
// Accumulated-alpha level at which the depth map latches a ray's depth.
inline constexpr float depth_alpha_threshold = 0.5f;
// Samples along the scene bounding-box diagonal (legacy step_size).
inline constexpr int steps = 512;
// Entries in a baked transfer-function lookup table (legacy: max_dens).
inline constexpr std::size_t lut_size = 1024;
// Tile edge in pixels for parallel rendering (legacy TILE_SIZE; rasters are
// no longer forced to multiples of it -- partial tiles render exactly).
inline constexpr int tile_size = 32;
// Output gain applied after shading (the legacy 0.9f damping in vrPhongShading).
inline constexpr float shading_gain = 0.9f;
// Plateau of the gradient-magnitude opacity ramp (legacy gradtbl peak).
inline constexpr double gradient_plateau = 0.9;
// Blinn-Phong specular exponent.  The legacy code hardcoded shining=15 but
// shaded through a baked x^10 table; 10 reproduces what it actually rendered.
inline constexpr float shininess = 10.0f;
// Vertical field of view in degrees (matches cvcGL/vtkCamera's default).
inline constexpr double vfov_degrees = 30.0;
// Sub-samples per pixel EDGE (render_settings::supersample).  1 is one ray
// through the pixel center -- the historical behavior, kept as the default
// because supersampling multiplies the ray count by its square.
inline constexpr int supersample = 1;
// Light-view shadow-map raster edge (shadow_settings::resolution).  512^2 is
// 262k rays, the same order as one main pass at cvcGL VolRenNode's default
// resolution_scale -- and the map is CACHED across camera motion, so it is
// paid once per scene change, not once per frame.
inline constexpr int shadow_resolution = 512;
// Constant shadow bias, in units of the light-view depth latch's own quantum
// (detail::shadow_bias -- the march step for an exact isosurface intersection,
// two cells for a transfer-function latch).  1.0 is the measured bound.
inline constexpr float shadow_bias_scale = 1.0f;
// Slope-scaled shadow bias in units of texel_world * tan(theta).
inline constexpr float shadow_slope_scale = 1.0f;
// Minimum isosurface opacity for that surface to cast a shadow.
inline constexpr float shadow_min_occluder_opacity = 0.5f;
} // namespace defaults

namespace limits {
// Largest raster edge render() accepts.  Keeps tile/pixel index arithmetic
// comfortably inside int range and rejects nonsense before allocating a
// multi-terabyte frame.
inline constexpr int max_raster_dim = 65536;
// Largest render_settings::supersample.  4 already means 16 rays per pixel;
// beyond that a larger raster buys strictly more information for the same rays
// (supersampling only refines what one output pixel averages), so the cap is a
// contract rather than an implementation limit -- neither backend carries
// per-sub-sample state, and both enforce this same range.
inline constexpr int max_supersample = 4;
// Shadow-casting lights per scene.  Each one costs a full extra render pass,
// and the CUDA kernel carries the maps' light-view frames by value in its
// parameter block (cuda_limits::max_shadow_maps must match this).
inline constexpr int max_shadow_maps = 4;
// Smallest light-view raster edge shadow_settings::resolution clamps up to.
// Below this the texel footprint is coarser than a march step on any scene the
// renderer is driven with, so the slope bias would have to swallow the whole
// object to keep the acne away.
inline constexpr int min_shadow_resolution = 64;
} // namespace limits

} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_TYPES_H
