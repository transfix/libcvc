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
} // namespace defaults

namespace limits {
// Largest raster edge render() accepts.  Keeps tile/pixel index arithmetic
// comfortably inside int range and rejects nonsense before allocating a
// multi-terabyte frame.
inline constexpr int max_raster_dim = 65536;
} // namespace limits

} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_TYPES_H
