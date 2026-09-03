#include <cvc/volren/types.h>

namespace cvc {
namespace volren {

mat4 mat4::affine_inverse() const {
  if (!is_affine())
    throw volren_error("mat4::affine_inverse: matrix is not affine (bottom row must be 0,0,0,1)");

  const double a = m[0], b = m[1], c = m[2];
  const double d = m[4], e = m[5], f = m[6];
  const double g = m[8], h = m[9], i = m[10];

  const double co00 = e * i - f * h;
  const double co01 = f * g - d * i;
  const double co02 = d * h - e * g;
  const double det = a * co00 + b * co01 + c * co02;
  if (det == 0.0)
    throw volren_error("mat4::affine_inverse: singular linear part");
  const double inv_det = 1.0 / det;

  mat4 out;
  // Inverse of the 3x3 linear part (adjugate / det).
  out.m[0] = co00 * inv_det;
  out.m[1] = (c * h - b * i) * inv_det;
  out.m[2] = (b * f - c * e) * inv_det;
  out.m[4] = co01 * inv_det;
  out.m[5] = (a * i - c * g) * inv_det;
  out.m[6] = (c * d - a * f) * inv_det;
  out.m[8] = co02 * inv_det;
  out.m[9] = (b * g - a * h) * inv_det;
  out.m[10] = (a * e - b * d) * inv_det;
  // Translation: -A_inv * t.
  const double tx = m[3], ty = m[7], tz = m[11];
  out.m[3] = -(out.m[0] * tx + out.m[1] * ty + out.m[2] * tz);
  out.m[7] = -(out.m[4] * tx + out.m[5] * ty + out.m[6] * tz);
  out.m[11] = -(out.m[8] * tx + out.m[9] * ty + out.m[10] * tz);
  return out;
}

vec3d mat4::transform_normal(const vec3d &v) const {
  // transpose(inverse(A)) * v == v * inverse(A) row-vector style; reuse the
  // affine inverse's linear part transposed.
  const mat4 inv = affine_inverse();
  return {inv.m[0] * v.x + inv.m[4] * v.y + inv.m[8] * v.z,
          inv.m[1] * v.x + inv.m[5] * v.y + inv.m[9] * v.z,
          inv.m[2] * v.x + inv.m[6] * v.y + inv.m[10] * v.z};
}

} // namespace volren
} // namespace cvc
