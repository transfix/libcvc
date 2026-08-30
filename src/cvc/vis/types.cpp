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

#include <cmath>
#include <cvc/vis/types.h>

namespace cvc {
namespace vis {

namespace {

// Normalize a plane by the length of its normal so plane_distance() is a true
// signed distance. A degenerate (zero) normal is left as-is; the frustum builders
// never produce one from a valid projection.
plane normalized(double a, double b, double c, double d) noexcept {
  const double len = std::sqrt(a * a + b * b + c * c);
  plane p;
  if (len > 0.0) {
    a /= len;
    b /= len;
    c /= len;
    d /= len;
  }
  p.n[0] = static_cast<float>(a);
  p.n[1] = static_cast<float>(b);
  p.n[2] = static_cast<float>(c);
  p.d = static_cast<float>(d);
  return p;
}

} // namespace

frustum frustum::from_vtk_planes(const double planes24[24]) noexcept {
  // vtk order: L, R, B, T, FAR, NEAR. Ours: L, R, B, T, NEAR, FAR. So planes
  // 0..3 map straight across, and 4/5 swap. This is the whole reason the
  // function exists (see the header's warning).
  static const int remap[6] = {0, 1, 2, 3, 5, 4};
  frustum f;
  for (int i = 0; i < 6; ++i) {
    const double *pl = planes24 + remap[i] * 4;
    f.p[i] = normalized(pl[0], pl[1], pl[2], pl[3]);
  }
  return f;
}

frustum frustum::from_view_proj(const double m[16]) noexcept {
  // Row-major M: row r is m[4*r + 0..3]. Gribb-Hartmann combinations, in our
  // L, R, B, T, NEAR, FAR order.
  const double *m0 = m + 0;
  const double *m1 = m + 4;
  const double *m2 = m + 8;
  const double *m3 = m + 12;
  frustum f;
  auto row_combo = [](const double *a, const double *b, double s) {
    return normalized(a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2], a[3] + s * b[3]);
  };
  f.p[0] = row_combo(m3, m0, +1.0); // left   = m3 + m0
  f.p[1] = row_combo(m3, m0, -1.0); // right  = m3 - m0
  f.p[2] = row_combo(m3, m1, +1.0); // bottom = m3 + m1
  f.p[3] = row_combo(m3, m1, -1.0); // top    = m3 - m1
  f.p[4] = row_combo(m3, m2, +1.0); // near   = m3 + m2
  f.p[5] = row_combo(m3, m2, -1.0); // far    = m3 - m2
  return f;
}

frustum_test sphere_vs_frustum(const sphere &s, const frustum &f) noexcept {
  bool intersects = false;
  for (int i = 0; i < 6; ++i) {
    const float dist = plane_distance(f.p[i], s.c);
    if (dist < -s.r)
      return frustum_test::outside;
    if (dist < s.r)
      intersects = true; // straddles this plane
  }
  return intersects ? frustum_test::intersect : frustum_test::inside;
}

frustum_test aabb_vs_frustum(const aabb &b, const frustum &f, std::uint8_t &mask) noexcept {
  frustum_test result = frustum_test::inside;
  for (int i = 0; i < 6; ++i) {
    if (!(mask & (1u << i)))
      continue;
    const plane &p = f.p[i];
    // p-vertex: the corner farthest along n, chosen by sign bits (no branch on
    // the box, just on the plane normal, which is loop-invariant per plane).
    const float px = p.n[0] >= 0 ? b.mx[0] : b.mn[0];
    const float py = p.n[1] >= 0 ? b.mx[1] : b.mn[1];
    const float pz = p.n[2] >= 0 ? b.mx[2] : b.mn[2];
    if (p.n[0] * px + p.n[1] * py + p.n[2] * pz + p.d < 0.0f)
      return frustum_test::outside; // farthest corner is behind: wholly outside
    // n-vertex: the opposite corner. Inside too => this plane is satisfied for
    // the whole subtree, so clear it from the mask.
    const float nx = p.n[0] >= 0 ? b.mn[0] : b.mx[0];
    const float ny = p.n[1] >= 0 ? b.mn[1] : b.mx[1];
    const float nz = p.n[2] >= 0 ? b.mn[2] : b.mx[2];
    if (p.n[0] * nx + p.n[1] * ny + p.n[2] * nz + p.d < 0.0f)
      result = frustum_test::intersect;
    else
      mask &= static_cast<std::uint8_t>(~(1u << i));
  }
  return result;
}

sphere bounding_sphere(const aabb &b) noexcept {
  sphere s;
  s.c[0] = 0.5f * (b.mn[0] + b.mx[0]);
  s.c[1] = 0.5f * (b.mn[1] + b.mx[1]);
  s.c[2] = 0.5f * (b.mn[2] + b.mx[2]);
  const float ex = b.mx[0] - s.c[0];
  const float ey = b.mx[1] - s.c[1];
  const float ez = b.mx[2] - s.c[2];
  s.r = std::sqrt(ex * ex + ey * ey + ez * ez);
  return s;
}

} // namespace vis
} // namespace cvc
