// Per-cell ray/isosurface intersection -- the port of the one libiso path
// the legacy renderer actually used (iso_intersectW -> extract_contour ->
// intersect_triangle/in_triangle).
//
// Contracts kept bit-faithful to libiso so raycast isosurfaces stay
// consistent with cvc::iso(..., FASTCONTOURING) meshes of the same volume:
//  - corner classification with strict `<` against the isovalue;
//  - edge vertices by linear inverse interpolation in world space;
//  - triangulation straight from the VTK case table (max 5 triangles);
//  - nearest hit with t >= 0 wins;
//  - the returned local weights are NOT clamped to [0,1] (they can fall
//    epsilon outside; the spline gradient tolerates that, as it always has).
//
// Everything here is a pure function over its arguments -- libiso itself had
// no mutable state, and neither does the port.
// Derived from volrover's volren/libiso (C) 2000-2005 University of Texas at
// Austin (Park/Zhang/Rivera, advisor Bajaj), LGPL 2.1 -- the same license as
// libcvc.
#ifndef CVC_VOLREN_DETAIL_CELL_INTERSECT_H
#define CVC_VOLREN_DETAIL_CELL_INTERSECT_H

#include <cvc/volren/camera.h>
#include <cvc/volren/detail/mc_tables.h>
#include <cvc/volren/detail/sampler.h>
#include <cvc/volren/types.h>

#include <cmath>
#include <cstdint>

namespace cvc {
namespace volren {
namespace detail {

struct mc_triangle {
  vec3d vert[3];
};

// One voxel cell prepared for marching cubes: grid index, geometry, and the
// 8 corner values in MC (VTK) vertex order.
struct mc_cell {
  std::int64_t id[3] = {0, 0, 0};
  vec3d orig; // world position of voxel (0,0,0) of the grid
  vec3d span;
  float func[8] = {}; // MC order; use from_binary_corners for raster-order input

  // Fill func[] from corner values fetched in binary/raster order
  // (grid_sampler::corners), applying the legacy vals->func remap.
  void from_binary_corners(const float vals[8]) {
    for (int v = 0; v < 8; ++v)
      func[v] = vals[mc_vertex_from_binary[v]];
  }
};

// Extract the (up to 5) marching-cubes triangles of `cell` at `isovalue`.
// Returns the triangle count; 0 for the common no-crossing case.
inline int extract_contour(float isovalue, const mc_cell &cell, mc_triangle tris[5]) {
  int code = 0;
  for (int v = 0; v < 8; ++v)
    if (cell.func[v] < isovalue)
      code |= 1 << v;

  const int nedges = cube_edges[code][0];
  if (nedges == 0)
    return 0;

  vec3d edge_v[12];
  for (int e = 0; e < nedges; ++e) {
    const int edge = cube_edges[code][1 + e];
    const mc_edge_info &ei = mc_edges[edge];
    const double i = double(cell.id[0] + ei.di);
    const double j = double(cell.id[1] + ei.dj);
    const double k = double(cell.id[2] + ei.dk);
    // Linear inverse interpolation along the edge's axis, in world space.
    const double x = (double(isovalue) - double(cell.func[ei.v1])) /
                     (double(cell.func[ei.v2]) - double(cell.func[ei.v1]));
    vec3d p{cell.orig.x + cell.span.x * i, cell.orig.y + cell.span.y * j,
            cell.orig.z + cell.span.z * k};
    switch (ei.dir) {
    case 0:
      p.x = cell.orig.x + cell.span.x * (i + x);
      break;
    case 1:
      p.y = cell.orig.y + cell.span.y * (j + x);
      break;
    default:
      p.z = cell.orig.z + cell.span.z * (k + x);
      break;
    }
    edge_v[edge] = p;
  }

  int ntris = 0;
  for (int t = 0; tri_cases[code][t] != -1; t += 3, ++ntris)
    for (int v = 0; v < 3; ++v)
      tris[ntris].vert[v] = edge_v[tri_cases[code][t + v]];
  return ntris;
}

// 2D dominant-axis projection point-in-triangle test (Graphics Gems p.390,
// as in libiso's in_triangle).
inline bool in_triangle(const vec3d &point, const mc_triangle &tri, const vec3d &normal) {
  int i1, i2;
  const double ax = std::fabs(normal.x), ay = std::fabs(normal.y), az = std::fabs(normal.z);
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
  const auto at = [](const vec3d &v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); };
  const double u0 = at(point, i1) - at(tri.vert[0], i1);
  const double v0 = at(point, i2) - at(tri.vert[0], i2);
  const double u1 = at(tri.vert[1], i1) - at(tri.vert[0], i1);
  const double v1 = at(tri.vert[1], i2) - at(tri.vert[0], i2);
  const double u2 = at(tri.vert[2], i1) - at(tri.vert[0], i1);
  const double v2 = at(tri.vert[2], i2) - at(tri.vert[0], i2);

  if (u1 == 0.0) {
    const double denom = u2;
    if (denom == 0.0 || v1 == 0.0)
      return false; // degenerate triangle (the legacy code divided by zero here)
    const double beta = u0 / denom;
    if (beta < 0.0 || beta > 1.0)
      return false;
    const double alpha = (v0 - beta * v2) / v1;
    return alpha >= 0.0 && alpha + beta <= 1.0;
  }
  const double denom = v2 * u1 - u2 * v1;
  if (denom == 0.0)
    return false; // degenerate
  const double beta = (v0 * u1 - u0 * v1) / denom;
  if (beta < 0.0 || beta > 1.0)
    return false;
  const double alpha = (u0 - beta * u2) / u1;
  return alpha >= 0.0 && alpha + beta <= 1.0;
}

// Ray/triangle: plane intersection then the projection inside-test.
// Returns the ray parameter t >= 0 on a hit, or a negative value on a miss
// (libiso's intersect_triangle contract).
inline double intersect_triangle(const ray &r, const mc_triangle &tri, vec3d &point) {
  const vec3d e1 = tri.vert[1] - tri.vert[0];
  const vec3d e2 = tri.vert[2] - tri.vert[0];
  const vec3d n = normalized(cross(e1, e2));
  const double fz = dot(n, tri.vert[0] - r.origin);
  const double fm = dot(n, r.direction);
  if (fm == 0.0)
    return -1.0;
  const double t = fz / fm;
  if (t < 0.0)
    return t;
  point = r.origin + r.direction * t;
  return in_triangle(point, tri, n) ? t : -1.0;
}

// The iso_intersectW driver: nearest MC-triangle hit of `r` in `cell` at
// `isovalue`.  On a hit, returns true and writes the intersection's local
// cell weights (unclamped) to w[3] and its ray parameter to t_hit.
inline bool intersect_isosurface_in_cell(const ray &r, float isovalue, const mc_cell &cell,
                                         float w[3], double &t_hit) {
  mc_triangle tris[5];
  const int nt = extract_contour(isovalue, cell, tris);
  if (nt == 0)
    return false;

  double t0 = -1.0;
  vec3d hit;
  for (int i = 0; i < nt; ++i) {
    vec3d p;
    const double t = intersect_triangle(r, tris[i], p);
    if (t < 0.0)
      continue;
    if (t0 < 0.0 || t < t0) {
      t0 = t;
      hit = p;
    }
  }
  if (t0 < 0.0)
    return false;

  w[0] = float((hit.x - (cell.orig.x + double(cell.id[0]) * cell.span.x)) / cell.span.x);
  w[1] = float((hit.y - (cell.orig.y + double(cell.id[1]) * cell.span.y)) / cell.span.y);
  w[2] = float((hit.z - (cell.orig.z + double(cell.id[2]) * cell.span.z)) / cell.span.z);
  t_hit = t0;
  return true;
}

} // namespace detail
} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_DETAIL_CELL_INTERSECT_H
