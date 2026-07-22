// pycvc_algorithm.cpp — implementation of the compute free functions. Bridges
// the directly-wrapped cvc::geometry / cvc::volume to cvc/utility/algorithm.h,
// using the app passed explicitly to sdf() (the only compute fn that needs one).
#include "pycvc_algorithm.h"

#include "pycvc_context.h"

#include <array>
#include <cvc/core/app.h>
#include <cvc/core/types.h>
#include <cvc/utility/algorithm.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/dimension.h>
#include <stdexcept>

namespace pycvc {

namespace {
// Flat row-major coords -> a single point_t.
cvc::geometry::point_t point_at(const std::vector<double> &xyz, std::size_t i) {
  cvc::geometry::point_t p;
  p[0] = xyz[3 * i + 0];
  p[1] = xyz[3 * i + 1];
  p[2] = xyz[3 * i + 2];
  return p;
}

void require_len(const std::vector<double> &xyz, std::size_t n, const char *what) {
  if (xyz.size() != n)
    throw std::invalid_argument(what);
}
} // namespace

// ── SDF ────────────────────────────────────────────────────────────────
#ifdef CVC_ENABLE_SDF
cvc::volume sdf(const std::shared_ptr<cvc::app> &app, const cvc::geometry &geom, unsigned long nx,
                unsigned long ny, unsigned long nz, int algorithm, bool flipNormals) {
  if (!app)
    throw std::invalid_argument("sdf: null app handle");
  cvc::dimension dim(nx, ny, nz);
  // cvc::sdf()'s header says a default (null) bbox uses the geometry's extents,
  // but the implementation does NOT perform that substitution — it forwards the
  // null box, which degenerates to a zero-size grid. Do the documented fallback
  // here so the bbox-less overload works as advertised.
  return cvc::sdf(*app, geom, dim, geom.extents(), static_cast<cvc::sdf_algorithm>(algorithm),
                  flipNormals);
}

cvc::volume sdf(const std::shared_ptr<cvc::app> &app, const cvc::geometry &geom, unsigned long nx,
                unsigned long ny, unsigned long nz, double minx, double miny, double minz,
                double maxx, double maxy, double maxz, int algorithm, bool flipNormals) {
  if (!app)
    throw std::invalid_argument("sdf: null app handle");
  cvc::dimension dim(nx, ny, nz);
  cvc::bounding_box box(minx, miny, minz, maxx, maxy, maxz);
  return cvc::sdf(*app, geom, dim, box, static_cast<cvc::sdf_algorithm>(algorithm), flipNormals);
}
#else
cvc::volume sdf(const std::shared_ptr<cvc::app> &, const cvc::geometry &, unsigned long,
                unsigned long, unsigned long, int, bool) {
  throw std::runtime_error("sdf: this libcvc build has the SDF module disabled (CVC_ENABLE_SDF)");
}
cvc::volume sdf(const std::shared_ptr<cvc::app> &, const cvc::geometry &, unsigned long,
                unsigned long, unsigned long, double, double, double, double, double, double, int,
                bool) {
  throw std::runtime_error("sdf: this libcvc build has the SDF module disabled (CVC_ENABLE_SDF)");
}
#endif // CVC_ENABLE_SDF

// ── Isosurface / meshing ───────────────────────────────────────────────
#ifdef CVC_ENABLE_MESHER
cvc::geometry isosurface(const cvc::volume &vol, double isovalue, int method,
                         int improve_iterations) {
  return cvc::iso(vol, isovalue, static_cast<cvc::extraction_method>(method), improve_iterations);
}

cvc::geometry tetrahedralize(const cvc::volume &vol, double isovalue, int method,
                             int improve_method, int improve_iterations) {
  return cvc::tetrahedralize(vol, isovalue, static_cast<cvc::extraction_method>(method),
                             static_cast<cvc::improvement_method>(improve_method),
                             cvc::BSPLINE_CONVOLUTION, improve_iterations);
}

cvc::geometry hexahedralize(const cvc::volume &vol, double isovalue, int method, int improve_method,
                            int improve_iterations) {
  return cvc::hexahedralize(vol, isovalue, static_cast<cvc::extraction_method>(method),
                            static_cast<cvc::improvement_method>(improve_method),
                            cvc::BSPLINE_CONVOLUTION, improve_iterations);
}

cvc::geometry tetrahedralize2(const cvc::volume &vol, double isovalue, int method,
                              int improve_method, int improve_iterations) {
  return cvc::tetrahedralize2(vol, isovalue, static_cast<cvc::extraction_method>(method),
                              static_cast<cvc::improvement_method>(improve_method),
                              cvc::BSPLINE_CONVOLUTION, improve_iterations);
}

cvc::geometry tetrahedralize2(const cvc::volume &vol, double isovalue_outer, double isovalue_inner,
                              int method, int improve_method, int improve_iterations) {
  return cvc::tetrahedralize2(vol, isovalue_outer, isovalue_inner,
                              static_cast<cvc::extraction_method>(method),
                              static_cast<cvc::improvement_method>(improve_method),
                              cvc::BSPLINE_CONVOLUTION, improve_iterations);
}
#else
static std::runtime_error mesher_disabled() {
  return std::runtime_error(
      "meshing: this libcvc build has the mesher module disabled (CVC_ENABLE_MESHER)");
}
cvc::geometry isosurface(const cvc::volume &, double, int, int) { throw mesher_disabled(); }
cvc::geometry tetrahedralize(const cvc::volume &, double, int, int, int) {
  throw mesher_disabled();
}
cvc::geometry hexahedralize(const cvc::volume &, double, int, int, int) { throw mesher_disabled(); }
cvc::geometry tetrahedralize2(const cvc::volume &, double, int, int, int) {
  throw mesher_disabled();
}
cvc::geometry tetrahedralize2(const cvc::volume &, double, double, int, int, int) {
  throw mesher_disabled();
}
#endif // CVC_ENABLE_MESHER

// ── Surface extraction (always available) ──────────────────────────────
cvc::geometry extract_surface(const cvc::geometry &geom) { return cvc::extract_surface(geom); }

// ── Per-element mesh quality (always available) ────────────────────────
double tet_volume(const std::vector<double> &v) {
  require_len(v, 12, "tet_volume: expected 12 doubles (4 vertices)");
  return cvc::tet_volume(point_at(v, 0), point_at(v, 1), point_at(v, 2), point_at(v, 3));
}
double tet_aspect_ratio(const std::vector<double> &v) {
  require_len(v, 12, "tet_aspect_ratio: expected 12 doubles (4 vertices)");
  return cvc::tet_aspect_ratio(point_at(v, 0), point_at(v, 1), point_at(v, 2), point_at(v, 3));
}
double tet_min_dihedral_angle(const std::vector<double> &v) {
  require_len(v, 12, "tet_min_dihedral_angle: expected 12 doubles (4 vertices)");
  return cvc::tet_min_dihedral_angle(point_at(v, 0), point_at(v, 1), point_at(v, 2),
                                     point_at(v, 3));
}
double hex_volume(const std::vector<double> &v) {
  require_len(v, 24, "hex_volume: expected 24 doubles (8 vertices)");
  cvc::geometry::point_t hv[8];
  for (std::size_t i = 0; i < 8; ++i)
    hv[i] = point_at(v, i);
  return cvc::hex_volume(hv);
}
double hex_scaled_jacobian(const std::vector<double> &v) {
  require_len(v, 24, "hex_scaled_jacobian: expected 24 doubles (8 vertices)");
  cvc::geometry::point_t hv[8];
  for (std::size_t i = 0; i < 8; ++i)
    hv[i] = point_at(v, i);
  return cvc::hex_scaled_jacobian(hv);
}

QualityStats compute_tet_quality_stats(const cvc::geometry &geom, int metric) {
  cvc::quality_stats s = cvc::compute_tet_quality_stats(geom.tets(), geom.points(),
                                                        static_cast<cvc::quality_metric>(metric));
  QualityStats out;
  out.min = s.min;
  out.max = s.max;
  out.mean = s.mean;
  out.std_dev = s.std_dev;
  return out;
}

QualityStats compute_hex_quality_stats(const cvc::geometry &geom, int metric) {
  cvc::quality_stats s = cvc::compute_hex_quality_stats(geom.hexs(), geom.points(),
                                                        static_cast<cvc::quality_metric>(metric));
  QualityStats out;
  out.min = s.min;
  out.max = s.max;
  out.mean = s.mean;
  out.std_dev = s.std_dev;
  return out;
}

std::vector<double> compute_mesh_bounds(const cvc::geometry &geom) {
  std::array<double, 6> b = cvc::compute_mesh_bounds(geom);
  return std::vector<double>(b.begin(), b.end());
}

// ── Procedural generators (always available) ───────────────────────────
cvc::geometry sphere(double cx, double cy, double cz, double radius, int thetaRes, int phiRes) {
  return cvc::generate_sphere(cx, cy, cz, radius, thetaRes, phiRes);
}
cvc::geometry cube(double cx, double cy, double cz, double sizeX, double sizeY, double sizeZ) {
  return cvc::generate_cube(cx, cy, cz, sizeX, sizeY, sizeZ);
}
cvc::geometry torus(double cx, double cy, double cz, double majorRadius, double minorRadius,
                    int majorRes, int minorRes) {
  return cvc::generate_torus(cx, cy, cz, majorRadius, minorRadius, majorRes, minorRes);
}
cvc::geometry cone(double cx, double cy, double cz, double radius, double height, int res) {
  return cvc::generate_cone(cx, cy, cz, radius, height, res);
}

} // namespace pycvc
