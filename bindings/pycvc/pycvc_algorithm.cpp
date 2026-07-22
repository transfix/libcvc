// pycvc_algorithm.cpp — algorithm facade implementation (the only TU here
// that includes libcvc). Bridges pycvc::Geometry / pycvc::Volume to the free
// functions in cvc/utility/algorithm.h.
#include "pycvc_algorithm.h"

#include <cvc/core/app.h>
#include <cvc/core/types.h>
#include <cvc/geometry/geometry.h>
#include <cvc/utility/algorithm.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/dimension.h>
#include <cvc/volume/volume.h>

#include <array>
#include <stdexcept>

namespace pycvc {

namespace {
// Process-wide app context — same idiom as pycvc_volume.cpp. cvc::sdf() needs
// an app& (thread pool / state root).
cvc::app &ctx() {
  static cvc::app app;
  return app;
}

// Wrap a value returned by-value from a cvc algorithm into the corresponding
// pycvc facade (assign through native(), which the facade owns via shared_ptr).
Geometry wrap_geometry(cvc::geometry &&g) {
  Geometry out;
  out.native() = std::move(g);
  return out;
}
Volume wrap_volume(cvc::volume &&v) {
  Volume out;
  out.native() = std::move(v);
  return out;
}

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
Volume sdf(const Geometry &geom, unsigned long nx, unsigned long ny, unsigned long nz, int algorithm,
           bool flipNormals) {
  cvc::dimension dim(nx, ny, nz);
  // cvc::sdf()'s header says a default (null) bbox uses the geometry's extents,
  // but the implementation does NOT perform that substitution — it forwards the
  // null box, which degenerates to a zero-size grid. Do the documented fallback
  // here so the bbox-less overload works as advertised.
  return wrap_volume(cvc::sdf(ctx(), geom.native(), dim, geom.native().extents(),
                             static_cast<cvc::sdf_algorithm>(algorithm), flipNormals));
}

Volume sdf(const Geometry &geom, unsigned long nx, unsigned long ny, unsigned long nz, double minx,
           double miny, double minz, double maxx, double maxy, double maxz, int algorithm,
           bool flipNormals) {
  cvc::dimension dim(nx, ny, nz);
  cvc::bounding_box box(minx, miny, minz, maxx, maxy, maxz);
  return wrap_volume(cvc::sdf(ctx(), geom.native(), dim, box,
                             static_cast<cvc::sdf_algorithm>(algorithm), flipNormals));
}
#else
Volume sdf(const Geometry &, unsigned long, unsigned long, unsigned long, int, bool) {
  throw std::runtime_error("sdf: this libcvc build has the SDF module disabled (CVC_ENABLE_SDF)");
}
Volume sdf(const Geometry &, unsigned long, unsigned long, unsigned long, double, double, double,
           double, double, double, int, bool) {
  throw std::runtime_error("sdf: this libcvc build has the SDF module disabled (CVC_ENABLE_SDF)");
}
#endif // CVC_ENABLE_SDF

// ── Isosurface / meshing ───────────────────────────────────────────────
#ifdef CVC_ENABLE_MESHER
Geometry isosurface(const Volume &vol, double isovalue, int method, int improve_iterations) {
  return wrap_geometry(cvc::iso(vol.native(), isovalue,
                                static_cast<cvc::extraction_method>(method), improve_iterations));
}

Geometry tetrahedralize(const Volume &vol, double isovalue, int method, int improve_method,
                        int improve_iterations) {
  return wrap_geometry(cvc::tetrahedralize(vol.native(), isovalue,
                                           static_cast<cvc::extraction_method>(method),
                                           static_cast<cvc::improvement_method>(improve_method),
                                           cvc::BSPLINE_CONVOLUTION, improve_iterations));
}

Geometry hexahedralize(const Volume &vol, double isovalue, int method, int improve_method,
                       int improve_iterations) {
  return wrap_geometry(cvc::hexahedralize(vol.native(), isovalue,
                                          static_cast<cvc::extraction_method>(method),
                                          static_cast<cvc::improvement_method>(improve_method),
                                          cvc::BSPLINE_CONVOLUTION, improve_iterations));
}

Geometry tetrahedralize2(const Volume &vol, double isovalue, int method, int improve_method,
                         int improve_iterations) {
  return wrap_geometry(cvc::tetrahedralize2(vol.native(), isovalue,
                                            static_cast<cvc::extraction_method>(method),
                                            static_cast<cvc::improvement_method>(improve_method),
                                            cvc::BSPLINE_CONVOLUTION, improve_iterations));
}

Geometry tetrahedralize2(const Volume &vol, double isovalue_outer, double isovalue_inner,
                         int method, int improve_method, int improve_iterations) {
  return wrap_geometry(cvc::tetrahedralize2(vol.native(), isovalue_outer, isovalue_inner,
                                            static_cast<cvc::extraction_method>(method),
                                            static_cast<cvc::improvement_method>(improve_method),
                                            cvc::BSPLINE_CONVOLUTION, improve_iterations));
}
#else
static std::runtime_error mesher_disabled() {
  return std::runtime_error(
      "meshing: this libcvc build has the mesher module disabled (CVC_ENABLE_MESHER)");
}
Geometry isosurface(const Volume &, double, int, int) { throw mesher_disabled(); }
Geometry tetrahedralize(const Volume &, double, int, int, int) { throw mesher_disabled(); }
Geometry hexahedralize(const Volume &, double, int, int, int) { throw mesher_disabled(); }
Geometry tetrahedralize2(const Volume &, double, int, int, int) { throw mesher_disabled(); }
Geometry tetrahedralize2(const Volume &, double, double, int, int, int) { throw mesher_disabled(); }
#endif // CVC_ENABLE_MESHER

// ── Surface extraction (always available) ──────────────────────────────
Geometry extract_surface(const Geometry &geom) {
  return wrap_geometry(cvc::extract_surface(geom.native()));
}

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

QualityStats compute_tet_quality_stats(const Geometry &geom, int metric) {
  cvc::quality_stats s = cvc::compute_tet_quality_stats(
      geom.native().tets(), geom.native().points(), static_cast<cvc::quality_metric>(metric));
  QualityStats out;
  out.min = s.min;
  out.max = s.max;
  out.mean = s.mean;
  out.std_dev = s.std_dev;
  return out;
}

QualityStats compute_hex_quality_stats(const Geometry &geom, int metric) {
  cvc::quality_stats s = cvc::compute_hex_quality_stats(
      geom.native().hexs(), geom.native().points(), static_cast<cvc::quality_metric>(metric));
  QualityStats out;
  out.min = s.min;
  out.max = s.max;
  out.mean = s.mean;
  out.std_dev = s.std_dev;
  return out;
}

std::vector<double> compute_mesh_bounds(const Geometry &geom) {
  std::array<double, 6> b = cvc::compute_mesh_bounds(geom.native());
  return std::vector<double>(b.begin(), b.end());
}

// ── Procedural generators (always available) ───────────────────────────
Geometry sphere(double cx, double cy, double cz, double radius, int thetaRes, int phiRes) {
  return wrap_geometry(cvc::generate_sphere(cx, cy, cz, radius, thetaRes, phiRes));
}
Geometry cube(double cx, double cy, double cz, double sizeX, double sizeY, double sizeZ) {
  return wrap_geometry(cvc::generate_cube(cx, cy, cz, sizeX, sizeY, sizeZ));
}
Geometry torus(double cx, double cy, double cz, double majorRadius, double minorRadius,
               int majorRes, int minorRes) {
  return wrap_geometry(
      cvc::generate_torus(cx, cy, cz, majorRadius, minorRadius, majorRes, minorRes));
}
Geometry cone(double cx, double cy, double cz, double radius, double height, int res) {
  return wrap_geometry(cvc::generate_cone(cx, cy, cz, radius, height, res));
}

} // namespace pycvc
