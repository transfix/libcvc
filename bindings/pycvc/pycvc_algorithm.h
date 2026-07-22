// pycvc_algorithm.h — module-level free functions over libcvc's compute layer
// (inc/cvc/utility/algorithm.h): signed-distance fields, isosurface /
// volumetric meshing, surface extraction, per-element mesh-quality metrics, and
// procedural geometry generators.
//
// Phase-2 rearchitecture: these are DIRECT free functions that take and return
// the real, directly-wrapped cvc::geometry / cvc::volume (no facade, no
// native() bridge). SWIG wraps them from this header (it already knows the cvc
// value types); the .cpp is the TU that implements them against libcvc and the
// Phase-0 injected app (pycvc::ctx()). The heavy compute paths (sdf, iso,
// meshing) compile iff libcvc was built with CVC_ENABLE_SDF / CVC_ENABLE_MESHER
// (propagated on cvc::cvc's interface); a build that lacks them throws a clear
// "built without …" error, mirroring the CUDA-disabled pattern.
#pragma once

// Real cvc value types. SWIG ignores these #includes (it does not follow
// #include) but already has the wrapped cvc::geometry/cvc::volume from pycvc.i;
// the C++ compiler needs them complete for the by-value returns below.
#include <cvc/geometry/geometry.h>
#include <cvc/volume/volume.h>
#include <memory>
#include <vector>

namespace cvc {
class app;
}

namespace pycvc {

// ── Enum mirrors (values match cvc::* in inc/cvc/core/types.h) ─────────
// Function parameters take plain int so any of these constants (or a bare
// int) works from Python; the .cpp casts to the matching cvc enum.

// cvc::sdf_algorithm
enum sdf_algorithm { SDF_V1 = 0, SDF_V2 = 1 };

// cvc::extraction_method
enum extraction_method { DUALLIB = 0, FASTCONTOURING = 1, LIBISOCONTOUR = 2 };

// cvc::improvement_method
enum improvement_method {
  NO_IMPROVE = 0,
  GEO_FLOW = 1,
  EDGE_CONTRACT = 2,
  JOE_LIU = 3,
  MINIMAL_VOL = 4,
  OPTIMIZATION = 5
};

// cvc::normal_type
enum normal_type { BSPLINE_CONVOLUTION = 0, CENTRAL_DIFFERENCE = 1, BSPLINE_INTERPOLATION = 2 };

// cvc::quality_metric
enum quality_metric {
  TET_VOLUME = 0,
  TET_ASPECT_RATIO = 1,
  TET_MIN_ANGLE = 2,
  HEX_VOLUME = 3,
  HEX_JACOBIAN = 4,
  HEX_SCALED_JACOBIAN = 5
};

// {min, max, mean, std_dev} for a mesh-quality metric (wraps cvc::quality_stats).
struct QualityStats {
  double min = 0.0;
  double max = 0.0;
  double mean = 0.0;
  double std_dev = 0.0;
};

// ── Signed distance field (CVC_ENABLE_SDF) ─────────────────────────────
// Sample a signed-distance field of `geom` onto an nx*ny*nz grid, in `app`'s
// context (sdf needs the app's thread pool). Sign convention follows libcvc
// (inside vs outside differ; flip with flipNormals). The bbox-less overload
// uses the geometry's own extents.
cvc::volume sdf(const std::shared_ptr<cvc::app> &app, const cvc::geometry &geom, unsigned long nx,
                unsigned long ny, unsigned long nz, int algorithm = SDF_V1,
                bool flipNormals = false);
cvc::volume sdf(const std::shared_ptr<cvc::app> &app, const cvc::geometry &geom, unsigned long nx,
                unsigned long ny, unsigned long nz, double minx, double miny, double minz,
                double maxx, double maxy, double maxz, int algorithm = SDF_V1,
                bool flipNormals = false);

// ── Isosurface / volumetric meshing (CVC_ENABLE_MESHER) ────────────────
cvc::geometry isosurface(const cvc::volume &vol, double isovalue, int method = DUALLIB,
                         int improve_iterations = 0);

cvc::geometry tetrahedralize(const cvc::volume &vol, double isovalue, int method = DUALLIB,
                             int improve_method = NO_IMPROVE, int improve_iterations = 0);
cvc::geometry hexahedralize(const cvc::volume &vol, double isovalue, int method = DUALLIB,
                            int improve_method = NO_IMPROVE, int improve_iterations = 0);
cvc::geometry tetrahedralize2(const cvc::volume &vol, double isovalue, int method = DUALLIB,
                              int improve_method = NO_IMPROVE, int improve_iterations = 0);
// Layer/interval overload: mesh the region between two isosurfaces.
cvc::geometry tetrahedralize2(const cvc::volume &vol, double isovalue_outer, double isovalue_inner,
                              int method = DUALLIB, int improve_method = NO_IMPROVE,
                              int improve_iterations = 0);

// ── Surface extraction ─────────────────────────────────────────────────
cvc::geometry extract_surface(const cvc::geometry &geom);

// ── Per-element mesh-quality metrics ───────────────────────────────────
// Each takes a flat, row-major list of the element's vertex coordinates:
// a tet is 4 points (12 doubles), a hex is 8 points (24 doubles).
double tet_volume(const std::vector<double> &verts_xyz);
double tet_aspect_ratio(const std::vector<double> &verts_xyz);
double tet_min_dihedral_angle(const std::vector<double> &verts_xyz); // degrees
double hex_volume(const std::vector<double> &verts_xyz);
double hex_scaled_jacobian(const std::vector<double> &verts_xyz); // [-1, 1], 1 = perfect cube

// Quality statistics over every tet/hex element of a mesh.
QualityStats compute_tet_quality_stats(const cvc::geometry &geom, int metric = TET_ASPECT_RATIO);
QualityStats compute_hex_quality_stats(const cvc::geometry &geom, int metric = HEX_SCALED_JACOBIAN);

// {min_x, min_y, min_z, max_x, max_y, max_z} over all vertices.
std::vector<double> compute_mesh_bounds(const cvc::geometry &geom);

// ── Procedural geometry generators ─────────────────────────────────────
cvc::geometry sphere(double cx, double cy, double cz, double radius, int thetaRes = 32,
                     int phiRes = 16);
cvc::geometry cube(double cx, double cy, double cz, double sizeX, double sizeY, double sizeZ);
cvc::geometry torus(double cx, double cy, double cz, double majorRadius, double minorRadius,
                    int majorRes = 32, int minorRes = 16);
cvc::geometry cone(double cx, double cy, double cz, double radius, double height, int res = 32);

} // namespace pycvc
