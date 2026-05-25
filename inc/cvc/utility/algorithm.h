/*
  Copyright 2008 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolumeRover.

  VolumeRover is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolumeRover is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __CVC_ALGORITHM_H__
#define __CVC_ALGORITHM_H__

#include <boost/array.hpp>
#include <boost/cstdint.hpp>
#include <boost/optional.hpp>
#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <cvc/geometry.h>
#include <cvc/namespace.h>
#include <cvc/types.h>
#include <cvc/volmagick.h>

// Forward declaration to avoid circular dependency
#ifdef CVC_ENABLE_MESHER
namespace LBIE {
class Mesher;
}
#endif

namespace cvc {
#ifdef CVC_ENABLE_SDF
// ---
// sdf
// ---
// Purpose:
//   Compute a signed distance field using the arguments specified.
// ---- Change History ----
// 12/29/2013 -- Joe R. -- Creation.
// 01/08/2014 - removing sdf_method, always using SDFLibrary now
// 12/23/2025 - adding algorithm selection enum to switch between v1 and v2
// 12/27/2025 - adding flipNormals parameter to invert inside/outside
volume sdf(app &ctx, const geometry &geom,
           /*
             Dimension of output sdf vol.
           */
           const dimension &dim,
           /*
             Bounding box of output vol. If default initialized,
             use extents of geometry.
           */
           const bounding_box &bbox = bounding_box(),
           /*
             SDF algorithm to use (SDF_V1 or SDF_V2)
           */
           sdf_algorithm algorithm = SDF_V1,
           /*
             Flip normals to invert inside/outside (true = flip, false = no flip)
           */
           bool flipNormals = false);
#endif // CVC_ENABLE_SDF

#ifdef CVC_ENABLE_MESHER
// ---
// iso
// ---
// Purpose:
//   Returns geometry representing an isosurface of the specified volume.
// ---- Change History ----
// 12/29/2013 -- Joe R. -- Creation.
// 01/08/2014 -- Joe R. -- Removing color args.
// 12/25/2025 -- Joe R. -- Adding extraction_method and improve_iterations parameters.
// 12/28/2025 -- Joe R. -- Adding optional property volume for property interpolation.
// 12/28/2025 -- Joe R. -- Adding normal_type parameter.
geometry iso(const volume &vol, double isovalue, extraction_method method = DUALLIB,
             int improve_iterations = 0, normal_type normals = BSPLINE_CONVOLUTION,
             boost::optional<const volume &> propertyVol = boost::none);

// ------------
// tetrahedralize
// ------------
// Purpose:
//   Extract a tetrahedral volumetric mesh from a volume (e.g., from SDF).
// ---- Change History ----
// 12/26/2025 -- Joe R. -- Creation.
// 12/27/2025 -- Joe R. -- Added improvement_method parameter.
// 12/28/2025 -- Joe R. -- Adding optional property volume for property interpolation.
// 12/28/2025 -- Joe R. -- Adding normal_type parameter.
geometry tetrahedralize(const volume &vol, double isovalue, extraction_method method = DUALLIB,
                        improvement_method improve_method = NO_IMPROVE,
                        normal_type normals = BSPLINE_CONVOLUTION, int improve_iterations = 0,
                        boost::optional<const volume &> propertyVol = boost::none);

// --------------
// hexahedralize
// --------------
// Purpose:
//   Extract a hexahedral volumetric mesh from a volume (e.g., from SDF).
// ---- Change History ----
// 12/27/2025 -- Joe R. -- Creation.
// 12/28/2025 -- Joe R. -- Adding optional property volume for property interpolation.
// 12/28/2025 -- Joe R. -- Adding normal_type parameter.
geometry hexahedralize(const volume &vol, double isovalue, extraction_method method = DUALLIB,
                       improvement_method improve_method = NO_IMPROVE,
                       normal_type normals = BSPLINE_CONVOLUTION, int improve_iterations = 0,
                       boost::optional<const volume &> propertyVol = boost::none);

// ---------------
// tetrahedralize2
// ---------------
// Purpose:
//   Extract a dual tetrahedral (tet2) volumetric mesh from a volume.
// ---- Change History ----
// 12/27/2025 -- Joe R. -- Creation.
// 12/28/2025 -- Joe R. -- Adding normal_type parameter.
geometry tetrahedralize2(const volume &vol, double isovalue, extraction_method method = DUALLIB,
                         improvement_method improve_method = NO_IMPROVE,
                         normal_type normals = BSPLINE_CONVOLUTION, int improve_iterations = 0);

// tetrahedralize2 (interval/layer meshing)
// ----------------------------------------
// Purpose:
//   Extract a volumetric tetrahedral mesh of the layer/interval between
//   two isosurfaces. This creates a 3D mesh of the region between
//   isovalue_outer and isovalue_inner.
// ---- Change History ----
// 12/27/2025 -- Joe R. -- Creation.
// 12/28/2025 -- Joe R. -- Adding normal_type parameter.
geometry tetrahedralize2(const volume &vol, double isovalue_outer, double isovalue_inner,
                         extraction_method method = DUALLIB,
                         improvement_method improve_method = NO_IMPROVE,
                         normal_type normals = BSPLINE_CONVOLUTION, int improve_iterations = 0);
#endif // CVC_ENABLE_MESHER

#if 0
  /*
   * volren - Volume raycaster interface
   */
  class VolrenParameters
  {
  public:

    VolrenParameters() :
      _perspective(true),
      _fov(45.0)
        {
          _cameraPosition[0] = 0.0;
          _cameraPosition[1] = 0.0;
          _cameraPosition[2] = -1000.0;

          _viewUpVector[0] = 0.0;
          _viewUpVector[1] = 1.0;
          _viewUpVector[2] = 0.0;

          _viewPlaneNormal[0] = 0.0;
          _viewPlaneNormal[1] = 0.0;
          _viewPlaneNormal[2] = 1.0;

          _viewPlaneResolution[0] = 512;
          _viewPlaneResolution[1] = 512;

          _finalImagePixelResolution[0] = 512;
          _finalImagePixelResolution[1] = 512;
        }

    VolrenParameters(const VolrenParameters& copy)
      {
        _perspective = copy._perspective;
        _fov = copy._fov;
        _cameraPosition = copy._cameraPosition;
        _viewUpVector = copy._viewUpVector;
        _viewPlaneNormal = copy._viewPlaneNormal;
        for(int i = 0; i < 2; i++)
          _viewPlaneResolution[i] = copy._viewPlaneResolution[i];
        for(int i = 0; i < 2; i++)
          _finalImagePixelResolution[i] = copy._finalImagePixelResolution[i];
      }

    // camera settings
    bool perspective() const { return _perspective; }
    VolrenParameters& perspective(bool flag) { _perspective = flag; return *this; }
    float fov() const { return _fov; }
    VolrenParameters& fov(float val) { _fov = val; return *this; }
    point_t cameraPosition() const { return _cameraPosition; }
    VolrenParameters& cameraPosition(const point_t& p) { _cameraPosition = p; return *this; }
    vector_t viewUpVector() const { return _viewUpVector; }
    VolrenParameters& viewUpVector(const vector_t& v) { _viewUpVector = v; return *this; }
    vector_t viewPlaneNormal() const { return _viewPlaneNormal; }
    VolrenParameters& viewPlaneNormal(const vector_t& v) { _viewPlaneNormal = v; return *this; }
    const uint64_t* viewPlaneResolution() const { return _viewPlaneResolution; }
    template<class C>
      VolrenParameters& viewPlaneResolution(const C& v)
      {
        _viewPlaneResolution[0] = v[0];
        _viewPlaneResolution[1] = v[1];
        return *this;
      }
    const uint64_t* finalImagePixelResolution() const { return _finalImagePixelResolution; }
    template<class C>
      VolrenParameters& finalImagePixelResolution(const C& v)
      {
        _finalImagePixelResolution[0] = v[0];
        _finalImagePixelResolution[1] = v[1];
        return *this;
      }

    //material settings
    

  private:
    bool _perspective;
    float _fov;
    point_t _cameraPosition;
    vector_t _viewUpVector;
    vector_t _viewPlaneNormal;
    uint64_t _viewPlaneResolution[2];
    uint64_t _finalImagePixelResolution[2];
  };

  typedef boost::shared_array<unsigned char> Image;
#endif

// -------------------------
// Volumetric Mesh Utilities
// -------------------------
// Purpose:
//   Utility functions for converting between surface and volumetric mesh representations.
// ---- Change History ----
// 12/28/2025 -- Joe R. -- Creation.

// Extract surface faces from tetrahedral elements
// Returns triangles (boundary faces of tetrahedra)
geometry::tris_t tet_faces(const geometry::tets_t &tets);

// Extract surface faces from hexahedral elements
// Returns quads (boundary faces of hexahedra)
geometry::quads_t hex_faces(const geometry::hexs_t &hexs);

// Decode tetrahedral elements from triangle encoding
// Used when geoframe stores tets as triangles (TETRA mesh type)
geometry::tets_t decode_tets_from_triangles(const geometry::tris_t &encoded_tris);

// Decode hexahedral elements from quad encoding
// Used when geoframe stores hexs as quads (HEXA mesh type)
geometry::hexs_t decode_hexs_from_quads(const geometry::quads_t &encoded_quads);

// Encode tetrahedral elements as triangles
// Used when converting geometry tets to geoframe format
geometry::tris_t encode_triangles_from_tets(const geometry::tets_t &tets);

// Encode hexahedral elements as quads
// Used when converting geometry hexs to geoframe format
geometry::quads_t encode_quads_from_hexs(const geometry::hexs_t &hexs);

// ----------------------------
// Surface Extraction Utilities
// ----------------------------
// Purpose:
//   Extract surface representation from volumetric meshes.
// ---- Change History ----
// 12/28/2025 -- Joe R. -- Creation (Week 3 Option 1).

// Extract surface from a geometry (works for both surface and volumetric meshes)
// Returns a new geometry containing only the boundary surface
// For surface meshes (tris/quads), returns a copy
// For volumetric meshes (tets/hexs), extracts boundary faces
geometry extract_surface(const geometry &geom);

// ---------------------------------------
// Volumetric Property Interpolation (Week 3 Option 2)
// ---------------------------------------
// Purpose:
//   Interpolate property values within volumetric elements using barycentric coordinates.
// ---- Change History ----
// 12/28/2025 -- Joe R. -- Creation (Week 3 Option 2).

// Compute barycentric coordinates for a point within a tetrahedron
// Returns 4 weights (one per vertex) that sum to 1.0
// If point is outside tet, weights may be negative
std::array<double, 4> tet_barycentric(const geometry::point_t &p, const geometry::point_t &v0,
                                      const geometry::point_t &v1, const geometry::point_t &v2,
                                      const geometry::point_t &v3);

// Compute trilinear coordinates for a point within a hexahedron
// Returns 8 weights (one per vertex) for trilinear interpolation
// Uses parametric coordinates (r,s,t) in [-1,1]^3
std::array<double, 8> hex_trilinear(const geometry::point_t &p,
                                    const geometry::point_t vertices[8]);

// Interpolate property value at a point within a tetrahedron
// Uses barycentric interpolation of vertex property values
double interpolate_in_tet(const geometry::point_t &p, const geometry::tet_t &tet,
                          const geometry::points_t &vertices,
                          const std::vector<double> &vertex_properties);

// Interpolate property value at a point within a hexahedron
// Uses trilinear interpolation of vertex property values
double interpolate_in_hex(const geometry::point_t &p, const geometry::hex_t &hex,
                          const geometry::points_t &vertices,
                          const std::vector<double> &vertex_properties);

// ---------------------------------------
// Volumetric Mesh Quality Metrics (Week 3 Option 3)
// ---------------------------------------
// Purpose:
//   Compute quality metrics for volumetric mesh elements.
// ---- Change History ----
// 12/28/2025 -- Joe R. -- Creation (Week 3 Option 3).

// Compute volume of a tetrahedron
// Returns signed volume (positive if vertices are ordered correctly)
double tet_volume(const geometry::point_t &v0, const geometry::point_t &v1,
                  const geometry::point_t &v2, const geometry::point_t &v3);

// Compute aspect ratio of a tetrahedron
// Returns ratio of longest edge to shortest altitude
// Lower values indicate better quality (equilateral tet has ratio ~2.04)
double tet_aspect_ratio(const geometry::point_t &v0, const geometry::point_t &v1,
                        const geometry::point_t &v2, const geometry::point_t &v3);

// Compute minimum dihedral angle of a tetrahedron (in degrees)
// Quality measure: good tets have angles away from 0° and 180°
double tet_min_dihedral_angle(const geometry::point_t &v0, const geometry::point_t &v1,
                              const geometry::point_t &v2, const geometry::point_t &v3);

// Compute volume of a hexahedron
// Uses decomposition into tetrahedra
double hex_volume(const geometry::point_t vertices[8]);

// Compute Jacobian determinant at center of hexahedron
// Positive values indicate valid element
// Values close to zero or negative indicate distorted/inverted elements
double hex_jacobian(const geometry::point_t vertices[8]);

// Compute scaled Jacobian quality metric for hexahedron
// Range: [-1, 1], where 1 = perfect cube, 0 = degenerate
double hex_scaled_jacobian(const geometry::point_t vertices[8]);

// Compute quality statistics for all tets in a mesh
// Returns {min, max, mean, std_dev} for specified metric
struct quality_stats {
  double min;
  double max;
  double mean;
  double std_dev;
};

quality_stats compute_tet_quality_stats(const geometry::tets_t &tets,
                                        const geometry::points_t &vertices,
                                        quality_metric metric = TET_ASPECT_RATIO);

quality_stats compute_hex_quality_stats(const geometry::hexs_t &hexs,
                                        const geometry::points_t &vertices,
                                        quality_metric metric = HEX_SCALED_JACOBIAN);

// ---------------------------------------
// Advanced Mesh Utilities (Week 3 Option 4)
// ---------------------------------------
// Purpose:
//   Advanced operations for volumetric mesh analysis and processing.
// ---- Change History ----
// 12/28/2025 -- Joe R. -- Creation (Week 3 Option 4).

// Find all tets/hexs containing a given point
// Returns indices of elements that contain the point
// Useful for point location queries in volumetric meshes
std::vector<size_t> find_tets_containing_point(const geometry::point_t &p,
                                               const geometry::tets_t &tets,
                                               const geometry::points_t &vertices);

std::vector<size_t> find_hexs_containing_point(const geometry::point_t &p,
                                               const geometry::hexs_t &hexs,
                                               const geometry::points_t &vertices);

// Compute bounding box of a volumetric mesh
// Returns {min_x, min_y, min_z, max_x, max_y, max_z}
std::array<double, 6> compute_mesh_bounds(const geometry &geom);

// Filter elements by quality threshold
// Returns indices of elements that meet quality criteria
// For tets: keeps elements with aspect_ratio < threshold
// For hexs: keeps elements with scaled_jacobian > threshold
std::vector<size_t> filter_tets_by_quality(const geometry::tets_t &tets,
                                           const geometry::points_t &vertices,
                                           double threshold = 10.0,
                                           quality_metric metric = TET_ASPECT_RATIO);

std::vector<size_t> filter_hexs_by_quality(const geometry::hexs_t &hexs,
                                           const geometry::points_t &vertices,
                                           double threshold = 0.2,
                                           quality_metric metric = HEX_SCALED_JACOBIAN);

// Create a geometry containing only elements that pass quality filter
// Useful for removing low-quality elements from a mesh
geometry extract_quality_elements(const geometry &geom, double threshold,
                                  quality_metric metric = TET_ASPECT_RATIO);

// ---------------------------------------
// Procedural Geometry Generation
// ---------------------------------------
// Purpose:
//   Generate parametric primitive geometries with proper normals.
// ---- Change History ----
// 01/07/2026 -- Joe R. -- Creation.

// Generate a sphere centered at (cx, cy, cz) with specified radius and resolution
// thetaRes: number of segments around equator (longitude)
// phiRes: number of segments from pole to pole (latitude)
// Returns triangular mesh with normals
geometry generate_sphere(double cx, double cy, double cz, double radius, int thetaRes = 32,
                         int phiRes = 16);

// Generate a cube centered at (cx, cy, cz) with specified dimensions
// Returns triangular mesh with proper face normals (vertices duplicated per face)
geometry generate_cube(double cx, double cy, double cz, double sizeX, double sizeY, double sizeZ);

// Generate a torus centered at (cx, cy, cz) with specified radii and resolution
// majorRadius: distance from torus center to tube center
// minorRadius: radius of the tube
// majorRes: number of segments around the torus
// minorRes: number of segments around the tube
// Returns triangular mesh with normals
geometry generate_torus(double cx, double cy, double cz, double majorRadius, double minorRadius,
                        int majorRes = 32, int minorRes = 16);

// Generate a cone centered at (cx, cy, cz) with specified dimensions and resolution
// Cone extends from base at cy - height/2 to apex at cy + height/2
// res: number of segments around the cone
// Returns triangular mesh with normals (separate vertices for side and base cap)
geometry generate_cone(double cx, double cy, double cz, double radius, double height, int res = 32);

} // namespace cvc

#endif
