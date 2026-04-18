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

#include <cvc/algorithm.h>
#include <cvc/utility.h>
#include <cvc/app.h>

// CGAL headers must come before SDF headers due to macro conflicts
#ifndef DISABLE_CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Point_3.h>
#include <CGAL/Bbox_3.h>
#endif

#ifdef CVC_ENABLE_SDF
#include <SDF/SignDistanceFunction/sdfLib.h>

// Include SDF v2 headers
#include <SDF/SignDistanceFunction_v2/DistanceTransform.h>
#include <SDF/SignDistanceFunction_v2/FaceVertSet3D.h>
#include <SDF/SignDistanceFunction_v2/reg3data.h>
#endif // CVC_ENABLE_SDF

// Undef conflicting macros from SDF v2 before including mesher headers
#ifdef MIN
#undef MIN
#endif
#ifdef MAX
#undef MAX
#endif

#ifdef CVC_ENABLE_MESHER
#include <cvc-mesher/Mesher/mesher.h>
#endif // CVC_ENABLE_MESHER

#include <boost/any.hpp>
#include <boost/scoped_array.hpp>

#include <iostream>
#include <cstring>
#include <algorithm>
#include <numeric>
#include <map>

#ifndef DISABLE_CGAL
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

namespace {
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point_3;
  typedef K::Triangle_3 Triangle_3;
  typedef std::vector<Triangle_3>::iterator Iterator;
  typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> AABB_traits;
  typedef CGAL::AABB_tree<AABB_traits> Tree;
}
#endif

namespace {
  // Wrapper for tet with index and bbox
  struct TetElement {
    size_t index;
    CVC_NAMESPACE::bounding_box bbox;
    
    TetElement(size_t idx,
               const CVC_NAMESPACE::geometry::tets_t& t,
               const CVC_NAMESPACE::geometry::points_t& v)
      : index(idx) {
      // Compute bbox from 4 tet vertices
      const auto& tet = t[idx];
      double minx = v[tet[0]][0], miny = v[tet[0]][1], minz = v[tet[0]][2];
      double maxx = minx, maxy = miny, maxz = minz;
      
      for(int i = 1; i < 4; ++i) {
        minx = std::min(minx, v[tet[i]][0]);
        miny = std::min(miny, v[tet[i]][1]);
        minz = std::min(minz, v[tet[i]][2]);
        maxx = std::max(maxx, v[tet[i]][0]);
        maxy = std::max(maxy, v[tet[i]][1]);
        maxz = std::max(maxz, v[tet[i]][2]);
      }
      bbox = CVC_NAMESPACE::bounding_box(minx, miny, minz, maxx, maxy, maxz);
    }
  };
  
  // Wrapper for hex with index and bbox
  struct HexElement {
    size_t index;
    CVC_NAMESPACE::bounding_box bbox;
    
    HexElement(size_t idx,
               const CVC_NAMESPACE::geometry::hexs_t& h,
               const CVC_NAMESPACE::geometry::points_t& v)
      : index(idx) {
      // Compute bbox from 8 hex vertices
      const auto& hex = h[idx];
      double minx = v[hex[0]][0], miny = v[hex[0]][1], minz = v[hex[0]][2];
      double maxx = minx, maxy = miny, maxz = minz;
      
      for(int i = 1; i < 8; ++i) {
        minx = std::min(minx, v[hex[i]][0]);
        miny = std::min(miny, v[hex[i]][1]);
        minz = std::min(minz, v[hex[i]][2]);
        maxx = std::max(maxx, v[hex[i]][0]);
        maxy = std::max(maxy, v[hex[i]][1]);
        maxz = std::max(maxz, v[hex[i]][2]);
      }
      bbox = CVC_NAMESPACE::bounding_box(minx, miny, minz, maxx, maxy, maxz);
    }
  };
}

namespace
{
#ifdef CVC_ENABLE_SDF
  CVC_DEF_EXCEPTION(sign_distance_function_error);
#endif
#ifdef CVC_ENABLE_MESHER
  CVC_DEF_EXCEPTION(cvc_mesher_error);
#endif
  
  // Helper function to round up to nearest power of 2
  CVC_NAMESPACE::uint64 next_power_of_2(CVC_NAMESPACE::uint64 n) {
    if (n == 0) return 1;
    // Check if already power of 2
    if ((n & (n - 1)) == 0) return n;
    // Round up to next power of 2
    CVC_NAMESPACE::uint64 power = 1;
    while (power < n) power <<= 1;
    return power;
  }
  
#ifdef CVC_ENABLE_SDF
  // -----------
  // sdf_library
  // -----------
  // Purpose: 
  //   Interface between the old SDF API and the new one.
  // ---- Change History ----
  // 01/10/2014 -- Joe R. -- Creation.
  // 12/10/2025 -- Joe R. -- Updated to use thread-safe SDFContext API.
  // 12/23/2025 -- Joe R. -- Added power-of-2 rounding for arbitrary dimensions.
  // 12/27/2025 -- Joe R. -- Added flipNormals parameter.
  CVC_NAMESPACE::volume sdf_library(const CVC_NAMESPACE::geometry& geom,
				    const CVC_NAMESPACE::dimension& dim,
				    const CVC_NAMESPACE::bounding_box& bbox,
				    bool flipNormals)
  {
    using namespace std;
    using namespace CVC_NAMESPACE;
    // NOTE: Don't pass flipNormals to SDFLibrary - its "fireworks" normal orientation
    // will override it. Instead, we negate the SDF values after computation.
    const int flipNormalsInt = 0;
    float mins[3] = { static_cast<float>(bbox[0]), 
                      static_cast<float>(bbox[1]), 
                      static_cast<float>(bbox[2]) };
    float maxs[3] = { static_cast<float>(bbox[3]), 
                      static_cast<float>(bbox[4]), 
                      static_cast<float>(bbox[5]) };

    // SDF lib only supports cubic power-of-2 sizes
    // Find max dimension and round up to nearest power of 2
    uint64 max_dim = *max_element(dim.dim_.begin(), dim.dim_.end());
    uint64 size = next_power_of_2(max_dim);

    cvc::app::instance().threadProgress(0.05);  // Starting

    // Use new thread-safe API
    std::unique_ptr<float[]> values;
    {
      boost::scoped_array<float> v(new float[geom.num_points()*3]);
      for(int i = 0; i < geom.num_points(); i++)
	for(int j = 0; j < 3; j++)
	  v[i*3+j] = geom.points()[i][j];
      boost::scoped_array<int> t(new int[geom.num_tris()*3]);
      for(int i = 0; i < geom.num_tris(); i++)
	for(int j = 0; j < 3; j++)
	  t[i*3+j] = geom.tris()[i][j];
      
      cvc::app::instance().threadProgress(0.10);  // Geometry prepared
      
      // Call the new thread-safe API
      // Note: SDFLibrary::computeSDF_MT is external and doesn't report progress
      // This computation typically takes 60-80% of total time
      values = SDFLibrary::computeSDF_MT(geom.num_points(), v.get(), 
                                          geom.num_tris(), t.get(),
                                          static_cast<int>(size), flipNormalsInt,
                                          mins, maxs);
      if(!values) throw sign_distance_function_error("SDFLibrary::computeSDF_MT() failed");
    }

    cvc::app::instance().threadProgress(0.85);  // SDF computation complete

    volume cv(dimension(size,size,size),Float,bbox);
    float* choppedValues = reinterpret_cast<float*>(*cv);
    {
      int i, j, k;
      int c=0;
      uint64 total_iterations = (size + 1) * (size + 1) * (size + 1);
      uint64 iteration = 0;
      for( i=0; i<=size; i++ )
	for( j=0; j<=size; j++ )
	  for( k=0; k<=size; k++ ) {
	    if( i!=size && j!=size && k!=size )
	      choppedValues[c++] = values[i*(size+1)*(size+1) + j*(size+1) + k];
	    
	    // Update progress every 10% of iterations
	    if (++iteration % (total_iterations / 10) == 0) {
	      cvc::app::instance().threadProgress(0.85 + 0.05 * (float(iteration) / float(total_iterations)));
	    }
	  }
    }
    // Smart pointer automatically cleans up values

    cvc::app::instance().threadProgress(0.92);  // Data extraction complete

    // Negate all SDF values if flipNormals is true (inverts inside/outside)
    if (flipNormals) {
      uint64 total_values = size * size * size;
      for (uint64 i = 0; i < total_values; i++) {
        choppedValues[i] = -choppedValues[i];
        
        // Update progress every 10%
        if (i % (total_values / 10) == 0) {
          cvc::app::instance().threadProgress(0.92 + 0.04 * (float(i) / float(total_values)));
        }
      }
    }

    cvc::app::instance().threadProgress(0.96);  // Flip normals complete (if needed)

    // Resize to requested dimensions if different from computed size
    if (dim.xdim != size || dim.ydim != size || dim.zdim != size) {
      cv.resize(dim);
    }

    cvc::app::instance().threadProgress(1.0);  // Complete
    return cv;
  }

  // ---------------
  // sdf_library_v2
  // ---------------
  // Purpose: 
  //   Interface to the SDF v2 API (DistanceTransform).
  // ---- Change History ----
  // 12/23/2025 -- Joe R. -- Creation.
  // 12/24/2025 -- Joe R. -- Fixed to respect provided bounding box.
  // 12/27/2025 -- Joe R. -- Added flipNormals parameter.
  CVC_NAMESPACE::volume sdf_library_v2(const CVC_NAMESPACE::geometry& geom,
				       const CVC_NAMESPACE::dimension& dim,
				       const CVC_NAMESPACE::bounding_box& bbox,
				       bool flipNormals)
  {
    using namespace std;
    using namespace CVC_NAMESPACE;

    cvc::app::instance().threadProgress(0.05);  // Starting

    // Convert cvc::geometry to FaceVertSet3D
    // Use vectors to avoid default constructor issues
    std::vector<Point3f> verts_vec;
    verts_vec.reserve(geom.num_points());
    for(int i = 0; i < geom.num_points(); i++)
      verts_vec.push_back(Point3f(geom.points()[i][0], geom.points()[i][1], geom.points()[i][2]));

    std::vector<TriId3i> tris_vec;
    tris_vec.reserve(geom.num_tris());
    for(int i = 0; i < geom.num_tris(); i++)
      tris_vec.push_back(TriId3i(geom.tris()[i][0], geom.tris()[i][1], geom.tris()[i][2]));

    FaceVertSet3D fvs(geom.num_points(), geom.num_tris(), 
                      verts_vec.data(), tris_vec.data(), 0);

    // Compute triangle normals (required for sign computation)
    fvs.computeTriNormals();

    // Flip normals if requested (inverts inside/outside)
    if (flipNormals) {
      fvs.flipTriNormals();
    }

    cvc::app::instance().threadProgress(0.10);  // Geometry prepared

    // Calculate scale factors to match the requested bounding box
    // With the new constructor, we can specify the center directly
    
    BoundingBox geom_bbox = fvs.getExtent();
    float geom_ext[3];
    float max_ext = 0;
    
    for(int i = 0; i < 3; i++) {
      geom_ext[i] = geom_bbox.upper[i] - geom_bbox.lower[i];
      if(geom_ext[i] > max_ext) max_ext = geom_ext[i];
    }
    
    // Calculate requested bbox center and half-size
    float bbox_center[3], bbox_half_size[3];
    for(int i = 0; i < 3; i++) {
      bbox_center[i] = (bbox[i] + bbox[i+3]) / 2.0f;
      bbox_half_size[i] = (bbox[i+3] - bbox[i]) / 2.0f;
    }
    
    // Calculate scale factors: factor[i] = 2 * half_size / max_ext
    // This ensures: orig[i] = center[i] - factor[i]*max_ext/2 = bbox[i]
    //               orig[i] + factor[i]*max_ext = bbox[i+3]
    // Note: bbox has already been validated/expanded by sdf() wrapper if needed
    float scale_factors[3];
    for(int i = 0; i < 3; i++) {
      scale_factors[i] = 2.0f * bbox_half_size[i] / max_ext;
    }

    int dims[3] = { static_cast<int>(dim.xdim), 
                    static_cast<int>(dim.ydim), 
                    static_cast<int>(dim.zdim) };
    
    // Use new constructor with user-specified center to respect arbitrary bbox
    DistanceTransform dt(fvs, dims, bbox_center, 0.5f, 
                        scale_factors[0], scale_factors[1], scale_factors[2]);
    
    cvc::app::instance().threadProgress(0.15);  // Distance transform initialized
    
    // Note: dt.transform() is the main computation, typically 60-80% of total time
    dt.transform();

    cvc::app::instance().threadProgress(0.85);  // Distance transform complete

    // Get the result data directly from DistanceTransform
    const Reg3Data<float>& result = dt.getReg3Data();
    
    // Create output volume with the bbox (already validated by sdf() wrapper)
    volume cv(dim, Float, bbox);
    float* volData = reinterpret_cast<float*>(*cv);
    
    // Copy data from Reg3Data to volume
    int nverts = result.getNVerts();
    if (nverts != static_cast<int>(dim.xdim * dim.ydim * dim.zdim)) {
      throw sign_distance_function_error("SDF v2: dimension mismatch");
    }
    
    std::memcpy(volData, result.getData(), nverts * sizeof(float));
    
    cvc::app::instance().threadProgress(1.0);  // Complete
    
    return cv;
  }
#endif // CVC_ENABLE_SDF

#ifdef CVC_ENABLE_MESHER
  // -------
  // get_arg
  // -------
  // Purpose: 
  //   Utility function for extracting arguments from an argument map.
  // ---- Change History ----
  // 01/10/2014 -- Joe R. -- Creation.
  template<class T, class A>
  bool get_arg(T& lhs, A& argv, const std::string& var)
  {
    if(argv.find(var)!=argv.end()) 
      {
	try
	  {
	    lhs = boost::any_cast<T>(argv[var]);
	    return true;
	  }
	catch(const boost::bad_any_cast&)
	  {
	    return false;
	  }
      }
    else
      return false;
  }

  CVC_NAMESPACE::geometry convert(const LBIE::geoframe& geo)
  {
    using namespace std;
    CVC_NAMESPACE::geometry ret_geom;
    ret_geom.points().resize(geo.verts.size());
    copy(geo.verts.begin(),
	 geo.verts.end(),
	 ret_geom.points().begin());
    ret_geom.normals().resize(geo.normals.size());
    copy(geo.normals.begin(),
	 geo.normals.end(),
	 ret_geom.normals().begin());
    ret_geom.colors().resize(geo.color.size());
    copy(geo.color.begin(),
	 geo.color.end(),
	 ret_geom.colors().begin());
    ret_geom.boundary().resize(geo.bound_sign.size());
    for(size_t j = 0; j < geo.bound_sign.size(); j++)
      ret_geom.boundary()[j] = geo.bound_sign[j];
    
    // Handle volumetric meshes (decode from face representation)
    if(geo.mesh_type == LBIE::geoframe::TETRA || geo.mesh_type == LBIE::geoframe::TETRA2) {
      // Tetrahedral mesh: triangles array contains tet faces (4 tris per tet)
      // Convert from geoframe's unsigned int to geometry's uint64_t
      CVC_NAMESPACE::geometry::tris_t encoded_tris;
      encoded_tris.reserve(geo.triangles.size());
      for(const auto& tri : geo.triangles) {
        CVC_NAMESPACE::geometry::tri_t converted_tri;
        converted_tri[0] = tri[0];
        converted_tri[1] = tri[1];
        converted_tri[2] = tri[2];
        encoded_tris.push_back(converted_tri);
      }
      ret_geom.tets() = CVC_NAMESPACE::decode_tets_from_triangles(encoded_tris);
    } else if(geo.mesh_type == LBIE::geoframe::HEXA) {
      // Hexahedral mesh: quads array contains hex faces (6 quads per hex)
      // Convert from geoframe's unsigned int to geometry's uint64_t
      CVC_NAMESPACE::geometry::quads_t encoded_quads;
      encoded_quads.reserve(geo.quads.size());
      for(const auto& quad : geo.quads) {
        CVC_NAMESPACE::geometry::quad_t converted_quad;
        converted_quad[0] = quad[0];
        converted_quad[1] = quad[1];
        converted_quad[2] = quad[2];
        converted_quad[3] = quad[3];
        encoded_quads.push_back(converted_quad);
      }
      ret_geom.hexs() = CVC_NAMESPACE::decode_hexs_from_quads(encoded_quads);
    } else {
      // Surface mesh: copy triangles and quads directly
      ret_geom.tris().resize(geo.triangles.size());
      copy(geo.triangles.begin(),
	   geo.triangles.end(),
	   ret_geom.tris().begin());
      ret_geom.quads().resize(geo.quads.size());
      copy(geo.quads.begin(),
	   geo.quads.end(),
	   ret_geom.quads().begin());
    }
    
    // Copy function values if present
    if(!geo.funcs.empty()) {
      ret_geom.functions().resize(geo.funcs.size());
      for(size_t j = 0; j < geo.funcs.size(); j++)
        ret_geom.functions()[j] = geo.funcs[j][0];
    }
    return ret_geom;
  }

  LBIE::geoframe convert(const CVC_NAMESPACE::geometry& geo)
  {
    using namespace std;
    LBIE::geoframe ret_geom;
    ret_geom.verts.resize(geo.num_points());
    copy(geo.points().begin(),
	 geo.points().end(),
	 ret_geom.verts.begin());
    ret_geom.normals.resize(geo.num_points());
    copy(geo.normals().begin(),
	 geo.normals().end(),
	 ret_geom.normals.begin());
    ret_geom.color.resize(geo.num_points());
    copy(geo.colors().begin(),
	 geo.colors().end(),
	 ret_geom.color.begin());
    ret_geom.bound_sign.resize(geo.boundary().size());
    for(size_t j = 0; j < geo.boundary().size(); j++)
      ret_geom.bound_sign[j] = geo.boundary()[j];
    
    // Determine mesh type and encode appropriately
    bool has_tets = geo.num_tets() > 0;
    bool has_hexs = geo.num_hexs() > 0;
    bool has_tris = geo.num_tris() > 0;
    bool has_quads = geo.num_quads() > 0;
    
    if(has_tets && !has_hexs && !has_tris && !has_quads) {
      // Pure tetrahedral mesh: encode tets as triangles (4 tris per tet)
      ret_geom.mesh_type = LBIE::geoframe::TETRA;
      CVC_NAMESPACE::geometry::tris_t encoded = CVC_NAMESPACE::encode_triangles_from_tets(geo.const_tets());
      // Convert from geometry's uint64_t to geoframe's unsigned int
      ret_geom.triangles.reserve(encoded.size());
      for(const auto& tri : encoded) {
        LBIE::geoframe::uint_3 converted_tri;
        converted_tri[0] = static_cast<unsigned int>(tri[0]);
        converted_tri[1] = static_cast<unsigned int>(tri[1]);
        converted_tri[2] = static_cast<unsigned int>(tri[2]);
        ret_geom.triangles.push_back(converted_tri);
      }
      ret_geom.numtris = ret_geom.triangles.size();
      ret_geom.numquads = 0;
    } else if(has_hexs && !has_tets && !has_tris && !has_quads) {
      // Pure hexahedral mesh: encode hexs as quads (6 quads per hex)
      ret_geom.mesh_type = LBIE::geoframe::HEXA;
      CVC_NAMESPACE::geometry::quads_t encoded = CVC_NAMESPACE::encode_quads_from_hexs(geo.const_hexs());
      // Convert from geometry's uint64_t to geoframe's unsigned int
      ret_geom.quads.reserve(encoded.size());
      for(const auto& quad : encoded) {
        LBIE::geoframe::uint_4 converted_quad;
        converted_quad[0] = static_cast<unsigned int>(quad[0]);
        converted_quad[1] = static_cast<unsigned int>(quad[1]);
        converted_quad[2] = static_cast<unsigned int>(quad[2]);
        converted_quad[3] = static_cast<unsigned int>(quad[3]);
        ret_geom.quads.push_back(converted_quad);
      }
      ret_geom.numquads = ret_geom.quads.size();
      ret_geom.numhexas = geo.num_hexs();
      ret_geom.numtris = 0;
    } else {
      // Surface mesh or mixed: copy triangles and quads directly
      ret_geom.mesh_type = (has_tris && !has_quads) ? LBIE::geoframe::SINGLE :
                           (!has_tris && has_quads) ? LBIE::geoframe::QUAD :
                           LBIE::geoframe::SINGLE;  // Default to SINGLE for mixed
      ret_geom.triangles.resize(geo.num_tris());
      copy(geo.tris().begin(),
	   geo.tris().end(),
	   ret_geom.triangles.begin());
      ret_geom.quads.resize(geo.num_quads());
      copy(geo.quads().begin(),
	   geo.quads().end(),
	   ret_geom.quads.begin());
      ret_geom.numtris = geo.num_tris();
      ret_geom.numquads = geo.num_quads();
    }
    
    // Set the integer counts for legacy code compatibility
    ret_geom.numverts = geo.num_points();
    
    // Copy function values if present
    if(!geo.const_functions().empty()) {
      ret_geom.funcs.resize(geo.num_points());
      for(size_t j = 0; j < geo.num_points(); j++)
        ret_geom.funcs[j][0] = geo.const_functions()[j];
    }
    return ret_geom;
  }

  // ----------
  // cvc_mesher
  // ----------
  // Purpose: 
  //   Interface between the old LBIE meshing API and the new one.
  // ---- Change History ----
  // 01/10/2014 -- Joe R. -- Creation.
  typedef std::map<std::string, boost::any> Arguments;
  CVC_NAMESPACE::geometry cvc_mesher(const CVC_NAMESPACE::volume& vol, Arguments argv)
  {
    using namespace std;
    using namespace CVC_NAMESPACE;
    
    float isovalue = LBIE::DEFAULT_IVAL, isovalue_in = LBIE::DEFAULT_IVAL_IN, 
      err = LBIE::DEFAULT_ERR, err_in = LBIE::DEFAULT_ERR_IN;
    std::string operation = "mesh", meshtype = "single", improve_method = "geo_flow";
    normal_type normaltype_enum = BSPLINE_CONVOLUTION;
    bool dual_contouring = false;
    int improve_iterations = 1;
    LBIE::Mesher::ExtractionMethod extraction_method_enum = LBIE::Mesher::DUALLIB;
    LBIE::Mesher::ImproveMethod improvement_method_enum = LBIE::Mesher::GEO_FLOW;
    LBIE::geoframe::GEOTYPE meshtype_enum = LBIE::geoframe::SINGLE;

    get_arg(isovalue, argv, string("isovalue"));
    get_arg(isovalue_in, argv, string("isovalue_in"));
    get_arg(err, argv, string("err"));
    get_arg(err_in, argv, string("err_in"));
    get_arg(operation, argv, string("operation"));
    get_arg(meshtype, argv, string("meshtype"));
    get_arg(improve_method, argv, string("improve_method"));
    get_arg(normaltype_enum, argv, string("normaltype_enum"));
    get_arg(extraction_method_enum, argv, string("extraction_method_enum"));
    get_arg(improvement_method_enum, argv, string("improvement_method_enum"));
    get_arg(meshtype_enum, argv, string("meshtype_enum"));
    get_arg(dual_contouring, argv, string("dual_contouring"));
    get_arg(improve_iterations, argv, string("improve_iterations"));

    // Extract optional property volume
    boost::optional<const CVC_NAMESPACE::volume&> propertyVol = boost::none;
    if(argv.count("propertyVol")) {
      try {
        propertyVol = boost::any_cast<const CVC_NAMESPACE::volume&>(argv["propertyVol"]);
      } catch(...) {}
    }



    // Force dual contouring for DOUBLE and TETRA2 mesh types
    if(meshtype_enum == LBIE::geoframe::DOUBLE || meshtype_enum == LBIE::geoframe::TETRA2) {
      dual_contouring = true;
    }

    // Convert geoframe mesh type to geometry type
    CVC_NAMESPACE::geometry::geometry_type geom_type;
    switch(meshtype_enum) {
      case LBIE::geoframe::SINGLE: geom_type = CVC_NAMESPACE::geometry::SURFACE_TRI; break;
      case LBIE::geoframe::TETRA:  geom_type = CVC_NAMESPACE::geometry::VOLUME_TET; break;
      case LBIE::geoframe::TETRA2: geom_type = CVC_NAMESPACE::geometry::VOLUME_TET; break;
      case LBIE::geoframe::QUAD:   geom_type = CVC_NAMESPACE::geometry::SURFACE_QUAD; break;
      case LBIE::geoframe::HEXA:   geom_type = CVC_NAMESPACE::geometry::VOLUME_HEX; break;
      case LBIE::geoframe::DOUBLE: geom_type = CVC_NAMESPACE::geometry::MIXED; break;
      default: geom_type = CVC_NAMESPACE::geometry::SURFACE_TRI; break;
    }

    // Convert CVC normal_type to LBIE::Mesher::NormalType
    LBIE::Mesher::NormalType normaltype_lbie;
    switch(normaltype_enum) {
      case BSPLINE_CONVOLUTION: normaltype_lbie = LBIE::Mesher::BSPLINE_CONVOLUTION; break;
      case CENTRAL_DIFFERENCE: normaltype_lbie = LBIE::Mesher::CENTRAL_DIFFERENCE; break;
      case BSPLINE_INTERPOLATION: normaltype_lbie = LBIE::Mesher::BSPLINE_INTERPOLATION; break;
      default: normaltype_lbie = LBIE::Mesher::BSPLINE_CONVOLUTION; break;
    }

    // Week 4: Use new geometry-based API (no conversion needed)
    return LBIE::do_mesh_geometry(vol,
                                  isovalue, isovalue_in, err, err_in,
                                  geom_type, improvement_method_enum, normaltype_lbie,
                                  extraction_method_enum, improve_iterations, dual_contouring,
                                  false, propertyVol);
  }

  // ----------
  // cvc_mesher
  // ----------
  // Purpose: 
  //   Interface between the old LBIE mesh quality improvement API and the new one.
  // ---- Change History ----
  // 01/10/2014 -- Joe R. -- Creation.
  // 12/28/2024 -- Joe R. -- Week 4: Use new geometry-based API.
  CVC_NAMESPACE::geometry cvc_mesher(const CVC_NAMESPACE::geometry& geom, Arguments argv)
  {
    using namespace std;
    using namespace CVC_NAMESPACE;
    int improve_iterations = 1;
    LBIE::Mesher::ImproveMethod improvement_method_enum = LBIE::Mesher::GEO_FLOW;

    get_arg(improvement_method_enum, argv, string("improvement_method_enum"));
    get_arg(improve_iterations, argv, string("improve_iterations"));

    // Week 4: Use new geometry-based API (no conversion needed)
    return LBIE::quality_improve_geometry(geom, improvement_method_enum, improve_iterations);
  }
#endif // CVC_ENABLE_MESHER
}

namespace CVC_NAMESPACE
{
#ifdef CVC_ENABLE_SDF
  // ---
  // sdf
  // ---
  // Purpose: 
  //   Returns a volume representing the signed distance function of the input geometry.
  // ---- Change History ----
  // 12/29/2013 -- Joe R. -- Creation.
  // 12/23/2025 -- Joe R. -- Added algorithm selection parameter.
  // 12/27/2025 -- Joe R. -- Added flipNormals parameter.
  volume sdf(const geometry& geom,
	     const dimension& dim,
	     const bounding_box& bbox_in,
	     sdf_algorithm algorithm,
	     bool flipNormals)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    volume vol;
    
    // Note: SDF v1 and v2 may use slightly different bounding boxes:
    // - v1 uses the requested bbox directly
    // - v2 requires minimum scale factor of 1.1 and may expand bbox
    // The actual bbox used is stored in the returned volume
    
    bounding_box bbox = bbox_in;
    
    switch(algorithm)
    {
      case SDF_V1:
        vol = sdf_library(geom, dim, bbox, flipNormals);
        vol.desc("Signed Distance Function - SDFLibrary v1");
        break;
        
      case SDF_V2:
        {
          // v2 requires minimum scale factor - expand bbox if needed
          point_t geom_min = geom.min_point();
          point_t geom_max = geom.max_point();
          
          double geom_ext[3];
          double max_ext = 0;
          for(int i = 0; i < 3; i++) {
            geom_ext[i] = geom_max[i] - geom_min[i];
            if(geom_ext[i] > max_ext) max_ext = geom_ext[i];
          }
          
          double bbox_center[3], bbox_half_size[3];
          for(int i = 0; i < 3; i++) {
            bbox_center[i] = (bbox[i] + bbox[i+3]) / 2.0;
            bbox_half_size[i] = (bbox[i+3] - bbox[i]) / 2.0;
          }
          
          bool needs_expansion = false;
          bounding_box expanded_bbox = bbox;
          for(int i = 0; i < 3; i++) {
            double scale_factor = 2.0 * bbox_half_size[i] / max_ext;
            if(scale_factor < 1.1) {
              needs_expansion = true;
              double new_half_size = 1.1 * max_ext / 2.0;
              expanded_bbox[i] = bbox_center[i] - new_half_size;
              expanded_bbox[i+3] = bbox_center[i] + new_half_size;
            }
          }
          
          if(needs_expansion) {
            // Compute SDF on expanded bbox, then resample to match requested bbox
            vol = sdf_library_v2(geom, dim, expanded_bbox, flipNormals);
            
            // Use GPU-accelerated resize to resample from expanded bbox to requested bbox
            // This ensures both v1 and v2 use exactly the same bounding box
            vol.resize(bbox);
          } else {
            // No expansion needed, use bbox directly
            vol = sdf_library_v2(geom, dim, bbox, flipNormals);
          }
          
          vol.desc("Signed Distance Function - DistanceTransform v2");
        }
        break;
        
      default:
        vol = sdf_library(geom, dim, bbox, flipNormals);
        vol.desc("Signed Distance Function - SDFLibrary v1 (default)");
        break;
    }
    
    return vol;
  }
#endif // CVC_ENABLE_SDF

#ifdef CVC_ENABLE_MESHER
  // ---
  // iso
  // ---
  // Purpose: 
  //   Returns geometry representing an isosurface of the specified volume.
  // ---- Change History ----
  // 12/29/2013 -- Joe R. -- Creation.
  // 01/08/2014 -- Joe R. -- Removing color args and preparing for cvc-mesher.
  // 12/25/2025 -- Joe R. -- Changed extraction_method to use enum and added improve_iterations.
  // 12/28/2025 -- Joe R. -- Adding optional property volume for property interpolation.
  // 12/28/2025 -- Joe R. -- Adding normal_type parameter.
  geometry iso(const volume& vol, double isovalue, extraction_method method, int improve_iterations,
               normal_type normals, boost::optional<const volume&> propertyVol)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    Arguments args;
    args["isovalue"] = float(isovalue);
    args["improve_iterations"] = improve_iterations;
    args["extraction_method_enum"] = static_cast<LBIE::Mesher::ExtractionMethod>(method);
    args["normaltype_enum"] = normals;
    if(propertyVol) {
      args["propertyVol"] = propertyVol.get();
    }

    return cvc_mesher(vol,args);
  }

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
  geometry tetrahedralize(const volume& vol, double isovalue,
                          extraction_method method,
                          improvement_method improve_method,
                          normal_type normals,
                          int improve_iterations,
                          boost::optional<const volume&> propertyVol)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    Arguments args;
    args["isovalue"] = float(isovalue);
    args["improve_iterations"] = improve_iterations;
    args["extraction_method_enum"] = static_cast<LBIE::Mesher::ExtractionMethod>(method);
    args["improvement_method_enum"] = static_cast<LBIE::Mesher::ImproveMethod>(improve_method);
    args["meshtype_enum"] = LBIE::geoframe::TETRA;
    args["normaltype_enum"] = normals;
    if(propertyVol) {
      args["propertyVol"] = propertyVol.get();
    }

    return cvc_mesher(vol,args);
  }

  // --------------
  // hexahedralize
  // --------------
  // Purpose:
  //   Extract a hexahedral volumetric mesh from a volume (e.g., from SDF).
  // ---- Change History ----
  // 12/27/2025 -- Joe R. -- Creation.
  // 12/28/2025 -- Joe R. -- Adding optional property volume for property interpolation.
  // 12/28/2025 -- Joe R. -- Adding normal_type parameter.
  geometry hexahedralize(const volume& vol, double isovalue,
                         extraction_method method,
                         improvement_method improve_method,
                         normal_type normals,
                         int improve_iterations,
                         boost::optional<const volume&> propertyVol)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    Arguments args;
    args["isovalue"] = float(isovalue);
    args["improve_iterations"] = improve_iterations;
    args["extraction_method_enum"] = static_cast<LBIE::Mesher::ExtractionMethod>(method);
    args["improvement_method_enum"] = static_cast<LBIE::Mesher::ImproveMethod>(improve_method);
    args["meshtype_enum"] = LBIE::geoframe::HEXA;
    args["normaltype_enum"] = normals;
    if(propertyVol) {
      args["propertyVol"] = propertyVol.get();
    }

    return cvc_mesher(vol,args);
  }

  // ---------------
  // tetrahedralize2
  // ---------------
  // Purpose:
  //   Extract a dual tetrahedral (tet2) volumetric mesh from a volume.
  // ---- Change History ----
  // 12/27/2025 -- Joe R. -- Creation.
  // 12/28/2025 -- Joe R. -- Adding normal_type parameter.
  geometry tetrahedralize2(const volume& vol, double isovalue,
                           extraction_method method,
                           improvement_method improve_method,
                           normal_type normals,
                           int improve_iterations)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    Arguments args;
    args["isovalue"] = float(isovalue);
    args["improve_iterations"] = improve_iterations;
    args["extraction_method_enum"] = static_cast<LBIE::Mesher::ExtractionMethod>(method);
    args["improvement_method_enum"] = static_cast<LBIE::Mesher::ImproveMethod>(improve_method);
    args["meshtype_enum"] = LBIE::geoframe::TETRA2;
    args["normaltype_enum"] = normals;

    return cvc_mesher(vol,args);
  }

  geometry tetrahedralize2(const volume& vol, double isovalue_outer, double isovalue_inner,
                           extraction_method method,
                           improvement_method improve_method,
                           normal_type normals,
                           int improve_iterations)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    Arguments args;
    args["isovalue"] = float(isovalue_outer);
    args["isovalue_in"] = float(isovalue_inner);  // Set the inner isovalue for interval meshing
    args["improve_iterations"] = improve_iterations;
    args["extraction_method_enum"] = static_cast<LBIE::Mesher::ExtractionMethod>(method);
    args["improvement_method_enum"] = static_cast<LBIE::Mesher::ImproveMethod>(improve_method);
    args["meshtype_enum"] = LBIE::geoframe::TETRA2;
    args["normaltype_enum"] = normals;

    return cvc_mesher(vol,args);
  }

  // ---------------
  // quality_improve
  // ---------------
  // Purpose: 
  //   Filters geometry via various improvement methods.  Defaults to using geometric flow.
  // ---- Change History ----
  // 01/10/2014 -- Joe R. -- Creation.
  // 12/26/2025 -- Joe R. -- Changed improve_method from string to enum.
  geometry& geometry::quality_improve(int iterations, improvement_method method)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    Arguments args;
    args["improve_iterations"] = iterations;
    args["improvement_method_enum"] = static_cast<LBIE::Mesher::ImproveMethod>(method);
    *this = cvc_mesher(*this, args);
    return *this;
  }
#endif // CVC_ENABLE_MESHER

  // -------------------------
  // Volumetric Mesh Utilities
  // -------------------------
  // Purpose:
  //   Utility functions for converting between surface and volumetric mesh representations.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation.

  // Extract surface (boundary) triangular faces from tetrahedral elements
  geometry::tris_t tet_faces(const geometry::tets_t& tets)
  {
    using namespace std;
    
    // Each tetrahedron has 4 triangular faces
    // Face 0: vertices (0, 2, 1) - ordered for outward normal
    // Face 1: vertices (0, 1, 3)
    // Face 2: vertices (0, 3, 2)
    // Face 3: vertices (1, 2, 3)
    
    // Use a map to track face counts (boundary faces appear exactly once)
    map<geometry::tri_t, int> face_count;
    
    for(size_t i = 0; i < tets.size(); i++) {
      const geometry::tet_t& tet = tets[i];
      
      // Generate all 4 faces, sorted to create canonical representation
      geometry::tri_t faces[4];
      
      // Create faces with sorted indices for consistent comparison
      vector<geometry::index_t> f0 = {tet[0], tet[2], tet[1]};
      vector<geometry::index_t> f1 = {tet[0], tet[1], tet[3]};
      vector<geometry::index_t> f2 = {tet[0], tet[3], tet[2]};
      vector<geometry::index_t> f3 = {tet[1], tet[2], tet[3]};
      
      sort(f0.begin(), f0.end());
      sort(f1.begin(), f1.end());
      sort(f2.begin(), f2.end());
      sort(f3.begin(), f3.end());
      
      faces[0][0] = f0[0]; faces[0][1] = f0[1]; faces[0][2] = f0[2];
      faces[1][0] = f1[0]; faces[1][1] = f1[1]; faces[1][2] = f1[2];
      faces[2][0] = f2[0]; faces[2][1] = f2[1]; faces[2][2] = f2[2];
      faces[3][0] = f3[0]; faces[3][1] = f3[1]; faces[3][2] = f3[2];
      
      for(int j = 0; j < 4; j++) {
        face_count[faces[j]]++;
      }
    }
    
    // Extract boundary faces (those that appear exactly once)
    geometry::tris_t boundary_faces;
    for(const auto& entry : face_count) {
      if(entry.second == 1) {
        boundary_faces.push_back(entry.first);
      }
    }
    
    return boundary_faces;
  }

  // Extract surface (boundary) quad faces from hexahedral elements
  geometry::quads_t hex_faces(const geometry::hexs_t& hexs)
  {
    using namespace std;
    
    // Each hexahedron has 6 quad faces
    // Standard hex vertex ordering (same as VTK):
    //   Bottom: 0-1-2-3, Top: 4-5-6-7
    // Face 0: (0, 3, 2, 1) - bottom
    // Face 1: (4, 5, 6, 7) - top
    // Face 2: (0, 1, 5, 4) - front
    // Face 3: (2, 3, 7, 6) - back
    // Face 4: (0, 4, 7, 3) - left
    // Face 5: (1, 2, 6, 5) - right
    
    map<geometry::quad_t, int> face_count;
    
    for(size_t i = 0; i < hexs.size(); i++) {
      const geometry::hex_t& hex = hexs[i];
      
      // Generate all 6 faces, sorted for canonical representation
      geometry::quad_t faces[6];
      
      vector<geometry::index_t> f0 = {hex[0], hex[3], hex[2], hex[1]};
      vector<geometry::index_t> f1 = {hex[4], hex[5], hex[6], hex[7]};
      vector<geometry::index_t> f2 = {hex[0], hex[1], hex[5], hex[4]};
      vector<geometry::index_t> f3 = {hex[2], hex[3], hex[7], hex[6]};
      vector<geometry::index_t> f4 = {hex[0], hex[4], hex[7], hex[3]};
      vector<geometry::index_t> f5 = {hex[1], hex[2], hex[6], hex[5]};
      
      sort(f0.begin(), f0.end());
      sort(f1.begin(), f1.end());
      sort(f2.begin(), f2.end());
      sort(f3.begin(), f3.end());
      sort(f4.begin(), f4.end());
      sort(f5.begin(), f5.end());
      
      faces[0][0] = f0[0]; faces[0][1] = f0[1]; faces[0][2] = f0[2]; faces[0][3] = f0[3];
      faces[1][0] = f1[0]; faces[1][1] = f1[1]; faces[1][2] = f1[2]; faces[1][3] = f1[3];
      faces[2][0] = f2[0]; faces[2][1] = f2[1]; faces[2][2] = f2[2]; faces[2][3] = f2[3];
      faces[3][0] = f3[0]; faces[3][1] = f3[1]; faces[3][2] = f3[2]; faces[3][3] = f3[3];
      faces[4][0] = f4[0]; faces[4][1] = f4[1]; faces[4][2] = f4[2]; faces[4][3] = f4[3];
      faces[5][0] = f5[0]; faces[5][1] = f5[1]; faces[5][2] = f5[2]; faces[5][3] = f5[3];
      
      for(int j = 0; j < 6; j++) {
        face_count[faces[j]]++;
      }
    }
    
    // Extract boundary faces
    geometry::quads_t boundary_faces;
    for(const auto& entry : face_count) {
      if(entry.second == 1) {
        boundary_faces.push_back(entry.first);
      }
    }
    
    return boundary_faces;
  }

  // Decode tetrahedral elements from triangle encoding
  // LBIE geoframe stores each tet as 4 consecutive triangles (one per face)
  geometry::tets_t decode_tets_from_triangles(const geometry::tris_t& encoded_tris)
  {
    geometry::tets_t tets;
    
    // Each tet is represented by 4 consecutive triangles
    if(encoded_tris.size() % 4 != 0) {
      std::cerr << "Warning: Triangle count (" << encoded_tris.size() 
                << ") is not divisible by 4. Cannot decode tets." << std::endl;
      return tets;
    }
    
    size_t num_tets = encoded_tris.size() / 4;
    tets.reserve(num_tets);
    
    for(size_t i = 0; i < num_tets; i++) {
      // Extract the 4 triangular faces of this tet
      const geometry::tri_t& tri0 = encoded_tris[4*i + 0];
      const geometry::tri_t& tri1 = encoded_tris[4*i + 1];
      const geometry::tri_t& tri2 = encoded_tris[4*i + 2];
      const geometry::tri_t& tri3 = encoded_tris[4*i + 3];
      
      // Collect all unique vertices from the 4 faces
      // A tet has 4 vertices, and each appears in 3 of the 4 faces
      std::set<geometry::index_t> vertices;
      vertices.insert(tri0[0]); vertices.insert(tri0[1]); vertices.insert(tri0[2]);
      vertices.insert(tri1[0]); vertices.insert(tri1[1]); vertices.insert(tri1[2]);
      vertices.insert(tri2[0]); vertices.insert(tri2[1]); vertices.insert(tri2[2]);
      vertices.insert(tri3[0]); vertices.insert(tri3[1]); vertices.insert(tri3[2]);
      
      if(vertices.size() != 4) {
        std::cerr << "Warning: Tet " << i << " has " << vertices.size() 
                  << " unique vertices (expected 4). Skipping." << std::endl;
        continue;
      }
      
      // Convert set to tet (vertex order may not match original, but that's okay)
      geometry::tet_t tet;
      auto it = vertices.begin();
      tet[0] = *it++; tet[1] = *it++; tet[2] = *it++; tet[3] = *it;
      tets.push_back(tet);
    }
    
    return tets;
  }

  // Decode hexahedral elements from quad encoding  
  // LBIE geoframe stores each hex as 6 consecutive quads (one per face)
  geometry::hexs_t decode_hexs_from_quads(const geometry::quads_t& encoded_quads)
  {
    geometry::hexs_t hexs;
    
    // Each hex is represented by 6 consecutive quads
    if(encoded_quads.size() % 6 != 0) {
      std::cerr << "Warning: Quad count (" << encoded_quads.size() 
                << ") is not divisible by 6. Cannot decode hexs." << std::endl;
      return hexs;
    }
    
    size_t num_hexs = encoded_quads.size() / 6;
    hexs.reserve(num_hexs);
    
    for(size_t i = 0; i < num_hexs; i++) {
      // Based on saveHexa, the hex vertices are reconstructed from the first two quads:
      // Hex vertices: quads[6*i][0,1,2,3] and quads[6*i+1][1,0,3,2]
      // This gives us 8 vertices in standard hex ordering
      const geometry::quad_t& quad0 = encoded_quads[6*i + 0];
      const geometry::quad_t& quad1 = encoded_quads[6*i + 1];
      
      geometry::hex_t hex;
      hex[0] = quad0[0];  // Bottom face
      hex[1] = quad0[1];
      hex[2] = quad0[2];
      hex[3] = quad0[3];
      hex[4] = quad1[1];  // Top face (note the reordering from quad1)
      hex[5] = quad1[0];
      hex[6] = quad1[3];
      hex[7] = quad1[2];
      
      hexs.push_back(hex);
    }
    
    return hexs;
  }

  // Encode tetrahedral elements as triangles for geoframe
  // Each tet becomes 4 triangles (one per face)
  geometry::tris_t encode_triangles_from_tets(const geometry::tets_t& tets)
  {
    geometry::tris_t encoded;
    encoded.reserve(tets.size() * 4);
    
    for(size_t i = 0; i < tets.size(); i++) {
      const geometry::tet_t& tet = tets[i];
      
      // Create 4 triangular faces (matching AddTetra order for sign==1)
      // These faces should match the pattern used in LBIE
      geometry::tri_t tri0 = {{tet[0], tet[2], tet[1]}};
      geometry::tri_t tri1 = {{tet[1], tet[2], tet[3]}};
      geometry::tri_t tri2 = {{tet[0], tet[3], tet[2]}};
      geometry::tri_t tri3 = {{tet[0], tet[1], tet[3]}};
      
      encoded.push_back(tri0);
      encoded.push_back(tri1);
      encoded.push_back(tri2);
      encoded.push_back(tri3);
    }
    
    return encoded;
  }

  // Encode hexahedral elements as quads for geoframe
  // Each hex becomes 6 quads (one per face)
  geometry::quads_t encode_quads_from_hexs(const geometry::hexs_t& hexs)
  {
    geometry::quads_t encoded;
    encoded.reserve(hexs.size() * 6);
    
    for(size_t i = 0; i < hexs.size(); i++) {
      const geometry::hex_t& hex = hexs[i];
      
      // Create 6 quad faces
      // Standard hex ordering: bottom (0-3), top (4-7)
      // Match the pattern from saveHexa reconstruction
      
      geometry::quad_t quad0 = {{hex[0], hex[1], hex[2], hex[3]}}; // Bottom
      geometry::quad_t quad1 = {{hex[5], hex[4], hex[7], hex[6]}}; // Top (reordered for saveHexa)
      geometry::quad_t quad2 = {{hex[0], hex[1], hex[5], hex[4]}}; // Front
      geometry::quad_t quad3 = {{hex[2], hex[3], hex[7], hex[6]}}; // Back
      geometry::quad_t quad4 = {{hex[0], hex[4], hex[7], hex[3]}}; // Left
      geometry::quad_t quad5 = {{hex[1], hex[2], hex[6], hex[5]}}; // Right
      
      encoded.push_back(quad0);
      encoded.push_back(quad1);
      encoded.push_back(quad2);
      encoded.push_back(quad3);
      encoded.push_back(quad4);
      encoded.push_back(quad5);
    }
    
    return encoded;
  }

  // ----------------
  // extract_surface
  // ----------------
  // Purpose:
  //   Extract surface (boundary) representation from a geometry.
  //   For surface meshes, returns a copy.
  //   For volumetric meshes, extracts boundary faces.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 1).
  geometry extract_surface(const geometry& geom)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    geometry surface;
    
    // Copy vertex data
    surface.points() = geom.points();
    surface.normals() = geom.normals();
    surface.colors() = geom.colors();
    surface.boundary() = geom.boundary();
    surface.functions() = geom.functions();
    
    // Extract surface elements based on mesh type
    if(geom.num_tets() > 0) {
      // Tetrahedral mesh: extract boundary triangles
      surface.tris() = tet_faces(geom.const_tets());
    } else if(geom.num_hexs() > 0) {
      // Hexahedral mesh: extract boundary quads
      surface.quads() = hex_faces(geom.const_hexs());
    } else {
      // Surface mesh: copy triangles and quads directly
      surface.tris() = geom.tris();
      surface.quads() = geom.quads();
      surface.lines() = geom.lines();
    }
    
    return surface;
  }

  // ---------------------------------------
  // Volumetric Property Interpolation
  // ---------------------------------------
  // Purpose:
  //   Interpolate property values within volumetric elements.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 2).

  // -------------------
  // tet_barycentric
  // -------------------
  // Purpose:
  //   Compute barycentric coordinates for a point within a tetrahedron.
  //   Returns 4 weights (w0, w1, w2, w3) such that:
  //   p = w0*v0 + w1*v1 + w2*v2 + w3*v3, where w0+w1+w2+w3 = 1.
  //
  //   Uses the volume method: each weight is the ratio of the volume
  //   of the sub-tetrahedron formed by p and the opposite face to the
  //   total volume of the tetrahedron.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 2).
  std::array<double, 4> tet_barycentric(const geometry::point_t& p,
                                        const geometry::point_t& v0,
                                        const geometry::point_t& v1,
                                        const geometry::point_t& v2,
                                        const geometry::point_t& v3)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Helper lambda to compute signed volume of tetrahedron
    auto tet_volume = [](const geometry::point_t& a,
                        const geometry::point_t& b,
                        const geometry::point_t& c,
                        const geometry::point_t& d) -> double {
      // V = (1/6) * dot(ab, cross(ac, ad))
      double ab[3] = {b[0]-a[0], b[1]-a[1], b[2]-a[2]};
      double ac[3] = {c[0]-a[0], c[1]-a[1], c[2]-a[2]};
      double ad[3] = {d[0]-a[0], d[1]-a[1], d[2]-a[2]};
      
      // Cross product ac x ad
      double cross[3] = {
        ac[1]*ad[2] - ac[2]*ad[1],
        ac[2]*ad[0] - ac[0]*ad[2],
        ac[0]*ad[1] - ac[1]*ad[0]
      };
      
      // Dot product ab . (ac x ad)
      double dot = ab[0]*cross[0] + ab[1]*cross[1] + ab[2]*cross[2];
      return dot / 6.0;
    };
    
    // Compute total volume
    double V = tet_volume(v0, v1, v2, v3);
    
    // Avoid division by zero for degenerate tets
    if(std::abs(V) < 1e-15) {
      return {{0.25, 0.25, 0.25, 0.25}};  // Equal weights for degenerate case
    }
    
    // Compute sub-volumes (each opposite to a vertex)
    double V0 = tet_volume(p, v1, v2, v3);  // Opposite v0
    double V1 = tet_volume(v0, p, v2, v3);  // Opposite v1
    double V2 = tet_volume(v0, v1, p, v3);  // Opposite v2
    double V3 = tet_volume(v0, v1, v2, p);  // Opposite v3
    
    // Barycentric coordinates are ratios of volumes
    return {{V0/V, V1/V, V2/V, V3/V}};
  }

  // ----------------
  // hex_trilinear
  // ----------------
  // Purpose:
  //   Compute trilinear interpolation weights for a point within a hexahedron.
  //   Returns 8 weights (one per vertex) for trilinear interpolation.
  //
  //   Uses Newton iteration to find parametric coordinates (r,s,t) in [-1,1]^3,
  //   then computes shape functions N_i(r,s,t).
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 2).
  std::array<double, 8> hex_trilinear(const geometry::point_t& p,
                                      const geometry::point_t vertices[8])
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Shape functions for hexahedron in parametric space (r,s,t) in [-1,1]^3
    // Standard ordering: vertices 0-3 bottom face, 4-7 top face
    auto shape_function = [](int i, double r, double s, double t) -> double {
      // Map vertex index to parametric coordinates
      double ri = (i & 1) ? 1.0 : -1.0;
      double si = (i & 2) ? 1.0 : -1.0;
      double ti = (i & 4) ? 1.0 : -1.0;
      return 0.125 * (1.0 + ri*r) * (1.0 + si*s) * (1.0 + ti*t);
    };
    
    // Start with center of parametric space
    double r = 0.0, s = 0.0, t = 0.0;
    
    // Newton iteration to find parametric coordinates
    const int max_iter = 10;
    const double tol = 1e-6;
    
    for(int iter = 0; iter < max_iter; ++iter) {
      // Compute current position in physical space
      geometry::point_t x_current = {{0.0, 0.0, 0.0}};
      for(int i = 0; i < 8; ++i) {
        double N = shape_function(i, r, s, t);
        x_current[0] += N * vertices[i][0];
        x_current[1] += N * vertices[i][1];
        x_current[2] += N * vertices[i][2];
      }
      
      // Compute residual
      double dx = p[0] - x_current[0];
      double dy = p[1] - x_current[1];
      double dz = p[2] - x_current[2];
      
      if(std::sqrt(dx*dx + dy*dy + dz*dz) < tol) break;
      
      // Compute Jacobian derivatives (simplified - use finite differences)
      const double h = 0.01;
      
      // dx/dr, dy/dr, dz/dr
      geometry::point_t xr = {{0.0, 0.0, 0.0}};
      for(int i = 0; i < 8; ++i) {
        double N_plus = shape_function(i, r+h, s, t);
        double N_minus = shape_function(i, r-h, s, t);
        xr[0] += (N_plus - N_minus) * vertices[i][0] / (2*h);
        xr[1] += (N_plus - N_minus) * vertices[i][1] / (2*h);
        xr[2] += (N_plus - N_minus) * vertices[i][2] / (2*h);
      }
      
      // Update parametric coordinates (simple gradient step)
      double step = 0.1;
      r += step * dx / (std::abs(xr[0]) + 1e-10);
      s += step * dy / (std::abs(xr[1]) + 1e-10);
      t += step * dz / (std::abs(xr[2]) + 1e-10);
      
      // Clamp to valid range
      r = std::max(-1.0, std::min(1.0, r));
      s = std::max(-1.0, std::min(1.0, s));
      t = std::max(-1.0, std::min(1.0, t));
    }
    
    // Compute final shape function values
    std::array<double, 8> weights;
    for(int i = 0; i < 8; ++i) {
      weights[i] = shape_function(i, r, s, t);
    }
    
    return weights;
  }

  // ---------------------
  // interpolate_in_tet
  // ---------------------
  // Purpose:
  //   Interpolate property value at a point within a tetrahedron
  //   using barycentric coordinates.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 2).
  double interpolate_in_tet(const geometry::point_t& p,
                           const geometry::tet_t& tet,
                           const geometry::points_t& vertices,
                           const std::vector<double>& vertex_properties)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Get barycentric coordinates
    auto weights = tet_barycentric(p,
                                   vertices[tet[0]],
                                   vertices[tet[1]],
                                   vertices[tet[2]],
                                   vertices[tet[3]]);
    
    // Interpolate using barycentric weights
    double value = 0.0;
    for(int i = 0; i < 4; ++i) {
      value += weights[i] * vertex_properties[tet[i]];
    }
    
    return value;
  }

  // ---------------------
  // interpolate_in_hex
  // ---------------------
  // Purpose:
  //   Interpolate property value at a point within a hexahedron
  //   using trilinear interpolation.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 2).
  double interpolate_in_hex(const geometry::point_t& p,
                           const geometry::hex_t& hex,
                           const geometry::points_t& vertices,
                           const std::vector<double>& vertex_properties)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Get vertices of hex
    geometry::point_t hex_verts[8];
    for(int i = 0; i < 8; ++i) {
      hex_verts[i] = vertices[hex[i]];
    }
    
    // Get trilinear weights
    auto weights = hex_trilinear(p, hex_verts);
    
    // Interpolate using trilinear weights
    double value = 0.0;
    for(int i = 0; i < 8; ++i) {
      value += weights[i] * vertex_properties[hex[i]];
    }
    
    return value;
  }

  // ---------------------------------------
  // Volumetric Mesh Quality Metrics
  // ---------------------------------------
  // Purpose:
  //   Compute quality metrics for volumetric mesh elements.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 3).

  // -------------
  // tet_volume
  // -------------
  // Purpose:
  //   Compute signed volume of a tetrahedron.
  //   V = (1/6) * dot(v0v1, cross(v0v2, v0v3))
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 3).
  double tet_volume(const geometry::point_t& v0,
                   const geometry::point_t& v1,
                   const geometry::point_t& v2,
                   const geometry::point_t& v3)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Compute edges from v0
    double v0v1[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
    double v0v2[3] = {v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]};
    double v0v3[3] = {v3[0]-v0[0], v3[1]-v0[1], v3[2]-v0[2]};
    
    // Cross product v0v2 x v0v3
    double cross[3] = {
      v0v2[1]*v0v3[2] - v0v2[2]*v0v3[1],
      v0v2[2]*v0v3[0] - v0v2[0]*v0v3[2],
      v0v2[0]*v0v3[1] - v0v2[1]*v0v3[0]
    };
    
    // Dot product v0v1 . (v0v2 x v0v3)
    double dot = v0v1[0]*cross[0] + v0v1[1]*cross[1] + v0v1[2]*cross[2];
    
    return dot / 6.0;
  }

  // -------------------
  // tet_aspect_ratio
  // -------------------
  // Purpose:
  //   Compute aspect ratio of a tetrahedron.
  //   Ratio of longest edge to inradius (times a constant).
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 3).
  double tet_aspect_ratio(const geometry::point_t& v0,
                         const geometry::point_t& v1,
                         const geometry::point_t& v2,
                         const geometry::point_t& v3)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Compute all 6 edge lengths
    auto edge_length = [](const geometry::point_t& a, const geometry::point_t& b) {
      double dx = b[0] - a[0];
      double dy = b[1] - a[1];
      double dz = b[2] - a[2];
      return std::sqrt(dx*dx + dy*dy + dz*dz);
    };
    
    double e01 = edge_length(v0, v1);
    double e02 = edge_length(v0, v2);
    double e03 = edge_length(v0, v3);
    double e12 = edge_length(v1, v2);
    double e13 = edge_length(v1, v3);
    double e23 = edge_length(v2, v3);
    
    double longest_edge = std::max({e01, e02, e03, e12, e13, e23});
    
    // Compute volume
    double vol = std::abs(tet_volume(v0, v1, v2, v3));
    
    // Compute surface area (sum of 4 triangle areas)
    auto tri_area = [](const geometry::point_t& a, 
                      const geometry::point_t& b, 
                      const geometry::point_t& c) {
      double ab[3] = {b[0]-a[0], b[1]-a[1], b[2]-a[2]};
      double ac[3] = {c[0]-a[0], c[1]-a[1], c[2]-a[2]};
      double cross[3] = {
        ab[1]*ac[2] - ab[2]*ac[1],
        ab[2]*ac[0] - ab[0]*ac[2],
        ab[0]*ac[1] - ab[1]*ac[0]
      };
      return 0.5 * std::sqrt(cross[0]*cross[0] + cross[1]*cross[1] + cross[2]*cross[2]);
    };
    
    double area = tri_area(v0, v1, v2) + tri_area(v0, v1, v3) + 
                  tri_area(v0, v2, v3) + tri_area(v1, v2, v3);
    
    // Inradius = 3*volume / surface_area
    double inradius = (area > 1e-15) ? (3.0 * vol / area) : 1e-15;
    
    // Aspect ratio = longest_edge / (2*sqrt(6)*inradius)
    // Normalized so equilateral tet has ratio ~1.0
    return longest_edge / (2.0 * std::sqrt(6.0) * inradius);
  }

  // -------------------------
  // tet_min_dihedral_angle
  // -------------------------
  // Purpose:
  //   Compute minimum dihedral angle of a tetrahedron (in degrees).
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 3).
  double tet_min_dihedral_angle(const geometry::point_t& v0,
                                const geometry::point_t& v1,
                                const geometry::point_t& v2,
                                const geometry::point_t& v3)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Helper to compute face normal
    auto face_normal = [](const geometry::point_t& a,
                         const geometry::point_t& b,
                         const geometry::point_t& c) -> std::array<double, 3> {
      double ab[3] = {b[0]-a[0], b[1]-a[1], b[2]-a[2]};
      double ac[3] = {c[0]-a[0], c[1]-a[1], c[2]-a[2]};
      std::array<double, 3> n = {{
        ab[1]*ac[2] - ab[2]*ac[1],
        ab[2]*ac[0] - ab[0]*ac[2],
        ab[0]*ac[1] - ab[1]*ac[0]
      }};
      double len = std::sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
      if(len > 1e-15) {
        n[0] /= len; n[1] /= len; n[2] /= len;
      }
      return n;
    };
    
    // Compute normals of all 4 faces
    auto n0 = face_normal(v1, v2, v3);  // Opposite v0
    auto n1 = face_normal(v0, v3, v2);  // Opposite v1
    auto n2 = face_normal(v0, v1, v3);  // Opposite v2
    auto n3 = face_normal(v0, v2, v1);  // Opposite v3
    
    // Compute angles between adjacent faces
    auto dihedral_angle = [](const std::array<double, 3>& n1,
                            const std::array<double, 3>& n2) -> double {
      double dot = n1[0]*n2[0] + n1[1]*n2[1] + n1[2]*n2[2];
      dot = std::max(-1.0, std::min(1.0, dot));  // Clamp to [-1,1]
      return std::acos(-dot) * 180.0 / M_PI;  // Negative for dihedral angle
    };
    
    // Check all 6 edges (pairs of adjacent faces)
    double min_angle = 180.0;
    min_angle = std::min(min_angle, dihedral_angle(n0, n1));
    min_angle = std::min(min_angle, dihedral_angle(n0, n2));
    min_angle = std::min(min_angle, dihedral_angle(n0, n3));
    min_angle = std::min(min_angle, dihedral_angle(n1, n2));
    min_angle = std::min(min_angle, dihedral_angle(n1, n3));
    min_angle = std::min(min_angle, dihedral_angle(n2, n3));
    
    return min_angle;
  }

  // ------------
  // hex_volume
  // ------------
  // Purpose:
  //   Compute volume of a hexahedron by decomposing into tetrahedra.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 3).
  double hex_volume(const geometry::point_t vertices[8])
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Decompose hex into 5 tets
    // Using center decomposition for better accuracy
    geometry::point_t center = {{0.0, 0.0, 0.0}};
    for(int i = 0; i < 8; ++i) {
      center[0] += vertices[i][0];
      center[1] += vertices[i][1];
      center[2] += vertices[i][2];
    }
    center[0] /= 8.0;
    center[1] /= 8.0;
    center[2] /= 8.0;
    
    // Sum volumes of 6 pyramids (one per face)
    double total_volume = 0.0;
    
    // Bottom face (0-1-2-3)
    total_volume += std::abs(tet_volume(center, vertices[0], vertices[1], vertices[2]));
    total_volume += std::abs(tet_volume(center, vertices[0], vertices[2], vertices[3]));
    
    // Top face (4-5-6-7)
    total_volume += std::abs(tet_volume(center, vertices[5], vertices[4], vertices[7]));
    total_volume += std::abs(tet_volume(center, vertices[5], vertices[7], vertices[6]));
    
    // Front face (0-1-5-4)
    total_volume += std::abs(tet_volume(center, vertices[0], vertices[1], vertices[5]));
    total_volume += std::abs(tet_volume(center, vertices[0], vertices[5], vertices[4]));
    
    // Back face (2-3-7-6)
    total_volume += std::abs(tet_volume(center, vertices[2], vertices[3], vertices[7]));
    total_volume += std::abs(tet_volume(center, vertices[2], vertices[7], vertices[6]));
    
    // Left face (0-4-7-3)
    total_volume += std::abs(tet_volume(center, vertices[0], vertices[4], vertices[7]));
    total_volume += std::abs(tet_volume(center, vertices[0], vertices[7], vertices[3]));
    
    // Right face (1-2-6-5)
    total_volume += std::abs(tet_volume(center, vertices[1], vertices[2], vertices[6]));
    total_volume += std::abs(tet_volume(center, vertices[1], vertices[6], vertices[5]));
    
    return total_volume;
  }

  // -------------
  // hex_jacobian
  // -------------
  // Purpose:
  //   Compute Jacobian determinant at center of hexahedron.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 3).
  double hex_jacobian(const geometry::point_t vertices[8])
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Evaluate at parametric center (r=s=t=0)
    // Shape function derivatives at center
    double dNdr[8] = {-0.125, 0.125, 0.125, -0.125, -0.125, 0.125, 0.125, -0.125};
    double dNds[8] = {-0.125, -0.125, 0.125, 0.125, -0.125, -0.125, 0.125, 0.125};
    double dNdt[8] = {-0.125, -0.125, -0.125, -0.125, 0.125, 0.125, 0.125, 0.125};
    
    // Compute Jacobian matrix elements
    double dxdr = 0, dydr = 0, dzdr = 0;
    double dxds = 0, dyds = 0, dzds = 0;
    double dxdt = 0, dydt = 0, dzdt = 0;
    
    for(int i = 0; i < 8; ++i) {
      dxdr += dNdr[i] * vertices[i][0];
      dydr += dNdr[i] * vertices[i][1];
      dzdr += dNdr[i] * vertices[i][2];
      
      dxds += dNds[i] * vertices[i][0];
      dyds += dNds[i] * vertices[i][1];
      dzds += dNds[i] * vertices[i][2];
      
      dxdt += dNdt[i] * vertices[i][0];
      dydt += dNdt[i] * vertices[i][1];
      dzdt += dNdt[i] * vertices[i][2];
    }
    
    // Compute determinant of Jacobian matrix
    double det = dxdr * (dyds*dzdt - dydt*dzds)
               - dydr * (dxds*dzdt - dxdt*dzds)
               + dzdr * (dxds*dydt - dxdt*dyds);
    
    return det;
  }

  // ---------------------
  // hex_scaled_jacobian
  // ---------------------
  // Purpose:
  //   Compute scaled Jacobian quality metric for hexahedron.
  //   Range: [-1, 1], where 1 = perfect cube.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 3).
  double hex_scaled_jacobian(const geometry::point_t vertices[8])
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Compute Jacobian at center
    double jac = hex_jacobian(vertices);
    
    // Compute edge length scale
    double max_edge = 0.0;
    for(int i = 0; i < 8; ++i) {
      for(int j = i+1; j < 8; ++j) {
        double dx = vertices[j][0] - vertices[i][0];
        double dy = vertices[j][1] - vertices[i][1];
        double dz = vertices[j][2] - vertices[i][2];
        double len = std::sqrt(dx*dx + dy*dy + dz*dz);
        max_edge = std::max(max_edge, len);
      }
    }
    
    // Scale Jacobian by edge length cubed
    double scale = max_edge * max_edge * max_edge;
    return (scale > 1e-15) ? (jac / scale) : 0.0;
  }

  // ---------------------------
  // compute_tet_quality_stats
  // ---------------------------
  // Purpose:
  //   Compute quality statistics for all tets in a mesh.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 3).
  quality_stats compute_tet_quality_stats(const geometry::tets_t& tets,
                                          const geometry::points_t& vertices,
                                          quality_metric metric)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    if(tets.empty()) {
      return {0.0, 0.0, 0.0, 0.0};
    }
    
    std::vector<double> values;
    values.reserve(tets.size());
    
    for(const auto& tet : tets) {
      double val = 0.0;
      switch(metric) {
        case TET_VOLUME:
          val = std::abs(tet_volume(vertices[tet[0]], vertices[tet[1]], 
                                    vertices[tet[2]], vertices[tet[3]]));
          break;
        case TET_ASPECT_RATIO:
          val = tet_aspect_ratio(vertices[tet[0]], vertices[tet[1]], 
                                vertices[tet[2]], vertices[tet[3]]);
          break;
        case TET_MIN_ANGLE:
          val = tet_min_dihedral_angle(vertices[tet[0]], vertices[tet[1]], 
                                      vertices[tet[2]], vertices[tet[3]]);
          break;
        default:
          val = tet_aspect_ratio(vertices[tet[0]], vertices[tet[1]], 
                                vertices[tet[2]], vertices[tet[3]]);
          break;
      }
      values.push_back(val);
    }
    
    double min_val = *std::min_element(values.begin(), values.end());
    double max_val = *std::max_element(values.begin(), values.end());
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    double mean = sum / values.size();
    
    double sq_sum = 0.0;
    for(double v : values) {
      sq_sum += (v - mean) * (v - mean);
    }
    double std_dev = std::sqrt(sq_sum / values.size());
    
    return {min_val, max_val, mean, std_dev};
  }

  // ---------------------------
  // compute_hex_quality_stats
  // ---------------------------
  // Purpose:
  //   Compute quality statistics for all hexs in a mesh.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 3).
  quality_stats compute_hex_quality_stats(const geometry::hexs_t& hexs,
                                          const geometry::points_t& vertices,
                                          quality_metric metric)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    if(hexs.empty()) {
      return {0.0, 0.0, 0.0, 0.0};
    }
    
    std::vector<double> values;
    values.reserve(hexs.size());
    
    for(const auto& hex : hexs) {
      geometry::point_t hex_verts[8];
      for(int i = 0; i < 8; ++i) {
        hex_verts[i] = vertices[hex[i]];
      }
      
      double val = 0.0;
      switch(metric) {
        case HEX_VOLUME:
          val = hex_volume(hex_verts);
          break;
        case HEX_JACOBIAN:
          val = hex_jacobian(hex_verts);
          break;
        case HEX_SCALED_JACOBIAN:
          val = hex_scaled_jacobian(hex_verts);
          break;
        default:
          val = hex_scaled_jacobian(hex_verts);
          break;
      }
      values.push_back(val);
    }
    
    double min_val = *std::min_element(values.begin(), values.end());
    double max_val = *std::max_element(values.begin(), values.end());
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    double mean = sum / values.size();
    
    double sq_sum = 0.0;
    for(double v : values) {
      sq_sum += (v - mean) * (v - mean);
    }
    double std_dev = std::sqrt(sq_sum / values.size());
    
    return {min_val, max_val, mean, std_dev};
  }

  // ---------------------------------------
  // Advanced Mesh Utilities
  // ---------------------------------------
  // Purpose:
  //   Advanced operations for volumetric mesh analysis.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 4).

  // -----------------------------
  // find_tets_containing_point
  // -----------------------------
  // Purpose:
  //   Find all tets that contain a given point.
  //   Uses CGAL bbox spatial indexing for O(n) with highly efficient rejection.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 4).
  // 12/28/2025 -- Joe R. -- Added CGAL bbox spatial acceleration.
  std::vector<size_t> find_tets_containing_point(const geometry::point_t& p,
                                                  const geometry::tets_t& tets,
                                                  const geometry::points_t& vertices)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    std::vector<size_t> result;
    
#ifndef DISABLE_CGAL
    // For very large meshes, use CGAL AABB tree with surface triangles for O(log n)
    if(tets.size() > 10000) {
      // Extract surface triangles from tets
      geometry geom;
      geom.points() = vertices;
      geom.tets() = tets;
      geometry surface = extract_surface(geom);
      
      // Build CGAL triangles from surface
      std::vector<Triangle_3> triangles;
      triangles.reserve(surface.const_tris().size());
      
      for(const auto& tri : surface.const_tris()) {
        Point_3 p0(surface.points()[tri[0]][0], surface.points()[tri[0]][1], surface.points()[tri[0]][2]);
        Point_3 p1(surface.points()[tri[1]][0], surface.points()[tri[1]][1], surface.points()[tri[1]][2]);
        Point_3 p2(surface.points()[tri[2]][0], surface.points()[tri[2]][1], surface.points()[tri[2]][2]);
        triangles.emplace_back(p0, p1, p2);
      }
      
      // Build AABB tree
      Tree tree(triangles.begin(), triangles.end());
      tree.accelerate_distance_queries();
      
      Point_3 query(p[0], p[1], p[2]);
      
      // Quick inside/outside test - if outside, return empty
      // Use ray casting: count intersections with surface
      // Even count = outside, odd count = inside
      if(!tree.do_intersect(query)) {
        // Point is far from surface, do bbox test on full mesh
        bounding_box mesh_bbox;
        for(const auto& pt : vertices) {
          bounding_box pt_bbox(pt[0], pt[1], pt[2], pt[0], pt[1], pt[2]);
          mesh_bbox += pt_bbox;
        }
        if(!mesh_bbox.contains(p[0], p[1], p[2])) {
          return result;  // Outside mesh entirely
        }
      }
      
      // Point might be inside - fall through to bbox-accelerated search
      // but with pre-computed bboxes
      std::vector<TetElement> elements;
      elements.reserve(tets.size());
      for(size_t i = 0; i < tets.size(); ++i) {
        elements.emplace_back(i, tets, vertices);
      }
      
      for(const auto& elem : elements) {
        if(elem.bbox.contains(p[0], p[1], p[2])) {
          const auto& tet = tets[elem.index];
          auto weights = tet_barycentric(p, vertices[tet[0]], vertices[tet[1]],
                                        vertices[tet[2]], vertices[tet[3]]);
          
          bool inside = true;
          for(int j = 0; j < 4; ++j) {
            if(weights[j] < -1e-10) {
              inside = false;
              break;
            }
          }
          
          if(inside) {
            result.push_back(elem.index);
          }
        }
      }
      
      return result;
    }
#endif
    
    // Use pre-computed bboxes for large meshes (1000-10000)
    if(tets.size() > 1000) {
      std::vector<TetElement> elements;
      elements.reserve(tets.size());
      for(size_t i = 0; i < tets.size(); ++i) {
        elements.emplace_back(i, tets, vertices);
      }
      
      for(const auto& elem : elements) {
        if(elem.bbox.contains(p[0], p[1], p[2])) {
          const auto& tet = tets[elem.index];
          auto weights = tet_barycentric(p, vertices[tet[0]], vertices[tet[1]],
                                        vertices[tet[2]], vertices[tet[3]]);
          
          bool inside = true;
          for(int j = 0; j < 4; ++j) {
            if(weights[j] < -1e-10) {
              inside = false;
              break;
            }
          }
          
          if(inside) {
            result.push_back(elem.index);
          }
        }
      }
      
      return result;
    }
    
    // Bbox-accelerated linear search for medium meshes
    const bool use_bbox_filter = tets.size() > 100;
    
    for(size_t i = 0; i < tets.size(); ++i) {
      const auto& tet = tets[i];
      
      // Quick bounding box rejection test for large meshes
      if(use_bbox_filter) {
        double min_x = vertices[tet[0]][0], max_x = vertices[tet[0]][0];
        double min_y = vertices[tet[0]][1], max_y = vertices[tet[0]][1];
        double min_z = vertices[tet[0]][2], max_z = vertices[tet[0]][2];
        
        for(int j = 1; j < 4; ++j) {
          min_x = std::min(min_x, vertices[tet[j]][0]);
          max_x = std::max(max_x, vertices[tet[j]][0]);
          min_y = std::min(min_y, vertices[tet[j]][1]);
          max_y = std::max(max_y, vertices[tet[j]][1]);
          min_z = std::min(min_z, vertices[tet[j]][2]);
          max_z = std::max(max_z, vertices[tet[j]][2]);
        }
        
        // Skip if point is outside tet's bounding box
        if(p[0] < min_x || p[0] > max_x ||
           p[1] < min_y || p[1] > max_y ||
           p[2] < min_z || p[2] > max_z) {
          continue;
        }
      }
      
      // Point is in bbox (or small mesh), do precise barycentric test
      auto weights = tet_barycentric(p, vertices[tet[0]], vertices[tet[1]],
                                    vertices[tet[2]], vertices[tet[3]]);
      
      // Point is inside if all weights are non-negative
      bool inside = true;
      for(int j = 0; j < 4; ++j) {
        if(weights[j] < -1e-10) {  // Small tolerance for numerical errors
          inside = false;
          break;
        }
      }
      
      if(inside) {
        result.push_back(i);
      }
    }
    
    return result;
  }

  // -----------------------------
  // find_hexs_containing_point
  // -----------------------------
  // Purpose:
  //   Find all hexs that contain a given point.
  //   Uses CGAL bbox spatial indexing for O(n) with highly efficient rejection.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 4).
  // 12/28/2025 -- Joe R. -- Added CGAL bbox spatial acceleration.
  std::vector<size_t> find_hexs_containing_point(const geometry::point_t& p,
                                                  const geometry::hexs_t& hexs,
                                                  const geometry::points_t& vertices)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    std::vector<size_t> result;
    
#ifndef DISABLE_CGAL
    // For very large meshes, use CGAL AABB tree with surface triangles
    if(hexs.size() > 10000) {
      // Extract surface triangles from hexs
      geometry geom;
      geom.points() = vertices;
      geom.hexs() = hexs;
      geometry surface = extract_surface(geom);
      
      // Build CGAL triangles
      std::vector<Triangle_3> triangles;
      triangles.reserve(surface.const_tris().size());
      
      for(const auto& tri : surface.const_tris()) {
        Point_3 p0(surface.points()[tri[0]][0], surface.points()[tri[0]][1], surface.points()[tri[0]][2]);
        Point_3 p1(surface.points()[tri[1]][0], surface.points()[tri[1]][1], surface.points()[tri[1]][2]);
        Point_3 p2(surface.points()[tri[2]][0], surface.points()[tri[2]][1], surface.points()[tri[2]][2]);
        triangles.emplace_back(p0, p1, p2);
      }
      
      // Build AABB tree
      Tree tree(triangles.begin(), triangles.end());
      tree.accelerate_distance_queries();
      
      Point_3 query(p[0], p[1], p[2]);
      
      // Quick inside/outside test
      if(!tree.do_intersect(query)) {
        bounding_box mesh_bbox;
        for(const auto& pt : vertices) {
          bounding_box pt_bbox(pt[0], pt[1], pt[2], pt[0], pt[1], pt[2]);
          mesh_bbox += pt_bbox;
        }
        if(!mesh_bbox.contains(p[0], p[1], p[2])) {
          return result;
        }
      }
      
      // Use bbox-accelerated search with pre-computed bboxes
      std::vector<HexElement> elements;
      elements.reserve(hexs.size());
      for(size_t i = 0; i < hexs.size(); ++i) {
        elements.emplace_back(i, hexs, vertices);
      }
      
      for(const auto& elem : elements) {
        if(elem.bbox.contains(p[0], p[1], p[2])) {
          const auto& hex = hexs[elem.index];
          
          geometry::point_t hex_verts[8];
          for(int j = 0; j < 8; ++j) {
            hex_verts[j] = vertices[hex[j]];
          }
          
          auto weights = hex_trilinear(p, hex_verts);
          
          bool inside = true;
          for(int j = 0; j < 8; ++j) {
            if(weights[j] < -0.1 || weights[j] > 1.1) {
              inside = false;
              break;
            }
          }
          
          if(inside) {
            result.push_back(elem.index);
          }
        }
      }
      
      return result;
    }
#endif
    
    // Use pre-computed bboxes for large meshes (1000-10000)
    if(hexs.size() > 1000) {
      std::vector<HexElement> elements;
      elements.reserve(hexs.size());
      for(size_t i = 0; i < hexs.size(); ++i) {
        elements.emplace_back(i, hexs, vertices);
      }
      
      for(const auto& elem : elements) {
        if(elem.bbox.contains(p[0], p[1], p[2])) {
          const auto& hex = hexs[elem.index];
          
          geometry::point_t hex_verts[8];
          for(int j = 0; j < 8; ++j) {
            hex_verts[j] = vertices[hex[j]];
          }
          
          auto weights = hex_trilinear(p, hex_verts);
          
          bool inside = true;
          for(int j = 0; j < 8; ++j) {
            if(weights[j] < -0.1 || weights[j] > 1.1) {
              inside = false;
              break;
            }
          }
          
          if(inside) {
            result.push_back(elem.index);
          }
        }
      }
      
      return result;
    }
    
    // Bbox-accelerated linear search for smaller meshes or when CGAL disabled
    for(size_t i = 0; i < hexs.size(); ++i) {
      const auto& hex = hexs[i];
      
      // Compute hex bounding box for quick rejection
      double min_x = vertices[hex[0]][0], max_x = vertices[hex[0]][0];
      double min_y = vertices[hex[0]][1], max_y = vertices[hex[0]][1];
      double min_z = vertices[hex[0]][2], max_z = vertices[hex[0]][2];
      
      for(int j = 1; j < 8; ++j) {
        min_x = std::min(min_x, vertices[hex[j]][0]);
        max_x = std::max(max_x, vertices[hex[j]][0]);
        min_y = std::min(min_y, vertices[hex[j]][1]);
        max_y = std::max(max_y, vertices[hex[j]][1]);
        min_z = std::min(min_z, vertices[hex[j]][2]);
        max_z = std::max(max_z, vertices[hex[j]][2]);
      }
      
      // Skip if point is outside hex's bounding box
      if(p[0] < min_x || p[0] > max_x ||
         p[1] < min_y || p[1] > max_y ||
         p[2] < min_z || p[2] > max_z) {
        continue;
      }
      
      // Point is in bbox, do precise test with trilinear coordinates
      geometry::point_t hex_verts[8];
      for(int j = 0; j < 8; ++j) {
        hex_verts[j] = vertices[hex[j]];
      }
      
      auto weights = hex_trilinear(p, hex_verts);
      
      // Check if point is inside (all weights should be in reasonable range)
      bool inside = true;
      for(int j = 0; j < 8; ++j) {
        if(weights[j] < -0.1 || weights[j] > 1.1) {  // Tolerance for hex
          inside = false;
          break;
        }
      }
      
      if(inside) {
        result.push_back(i);
      }
    }
    
    return result;
  }

  // ----------------------
  // compute_mesh_bounds
  // ----------------------
  // Purpose:
  //   Compute bounding box of a geometry.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 4).
  std::array<double, 6> compute_mesh_bounds(const geometry& geom)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    if(geom.points().empty()) {
      return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    }
    
    double min_x = geom.points()[0][0];
    double min_y = geom.points()[0][1];
    double min_z = geom.points()[0][2];
    double max_x = min_x;
    double max_y = min_y;
    double max_z = min_z;
    
    for(const auto& p : geom.points()) {
      min_x = std::min(min_x, p[0]);
      min_y = std::min(min_y, p[1]);
      min_z = std::min(min_z, p[2]);
      max_x = std::max(max_x, p[0]);
      max_y = std::max(max_y, p[1]);
      max_z = std::max(max_z, p[2]);
    }
    
    return {{min_x, min_y, min_z, max_x, max_y, max_z}};
  }

  // --------------------------
  // filter_tets_by_quality
  // --------------------------
  // Purpose:
  //   Filter tets by quality threshold.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 4).
  std::vector<size_t> filter_tets_by_quality(const geometry::tets_t& tets,
                                             const geometry::points_t& vertices,
                                             double threshold,
                                             quality_metric metric)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    std::vector<size_t> result;
    
    for(size_t i = 0; i < tets.size(); ++i) {
      const auto& tet = tets[i];
      
      double quality = 0.0;
      bool keep = false;
      
      switch(metric) {
        case TET_ASPECT_RATIO:
          quality = tet_aspect_ratio(vertices[tet[0]], vertices[tet[1]],
                                     vertices[tet[2]], vertices[tet[3]]);
          // Keep if aspect ratio is below threshold (lower is better)
          keep = (quality < threshold);
          break;
        case TET_VOLUME:
          quality = std::abs(tet_volume(vertices[tet[0]], vertices[tet[1]],
                                        vertices[tet[2]], vertices[tet[3]]));
          // Keep if volume is above threshold
          keep = (quality > threshold);
          break;
        case TET_MIN_ANGLE:
          quality = tet_min_dihedral_angle(vertices[tet[0]], vertices[tet[1]],
                                          vertices[tet[2]], vertices[tet[3]]);
          // Keep if min angle is above threshold (higher is better)
          keep = (quality > threshold);
          break;
        default:
          quality = tet_aspect_ratio(vertices[tet[0]], vertices[tet[1]],
                                     vertices[tet[2]], vertices[tet[3]]);
          keep = (quality < threshold);
          break;
      }
      
      if(keep) {
        result.push_back(i);
      }
    }
    
    return result;
  }

  // --------------------------
  // filter_hexs_by_quality
  // --------------------------
  // Purpose:
  //   Filter hexs by quality threshold.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 4).
  std::vector<size_t> filter_hexs_by_quality(const geometry::hexs_t& hexs,
                                             const geometry::points_t& vertices,
                                             double threshold,
                                             quality_metric metric)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    std::vector<size_t> result;
    
    for(size_t i = 0; i < hexs.size(); ++i) {
      const auto& hex = hexs[i];
      
      geometry::point_t hex_verts[8];
      for(int j = 0; j < 8; ++j) {
        hex_verts[j] = vertices[hex[j]];
      }
      
      double quality = 0.0;
      bool keep = false;
      
      switch(metric) {
        case HEX_SCALED_JACOBIAN:
          quality = hex_scaled_jacobian(hex_verts);
          // Keep if scaled Jacobian is above threshold (higher is better)
          keep = (quality > threshold);
          break;
        case HEX_VOLUME:
          quality = hex_volume(hex_verts);
          // Keep if volume is above threshold
          keep = (quality > threshold);
          break;
        case HEX_JACOBIAN:
          quality = hex_jacobian(hex_verts);
          // Keep if Jacobian is above threshold (positive is valid)
          keep = (quality > threshold);
          break;
        default:
          quality = hex_scaled_jacobian(hex_verts);
          keep = (quality > threshold);
          break;
      }
      
      if(keep) {
        result.push_back(i);
      }
    }
    
    return result;
  }

  // ---------------------------
  // extract_quality_elements
  // ---------------------------
  // Purpose:
  //   Create a new geometry with only high-quality elements.
  // ---- Change History ----
  // 12/28/2025 -- Joe R. -- Creation (Week 3 Option 4).
  geometry extract_quality_elements(const geometry& geom,
                                   double threshold,
                                   quality_metric metric)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    
    geometry result;
    
    // Copy all vertex data
    result.points() = geom.points();
    result.normals() = geom.normals();
    result.colors() = geom.colors();
    result.boundary() = geom.boundary();
    result.functions() = geom.functions();
    
    // Filter and copy elements
    if(geom.num_tets() > 0) {
      auto good_tets = filter_tets_by_quality(geom.const_tets(), geom.points(),
                                             threshold, metric);
      for(size_t idx : good_tets) {
        result.tets().push_back(geom.const_tets()[idx]);
      }
    } else if(geom.num_hexs() > 0) {
      auto good_hexs = filter_hexs_by_quality(geom.const_hexs(), geom.points(),
                                             threshold, metric);
      for(size_t idx : good_hexs) {
        result.hexs().push_back(geom.const_hexs()[idx]);
      }
    } else {
      // For surface meshes, just copy elements
      result.tris() = geom.tris();
      result.quads() = geom.quads();
      result.lines() = geom.lines();
    }
    
    return result;
  }

  // ---------------------------------------
  // Procedural Geometry Generation
  // ---------------------------------------
  // Purpose:
  //   Generate parametric primitive geometries with proper normals.
  // ---- Change History ----
  // 01/07/2026 -- Joe R. -- Creation.

  geometry generate_sphere(double cx, double cy, double cz,
                          double radius,
                          int thetaRes,
                          int phiRes)
  {
    geometry geom;
    
    // Add top pole
    geom.points().push_back(point_t{cx, cy + radius, cz});
    geom.normals().push_back(vector_t{0.0, 1.0, 0.0});
    
    // Add middle vertices
    for (int i = 1; i < phiRes; ++i) {
      double phi = M_PI * i / phiRes;
      double y = radius * std::cos(phi);
      double ringRadius = radius * std::sin(phi);
      
      for (int j = 0; j < thetaRes; ++j) {
        double theta = 2.0 * M_PI * j / thetaRes;
        double x = ringRadius * std::cos(theta);
        double z = ringRadius * std::sin(theta);
        
        point_t p{cx + x, cy + y, cz + z};
        vector_t n{x / radius, y / radius, z / radius};
        
        geom.points().push_back(p);
        geom.normals().push_back(n);
      }
    }
    
    // Add bottom pole
    geom.points().push_back(point_t{cx, cy - radius, cz});
    geom.normals().push_back(vector_t{0.0, -1.0, 0.0});
    
    // Generate triangles - top cap
    for (int j = 0; j < thetaRes; ++j) {
      int next = (j + 1) % thetaRes;
      geom.tris().push_back(tri_t{0, 
        static_cast<unsigned int>(1 + j), 
        static_cast<unsigned int>(1 + next)});
    }
    
    // Middle rows
    for (int i = 0; i < phiRes - 2; ++i) {
      int rowStart = 1 + i * thetaRes;
      int nextRowStart = 1 + (i + 1) * thetaRes;
      
      for (int j = 0; j < thetaRes; ++j) {
        int next = (j + 1) % thetaRes;
        
        geom.tris().push_back(tri_t{
          static_cast<unsigned int>(rowStart + j),
          static_cast<unsigned int>(nextRowStart + j),
          static_cast<unsigned int>(nextRowStart + next)});
        geom.tris().push_back(tri_t{
          static_cast<unsigned int>(rowStart + j),
          static_cast<unsigned int>(nextRowStart + next),
          static_cast<unsigned int>(rowStart + next)});
      }
    }
    
    // Bottom cap
    int bottomPole = static_cast<int>(geom.points().size()) - 1;
    int lastRowStart = 1 + (phiRes - 2) * thetaRes;
    for (int j = 0; j < thetaRes; ++j) {
      int next = (j + 1) % thetaRes;
      geom.tris().push_back(tri_t{
        static_cast<unsigned int>(lastRowStart + j),
        static_cast<unsigned int>(bottomPole),
        static_cast<unsigned int>(lastRowStart + next)});
    }
    
    return geom;
  }

  geometry generate_cube(double cx, double cy, double cz,
                        double sizeX, double sizeY, double sizeZ)
  {
    double sx = sizeX / 2.0;
    double sy = sizeY / 2.0;
    double sz = sizeZ / 2.0;
    
    geometry geom;
    
    // 8 corner vertices
    point_t vertices[8] = {
      {cx - sx, cy - sy, cz - sz},  // 0: left-bottom-back
      {cx + sx, cy - sy, cz - sz},  // 1: right-bottom-back
      {cx + sx, cy + sy, cz - sz},  // 2: right-top-back
      {cx - sx, cy + sy, cz - sz},  // 3: left-top-back
      {cx - sx, cy - sy, cz + sz},  // 4: left-bottom-front
      {cx + sx, cy - sy, cz + sz},  // 5: right-bottom-front
      {cx + sx, cy + sy, cz + sz},  // 6: right-top-front
      {cx - sx, cy + sy, cz + sz}   // 7: left-top-front
    };
    
    // Duplicate vertices for proper face normals
    // Front face (+Z)
    geom.points().push_back(vertices[4]); geom.normals().push_back({0, 0, 1});
    geom.points().push_back(vertices[5]); geom.normals().push_back({0, 0, 1});
    geom.points().push_back(vertices[6]); geom.normals().push_back({0, 0, 1});
    geom.points().push_back(vertices[7]); geom.normals().push_back({0, 0, 1});
    geom.tris().push_back({0, 1, 2});
    geom.tris().push_back({0, 2, 3});
    
    // Back face (-Z)
    geom.points().push_back(vertices[1]); geom.normals().push_back({0, 0, -1});
    geom.points().push_back(vertices[0]); geom.normals().push_back({0, 0, -1});
    geom.points().push_back(vertices[3]); geom.normals().push_back({0, 0, -1});
    geom.points().push_back(vertices[2]); geom.normals().push_back({0, 0, -1});
    geom.tris().push_back({4, 5, 6});
    geom.tris().push_back({4, 6, 7});
    
    // Right face (+X)
    geom.points().push_back(vertices[5]); geom.normals().push_back({1, 0, 0});
    geom.points().push_back(vertices[1]); geom.normals().push_back({1, 0, 0});
    geom.points().push_back(vertices[2]); geom.normals().push_back({1, 0, 0});
    geom.points().push_back(vertices[6]); geom.normals().push_back({1, 0, 0});
    geom.tris().push_back({8, 9, 10});
    geom.tris().push_back({8, 10, 11});
    
    // Left face (-X)
    geom.points().push_back(vertices[0]); geom.normals().push_back({-1, 0, 0});
    geom.points().push_back(vertices[4]); geom.normals().push_back({-1, 0, 0});
    geom.points().push_back(vertices[7]); geom.normals().push_back({-1, 0, 0});
    geom.points().push_back(vertices[3]); geom.normals().push_back({-1, 0, 0});
    geom.tris().push_back({12, 13, 14});
    geom.tris().push_back({12, 14, 15});
    
    // Top face (+Y)
    geom.points().push_back(vertices[7]); geom.normals().push_back({0, 1, 0});
    geom.points().push_back(vertices[6]); geom.normals().push_back({0, 1, 0});
    geom.points().push_back(vertices[2]); geom.normals().push_back({0, 1, 0});
    geom.points().push_back(vertices[3]); geom.normals().push_back({0, 1, 0});
    geom.tris().push_back({16, 17, 18});
    geom.tris().push_back({16, 18, 19});
    
    // Bottom face (-Y)
    geom.points().push_back(vertices[0]); geom.normals().push_back({0, -1, 0});
    geom.points().push_back(vertices[1]); geom.normals().push_back({0, -1, 0});
    geom.points().push_back(vertices[5]); geom.normals().push_back({0, -1, 0});
    geom.points().push_back(vertices[4]); geom.normals().push_back({0, -1, 0});
    geom.tris().push_back({20, 21, 22});
    geom.tris().push_back({20, 22, 23});
    
    return geom;
  }

  geometry generate_torus(double cx, double cy, double cz,
                         double majorRadius, double minorRadius,
                         int majorRes,
                         int minorRes)
  {
    geometry geom;
    
    // Generate vertices
    for (int i = 0; i < majorRes; ++i) {
      double theta = 2.0 * M_PI * i / majorRes;
      double cosTheta = std::cos(theta);
      double sinTheta = std::sin(theta);
      
      for (int j = 0; j < minorRes; ++j) {
        double phi = 2.0 * M_PI * j / minorRes;
        double cosPhi = std::cos(phi);
        double sinPhi = std::sin(phi);
        
        // Position
        double x = (majorRadius + minorRadius * cosPhi) * cosTheta;
        double y = minorRadius * sinPhi;
        double z = (majorRadius + minorRadius * cosPhi) * sinTheta;
        
        // Normal
        double nx = cosPhi * cosTheta;
        double ny = sinPhi;
        double nz = cosPhi * sinTheta;
        
        geom.points().push_back(point_t{cx + x, cy + y, cz + z});
        geom.normals().push_back(vector_t{nx, ny, nz});
      }
    }
    
    // Generate triangles
    for (int i = 0; i < majorRes; ++i) {
      int nextI = (i + 1) % majorRes;
      
      for (int j = 0; j < minorRes; ++j) {
        int nextJ = (j + 1) % minorRes;
        
        int v0 = i * minorRes + j;
        int v1 = nextI * minorRes + j;
        int v2 = nextI * minorRes + nextJ;
        int v3 = i * minorRes + nextJ;
        
        geom.tris().push_back(tri_t{
          static_cast<unsigned int>(v0),
          static_cast<unsigned int>(v1),
          static_cast<unsigned int>(v2)});
        geom.tris().push_back(tri_t{
          static_cast<unsigned int>(v0),
          static_cast<unsigned int>(v2),
          static_cast<unsigned int>(v3)});
      }
    }
    
    return geom;
  }

  geometry generate_cone(double cx, double cy, double cz,
                        double radius, double height,
                        int res)
  {
    geometry geom;
    
    double apexY = cy + height / 2.0;
    double baseY = cy - height / 2.0;
    
    // Apex vertex
    geom.points().push_back(point_t{cx, apexY, cz});
    geom.normals().push_back(vector_t{0, 1, 0});
    
    // Side vertices
    double slopeAngle = std::atan2(radius, height);
    double normalY = std::cos(slopeAngle);
    double normalXZ = std::sin(slopeAngle);
    
    for (int i = 0; i < res; ++i) {
      double theta = 2.0 * M_PI * i / res;
      double cosTheta = std::cos(theta);
      double sinTheta = std::sin(theta);
      
      double x = radius * cosTheta;
      double z = radius * sinTheta;
      
      geom.points().push_back(point_t{cx + x, baseY, cz + z});
      geom.normals().push_back(vector_t{normalXZ * cosTheta, normalY, normalXZ * sinTheta});
    }
    
    // Generate side triangles
    for (int i = 0; i < res; ++i) {
      int next = (i + 1) % res;
      geom.tris().push_back(tri_t{
        0,
        static_cast<unsigned int>(1 + next),
        static_cast<unsigned int>(1 + i)});
    }
    
    // Base cap center
    int baseCenterIdx = static_cast<int>(geom.points().size());
    geom.points().push_back(point_t{cx, baseY, cz});
    geom.normals().push_back(vector_t{0, -1, 0});
    
    // Base cap ring vertices
    int baseRingStart = static_cast<int>(geom.points().size());
    for (int i = 0; i < res; ++i) {
      double theta = 2.0 * M_PI * i / res;
      double x = radius * std::cos(theta);
      double z = radius * std::sin(theta);
      
      geom.points().push_back(point_t{cx + x, baseY, cz + z});
      geom.normals().push_back(vector_t{0, -1, 0});
    }
    
    // Generate base cap triangles
    for (int i = 0; i < res; ++i) {
      int next = (i + 1) % res;
      geom.tris().push_back(tri_t{
        static_cast<unsigned int>(baseCenterIdx),
        static_cast<unsigned int>(baseRingStart + i),
        static_cast<unsigned int>(baseRingStart + next)});
    }
    
    return geom;
  }
}
