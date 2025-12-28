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

#include <SDF/SignDistanceFunction/sdfLib.h>

// Include SDF v2 headers
#include <SDF/SignDistanceFunction_v2/DistanceTransform.h>
#include <SDF/SignDistanceFunction_v2/FaceVertSet3D.h>
#include <SDF/SignDistanceFunction_v2/reg3data.h>

// Undef conflicting macros from SDF v2 before including mesher headers
#ifdef MIN
#undef MIN
#endif
#ifdef MAX
#undef MAX
#endif

#include <cvc-mesher/Mesher/mesher.h>

#include <boost/any.hpp>
#include <boost/scoped_array.hpp>

#include <iostream>
#include <cstring>
#include <algorithm>
#include <map>

namespace
{
  CVC_DEF_EXCEPTION(sign_distance_function_error);
  CVC_DEF_EXCEPTION(cvc_mesher_error);
  
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
      
      // Call the new thread-safe API
      values = SDFLibrary::computeSDF_MT(geom.num_points(), v.get(), 
                                          geom.num_tris(), t.get(),
                                          static_cast<int>(size), flipNormalsInt,
                                          mins, maxs);
      if(!values) throw sign_distance_function_error("SDFLibrary::computeSDF_MT() failed");
    }

    volume cv(dimension(size,size,size),Float,bbox);
    float* choppedValues = reinterpret_cast<float*>(*cv);
    {
      int i, j, k;
      int c=0;
      for( i=0; i<=size; i++ )
	for( j=0; j<=size; j++ )
	  for( k=0; k<=size; k++ )
	    if( i!=size && j!=size && k!=size )
	      choppedValues[c++] = values[i*(size+1)*(size+1) + j*(size+1) + k];
    }
    // Smart pointer automatically cleans up values

    // Negate all SDF values if flipNormals is true (inverts inside/outside)
    if (flipNormals) {
      uint64 total_values = size * size * size;
      for (uint64 i = 0; i < total_values; i++) {
        choppedValues[i] = -choppedValues[i];
      }
    }

    // Resize to requested dimensions if different from computed size
    if (dim.xdim != size || dim.ydim != size || dim.zdim != size) {
      cv.resize(dim);
    }
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
    dt.transform();

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
    
    return cv;
  }

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
    std::string operation = "mesh", meshtype = "single", improve_method = "geo_flow", 
      normaltype = "bspline_convolution", extract_method = "duallib";
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
    get_arg(normaltype, argv, string("normaltype"));
    get_arg(extract_method, argv, string("extract_method"));
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

    // Convert string to enum for backward compatibility
    // Only do this if the string parameter was actually provided (i.e., not using defaults)
    if(argv.count("extract_method")) {
      if(extract_method == "fastcontouring") extraction_method_enum = LBIE::Mesher::FASTCONTOURING;
      else if(extract_method == "libisocontour") extraction_method_enum = LBIE::Mesher::LIBISOCONTOUR;
      else if(extract_method == "duallib") extraction_method_enum = LBIE::Mesher::DUALLIB;
    }

    if(argv.count("improve_method")) {
      if(improve_method == "no_improve") improvement_method_enum = LBIE::Mesher::NO_IMPROVE;
      else if(improve_method == "geo_flow") improvement_method_enum = LBIE::Mesher::GEO_FLOW;
      else if(improve_method == "edge_contract") improvement_method_enum = LBIE::Mesher::EDGE_CONTRACT;
      else if(improve_method == "joe_liu") improvement_method_enum = LBIE::Mesher::JOE_LIU;
      else if(improve_method == "minimal_vol") improvement_method_enum = LBIE::Mesher::MINIMAL_VOL;
      else if(improve_method == "optimization") improvement_method_enum = LBIE::Mesher::OPTIMIZATION;
    }

    if(argv.count("meshtype")) {
      if(meshtype == "single") meshtype_enum = LBIE::geoframe::SINGLE;
      else if(meshtype == "tetra") meshtype_enum = LBIE::geoframe::TETRA;
      else if(meshtype == "quad") meshtype_enum = LBIE::geoframe::QUAD;
      else if(meshtype == "hexa") meshtype_enum = LBIE::geoframe::HEXA;
      else if(meshtype == "double") meshtype_enum = LBIE::geoframe::DOUBLE;
      else if(meshtype == "tetra2") meshtype_enum = LBIE::geoframe::TETRA2;
    }

    // Force dual contouring for DOUBLE and TETRA2 mesh types
    if(meshtype_enum == LBIE::geoframe::DOUBLE || meshtype_enum == LBIE::geoframe::TETRA2) {
      dual_contouring = true;
    }

    //do the meshing - mesher now uses CVC volumes directly via compat layer
    LBIE::geoframe g_frame = LBIE::do_mesh(vol,
					   isovalue, isovalue_in, err, err_in,
					   meshtype_enum, improvement_method_enum, normaltype,
					   extraction_method_enum, improve_iterations, dual_contouring,
					   false, propertyVol);
    
    //convert the geoframe back to cvc::geometry and return it
    return convert(g_frame);
  }

  // ----------
  // cvc_mesher
  // ----------
  // Purpose: 
  //   Interface between the old LBIE mesh quality improvement API and the new one.
  // ---- Change History ----
  // 01/10/2014 -- Joe R. -- Creation.
  CVC_NAMESPACE::geometry cvc_mesher(const CVC_NAMESPACE::geometry& geom, Arguments argv)
  {
    using namespace std;
    using namespace CVC_NAMESPACE;
    int improve_iterations = 1;
    LBIE::Mesher::ImproveMethod improvement_method_enum = LBIE::Mesher::GEO_FLOW;

    get_arg(improvement_method_enum, argv, string("improvement_method_enum"));
    get_arg(improve_iterations, argv, string("improve_iterations"));

    return convert(LBIE::quality_improve(convert(geom), improvement_method_enum, improve_iterations));
  }
}

namespace CVC_NAMESPACE
{
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
  geometry iso(const volume& vol, double isovalue, extraction_method method, int improve_iterations,
               boost::optional<const volume&> propertyVol)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    Arguments args;
    args["isovalue"] = float(isovalue);
    args["improve_iterations"] = improve_iterations;
    args["extraction_method_enum"] = static_cast<LBIE::Mesher::ExtractionMethod>(method);
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
  geometry tetrahedralize(const volume& vol, double isovalue,
                          extraction_method method,
                          improvement_method improve_method,
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
  geometry hexahedralize(const volume& vol, double isovalue,
                         extraction_method method,
                         improvement_method improve_method,
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
  geometry tetrahedralize2(const volume& vol, double isovalue,
                           extraction_method method,
                           improvement_method improve_method,
                           int improve_iterations)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    Arguments args;
    args["isovalue"] = float(isovalue);
    args["improve_iterations"] = improve_iterations;
    args["extraction_method_enum"] = static_cast<LBIE::Mesher::ExtractionMethod>(method);
    args["improvement_method_enum"] = static_cast<LBIE::Mesher::ImproveMethod>(improve_method);
    args["meshtype_enum"] = LBIE::geoframe::TETRA2;

    return cvc_mesher(vol,args);
  }

  geometry tetrahedralize2(const volume& vol, double isovalue_outer, double isovalue_inner,
                           extraction_method method,
                           improvement_method improve_method,
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
}

