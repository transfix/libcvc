#include <VolMagickCompat.h>
#include <mesher.h>
#include <LBIE/geoframe_adapter.h>

#include <iostream>
#include <string>

// Change Log
// ----------
// 01/11/2014 - Joe R. - separating meshing and quality improve operations into their own functions
//                       to make them easy to call from outside.
// 12/28/2024 - Joe R. - Week 4: Added geometry-based API using geoframe_adapter

namespace LBIE
{
  VOLMAGICK_DEF_EXCEPTION(cvc_mesher_exception);

  // -------
  // do_mesh
  // -------
  // Purpose: 
  //   Main entry point into LBIE meshing.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  // 12/26/2025 -- Joe R. -- Changed improve_method from string to enum.
  geoframe do_mesh(const VolMagick::Volume& vol,
		   float isovalue, float isovalue_in, float err, float err_in,
		   geoframe::GEOTYPE meshtype, Mesher::ImproveMethod improve_method, Mesher::NormalType normaltype, 
		   Mesher::ExtractionMethod extract_method, int improve_iterations, bool dual_contouring,
		   bool verbose,
		   boost::optional<const VolMagick::Volume&> propertyVol)
  {
    using namespace std;

    Mesher mesher;

    if(verbose)
      {
	cout << "isovalue: " << isovalue << endl;
	cout << "inner_isovalue: " << isovalue_in << endl;
	cout << "error: " << err << endl;
	cout << "inner_error: " << err_in << endl;
	cout << "mesh_type: " << static_cast<int>(meshtype) << endl;
	cout << "improvement_method: " << static_cast<int>(improve_method) << endl;
	cout << "iterations: " << improve_iterations << endl;
	cout << "normal_type: " << normaltype << endl;
	cout << "dual: " << (dual_contouring ? "true" : "false") << endl;
      }

    mesher.isovalue(isovalue);
    mesher.isovalue_in(isovalue_in);
    mesher.err(err);
    mesher.err_in(err_in);
      
    // meshtype is already an enum, convert to Mesher::MeshType
    Mesher::MeshType mt;
    switch(meshtype) {
      case geoframe::SINGLE: mt = Mesher::SINGLE; break;
      case geoframe::TETRA:  mt = Mesher::TETRA; break;
      case geoframe::QUAD:   mt = Mesher::QUAD; break;
      case geoframe::HEXA:   mt = Mesher::HEXA; break;
      case geoframe::DOUBLE: mt = Mesher::DOUBLE; break;
      case geoframe::TETRA2: mt = Mesher::TETRA2; break;
      default: throw cvc_mesher_exception("invalid mesh type");
    }
    mesher.meshType(mt);

    // extract_method, improve_method, and normaltype are now enums, use them directly
    mesher.extractionMethod(extract_method);
    mesher.improveMethod(improve_method);
    mesher.normalType(normaltype);
    mesher.dual(dual_contouring);
    mesher.extractMesh(vol); //sets the internal geoframe to the extracted mesh
    mesher.qualityImprove(improve_iterations);
    
    // If property volume provided, interpolate property values to mesh vertices
    if(propertyVol) {
      mesher.interpolateProperties(propertyVol.get());
    }
    
    geoframe result = mesher.mesh();
    // BUGFIX: Ensure mesh_type is preserved (sometimes gets cleared)
    result.mesh_type = meshtype;
    return result;
  }

  // ---------------
  // quality_improve
  // ---------------
  // Purpose: 
  //   Main entry point into LBIE mesh quality improvement
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  // 12/26/2025 -- Joe R. -- Removed string-based version, use enum only.
  geoframe quality_improve(const geoframe& g_frame, Mesher::ImproveMethod improve_method, int improve_iterations,
			   bool verbose)
  {
    using namespace std;
    LBIE::Mesher mesher;
    
    if(verbose)
      {
	cout << "improvement_method: " << static_cast<int>(improve_method) << endl;
	cout << "iterations: " << improve_iterations << endl;
      }

    mesher.improveMethod(improve_method);
    mesher.mesh(g_frame);
    mesher.qualityImprove(improve_iterations);
    geoframe result = mesher.mesh();
    // BUGFIX: Preserve mesh_type through quality improvement
    result.mesh_type = g_frame.mesh_type;
    return result;
  }

  // -----------------------------------------------------------------------
  // Week 4: Geometry-based API using geoframe_adapter
  // These functions use CVC::geometry at boundaries for zero-copy semantics
  // -----------------------------------------------------------------------

  // ----------------
  // do_mesh_geometry
  // ----------------
  // Purpose: 
  //   Geometry-based entry point into LBIE meshing.
  //   Uses geoframe_adapter for zero-copy conversion.
  // ---- Change History ----
  // 12/28/2024 -- Joe R. -- Creation (Week 4).
  CVC_NAMESPACE::geometry do_mesh_geometry(
      const VolMagick::Volume& vol,
      float isovalue, float isovalue_in, float err, float err_in,
      CVC_NAMESPACE::geometry::geometry_type geom_type,
      Mesher::ImproveMethod improve_method,
      Mesher::NormalType normaltype, 
      Mesher::ExtractionMethod extract_method,
      int improve_iterations,
      bool dual_contouring,
      bool verbose,
      boost::optional<const VolMagick::Volume&> propertyVol)
  {
    // Convert geometry_type to geoframe::GEOTYPE
    geoframe::GEOTYPE meshtype;
    switch(geom_type) {
      case CVC_NAMESPACE::geometry::SURFACE_TRI:  meshtype = geoframe::SINGLE; break;
      case CVC_NAMESPACE::geometry::SURFACE_QUAD: meshtype = geoframe::QUAD; break;
      case CVC_NAMESPACE::geometry::VOLUME_TET:   meshtype = geoframe::TETRA; break;
      case CVC_NAMESPACE::geometry::VOLUME_HEX:   meshtype = geoframe::HEXA; break;
      case CVC_NAMESPACE::geometry::MIXED:        meshtype = geoframe::SINGLE; break;
      default: throw cvc_mesher_exception("invalid geometry type");
    }
    
    // Call existing geoframe-based implementation
    geoframe result = do_mesh(vol, isovalue, isovalue_in, err, err_in,
                              meshtype, improve_method, normaltype,
                              extract_method, improve_iterations,
                              dual_contouring, verbose, propertyVol);
    
    // Convert result to geometry
    return to_geometry(result);
  }

  // ------------------------
  // quality_improve_geometry
  // ------------------------
  // Purpose: 
  //   Geometry-based entry point into LBIE mesh quality improvement.
  //   Uses geoframe_adapter for one-copy conversion.
  // ---- Change History ----
  // 12/28/2024 -- Joe R. -- Creation (Week 4).
  // 12/28/2025 -- Joe R. -- Fixed: work with copy to avoid temporary reference issues.
  CVC_NAMESPACE::geometry quality_improve_geometry(
      const CVC_NAMESPACE::geometry& geom,
      Mesher::ImproveMethod improve_method,
      int improve_iterations,
      bool verbose)
  {
    // Create a working copy of the geometry (needed because we modify it)
    CVC_NAMESPACE::geometry working_copy(geom);
    
    // Convert geometry to geoframe
    geoframe gf = to_geoframe(working_copy);
    
    // Call existing geoframe-based implementation
    geoframe result = quality_improve(gf, improve_method, improve_iterations, verbose);
    
    // Convert result back to geometry
    return to_geometry(result);
  }

}
