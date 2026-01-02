#ifndef __MESHER_MESHER_H__
#define __MESHER_MESHER_H__

#include <VolMagickCompat.h>
#include <LBIE_Mesher.h>
#include <cvc/geometry.h>
#include <boost/optional.hpp>

namespace LBIE
{
  // Forward declaration for Mesher class enums
  class Mesher;

  // -------
  // do_mesh
  // -------
  // Purpose: 
  //   Main entry point into LBIE meshing (geoframe-based, internal use).
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  // 12/25/2025 -- Joe R. -- Changed extract_method from string to enum.
  // 12/26/2025 -- Joe R. -- Changed improve_method from string to enum.
  // 12/26/2025 -- Joe R. -- Changed meshtype from string to enum.
  // 12/28/2025 -- Joe R. -- Changed normaltype from string to enum.
  geoframe do_mesh(const VolMagick::Volume& vol,
		   float isovalue, float isovalue_in, float err, float err_in,
		   geoframe::GEOTYPE meshtype, Mesher::ImproveMethod improve_method, Mesher::NormalType normaltype, 
		   Mesher::ExtractionMethod extract_method, int improve_iterations, bool dual_contouring,
		   bool verbose = false,
		   boost::optional<const VolMagick::Volume&> propertyVol = boost::none);

  // ---------------
  // quality_improve
  // ---------------
  // Purpose: 
  //   Main entry point into LBIE mesh quality improvement (geoframe-based, internal use).
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  // 12/26/2025 -- Joe R. -- Removed string-based version, use enum only.
  geoframe quality_improve(const geoframe& g_frame, Mesher::ImproveMethod improve_method, int improve_iterations,
			   bool verbose = false);

  // -----------------------------------------------------------------------
  // Week 4: Geometry-based API (PREFERRED for external use)
  // These functions use CVC::geometry at boundaries for modern API
  // -----------------------------------------------------------------------

  // ----------------
  // do_mesh_geometry
  // ----------------
  // Purpose: 
  //   Geometry-based entry point into LBIE meshing.
  //   PREFERRED API for external callers.
  // ---- Change History ----
  // 12/28/2024 -- Joe R. -- Creation (Week 4).
  // 12/28/2025 -- Joe R. -- Changed normaltype from string to enum.
  CVC_NAMESPACE::geometry do_mesh_geometry(
      const VolMagick::Volume& vol,
      float isovalue, float isovalue_in, float err, float err_in,
      CVC_NAMESPACE::geometry::geometry_type geom_type,
      Mesher::ImproveMethod improve_method,
      Mesher::NormalType normaltype, 
      Mesher::ExtractionMethod extract_method,
      int improve_iterations,
      bool dual_contouring,
      bool verbose = false,
      boost::optional<const VolMagick::Volume&> propertyVol = boost::none);

  // ------------------------
  // quality_improve_geometry
  // ------------------------
  // Purpose: 
  //   Geometry-based entry point into LBIE mesh quality improvement.
  //   PREFERRED API for external callers.
  // ---- Change History ----
  // 12/28/2024 -- Joe R. -- Creation (Week 4).
  CVC_NAMESPACE::geometry quality_improve_geometry(
      const CVC_NAMESPACE::geometry& geom,
      Mesher::ImproveMethod improve_method,
      int improve_iterations,
      bool verbose = false);
}

#endif
