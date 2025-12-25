#ifndef __MESHER_MESHER_H__
#define __MESHER_MESHER_H__

#include <VolMagickCompat.h>
#include <LBIE_Mesher.h>

namespace LBIE
{
  // Forward declaration for Mesher class enums
  class Mesher;

  // -------
  // do_mesh
  // -------
  // Purpose: 
  //   Main entry point into LBIE meshing.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  // 12/25/2025 -- Joe R. -- Changed extract_method from string to enum.
  geoframe do_mesh(const VolMagick::Volume& vol,
		   float isovalue, float isovalue_in, float err, float err_in,
		   const std::string& meshtype, const std::string& improve_method, const std::string& normaltype, 
		   Mesher::ExtractionMethod extract_method, int improve_iterations, bool dual_contouring,
		   bool verbose = false);

  // ---------------
  // quality_improve
  // ---------------
  // Purpose: 
  //   Main entry point into LBIE mesh quality improvement
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  geoframe quality_improve(const geoframe& g_frame, const std::string& improve_method, int improve_iterations,
			   bool verbose = false);
}

#endif
