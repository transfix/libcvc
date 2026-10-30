#ifndef __MESHER_MESHER_H__
#define __MESHER_MESHER_H__

#include <VolMagickCompat.h>
#include <LBIE_Mesher.h>
#include <cvc/geometry/geometry.h>
#include <boost/optional.hpp>
#include <cstddef>
#include <memory>

namespace LBIE
{
  // Forward declaration for Mesher class enums
  class Mesher;
  class to_geometry_stepper;

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
  geoframe quality_improve(cvc::app& ctx, const geoframe& g_frame, Mesher::ImproveMethod improve_method, int improve_iterations,
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
  cvc::geometry do_mesh_geometry(
      const VolMagick::Volume& vol,
      float isovalue, float isovalue_in, float err, float err_in,
      cvc::geometry::geometry_type geom_type,
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
  cvc::geometry quality_improve_geometry(
      cvc::app& ctx,
      const cvc::geometry& geom,
      Mesher::ImproveMethod improve_method,
      int improve_iterations,
      bool verbose = false);

  // -------------
  // mesh_type_for
  // -------------
  // Purpose:
  //   The geoframe mesh type do_mesh_geometry() makes for a geometry type (throws for one it
  //   does not mesh).
  // ---- Change History ----
  // 10/29/2026 -- Joe R. -- Creation.
  geoframe::GEOTYPE mesh_type_for(cvc::geometry::geometry_type geom_type, bool dual_contouring);

  // ------------
  // mesh_stepper
  // ------------
  // Purpose:
  //   do_mesh() and do_mesh_geometry() a slice at a time, for a caller that must not block.
  //   Made with either one's arguments, step(budget) does about `budget` units of the meshing (a
  //   voxel, an octree cell, a leaf's edge, a vertex of a smoothing round: see
  //   Octree::begin_set_volume(); budget 0 runs to the end) and returns true once the mesh is
  //   ready, which take_geometry() (do_mesh_geometry()'s, converted a slice at a time too) or
  //   take_geoframe() (do_mesh()'s, made with geometry false) then gives. The result is theirs
  //   however the work is sliced; step() throws where they would. phase() names the kind of unit
  //   the next step spends (a static string), progress() is in [0,1], 1 once done. The volumes are
  //   held by value (copy-on-write), so the caller's may go away.
  // ---- Change History ----
  // 10/29/2026 -- Joe R. -- Creation.
  class mesh_stepper
  {
  public:
    // do_mesh()'s arguments
    mesh_stepper(const VolMagick::Volume& vol,
		 float isovalue, float isovalue_in, float err, float err_in,
		 geoframe::GEOTYPE meshtype, Mesher::ImproveMethod improve_method,
		 Mesher::NormalType normaltype, Mesher::ExtractionMethod extract_method,
		 int improve_iterations, bool dual_contouring,
		 boost::optional<const VolMagick::Volume&> propertyVol = boost::none,
		 bool geometry = true);
    // do_mesh_geometry()'s arguments
    mesh_stepper(const VolMagick::Volume& vol,
		 float isovalue, float isovalue_in, float err, float err_in,
		 cvc::geometry::geometry_type geom_type, Mesher::ImproveMethod improve_method,
		 Mesher::NormalType normaltype, Mesher::ExtractionMethod extract_method,
		 int improve_iterations, bool dual_contouring,
		 boost::optional<const VolMagick::Volume&> propertyVol = boost::none);
    ~mesh_stepper();
    mesh_stepper(const mesh_stepper&) = delete;
    mesh_stepper& operator=(const mesh_stepper&) = delete;

    bool step(std::size_t budget);
    bool done() const;
    double progress() const;
    const char* phase() const;
    cvc::geometry take_geometry();
    geoframe take_geoframe();

  private:
    VolMagick::Volume _vol;
    boost::optional<VolMagick::Volume> _prop;
    float _isovalue, _isovalue_in, _err, _err_in;
    int _geom_type; // do_mesh_geometry()'s geometry type, converted at the first step; or -1
    geoframe::GEOTYPE _meshtype;
    Mesher::ImproveMethod _improve;
    Mesher::NormalType _normals;
    Mesher::ExtractionMethod _extract;
    int _iterations;
    bool _dual, _geometry;
    int _stage;
    std::unique_ptr<Mesher> _mesher;
    std::unique_ptr<to_geometry_stepper> _convert;
    cvc::geometry _out;
  };
}

#endif
