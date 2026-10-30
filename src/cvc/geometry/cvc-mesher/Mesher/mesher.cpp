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

  namespace
  {
    enum { MS_SETUP, MS_EXTRACT, MS_IMPROVE_BEGIN, MS_IMPROVE, MS_PROPERTY, MS_FINISH, MS_CONVERT,
	   MS_RELEASE, MS_DONE };

    void print_mesh_params(float isovalue, float isovalue_in, float err, float err_in,
			   geoframe::GEOTYPE meshtype, Mesher::ImproveMethod improve_method,
			   Mesher::NormalType normaltype, int improve_iterations, bool dual_contouring)
    {
      using namespace std;
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
  }

  // ------------
  // mesh_type_for
  // ------------
  // Purpose:
  //   The geoframe mesh type do_mesh_geometry() makes for a geometry type.
  // ---- Change History ----
  // 10/29/2026 -- Joe R. -- Out of do_mesh_geometry, for mesh_stepper.
  geoframe::GEOTYPE mesh_type_for(cvc::geometry::geometry_type geom_type, bool dual_contouring)
  {
    switch(geom_type) {
      case cvc::geometry::SURFACE_TRI:  return geoframe::SINGLE;
      case cvc::geometry::SURFACE_QUAD: return geoframe::QUAD;
      // cvc::tetrahedralize2 asks for the layer between two isovalues as dual_contouring tets:
      // TETRA2, not TETRA's solid on one side of the first isovalue
      case cvc::geometry::VOLUME_TET:   return dual_contouring ? geoframe::TETRA2 : geoframe::TETRA;
      case cvc::geometry::VOLUME_HEX:   return geoframe::HEXA;
      case cvc::geometry::MIXED:        return geoframe::SINGLE;
      default: throw cvc_mesher_exception("invalid geometry type");
    }
  }

  // ------------
  // mesh_stepper
  // ------------
  // Purpose:
  //   do_mesh() and do_mesh_geometry() a slice at a time (see mesher.h).
  // ---- Change History ----
  // 10/29/2026 -- Joe R. -- Creation.
  mesh_stepper::mesh_stepper(const VolMagick::Volume& vol,
			     float isovalue, float isovalue_in, float err, float err_in,
			     geoframe::GEOTYPE meshtype, Mesher::ImproveMethod improve_method,
			     Mesher::NormalType normaltype, Mesher::ExtractionMethod extract_method,
			     int improve_iterations, bool dual_contouring,
			     boost::optional<const VolMagick::Volume&> propertyVol, bool geometry)
    : _vol(vol), _isovalue(isovalue), _isovalue_in(isovalue_in), _err(err), _err_in(err_in),
      _geom_type(-1), _meshtype(meshtype), _improve(improve_method), _normals(normaltype),
      _extract(extract_method), _iterations(improve_iterations), _dual(dual_contouring),
      _geometry(geometry), _stage(MS_SETUP)
  {
    if(propertyVol)
      _prop = propertyVol.get();
  }

  mesh_stepper::mesh_stepper(const VolMagick::Volume& vol,
			     float isovalue, float isovalue_in, float err, float err_in,
			     cvc::geometry::geometry_type geom_type, Mesher::ImproveMethod improve_method,
			     Mesher::NormalType normaltype, Mesher::ExtractionMethod extract_method,
			     int improve_iterations, bool dual_contouring,
			     boost::optional<const VolMagick::Volume&> propertyVol)
    : _vol(vol), _isovalue(isovalue), _isovalue_in(isovalue_in), _err(err), _err_in(err_in),
      _geom_type(int(geom_type)), _meshtype(geoframe::SINGLE), _improve(improve_method),
      _normals(normaltype), _extract(extract_method), _iterations(improve_iterations),
      _dual(dual_contouring), _geometry(true), _stage(MS_SETUP)
  {
    if(propertyVol)
      _prop = propertyVol.get();
  }

  mesh_stepper::~mesh_stepper() {}

  // do_mesh()'s Mesher, extractMesh() and qualityImprove() (Mesher::step()), the property volume's
  // interpolation in one piece, to_geometry() (to_geometry_stepper), and the Mesher let go of a
  // slice at a time.
  bool mesh_stepper::step(std::size_t budget)
  {
    if(budget == 0)
      budget = std::size_t(-1);
    while(budget > 0)
      switch(_stage)
	{
	case MS_SETUP:
	  {
	    if(_geom_type >= 0)
	      _meshtype = mesh_type_for(cvc::geometry::geometry_type(_geom_type), _dual);
	    _mesher.reset(new Mesher(_vol.ctx()));
	    Mesher& mesher = *_mesher;

	    mesher.isovalue(_isovalue);
	    mesher.isovalue_in(_isovalue_in);
	    mesher.err(_err);
	    mesher.err_in(_err_in);

	    // meshtype is already an enum, convert to Mesher::MeshType
	    Mesher::MeshType mt;
	    switch(_meshtype) {
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
	    mesher.extractionMethod(_extract);
	    mesher.improveMethod(_improve);
	    mesher.normalType(_normals);
	    mesher.dual(_dual);
	    mesher.begin_extract_mesh(_vol); //sets the internal geoframe to the extracted mesh
	    _stage = MS_EXTRACT;
	    budget--;
	  }
	  break;
	case MS_EXTRACT:
	  if(_mesher->step(budget))
	    _stage = MS_IMPROVE_BEGIN;
	  break;
	case MS_IMPROVE_BEGIN:
	  _mesher->begin_quality_improve(_iterations);
	  _stage = MS_IMPROVE;
	  break;
	case MS_IMPROVE:
	  if(_mesher->step(budget))
	    _stage = MS_PROPERTY;
	  break;
	case MS_PROPERTY:
	  // If property volume provided, interpolate property values to mesh vertices
	  if(_prop)
	    {
	      _mesher->interpolateProperties(_prop.get());
	      const std::size_t units = std::size_t(_mesher->mesh().numverts)/16 + 1;
	      budget = units >= budget ? 0 : budget - units;
	    }
	  _stage = MS_FINISH;
	  break;
	case MS_FINISH:
	  // BUGFIX: Ensure mesh_type is preserved (sometimes gets cleared)
	  _mesher->mesh().mesh_type = _meshtype;
	  if(_geometry)
	    {
	      _convert.reset(new to_geometry_stepper(_mesher->mesh()));
	      _stage = MS_CONVERT;
	    }
	  else
	    _stage = MS_DONE;
	  break;
	case MS_CONVERT:
	  if(_convert->step(budget))
	    {
	      _out = _convert->geometry();
	      _convert.reset();
	      _mesher->begin_release();
	      _stage = MS_RELEASE;
	    }
	  break;
	case MS_RELEASE:
	  if(_mesher->step(budget))
	    {
	      _mesher.reset();
	      _stage = MS_DONE;
	    }
	  break;
	default:
	  return true;
	}
    return _stage == MS_DONE;
  }

  bool mesh_stepper::done() const { return _stage == MS_DONE; }

  double mesh_stepper::progress() const
  {
    switch(_stage)
      {
      case MS_SETUP: return 0.0;
      case MS_EXTRACT: return 0.8*_mesher->step_progress();
      case MS_IMPROVE_BEGIN: return 0.8;
      case MS_IMPROVE: return 0.8 + 0.15*_mesher->step_progress();
      case MS_PROPERTY: return 0.95;
      case MS_FINISH: return 0.96;
      case MS_CONVERT: return 0.96 + 0.03*_convert->progress();
      case MS_RELEASE: return 0.99 + 0.01*_mesher->step_progress();
      default: return 1.0;
      }
  }

  const char* mesh_stepper::phase() const
  {
    switch(_stage)
      {
      case MS_SETUP: return "preparing";
      case MS_EXTRACT:
      case MS_IMPROVE: return _mesher->step_phase();
      case MS_IMPROVE_BEGIN: return "improving";
      case MS_PROPERTY: return "properties";
      case MS_FINISH:
      case MS_CONVERT:
      case MS_RELEASE: return "output";
      default: return "done";
      }
  }

  cvc::geometry mesh_stepper::take_geometry()
  {
    if(_stage != MS_DONE || !_geometry)
      throw cvc_mesher_exception("mesh_stepper: the geometry is not ready");
    return _out;
  }

  geoframe mesh_stepper::take_geoframe()
  {
    if(_stage != MS_DONE || !_mesher)
      throw cvc_mesher_exception("mesh_stepper: the mesh is not ready");
    geoframe result = _mesher->mesh();
    return result;
  }

  // -------
  // do_mesh
  // -------
  // Purpose:
  //   Main entry point into LBIE meshing.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  // 12/26/2025 -- Joe R. -- Changed improve_method from string to enum.
  // 10/29/2026 -- Joe R. -- Runs a mesh_stepper to the end.
  geoframe do_mesh(const VolMagick::Volume& vol,
		   float isovalue, float isovalue_in, float err, float err_in,
		   geoframe::GEOTYPE meshtype, Mesher::ImproveMethod improve_method, Mesher::NormalType normaltype,
		   Mesher::ExtractionMethod extract_method, int improve_iterations, bool dual_contouring,
		   bool verbose,
		   boost::optional<const VolMagick::Volume&> propertyVol)
  {
    if(verbose)
      print_mesh_params(isovalue, isovalue_in, err, err_in, meshtype, improve_method, normaltype,
			improve_iterations, dual_contouring);

    mesh_stepper stepper(vol, isovalue, isovalue_in, err, err_in, meshtype, improve_method,
			 normaltype, extract_method, improve_iterations, dual_contouring, propertyVol,
			 false);
    stepper.step(0);
    return stepper.take_geoframe();
  }

  // ---------------
  // quality_improve
  // ---------------
  // Purpose: 
  //   Main entry point into LBIE mesh quality improvement
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  // 12/26/2025 -- Joe R. -- Removed string-based version, use enum only.
  geoframe quality_improve(cvc::app& ctx, const geoframe& g_frame, Mesher::ImproveMethod improve_method, int improve_iterations,
			   bool verbose)
  {
    using namespace std;
    LBIE::Mesher mesher(ctx);
    
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
  cvc::geometry do_mesh_geometry(
      const VolMagick::Volume& vol,
      float isovalue, float isovalue_in, float err, float err_in,
      cvc::geometry::geometry_type geom_type,
      Mesher::ImproveMethod improve_method,
      Mesher::NormalType normaltype, 
      Mesher::ExtractionMethod extract_method,
      int improve_iterations,
      bool dual_contouring,
      bool verbose,
      boost::optional<const VolMagick::Volume&> propertyVol)
  {
    if(verbose)
      print_mesh_params(isovalue, isovalue_in, err, err_in, mesh_type_for(geom_type, dual_contouring),
			improve_method, normaltype, improve_iterations, dual_contouring);

    mesh_stepper stepper(vol, isovalue, isovalue_in, err, err_in, geom_type, improve_method,
			 normaltype, extract_method, improve_iterations, dual_contouring, propertyVol);
    stepper.step(0);
    return stepper.take_geometry();
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
  cvc::geometry quality_improve_geometry(
      cvc::app& ctx,
      const cvc::geometry& geom,
      Mesher::ImproveMethod improve_method,
      int improve_iterations,
      bool verbose)
  {
    // Create a working copy of the geometry (needed because we modify it)
    cvc::geometry working_copy(geom);
    
    // Convert geometry to geoframe
    geoframe gf = to_geoframe(working_copy);
    
    // Call existing geoframe-based implementation
    geoframe result = quality_improve(ctx, gf, improve_method, improve_iterations, verbose);
    
    // Convert result back to geometry
    return to_geometry(result);
  }

}
