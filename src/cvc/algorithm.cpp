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
  
  // -----------
  // sdf_library
  // -----------
  // Purpose: 
  //   Interface between the old SDF API and the new one.
  // ---- Change History ----
  // 01/10/2014 -- Joe R. -- Creation.
  CVC_NAMESPACE::volume sdf_library(const CVC_NAMESPACE::geometry& geom,
				    const CVC_NAMESPACE::dimension& dim,
				    const CVC_NAMESPACE::bounding_box& bbox)
  {
    using namespace std;
    using namespace CVC_NAMESPACE;
    const int flipNormals = 0;
    float mins[3] = { bbox[0], bbox[1], bbox[2] };
    float maxs[3] = { bbox[3], bbox[4], bbox[5] };

    //sdf lib only supports cubic sizes
    uint64 size = *max_element(dim.dim_.begin(), dim.dim_.end());

    SDFLibrary::setParameters(size, flipNormals, mins, maxs);

    float* values = 0;
    {
      boost::scoped_array<float> v(new float[geom.num_points()*3]);
      for(int i = 0; i < geom.num_points(); i++)
	for(int j = 0; j < 3; j++)
	  v[i*3+j] = geom.points()[i][j];
      boost::scoped_array<int> t(new int[geom.num_tris()*3]);
      for(int i = 0; i < geom.num_tris(); i++)
	for(int j = 0; j < 3; j++)
	  t[i*3+j] = geom.tris()[i][j];
      values = SDFLibrary::computeSDF(geom.num_points(), v.get(), 
				      geom.num_tris(), t.get());
      if(!values) throw sign_distance_function_error("SDFLibrary::computeSDF() failed");
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
    delete [] values;

    cv.resize(dim);
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
    ret_geom.tris().resize(geo.triangles.size());
    copy(geo.triangles.begin(),
	 geo.triangles.end(),
	 ret_geom.tris().begin());
    ret_geom.quads().resize(geo.quads.size());
    copy(geo.quads.begin(),
	 geo.quads.end(),
	 ret_geom.quads().begin());
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
    ret_geom.triangles.resize(geo.num_tris());
    copy(geo.tris().begin(),
	 geo.tris().end(),
	 ret_geom.triangles.begin());
    ret_geom.quads.resize(geo.num_quads());
    copy(geo.quads().begin(),
	 geo.quads().end(),
	 ret_geom.quads.begin());
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

    get_arg(isovalue, argv, string("isovalue"));
    get_arg(isovalue_in, argv, string("isovalue_in"));
    get_arg(err, argv, string("err"));
    get_arg(err_in, argv, string("err_in"));
    get_arg(operation, argv, string("operation"));
    get_arg(meshtype, argv, string("meshtype"));
    get_arg(improve_method, argv, string("improve_method"));
    get_arg(normaltype, argv, string("normaltype"));
    get_arg(extract_method, argv, string("extract_method"));
    get_arg(dual_contouring, argv, string("dual_contouring"));
    get_arg(improve_iterations, argv, string("improve_iterations"));

    //copy new volume to old volume type for mesher
    VolMagick::Volume old_school_vol(VolMagick::Dimension(vol.voxel_dimensions()),VolMagick::VoxelType(vol.voxelType()),
				     VolMagick::BoundingBox(vol.boundingBox()));
    old_school_vol.data() = vol.data_as_shared_array();

    //do the meshing
    LBIE::geoframe g_frame = LBIE::do_mesh(old_school_vol,
					   isovalue, isovalue_in, err, err_in,
					   meshtype, improve_method, normaltype,
					   extract_method, improve_iterations, dual_contouring);
    
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
    std::string improve_method = "geo_flow";
    int improve_iterations = 1;

    get_arg(improve_method, argv, string("improve_method"));
    get_arg(improve_iterations, argv, string("improve_iterations"));

    return convert(LBIE::quality_improve(convert(geom), improve_method, improve_iterations));
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
  volume sdf(const geometry& geom,
	     const dimension& dim,
	     const bounding_box& bbox)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    volume vol = sdf_library(geom,dim,bbox);
    vol.desc("Signed Distance Function - SDFLibrary");
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
  geometry iso(const volume& vol, double isovalue)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    Arguments args;
    args["isovalue"] = float(isovalue);
    args["improve_iterations"] = int(0);

    //need to test fastcontouring and contour lib
    //args["extract_method"] = std::string("fastcontouring");
    //args["extract_method"] = std::string("libisocontour");

    return cvc_mesher(vol,args);
  }

  // ---------------
  // quality_improve
  // ---------------
  // Purpose: 
  //   Filters geometry via various improvement methods.  Defaults to using geometric flow.
  // ---- Change History ----
  // 01/10/2014 -- Joe R. -- Creation.
  geometry& geometry::quality_improve(int iterations, const std::string& improve_method)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);
    Arguments args;
    args["improve_method"] = improve_method;
    args["improve_iterations"] = iterations;
    *this = cvc_mesher(*this, args);
    return *this;
  }
}
