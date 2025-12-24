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

#include <cvc/namespace.h>
#include <cvc/geometry.h>
#include <cvc/volmagick.h>

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/array.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/cstdint.hpp>

namespace CVC_NAMESPACE
{
  // Enum for selecting SDF algorithm implementation
  enum sdf_algorithm
  {
    SDF_V1,  // Original SDFLibrary implementation (thread-safe, octree-based)
    SDF_V2   // Alternative DistanceTransform implementation (brute-force)
  };

  // ---
  // sdf
  // ---
  // Purpose: 
  //   Compute a signed distance field using the arguments specified.
  // ---- Change History ----
  // 12/29/2013 -- Joe R. -- Creation.
  // 01/08/2014 - removing sdf_method, always using SDFLibrary now
  // 12/23/2025 - adding algorithm selection enum to switch between v1 and v2
  volume sdf(const geometry& geom,
	     /*
	       Dimension of output sdf vol.
	     */
	     const dimension& dim,
	     /*
	       Bounding box of output vol. If default initialized,
	       use extents of geometry.
	     */
	     const bounding_box& bbox = bounding_box(),
	     /*
	       SDF algorithm to use (SDF_V1 or SDF_V2)
	     */
	     sdf_algorithm algorithm = SDF_V1);

  // ---
  // iso
  // ---
  // Purpose: 
  //   Returns geometry representing an isosurface of the specified volume.
  // ---- Change History ----
  // 12/29/2013 -- Joe R. -- Creation.
  // 01/08/2014 -- Joe R. -- Removing color args.
  geometry iso(const volume& vol, double isovalue);

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
  
}

#endif
