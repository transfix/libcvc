/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __CVC_TYPES_H__
#define __CVC_TYPES_H__

#include <cvc/namespace.h>

#include <boost/cstdint.hpp>
#include <boost/signals2.hpp>
#include <boost/function.hpp>
#include <boost/any.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <map>
#include <string>
#include <vector>

#ifndef CVC_VERSION_STRING
#define CVC_VERSION_STRING "1.0.0"
#endif

#define CVC_ENABLE_LOCALE_BOOL
#ifdef CVC_ENABLE_LOCALE_BOOL
#include <iostream>
#endif

namespace CVC_NAMESPACE
{
  typedef boost::int64_t  int64;
  typedef boost::uint64_t uint64;

  enum data_type
    { 
      UChar = 0, 
      UShort, 
      UInt, 
      Float, 
      Double, 
      UInt64,
      Char,
      Int,
      Int64,
      Undefined
    };

  static const unsigned int data_type_sizes[] = 
    { 
      sizeof(unsigned char), 
      sizeof(unsigned short), 
      sizeof(unsigned int), 
      sizeof(float), 
      sizeof(double), 
      sizeof(uint64),
      sizeof(char),
      sizeof(int),
      sizeof(int64),
      0
    };

  static const char * data_type_strings[] = 
    {
      "unsigned char",
      "unsigned short",
      "unsigned int",
      "float",
      "double",
      "uint64",
      "char",
      "int",
      "int64",
      "void"
    };


  //This is to be used with boost::lexical_cast<> like so
  // bool b = boost::lexical_cast< CVC_NAMESPACE::locale_bool >("true");
  //Found here on stack overflow: http://bit.ly/oR1wnk
#ifdef CVC_ENABLE_LOCALE_BOOL
  struct locale_bool {
    bool data;
    locale_bool() {}
    locale_bool( bool data ) : data(data) {}
    operator bool() const { return data; }
    friend std::ostream & operator << ( std::ostream &out, locale_bool b ) {
        out << std::boolalpha << b.data;
        return out;
    }
    friend std::istream & operator >> ( std::istream &in, locale_bool &b ) {
        in >> std::boolalpha >> b.data;
        return in;
    }
  };
#endif

  typedef boost::signals2::signal<void ()>                   signal;
  typedef boost::signals2::signal<void (const std::string&)> map_change_signal;
  typedef std::map<std::string, boost::any>                  data_map;
  typedef std::map<std::string, std::string>                 data_type_name_map;
  typedef std::map<std::string, data_type>                   data_type_enum_map;
  typedef std::map<std::string, std::string>                 property_map;
  typedef boost::shared_ptr<boost::thread>                   thread_ptr;
  typedef std::map<std::string, thread_ptr>                  thread_map;
  typedef std::map<boost::thread::id, double>                thread_progress_map;
  typedef std::map<boost::thread::id, std::string>           thread_key_map;
  typedef std::map<boost::thread::id, std::string>           thread_info_map;
  typedef boost::function<bool (const std::string&)>         data_reader;
  typedef std::vector<data_reader>                           data_reader_collection;
  typedef boost::shared_ptr<boost::mutex>                    mutex_ptr;
  typedef boost::tuple<mutex_ptr,std::string>                mutex_map_element;
  typedef std::map<std::string, mutex_map_element>           mutex_map;

  // Thread pool priority levels
  enum thread_priority
  {
    PRIORITY_LOW = 0,
    PRIORITY_NORMAL = 1,
    PRIORITY_HIGH = 2,
    PRIORITY_CRITICAL = 3
  };

  // Enum for selecting SDF algorithm implementation
  enum sdf_algorithm
  {
    SDF_V1,  // Original SDFLibrary implementation (thread-safe, octree-based)
    SDF_V2   // Alternative DistanceTransform implementation (brute-force)
  };

  // Enum for selecting isosurface extraction method
  // These values match LBIE::Mesher::ExtractionMethod
  enum extraction_method
  {
    DUALLIB = 0,        // Dual contouring library (default)
    FASTCONTOURING = 1, // Fast contouring implementation
    LIBISOCONTOUR = 2   // ISO contouring library
  };

  // Enum for selecting mesh quality improvement method
  // These values match LBIE::Mesher::ImproveMethod
  enum improvement_method
  {
    NO_IMPROVE = 0,     // No improvement (default for fast extraction)
    GEO_FLOW = 1,       // Geometric flow smoothing
    EDGE_CONTRACT = 2,  // Edge contraction
    JOE_LIU = 3,        // Joe-Liu method
    MINIMAL_VOL = 4,    // Minimal volume
    OPTIMIZATION = 5    // Optimization-based improvement
  };

  // Enum for selecting mesh type for volume meshing
  // These values match LBIE::Mesher::MeshType
  enum mesh_type
  {
    SURFACE_MESH = 0,   // Surface triangle mesh (default)
    TETRAHEDRAL = 1,    // Tetrahedral volume mesh
    QUAD_MESH = 2,      // Quadrilateral surface mesh
    HEXAHEDRAL = 3,     // Hexahedral volume mesh
    DUAL_SURFACE = 4,   // Dual surface mesh
    TETRAHEDRAL2 = 5    // Alternative tetrahedral mesh
  };

  // Enum for selecting quality metric type for volumetric meshes
  // Used for quality analysis, filtering, and element extraction
  enum quality_metric
  {
    // Tetrahedral mesh metrics
    TET_VOLUME = 0,         // Volume of tetrahedron
    TET_ASPECT_RATIO = 1,   // Aspect ratio (lower is better)
    TET_MIN_ANGLE = 2,      // Minimum dihedral angle (higher is better)
    
    // Hexahedral mesh metrics
    HEX_VOLUME = 3,         // Volume of hexahedron
    HEX_JACOBIAN = 4,       // Jacobian determinant (positive is valid)
    HEX_SCALED_JACOBIAN = 5 // Scaled Jacobian quality [-1, 1]
  };
}

#endif
