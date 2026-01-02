/*
  Copyright 2008-2011 The University of Texas at Austin

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

#ifndef __CVCGEOM_H__
#define __CVCGEOM_H__

#include <cvc/namespace.h>
#include <cvc/types.h>
#include <cvc/bounding_box.h>

#include <boost/cstdint.hpp>
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/dynamic_bitset.hpp>

#include <vector>

namespace CVC_NAMESPACE
{
  typedef boost::uint64_t uint64_t;

  // --------
  // geometry
  // --------
  // Purpose: 
  //   Standard geometry container for cvc algorithms.
  // ---- Change History ----
  // 07/03/2010 -- Joe R. -- Creation.
  // 04/11/2011 -- arand  -- Directly read a cvc-raw type file into the data structure
  // 11/16/2011 -- arand  -- Added off reader. This is not fully functional but it does
  //                         handle the most common variants of off files...
  // 12/29/2013 -- Joe R. -- Making read_off, read_raw protected, adding read function.
  // 01/11/2014 -- Joe R. -- Added missing line_t at CVC_NAMESPACE
  // 01/12/2014 -- Joe R. -- Moving bunny() to its own bunny_io class.
  // 12/27/2024 -- Joe R. -- Added tetrahedral/hexahedral support, auxiliary data, geometry type tracking
  class geometry
  {
  public:
    typedef double                         scalar_t;
    typedef uint64_t                       index_t;
    typedef boost::array<scalar_t,3>       point_t;
    typedef boost::array<scalar_t,3>       vector_t;
    typedef vector_t                       normal_t;
    typedef boost::array<scalar_t,3>       color_t;
    typedef boost::array<scalar_t,2>       curvature_t;  // Principal curvatures (k1, k2)
    typedef scalar_t                       function_t;   // Scalar function value
    typedef boost::array<index_t,2>        line_t;
    typedef boost::array<index_t,3>        tri_t;
    typedef boost::array<index_t,4>        quad_t;
    typedef boost::array<index_t,4>        tet_t;        // Tetrahedral element (4 vertices)
    typedef boost::array<index_t,8>        hex_t;        // Hexahedral element (8 vertices)

    typedef std::vector<point_t>           points_t;
    typedef boost::dynamic_bitset<>        boundary_t;
    typedef std::vector<vector_t>          normals_t;
    typedef std::vector<color_t>           colors_t;
    typedef std::vector<curvature_t>       curvatures_t;
    typedef std::vector<function_t>        functions_t;
    typedef std::vector<line_t>            lines_t;
    typedef std::vector<tri_t>             tris_t;
    typedef std::vector<quad_t>            quads_t;
    typedef std::vector<tet_t>             tets_t;
    typedef std::vector<hex_t>             hexs_t;

    typedef boost::shared_ptr<points_t>    points_ptr_t;
    typedef boost::shared_ptr<boundary_t>  boundary_ptr_t;
    typedef boost::shared_ptr<normals_t>   normals_ptr_t;
    typedef boost::shared_ptr<colors_t>    colors_ptr_t;
    typedef boost::shared_ptr<curvatures_t> curvatures_ptr_t;
    typedef boost::shared_ptr<functions_t>  functions_ptr_t;
    typedef boost::shared_ptr<lines_t>     lines_ptr_t;
    typedef boost::shared_ptr<tris_t>      tris_ptr_t;
    typedef boost::shared_ptr<quads_t>     quads_ptr_t;
    typedef boost::shared_ptr<tets_t>      tets_ptr_t;
    typedef boost::shared_ptr<hexs_t>      hexs_ptr_t;

    enum geometry_type
      {
        SURFACE_TRI = 0,      // Triangle surface mesh
        SURFACE_QUAD = 1,     // Quad surface mesh
        VOLUME_TET = 2,       // Tetrahedral volume mesh
        VOLUME_HEX = 3,       // Hexahedral volume mesh
        MIXED = 4             // Mixed element types
      };

    enum ARRAY_TYPE
      {
        POINTS,
        BOUNDARY,
        NORMALS,
        COLORS,
        CURVATURES,
        FUNCTIONS,
        LINES,
        TRIS,
        QUADS,
        TETS,
        HEXS
      };

    geometry();
    geometry(const geometry& geom);
    geometry(const std::string & filename);
    ~geometry();

    void copy(const geometry& geom, bool deepCopy = false);
    geometry& operator=(const geometry& geom);

    points_t&    points() { pre_write(POINTS); return *_points; }
    boundary_t&  boundary() { pre_write(BOUNDARY); return *_boundary; }
    normals_t&   normals() { pre_write(NORMALS); return *_normals; }
    colors_t&    colors() { pre_write(COLORS); return *_colors; }
    curvatures_t& curvatures() { pre_write(CURVATURES); return *_curvatures; }
    functions_t& functions() { pre_write(FUNCTIONS); return *_functions; }
    lines_t&     lines() { pre_write(LINES); return *_lines; }
    tris_t&      tris() { pre_write(TRIS); return *_tris; }
    quads_t&     quads() { pre_write(QUADS); return *_quads; }
    tets_t&      tets() { pre_write(TETS); return *_tets; }
    hexs_t&      hexs() { pre_write(HEXS); return *_hexs; }

    const points_t&    const_points() const { return *_points; }
    const boundary_t&  const_boundary() const { return *_boundary; }
    const normals_t&   const_normals() const { return *_normals; }
    const colors_t&    const_colors() const { return *_colors; }
    const curvatures_t& const_curvatures() const { return *_curvatures; }
    const functions_t& const_functions() const { return *_functions; }
    const lines_t&     const_lines() const { return *_lines; }
    const tris_t&      const_tris() const { return *_tris; }
    const quads_t&     const_quads() const { return *_quads; }
    const tets_t&      const_tets() const { return *_tets; }
    const hexs_t&      const_hexs() const { return *_hexs; }

    const points_t&    points() const { return const_points(); }
    const boundary_t&  boundary() const { return const_boundary(); }
    const normals_t&   normals() const { return const_normals(); }
    const colors_t&    colors() const { return const_colors(); }
    const curvatures_t& curvatures() const { return const_curvatures(); }
    const functions_t& functions() const { return const_functions(); }
    const lines_t&     lines() const { return const_lines(); }
    const tris_t&      tris() const { return const_tris(); }
    const quads_t&     quads() const { return const_quads(); }
    const tets_t&      tets() const { return const_tets(); }
    const hexs_t&      hexs() const { return const_hexs(); }
    
    point_t min_point() const;
    point_t max_point() const;
    bounding_box extents() const;

    uint64_t num_points() const;
    uint64_t num_lines() const;
    uint64_t num_tris() const;
    uint64_t num_quads() const;
    uint64_t num_tets() const;
    uint64_t num_hexs() const;

    geometry_type get_geometry_type() const { return _geom_type; }
    void set_geometry_type(geometry_type type) { _geom_type = type; }

    bool empty() const;
    
    geometry& merge(const geometry& geom);
    
    //returns a simple tri surface for the boundary
    //doesn't remove extra non boundary points
    geometry tri_surface() const;

    //calculates normals for boundary vertices
    //sets non boundary vertex normals to 0.0,0.0,0.0 until further notice
    geometry& calculate_surf_normals();

    //This is a little hack to get a simple tetra or hex mesh rendering,
    //using the lines array to draw an internal wireframe
    geometry generate_wire_interior() const;

    //simply inverts all the normals
    geometry& invert_normals();

    //makes normals consistent... TODO: make this actually re-orient triangles
    //(i.e. CCW => CW or CW => CCW depending on which direction we need the normal
    geometry& reorient();

    //Clears this object
    geometry& clear();

    //Projects boundary vertices of this geometry to the input geometry
    geometry& project(const geometry &input);

    //sangmin park's smoothing method
    geometry& smoothing(float delta = 0.1f, bool fix_boundary = false,
                        bool perturb_1 = false, bool geometric_flow = true,
                        bool smoothing_enabled = true, bool perturb_2 = false);

    //LBIE mesh quality improvement
    geometry& quality_improve(int iterations = 1, improvement_method method = GEO_FLOW);

    //read file directly into data structure
    geometry& read(const std::string& filename);

    //write data structure to file
    void write(const std::string& filename) const;

  protected:
    void init_ptrs();
    void calc_extents() const;
    virtual void pre_write(ARRAY_TYPE at);

    void read_raw(const std::string & filename);
    void read_off(const std::string & filename);

    points_ptr_t    _points;
    boundary_ptr_t  _boundary;
    normals_ptr_t   _normals;
    colors_ptr_t    _colors;
    curvatures_ptr_t _curvatures;
    functions_ptr_t  _functions;
    lines_ptr_t     _lines;
    tris_ptr_t      _tris;
    quads_ptr_t     _quads;
    tets_ptr_t      _tets;
    hexs_ptr_t      _hexs;

    geometry_type   _geom_type;

    //calculated on demand even for const, so the following must be mutable.
    mutable bool _extents_set; //if true, the min/max extents are valid
    mutable point_t _min;
    mutable point_t _max;
  };

  typedef geometry::scalar_t    scalar_t;
  typedef geometry::index_t     index_t;
  typedef geometry::vector_t    vector_t;
  typedef geometry::point_t     point_t;
  typedef geometry::color_t     color_t;
  typedef geometry::curvature_t curvature_t;
  typedef geometry::function_t  function_t;
  typedef geometry::line_t      line_t;
  typedef geometry::tri_t       tri_t;
  typedef geometry::quad_t      quad_t;
  typedef geometry::tet_t       tet_t;
  typedef geometry::hex_t       hex_t;
  typedef geometry::points_t    points_t;
  typedef geometry::boundary_t  boundary_t;
  typedef geometry::normals_t   normals_t;
  typedef geometry::colors_t    colors_t;
  typedef geometry::curvatures_t curvatures_t;
  typedef geometry::functions_t  functions_t;
  typedef geometry::lines_t     lines_t;
  typedef geometry::tris_t      tris_t;
  typedef geometry::quads_t     quads_t;
  typedef geometry::tets_t      tets_t;
  typedef geometry::hexs_t      hexs_t;
}

#endif
