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
  class geometry
  {
  public:
    typedef double                         scalar_t;
    typedef uint64_t                       index_t;
    typedef boost::array<scalar_t,3>       point_t;
    typedef boost::array<scalar_t,3>       vector_t;
    typedef vector_t                       normal_t;
    typedef boost::array<scalar_t,3>       color_t;
    typedef boost::array<index_t,2>        line_t;
    typedef boost::array<index_t,3>        tri_t;
    typedef boost::array<index_t,4>        quad_t;

    typedef std::vector<point_t>           points_t;
    typedef boost::dynamic_bitset<>        boundary_t;
    typedef std::vector<vector_t>          normals_t;
    typedef std::vector<color_t>           colors_t;
    typedef std::vector<line_t>            lines_t;
    typedef std::vector<tri_t>             tris_t;
    typedef std::vector<quad_t>            quads_t;

    typedef boost::shared_ptr<points_t>    points_ptr_t;
    typedef boost::shared_ptr<boundary_t>  boundary_ptr_t;
    typedef boost::shared_ptr<normals_t>   normals_ptr_t;
    typedef boost::shared_ptr<colors_t>    colors_ptr_t;
    typedef boost::shared_ptr<lines_t>     lines_ptr_t;
    typedef boost::shared_ptr<tris_t>      tris_ptr_t;
    typedef boost::shared_ptr<quads_t>     quads_ptr_t;

    enum ARRAY_TYPE
      {
        POINTS,
        BOUNDARY,
        NORMALS,
        COLORS,
        LINES,
        TRIS,
        QUADS
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
    lines_t&     lines() { pre_write(LINES); return *_lines; }
    tris_t&      tris() { pre_write(TRIS); return *_tris; }
    quads_t&     quads() { pre_write(QUADS); return *_quads; }

    const points_t&    const_points() const { return *_points; }
    const boundary_t&  const_boundary() const { return *_boundary; }
    const normals_t&   const_normals() const { return *_normals; }
    const colors_t&    const_colors() const { return *_colors; }
    const lines_t&     const_lines() const { return *_lines; }
    const tris_t&      const_tris() const { return *_tris; }
    const quads_t&     const_quads() const { return *_quads; }

    const points_t&    points() const { return const_points(); }
    const boundary_t&  boundary() const { return const_boundary(); }
    const normals_t&   normals() const { return const_normals(); }
    const colors_t&    colors() const { return const_colors(); }
    const lines_t&     lines() const { return const_lines(); }
    const tris_t&      tris() const { return const_tris(); }
    const quads_t&     quads() const { return const_quads(); }
    
    point_t min_point() const;
    point_t max_point() const;
    bounding_box extents() const;

    uint64_t num_points() const;
    uint64_t num_lines() const;
    uint64_t num_tris() const;
    uint64_t num_quads() const;

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
    geometry& smoothing(float delta = 0.1f, bool fix_boundary = false);

    //LBIE mesh quality improvement
    geometry& quality_improve(int iterations, const std::string& improve_method = "geo_flow");

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
    lines_ptr_t     _lines;
    tris_ptr_t      _tris;
    quads_ptr_t     _quads;

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
  typedef geometry::line_t      line_t;
  typedef geometry::tri_t       tri_t;
  typedef geometry::quad_t      quad_t;
  typedef geometry::points_t    points_t;
  typedef geometry::boundary_t  boundary_t;
  typedef geometry::normals_t   normals_t;
  typedef geometry::colors_t    colors_t;
  typedef geometry::lines_t     lines_t;
  typedef geometry::tris_t      tris_t;
  typedef geometry::quads_t     quads_t;
}

#endif
