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

#include <cvc/geometry.h>
#include <cvc/geometry_file_io.h>
#include <cvc/utility.h>
#include <cvc/app.h>
#include <cmath>

#ifdef CVC_GEOMETRY_ENABLE_PROJECT
//Requires CGAL
#include <cvc/project_verts.h>
#endif

#include <map>
#include <list>
#include <limits>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/regex.hpp>

//#define CVC_GEOMETRY_CORRECT_INDEX_START

namespace CVC_NAMESPACE
{
  geometry::geometry()
    : _geom_type(SURFACE_TRI), _extents_set(false),
      _ctx(&app::instance())
  {
    for(uint64_t i = 0; i < 3; i++)
      _min[i] = _max[i] = 0.0;

    init_ptrs();
  }

  geometry::geometry(app& ctx)
    : _geom_type(SURFACE_TRI), _extents_set(false),
      _ctx(&ctx)
  {
    for(uint64_t i = 0; i < 3; i++)
      _min[i] = _max[i] = 0.0;

    init_ptrs();
  }

  geometry::geometry(const geometry& geom)
    : _points(geom._points), _boundary(geom._boundary),
      _normals(geom._normals), _colors(geom._colors),
      _curvatures(geom._curvatures), _functions(geom._functions),
      _lines(geom._lines), _tris(geom._tris),
      _quads(geom._quads), _tets(geom._tets), _hexs(geom._hexs),
      _geom_type(geom._geom_type), _extents_set(geom._extents_set),
      _min(geom._min), _max(geom._max),
      _ctx(geom._ctx)
  {
    //make sure all our pointers are valid
    init_ptrs();
  }

  geometry::~geometry() {}

  void geometry::copy(const geometry &geom, bool deepCopy)
  {
    if (deepCopy)
    {
      // Deep copy: allocate new shared_ptr instances with copied data
      _points = points_ptr_t(new points_t(*geom._points));
      _boundary = boundary_ptr_t(new boundary_t(*geom._boundary));
      _normals = normals_ptr_t(new normals_t(*geom._normals));
      _colors = colors_ptr_t(new colors_t(*geom._colors));
      _curvatures = curvatures_ptr_t(new curvatures_t(*geom._curvatures));
      _functions = functions_ptr_t(new functions_t(*geom._functions));
      _lines = lines_ptr_t(new lines_t(*geom._lines));
      _tris = tris_ptr_t(new tris_t(*geom._tris));
      _quads = quads_ptr_t(new quads_t(*geom._quads));
      _tets = tets_ptr_t(new tets_t(*geom._tets));
      _hexs = hexs_ptr_t(new hexs_t(*geom._hexs));
    }
    else
    {
      // Shallow copy: copy-on-write via shared_ptr
      _points = geom._points;
      _boundary = geom._boundary;
      _normals = geom._normals;
      _colors = geom._colors;
      _curvatures = geom._curvatures;
      _functions = geom._functions;
      _lines = geom._lines;
      _tris = geom._tris;
      _quads = geom._quads;
      _tets = geom._tets;
      _hexs = geom._hexs;
    }
    
    _geom_type = geom._geom_type;
    _extents_set = geom._extents_set;
    _min = geom._min;
    _max = geom._max;
    _ctx = geom._ctx;
    //make sure all our pointers are valid
    init_ptrs();
  }

  geometry& geometry::operator=(const geometry& geom)
  {
    copy(geom); return *this;
  }

  point_t geometry::min_point() const
  {
    if(!_extents_set) calc_extents();
    return _min;
  }

  point_t geometry::max_point() const
  {
    if(!_extents_set) calc_extents();
    return _max;
  }

  // 01/11/2014 - Joe R. - Creation.
  bounding_box geometry::extents() const
  {
    point_t min_pt = min_point();
    point_t max_pt = max_point();
    return bounding_box(min_pt[0],min_pt[1],min_pt[2],
			max_pt[0],max_pt[1],max_pt[2]);
  }

  uint64_t geometry::num_points() const
  {
    //TODO: throw an exception if _points.size() != _boundary.size() 
    //!= _normals.size() != _colors.size()
    return _points ? _points->size() : 0;
  }

  uint64_t geometry::num_lines() const
  {
    return _lines ? _lines->size() : 0;
  }

  uint64_t geometry::num_tris() const
  {
    return _tris ? _tris->size() : 0;
  }

  uint64_t geometry::num_quads() const
  {
    return _quads ? _quads->size() : 0;
  }

  uint64_t geometry::num_tets() const
  {
    return _tets ? _tets->size() : 0;
  }

  uint64_t geometry::num_hexs() const
  {
    return _hexs ? _hexs->size() : 0;
  }

  bool geometry::empty() const
  {
    if(num_points() == 0) // ||   (num_lines() == 0 && num_tris() == 0 && num_quads() == 0))
      return true;
    return false;
  }

  geometry& geometry::merge(const geometry& geom)
  {
    using namespace std;

    //append vertex info
    points().insert(points().end(),
                    geom.points().begin(),
                    geom.points().end());
    normals().insert(normals().end(),
                     geom.normals().begin(),
                     geom.normals().end());
    colors().insert(colors().end(),
                    geom.colors().begin(),
                    geom.colors().end());
        
    //we apparently don't have normal iterators for boundary_t :/
    unsigned int old_size = boundary().size();
    boundary().resize(boundary().size() + geom.boundary().size());
    for(unsigned int i = 0; i < geom.boundary().size(); i++)
      boundary()[old_size + i] = geom.boundary()[i];

    //append and modify index info
    lines().insert(lines().end(),
                   geom.lines().begin(),
                   geom.lines().end());
    {
      lines_t::iterator i = lines().begin();
      advance(i, lines().size() - geom.lines().size());
      while(i != lines().end())
        {
          for(line_t::iterator j = i->begin();
              j != i->end();
              j++)
            *j += points().size() - geom.points().size();
          i++;
        }
    }
    
    tris().insert(tris().end(),
                  geom.tris().begin(),
                  geom.tris().end());
    {
      tris_t::iterator i = tris().begin();
      advance(i, tris().size() - geom.tris().size());
      while(i != tris().end())
        {
          for(tri_t::iterator j = i->begin();
              j != i->end();
              j++)
            *j += points().size() - geom.points().size();
          i++;
        }
    }

    quads().insert(quads().end(),
                   geom.quads().begin(),
                   geom.quads().end());
    {
      quads_t::iterator i = quads().begin();
      advance(i, quads().size() - geom.quads().size());
      while(i != quads().end())
        {
          for(quad_t::iterator j = i->begin();
              j != i->end();
              j++)
            *j += points().size() - geom.points().size();
          i++;
        }
    }

    return *this;
  }

  geometry geometry::tri_surface() const
  {
    geometry new_geom;

    new_geom._points = _points;
    new_geom._colors = _colors;

    //if we don't have boundary info, just insert the tris directly
    if(boundary().size() != points().size())
      {
        new_geom._tris = _tris;

        for(quads_t::const_iterator i = quads().begin();
            i != quads().end();
            i++)
          {
            tri_t quad_tris[2];
            quad_tris[0][0] = (*i)[0];
            quad_tris[0][1] = (*i)[1];
            quad_tris[0][2] = (*i)[3];

            quad_tris[1][0] = (*i)[1];
            quad_tris[1][1] = (*i)[2];
            quad_tris[1][2] = (*i)[3];

            new_geom.tris().push_back(quad_tris[0]);
            new_geom.tris().push_back(quad_tris[1]);
          }
      }
    else
      {
        for(tris_t::const_iterator i = tris().begin();
            i != tris().end();
            i++)
          if(boundary()[(*i)[0]] &&
             boundary()[(*i)[1]] &&
             boundary()[(*i)[2]])
            new_geom.tris().push_back(*i);

        for(quads_t::const_iterator i = quads().begin();
            i != quads().end();
            i++)
          if(boundary()[(*i)[0]] &&
             boundary()[(*i)[1]] &&
             boundary()[(*i)[2]] &&
             boundary()[(*i)[3]])
            {
              tri_t quad_tris[2];
              quad_tris[0][0] = (*i)[0];
              quad_tris[0][1] = (*i)[1];
              quad_tris[0][2] = (*i)[3];
                
              quad_tris[1][0] = (*i)[1];
              quad_tris[1][1] = (*i)[2];
              quad_tris[1][2] = (*i)[3];
                
              new_geom.tris().push_back(quad_tris[0]);
              new_geom.tris().push_back(quad_tris[1]);
            }
      }
      
    return new_geom;
  }

  geometry& geometry::calculate_surf_normals()
  {
    using namespace std;
    geometry tri_geom = this->tri_surface();
      
    //find all the triangles that each point is a part of
    map<point_t, list<unsigned int> > neighbor_tris;
    for(tris_t::const_iterator i = tri_geom.const_tris().begin();
        i != tri_geom.const_tris().end();
        i++)
      for(tri_t::const_iterator j = i->begin();
          j != i->end();
          j++)
        neighbor_tris[tri_geom.const_points()[*j]]
          .push_back(distance(tri_geom.const_tris().begin(),i));

    //now iterate over our points, and for each point push back a new normal vector
    normals().clear();
    for(points_t::const_iterator i = const_points().begin();
        i != const_points().end();
        i++)
      {
        vector_t norm = {{ 0.0, 0.0, 0.0 }};
        list<unsigned int> &neighbors = neighbor_tris[*i];
        for(list<unsigned int>::iterator j = neighbors.begin();
            j != neighbors.end();
            j++)
          {
            vector_t cur_tri_norm;
            const tri_t &tri = tri_geom.const_tris()[*j];
            point_t p[3] = { tri_geom.const_points()[tri[0]],
                             tri_geom.const_points()[tri[1]],
                             tri_geom.const_points()[tri[2]] };
            vector_t v1 = {{ p[1][0] - p[0][0],
                             p[1][1] - p[0][1],
                             p[1][2] - p[0][2] }};
            vector_t v2 = {{ p[2][0] - p[0][0], 
                             p[2][1] - p[0][1],
                             p[2][2] - p[0][2] }};
            cross(cur_tri_norm,v1,v2);
            normalize(cur_tri_norm);
            for(int k = 0; k < 3; k++)
              norm[k] += cur_tri_norm[k];
          }
        if(!neighbors.empty())
          for(int j = 0; j < 3; j++)
            norm[j] /= neighbors.size();
        normals().push_back(norm);
      }

    return *this;
  }
  
  geometry& geometry::compute_normals()
  {
    using namespace std;
    
    // Clear existing normals and resize to match points
    normals().clear();
    normals().resize(num_points(), vector_t{{0.0, 0.0, 0.0}});
    
    // Process triangles
    for (size_t i = 0; i < const_tris().size(); ++i) {
      const tri_t& tri = const_tris()[i];
      
      // Validate triangle indices
      if (tri[0] >= num_points() || tri[1] >= num_points() || tri[2] >= num_points()) 
        continue;
      
      const point_t& v0 = const_points()[tri[0]];
      const point_t& v1 = const_points()[tri[1]];
      const point_t& v2 = const_points()[tri[2]];
      
      // Compute face normal using cross product
      vector_t e1 = {{ v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2] }};
      vector_t e2 = {{ v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2] }};
      vector_t face_normal;
      cross(face_normal, e1, e2);
      
      // Add to each vertex normal
      for (int j = 0; j < 3; ++j) {
        normals()[tri[0]][j] += face_normal[j];
        normals()[tri[1]][j] += face_normal[j];
        normals()[tri[2]][j] += face_normal[j];
      }
    }
    
    // Process quads 
    for (size_t i = 0; i < const_quads().size(); ++i) {
      const quad_t& quad = const_quads()[i];
      
      // Validate quad indices
      if (quad[0] >= num_points() || quad[1] >= num_points() || 
          quad[2] >= num_points() || quad[3] >= num_points()) 
        continue;
      
      const point_t& v0 = const_points()[quad[0]];
      const point_t& v1 = const_points()[quad[1]];
      const point_t& v2 = const_points()[quad[2]];
      const point_t& v3 = const_points()[quad[3]];
      
      // Split quad into two triangles and compute normals
      // Triangle 1: v0, v1, v2
      vector_t e1 = {{ v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2] }};
      vector_t e2 = {{ v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2] }};
      vector_t normal1;
      cross(normal1, e1, e2);
      
      // Triangle 2: v0, v2, v3
      e1[0] = v2[0] - v0[0]; e1[1] = v2[1] - v0[1]; e1[2] = v2[2] - v0[2];
      e2[0] = v3[0] - v0[0]; e2[1] = v3[1] - v0[1]; e2[2] = v3[2] - v0[2];
      vector_t normal2;
      cross(normal2, e1, e2);
      
      // Average the two triangle normals
      vector_t avg_normal = {{ (normal1[0] + normal2[0]) * 0.5,
                               (normal1[1] + normal2[1]) * 0.5,
                               (normal1[2] + normal2[2]) * 0.5 }};
      
      // Add to each vertex normal
      for (int j = 0; j < 3; ++j) {
        normals()[quad[0]][j] += avg_normal[j];
        normals()[quad[1]][j] += avg_normal[j];
        normals()[quad[2]][j] += avg_normal[j];
        normals()[quad[3]][j] += avg_normal[j];
      }
    }
    
    // Normalize all vertex normals
    for (size_t i = 0; i < normals().size(); ++i) {
      vector_t& n = normals()[i];
      scalar_t len = sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
      if (len > 1e-9) {
        n[0] /= len;
        n[1] /= len;
        n[2] /= len;
      } else {
        // Degenerate case - set to default upward normal
        n[0] = 0.0; n[1] = 0.0; n[2] = 1.0;
      }
    }
    
    return *this;
  }
  
  geometry geometry::generate_wire_interior() const
  {
    geometry new_geom(*this);

    if(!boundary().empty() && 
       boundary().size() == points().size())
      {
        //tet mesh
        if(!tris().empty())
          {
            new_geom.tris().clear();
            for(tris_t::const_iterator j = tris().begin();
                j != tris().end();
                j++)
              {
                //add the tri face boundary lines if both verts
                //of the potential line to be added do not lie 
                //on the boundary
                if(!boundary()[(*j)[0]] || !boundary()[(*j)[1]])
                  {
                    line_t line = {{ (*j)[0], (*j)[1] }};
                    new_geom.lines().push_back(line);
                  }
                if(!boundary()[(*j)[1]] || !boundary()[(*j)[2]])
                  {
                    line_t line = {{ (*j)[1], (*j)[2] }};
                    new_geom.lines().push_back(line);
                  }
                if(!boundary()[(*j)[2]] || !boundary()[(*j)[0]])
                  {
                    line_t line = {{ (*j)[2], (*j)[0] }};
                    new_geom.lines().push_back(line);
                  }
                  
                //add the triangle to new_geom if all verts are on the boundary
                if(boundary()[(*j)[0]] && boundary()[(*j)[1]] && boundary()[(*j)[2]])
                  new_geom.tris().push_back(*j);
              }
          }
        //hex mesh
        if(!quads().empty())
          {
            new_geom.quads().clear();
            for(quads_t::const_iterator j = quads().begin();
                j != quads().end();
                j++)
              {
                //same as comment above except for quads...
                if(!boundary()[(*j)[0]] || !boundary()[(*j)[1]])
                  {
                    line_t line = {{ (*j)[0], (*j)[1] }};
                    new_geom.lines().push_back(line);
                  }
                if(!boundary()[(*j)[1]] || !boundary()[(*j)[2]])
                  {
                    line_t line = {{ (*j)[1], (*j)[2] }};
                    new_geom.lines().push_back(line);
                  }
                if(!boundary()[(*j)[2]] || !boundary()[(*j)[3]])
                  {
                    line_t line = {{ (*j)[2], (*j)[3] }};
                    new_geom.lines().push_back(line);
                  }
                if(!boundary()[(*j)[3]] || !boundary()[(*j)[0]])
                  {
                    line_t line = {{ (*j)[3], (*j)[0] }};
                    new_geom.lines().push_back(line);
                  }
                  
                if(boundary()[(*j)[0]] && boundary()[(*j)[1]] && 
                   boundary()[(*j)[2]] && boundary()[(*j)[3]])
                  new_geom.quads().push_back(*j);
              }
          }
      }

    return new_geom;
  }
  
  geometry& geometry::invert_normals()
  {
    for(normals_t::iterator i = normals().begin();
        i != normals().end();
        i++)
      for(int j = 0; j < 3; j++)
        (*i)[j] *= -1.0;
    return *this;
  }

  geometry& geometry::reorient()
  {
    using namespace std;

    //TODO: make this work for quads

    if(!tris().empty())
      {
        //find all the triangles that each point is a part of
        map<point_t, list<unsigned int> > neighbor_tris;
        for(tris_t::iterator i = tris().begin();
            i != tris().end();
            i++)
          for(tri_t::iterator j = i->begin();
              j != i->end();
              j++)
            neighbor_tris[points()[*j]]
              .push_back(distance(tris().begin(),i));
          
        /*
          IGNORE THIS
            
          for each triangle (i),
          for each point (j) in that triangle (i)
          for each triangle (k) that point (j) is a part of
          if the angle between the normal of k and i is obtuse,
          reverse the vertex order of k
        */
          
        //if we don't have normals, lets calculate them
        if(points().size() != normals().size())
          calculate_surf_normals();
          
        for(points_t::iterator i = points().begin();
            i != points().end();
            i++)
          {
            //skip non boundary verts because their normals are null
            if(!boundary()[distance(points().begin(),i)]) continue;
              
            list<unsigned int> &neighbors = neighbor_tris[*i];
            for(list<unsigned int>::iterator j = neighbors.begin();
                j != neighbors.end();
                j++)
              {
                tri_t &tri = tris()[*j];
                for(int k=0; k<3; k++)
                  if(dot(normals()[tri[k]],
                         normals()[distance(points().begin(),i)]) < 0)
                    for(int l=0; l<3; l++)
                      normals()[tri[k]][l] *= -1.0;
              }
          }
      }
      
    return *this;
  }

  geometry& geometry::clear()
  {
    return (*this = geometry());
  }

  geometry& geometry::project(const geometry& input)
  {
#ifdef CVC_GEOMETRY_ENABLE_PROJECT
    geometry ref = input.tri_surface();
    project_verts::project(points().begin(),
                           points().end(),
                           ref.const_points().begin(),
                           ref.const_points().end(),
                           ref.const_tris().begin(),
                           ref.const_tris().end());
#endif
    return *this;
  }

  void geometry::init_ptrs()
  {
    if(!_points)
      _points.reset(new points_t);
    if(!_boundary)
      _boundary.reset(new boundary_t);
    if(!_normals)
      _normals.reset(new normals_t);
    if(!_colors)
      _colors.reset(new colors_t);
    if(!_curvatures)
      _curvatures.reset(new curvatures_t);
    if(!_functions)
      _functions.reset(new functions_t);
    if(!_lines)
      _lines.reset(new lines_t);
    if(!_tris)
      _tris.reset(new tris_t);
    if(!_quads)
      _quads.reset(new quads_t);
    if(!_tets)
      _tets.reset(new tets_t);
    if(!_hexs)
      _hexs.reset(new hexs_t);
  }

  void geometry::calc_extents() const
  {
    using namespace std;
    
    if(empty())
      {
        fill(_min.begin(),_min.end(),numeric_limits<scalar_t>::max());
        fill(_max.begin(),_max.end(),-numeric_limits<scalar_t>::max());
        return;
      }
    
    _min = _max = points()[0];

    for(points_t::const_iterator i = points().begin();
        i != points().end();
        i++)
      {
        if(_min[0] > (*i)[0]) _min[0] = (*i)[0];
        if(_min[1] > (*i)[1]) _min[1] = (*i)[1];
        if(_min[2] > (*i)[2]) _min[2] = (*i)[2];
        if(_max[0] < (*i)[0]) _max[0] = (*i)[0];
        if(_max[1] < (*i)[1]) _max[1] = (*i)[1];
        if(_max[2] < (*i)[2]) _max[2] = (*i)[2];
      }

    _extents_set = true;

    return;
  }

  template<class container_ptr_t>
  void make_unique(container_ptr_t& cp)
  {
    if(!cp.unique())
      {
        container_ptr_t tmp(cp);
        cp.reset(new typename container_ptr_t::element_type(*tmp));
      }
  }

  void geometry::pre_write(ARRAY_TYPE at)
  {
    //invalidate calculated extents
    _extents_set = false;

    switch(at)
      {
      case POINTS: make_unique(_points); break;
      case BOUNDARY: make_unique(_boundary); break;
      case NORMALS: make_unique(_normals); break;
      case COLORS: make_unique(_colors); break;
      case CURVATURES: make_unique(_curvatures); break;
      case FUNCTIONS: make_unique(_functions); break;
      case LINES: make_unique(_lines); break;
      case TRIS: make_unique(_tris); break;
      case QUADS: make_unique(_quads); break;
      case TETS: make_unique(_tets); break;
      case HEXS: make_unique(_hexs); break;
      }
  }

  // 01/12/2014 - Joe R. -- calling read_geometry now.
  geometry& geometry::read(const std::string& filename)
  {
    app* saved_ctx = _ctx;
    *this = read_geometry(filename);
    _ctx = saved_ctx;    // preserve the caller's ctx across the read
    return *this;
  }

  // 12/27/2013 -- Joe R. -- Adapted from old io.h header in VolumeRover
  // 01/12/2014 -- Joe R. -- calling write_geometry now.
  void geometry::write(const std::string& filename) const
  {
    write_geometry(*this, filename);
  }

  // arand: written 4-11-2011
  //        directly read a cvc-raw type file into the data structure
  geometry::geometry(const std::string& filename)
    : _geom_type(SURFACE_TRI), _extents_set(false),
      _ctx(&app::instance())
  {
    for(uint64_t i = 0; i < 3; i++)
      _min[i] = _max[i] = 0.0;
    init_ptrs();
    read(filename);
  }

  geometry::geometry(app& ctx, const std::string& filename)
    : _geom_type(SURFACE_TRI), _extents_set(false),
      _ctx(&ctx)
  {
    for(uint64_t i = 0; i < 3; i++)
      _min[i] = _max[i] = 0.0;
    init_ptrs();
    read(filename);
  }
}
