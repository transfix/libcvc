/*
  Copyright 2008-2012, 2026 The University of Texas at Austin

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

// Vendored from VolumeRover's ContourTiler (John Edwards, UT Austin; transfix/volrover
// src/ContourTiler at 1cf0fbd7) and ported onto libcvc's tiler kernel and cvc::planar
// (VOLROVER-LIBRARY-PORTS-ROADMAP.md §3).

#ifndef __TILES_H__
#define __TILES_H__

#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include "common.h"

namespace cvc::tiler {

class Tiles
{
  // _vertices is a map with key/value of Point_3/size_t
  // where size_t is the index of the point in iteration
  // order.
  //
  // _tiles is a list of iterators, one for each point in
  // the tile.

public:
  typedef boost::tuple<double, double, double> Color;

private:
  typedef std::map<Point_3, std::pair<size_t, Color> > Vertex_list;
  typedef Vertex_list::iterator Vertex_iterator;
  typedef std::vector<Vertex_iterator> Tile_list;
  typedef Tile_list::iterator Tile_iterator;
  typedef Tile_list::const_iterator Tile_const_iterator;

public:
  Tiles() : _vertices(new Vertex_list()), _tiles(new Tile_list()) {}
  ~Tiles() {}

//   std::list<Point_3> vertices() const;
  std::list<std::pair<Point_3, Color> > vertices() const;

  template <typename Output_iterator>
  void vertices(Output_iterator verts) const
  {
    for (Vertex_list::const_iterator it = _vertices->begin(); it != _vertices->end(); ++it)
    {
      *verts = std::make_pair(it->first, it->second.second);
      ++verts;
    }
  }

  std::list<size_t> tile_indices() const;

  void insert(const Point_3& v0, const Point_3& v1, const Point_3& v2);

  void insert(const Point_3& v0, const Point_3& v1, const Point_3& v2, double r, double g, double b);

  void insert(const Tiles& tiles);

  template <typename Output_iterator>
  void as_single_array(Output_iterator tiles) const
  {
    for (Tile_const_iterator it = _tiles->begin(); it != _tiles->end(); ++it)
    {
      *tiles = (*it)->first;
      ++tiles;
    }
  }

  size_t num_verts() const { return _vertices->size(); }
  size_t num_tiles() const { return _tiles->size() / 3; }

private:
  boost::shared_ptr<Vertex_list> _vertices;
  boost::shared_ptr<Tile_list> _tiles;
};

} // namespace cvc::tiler

#endif
