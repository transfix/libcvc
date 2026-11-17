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

#ifndef __SLICE_H__
#define __SLICE_H__

#include "Contour.h"

#include <boost/unordered_map.hpp>
#include <list>
#include <vector>

namespace cvc::tiler {

/// Represents a slice that contains contours.  Maps component names to
/// a list of contours.
class Slice {
private:
  //   typedef vector<Contour_handle> Contour_container;
  typedef list<Contour_handle> Contour_container;
  typedef boost::unordered_map<std::string, Contour_container> Map;

public:
  typedef Contour_container::iterator Contour_iterator;
  typedef Contour_container::const_iterator Contour_const_iterator;

public:
  Slice() {}
  ~Slice() {}

  void push_back(Contour_handle contour) { _map[contour->info().object_name()].push_back(contour); }

  void push_back(const std::string &component, Contour_handle contour) {
    _map[component].push_back(contour);
  }

  template <typename Contour_iter> void insert(Contour_iter begin, Contour_iter end) {
    for (Contour_iter it = begin; it != end; ++it) {
      _map[(*it)->info().object_name()].push_back(*it);
    }
  }

  template <typename Contour_iter>
  void insert(const std::string &component, Contour_iter begin, Contour_iter end) {
    _map[component].insert(_map[component].end(), begin, end);
  }

  template <typename Contour_iter>
  void replace(const std::string &component, Contour_iter begin, Contour_iter end) {
    _map[component].clear();
    _map[component].insert(_map[component].end(), begin, end);
  }

  void erase(const std::string &component) { _map.erase(component); }

  Contour_iterator begin(const std::string &component) { return _map[component].begin(); }

  Contour_iterator end(const std::string &component) { return _map[component].end(); }

  Contour_const_iterator begin(const std::string &component) const {
    if (_map.find(component) != _map.end())
      return _map.find(component)->second.begin();
    return _empty.begin();
  }

  Contour_const_iterator end(const std::string &component) const {
    if (_map.find(component) != _map.end())
      return _map.find(component)->second.end();
    return _empty.end();
  }

  bool contains(const std::string &component) const { return _map.find(component) != _map.end(); }

  template <typename Out_iter> void components(Out_iter comps) const {
    for (Map::const_iterator it = _map.begin(); it != _map.end(); ++it)
      *comps++ = it->first;
  }

  void remove_collinear(Number_type epsilon);

  /// delta - After calling this function, all contours will be
  /// separated by at least delta.
  ///
  /// This function will fail if any points are close to being collinear.
  /// Be sure to call remove_collinear first.
  void remove_intersections(Number_type delta, tiler_context &ctx);

  void validate();

  bool empty() const { return _map.empty(); }

  Number_type z() const;

  void augment(const boost::unordered_map<Segment_3_undirected, std::list<Point_3>> &edge2points);

  string to_string() const;

private:
  Map _map;
  Contour_container _empty;
};

} // namespace cvc::tiler

#endif
