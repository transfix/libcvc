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

#ifndef __CONTOUR_UTILS_H__
#define __CONTOUR_UTILS_H__

#include <list>

#include <boost/unordered_map.hpp>


#include "Right_enum.h"
#include "Statistics.h"
#include "polygon_utils.h"
#include "Contour.h"

namespace cvc::tiler {

/// Returns true if the segment intersects any contour's boundary.
/// Precondition: no contours intersect
template <typename ContourIterator>
bool intersects_proper(const Segment_2& segment, ContourIterator contours_begin, ContourIterator contours_end)
{
  for (ContourIterator it = contours_begin; it != contours_end; ++it)
  {
    const Polygon_2& polygon = (*it)->polygon();
    if (intersects_proper(segment, polygon))
      return true;
  }
  return false;
}

/// Returns map<string, Contour>
/// Access contours by ret_map["name"]
template <typename ContourIterator>
boost::unordered_map<std::string, std::list<Contour_handle> > 
contours_by_name(ContourIterator start, ContourIterator end)
{
  typedef std::list<typename std::iterator_traits<ContourIterator>::value_type> List;
  boost::unordered_map<std::string, List> map;

  for (ContourIterator it = start; it != end; ++it)
  {
    std::string name = (*it)->info().name();
    map[name].push_back(*it);
  }
  return map;
}

/// Returns map<size_t, Contour>
/// Access contours by ret_map[slice]
template <typename ContourIterator>
boost::unordered_map<size_t, std::list<Contour_handle> > 
contours_by_slice(ContourIterator start, ContourIterator end)
{
  typedef std::list<typename std::iterator_traits<ContourIterator>::value_type> List;
  boost::unordered_map<size_t, List> map;

  for (ContourIterator it = start; it != end; ++it)
  {
    size_t slice = (*it)->info().slice();
    map[slice].push_back(*it);
  }
  return map;
}

/// Assumes output iterator is associated with a set
template <typename ContourIterator, typename OutputIterator>
void contour_slices(ContourIterator start, ContourIterator end, OutputIterator slices)
{
  for (ContourIterator it = start; it != end; ++it)
  {
    *slices = (*it)->info().slice();
    ++slices;
  }
}

/// Assumes output iterator is associated with a set
template <typename ContourIterator, typename OutputIterator>
void contour_names(ContourIterator start, ContourIterator end, OutputIterator names)
{
  for (ContourIterator it = start; it != end; ++it)
  {
    *names = (*it)->info().name();
    ++names;
  }
}

/// Returns map<string, map<size_t, Contour> >
/// Access contours by ret_map["name"][slice]
template <typename ContourIterator>
boost::unordered_map<std::string, boost::unordered_map<size_t, std::list<Contour_handle> > > 
contours_by_name_slice (ContourIterator start, ContourIterator end)
{
  typedef boost::unordered_map<std::string, std::list<typename std::iterator_traits<ContourIterator>::value_type> > by_name_t;
  typedef boost::unordered_map<std::string, boost::unordered_map<size_t, std::list<typename std::iterator_traits<ContourIterator>::value_type> > > by_name_slice_t;

  by_name_t by_name = contours_by_name(start, end);
  by_name_slice_t by_name_slice;
  
  for (typename by_name_t::iterator it = by_name.begin(); it != by_name.end(); ++it)
  {
    by_name_slice[it->first] = contours_by_slice(it->second.begin(), it->second.end());
  }

  return by_name_slice;
}

/// Returns all contours with the given name
template <typename ContourIterator, typename OutputIterator>
void contours_by_name(ContourIterator start, ContourIterator end, OutputIterator out, const std::string& name)
{
  for (ContourIterator it = start; it != end; ++it)
  {
    std::string n = (*it)->info().name();
    if (n == name)
    {
      *out = *it;
      ++out;
    }
  }
}

/// Returns all contours in the given slice
template <typename ContourIterator, typename OutputIterator>
void contours_by_slice(ContourIterator start, ContourIterator end, OutputIterator out, size_t slice)
{
  for (ContourIterator it = start; it != end; ++it)
  {
    size_t s = (*it)->info().slice();
    if (s == slice)
    {
      *out = *it;
      ++out;
    }
  }
}

size_t vertex_idx(Contour_handle contour, Polygon_2::Vertex_iterator vertex);

size_t vertex_idx(Contour_handle contour, Polygon_2::Vertex_circulator vertex);

template <typename InputIterator, typename OutputIterator>
void get_intersecting_pairs(InputIterator begin, InputIterator end, OutputIterator intersecting_pairs);

} // namespace cvc::tiler

#endif
