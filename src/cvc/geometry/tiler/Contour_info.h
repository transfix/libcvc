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

#ifndef __CONTOUR_INFO_H__
#define __CONTOUR_INFO_H__

// #include <SeriesFileReader.h>
#include "common.h"

namespace cvc::tiler {

//--------------------------------------------------------
// Contour_info class
//--------------------------------------------------------
template <typename Slice_t = Number_type>
class Contour_info
{
public:
  typedef Slice_t Slice_type;

public:
  Contour_info() : _slice(0), _name(""), _object_name("") {}
  Contour_info(Slice_t slice, const std::string& name, const std::string& object_name) 
    : _slice(slice), _name(name), _object_name(object_name) {}
//   Contour_info(SurfRecon::CurvePtr cp) : _slice((int)cp->get<0>()), _name(cp->get<3>()), _object_name(cp->get<3>()) {}
  ~Contour_info() {}

  Slice_t& slice() { return _slice; }
  Slice_t slice() const { return _slice; }
  std::string& name() { return _name; }
  const std::string& name() const { return _name; }

  /// The object name is the name of the object of which this
  /// contour is part.  For example, a dendrite d001 is composed of many
  /// contours.  Each of these contours would have "d001" as their object
  /// name.
  std::string& object_name() { return _object_name; }

  /// The object name is the name of the object of which this
  /// contour is part.  For example, a dendrite d001 is composed of many
  /// contours.  Each of these contours would have "d001" as their object
  /// name.
  const std::string& object_name() const { return _object_name; }

private:
  Slice_t _slice;
  std::string _name;
  std::string _object_name;
};

template <typename Slice_t>
std::ostream& operator<<(std::ostream& out, const Contour_info<Slice_t>& info)
{
  out << "\"" << info.name() << "\"  slice " << info.slice();
  return out;
}

template <typename Slice_t>
std::size_t hash_value(const Contour_info<Slice_t>& info)
{
  std::size_t seed = 0;
  boost::hash_combine(seed, info.name());
  boost::hash_combine(seed, info.slice());
  return seed;
}

template <typename Slice_t>
bool operator==(const Contour_info<Slice_t>& a, const Contour_info<Slice_t>& b)
{
  return a.name() == b.name() && a.slice() == b.slice();
}

} // namespace cvc::tiler

#endif
