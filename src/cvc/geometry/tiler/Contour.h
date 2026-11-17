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

#ifndef __CONTOUR_H__
#define __CONTOUR_H__

#include <iostream>
#include <vector>
#include <list>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include "common.h"

#include "polygon_utils.h"
#include "Contour_info.h"

namespace cvc::tiler {

/// Walk_direction is in the direction of the contour.  For example,
/// if walk_direction is FORWARD and the contour is CCW, then the
/// direction to walk is CCW.  If walk_direction is backward and the
/// contour is CCW, then the direction to walk is CW.
struct Walk_direction_def
{
  enum type { FORWARD, BACKWARD };
};

typedef Right_enum<Walk_direction_def> Walk_direction;

class Contour_exception : public std::runtime_error
{
public:
  template <typename Info>
  Contour_exception(const std::string& error, const Info& info) : std::runtime_error(error)
  {
    _slice = boost::lexical_cast<std::string>(info.slice());
    _object_name = info.object_name();
  }

  ~Contour_exception() throw() {}

  std::string& slice() { return _slice; }
  const std::string& slice() const { return _slice; }
  std::string& object_name() { return _object_name; }
  const std::string& object_name() const { return _object_name; }

private:
  std::string _slice;
  std::string _object_name;
};

//--------------------------------------------------------
// Contour class
//--------------------------------------------------------
class Contour
{
public:
  typedef Contour_info<Number_type>                     Info;
  typedef Contour                                       Self;
  typedef boost::shared_ptr<Self>                       Handle;
  typedef Polygon_2                Polygon;
  typedef PolygonTraits                                 Kernel;
  typedef Info                                          Info_type;
  typedef PolygonTraits::Point_2                        Point;

private:
  typedef boost::weak_ptr<Self>                         Self_handle;

public:
  //--------------------------------------------------------
  // Lifetime
  //--------------------------------------------------------

  static Handle create(const Polygon& polygon);
  static Handle create(const Polygon& polygon, const Info& info);

  /// Create a copy of this contour
  Handle copy();

  /// Implementation detail.  Do not call.
  void set_self(Self_handle self);

  Contour();
  ~Contour();

private:
  Contour(const Polygon& polygon, const Info& info);

public:

  //--------------------------------------------------------
  // Accessors
  //--------------------------------------------------------

  Polygon& polygon()
  { return _polygon; }
  
  const Polygon& polygon() const
  { return _polygon; }
  
  const size_t size() const
  { return _polygon.size(); }

  Info& info()
  { return _info; }

  const Info& info() const
  { return _info; }

  Number_type slice() const
  { return _info.slice(); }

  //--------------------------------------------------------
  // Modifiers
  //--------------------------------------------------------

  void force_counterclockwise()
  { if (is_clockwise_oriented()) reverse_orientation(); }

  void force_clockwise()
  { if (is_counterclockwise_oriented()) reverse_orientation(); }

  //--------------------------------------------------------
  // Convenience functions
  //--------------------------------------------------------
  
  bool is_counterclockwise_oriented()
  { return _polygon.is_counterclockwise_oriented(); }

  bool is_clockwise_oriented()
  { return _polygon.is_clockwise_oriented(); }

  bool is_collinear_oriented()
  { return _polygon.is_collinear_oriented(); }

  bool has_on_positive_side(Point q)
  { return _polygon.has_on_positive_side(q); }

  bool has_on_negative_side(Point q)
  { return _polygon.has_on_negative_side(q); }

  bool has_on_boundary(Point q)
  { return _polygon.has_on_boundary(q); }

  bool has_on_bounded_side(Point q) 
  { return _polygon.has_on_bounded_side(q); }

  void reverse_orientation()
  { _polygon.reverse_orientation(); }

private:
  Polygon _polygon;
  Info _info;
  Self_handle _self;
};

typedef Contour::Handle Contour_handle;

//-------------------------------------
// Other functions
//-------------------------------------

std::ostream& operator<<(std::ostream& out, const Contour& contour);
std::ostream& operator<<(std::ostream& out, boost::shared_ptr<Contour> contour);

} // namespace cvc::tiler

#endif
