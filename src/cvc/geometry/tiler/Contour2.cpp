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

#include "Contour2.h"

#include "arrange_polygons.h"
#include "boost/lexical_cast.hpp"

namespace cvc::tiler {

Contour2::Handle Contour2::create() { return create(Info()); }

Contour2::Handle Contour2::create(const Info &info) {
  list<Polygon_2> polys;
  return create(polys.begin(), polys.end(), info);
  ;
}

Contour2::Handle Contour2::create(const Polygon_2 &polygon) { return create(polygon, Info()); }

Contour2::Handle Contour2::create(const Polygon_2 &polygon, const Info &info) {
  list<Polygon_2> polys;
  polys.push_back(polygon);
  return create(polys.begin(), polys.end(), info);
}

// /// Create a copy of this contour
// Contour2::Handle Contour2::copy()
// {
//   return create(begin(), end(), info());
// }

/// Implementation detail.  Do not call.
void Contour2::set_self(Self_handle self) {
  if (self.lock().get() != this)
    throw std::logic_error("Illegal self handle");
  _self = self;
}

Contour2::Contour2() {}

// template <typename Poly_iter>
// Contour2::Contour2(Poly_iter begin, Poly_iter end, const Info& info) : _info(info)
// {
//   list<Polygon_with_holes_2> all;
//   arrange_polygons(begin, end, back_inserter(all));
// }

// // template Contour2::Contour2(list<Polygon_2>::const_iterator begin,
// list<Polygon_2>::const_iterator end, const Info& info); template
// Contour2::Contour2(list<Polygon_2>::iterator begin, list<Polygon_2>::iterator end, const Info&
// info);
// // template Contour2::Contour2(vector<Polygon_2>::const_iterator begin,
// vector<Polygon_2>::const_iterator end, const Info& info); template
// Contour2::Contour2(vector<Polygon_2>::iterator begin, vector<Polygon_2>::iterator end, const
// Info& info);

// Contour2::Contour2(const Polygon& polygon, const Info& info) : _polygon(polygon), _info(info)
// {
//   if (!polygon.is_simple())
//     throw Contour_exception("Polygon is not simple in contour " + info.object_name(), info);
// }

template <typename Poly_iter>
Contour2::Contour2(Poly_iter begin, Poly_iter end, const Info &info) : _info(info) {
  try {
    arrange_polygons(begin, end, back_inserter(_polygons));
  } catch (exception &e) {
    throw runtime_error("Error creating contour of component " + info.name() + " at slice " +
                        boost::lexical_cast<string>(info.slice()) + ": " + e.what());
  }
}

template Contour2::Contour2(list<Polygon_2>::iterator begin, list<Polygon_2>::iterator end,
                            const Info &info);
template Contour2::Contour2(list<Polygon_2>::const_iterator begin,
                            list<Polygon_2>::const_iterator end, const Info &info);
template Contour2::Contour2(vector<Polygon_2>::iterator begin, vector<Polygon_2>::iterator end,
                            const Info &info);
template Contour2::Contour2(vector<Polygon_2>::const_iterator begin,
                            vector<Polygon_2>::const_iterator end, const Info &info);

Contour2::~Contour2() {}

// std::ostream& operator<<(std::ostream& out, const Contour2& contour)
// {
//   out << contour.polygon();
//   return out;
// }

// std::ostream& operator<<(std::ostream& out, boost::shared_ptr<Contour2> contour)
// {
//   out << *contour;
//   return out;
// }

// std::size_t hash_value(const Contour2_handle& contour)
// {
//   return boost::hash_value(contour.get());
// }

// bool operator==(const Contour2_handle& contour0, const Contour2_handle& contour1)
// {
//   return contour0.get() == contour1.get();
// }

void Contour2::validate() const {

  for (Polygon_const_iterator it = begin(); it != end(); ++it) {
    const Polygon_with_holes_2 &pwh = *it;
    if (!pwh.outer_boundary().is_simple()) {
      throw logic_error("Contour is not simple: outer boundary of component " +
                        info().object_name() + ": " + pp(pwh.outer_boundary()));
    }
    for (Polygon_with_holes_2::Hole_const_iterator hit = pwh.holes_begin(); hit != pwh.holes_end();
         ++hit) {
      const Polygon_2 &p = *hit;
      if (!p.is_simple()) {
        throw logic_error("Contour is not simple: hole of component " + info().object_name() +
                          ": " + pp(p));
      }
    }
  }
}

} // namespace cvc::tiler
