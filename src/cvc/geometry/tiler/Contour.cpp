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

#include "Contour.h"

namespace cvc::tiler {

Contour::Handle Contour::create(const Polygon &polygon) { return create(polygon, Info()); }

Contour::Handle Contour::create(const Polygon &polygon, const Info &info) {
  Handle instance(new Contour(polygon, info));
  // Why can't I access _self from here?  Even though this function
  // is static it seems like I should still be able to...  Ah, the
  // mysteries of life.
  //     instance->_self = Self_handle(instance);
  instance->set_self(Self_handle(instance));
  return instance;
}

/// Create a copy of this contour
Contour::Handle Contour::copy() { return create(polygon(), info()); }

/// Implementation detail.  Do not call.
void Contour::set_self(Self_handle self) {
  if (self.lock().get() != this)
    throw std::logic_error("Illegal self handle");
  _self = self;
}

Contour::Contour() {}

// Defined in print_utils.cpp
std::string pp(const Polygon_2 &polygon);

Contour::Contour(const Polygon &polygon, const Info &info) : _polygon(polygon), _info(info) {

  if (!polygon.is_simple()) {
    throw Contour_exception("Polygon is not simple in contour " + info.object_name(), info);
  }
}

Contour::~Contour() {}

std::ostream &operator<<(std::ostream &out, const Contour &contour) {
  out << contour.polygon();
  return out;
}

std::ostream &operator<<(std::ostream &out, boost::shared_ptr<Contour> contour) {
  out << *contour;
  return out;
}

// std::size_t hash_value(const Contour_handle& contour)
// {
//   return boost::hash_value(contour.get());
// }

// bool operator==(const Contour_handle& contour0, const Contour_handle& contour1)
// {
//   return contour0.get() == contour1.get();
// }

} // namespace cvc::tiler
