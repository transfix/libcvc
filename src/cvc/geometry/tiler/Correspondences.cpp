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

#include "Correspondences.h"

namespace cvc::tiler {

Correspondences::Correspondences() {}

Correspondences::~Correspondences() {}

/// Adds a correspondence between the two given contours
void Correspondences::add(Contour_handle c1, Contour_handle c2) {
  _corr[c1].push_back(c2);
  _corr[c2].push_back(c1);
}

Correspondences::iterator Correspondences::begin(Contour_handle contour) {
  return get(contour).begin();
}

Correspondences::const_iterator Correspondences::begin(Contour_handle contour) const {
  return get(contour).begin();
}

Correspondences::iterator Correspondences::end(Contour_handle contour) {
  return get(contour).end();
}

Correspondences::const_iterator Correspondences::end(Contour_handle contour) const {
  return get(contour).end();
}

size_t Correspondences::count(Contour_handle contour) const { return get(contour).size(); }

const Correspondences::Container &Correspondences::get(Contour_handle contour) const {
  Map::const_iterator it = _corr.find(contour);
  if (it == _corr.end())
    return _empty;
  return it->second;
}

Correspondences::Container &Correspondences::get(Contour_handle contour) {
  Map::iterator it = _corr.find(contour);
  if (it == _corr.end())
    return _empty;
  return it->second;
}

} // namespace cvc::tiler
