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

#ifndef __CORRESPONDENCES_H__
#define __CORRESPONDENCES_H__

#include <list>
#include <boost/unordered_map.hpp>
#include "Contour.h"

namespace cvc::tiler {

class Correspondences
{
private:
  typedef std::list<Contour_handle> Container;
  typedef boost::unordered_map<Contour_handle, Container> Map;

public:
  typedef Container::iterator iterator;
  typedef Container::const_iterator const_iterator;

public:
  Correspondences();
  ~Correspondences();
  
  /// Adds a correspondence between the two given contours
  void add(Contour_handle c1, Contour_handle c2);

  iterator begin(Contour_handle contour);
  const_iterator begin(Contour_handle contour) const;
  iterator end(Contour_handle contour);
  const_iterator end(Contour_handle contour) const;

  size_t count(Contour_handle contour) const;

private:
  Container& get(Contour_handle contour);
  const Container& get(Contour_handle contour) const;

private:
  Map _corr;
  Container _empty;
};


} // namespace cvc::tiler

#endif
