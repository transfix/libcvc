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

#ifndef __OTV_PAIR_H__
#define __OTV_PAIR_H__

#include "common.h"

namespace cvc::tiler {

class OTV_pair
{
public:
  OTV_pair() {}

  OTV_pair(const Point_3& u, const Point_3& v)
  {
    _top = u;
    _bottom = v;
    if (u.z() < v.z())
      std::swap(_top, _bottom);
  }

  ~OTV_pair() {}

  const Point_3& top() const { return _top; }
  Point_3& top() { return _top; }
  const Point_3& bottom() const { return _bottom; }
  Point_3& bottom() { return _bottom; }

private:
  Point_3 _top;
  Point_3 _bottom;
};

} // namespace cvc::tiler

#endif
