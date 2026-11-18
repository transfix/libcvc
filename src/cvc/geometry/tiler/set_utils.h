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

#ifndef __SET_UTILS_H__
#define __SET_UTILS_H__

#include "common.h"

#include <boost/unordered_set.hpp>

namespace cvc::tiler {

template <typename T, typename Out_iter>
void set_intersection(const boost::unordered_set<T> &a, const boost::unordered_set<T> &b,
                      Out_iter results) {
  typename boost::unordered_set<T>::const_iterator it = a.begin();
  for (; it != a.end(); ++it) {
    if (b.find(*it) != b.end())
      *results++ = *it;
  }
}

template <typename T>
boost::unordered_set<T> set_intersection(const boost::unordered_set<T> &a,
                                         const boost::unordered_set<T> &b) {
  boost::unordered_set<T> c;
  set_intersection(a, b, inserter(c, c.end()));
  return c;
}

template <typename T>
boost::unordered_set<T> set_union(const boost::unordered_set<T> &a,
                                  const boost::unordered_set<T> &b) {
  boost::unordered_set<T> c;
  c.insert(a.begin(), a.end());
  c.insert(b.begin(), b.end());
  return c;
}

template <typename T, typename Out_iter>
void set_union(const boost::unordered_set<T> &a, const boost::unordered_set<T> &b,
               Out_iter results) {
  boost::unordered_set<T> u = set_union(a, b);
  typename boost::unordered_set<T>::const_iterator it = u.begin();
  for (; it != u.end(); ++it) {
    *results++ = *it;
  }
}

} // namespace cvc::tiler

#endif
