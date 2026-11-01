/*
  Copyright 2026 The University of Texas at Austin

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

// The lock every caller of the libContour copy vendored in geometry/cvc-mesher/contour holds
// across its libContour calls. libContour is not re-entrant: it keeps process-wide state, among
// it IntTree's global_tree (read by its qsort comparators), rangesweep.cpp's maxqsize and static
// queue, Range's static results, Data's static contour and color variables and the `verbose`
// global. Its callers are the contour spectrum oracle (contour_spectrum_libcontour.cpp) and LBIE's
// LIBISOCONTOUR extraction (LBIE/LBIE_Mesher.cpp). Internal to a CVC_ENABLE_MESHER build of
// libcvc; never installed.

#ifndef CVC_GEOMETRY_CVC_MESHER_LIBCONTOUR_MUTEX_H
#define CVC_GEOMETRY_CVC_MESHER_LIBCONTOUR_MUTEX_H

#include <mutex>

namespace cvc {

std::mutex &libcontour_mutex();

} // namespace cvc

#endif
