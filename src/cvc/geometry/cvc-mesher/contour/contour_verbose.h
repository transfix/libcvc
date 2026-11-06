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

// libContour's diagnostics level. It was a process-wide int (1 unless setVerboseLevel changed
// it), which libcvc's contour spectrum oracle zeroed around each computation and nothing else
// set. As a constant every `if (verbose)` diagnostic stays compiled and off, and libContour keeps
// no state between threads or tests.

#ifndef CONTOUR_VERBOSE_H
#define CONTOUR_VERBOSE_H

constexpr int verbose = 0;

#endif
