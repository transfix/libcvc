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

// How the vendored tiler is made resumable (VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.9, §3): as the
// skeleton port is (skeleton/stepping.h, whose step_task, SK_UNIT and SK_RUN it uses). Each long
// stage of a slice pair is a coroutine named <stage>_steps that yields once per contour pair,
// vertex, OTV pair, chord step or untiled region, its result in an out-parameter; the legacy
// function of the same name runs it to the end, so the legacy code and tests call it as before.

#ifndef CVC_GEOMETRY_TILER_STEPPING_H
#define CVC_GEOMETRY_TILER_STEPPING_H

#include "../skeleton/stepping.h"

namespace cvc::tiler {

using cocone::step_task;

// Runs a stage to its end.
inline void run_to_end(step_task t) {
  while (!t.resume()) {
  }
}

} // namespace cvc::tiler

#endif // CVC_GEOMETRY_TILER_STEPPING_H
