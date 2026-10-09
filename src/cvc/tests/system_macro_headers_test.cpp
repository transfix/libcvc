/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Header-hygiene regression: libcvc's public headers must compile when a
// consumer includes system headers FIRST. Those headers define lowercase
// macros that silently rewrite any identifier of the same name, e.g. glibc's
// <signal.h> has `#define si_value _sifields._rt.si_sigval`, which turned the
// parameter in `double to_display(double si_value, dimension d) const;` into a
// syntax error for every TU that included <signal.h> before
// <cvc/core/world_units.h> (seen in volrover3's dialogs against libcvc 3.5.0).
// The test is that this TU compiles; the TEST below only makes it a real
// executable that links and runs.
//
// The system headers stay in their own preprocessor block, ahead of the libcvc
// includes, so include sorting can never move them after the headers they guard.

#ifndef _WIN32
#include <signal.h>    // si_value, si_pid, si_addr, ... (glibc/musl siginfo_t accessors)
#include <sys/param.h> // function-like MIN(a,b) / MAX(a,b)
#include <sys/stat.h>  // st_atime / st_mtime / st_ctime
#include <sys/types.h> // major / minor / makedev on macOS and older glibc
#else
// Same guards the library uses around <windows.h> (see src/cvc/image/magick_io.cpp).
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h> // ERROR, DELETE, IN, OUT, OPTIONAL, near, far, RGB(), ...
#endif

// A representative public header from every module in the cvc target. Not
// included: cvc/gl/* (the optional cvcGL library, needs VTK/OpenGL),
// cvc/volume/hdf5_utils.h (HDF5), cvc/geometry/project_verts.h (CGAL) and
// cvc/utility/cuda_utils.h / cvc/volren/raycaster_cuda.h (CUDA).
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h>
#include <cvc/core/async_task.h>
#include <cvc/core/exception.h>
#include <cvc/core/text.h>
#include <cvc/core/thread_pool.h>
#include <cvc/core/types.h>
#include <cvc/core/world_clock.h>
#include <cvc/core/world_units.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/image/image.h>
#include <cvc/lod/select.h>
#include <cvc/lsys/grammar.h>
#include <cvc/model/model.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/sim_world.h>
#include <cvc/state/state.h>
#include <cvc/utility/utility.h>
#include <cvc/vis/visible_set.h>
#include <cvc/volren/raycaster.h>
#include <cvc/volslice/slicer.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/volume/volume_ops.h>
#include <cvc/world/heightfield.h>
#include <gtest/gtest.h>

TEST(SystemMacroHeaders, WorldUnitsUsableAfterSystemHeaders) {
  cvc::world_units u;
  EXPECT_DOUBLE_EQ(u.to_display(1.5, cvc::world_units::dimension::length), 1.5);
  const cvc::world_units::measurement m = u.format(2500.0, cvc::world_units::dimension::length);
  EXPECT_DOUBLE_EQ(m.value, 2.5);
  EXPECT_EQ(m.unit, "km");
}

TEST(SystemMacroHeaders, UtilityMinMaxUsableAfterSysParam) {
  // <sys/param.h> may define function-like MIN/MAX macros in this TU, so call
  // the cvc templates the macro-proof way.
  EXPECT_EQ((cvc::MIN)(3, 5), 3);
  EXPECT_EQ((cvc::MAX)(3, 5), 5);
}
