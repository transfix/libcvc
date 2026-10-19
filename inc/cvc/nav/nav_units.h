/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav <-> cvc::world_units: the frame a sim_world's numbers live in.
//
// nav's kernels, learned coefficients and .cvcnav weights all work in the
// normalized frame, normalized = (world - centre) * S. frame_of() names that
// frame for a sim_world, so a boundary (a physics plant adapter, a SI readout,
// an SI input builder) converts with world_units::to_normalized /
// from_normalized and the point helpers instead of re-deriving S and the centre.
// Nothing here is called by a kernel: the trained numbers never see it
// (docs/roadmap/NAV_UNITS_ROADMAP.md).

#ifndef __CVC_NAV_NAV_UNITS_H__
#define __CVC_NAV_NAV_UNITS_H__

#include <cvc/core/world_units.h>
#include <cvc/nav/sim_world.h>

namespace cvc {
namespace nav {

// The normalized frame of a sim_world configuration: S = config.scale, centre
// (config.cx, config.cy). nav is planar, so the frame's z centre is 0.
inline cvc::world_units::normalized_frame frame_of(const sim_world::config &c) {
  cvc::world_units::normalized_frame f;
  f.scale = c.scale;
  f.cx = c.cx;
  f.cy = c.cy;
  f.cz = 0.0;
  return f;
}

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_NAV_UNITS_H__
