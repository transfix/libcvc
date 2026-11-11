/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The transcendental functions a cvc::physics step calls (internal; not installed): det::sin,
// det::tanh and the rest are cvc::algo's (cvc/algo/det_math.h), which give the same bits on
// every platform, so the state_hash reproduces across them. Everything that feeds the hash calls
// these, never std::'s: vehicle.cpp (the steering actuator, Ackermann, the rolling-resistance
// sign), backend_impl.h (Bekker sinkage), field_step.cpp (the fields' implicit drag, a hull's
// drag area) and nav_plant.cpp (a spawned vehicle's rotation, and the pose it reports to the
// navigator).

#ifndef __CVC_PHYSICS_DET_MATH_H__
#define __CVC_PHYSICS_DET_MATH_H__

#include <cvc/algo/det_math.h>

namespace cvc {
namespace physics {
namespace det = ::cvc::algo::det;
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_DET_MATH_H__
