/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics backends. Which engines a libcvc build carries is decided at
// configure time (CVC_WITH_JOLT) and asked at run time: no public header names an
// engine, so an engine's compile definitions never reach a consumer.

#ifndef __CVC_PHYSICS_BACKEND_H__
#define __CVC_PHYSICS_BACKEND_H__

#include <cstdint>
#include <string>
#include <vector>

namespace cvc {
namespace physics {

enum class backend : std::uint8_t {
  null, // bookkeeping only: bodies, ticks, save/restore; no gravity, no collision
  jolt, // Jolt Physics
};

// What a backend can do (bit flags).
enum capability : std::uint32_t {
  cap_dynamics = 1u << 0,                   // gravity, forces and a constraint solver
  cap_collision = 1u << 1,                  // contacts and contact events
  cap_heightfield = 1u << 2,                // heightfield shapes
  cap_mesh = 1u << 3,                       // triangle_mesh shapes
  cap_raycast = 1u << 4,                    // world::raycast
  cap_threads = 1u << 5,                    // world_config::threads > 0
  cap_determinism_same_binary = 1u << 6,    // same build, same inputs -> same state
  cap_determinism_cross_platform = 1u << 7, // ... on every platform the build targets
  cap_exact_save_restore = 1u << 8,         // restore + re-run reproduces the original run
};

const char *backend_name(backend b);

// The backends this build carries. null is always present.
std::vector<backend> available_backends();
bool backend_available(backend b);

// The capability bits of a backend in this build (0 when it is not available).
std::uint32_t backend_capabilities(backend b);

// The engine and its build options, e.g. "Jolt 5.6.0 (cross-platform deterministic)".
std::string backend_version(backend b);

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_BACKEND_H__
