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
*/

#ifndef __CVC_PHYSICS_PHYSICS_CODEC_H__
#define __CVC_PHYSICS_PHYSICS_CODEC_H__

#include <cvc/core/namespace.h>

namespace cvc {

class app;
class state_codec_registry;

namespace physics {

// ----------------
// cvc::physics — pose / body_state replication codec
// ----------------
// A holon replicates a participant's avatar as a cvc::physics::body_state (or a
// bare cvc::physics::pose) carried in a state node's data(). For that payload to
// cross the cluster, the distributed-state pipeline needs the type to have both a
// registered data-type name and a (de)serialization codec in the shard's codec
// registry -- the same contract cvc::ariadne::stream::format_desc uses.
//
// The wire is little-endian, fixed-width, FIELD-BY-FIELD (never a struct memcpy,
// so trailing-bool padding and ABI differences never leak onto the wire): each
// double is its IEEE-754 binary64 bit pattern as 8 LE bytes (matching the
// built-in "cvc.f64.v1" double codec), and `active` is one byte. So the encoding
// is byte-identical across the (little-endian) fleet, which content-addressed
// blob offload and live/snapshot digest agreement both require.
//
// These helpers bridge cvc::physics types to the cvc::state codec registry; like
// the stream codec they are registered INTO a shard's (mutable) registry rather
// than baked into cvc::state, which sits below physics.

// The canonical, cross-ABI-stable data-type names, used BOTH as the app's
// registerDataType name and the codec-registry key, so dataTypeName<T>() resolves
// to this exact string rather than the compiler's mangled typeid name (an
// unregistered type would fall back to the mangled name, which differs across
// toolchains and would silently fail to replicate).
//
// Returned by an inline accessor (a plain string literal) rather than held in a
// namespace-scope constant: no static global to initialize or to export across a
// shared-library boundary (a `const char* const` data symbol is not exported by
// CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS, which covers functions only).
inline const char *body_state_type_name() { return "cvc::physics::body_state"; }
inline const char *pose_type_name() { return "cvc::physics::pose"; }

// Register body_state + pose as named data types on `ctx`, so a node holding one
// in data() reports the stable name from dataTypeName(). Idempotent.
void register_physics_data_types(app &ctx);

// Register the body_state + pose (de)serialization codecs into a shard's codec
// registry so a node's physics data() replicates cross-node. Call on every
// participating node's shard registry (alongside the built-in / volume-geometry
// codecs the shard installs itself). The registry key is body_state_type_name() /
// pose_type_name() -- identical to the registerDataType name above, so the key the
// emit path looks up (dataTypeName()) always matches a registered codec.
void register_physics_codecs(state_codec_registry &registry);

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_PHYSICS_CODEC_H__
