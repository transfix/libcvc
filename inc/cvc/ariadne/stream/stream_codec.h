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

#ifndef __CVC_ARIADNE_STREAM_STREAM_CODEC_H__
#define __CVC_ARIADNE_STREAM_STREAM_CODEC_H__

#include <cvc/core/namespace.h>

namespace cvc {

class app;
class state_codec_registry;

namespace ariadne {
namespace stream {

// ----------------
// cvc::ariadne::stream — descriptor replication codec (Phase 10, PR5)
// ----------------
// A stream publishes its format_desc into the /streams/<id> descriptor node's
// data() (stream::open). For a remote node to DISCOVER the stream, that data()
// must cross the cluster — which the distributed-state pipeline does only for
// node data() whose type has both a registered name and a codec.
//
// These helpers live in cvc::ariadne::stream (not cvc::state) because
// format_desc lives here: cvc::state sits below ariadne and must not depend on
// it. The ariadne layer registers the codec INTO a shard's (state-owned,
// mutable) codec registry.

// The canonical, cross-platform-stable data-type name under which a stream's
// format_desc is stored on the descriptor node AND keyed in the codec registry.
// Both the producing app's registerDataType and every node's codec use this
// exact string, so the descriptor replicates regardless of compiler/ABI (an
// unregistered type would fall back to the mangled typeid name, which differs
// across toolchains).
extern const char *const kFormatDescTypeName;

// Register format_desc as a named data type on `ctx`, so a descriptor node's
// dataTypeName() resolves to kFormatDescTypeName rather than the compiler's
// mangled typeid name. Idempotent. stream::open calls this before it writes the
// descriptor, so the producing app always emits the stable name.
void register_stream_data_types(app &ctx);

// Register the format_desc (de)serialization codec into a shard's codec
// registry so the /streams/<id> descriptor's data() replicates cross-node.
// Call on every participating node's shard codec registry (alongside the
// built-in and volume/geometry codecs the shard installs itself).
void register_stream_codecs(state_codec_registry &registry);

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_STREAM_CODEC_H__
