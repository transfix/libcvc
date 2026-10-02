/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_ARIADNE_STREAM_NODE_H__
#define __CVC_GL_ARIADNE_STREAM_NODE_H__

// Phase-2 PR7: the cvcGL stream SINK as a DECLARATIVE scene-node type. Opt-in, exactly
// like the other register_scene_node_type consumers: a host/test/demo calls
// register_stream_node_type() ONCE (before a scene is loaded/realized) to make the
// "stream" node type available. A document then declares, e.g.
//
//   { "id": "screen", "type": "stream", "stream": "<token>", "width": 1.6, "height": 0.9 }
//
// and the realizer builds a UV'd quad, resolves the ALREADY-OPEN stream named by the
// node's "stream" (or "token") attribute through cvc::ariadne::stream::stream_registry,
// subscribes deliver_mode::latest, and wires a per-frame StreamTextureBinding tick into
// the RealizedScene (driven by tick_scene() on the render thread, which is where the
// binding's single-thread contract requires its tick()/dtor to run). The binding is
// owned by the RealizedScene (its custom_ticks closure), so it is torn down — and
// unsubscribes — with the scene, on the render thread.
//
// It NEVER opens a stream ((stream-open) does that) and owns NO producer (device capture
// is Phase 3). The token is the canonical key (stream-open) returns in its handle dict,
// i.e. cvc::ariadne::stream::stream::token(); a bare id only resolves if it was opened at
// the empty root scope. Degradation is graceful: a missing "stream" attr rejects the node
// with a warning; a token that names no live stream still builds the (texture-less) quad
// with a warning, so the stream must be open before the scene realizes.
//
// LIFETIME: the binding (hence its unsubscribe) is owned by the RealizedScene and torn
// down with it, so — like every StreamTextureBinding and the scene's other render-thread
// objects — the host MUST destroy/rebuild the RealizedScene (and the Runtime that owns it)
// on the render thread. Off-thread teardown concurrent with stream close would race the
// binding's unsubscribe against the channel; the ownership model already serializes both
// under Runtime teardown, so this adds no new invariant, only stakes.
//
// POOL SIZING: each realized stream node is one subscriber on its token's frame pool, which
// is fixed-size from the (stream-open) expected_subscribers (refuse-rather-than-under-
// provision). Size it for the number of declarative sinks on that token, plus headroom: a
// re-realize (hot reload, or a duplicate node id in one scene) subscribes the new sink
// BEFORE releasing the old, so peak demand is briefly N+1 — a pool sized to exactly N makes
// the new sink render texture-less with a "frame pool full" warning until a tick frees the
// old one.
//
// Imperative binding of a stream to an existing node at runtime (a (gl-bind-stream …)
// verb) is a separate follow-up: it needs a cvcGL DSL-verb seam carrying the live
// SceneGraph + a per-frame tick sink, which the neutral register_action_intrinsics path
// (env, ictx only) does not provide.

namespace cvc {
namespace gl {
namespace ariadne {

// Register the "stream" scene-node type. Idempotent (a no-op if already registered) and,
// like register_scene_node_type itself, process-global and thread-safe; call it before
// realize_scene() / document load so verify_scene_customs() sees the type.
void register_stream_node_type();

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_ARIADNE_STREAM_NODE_H__
