/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_ARIADNE_STREAM_VERBS_H__
#define __CVC_GL_ARIADNE_STREAM_VERBS_H__

// Phase-2 PR8: the IMPERATIVE cvcGL stream DSL verb
//
//   (gl-bind-stream NODE-ID TOKEN)  -> { "ok" #t "node" "token" }  (or { "ok" #f "error" })
//
// binds an ALREADY-OPEN cvc::ariadne::stream video (named by TOKEN, the canonical key
// (stream-open) returns in its handle dict — a bare id only if opened at the empty root
// scope) onto an EXISTING realized cvc::gl::GeometryNode (named by NODE-ID, a top-level
// scene node) at run time, from a program `on:`/resident action lane. It is the runtime
// counterpart to the DECLARATIVE "stream" scene node (stream_node.h): same sink
// (StreamTextureBinding), but the author points an existing node at a stream from a button
// or tick instead of declaring a dedicated quad. It NEVER opens a stream ((stream-open)
// does) and owns NO producer (device capture is Phase 3). NODE-ID should name a UV-bearing
// node — GeometryNode::setTexture only samples through uvs, so a node with no uvs binds but
// shows nothing.
//
// ── WHY THIS NEEDS A DEDICATED SEAM (not just register_action_intrinsics) ──────────────
// A normal program-lane verb is registered by a register_action_intrinsics provider, which
// is handed only (environment, intrinsics_context) — it never sees the live SceneGraph or a
// per-frame render-thread hook, both of which binding a stream to a node requires. And that
// provider registry is PROCESS-GLOBAL (shared by every document in the process) and
// append-only, so a provider that simply CAPTURED one document's SceneGraph/adapter would
// dangle once that document died and would mis-route across two live documents.
//
// The fix routes PER DOCUMENT through the document's own se::document_scope: the cvcGL host
// publishes a non-owning handle to its render-thread sink (below) into
// Runtime::document_scope(); the process-global verb provider captures nothing host-specific
// and, at call time, resolves THIS document's sink from ictx.document. Two documents each
// resolve their own sink; a torn-down document's handle is dropped with its Runtime before
// the sink itself, so the verb never touches a freed sink.
//
// ── OWNERSHIP / THREADING (host contract) ──────────────────────────────────────────────
// The sink (the cvcGL GlSceneAdapter) OWNS the StreamTextureBinding a (gl-bind-stream) call
// creates and ticks it every frame on the render thread (right after the declarative
// bindings). Because the action lane and the per-frame scene tick run on the SAME render
// thread in one drain(), bind_stream() constructs the binding on the render thread and it
// first-ticks the same frame — satisfying StreamTextureBinding's ctor/tick/dtor-all-on-one-
// thread contract with no cross-thread handoff. The binding is NOT placed in
// RealizedScene::custom_ticks (that vector is rebuilt wholesale on every reload, which would
// silently drop a runtime binding) and NOT in any process-static/app*-keyed table.

#include <string>

namespace cvc {
namespace gl {
namespace ariadne {

// The render-thread seam the (gl-bind-stream) verb calls into. Implemented by the cvcGL host
// (GlSceneAdapter): it holds the live SceneGraph and owns + ticks the verb-created bindings.
class StreamBindingSink {
public:
  virtual ~StreamBindingSink() = default;

  // Bind TOKEN's already-open stream onto the GeometryNode named NODE-ID. On SUCCESS it replaces
  // any prior binding on that node; on FAILURE (unknown token, node missing, pool full) it leaves
  // an existing binding intact — the new token is resolved and subscribed before the old binding is
  // released, so a failed rebind never drops a working stream. Returns "" on success, else a
  // human-readable "gl-bind-stream: ..." error. MUST be called on the render thread (the verb runs
  // inline in the action lane, which is that thread).
  virtual std::string bind_stream(const std::string &node_id, const std::string &token) = 0;
};

// The per-document routing handle the host publishes into Runtime::document_scope() under
// kGlStreamSinkSlot, and the verb resolves from ictx.document. Non-owning: the sink outlives
// this handle (the Runtime, hence its document_scope, is destroyed before the adapter).
struct GlStreamSinkHandle {
  StreamBindingSink *sink = nullptr;
};
inline constexpr const char *kGlStreamSinkSlot = "cvc.gl.ariadne.stream-sink";

// Register the process-global (gl-bind-stream) verb provider. Appends (like
// register_stream_intrinsics) — a second cvcGL host adds an identical, harmless duplicate rather
// than racing a once-guard, and it re-registers correctly after a clear_action_intrinsics(). The
// provider is STATELESS — it captures no host object — and resolves the calling document's sink
// from ictx.document at call time, so it never dangles and needs no teardown. The cvcGL host calls
// this at construction AND publishes its sink handle via Runtime::document_scope() (see
// GlSceneAdapter). A document whose host published no sink gets a clean "no GL scene sink" error.
void register_gl_stream_intrinsics();

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_ARIADNE_STREAM_VERBS_H__
