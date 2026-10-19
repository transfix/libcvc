/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_STREAM_INTRINSICS_H__
#define __CVC_ARIADNE_STREAM_INTRINSICS_H__

// Ariadne DSL surface for cvc::ariadne::stream (roadmap STATE_BINARY_STREAMING.md Phase 2,
// PR6 core). Opt-in, exactly like register_net_intrinsics: a host/test calls
// register_stream_intrinsics(app) to add the program-lane verbs:
//
//   (stream-open OPTS)   OPTS = dict { "id" "codec" … [sizing/heartbeat] }. A VIDEO stream for a
//                        raster codec ("rgba8"/"rgb8"/"graya8"/"gray8") takes "w"/"h"; an AUDIO
//                        stream (codec "f32", interleaved float PCM) takes "rate"/"ch" and an
//                        optional "frames_per_chunk" (default 1024) instead. Scoped to the calling
//                        document (ictx.root_path), returns a handle dict
//                        { "ok" "id" "token" "seq_channel" "evt_channel" } (or
//                        { "ok" #f "error" } on failure).
//   (stream-info H)      H = the handle dict or its "token" string. Returns a dict
//                        { "ok" "live" "id" "token" "seq_channel" "evt_channel"
//                          "published" "subscribers" } for a live stream.
//   (stream-close H)     Closes + drops the stream. Returns { "ok" #t/#f }.
//
// OWNERSHIP (PR6-core model B): the opened stream is owned by the CALLING DOCUMENT's
// se::document_scope (reached via ictx.document), keyed by its canonical token — NOT a
// process-static table keyed by app*. It lives until (stream-close) or document teardown:
// when the Runtime tears down it clear()s the scope, closing any stream the author left
// open while the app and its state tree are still alive (the non-singleton lifetime the
// app*-keyed table could not give). Opening requires a document scope, so the verbs run
// on the action/resident lanes (ictx.document is installed there), not the load-time
// init: lane (which fails cleanly). A stream opened here has NO producer (device capture
// is Phase 3); a producer is attached separately and must be stopped before (stream-close).

#include <memory>
#include <string>

namespace cvc {
class app;
namespace state_exec {
class document_scope;
}
namespace ariadne {
namespace stream {
class stream;
}

void register_stream_intrinsics(cvc::app &app);

// Look up a stream opened by (stream-open) in this document's scope, by its canonical token (the
// value (stream-open) returns as "token"; a handle dict's "token" field). Returns nullptr when the
// document holds no such open stream (or `doc` is null). This is the cross-module accessor onto the
// document-owned stream table that the (stream-open) verbs keep private: it lets a companion module
// — e.g. the cvcGL media verbs (stream-capture-*) — attach a producer to a document-owned stream
// without owning, or even seeing the type of, that table. The returned stream stays owned by the
// document scope; the caller borrows it (e.g. to call start_producer) and must not delete it.
stream::stream *document_stream(cvc::state_exec::document_scope *doc, const std::string &token);

// Transfer ownership of an already-open stream S into DOC's document-scoped stream table (the same
// table (stream-open) fills), so it lives until (stream-close) or document teardown and is
// reachable by document_stream() and the (stream-info/-close/-capture/-play) verbs. Returns S's
// token, or "" when DOC is null / its table is unavailable (S is then destroyed). Used e.g. by
// (room-subscribe) to hand a pulled proxy stream to the document so the usual playback/render verbs
// can consume it.
std::string own_document_stream(cvc::state_exec::document_scope *doc,
                                std::unique_ptr<cvc::ariadne::stream::stream> s);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_INTRINSICS_H__
