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
//   (stream-open OPTS)   OPTS = dict { "id" "codec" "w" "h" [sizing/heartbeat] }
//                        Opens a video stream scoped to the calling document
//                        (ictx.root_path), returns a handle dict
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

namespace cvc {
class app;
namespace ariadne {

void register_stream_intrinsics(cvc::app &app);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_INTRINSICS_H__
