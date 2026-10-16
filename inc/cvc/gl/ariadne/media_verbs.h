/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_ARIADNE_MEDIA_VERBS_H__
#define __CVC_GL_ARIADNE_MEDIA_VERBS_H__

// Conference roadmap §4a — the cvcGL MEDIA DSL verbs: local capture/playback that attach a
// producer (or, later, a playback sink) to an ALREADY-OPEN cvc::ariadne::stream. They wrap
// already-merged C++ (SDL camera/mic capture, the audio playback sink), so they live in cvcGL, and
// are registered like (gl-bind-stream): one process-global, APPEND, STATELESS register_action_
// intrinsics provider that resolves the calling document from ictx.document and attaches to that
// document's streams (opened via the CORE (stream-open) verb, cvc::ariadne::register_stream_
// intrinsics). Registering is opt-in; a conference host calls register_stream_intrinsics +
// register_gl_stream_intrinsics + register_media_intrinsics together.
//
//   (stream-capture-camera TOKEN [OPTS]) -> { "ok" #t "token" "synthetic" "hz" }
//                                           (or { "ok" #f "error" })
//       OPTS = dict { "device" int, "w" int, "h" int, "hz" num, "synthetic" bool }
//     Attach a frame producer to TOKEN's open VIDEO stream. A real camera (SDL3, CVC_ENABLE_SDL)
//     when `device` >= 0; a hardware-free moving-test-pattern synthetic_source otherwise —
//     `synthetic:#t`, a negative/absent `device`, or an SDL-less build. `w`/`h` size the SYNTHETIC
//     frames and MUST match the stream's open w/h (a camera dictates its own; the stream was sized
//     to it) — a mismatch is rejected by stream::start_producer (the frame then never flows). `hz`
//     is the producer cadence (default 30; a camera's own fps is used when `hz` is omitted).
//
// TOKEN is the handle (stream-open) returned (a dict carrying "token", or the bare token string).
// The verb NEVER opens a stream ((stream-open) does) and owns no producer beyond the stream's own
// (start_producer's thread is joined by stream::close()/teardown). It runs on the action/resident
// lane (it needs ictx.document); the load-time init: lane fails cleanly. Calling it again on the
// same stream REPLACES the producer (start_producer stops+joins the old one first).

namespace cvc {
namespace gl {
namespace ariadne {

// Register the process-global media verb provider. Appends (like register_stream_intrinsics /
// register_gl_stream_intrinsics) — a second host adds a harmless duplicate rather than racing a
// once-guard, and it re-registers after a clear_action_intrinsics(). The provider is STATELESS
// (captures only each lane's ictx.document) and needs no teardown.
void register_media_intrinsics();

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_ARIADNE_MEDIA_VERBS_H__
