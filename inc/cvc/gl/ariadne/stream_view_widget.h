/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_ARIADNE_STREAM_VIEW_WIDGET_H__
#define __CVC_GL_ARIADNE_STREAM_VIEW_WIDGET_H__

// Ariadne "stream_view" widget: an ImGui image view backed by a LIVE cvc::ariadne::stream (video) —
// the WIDGET half of Phase-3 streaming, the counterpart to the "stream" SCENE node (a 3D textured
// quad). Where the scene node fills a viewport quad, this draws the stream inside an ordinary
// Ariadne window, which ImGui makes draggable/resizable for free (window geometry persists via the
// §11.4 tree.<id>.geometry edge). Declare it in a DSL document:
//
//   - window: Camera
//     id: cam
//     children:
//       - type: stream_view
//         stream: <stream token>   # the token stream::open() installed; OR
//         bind: <state key>        #   a state key holding the token (switchable at runtime)
//         size: 480                # display width in px; height follows the frame's aspect
//
// Each rendered frame it resolves the token to the already-open stream, pulls the latest frame
// (non-destructive, lock-free) and — only when a NEW frame arrived (seq advanced) — uploads it to
// the backend's image for that token, then draws it with ImGui::Image. It doubles as a STATIC image
// view: with no live stream for the token it simply draws whatever image the host published under
// that name via ImGuiBackend::set_image, so one widget shows "a static image OR video".
//
// register_stream_view_widget() wires the "stream_view" type onto an ImGuiBackend (via its
// register_widget seam). It needs the cvc::app to resolve tokens through the app's stream_registry;
// call it once after creating the backend, before running the UI. The widget draw — and the
// subscription cache it keeps (one slot per widget identity: Widget::id, else the token) — run on
// the backend's render thread ONLY, the same single-thread contract as StreamTextureBinding;
// `app` and `backend` must outlive the backend's registered handlers.
//
// Stream lifetime: calling latest() after the stream/channel is destroyed is memory-safe (the
// widget holds its own subscription ref, which outlives the channel), so — unlike
// StreamTextureBinding — the stream need NOT outlive the widget. The widget re-resolves the channel
// every frame and re-subscribes when the token switches or the stream closes/reopens under the same
// token, so a reconnecting camera or a re-opened record/playback stream recovers on its own. One
// residual: a stream_view REMOVED from the UI tree stops being drawn, so its subscription is
// released only when its slot is next reconciled to a different channel or at backend teardown —
// not the instant it leaves the tree (there is no per-frame sweep); a displayed widget never leaks.
//
// cvcGL-only (ImGui + GL upload), so it lives in cvc::gl::ariadne; the stream core stays pure.

#include <cstdint>

namespace cvc {
class app;
class image;
namespace ariadne {
namespace stream {
class subscription;
} // namespace stream
} // namespace ariadne
namespace gl {
class ImGuiBackend;
namespace ariadne {

// Register the "stream_view" custom widget type on `backend`, resolving stream tokens through
// `app`'s registry. Idempotent per backend (re-registering replaces the handler).
void register_stream_view_widget(ImGuiBackend &backend, cvc::app &app);

// The render-thread pull step the widget uses, exposed for reuse and testing: if `sub` holds a
// frame NEWER than `last_seq`, convert it to an image into `out`, advance `last_seq`, and return
// true; otherwise leave `out`/`last_seq` untouched and return false. Returns false (not throw) when
// the frame cannot be losslessly aliased to an image (padded stride, flipped origin, odd codec).
bool next_stream_frame_image(cvc::ariadne::stream::subscription &sub, std::int64_t &last_seq,
                             cvc::image &out);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_ARIADNE_STREAM_VIEW_WIDGET_H__
