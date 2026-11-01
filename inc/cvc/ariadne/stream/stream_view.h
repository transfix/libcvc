/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The `stream_view` widget's frame pull, GL-free: what turns a live video stream into an image a
// backend can draw. Every backend's stream_view uses it (cvcGL's ImGui one, and the terminal one
// declared below), so a document's
//
//   - type: stream_view
//     stream: <stream token>   # the token stream::open() installed; OR
//     bind: <state key>        #   a state key holding the token (switchable at runtime)
//     size: 480                # display width in px; height follows the frame's aspect
//
// draws the same stream under ImGui and in a terminal (`ari`, `arigl --terminal`).
//
// stream_view_puller keeps one subscription per widget (keyed by Widget::id, else the token) and
// reconciles it every frame: the token can switch, and a stream can close and reopen under the same
// token (a NEW channel), so it re-resolves the channel through the app's stream_registry and
// re-subscribes when either changes. pull() then reports a frame only when one NEWER than the last
// drawn arrived (seq dedup). Raw rasters (rgba8 / rgb8 / gray8 ...) alias straight into a
// cvc::image; a compressed codec ("mjpeg") needs a decoder the host adds (cvcGL adds a JPEG one),
// since the stream core links no codec. Render thread only, like the widget draw that calls it.
//
// Lifetime: a subscription outlives its channel safely, so the stream need not outlive the view. A
// widget removed from the tree keeps its slot until the slot is reconciled to another channel or
// the puller is destroyed (there is no per-frame sweep); a displayed widget never leaks.

#ifndef __CVC_ARIADNE_STREAM_STREAM_VIEW_H__
#define __CVC_ARIADNE_STREAM_STREAM_VIEW_H__

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>

namespace cvc {
class app;
class image;
namespace ariadne {
class FtxuiBackend;
struct Value;
namespace stream {

class subscription;
class stream_channel;

// A stream_view's `size:` prop, either a width (`size: 320`: the height follows each frame's
// aspect, so a portrait frame stands taller than a landscape one) or a box (`size: [320, 240]`:
// each frame fitted inside it with its shape kept, Backend::draw_image_fit, so the tile stays one
// size whatever the camera sends). Sets W, and H for a box (0 for a width), and returns whether it
// is a box. A missing, malformed or non-positive size is the 256 px width.
bool stream_view_size(const Value &props, float &w, float &h);

// If `sub` holds a frame NEWER than `last_seq`, alias it into `out`, advance `last_seq` and return
// true; else leave both alone and return false. A frame that cannot be aliased losslessly (padded
// stride, flipped origin, a compressed codec) also returns false, after advancing `last_seq` so it
// is not retried every frame.
bool next_frame_image(subscription &sub, std::int64_t &last_seq, cvc::image &out);

// Decode one compressed frame's bytes to an image; false when it cannot.
using frame_decoder = std::function<bool(const void *data, std::size_t size, cvc::image &out)>;

class stream_view_puller {
public:
  explicit stream_view_puller(cvc::app &app);
  ~stream_view_puller();
  stream_view_puller(const stream_view_puller &) = delete;
  stream_view_puller &operator=(const stream_view_puller &) = delete;

  // Decode streams whose format codec is `codec` (e.g. "mjpeg") with `decode`.
  void set_decoder(const std::string &codec, frame_decoder decode);

  // Reconcile the view `key` to `token`'s current channel, then pull. True, with the new frame in
  // `out`, when a frame newer than the one this view last drew arrived.
  bool pull(const std::string &key, const std::string &token, cvc::image &out);

private:
  struct slot {
    std::string token;                 // the token this slot is subscribed to
    stream_channel *bound = nullptr;   // the channel it subscribed against (switch / reopen check)
    std::shared_ptr<subscription> sub; // null until bound, and after a release
    std::int64_t last_seq = -1;
  };
  cvc::app &app_;
  std::map<std::string, slot> slots_;
  std::map<std::string, frame_decoder> decoders_;
};

} // namespace stream

// Register the `stream_view` widget on a terminal backend (FtxuiBackend, or the TvisionBackend that
// derives from it): each live frame is published under the stream's token with set_image and drawn
// with draw_image, so it shows as Kitty / Sixel pixels or half-blocks, as the backend's images do.
// With no live stream it draws whatever image is published under that name, else a placeholder
// line. Returns the widget's puller, so a host can add decoders (cvcGL: "mjpeg"). `app` and
// `backend` must outlive the registration.
std::shared_ptr<stream::stream_view_puller> register_stream_view_widget(FtxuiBackend &backend,
                                                                        cvc::app &app);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_STREAM_VIEW_H__
