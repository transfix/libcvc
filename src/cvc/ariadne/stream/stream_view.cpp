/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// stream_view's GL-free frame pull and the terminal widget — see
// inc/cvc/ariadne/stream/stream_view.h.

#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/stream/stream_registry.h>
#include <cvc/ariadne/stream/stream_view.h>
#include <cvc/ariadne/stream/to_image.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/image/image.h>
#include <exception>
#include <utility>

namespace cvc {
namespace ariadne {
namespace stream {

bool next_frame_image(subscription &sub, std::int64_t &last_seq, cvc::image &out) {
  const frame_ptr f = sub.latest(); // non-destructive, lock-free (an atomic load)
  if (!f || f->seq == last_seq)
    return false; // nothing yet, or the frame already consumed
  last_seq = f->seq;
  try {
    out = to_image(f); // aliases the frame bytes and co-owns its keepalive (no copy)
  } catch (const std::exception &) {
    return false; // not losslessly aliasable; last_seq has moved on, so it is not retried
  }
  return true;
}

stream_view_puller::stream_view_puller(cvc::app &app) : app_(app) {}

stream_view_puller::~stream_view_puller() {
  // Free each still-live channel's subscriber slot promptly; a gone channel needs nothing.
  for (auto &[key, s] : slots_)
    if (s.sub)
      if (stream_channel *ch = stream_registry::for_app(app_).lookup(s.token))
        if (ch == s.bound)
          ch->unsubscribe(s.sub);
}

void stream_view_puller::set_decoder(const std::string &codec, frame_decoder decode) {
  if (decode)
    decoders_[codec] = std::move(decode);
  else
    decoders_.erase(codec);
}

bool stream_view_puller::pull(const std::string &key, const std::string &token, cvc::image &out) {
  slot &s = slots_[key];
  // Reconcile to the CURRENT channel for `token`: the lookup is cheap, and it is what lets the view
  // follow a token switch or a close/reopen under the same token instead of freezing on a dead
  // subscription (whose latest() keeps returning the last frame).
  stream_channel *ch = stream_registry::for_app(app_).lookup(token);
  if (s.sub && (s.token != token || s.bound != ch)) {
    // Release the old subscription: unsubscribe only if its token still names the channel it was
    // made against (a closed or reopened one needs no unsubscribe; dropping the ref suffices).
    if (stream_channel *old = stream_registry::for_app(app_).lookup(s.token))
      if (old == s.bound)
        old->unsubscribe(s.sub);
    s.sub.reset();
    s.bound = nullptr;
    s.last_seq = -1;
  }
  if (!s.sub && ch) { // (re)bind; until the stream opens, ch is null and this retries next frame
    s.sub = ch->subscribe(deliver_mode::latest);
    s.bound = s.sub ? ch : nullptr;
    s.token = token;
    s.last_seq = -1;
  }
  if (!s.sub)
    return false;

  const auto dec = decoders_.find(s.bound->format().codec);
  if (dec == decoders_.end())
    return next_frame_image(*s.sub, s.last_seq, out);
  const frame_ptr f = s.sub->latest();
  if (!f || f->seq == s.last_seq)
    return false;
  s.last_seq = f->seq; // a frame that fails to decode is not retried every frame either
  return dec->second(f->data, f->size, out);
}

} // namespace stream

std::shared_ptr<stream::stream_view_puller> register_stream_view_widget(FtxuiBackend &backend,
                                                                        cvc::app &app) {
  auto puller = std::make_shared<stream::stream_view_puller>(app);
  backend.register_widget(
      "stream_view", [&backend, puller](const std::string &current, const Widget &w) -> CustomEdit {
        // A bound value (a switchable token in state) wins over the static `stream:` prop.
        const std::string token = !current.empty() ? current : w.props.str("stream");
        float width = static_cast<float>(w.props.num("size", 256.0));
        if (!(width > 0.0f))
          width = 256.0f; // size: 0, negative or NaN -> the default width
        if (token.empty())
          return {}; // nothing addressed: draw nothing
        cvc::image frame;
        if (puller->pull(!w.id.empty() ? w.id : token, token, frame))
          backend.set_image(token, frame); // the newest frame, under the token's name
        // The live frame published above, or a still image the host set under the same name.
        if (!backend.draw_image(token.c_str(), width))
          backend.text_line(("[stream_view: " + token + " - no frame]").c_str());
        return {};
      });
  return puller;
}

} // namespace ariadne
} // namespace cvc
