/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/stream/stream_registry.h>
#include <cvc/ariadne/stream/to_image.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/stream_view_widget.h>
#include <cvc/image/image.h>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>

namespace cvc {
namespace gl {
namespace ariadne {

namespace sx = cvc::ariadne::stream;

bool next_stream_frame_image(sx::subscription &sub, std::int64_t &last_seq, cvc::image &out) {
  const sx::frame_ptr f = sub.latest(); // non-destructive, lock-free (atomic load)
  if (!f || f->seq == last_seq)
    return false; // nothing yet, or the same frame we already consumed (seq dedup)
  try {
    out = sx::to_image(f); // aliases the frame bytes + co-owns its keepalive (zero copy)
  } catch (const std::exception &) {
    // Not losslessly aliasable (padded stride / flipped origin / channel-swapped codec). Advance
    // last_seq so we do not retry this exact frame every tick, and report "no image this tick".
    last_seq = f->seq;
    return false;
  }
  last_seq = f->seq;
  return true;
}

void register_stream_view_widget(ImGuiBackend &backend, cvc::app &app) {
  // Per-WIDGET state (the token it is bound to, the channel it subscribed against, the
  // subscription, and the last-drawn seq), kept across frames. Keyed by widget identity
  // (Widget::id, which defaults to the label) so one widget owns one subscription even as its bound
  // token changes; id-less widgets fall back to keying by token. The widget draw runs on the
  // backend's render thread ONLY, so this map needs no lock. Held by shared_ptr so the handler
  // closure owns it for the backend's lifetime.
  struct view_slot {
    std::string token;                      // the token this slot is currently subscribed to
    sx::stream_channel *ch_bound = nullptr; // the channel subscribed against (reopen/switch check)
    std::shared_ptr<sx::subscription> sub;  // null until bound / after a release
    std::int64_t last_seq = -1;
  };
  auto cache = std::make_shared<std::unordered_map<std::string, view_slot>>();

  backend.register_widget(
      "stream_view",
      [&backend, &app, cache](const std::string &current,
                              const cvc::ariadne::Widget &w) -> cvc::ariadne::CustomEdit {
        // A bound value (a switchable token in state) wins over the static `stream:` prop.
        const std::string token = !current.empty() ? current : w.props.str("stream");
        float width = static_cast<float>(w.props.num("size", 256.0));
        if (!(width > 0.0f))
          width = 256.0f; // guard size:0 / negative / NaN -> fall back to the default width
        if (token.empty())
          return {}; // nothing addressed — draw nothing (handled)

        // One slot per widget instance (so a switchable bind re-subscribes rather than leaking a
        // slot per token ever seen); id defaults to label, else key by token.
        const std::string &key = !w.id.empty() ? w.id : token;
        view_slot &s = (*cache)[key];

        // Reconcile the subscription to the CURRENT channel for `token` each frame. The per-frame
        // registry lookup is cheap and is what lets the view survive a token switch OR a
        // close/reopen under the same token (a reopened stream is a NEW channel) — without it the
        // view would freeze on the dead subscription (latest() keeps returning the stale frame).
        sx::stream_channel *ch = sx::stream_registry::for_app(app).lookup(token);
        if (s.sub && (s.token != token || s.ch_bound != ch)) {
          // Release the old subscription, mirroring StreamTextureBinding::release(): re-resolve the
          // OLD token and unsubscribe only if it is still that same live channel (frees its pool
          // slot promptly); if the old channel is gone (reopen/close), dropping the sub suffices.
          if (sx::stream_channel *old = sx::stream_registry::for_app(app).lookup(s.token))
            if (old == s.ch_bound)
              old->unsubscribe(s.sub);
          s.sub.reset();
          s.ch_bound = nullptr;
          s.last_seq = -1;
        }
        if (!s.sub && ch) {
          // (Re)bind. While the stream is not yet open ch is null and we simply retry next frame,
          // so a stream_view declared before its stream binds once it opens.
          s.sub = ch->subscribe(sx::deliver_mode::latest);
          s.ch_bound = ch;
          s.token = token;
          s.last_seq = -1;
        }

        if (s.sub) {
          cvc::image img;
          if (next_stream_frame_image(*s.sub, s.last_seq, img))
            backend.set_image(token,
                              img); // publish the new frame under the token (COW + keepalive)
        }
        // Draw whatever image is published under `token`: the live frame fed above, or a static
        // image the host set via set_image(token, ...). draw_image returns false if nothing is
        // published yet (stream not live and no static image) — then draw a quiet placeholder.
        if (!backend.draw_image(token.c_str(), width))
          backend.text_line(("[stream_view: " + token + " — no frame]").c_str());
        return {};
      });
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
