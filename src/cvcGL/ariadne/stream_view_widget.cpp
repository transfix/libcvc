/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cstdint>
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/stream/stream_registry.h>
#include <cvc/ariadne/stream/stream_view.h> // stream_view_puller, the shared frame pull
#include <cvc/ariadne/stream/to_image.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/stream_view_widget.h>
#include <cvc/gl/jpeg_codec.h> // jpeg_decoder (codec "mjpeg")
#include <cvc/image/image.h>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

namespace sx = cvc::ariadne::stream;

bool next_stream_frame_image(sx::subscription &sub, std::int64_t &last_seq, cvc::image &out) {
  return sx::next_frame_image(sub, last_seq, out);
}

sx::frame_decoder jpeg_frame_decoder() {
  // The decoder's own decompressor, reused frame to frame; the mutex keeps it to one frame at a
  // time if the decoder is shared.
  struct jpeg_state {
    std::mutex m;
    cvc::gl::jpeg_decoder decoder;
  };
  auto state = std::make_shared<jpeg_state>();
  return [state](const void *data, std::size_t size, cvc::image &out) {
    std::vector<std::uint8_t> rgba;
    int w = 0, h = 0;
    bool ok = false;
    {
      std::lock_guard<std::mutex> lock(state->m);
      ok = state->decoder.decode(static_cast<const std::uint8_t *>(data), size, rgba, w, h);
    }
    if (!ok || w <= 0 || h <= 0)
      return false;
    out = cvc::image(w, h, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8, rgba.data());
    return true;
  };
}

void register_stream_view_widget(ImGuiBackend &backend, cvc::app &app) {
  // The per-widget subscriptions and the frame pull are the core's (stream/stream_view.h), shared
  // with the terminal stream_view; this adds the JPEG decoder ("mjpeg": conference video) and the
  // ImGui draw. Held by the handler closure for the backend's lifetime.
  auto puller = std::make_shared<sx::stream_view_puller>(app);
  puller->set_decoder("mjpeg", jpeg_frame_decoder());
  puller->set_decoder("jpeg", jpeg_frame_decoder());

  backend.register_widget(
      app, "stream_view",
      [&backend, puller](const std::string &current,
                         const cvc::ariadne::Widget &w) -> cvc::ariadne::CustomEdit {
        // A bound value (a switchable token in state) wins over the static `stream:` prop.
        const std::string token = !current.empty() ? current : w.props.str("stream");
        // `size:` a width (height from the frame's aspect) or [w, h], a box the frame fits inside.
        float bw = 0.0f, bh = 0.0f;
        const bool box = sx::stream_view_size(w.props, bw, bh);
        if (token.empty())
          return {}; // nothing addressed — draw nothing (handled)
        cvc::image img;
        // One slot per widget instance (id defaults to label, else key by token), so a switchable
        // bind re-subscribes rather than leaking a slot per token ever seen.
        if (puller->pull(!w.id.empty() ? w.id : token, token, img))
          backend.set_image(token, img); // publish the new frame under the token (COW + keepalive)
        // Draw whatever image is published under `token`: the live frame fed above, or a static
        // image the host set via set_image(token, ...). draw_image returns false if nothing is
        // published yet (stream not live and no static image) — then draw a quiet placeholder.
        if (!(box ? backend.draw_image_fit(token.c_str(), bw, bh)
                  : backend.draw_image(token.c_str(), bw)))
          backend.text_line(("[stream_view: " + token + " — no frame]").c_str());
        return {};
      });
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
