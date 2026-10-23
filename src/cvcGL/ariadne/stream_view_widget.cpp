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
#include <cvc/gl/jpeg_codec.h> // decode_jpeg_to_rgba (codec "mjpeg")
#include <cvc/image/image.h>
#include <exception>
#include <memory>
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
  return [](const void *data, std::size_t size, cvc::image &out) {
    std::vector<std::uint8_t> rgba;
    int w = 0, h = 0;
    if (!cvc::gl::decode_jpeg_to_rgba(static_cast<const std::uint8_t *>(data), size, rgba, w, h) ||
        w <= 0 || h <= 0)
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
      "stream_view",
      [&backend, puller](const std::string &current,
                         const cvc::ariadne::Widget &w) -> cvc::ariadne::CustomEdit {
        // A bound value (a switchable token in state) wins over the static `stream:` prop.
        const std::string token = !current.empty() ? current : w.props.str("stream");
        float width = static_cast<float>(w.props.num("size", 256.0));
        if (!(width > 0.0f))
          width = 256.0f; // guard size:0 / negative / NaN -> fall back to the default width
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
        if (!backend.draw_image(token.c_str(), width))
          backend.text_line(("[stream_view: " + token + " — no frame]").c_str());
        return {};
      });
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
