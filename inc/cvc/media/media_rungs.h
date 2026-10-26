/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_MEDIA_RUNGS_H__
#define __CVC_MEDIA_RUNGS_H__

#include <cstddef>
#include <cstdint>
#include <cvc/media/media_endpoint.h>
#include <cvc/media/media_frame.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {

// ----------------
// Per-viewer video rungs (ARIADNE_CONFERENCE_ROADMAP §5j)
// ----------------
// A relay sends each viewer every camera at a RUNG: as its publisher sent it, or smaller. MJPEG
// frames stand alone, so the relay makes a smaller rung itself: it decodes the publisher's JPEG at
// 1/2 or 1/4 size (libjpeg-turbo scales in the DCT, so the decode is cheaper than a full one) and
// re-encodes it, once per frame and rung, and every viewer on that rung gets the same bytes. A
// publisher uploads one stream, and the policy -- which viewer gets which rung -- is the relay's
// alone (room_rung_controller adapts it to each viewer's link).
enum class video_rung : int {
  full = 0,    // as sent
  half = 1,    // 1/2 width and height, re-encoded
  quarter = 2, // 1/4
  thumb = 3,   // 1/4 at a thumbnail rate
  off = 4      // no video (audio still flows)
};
constexpr int kVideoRungs = 5;

// "full", "half", "quarter", "thumb", "off".
const char *video_rung_name(video_rung r) noexcept;
// False (OUT untouched) for anything else.
bool parse_video_rung(const std::string &name, video_rung &out) noexcept;

// Whether this build can transcode JPEG (libjpeg-turbo). Without it, half and quarter send the
// frame as is, and thumb only lowers the rate.
bool jpeg_transcode_available() noexcept;

// The JPEG `jpeg` [`size`] decoded at 1/`denom` of its size (1, 2, 4 or 8) and re-encoded at
// `quality` (1-100, 4:2:0). Empty on failure or without libjpeg-turbo; `out_w`/`out_h` get the new
// size. Thread-safe.
std::vector<std::uint8_t> jpeg_downscale(const std::uint8_t *jpeg, std::size_t size, int denom,
                                         int quality, int &out_w, int &out_h);

// W*H packed RGB8 encoded as a JPEG at QUALITY (1-100, 4:2:0). Empty on failure or without
// libjpeg-turbo. Thread-safe.
std::vector<std::uint8_t> jpeg_encode_rgb(const std::uint8_t *rgb, int w, int h, int quality);

// Shapes a relay's camera frames per viewer (a media_endpoint::media_shape_fn): an MJPEG camera
// frame goes to each viewer at that viewer's rung; every other frame (audio, other codecs) as
// sent. Thread-safe: publishers' reader threads call shape() while the relay's loop sets rungs.
class media_rung_shaper {
public:
  struct config {
    int half_quality = 55;
    int quarter_quality = 50;
    double thumb_fps = 2.0; // the thumb rung's rate, per viewer and publisher
  };
  explicit media_rung_shaper(config c);
  media_rung_shaper();
  ~media_rung_shaper();
  media_rung_shaper(const media_rung_shaper &) = delete;
  media_rung_shaper &operator=(const media_rung_shaper &) = delete;

  // VIEWER's rung (full until set).
  void set_rung(const std::string &viewer, video_rung r);
  video_rung rung(const std::string &viewer) const;
  // Drop what is kept for VIEWER (it left) or for PUBLISHER's frames.
  void forget_viewer(const std::string &viewer);
  void forget_publisher(const std::string &publisher);

  // F for VIEWER: as sent, a smaller variant (made once per frame and rung, shared), or dropped.
  media_shape shape(const media_frame &f, const std::string &viewer);

  // Variants made (one per frame and rung), deliveries of a variant, and the bytes those
  // deliveries saved against sending the frames as sent.
  std::uint64_t total_transcoded() const noexcept;
  std::uint64_t total_shaped() const noexcept;
  std::uint64_t total_bytes_saved() const noexcept;

private:
  struct impl; // file-local tables (they keep std:: template code out of cvc.dll's exports)
  std::unique_ptr<impl> _impl;
};

} // namespace cvc

#endif // __CVC_MEDIA_RUNGS_H__
