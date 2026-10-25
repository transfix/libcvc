/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Per-viewer video rungs (media_rungs.h): the relay's JPEG downscale, and the shaper that sends
// each viewer a camera at its rung -- as sent, smaller (one variant per frame and rung, shared),
// at a thumbnail rate, or not at all.

#include <chrono>
#include <cstdint>
#include <cvc/media/media_rungs.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

namespace {

// A WxH camera-like frame: a gradient with some texture, as JPEG.
std::vector<std::uint8_t> camera_jpeg(int w, int h) {
  std::vector<std::uint8_t> rgb(static_cast<std::size_t>(w) * h * 3);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      std::uint8_t *p = &rgb[(static_cast<std::size_t>(y) * w + x) * 3];
      p[0] = static_cast<std::uint8_t>(x * 255 / w);
      p[1] = static_cast<std::uint8_t>(y * 255 / h);
      p[2] = static_cast<std::uint8_t>(((x / 8 + y / 8) % 2) * 160 + (x * y) % 64);
    }
  return cvc::jpeg_encode_rgb(rgb.data(), w, h, 60);
}

cvc::media_frame camera_frame(const std::string &origin, std::int64_t seq) {
  cvc::media_frame f;
  f.origin_node_id = origin;
  f.stream_id = "camera";
  f.codec = "mjpeg";
  f.width = 320;
  f.height = 240;
  f.seq = seq;
  f.payload = camera_jpeg(320, 240);
  return f;
}

} // namespace

TEST(MediaRungs, NamesRoundTrip) {
  for (int i = 0; i < cvc::kVideoRungs; ++i) {
    const auto r = static_cast<cvc::video_rung>(i);
    cvc::video_rung back = cvc::video_rung::off;
    ASSERT_TRUE(cvc::parse_video_rung(cvc::video_rung_name(r), back)) << i;
    EXPECT_EQ(back, r);
  }
  cvc::video_rung r = cvc::video_rung::half;
  EXPECT_FALSE(cvc::parse_video_rung("medium", r));
  EXPECT_EQ(r, cvc::video_rung::half) << "untouched";
}

TEST(MediaRungs, AJpegIsDownscaledByHalvesAndShrinks) {
  if (!cvc::jpeg_transcode_available())
    GTEST_SKIP() << "no libjpeg-turbo in this build";
  const std::vector<std::uint8_t> full = camera_jpeg(320, 240);
  ASSERT_FALSE(full.empty());
  int w = 0, h = 0;
  const std::vector<std::uint8_t> half = cvc::jpeg_downscale(full.data(), full.size(), 2, 55, w, h);
  EXPECT_EQ(w, 160);
  EXPECT_EQ(h, 120);
  const std::vector<std::uint8_t> quarter =
      cvc::jpeg_downscale(full.data(), full.size(), 4, 50, w, h);
  EXPECT_EQ(w, 80);
  EXPECT_EQ(h, 60);
  EXPECT_LT(half.size(), full.size());
  EXPECT_LT(quarter.size(), half.size());
  EXPECT_TRUE(cvc::jpeg_downscale(full.data(), full.size(), 3, 50, w, h).empty()) << "1/3 is no "
                                                                                     "DCT scale";
  const std::uint8_t junk[] = {1, 2, 3, 4};
  EXPECT_TRUE(cvc::jpeg_downscale(junk, sizeof(junk), 2, 50, w, h).empty()) << "not a JPEG";
}

// Each viewer gets the camera at its rung. A smaller rung's variant is made once per frame and
// shared by every viewer on it; the mic and other codecs always go as sent.
TEST(MediaRungs, TheShaperSendsEachViewerItsRung) {
  cvc::media_rung_shaper shaper;
  shaper.set_rung("half1", cvc::video_rung::half);
  shaper.set_rung("half2", cvc::video_rung::half);
  shaper.set_rung("none", cvc::video_rung::off);
  EXPECT_EQ(shaper.rung("someone"), cvc::video_rung::full) << "full until set";

  const cvc::media_frame f = camera_frame("A", 1);
  const cvc::media_shape full = shaper.shape(f, "someone");
  EXPECT_FALSE(full.drop);
  EXPECT_EQ(full.variant, nullptr) << "full: as sent";
  EXPECT_TRUE(shaper.shape(f, "none").drop) << "off: nothing";

  const cvc::media_shape a = shaper.shape(f, "half1");
  const cvc::media_shape b = shaper.shape(f, "half2");
  if (cvc::jpeg_transcode_available()) {
    ASSERT_NE(a.variant, nullptr);
    EXPECT_EQ(a.variant, b.variant) << "one variant per frame and rung, shared";
    EXPECT_EQ(a.variant->width, 160u);
    EXPECT_EQ(a.variant->seq, 1);
    EXPECT_LT(a.variant->payload.size(), f.payload.size());
    EXPECT_EQ(shaper.total_transcoded(), 1u);
    EXPECT_EQ(shaper.total_shaped(), 2u);
    EXPECT_GT(shaper.total_bytes_saved(), 0u);
    const cvc::media_shape next = shaper.shape(camera_frame("A", 2), "half1");
    ASSERT_NE(next.variant, nullptr);
    EXPECT_EQ(next.variant->seq, 2) << "the next frame gets its own";
  } else {
    EXPECT_EQ(a.variant, nullptr) << "without libjpeg-turbo a smaller rung sends as is";
  }

  cvc::media_frame mic;
  mic.origin_node_id = "A";
  mic.stream_id = "mic";
  mic.codec = "f32";
  mic.payload.assign(4096, 0);
  const cvc::media_shape m = shaper.shape(mic, "none");
  EXPECT_FALSE(m.drop) << "audio always flows, whatever the video rung";
  EXPECT_EQ(m.variant, nullptr);
}

// The thumb rung sends a viewer at most thumb_fps frames a second from each publisher.
TEST(MediaRungs, TheThumbRungPacesEachPublisher) {
  cvc::media_rung_shaper::config c;
  c.thumb_fps = 5.0; // a frame every 200 ms
  cvc::media_rung_shaper shaper(c);
  shaper.set_rung("v", cvc::video_rung::thumb);
  EXPECT_FALSE(shaper.shape(camera_frame("A", 1), "v").drop) << "the first goes";
  EXPECT_TRUE(shaper.shape(camera_frame("A", 2), "v").drop) << "the next, too soon, does not";
  EXPECT_FALSE(shaper.shape(camera_frame("B", 1), "v").drop)
      << "another publisher has its own pace";
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  const cvc::media_shape later = shaper.shape(camera_frame("A", 3), "v");
  EXPECT_FALSE(later.drop) << "once the interval has passed";
  if (cvc::jpeg_transcode_available()) {
    ASSERT_NE(later.variant, nullptr);
    EXPECT_EQ(later.variant->width, 80u) << "thumbnails are quarter size";
  }
  shaper.forget_viewer("v");
  EXPECT_EQ(shaper.rung("v"), cvc::video_rung::full) << "a viewer that left is forgotten";
}
